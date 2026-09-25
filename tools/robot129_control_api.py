"""Simple control interface for the Robot 129 Isaac Sim simulation.

This is a thin wrapper around the existing ROS 2 topics that
`sim/scripts/run_robot129_ros_webrtc.py` already subscribes to. It does not
add a new control path: `set_joint_positions` publishes
`trajectory_msgs/JointTrajectory` on `/robot129_sim/arm_controller/joint_trajectory`
(joint1..joint6) and the gripper functions publish on
`/robot129_sim/gripper_controller/joint_trajectory` (joint7; the bridge mirrors
joint8 = -joint7 automatically), exactly like `tools/send_robot129_ros_pose.py`.

Prerequisites (see docs/LOCAL_ROS_CONTROL.md):
  1. Isaac Sim + ROS bridge + WebRTC already running:
         bash tools/start_robot129_ros_webrtc.sh
  2. This module imported inside the Robot 129 ROS environment, e.g. via:
         bash tools/run_with_robot129_control.sh your_script.py

Usage:
    from robot129_control_api import set_joint_positions, open_gripper, close_gripper, set_gripper

    set_joint_positions([0.0, 1.20, -1.25, 0.0, 0.15, 0.0])
    close_gripper()
    set_joint_positions([0.2, 1.35, -1.55, 0.3, -0.20, 0.10])
    open_gripper()

Every call blocks until Isaac's `/robot129_sim/joint_states` feedback confirms
the joints reached the target (max error < 0.015 rad/m for 5 consecutive
samples), same tolerance/window used by the rest of this workspace's ROS
tooling. Pass `wait=False` to fire-and-forget instead.
"""

from __future__ import annotations

import atexit
import os
import time
from typing import Sequence

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_NAMES = [f"joint{i}" for i in range(1, 7)]
ALL_NAMES = [f"joint{i}" for i in range(1, 9)]

# From robot/arm_parameter_summary.yaml (copied from the Piper URDF); joint2's
# lower bound is 0.0 and joint3's upper bound is 0.0 -- those are real limits,
# not a units bug.
ARM_LIMITS = [
    (-2.618, 2.618),
    (0.0, 3.14),
    (-2.967, 0.0),
    (-1.745, 1.745),
    (-1.22, 1.22),
    (-2.0944, 2.0944),
]
# joint7: 0.0 = fully closed, 0.035 = fully open (meters). joint8 mirrors
# -joint7 inside the Isaac bridge, so callers only ever set joint7.
GRIPPER_LIMITS = (0.0, 0.035)

_DISCOVERY_TIMEOUT_S = 8.0
# Generous margin: under GPU/render load the WebRTC-streaming Isaac process can
# advance simulated time slower than wall clock, so convergence can take
# noticeably longer than `duration` even for small joint deltas.
_FEEDBACK_TIMEOUT_MARGIN_S = 20.0
_FEEDBACK_TOLERANCE = 0.015
_STABLE_SAMPLES = 5

_state = {"node": None, "executor": None, "arm_pub": None, "gripper_pub": None, "latest": None}


def _ensure_ready() -> None:
    if os.environ.get("ROS_DOMAIN_ID") != "129":
        raise RuntimeError(
            "ROS_DOMAIN_ID is not 129. Run your script through "
            "tools/run_with_robot129_control.sh so the ROS environment matches "
            "the Robot 129 simulation bridge (domain 129, namespace /robot129_sim)."
        )
    if _state["node"] is not None:
        return

    rclpy.init(args=None)
    node = rclpy.create_node("robot129_control_api", namespace="/robot129_sim")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    arm_pub = node.create_publisher(JointTrajectory, "/robot129_sim/arm_controller/joint_trajectory", 10)
    gripper_pub = node.create_publisher(JointTrajectory, "/robot129_sim/gripper_controller/joint_trajectory", 10)

    def on_state(msg: JointState) -> None:
        if list(msg.name) == ALL_NAMES and len(msg.position) == 8:
            _state["latest"] = np.asarray(msg.position, dtype=np.float64)

    node.create_subscription(JointState, "/robot129_sim/joint_states", on_state, 10)

    deadline = time.monotonic() + _DISCOVERY_TIMEOUT_S
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if arm_pub.get_subscription_count() > 0 and gripper_pub.get_subscription_count() > 0:
            break
    discovered = arm_pub.get_subscription_count() > 0 and gripper_pub.get_subscription_count() > 0
    if not discovered:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(
            "Isaac Sim ROS bridge was not discovered on domain 129 within "
            f"{_DISCOVERY_TIMEOUT_S:.0f}s. Start it first with: "
            "bash tools/start_robot129_ros_webrtc.sh"
        )

    _state.update(node=node, executor=executor, arm_pub=arm_pub, gripper_pub=gripper_pub)
    atexit.register(shutdown)


def shutdown() -> None:
    """Cleanly release the ROS node. Called automatically at process exit."""
    if _state["node"] is None:
        return
    _state["executor"].remove_node(_state["node"])
    _state["node"].destroy_node()
    rclpy.shutdown()
    _state.update(node=None, executor=None, arm_pub=None, gripper_pub=None, latest=None)


def _make_point(positions: Sequence[float], duration: float) -> JointTrajectoryPoint:
    point = JointTrajectoryPoint()
    point.positions = [float(v) for v in positions]
    point.time_from_start.sec = int(duration)
    point.time_from_start.nanosec = int(round((duration - int(duration)) * 1e9))
    return point


def _wait_for_convergence(indices: np.ndarray, target: np.ndarray, duration: float) -> dict:
    deadline = time.monotonic() + duration + _FEEDBACK_TIMEOUT_MARGIN_S
    stable = 0
    max_error = None
    while time.monotonic() < deadline:
        _state["executor"].spin_once(timeout_sec=0.05)
        latest = _state["latest"]
        if latest is not None:
            max_error = float(np.max(np.abs(latest[indices] - target)))
            stable = stable + 1 if max_error < _FEEDBACK_TOLERANCE else 0
            if stable >= _STABLE_SAMPLES:
                break
    latest = _state["latest"]
    return {
        "status": "PASS" if stable >= _STABLE_SAMPLES else "FAIL",
        "target": target.tolist(),
        "feedback": latest[indices].tolist() if latest is not None else None,
        "max_error": max_error,
    }


def _check_duration(duration: float) -> None:
    if not 0.1 <= duration <= 30:
        raise ValueError("duration must be between 0.1 and 30 seconds")


def set_joint_positions(positions: Sequence[float], duration: float = 2.0, wait: bool = True) -> dict:
    """Send a joint1..joint6 target (radians) to the Robot 129 arm.

    Args:
        positions: exactly 6 values, joint1..joint6, in radians.
        duration: seconds to interpolate to the target (0.1-30).
        wait: if True (default), block until /robot129_sim/joint_states
            confirms convergence and return a report dict with "status"
            "PASS"/"FAIL". If False, publish and return immediately with
            "status": "SENT".
    """
    positions = list(positions)
    if len(positions) != 6:
        raise ValueError(f"set_joint_positions expects 6 values (joint1..joint6), got {len(positions)}")
    for value, (lo, hi), name in zip(positions, ARM_LIMITS, ARM_NAMES):
        if not lo - 1e-6 <= value <= hi + 1e-6:
            raise ValueError(f"{name}={value} is outside its joint limit [{lo}, {hi}]")
    _check_duration(duration)

    _ensure_ready()
    msg = JointTrajectory()
    msg.joint_names = ARM_NAMES
    msg.points = [_make_point(positions, duration)]
    _state["arm_pub"].publish(msg)

    if not wait:
        return {"status": "SENT", "positions": positions}
    return _wait_for_convergence(np.arange(6), np.asarray(positions, dtype=np.float64), duration)


def set_gripper(value: float, duration: float = 1.5, wait: bool = True) -> dict:
    """Send a joint7 target (meters, 0.0=closed .. 0.035=open) to the gripper.

    joint8 is mirrored automatically inside the Isaac bridge (joint8 = -joint7).
    """
    lo, hi = GRIPPER_LIMITS
    if not lo - 1e-6 <= value <= hi + 1e-6:
        raise ValueError(f"gripper value {value} is outside its limit [{lo}, {hi}]")
    _check_duration(duration)

    _ensure_ready()
    msg = JointTrajectory()
    msg.joint_names = ["joint7"]
    msg.points = [_make_point([value], duration)]
    _state["gripper_pub"].publish(msg)

    if not wait:
        return {"status": "SENT", "value": value}
    return _wait_for_convergence(np.array([6]), np.asarray([value], dtype=np.float64), duration)


def open_gripper(duration: float = 1.5, wait: bool = True) -> dict:
    """Fully open the gripper (joint7 -> 0.035 m)."""
    return set_gripper(GRIPPER_LIMITS[1], duration=duration, wait=wait)


def close_gripper(duration: float = 1.5, wait: bool = True) -> dict:
    """Fully close the gripper (joint7 -> 0.0 m)."""
    return set_gripper(GRIPPER_LIMITS[0], duration=duration, wait=wait)
