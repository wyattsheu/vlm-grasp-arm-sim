#!/usr/bin/env python3
"""Send an arbitrary 6-axis joint command (or a whole sequence of them) to the Robot 129
simulation and verify the arm actually reached it -- unlike tools/send_robot129_ros_pose.py
(5 named presets only, no arbitrary values), this accepts any 6 numbers, a --sequence YAML
file, or -i interactive input, and reports commanded-vs-actual error PER AXIS plus the
resulting gripper approach direction via FK ("did joint5=X really rotate that axis, and
does the gripper now point where I said it would") -- see
docs/dev_guide_paper_core_and_dashboard_plan.md §8 Phase 3 / demo scenario `joints`.

Uses the same JointTrajectory + settle-detection pattern as send_robot129_ros_pose.py
(arm_controller / gripper_controller topics, 5 consecutive samples under tolerance = done),
generalized to arbitrary positions and reporting.

REFUSES to run if /robot129_sim/piper/joint_cmd has an active publisher (e.g. reactive.py):
see handle_piper_joint_cmd() in sim/scripts/run_robot129_ros_webrtc.py -- "a streamed
command supersedes any active JointTrajectory plan", so a JointTrajectory sent while that's
running would just get silently overwritten, not fail loudly.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState as RosJointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "research" / "src"))

from mpg.urdf_fk import ARM_JOINT_NAMES, UrdfChainFk, classify_approach_direction  # noqa: E402

ARM_NAMES = list(ARM_JOINT_NAMES)
ALL_NAMES = [f"joint{i}" for i in range(1, 9)]
URDF_PATH = ROOT / "ros2_ws" / "src" / "robot129_description" / "urdf" / "robot129.urdf"
AB_POSES_PATH = ROOT / "research" / "configs" / "demo" / "ab_poses.yaml"

# From robot129.urdf's <limit> elements (docs/dev_guide_paper_core_and_dashboard_plan.md
# §7.1) -- checked client-side too (not just relying on the simulator's own make_callback
# check) so a bad value gets a clear message before anything is sent.
JOINT_LIMITS_RAD = {
    "joint1": (-2.618, 2.618), "joint2": (0.0, 3.140), "joint3": (-2.967, 0.0),
    "joint4": (-1.745, 1.745), "joint5": (-1.220, 1.220), "joint6": (-2.094, 2.094),
}
GRIPPER_OPEN_M_DEFAULT = 0.07  # 2 x joint7's 0.035m range -- matches HOME's fully-open gripper
# Matches ros2_ws/src/robot129_sim_execution/config/adapter_params.yaml's
# arm_goal_tolerance_rad -- NOT independently chosen. Tried 0.01 first; a real live test
# (2026-09-24, demo point A: joint1=-2.158 rad, far from HOME) never settled below
# max_error=0.013 rad even after 60s wall-clock, a genuine PD-controller (stiffness=800,
# damping=80, no gravity compensation) steady-state offset for that pose, not a timeout
# problem -- this repo already hit the same class of issue and settled on 0.03 rad for
# the execution chain's own adapter, so reusing that number here instead of inventing a
# stricter one that this controller may not actually be able to reach for some poses.
SETTLE_TOLERANCE_RAD = 0.03
SETTLE_STABLE_SAMPLES = 5
# Wall-clock, not sim-time: this machine has repeatedly measured ~0.16x realtime under GPU
# contention (docs/dev_guide_paper_core_and_dashboard_plan.md §7.6/§7.8), so a "2.5s"
# trajectory can take ~15s of wall time to actually complete server-side. 30s default
# gives real margin; --settle-timeout can raise it further under worse contention.
SETTLE_TIMEOUT_S_DEFAULT = 30.0


def build_trajectory(names, positions, duration_s):
    msg = JointTrajectory()
    msg.joint_names = names
    point = JointTrajectoryPoint()
    point.positions = [float(v) for v in positions]
    point.time_from_start.sec = int(duration_s)
    point.time_from_start.nanosec = int(round((duration_s - int(duration_s)) * 1e9))
    msg.points = [point]
    return msg


def check_joint_limits(joints_rad) -> str | None:
    for name, value in zip(ARM_NAMES, joints_rad):
        lo, hi = JOINT_LIMITS_RAD[name]
        if not (lo <= value <= hi):
            return f"{name}={value:.4f} rad is outside [{lo}, {hi}]"
    return None


class JointCmdSender:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("send_joint_cmd", namespace="/robot129_sim")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.arm_pub = self.node.create_publisher(JointTrajectory, "/robot129_sim/arm_controller/joint_trajectory", 10)
        self.gripper_pub = self.node.create_publisher(JointTrajectory, "/robot129_sim/gripper_controller/joint_trajectory", 10)
        # A throwaway subscription, purely to read get_publisher_count() on
        # /robot129_sim/piper/joint_cmd -- see module docstring's REFUSES note. Message
        # type must match the real publisher's (sensor_msgs/JointState, see reactive.py's
        # reactive_node()) for the topic to actually resolve to the same one.
        self._piper_cmd_probe = self.node.create_subscription(RosJointState, "/robot129_sim/piper/joint_cmd", lambda _msg: None, 1)
        self.latest_feedback = None
        self.node.create_subscription(RosJointState, "/robot129_sim/joint_states", self._on_state, 10)
        self.urdf = UrdfChainFk(URDF_PATH)

    def _on_state(self, msg):
        if list(msg.name) == ALL_NAMES and len(msg.position) == 8:
            self.latest_feedback = np.asarray(msg.position, dtype=np.float64)

    def reactive_controller_active(self) -> bool:
        self.executor.spin_once(timeout_sec=0.3)
        return self._piper_cmd_probe.get_publisher_count() > 0

    def wait_for_subscribers(self, timeout_s: float = 8.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            if self.arm_pub.get_subscription_count() > 0 and self.gripper_pub.get_subscription_count() > 0:
                return True
        return False

    def send_and_wait(self, joints_rad, gripper_m: float, duration_s: float, settle_timeout_s: float = SETTLE_TIMEOUT_S_DEFAULT) -> dict:
        expected = np.array(list(joints_rad) + [gripper_m / 2.0, -gripper_m / 2.0], dtype=np.float64)
        self.arm_pub.publish(build_trajectory(ARM_NAMES, joints_rad, duration_s))
        self.gripper_pub.publish(build_trajectory(["joint7"], [gripper_m / 2.0], duration_s))

        deadline = time.monotonic() + duration_s + settle_timeout_s
        stable = 0
        max_error = None
        per_axis_error = None
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            if self.latest_feedback is not None:
                per_axis_error = self.latest_feedback - expected
                max_error = float(np.max(np.abs(per_axis_error)))
                stable = stable + 1 if max_error < SETTLE_TOLERANCE_RAD else 0
                if stable >= SETTLE_STABLE_SAMPLES:
                    break

        settled = stable >= SETTLE_STABLE_SAMPLES
        actual6 = self.latest_feedback[:6] if self.latest_feedback is not None else None
        approach_direction = approach_angle_deg = None
        pinch_position_m = None
        if actual6 is not None:
            pos, quat = self.urdf.pinch_center_pose(actual6.tolist())
            approach_direction, approach_angle_deg = classify_approach_direction(quat)
            pinch_position_m = pos.tolist()

        return {
            "commanded_joints_rad": [float(v) for v in joints_rad],
            "commanded_gripper_m": gripper_m,
            "actual_joints_rad": actual6.tolist() if actual6 is not None else None,
            "per_axis_error_rad": {n: float(per_axis_error[i]) for i, n in enumerate(ARM_NAMES)} if per_axis_error is not None else None,
            "max_error_rad": max_error,
            "settled": settled,
            "pinch_position_m": pinch_position_m,
            "approach_direction": approach_direction,
            "approach_angle_deg": approach_angle_deg,
        }

    def close(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        rclpy.shutdown()


def load_ab_pose(point_name: str) -> list[float]:
    if not AB_POSES_PATH.exists():
        raise FileNotFoundError(f"{AB_POSES_PATH} not found -- run research/scripts/select_demo_poses.py first")
    doc = yaml.safe_load(AB_POSES_PATH.read_text())
    return [float(v) for v in doc["points"][point_name]["joint_values_rad"]]


def load_sequence(path: Path) -> list[dict]:
    doc = yaml.safe_load(Path(path).read_text())
    steps_in = doc["steps"] if isinstance(doc, dict) else doc
    steps = []
    for i, entry in enumerate(steps_in):
        name = entry.get("name", f"step_{i}")
        if "from_ab_poses" in entry:
            joints_rad = load_ab_pose(entry["from_ab_poses"])
        elif "joints_deg" in entry:
            joints_rad = [math.radians(v) for v in entry["joints_deg"]]
        else:
            joints_rad = [float(v) for v in entry["joints_rad"]]
        steps.append({
            "name": name, "joints_rad": joints_rad,
            "gripper_m": float(entry.get("gripper_m", GRIPPER_OPEN_M_DEFAULT)),
            "duration_s": float(entry.get("duration_s", 2.0)),
        })
    return steps


def print_step_report(name: str, result: dict) -> None:
    print(f"--- {name} ---", flush=True)
    if not result["settled"]:
        print(f"  FAIL: did not settle within tolerance {SETTLE_TOLERANCE_RAD} rad (max_error={result['max_error_rad']})", flush=True)
        return
    print(f"  {'joint':<8}{'commanded':>12}{'actual':>12}{'error':>12}", flush=True)
    for i, n in enumerate(ARM_NAMES):
        commanded = result["commanded_joints_rad"][i]
        actual = result["actual_joints_rad"][i]
        error = result["per_axis_error_rad"][n]
        print(f"  {n:<8}{commanded:>12.4f}{actual:>12.4f}{error:>12.4f}", flush=True)
    print(f"  pinch_position_m = {np.round(result['pinch_position_m'], 4).tolist()}", flush=True)
    print(f"  approach ~= {result['approach_direction']} (off by {result['approach_angle_deg']:.1f} deg)", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--joints", type=float, nargs=6, metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    mode.add_argument("--sequence", type=Path)
    mode.add_argument("-i", "--interactive", action="store_true")
    parser.add_argument("--deg", action="store_true", help="--joints only: interpret the 6 values as degrees, not radians")
    parser.add_argument("--gripper", type=float, default=GRIPPER_OPEN_M_DEFAULT, help="--joints only: total gripper opening in metres")
    parser.add_argument("--duration", type=float, default=2.0, help="--joints only: trajectory duration in seconds")
    parser.add_argument("--settle-timeout", type=float, default=SETTLE_TIMEOUT_S_DEFAULT, help="extra wall-clock seconds to wait for settling after --duration elapses (see SETTLE_TIMEOUT_S_DEFAULT's comment on this machine's slow realtime factor)")
    parser.add_argument("--out", type=Path, default=None, help="default out/demo/joints/<timestamp>/joint_response.json")
    args = parser.parse_args()

    if os.environ.get("ROS_DOMAIN_ID") != "129":
        print("REFUSED: ROS_DOMAIN_ID must be 129", flush=True)
        return 2

    sender = JointCmdSender()
    try:
        if sender.reactive_controller_active():
            print(
                "REFUSED: /robot129_sim/piper/joint_cmd has an active publisher (reactive.py?) -- "
                "a JointTrajectory sent now would be silently overwritten next control tick. "
                "Stop the reactive controller first.", flush=True,
            )
            return 3
        if not sender.wait_for_subscribers():
            print("FAIL: Isaac's arm_controller/gripper_controller were not discovered on ROS domain 129", flush=True)
            return 4

        if args.joints is not None:
            joints_rad = [math.radians(v) for v in args.joints] if args.deg else list(args.joints)
            bad = check_joint_limits(joints_rad)
            if bad:
                print(f"REFUSED: {bad}", flush=True)
                return 5
            steps = [{"name": "manual", "joints_rad": joints_rad, "gripper_m": args.gripper, "duration_s": args.duration}]
        elif args.sequence is not None:
            steps = load_sequence(args.sequence)
            for step in steps:
                bad = check_joint_limits(step["joints_rad"])
                if bad:
                    print(f"REFUSED: step {step['name']!r}: {bad}", flush=True)
                    return 5
        else:
            steps = []
            print("Interactive mode -- enter 6 joint values in radians (space-separated), or 'q' to quit.", flush=True)
            while True:
                try:
                    line = input("j1 j2 j3 j4 j5 j6> ").strip()
                except EOFError:
                    break
                if line.lower() in ("q", "quit", "exit", ""):
                    break
                try:
                    values = [float(v) for v in line.split()]
                    if len(values) != 6:
                        raise ValueError("need exactly 6 values")
                    bad = check_joint_limits(values)
                    if bad:
                        print(f"  REFUSED: {bad}", flush=True)
                        continue
                except ValueError as exc:
                    print(f"  invalid input: {exc}", flush=True)
                    continue
                result = sender.send_and_wait(values, args.gripper, args.duration, args.settle_timeout)
                print_step_report("interactive", result)
            return 0

        results = []
        all_ok = True
        for step in steps:
            result = sender.send_and_wait(step["joints_rad"], step["gripper_m"], step["duration_s"], args.settle_timeout)
            print_step_report(step["name"], result)
            results.append({"name": step["name"], **result})
            all_ok = all_ok and result["settled"]

        out_path = args.out or (ROOT / "out" / "demo" / "joints" / time.strftime("%Y%m%d_%H%M%S") / "joint_response.json")
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps({"status": "PASS" if all_ok else "FAIL", "steps": results}, indent=2))
        print(f"wrote {out_path} (status={'PASS' if all_ok else 'FAIL'})", flush=True)
        return 0 if all_ok else 1
    finally:
        sender.close()


if __name__ == "__main__":
    raise SystemExit(main())
