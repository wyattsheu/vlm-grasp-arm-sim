#!/usr/bin/env python3
"""Publish a guarded Robot 129 simulation pose and verify Isaac joint-state feedback."""

import argparse
import json
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

POSES = {
    "home": [0.0, 1.20, -1.25, 0.0, 0.15, 0.0, 0.035],
    "inspect": [0.25, 1.35, -1.55, 0.10, 0.20, 0.0, 0.035],
    "grasp": [0.0, 1.20, -1.25, 0.0, 0.15, 0.0, 0.0],
    "lift": [0.0, 1.35, -1.75, 0.0, 0.15, 0.0, 0.0],
    "release": [0.0, 1.35, -1.75, 0.0, 0.15, 0.0, 0.035],
}
ARM_NAMES = [f"joint{i}" for i in range(1, 7)]
ALL_NAMES = [f"joint{i}" for i in range(1, 9)]
ROOT = Path(__file__).resolve().parents[1]


def message(names, positions, duration):
    msg = JointTrajectory()
    msg.joint_names = names
    point = JointTrajectoryPoint()
    point.positions = [float(value) for value in positions]
    point.time_from_start.sec = int(duration)
    point.time_from_start.nanosec = int(round((duration - int(duration)) * 1e9))
    msg.points = [point]
    return msg


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("pose", choices=sorted(POSES))
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "129":
        raise SystemExit("REFUSED: ROS_DOMAIN_ID must be 129")
    if not 0.1 <= args.duration <= 30:
        raise SystemExit("REFUSED: duration must be between 0.1 and 30 seconds")

    desired7 = np.asarray(POSES[args.pose], dtype=np.float64)
    expected = np.concatenate([desired7, [-desired7[6]]])
    feedback = []

    rclpy.init()
    node = rclpy.create_node("pose_sender", namespace="/robot129_sim")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    arm_pub = node.create_publisher(
        JointTrajectory, "/robot129_sim/arm_controller/joint_trajectory", 10
    )
    gripper_pub = node.create_publisher(
        JointTrajectory, "/robot129_sim/gripper_controller/joint_trajectory", 10
    )

    def on_state(msg):
        if list(msg.name) == ALL_NAMES and len(msg.position) == 8:
            feedback.append(np.asarray(msg.position, dtype=np.float64))

    subscription = node.create_subscription(
        JointState, "/robot129_sim/joint_states", on_state, 10
    )

    discovery_deadline = time.monotonic() + 8
    while time.monotonic() < discovery_deadline:
        executor.spin_once(timeout_sec=0.05)
        if arm_pub.get_subscription_count() > 0 and gripper_pub.get_subscription_count() > 0:
            break
    discovered = arm_pub.get_subscription_count() > 0 and gripper_pub.get_subscription_count() > 0
    if not discovered:
        report = {
            "status": "FAIL",
            "reason": "Isaac command subscribers were not discovered on ROS domain 129",
            "pose": args.pose,
        }
    else:
        arm_pub.publish(message(ARM_NAMES, desired7[:6], args.duration))
        gripper_pub.publish(message(["joint7"], desired7[6:], args.duration))
        deadline = time.monotonic() + args.duration + 8
        stable = 0
        max_error = None
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if feedback:
                max_error = float(np.max(np.abs(feedback[-1] - expected)))
                stable = stable + 1 if max_error < 0.015 else 0
                if stable >= 5:
                    break
        report = {
            "status": "PASS" if stable >= 5 else "FAIL",
            "simulation_only": True,
            "ros_domain_id": 129,
            "namespace": "/robot129_sim",
            "pose": args.pose,
            "duration_s": args.duration,
            "expected": expected.tolist(),
            "feedback": feedback[-1].tolist() if feedback else None,
            "max_error": max_error,
            "feedback_samples": len(feedback),
            "hardware_drivers": 0,
        }

    output = ROOT / "out/ros_webrtc_robot129/last_command.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
