"""A minimal stand-in for the live Isaac runner's control-chain contract, used to unit
test the FollowJointTrajectory adapter without needing Isaac Sim / a GPU.

Reproduces exactly the parts of sim/scripts/run_robot129_ros_webrtc.py the adapter
depends on: JointTrajectory subscriptions on the arm/gripper command topics, a
JointState publisher, and a trajectory_events publisher mirroring ACCEPT/REJECT/
COMPLETE. Deliberately does NOT reproduce the real runner's t=0-first-point handling
or interpolation quality -- it only needs to be "a plausible enough feedback source"
for the adapter's own logic (validation, busy policy, cancel, stale-feedback,
timeout, goal-tolerance) to be exercised, since that logic is what these tests are
checking, not the physics.
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from robot129_sim_execution import robot_model as rm

ALL_JOINT_NAMES = rm.ARM_JOINT_NAMES + rm.GRIPPER_JOINT_NAMES + ["joint8"]


class FakeRunner(Node):
    def __init__(self, name="fake_isaac_runner", publish_hz=30.0):
        super().__init__(name)
        self.lock = threading.Lock()
        self.positions = {n: 0.0 for n in ALL_JOINT_NAMES}
        self.plans = {"arm": None, "gripper": None}
        self.publish_joint_states = True
        self.reject_next = {"arm": None, "gripper": None}  # str reason or None
        self.silently_ignore_next = {"arm": False, "gripper": False}

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
        )
        self.state_pub = self.create_publisher(JointState, rm.JOINT_STATES_TOPIC, sensor_qos)
        self.events_pub = self.create_publisher(String, rm.TRAJECTORY_EVENTS_TOPIC, 10)
        self.create_subscription(JointTrajectory, rm.ARM_COMMAND_TOPIC, self._make_cb("arm", rm.ARM_JOINT_NAMES), 10)
        self.create_subscription(
            JointTrajectory, rm.GRIPPER_COMMAND_TOPIC, self._make_cb("gripper", rm.GRIPPER_JOINT_NAMES), 10
        )
        self._timer = self.create_timer(1.0 / publish_hz, self._tick)

    def _make_cb(self, key, expected_names):
        def cb(msg: JointTrajectory):
            if self.silently_ignore_next[key]:
                self.silently_ignore_next[key] = False
                return
            reason = self.reject_next[key]
            if reason is None and list(msg.joint_names) != expected_names:
                reason = f"joint_names must be {expected_names}"
            if reason is not None:
                self.reject_next[key] = None
                self._publish_event("REJECT", key, reason=reason)
                return
            with self.lock:
                self.plans[key] = {
                    "started": time.monotonic(),
                    "names": expected_names,
                    "target": list(msg.points[-1].positions),
                    "duration": _point_seconds(msg.points[-1]),
                }
        return cb

    def _publish_event(self, kind, channel, **fields):
        import json
        payload = {"kind": kind, "channel": channel}
        payload.update(fields)
        m = String()
        m.data = json.dumps(payload)
        self.events_pub.publish(m)

    def _tick(self):
        now = time.monotonic()
        with self.lock:
            for key, plan in list(self.plans.items()):
                if plan is None:
                    continue
                elapsed = now - plan["started"]
                frac = min(1.0, elapsed / plan["duration"]) if plan["duration"] > 0 else 1.0
                for name, target in zip(plan["names"], plan["target"]):
                    self.positions[name] = target * frac  # ramps from 0, good enough for tests
                if frac >= 1.0:
                    self.plans[key] = None
                    self._publish_event("COMPLETE", key)
        if self.publish_joint_states:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ALL_JOINT_NAMES
            msg.position = [self.positions[n] for n in ALL_JOINT_NAMES]
            self.state_pub.publish(msg)


def _point_seconds(point) -> float:
    return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
