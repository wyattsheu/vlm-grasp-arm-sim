#!/usr/bin/env python3
"""Prove Robot 129 simulation topics are isolated from ROS domain 0."""
import json
import time
from pathlib import Path

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState

ctx129, ctx0 = Context(), Context()
rclpy.init(context=ctx129, domain_id=129)
rclpy.init(context=ctx0, domain_id=0)
pub_node = rclpy.create_node("joint_state_source", namespace="/robot129_sim", context=ctx129)
sub129_node = rclpy.create_node("acceptance_listener", namespace="/robot129_sim", context=ctx129)
sub0_node = rclpy.create_node("production_domain_probe", namespace="/robot129_sim", context=ctx0)
topic = "/robot129_sim/joint_states"
pub = pub_node.create_publisher(JointState, topic, 10)
received129, received0 = [], []
sub129_node.create_subscription(JointState, topic, lambda m: received129.append(m), 10)
sub0_node.create_subscription(JointState, topic, lambda m: received0.append(m), 10)
exec129 = SingleThreadedExecutor(context=ctx129)
exec0 = SingleThreadedExecutor(context=ctx0)
exec129.add_node(sub129_node)
exec0.add_node(sub0_node)
deadline = time.monotonic() + 2.5
while time.monotonic() < deadline:
    msg = JointState()
    msg.name = [f"joint{i}" for i in range(1, 9)]
    msg.position = [0.0] * 8
    pub.publish(msg)
    exec129.spin_once(timeout_sec=0.03)
    exec0.spin_once(timeout_sec=0.01)
    time.sleep(0.02)
report = {"status": "PASS" if received129 and not received0 else "FAIL", "domain_129_received": len(received129), "domain_0_received": len(received0), "topic": topic, "namespace": "/robot129_sim", "hardware_drivers": 0}
out = Path(__file__).resolve().parents[1] / "out/lesson_08"
out.mkdir(parents=True, exist_ok=True)
(out / "ros_isolation.json").write_text(json.dumps(report, indent=2)+"\n")
print(json.dumps(report, indent=2))
for node in (pub_node, sub129_node, sub0_node): node.destroy_node()
exec129.shutdown(); exec0.shutdown()
ctx129.shutdown(); ctx0.shutdown()
raise SystemExit(0 if report["status"] == "PASS" else 1)
