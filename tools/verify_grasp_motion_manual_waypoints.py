#!/usr/bin/env python3
"""S1/S2 manual-waypoint pick-and-place verification against the LIVE Isaac
pick_place scene (start with tools/start_robot129_grasp_sim.sh --scene pick_place).

Scope, precisely: this sends a hand-computed (numerical IK via
tools/analyze_top_down_reach.py, not MoveIt/KDL) sequence of top-down arm poses and
gripper open/close commands directly to the runner's raw JointTrajectory topics. It
does NOT go through MoveIt, MoveIt Task Constructor, or the robot129_sim_execution
FollowJointTrajectory adapter -- those are exercised separately (adapter: its own
pytest suite in ros2_ws/src/robot129_sim_execution/test; MTC: robot129_tasks). What
this DOES verify, with real evidence, is:

  1. The S1 runner changes (accepting a MoveIt-style t=0 first trajectory point,
     sim-time-paced interpolation instead of wallclock smoothstep, the pick_place
     scene with a real-gravity/real-friction dynamic cube and contact sensors) work
     end to end against a live Isaac process.
  2. A real, from-the-floor, non-kinematic-attach pick-and-place is achievable with
     this robot geometry and joint limits (this was previously UNVERIFIED -- see
     docs/progress/grasp_motion_s0_inventory.md section 3 -- the existing physics
     grasp demo spawns its cube already floating between the fingers).

Joint targets were derived once via a 5-DOF numerical IK (position + top-down
approach-axis constraint) solved with tools/analyze_top_down_reach.py's forward
kinematics; see docs/progress/grasp_motion_s2_manual_waypoints.md for the derivation.
They are specific to the S0-chosen cube pose (0.32, 0.0, 0.0175) and place pose
(0.27, -0.12, 0.0175) and will not generalize to other object poses -- that
generalization is exactly what the S3 grasp-candidate generator plus MTC/KDL IK
(not this script) is for.
"""
from __future__ import annotations

import argparse
import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_NAMES = [f"joint{i}" for i in range(1, 7)]
GRIPPER_NAMES = ["joint7"]

# Numerically solved (damped-least-squares, 4 constraints: x,y,z,approach_z=-1) for
# S0's chosen scene layout. cube rest height 0.0175m, place rest height 0.0175m.
HOME_ARM = [0.0, 1.20, -1.25, 0.0, 0.15, 0.0]
PREGRASP = [1e-05, 1.74947, -1.17473, -2e-05, 1.08296, 0.0]
GRASP = [1e-05, 1.85951, -1.05031, 1e-05, 0.84902, 0.0]
PLACE_LIFT = [-0.41822, 1.66998, -1.06856, 1e-05, 1.05645, 0.0]
PLACE_GRASP = [-0.41821, 1.78851, -0.94089, 0.0, 0.81029, 0.0]
OPEN_WIDTH = 0.035
CLOSED_WIDTH = 0.0

CUBE_XY = (0.32, 0.0)
PLACE_XY = (0.27, -0.12)
REST_Z = 0.0175
LIFT_THRESHOLD_M = 0.03      # matches the pre-existing lesson_06 physics-grasp threshold
PLACE_XY_TOLERANCE_M = 0.02
PLACE_Z_TOLERANCE_M = 0.01


class Runner(Node):
    def __init__(self):
        super().__init__("grasp_motion_manual_waypoints")
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
        )
        cg_state, cg_pose, cg_event = ReentrantCallbackGroup(), ReentrantCallbackGroup(), ReentrantCallbackGroup()
        self.arm_pub = self.create_publisher(JointTrajectory, "/robot129_sim/arm_controller/joint_trajectory", 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, "/robot129_sim/gripper_controller/joint_trajectory", 10)
        self.joint_state = None
        self.cube_pose = None
        self.last_event = {"arm": None, "gripper": None}
        self.create_subscription(JointState, "/robot129_sim/joint_states", self._on_state, sensor_qos, callback_group=cg_state)
        self.create_subscription(
            PoseStamped, "/robot129_sim/objects/target_cube/pose", self._on_pose, sensor_qos, callback_group=cg_pose
        )
        self.create_subscription(String, "/robot129_sim/trajectory_events", self._on_event, 10, callback_group=cg_event)
        self.reset_client = self.create_client(Trigger, "/robot129_sim/reset_scene")
        self.recording_client = self.create_client(__import__("std_srvs.srv", fromlist=["SetBool"]).SetBool, "/robot129_sim/recording")

    def _on_state(self, msg):
        self.joint_state = dict(zip(msg.name, msg.position))

    def _on_pose(self, msg):
        self.cube_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _on_event(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if payload.get("kind") in ("COMPLETE", "REJECT"):
            self.last_event[payload["channel"]] = payload["kind"]


def _point(target, duration_s, t0=None):
    pts = []
    if t0 is not None:
        p0 = JointTrajectoryPoint()
        p0.positions = t0
        pts.append(p0)
    p1 = JointTrajectoryPoint()
    p1.positions = target
    ns = int(duration_s * 1e9)
    p1.time_from_start.sec, p1.time_from_start.nanosec = ns // 1_000_000_000, ns % 1_000_000_000
    pts.append(p1)
    return pts


def send_arm(node: Runner, target, duration_s):
    t0 = [node.joint_state[n] for n in ARM_NAMES] if node.joint_state else None
    msg = JointTrajectory()
    msg.joint_names = ARM_NAMES
    msg.points = _point(target, duration_s, t0)
    node.last_event["arm"] = None
    node.arm_pub.publish(msg)


def send_gripper(node: Runner, target, duration_s):
    msg = JointTrajectory()
    msg.joint_names = GRIPPER_NAMES
    msg.points = _point([target], duration_s)
    node.last_event["gripper"] = None
    node.gripper_pub.publish(msg)


def wait_complete(node: Runner, channel, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if node.last_event[channel] in ("COMPLETE", "REJECT"):
            return node.last_event[channel]
        time.sleep(0.05)
    return "TIMEOUT"


def call_sync(client, request, timeout_s):
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    return future.result()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--min-realtime-factor", type=float, default=0.10,
                         help="conservative floor for this deployment's observed ~0.16 real-time factor")
    parser.add_argument("--timeout-margin-s", type=float, default=8.0)
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args()

    def wt(duration_s):
        return duration_s / args.min_realtime_factor + args.timeout_margin_s

    out_dir = args.out_dir or Path("out/grasp_motion/manual_waypoints") / time.strftime("%Y%m%dT%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Runner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    deadline = time.monotonic() + 10.0
    while node.joint_state is None and time.monotonic() < deadline:
        time.sleep(0.1)
    if node.joint_state is None:
        print("FAIL: no /robot129_sim/joint_states received; is the pick_place sim running?")
        return 2

    reset_resp = call_sync(node.reset_client, __import__("std_srvs.srv", fromlist=["Trigger"]).Trigger.Request(), 10.0)
    time.sleep(1.5)

    from std_srvs.srv import SetBool
    if args.record:
        call_sync(node.recording_client, SetBool.Request(data=True), 5.0)

    steps = [
        ("approach", "arm", lambda: send_arm(node, PREGRASP, 2.5), wt(2.5)),
        ("descend", "arm", lambda: send_arm(node, GRASP, 1.2), wt(1.2)),
        ("close", "gripper", lambda: send_gripper(node, CLOSED_WIDTH, 1.0), wt(1.0)),
        ("lift", "arm", lambda: send_arm(node, PREGRASP, 1.2), wt(1.2)),
        ("transit", "arm", lambda: send_arm(node, PLACE_LIFT, 2.0), wt(2.0)),
        ("place_descend", "arm", lambda: send_arm(node, PLACE_GRASP, 1.2), wt(1.2)),
        ("open", "gripper", lambda: send_gripper(node, OPEN_WIDTH, 1.0), wt(1.0)),
        ("retreat", "arm", lambda: send_arm(node, PLACE_LIFT, 1.0), wt(1.0)),
        ("home", "arm", lambda: send_arm(node, HOME_ARM, 2.0), wt(2.0)),
    ]

    log = []
    aborted_at = None
    for name, channel, action, timeout in steps:
        action()
        status = wait_complete(node, channel, timeout)
        entry = {"step": name, "status": status, "cube_pose": node.cube_pose}
        log.append(entry)
        print(entry, flush=True)
        if status != "COMPLETE":
            aborted_at = name
            break
        time.sleep(0.2)

    if args.record:
        call_sync(node.recording_client, SetBool.Request(data=False), 5.0)
        time.sleep(1.0)

    rest_pose = next((e["cube_pose"] for e in log if e["step"] == "descend"), None)
    lift_pose = next((e["cube_pose"] for e in log if e["step"] == "lift"), None)
    final_pose = log[-1]["cube_pose"] if log else None

    checks = {"trajectory_success": aborted_at is None}
    if rest_pose and lift_pose:
        lift_delta = lift_pose[2] - rest_pose[2]
        checks["lift_delta_m"] = lift_delta
        checks["physical_grasp_success"] = lift_delta > LIFT_THRESHOLD_M
    else:
        checks["physical_grasp_success"] = False
    if final_pose:
        dx = final_pose[0] - PLACE_XY[0]
        dy = final_pose[1] - PLACE_XY[1]
        dz = final_pose[2] - REST_Z
        checks["place_error_xy_m"] = (dx**2 + dy**2) ** 0.5
        checks["place_error_z_m"] = abs(dz)
        checks["task_success"] = (
            checks["place_error_xy_m"] <= PLACE_XY_TOLERANCE_M
            and checks["place_error_z_m"] <= PLACE_Z_TOLERANCE_M
            and aborted_at is None
        )
    else:
        checks["task_success"] = False

    report = {
        "status": "PASS" if all([checks.get("trajectory_success"), checks.get("physical_grasp_success"),
                                  checks.get("task_success")]) else "FAIL",
        "scope": "manual_waypoints_no_moveit_no_adapter",
        "plan_success": "NOT_APPLICABLE_hand_computed_IK",
        "kinematic_attachment": False,
        "aborted_at": aborted_at,
        "checks": checks,
        "log": log,
        "cube_xy": CUBE_XY,
        "place_xy": PLACE_XY,
    }
    (out_dir / "report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
