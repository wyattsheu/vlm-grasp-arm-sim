#!/usr/bin/env python3
"""S2 executor: read an MTC-exported sub_trajectories JSON (from
robot129_tasks/robot129_mtc_pick_place) and play each non-empty segment through the
robot129_sim_execution FollowJointTrajectory adapter against the live Isaac
pick_place scene, gating on real contact/lift evidence exactly like
tools/verify_grasp_motion_manual_waypoints.py did for hand-solved joint angles.

Unlike that manual-waypoint script, the joint targets here come from a real MTC plan
(OMPL RRTConnect for joint-space moves, CartesianPath for the linear approach/lift/
lower/retreat segments), so this is the actual S2 milestone: MoveIt Task Constructor
plan -> FollowJointTrajectory adapter -> live Isaac physical pick-and-place.

Usage:
  ros2 run robot129_sim_execution run_grasp_motion --plan out/grasp_motion/mtc_pick_place.json [--record]
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot129_sim_execution import robot_model as rm

# Fallback only -- see _load_default_rest_z_m() below. This used to be the sole value,
# silently wrong by ~2.5mm for any non-cube object (e.g. the pick_place_hammer scene's
# handle, rest_z=0.015 not 0.0175) since place_error_z_m compared against a cube-shaped
# assumption regardless of what was actually being placed.
_FALLBACK_REST_Z_M = 0.0175
_SCENE_GEOMETRY_PATH = Path(
    "/mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913/research/configs/scene_geometry.json"
)
LIFT_THRESHOLD_M = 0.03
PLACE_XY_TOLERANCE_M = 0.02
PLACE_Z_TOLERANCE_M = 0.01


def _load_default_rest_z_m() -> float:
    """cube.rest_z_m from research/configs/scene_geometry.json (the single source of
    truth introduced 2026-09-20 -- see that file and robot129_mtc_pick_place.cpp's
    matching loader). Only correct for the cube; a hammer-handle run must pass
    --rest-z-m explicitly (the handle's own rest height, not the cube's), since this
    file has no notion of "what object is this executor currently placing"."""
    try:
        geometry = json.loads(_SCENE_GEOMETRY_PATH.read_text())
        return float(geometry["cube"]["rest_z_m"])
    except Exception as exc:  # noqa: BLE001 -- any failure just falls back, loudly
        print(f"WARN: failed to load {_SCENE_GEOMETRY_PATH} ({exc}); "
              f"using hardcoded fallback rest_z_m={_FALLBACK_REST_Z_M}")
        return _FALLBACK_REST_Z_M


class Executor(Node):
    def __init__(self):
        super().__init__("grasp_motion_executor")
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
        )
        cg = ReentrantCallbackGroup()
        self.cube_pose = None
        self.create_subscription(PoseStamped, rm.OBJECT_POSE_TOPIC, self._on_pose, sensor_qos, callback_group=cg)
        self.arm_client = ActionClient(self, FollowJointTrajectory, rm.ARM_ACTION_NAME, callback_group=cg)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, rm.GRIPPER_ACTION_NAME, callback_group=cg)
        self.reset_client = self.create_client(Trigger, rm.RESET_SCENE_SERVICE, callback_group=cg)
        self.recording_client = self.create_client(SetBool, rm.RECORDING_SERVICE, callback_group=cg)

    def _on_pose(self, msg):
        self.cube_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)


def call_sync(client, request, timeout_s):
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError(f"service {client.srv_name} unavailable")
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_s
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not future.done():
        raise RuntimeError(f"service {client.srv_name} call timed out")
    return future.result()


def send_segment(node: Executor, client: ActionClient, segment: dict, timeout_s: float):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = segment["joint_names"]
    for pt in segment["points"]:
        p = JointTrajectoryPoint()
        p.positions = pt["positions"]
        ns = int(round(pt["time_from_start_s"] * 1e9))
        p.time_from_start.sec, p.time_from_start.nanosec = ns // 1_000_000_000, ns % 1_000_000_000
        goal.trajectory.points.append(p)

    if not client.wait_for_server(timeout_sec=10.0):
        return "SERVER_UNAVAILABLE", None
    send_future = client.send_goal_async(goal)
    deadline = time.monotonic() + 10.0
    while not send_future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    if not send_future.done():
        return "GOAL_SEND_TIMEOUT", None
    goal_handle = send_future.result()
    if not goal_handle.accepted:
        return "REJECTED", None

    result_future = goal_handle.get_result_async()
    deadline = time.monotonic() + timeout_s
    while not result_future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    if not result_future.done():
        return "TIMEOUT", None
    result = result_future.result()
    status = "SUCCEEDED" if result.status == 4 else f"STATUS_{result.status}"
    return status, result.result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--out-dir", type=Path, default=None)
    parser.add_argument("--segment-timeout-s", type=float, default=60.0)
    parser.add_argument(
        "--rest-z-m", type=float, default=None,
        help="Expected resting height (m) of the placed object's tracked pose, for "
             "place_error_z_m. Defaults to research/configs/scene_geometry.json's "
             "cube.rest_z_m -- pass this explicitly when executing against a "
             "differently-shaped object (e.g. the pick_place_hammer handle, 0.015).",
    )
    args = parser.parse_args()
    rest_z_m = args.rest_z_m if args.rest_z_m is not None else _load_default_rest_z_m()

    plan = json.loads(args.plan.read_text())
    if plan.get("status") != "PASS" or not plan.get("sub_trajectories"):
        print(f"FAIL: plan file status={plan.get('status')}, nothing to execute")
        return 2

    def is_noop(seg: dict) -> bool:
        # MTC sometimes plans a genuine zero-motion segment (e.g. the clamp-to-bounds
        # stage already left the gripper at/near a group_state's goal): a single point
        # at time_from_start_s==0. The adapter correctly treats a t=0 first point that
        # matches the current target as "nothing to do after dropping it" and rejects
        # it -- that is the adapter behaving correctly, not a failure to execute here.
        pts = seg.get("points", [])
        return len(pts) == 1 and pts[0]["time_from_start_s"] <= 0.0

    all_segments = [s for s in plan["sub_trajectories"] if s.get("joint_names")]
    segments = [s for s in all_segments if not is_noop(s)]
    print(f"Loaded {len(all_segments)} non-empty segments from {args.plan}, "
          f"{len(all_segments) - len(segments)} are zero-motion no-ops (skipped)")

    out_dir = args.out_dir or Path("out/grasp_motion/mtc_execution") / time.strftime("%Y%m%dT%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Executor()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    import threading
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        reset_result = call_sync(node.reset_client, Trigger.Request(), 10.0)
        if not reset_result.success:
            # Previously ignored: if the runner rejects the reset (its own busy guard
            # returns success=False, e.g. "a trajectory plan is active"), this executor
            # would silently continue and execute the plan against whatever scene state
            # happened to be left over from a prior run instead of the intended reset
            # one. Fail loudly instead of grading a run against an unknown starting state.
            print(f"FAIL: reset_scene rejected: {reset_result.message}")
            return 3
        time.sleep(1.5)
        if args.record:
            call_sync(node.recording_client, SetBool.Request(data=True), 5.0)

        log = []
        rest_pose = node.cube_pose
        aborted_at = None
        for i, seg in enumerate(segments):
            client = node.gripper_client if seg["joint_names"] == rm.GRIPPER_JOINT_NAMES else node.arm_client
            duration = seg["points"][-1]["time_from_start_s"] if seg["points"] else 0.0
            timeout = max(args.segment_timeout_s, duration / 0.08 + 10.0)
            status, result = send_segment(node, client, seg, timeout)
            entry = {
                "index": i, "comment": seg.get("comment", ""), "joint_names": seg["joint_names"],
                "status": status, "error_string": getattr(result, "error_string", None),
                "cube_pose": node.cube_pose,
            }
            log.append(entry)
            print(entry, flush=True)
            if status != "SUCCEEDED":
                aborted_at = i
                break
            time.sleep(0.2)

        if args.record:
            call_sync(node.recording_client, SetBool.Request(data=False), 5.0)
            time.sleep(1.0)

        # A segment's "comment" is the SOLVER's own message (e.g. "Solution generated
        # by RRTConnect"), not the originating MTC stage name -- moveit_task_constructor
        # only resolves stage names through an Introspection object, which the exporter
        # in robot129_mtc_pick_place.cpp does not attach (see toMsg(solution_msg,
        # nullptr) there). So rather than guess a stage identity from that text,
        # determine "was the object actually lifted" directly from the measured cube
        # height: take the highest point reached anywhere after the gripper's first
        # successful close (grasp), which is unambiguous regardless of stage naming.
        first_gripper_close_index = next(
            (i for i, e in enumerate(log) if e["joint_names"] == rm.GRIPPER_JOINT_NAMES and e["status"] == "SUCCEEDED"),
            None,
        )
        final_pose = log[-1]["cube_pose"] if log else None

        checks = {"trajectory_success": aborted_at is None}
        if rest_pose and first_gripper_close_index is not None:
            post_grasp_heights = [
                e["cube_pose"][2] for e in log[first_gripper_close_index:] if e["cube_pose"]
            ]
            peak_height = max(post_grasp_heights) if post_grasp_heights else rest_pose[2]
            lift_delta = peak_height - rest_pose[2]
            checks["lift_delta_m"] = lift_delta
            checks["physical_grasp_success"] = lift_delta > LIFT_THRESHOLD_M
        else:
            checks["physical_grasp_success"] = False
        if final_pose:
            checks["final_cube_pose"] = final_pose
            checks["place_error_z_m"] = abs(final_pose[2] - rest_z_m)
            checks["task_success"] = aborted_at is None and checks["place_error_z_m"] <= PLACE_Z_TOLERANCE_M * 3
        else:
            checks["task_success"] = False

        report = {
            "status": "PASS" if all([checks.get("trajectory_success"), checks.get("physical_grasp_success"),
                                      checks.get("task_success")]) else "FAIL",
            "scope": "mtc_planned_execution",
            "plan_success": plan.get("status"),
            "plan_source": str(args.plan),
            "rest_z_m": rest_z_m,
            "kinematic_attachment": False,
            "aborted_at": aborted_at,
            "checks": checks,
            "log": log,
        }
        (out_dir / "report.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
        return 0 if report["status"] == "PASS" else 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
