"""cuRobo grasp planner for robot129, wired to the EXISTING execution chain.

Reads a grasp_candidate_v0 JSON (research/src/mpg/grasp_contract.py, the same format
research/src/mpg/grasp_candidates.py or tools/generate_s2_cube_candidates.py produce),
plans pick (approach -> grasp -> close -> lift) and a simple place (transport -> lower
-> open) with cuRoboV2, and writes the SAME sub_trajectories JSON shape MTC's
robot129_mtc_pick_place node already writes -- so the existing, unmodified
run_grasp_motion.py / FollowJointTrajectory adapter replays it, and results are directly
A/B-comparable against an MTC run of the same candidates.

Must run under env_robot129_curobo (needs `curobo`), NOT env_robot129_research. See
docs/dev_guide_paper_core_and_dashboard_plan.md §7.6 for exact commands.

Known simplifications, not silently hidden (see docs/progress/curobo_step3_plan_grasp.md
for what's actually been verified against Isaac vs. only against cuRobo's own planner):
- The target object itself is NOT in cuRobo's collision world (only floor / pedestal /
  optionally the back-camera pole). MTC models the grasped object as an attached
  collision body during lift; this script does not yet. For our top-down candidates the
  approach/grasp/lift path is short and mostly vertical, so the practical risk is low,
  but this is a real gap for cluttered scenes.
- Only top-down candidates are supported for the lift/place height math (grasp_lift_axis
  in TOOL frame, which only means "world +z" because top-down grasps have tool +z ==
  world -z -- see research/src/mpg/grasp_candidates.py's top_down_grasp_quaternion).
- "Place" is a plain transport (plan_pose to place_xy at lift height) + lower (plan_pose
  down to the original grasp height) + open, not MTC's separately-constrained Cartesian
  lower stage or its retreat stage.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT / "research" / "src"))

import numpy as np
import torch

from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.scene import Cuboid, Scene
from curobo.types import GoalToolPose, JointState, Pose

from mpg.curobo_bridge.frames import candidate_tcp_pose_to_curobo
from mpg.grasp_contract import read_candidates_json

DEFAULT_ROBOT_CONFIG = REPO_ROOT / "research" / "configs" / "curobo" / "robot129.yml"
DEFAULT_SCENE_GEOMETRY = REPO_ROOT / "research" / "configs" / "scene_geometry.json"
DEFAULT_SCENE_CAMERA = REPO_ROOT / "research" / "configs" / "scene_camera.yaml"

ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINT_NAMES = ["joint7"]  # matches ros2_ws/.../robot129_sim_execution/robot_model.py
# SRDF open/closed (ros2_ws/src/robot129_moveit_config/config/robot129.srdf), same values
# MTC's "open gripper"/"close gripper" stages use -- not re-derived per candidate width,
# matching MTC's own (working) choice to always close to the same near-zero target and
# let contact + the implicit PD actuator stop it, not a computed target width.
GRIPPER_OPEN_M = 0.0345
GRIPPER_CLOSED_M = 0.004

# Approach standoff / lift height: matches tools/generate_s2_cube_candidates.py's
# pregrasp_standoff_m=0.05, and MTC's lift stage setMinMaxDistance(0.04, 0.06)
# (robot129_mtc_pick_place.cpp) -- 0.05 sits in the middle of that range.
APPROACH_STANDOFF_M = 0.05
LIFT_HEIGHT_M = 0.05

FAILURE_NO_ACCEPTED_CANDIDATES = "NO_ACCEPTED_CANDIDATES"
FAILURE_NO_IK = "NO_IK"
FAILURE_APPROACH_COLLISION = "APPROACH_COLLISION"
FAILURE_TRAJOPT_FAIL = "TRAJOPT_FAIL"


def _build_scene(scene_name: str, include_pole: bool, scene_geometry: dict, scene_camera: dict | None) -> tuple[Scene, float]:
    """floor (+ optional pedestal, + optional back-camera pole) as cuRobo Cuboid
    obstacles, matching sim/scripts/run_robot129_ros_webrtc.py's own geometry exactly
    (same source files) so cuRobo's world model doesn't silently diverge from what
    Isaac will actually execute against. Returns (scene, table_z_m).
    """
    cuboids = [
        # /World/DeepGrayFloor in the sim script: size (4,4,0.05), translation z=-0.025
        # -> top surface at world z=0.
        Cuboid(name="floor", pose=[0.0, 0.0, -0.025, 1.0, 0.0, 0.0, 0.0], dims=[4.0, 4.0, 0.05]),
    ]
    table_z_m = 0.0
    if scene_name == "pick_place_counter":
        counter = scene_geometry["counter"]
        cx, cy = counter["pedestal_center_xy_m"]
        sx, sy, sz = counter["pedestal_size_m"]
        table_z_m = sz
        cuboids.append(Cuboid(name="pedestal", pose=[cx, cy, sz / 2.0, 1.0, 0.0, 0.0, 0.0], dims=[sx, sy, sz]))
    if include_pole:
        if scene_camera is None:
            raise ValueError("--include-pole requires scene_camera.yaml to be readable")
        pole = scene_camera["pole_camera"]
        ox, oy, oz = pole["offset_from_base_m"]
        half = pole["pole"]["half_extent_m"]
        cuboids.append(Cuboid(name="scene_pole", pose=[ox, oy, oz / 2.0, 1.0, 0.0, 0.0, 0.0], dims=[half * 2.0, half * 2.0, oz]))
    return Scene(cuboid=cuboids), table_z_m


def _extract_segment(jt: JointState, last_tstep: torch.Tensor, arm_col_idx: list[int], comment: str, batch_idx: int = 0) -> dict:
    """Trim a padded interpolated JointState to its valid prefix (see module docstring
    in curobo's own motion_planning.py example: the raw buffer is a fixed-size horizon,
    e.g. 5000 steps, of which only [0:last_tstep) are meaningful) and keep only the arm
    joint columns (the gripper joint is locked in robot129.yml -- see
    docs/dev_guide_paper_core_and_dashboard_plan.md §7.1 -- so it isn't part of the
    optimized trajectory at all, only carried along at its locked value).
    """
    n = int(last_tstep.view(-1)[batch_idx].item())
    dt = float(jt.dt.view(-1)[batch_idx].item()) if jt.dt is not None else 0.05
    positions = jt.position[batch_idx, 0, :n, :][:, arm_col_idx].detach().cpu().numpy()
    points = [
        {"positions": row.tolist(), "time_from_start_s": round(i * dt, 6)}
        for i, row in enumerate(positions)
    ]
    return {"comment": comment, "joint_names": ARM_JOINT_NAMES, "points": points}


def _gripper_segment(comment: str, start_m: float, end_m: float, duration_s: float = 1.0) -> dict:
    return {
        "comment": comment,
        "joint_names": GRIPPER_JOINT_NAMES,
        "points": [
            {"positions": [start_m], "time_from_start_s": 0.0},
            {"positions": [end_m], "time_from_start_s": duration_s},
        ],
    }


def _pose_at(position_m, quaternion_wxyz, device: str) -> Pose:
    return Pose(
        position=torch.tensor([position_m], dtype=torch.float32, device=device),
        quaternion=torch.tensor([quaternion_wxyz], dtype=torch.float32, device=device),
    )


def plan(
    candidates: list[dict],
    *,
    scene_name: str,
    place_xy_m: tuple[float, float],
    start_joints_rad: list[float],
    robot_config: Path,
    include_pole: bool,
    device: str = "cuda",
) -> tuple[dict, dict]:
    """Returns (plan_dict, report_dict). plan_dict is the MTC-compatible
    sub_trajectories JSON (status/sub_trajectories/...); report_dict has cuRobo-specific
    diagnostics (which candidate was chosen, per-phase success, timings)."""
    t_start = time.monotonic()
    scene_geometry = json.loads(DEFAULT_SCENE_GEOMETRY.read_text())
    scene_camera = None
    if include_pole:
        import yaml

        scene_camera = yaml.safe_load(DEFAULT_SCENE_CAMERA.read_text())
    scene, table_z_m = _build_scene(scene_name, include_pole, scene_geometry, scene_camera)

    accepted = [c for c in candidates if c.get("accepted")]
    report = {
        "status": "PENDING", "scene": scene_name, "num_candidates_in": len(candidates),
        "num_accepted": len(accepted), "chosen_candidate_id": None, "failure_reason": None,
        "phase_success": {}, "planning_time_s": None,
    }
    if not accepted:
        report["status"] = "FAIL"
        report["failure_reason"] = FAILURE_NO_ACCEPTED_CANDIDATES
        return {"status": "FAIL", "sub_trajectories": [], "failures": [FAILURE_NO_ACCEPTED_CANDIDATES]}, report

    config = MotionPlannerCfg.create(robot=str(robot_config), scene_model=scene, max_goalset=max(len(accepted), 1))
    planner = MotionPlanner(config)
    planner.warmup(enable_graph=True, num_warmup_iterations=3)

    arm_col_idx = [planner.kinematics.all_articulated_joint_names.index(n) for n in ARM_JOINT_NAMES]

    q_start = JointState.from_position(
        torch.tensor([start_joints_rad], dtype=torch.float32, device=device), joint_names=ARM_JOINT_NAMES,
    )

    tool_frame = "pinch_center"
    poses = [candidate_tcp_pose_to_curobo(c, "tcp_pose") for c in accepted]
    positions = torch.tensor(np.stack([p.position_m for p in poses]), dtype=torch.float32, device=device)
    quaternions = torch.tensor(np.stack([p.quaternion_wxyz for p in poses]), dtype=torch.float32, device=device)
    goal_pose = Pose(position=positions, quaternion=quaternions)
    goalset = GoalToolPose.from_poses({tool_frame: goal_pose}, ordered_tool_frames=[tool_frame], num_goalset=len(accepted))

    grasp_result = planner.plan_grasp(
        goalset, q_start,
        grasp_approach_axis="z", grasp_approach_offset=-APPROACH_STANDOFF_M, grasp_approach_in_tool_frame=True,
        grasp_lift_axis="z", grasp_lift_offset=-LIFT_HEIGHT_M, grasp_lift_in_tool_frame=True,
    )
    report["phase_success"] = {
        "approach": bool(grasp_result.approach_success.any().item()) if grasp_result.approach_success is not None else False,
        "grasp": bool(grasp_result.grasp_success.any().item()) if grasp_result.grasp_success is not None else False,
        "lift": bool(grasp_result.lift_success.any().item()) if grasp_result.lift_success is not None else False,
    }
    if not grasp_result.success.any():
        report["status"] = "FAIL"
        # Distinguish "no candidate is even IK-reachable" from "reachable but the
        # approach/grasp/lift path collides" -- see the research plan's rule against
        # silent fallback; a caller needs to know which one happened.
        if not report["phase_success"]["approach"]:
            report["failure_reason"] = FAILURE_NO_IK if grasp_result.goalset_index is None else FAILURE_APPROACH_COLLISION
        else:
            report["failure_reason"] = FAILURE_TRAJOPT_FAIL
        report["curobo_status"] = grasp_result.status
        report["planning_time_s"] = time.monotonic() - t_start
        return {"status": "FAIL", "sub_trajectories": [], "failures": [report["failure_reason"]], "error": grasp_result.status}, report

    goal_index = int(grasp_result.goalset_index.view(-1)[0].item())
    chosen = accepted[goal_index]
    report["chosen_candidate_id"] = chosen.get("candidate_id")

    sub_trajectories = [
        _gripper_segment("ensure gripper open before approach", GRIPPER_OPEN_M, GRIPPER_OPEN_M, 0.1),
        _extract_segment(grasp_result.approach_interpolated_trajectory, grasp_result.approach_interpolated_last_tstep, arm_col_idx, "cuRobo approach"),
        _extract_segment(grasp_result.grasp_interpolated_trajectory, grasp_result.grasp_interpolated_last_tstep, arm_col_idx, "cuRobo grasp"),
        _gripper_segment("close gripper", GRIPPER_OPEN_M, GRIPPER_CLOSED_M, 1.0),
        _extract_segment(grasp_result.lift_interpolated_trajectory, grasp_result.lift_interpolated_last_tstep, arm_col_idx, "cuRobo lift"),
    ]

    # -- place: transport (plan_pose to place_xy at lift height) + lower (plan_pose down
    # to the original grasp height) + open. See module docstring for how this differs
    # from MTC's place stages.
    lift_end_js = JointState.from_position(
        grasp_result.lift_interpolated_trajectory.position[
            0, 0, int(grasp_result.lift_interpolated_last_tstep.view(-1)[0].item()) - 1, arm_col_idx
        ].unsqueeze(0),
        joint_names=ARM_JOINT_NAMES,
    )
    lift_z = float(poses[goal_index].position_m[2]) + LIFT_HEIGHT_M
    transport_pose = _pose_at([place_xy_m[0], place_xy_m[1], lift_z], poses[goal_index].quaternion_wxyz.tolist(), device)
    transport_goalset = GoalToolPose.from_poses({tool_frame: transport_pose}, ordered_tool_frames=[tool_frame], num_goalset=1)
    transport_result = planner.plan_pose(transport_goalset, lift_end_js)

    place_ok = transport_result is not None and bool(transport_result.success.any().item())
    report["phase_success"]["transport"] = place_ok
    if place_ok:
        sub_trajectories.append(
            _extract_segment(transport_result.interpolated_trajectory, transport_result.interpolated_last_tstep, arm_col_idx, "cuRobo transport")
        )
        transport_end_js = JointState.from_position(
            transport_result.interpolated_trajectory.position[
                0, 0, int(transport_result.interpolated_last_tstep.view(-1)[0].item()) - 1, arm_col_idx
            ].unsqueeze(0),
            joint_names=ARM_JOINT_NAMES,
        )
        lower_pose = _pose_at(
            [place_xy_m[0], place_xy_m[1], float(poses[goal_index].position_m[2])],
            poses[goal_index].quaternion_wxyz.tolist(), device,
        )
        lower_goalset = GoalToolPose.from_poses({tool_frame: lower_pose}, ordered_tool_frames=[tool_frame], num_goalset=1)
        lower_result = planner.plan_pose(lower_goalset, transport_end_js)
        lower_ok = lower_result is not None and bool(lower_result.success.any().item())
        report["phase_success"]["lower"] = lower_ok
        if lower_ok:
            sub_trajectories.append(
                _extract_segment(lower_result.interpolated_trajectory, lower_result.interpolated_last_tstep, arm_col_idx, "cuRobo lower")
            )
            sub_trajectories.append(_gripper_segment("open gripper (release)", GRIPPER_CLOSED_M, GRIPPER_OPEN_M, 1.2))
    else:
        report["phase_success"]["lower"] = False

    report["status"] = "PASS"
    report["planning_time_s"] = time.monotonic() - t_start
    return {
        "status": "PASS", "initialized": True, "planned_solutions": len(sub_trajectories),
        "error": None, "failures": [], "sub_trajectories": sub_trajectories,
    }, report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--scene", choices=["pick_place", "pick_place_hammer", "pick_place_counter"], default="pick_place")
    parser.add_argument("--place-xy", type=float, nargs=2, default=None, help="metres; default from scene_geometry.json")
    parser.add_argument(
        "--start-joints", type=float, nargs=6, default=None,
        help="6 arm joint values, radians; default SRDF home [0, 1.2, -1.25, 0, 0.15, 0]",
    )
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_ROBOT_CONFIG)
    parser.add_argument("--include-pole", action="store_true", help="add the back-camera pole as a known obstacle -- only if the sim was launched with --scene-camera pole")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--report", type=Path, default=None)
    args = parser.parse_args()

    candidates = read_candidates_json(args.candidates)
    scene_geometry = json.loads(DEFAULT_SCENE_GEOMETRY.read_text())
    if args.place_xy is not None:
        place_xy = tuple(args.place_xy)
    elif args.scene == "pick_place_counter":
        place_xy = tuple(scene_geometry["counter"]["place_xy_default_m"])
    else:
        place_xy = tuple(scene_geometry["place"]["xy_default_m"])
    start_joints = args.start_joints or [0.0, 1.2, -1.25, 0.0, 0.15, 0.0]

    plan_dict, report = plan(
        candidates, scene_name=args.scene, place_xy_m=place_xy, start_joints_rad=start_joints,
        robot_config=args.robot_config, include_pole=args.include_pole,
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(plan_dict, indent=2))
    print(f"wrote {args.out} status={plan_dict['status']}")
    report_path = args.report or args.out.with_name(args.out.stem + "_report.json")
    report_path.write_text(json.dumps(report, indent=2))
    print(f"wrote {report_path}")
    print(json.dumps(report, indent=2))
    return 0 if plan_dict["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
