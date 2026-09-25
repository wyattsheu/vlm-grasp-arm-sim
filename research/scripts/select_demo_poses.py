#!/usr/bin/env python3
"""Pick a demo A/B point-to-point pair with DIFFERENT gripper approach
directions (e.g. A points up, B points left), verified reachable by cuRobo's
own collision-aware IK -- not assumed.

Why this exists: docs/dev_guide_paper_core_and_dashboard_plan.md §7.1 records
that joint5 is limited to +-1.22 rad, so an arbitrary "gripper points up"
target is not guaranteed reachable on robot129/PiPER. Rather than hand-pick a
pose and discover at demo time it has no IK solution, this scans a small grid
of candidate positions for each requested direction, keeps only IK successes,
and reports the joint5 margin left at each one so a jittery near-limit
solution isn't silently chosen over a comfortable one.

Must run under env_robot129_curobo (needs `curobo`), NOT env_robot129_research.
See docs/dev_guide_paper_core_and_dashboard_plan.md §7 for exact commands.

Output: research/configs/demo/ab_poses.yaml (or --out), consumed by both
tools/send_joint_cmd.py (Phase 3) and reactive.py --waypoints (Phase 2) so the
two demos ("static joint input" and "point-to-point avoidance") exercise the
literal same two poses.
"""
from __future__ import annotations

import argparse
import itertools
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
import yaml

WORK_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = WORK_DIR.parent
sys.path.insert(0, str(WORK_DIR / "src"))

from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg  # noqa: E402
from curobo.scene import Cuboid, Scene  # noqa: E402
from curobo.types import GoalToolPose, Pose  # noqa: E402

from mpg.curobo_bridge.frames import (  # noqa: E402
    NAMED_DIRECTION_ALIASES as DIRECTION_ALIASES,
    approach_direction_to_quat_xyzw,
    curobo_pose_to_xyzw,
    xyzw_to_wxyz,
)
from mpg.urdf_fk import ARM_JOINT_NAMES, UrdfChainFk, classify_approach_direction  # noqa: E402

DEFAULT_ROBOT_CONFIG = REPO_ROOT / "research" / "configs" / "curobo" / "robot129.yml"
DEFAULT_SCENE_CAMERA = REPO_ROOT / "research" / "configs" / "scene_camera.yaml"
DEFAULT_URDF = REPO_ROOT / "ros2_ws" / "src" / "robot129_description" / "urdf" / "robot129.urdf"
DEFAULT_OUT = REPO_ROOT / "research" / "configs" / "demo" / "ab_poses.yaml"

TOOL_FRAME = "pinch_center"

# From robot129.urdf's joint5 <limit> (see docs/dev_guide_paper_core_and_dashboard_plan.md
# §7.1) -- kept as a named constant, not re-read from the URDF here, because the reason it
# matters (margin reporting, not correctness of the IK itself) only needs the number, and
# select_demo_poses.py's own IK solve already re-derives reachability against the real
# robot129.yml joint limits independently of this constant.
JOINT5_LIMIT_RAD = 1.22


def _known_world_cuboids(include_pole: bool) -> list[Cuboid]:
    """Same minimal known-geometry cuboids reactive.py's _known_world_cuboids()
    builds (floor, optional back-camera pole) -- duplicated rather than
    imported because reactive.py's version is a module-private helper tied to
    its own CLI args, not a shared utility.
    """
    cuboids = [Cuboid(name="floor", pose=[0.0, 0.0, -0.025, 1.0, 0.0, 0.0, 0.0], dims=[4.0, 4.0, 0.05])]
    if include_pole:
        scene_camera = yaml.safe_load(DEFAULT_SCENE_CAMERA.read_text())
        pole = scene_camera["pole_camera"]
        ox, oy, oz = pole["offset_from_base_m"]
        half = pole["pole"]["half_extent_m"]
        cuboids.append(Cuboid(name="scene_pole", pose=[ox, oy, oz / 2.0, 1.0, 0.0, 0.0, 0.0], dims=[half * 2.0, half * 2.0, oz]))
    return cuboids


def _build_position_grid(x_values, y_values, z_values) -> np.ndarray:
    return np.array(list(itertools.product(x_values, y_values, z_values)), dtype=float)


def scan_direction(
    ik_solver: "InverseKinematics",
    direction_name: str,
    approach_world: np.ndarray,
    closing_hint_world: np.ndarray,
    positions: np.ndarray,
    batch_size: int,
    device: str,
) -> list[dict]:
    """Solve IK for `direction_name` at every position in `positions`, batched.
    Returns one dict per position (success or not) with joint solution and
    joint5 margin so the caller can rank/filter afterward -- this function
    itself does not choose a winner.
    """
    quat_xyzw = approach_direction_to_quat_xyzw(approach_world, closing_hint_world)
    quat_wxyz = xyzw_to_wxyz(quat_xyzw)

    results: list[dict] = []
    for start in range(0, len(positions), batch_size):
        chunk = positions[start:start + batch_size]
        pos_t = torch.tensor(chunk, dtype=torch.float32, device=device)
        quat_t = torch.tensor(np.tile(quat_wxyz, (len(chunk), 1)), dtype=torch.float32, device=device)
        pose = Pose(position=pos_t, quaternion=quat_t)
        goal = GoalToolPose.from_poses({TOOL_FRAME: pose}, ordered_tool_frames=[TOOL_FRAME])
        ik_result = ik_solver.solve_pose(goal_tool_poses=goal)

        # ik_result.solution is a raw tensor [batch, return_seeds, dof] (6 arm dof --
        # joint7 is locked in robot129.yml, so it never appears here), NOT a JointState
        # (that's ik_result.js_solution, which does include the locked joint at a fixed
        # value) -- confirmed empirically, see docs/dev_guide_paper_core_and_dashboard_plan.md.
        success = ik_result.success.squeeze(-1).detach().cpu().numpy()
        solution = ik_result.solution.squeeze(1).detach().cpu().numpy()  # [batch, dof]
        pos_err = ik_result.position_error.squeeze(-1).detach().cpu().numpy()
        rot_err = ik_result.rotation_error.squeeze(-1).detach().cpu().numpy()

        for i, xyz in enumerate(chunk):
            joint5_val = float(solution[i, ARM_JOINT_NAMES.index("joint5")])
            results.append({
                "direction": direction_name,
                "position_m": xyz.tolist(),
                "success": bool(success[i]),
                "joint_values_rad": solution[i].tolist(),
                "joint5_margin_rad": JOINT5_LIMIT_RAD - abs(joint5_val),
                "position_error_m": float(pos_err[i]),
                "rotation_error_rad": float(rot_err[i]),
                "quaternion_xyzw": quat_xyzw.tolist(),
            })
    return results


def pick_best(candidates: list[dict], min_joint5_margin_rad: float, prefer_y_sign: float | None) -> dict | None:
    """Among IK successes, prefer higher joint5 margin (further from the
    limit that makes this direction risky); optionally prefer a given sign of
    y so A and B land on visibly different sides of the workspace, matching
    the previous --goal-a/--goal-b convention (y>0 / y<0).
    """
    feasible = [c for c in candidates if c["success"] and c["joint5_margin_rad"] >= min_joint5_margin_rad]
    if not feasible:
        return None
    if prefer_y_sign is not None:
        same_side = [c for c in feasible if np.sign(c["position_m"][1]) == np.sign(prefer_y_sign) or c["position_m"][1] == 0.0]
        if same_side:
            feasible = same_side
    feasible.sort(key=lambda c: c["joint5_margin_rad"], reverse=True)
    return feasible[0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_ROBOT_CONFIG)
    parser.add_argument("--direction-a", default="up", choices=sorted(DIRECTION_ALIASES))
    parser.add_argument("--direction-b", default="left", choices=sorted(DIRECTION_ALIASES))
    parser.add_argument("--x", type=float, nargs="+", default=[0.28, 0.34, 0.40])
    parser.add_argument("--y", type=float, nargs="+", default=[-0.20, -0.10, 0.0, 0.10, 0.20])
    parser.add_argument("--z", type=float, nargs="+", default=[0.28, 0.36, 0.44])
    parser.add_argument("--include-pole", action="store_true", help="also treat the back-camera pole as a known obstacle in the IK collision check")
    parser.add_argument("--min-joint5-margin-rad", type=float, default=0.10)
    parser.add_argument("--min-separation-m", type=float, default=0.15, help="reject an A/B pair closer together than this -- a trivial 'move' proves nothing")
    parser.add_argument("--num-seeds", type=int, default=24)
    parser.add_argument("--position-tolerance", type=float, default=0.005)
    parser.add_argument("--orientation-tolerance", type=float, default=0.05)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--report", type=Path, default=None, help="optional path to dump the full per-position scan (for debugging infeasible directions)")
    args = parser.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    cuboids = _known_world_cuboids(args.include_pole)
    scene = Scene(cuboid=cuboids)

    ik_cfg = InverseKinematicsCfg.create(
        robot=str(args.robot_config),
        scene_model=scene,
        num_seeds=args.num_seeds,
        position_tolerance=args.position_tolerance,
        orientation_tolerance=args.orientation_tolerance,
        max_batch_size=args.batch_size,
    )
    ik_solver = InverseKinematics(ik_cfg)

    positions = _build_position_grid(args.x, args.y, args.z)
    print(f"[select_demo_poses] scanning {len(positions)} positions x 2 directions ({args.direction_a}, {args.direction_b}) ...", flush=True)

    t0 = time.monotonic()
    results_a = scan_direction(
        ik_solver, args.direction_a, DIRECTION_ALIASES[args.direction_a], DIRECTION_ALIASES.get(args.direction_b, np.array([0, 0, 1.0])),
        positions, args.batch_size, device,
    )
    results_b = scan_direction(
        ik_solver, args.direction_b, DIRECTION_ALIASES[args.direction_b], DIRECTION_ALIASES.get(args.direction_a, np.array([0, 0, 1.0])),
        positions, args.batch_size, device,
    )
    elapsed = time.monotonic() - t0

    n_ok_a = sum(1 for c in results_a if c["success"])
    n_ok_b = sum(1 for c in results_b if c["success"])
    print(f"[select_demo_poses] IK done in {elapsed:.1f}s: direction_a={args.direction_a} {n_ok_a}/{len(results_a)} reachable, "
          f"direction_b={args.direction_b} {n_ok_b}/{len(results_b)} reachable", flush=True)

    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps({"direction_a": results_a, "direction_b": results_b}, indent=2))
        print(f"[select_demo_poses] full scan written to {args.report}", flush=True)

    best_a = pick_best(results_a, args.min_joint5_margin_rad, prefer_y_sign=+1.0)
    best_b = pick_best(results_b, args.min_joint5_margin_rad, prefer_y_sign=-1.0)

    if best_a is None or best_b is None:
        missing = []
        if best_a is None:
            missing.append(args.direction_a)
        if best_b is None:
            missing.append(args.direction_b)
        print(f"[select_demo_poses] FAIL: no IK solution with joint5 margin >= {args.min_joint5_margin_rad} rad "
              f"for direction(s) {missing} anywhere in the scanned grid.", flush=True)
        print("[select_demo_poses] Options: widen --x/--y/--z, lower --min-joint5-margin-rad, "
              "or pick a different --direction-a/--direction-b (this arm's joint5 range is only +-1.22 rad, "
              "see docs/dev_guide_paper_core_and_dashboard_plan.md §7.1).", flush=True)
        return 1

    if np.linalg.norm(np.array(best_a["position_m"]) - np.array(best_b["position_m"])) < args.min_separation_m:
        print(f"[select_demo_poses] FAIL: best A and B candidates are closer than --min-separation-m={args.min_separation_m}; "
              "widen the position grid so A and B aren't forced into the same neighborhood.", flush=True)
        return 1

    # Independent cross-check: recompute FK from the IK solver's own joint
    # solution using mpg.urdf_fk (a second, unrelated implementation) and
    # confirm it lands on the requested named direction -- catches a bug in
    # approach_direction_to_quat_xyzw or in the IK call itself, not just a
    # low residual error reported by cuRobo.
    urdf_chain = UrdfChainFk(DEFAULT_URDF)
    for label, best in (("A", best_a), ("B", best_b)):
        pos_fk, quat_fk = urdf_chain.pinch_center_pose(best["joint_values_rad"])
        direction_label, angle_deg = classify_approach_direction(quat_fk)
        print(f"[select_demo_poses] point {label}: position_m={np.round(best['position_m'], 4).tolist()} "
              f"joint5_margin_rad={best['joint5_margin_rad']:.3f} "
              f"urdf_fk_approach={direction_label} (off by {angle_deg:.1f} deg) "
              f"pos_error_m={best['position_error_m']:.2e} rot_error_rad={best['rotation_error_rad']:.2e}", flush=True)
        best["urdf_fk_verified_direction"] = direction_label
        best["urdf_fk_verified_angle_deg"] = angle_deg
        best["urdf_fk_position_m"] = pos_fk.tolist()

    midpoint = ((np.array(best_a["position_m"]) + np.array(best_b["position_m"])) / 2.0).tolist()

    out_doc = {
        "schema_version": "demo_ab_poses_v1",
        "generated_by": "research/scripts/select_demo_poses.py",
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "robot_config": str(args.robot_config.relative_to(REPO_ROOT)) if args.robot_config.is_relative_to(REPO_ROOT) else str(args.robot_config),
        "ik_settings": {
            "num_seeds": args.num_seeds, "position_tolerance": args.position_tolerance,
            "orientation_tolerance": args.orientation_tolerance, "include_pole": args.include_pole,
        },
        "points": {
            "A": {
                "name": "A", "direction": args.direction_a,
                "position_m": best_a["position_m"], "quaternion_xyzw": best_a["quaternion_xyzw"],
                "joint_values_rad": best_a["joint_values_rad"], "joint5_margin_rad": best_a["joint5_margin_rad"],
                "position_error_m": best_a["position_error_m"], "rotation_error_rad": best_a["rotation_error_rad"],
                "urdf_fk_verified_direction": best_a["urdf_fk_verified_direction"],
                "urdf_fk_verified_angle_deg": best_a["urdf_fk_verified_angle_deg"],
            },
            "B": {
                "name": "B", "direction": args.direction_b,
                "position_m": best_b["position_m"], "quaternion_xyzw": best_b["quaternion_xyzw"],
                "joint_values_rad": best_b["joint_values_rad"], "joint5_margin_rad": best_b["joint5_margin_rad"],
                "position_error_m": best_b["position_error_m"], "rotation_error_rad": best_b["rotation_error_rad"],
                "urdf_fk_verified_direction": best_b["urdf_fk_verified_direction"],
                "urdf_fk_verified_angle_deg": best_b["urdf_fk_verified_angle_deg"],
            },
        },
        "suggested_obstacle_midpoint_m": midpoint,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(yaml.safe_dump(out_doc, sort_keys=False, default_flow_style=False))
    print(f"[select_demo_poses] PASS: wrote {args.out}", flush=True)
    print(f"[select_demo_poses] suggested obstacle midpoint (for the cone manifest): {np.round(midpoint, 4).tolist()}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
