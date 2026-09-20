#!/usr/bin/env python3
"""S5 pilot (grasp_motion_research_plan_20260918.md Sec 9 / handoff plan's S5 step):
a small, 4-scene, GT-geometry pilot comparing three grasp-candidate baselines on a
synthetic hammer (handle + head) object with a task-conditioned grasp region:

  B0 -- naive top-down: single fixed yaw, whole object graspable (no task filtering)
  B1 -- multi-yaw: sweep yaws, whole object still graspable (no task filtering)
  B2 -- multi-yaw + task-region filter: sweep yaws, only the task-relevant sub-region
        (handle for a "swing" task, head for a "inspect the head" task) is graspable

This is deliberately GT-mask-driven, not VLM-driven -- it isolates "does restricting
the candidate search to the right sub-region change the outcome" from "can a VLM find
that sub-region", which is a separate, already-tested question (see S4's real VLM run).
Per the handoff plan's S5 scope: 4 scenes, pilot only, NOT the formal 12-scene/108-trial
evaluation -- do not treat these numbers as a benchmark result.

Geometry (world frame, meters): a hammer lying flat on a z=0 table, handle a thin rod
along its own local x-axis, head a wider/taller block at one end. Built the same way as
tools/generate_s2_cube_candidates.py's cube -- explicit surface-point sampling, not a
loaded mesh, so no new USD/mesh asset is needed for this pilot.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "research" / "src"))

import json

import numpy as np

from mpg.grasp_candidates import generate_grasp_candidates

# Hammer geometry in its own local frame, before yaw rotation about world Z:
# handle: thin rod, local x in [0, HANDLE_LEN], centered on local y=0
# head: wider/taller block at the far end, local x in [HANDLE_LEN-HEAD_OVERLAP, HANDLE_LEN+HEAD_LEN]
#
# Loaded from research/configs/scene_geometry.json (single source of truth introduced
# 2026-09-20) rather than redeclared here: this script's own hardcoded copy had drifted
# from what sim/scripts/run_robot129_ros_webrtc.py actually spawns in Isaac (handle
# height 0.040 here vs 0.030 there, head 0.050 vs 0.045) despite this file's docstring
# claiming they matched -- every candidate this pilot generated was for a taller object
# than the one a live-Isaac run would ever see. Isaac's spawned geometry is
# authoritative; this script must follow it, not the other way around.
_geometry = json.loads((ROOT / "research" / "configs" / "scene_geometry.json").read_text())
_hammer = _geometry["hammer"]
HANDLE_LEN, HANDLE_W, HANDLE_H = (
    _hammer["handle"]["len_m"], _hammer["handle"]["width_m"], _hammer["handle"]["height_m"]
)
HEAD_LEN, HEAD_W, HEAD_H = _hammer["head"]["len_m"], _hammer["head"]["width_m"], _hammer["head"]["height_m"]
HEAD_OVERLAP = _hammer["head_overlap_m"]
TABLE_Z = 0.0
# Both parts rest on the table (z=0) but have different heights, so each gets its own
# center z -- using one shared CENTER_Z for both (an earlier version of this script did)
# made the taller head's box dip to negative z, i.e. clip through the table, which
# silently produced a bad z range (bottom below 0) and cascaded into every candidate
# failing TABLE_CLEARANCE. Caught by checking points[:,2].min() during debugging.
HANDLE_CENTER_Z = HANDLE_H / 2
HEAD_CENTER_Z = HEAD_H / 2


def _box_surface_points(cx, cy, cz, sx, sy, sz, n=5):
    half = np.array([sx, sy, sz]) / 2
    lin = [np.linspace(-half[i], half[i], n) for i in range(3)]
    pts = []
    for a in lin[1]:
        for b in lin[2]:
            pts.append([half[0], a, b]); pts.append([-half[0], a, b])
    for a in lin[0]:
        for b in lin[2]:
            pts.append([a, half[1], b]); pts.append([a, -half[1], b])
    for a in lin[0]:
        for b in lin[1]:
            pts.append([a, b, half[2]]); pts.append([a, b, -half[2]])
    pts = np.array(pts) + np.array([cx, cy, cz])
    return pts


@dataclass
class HammerScene:
    scene_id: str
    yaw_deg: float
    task: str  # "handle" or "head"


def build_hammer_points(yaw_deg: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Returns (points_world, handle_mask, head_mask), all (N,) aligned."""
    handle_cx = HANDLE_LEN / 2
    handle_pts = _box_surface_points(handle_cx, 0, HANDLE_CENTER_Z, HANDLE_LEN, HANDLE_W, HANDLE_H, n=6)
    head_cx = HANDLE_LEN - HEAD_OVERLAP + HEAD_LEN / 2
    head_pts = _box_surface_points(head_cx, 0, HEAD_CENTER_Z, HEAD_LEN, HEAD_W, HEAD_H, n=6)

    points_local = np.concatenate([handle_pts, head_pts], axis=0)
    handle_mask = np.concatenate([np.ones(len(handle_pts), bool), np.zeros(len(head_pts), bool)])
    head_mask = ~handle_mask

    # object center is placed at world (0.32, 0, CENTER_Z), matching the S2 cube's xy so
    # this pilot sits in the same reachable region already validated against live Isaac.
    yaw = np.radians(yaw_deg)
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    points_world = points_local @ R.T + np.array([0.32, 0.0, 0.0])
    return points_world, handle_mask, head_mask


GRIPPER_KW = dict(
    gripper_max_opening_m=0.0690, gripper_min_opening_m=0.006,
    pinch_offset_m=0.125, table_clearance_m=0.015,
    pregrasp_standoff_m=0.05, edge_band_fraction=0.12,
    min_edge_points_per_side=3, antipodal_angle_tolerance_deg=35.0,
)


def region_bounds(points: np.ndarray, mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    region = points[mask]
    return region.min(axis=0), region.max(axis=0)


def in_bounds(point: np.ndarray, lo: np.ndarray, hi: np.ndarray, margin: float = 0.005) -> bool:
    return bool(np.all(point >= lo - margin) and np.all(point <= hi + margin))


def run_baseline(name: str, points: np.ndarray, task_mask: np.ndarray | None, num_yaws: int) -> dict:
    candidates = generate_grasp_candidates(
        points, frame_id="world", table_z_m=TABLE_Z, num_yaws=num_yaws,
        max_candidates=8, task_region_mask=task_mask, **GRIPPER_KW,
    )
    accepted = [c for c in candidates if c.accepted]
    return {"baseline": name, "n_candidates": len(candidates), "n_accepted": len(accepted),
            "best": accepted[0] if accepted else None}


def main() -> int:
    scenes = [
        HammerScene("hammer_handle_yaw0", 0.0, "handle"),
        HammerScene("hammer_handle_yaw90", 90.0, "handle"),
        HammerScene("hammer_head_yaw0", 0.0, "head"),
        HammerScene("hammer_head_yaw45", 45.0, "head"),
    ]

    rows = []
    for scene in scenes:
        points, handle_mask, head_mask = build_hammer_points(scene.yaw_deg)
        task_mask = handle_mask if scene.task == "handle" else head_mask
        other_mask = head_mask if scene.task == "handle" else handle_mask
        task_lo, task_hi = region_bounds(points, task_mask)
        other_lo, other_hi = region_bounds(points, other_mask)

        b0 = run_baseline("B0_top_down_single_yaw", points, None, num_yaws=1)
        b1 = run_baseline("B1_multi_yaw_whole_object", points, None, num_yaws=8)
        b2 = run_baseline("B2_multi_yaw_task_region", points, task_mask, num_yaws=8)

        print(f"\n=== {scene.scene_id} (task={scene.task}, yaw={scene.yaw_deg}deg) ===")
        for result in (b0, b1, b2):
            best = result["best"]
            if best is None:
                verdict = "NO_ACCEPTED_CANDIDATE"
                in_task, in_other = None, None
            else:
                tcp = np.array(best.tcp_position_m)
                in_task = in_bounds(tcp, task_lo, task_hi)
                in_other = in_bounds(tcp, other_lo, other_hi) and not in_task
                verdict = "IN_TASK_REGION" if in_task else ("IN_WRONG_REGION" if in_other else "AMBIGUOUS/BOUNDARY")
            print(f"  {result['baseline']:28s} accepted={result['n_accepted']}/{result['n_candidates']:2d} "
                  f"best_tcp={None if best is None else tuple(round(v,4) for v in best.tcp_position_m)} "
                  f"-> {verdict}")
            rows.append({"scene": scene.scene_id, "task": scene.task, **result,
                         "best_tcp": None if best is None else best.tcp_position_m, "verdict": verdict})

    n_b2_correct = sum(1 for r in rows if r["baseline"] == "B2_multi_yaw_task_region" and r["verdict"] == "IN_TASK_REGION")
    n_b1_correct = sum(1 for r in rows if r["baseline"] == "B1_multi_yaw_whole_object" and r["verdict"] == "IN_TASK_REGION")
    n_b0_correct = sum(1 for r in rows if r["baseline"] == "B0_top_down_single_yaw" and r["verdict"] == "IN_TASK_REGION")
    print(f"\nPilot summary (4 scenes, NOT a formal benchmark): "
          f"B0 landed in the correct task region {n_b0_correct}/4, "
          f"B1 {n_b1_correct}/4, B2 {n_b2_correct}/4.")

    import json
    out_path = ROOT / "out" / "grasp_motion" / "s5_pilot" / "hammer_pilot_results.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    serializable = [{k: v for k, v in r.items() if k != "best"} for r in rows]
    out_path.write_text(json.dumps({
        "schema_version": "s5_pilot_v0", "n_scenes": len(scenes),
        "summary": {"B0_correct": n_b0_correct, "B1_correct": n_b1_correct, "B2_correct": n_b2_correct},
        "rows": serializable,
    }, indent=2))
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
