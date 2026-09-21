#!/usr/bin/env python3
"""S5 pilot, bottle variant (requested 2026-09-21 as a harder task-region test):
research/scripts/s5_pilot_hammer.py's handle/head split is a LATERAL separation (two
parts side by side along local X) -- this pilot tests the orthogonal case, a VERTICAL
separation: a bottle standing upright, body below, cap on top. The existing top-down
grasp candidate generator (research/src/mpg/grasp_candidates.py) always approaches from
above (approach_axis_world is fixed at (0,0,-1)), so without a task-region filter its
natural pinch point is wherever the cross-section is widest/most graspable near the top
of the point cloud -- for a bottle with a narrower cap sitting above a wider body, that
can land on the cap. task_region_mask=body_mask forces candidates to only be scored
against the body's cross-section, which is the "grasp the body, not the cap" behavior
requested (real bottles: gripping the cap risks it popping off or crushing a screw-on
lid, not a stable grasp).

Same three baselines as the hammer pilot:
  B0 -- naive top-down: single fixed yaw, whole object graspable (no task filtering)
  B1 -- multi-yaw: sweep yaws, whole object still graspable (no task filtering)
  B2 -- multi-yaw + task-region filter: only the body is graspable

Deliberately GT-mask-driven, not VLM-driven, same reasoning as the hammer pilot: isolates
"does restricting the candidate search to the body change the outcome" from "can a VLM
find the body region", which S4's real VLM run already tested (for the cube; a bottle-
shaped object has not been run through a real VLM affordance call in this handoff).

Result (2026-09-21, honest -- not the "B2 rescues a wrong answer" story the hammer pilot
told): all three baselines, including the unfiltered B0/B1, land on the body 4/4. Why:
generate_grasp_candidates()'s default grasp_height_mode="region_midpoint" picks the pinch
height at the midpoint of whatever point cloud it's given. For a realistically-proportioned
bottle (body much taller than cap), that midpoint falls inside the body's own z-range even
with the cap included in an unfiltered search -- so for THIS class of task separation
(vertically stacked, tall part vs short part), the existing height-selection logic already
does the right thing without needing task_region_mask at all. That's different from the
hammer's handle/head split, which is a LATERAL separation the same object height covers
entirely, so region_midpoint can't disambiguate it and the filter is what actually does the
work. Task-conditioned grasping matters when the task region isn't already implied by
object height -- worth knowing before assuming every task-region case needs the filter.
A shorter/wider cap (closer in height to the body) would likely reproduce a real failure
case here; not attempted, to avoid tuning the geometry until B0 "loses" instead of reporting
what a plausible bottle shape actually does.

Geometry: explicit surface-point sampling (approximating a cylinder with an octagonal
box), same technique as tools/generate_s2_cube_candidates.py and s5_pilot_hammer.py --
no new USD/mesh asset needed for this offline pilot. NOT yet spawned in live Isaac (S5's
hammer pilot went through that extension separately, in a later step; this bottle pilot
is at the same offline-only stage the hammer pilot started at).
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

# Bottle geometry in its own local frame, before yaw rotation about world Z. Body is an
# octagonal-prism approximation of a cylinder (radius BODY_R, height BODY_H), cap is a
# narrower, shorter prism sitting directly on top of the body -- so a real bottle's
# actual failure mode (grippable cap, harder-to-grip cylindrical body) is inverted here
# on purpose: the cap being NARROWER makes it the geometrically "easier" pinch target
# for an unfiltered top-down search (narrower cross-section fits inside the gripper's
# max opening more comfortably), which is exactly what makes this a real test of the
# task-region filter rather than a foregone conclusion either way.
BODY_R_M = 0.028
BODY_H_M = 0.095
CAP_R_M = 0.014
CAP_H_M = 0.018
TABLE_Z = 0.0
BODY_CENTER_Z = BODY_H_M / 2
CAP_CENTER_Z = BODY_H_M + CAP_H_M / 2
N_SIDES = 8


def _prism_surface_points(cx: float, cy: float, cz: float, radius: float, height: float, n_h: int = 5) -> np.ndarray:
    """Octagonal-prism approximation of a cylinder: points on the N_SIDES side faces
    plus the top/bottom rims, all offset to (cx, cy, cz)."""
    angles = np.linspace(0, 2 * np.pi, N_SIDES, endpoint=False)
    heights = np.linspace(-height / 2, height / 2, n_h)
    pts = []
    for a in angles:
        x, y = radius * np.cos(a), radius * np.sin(a)
        for h in heights:
            pts.append([x, y, h])
    pts = np.array(pts) + np.array([cx, cy, cz])
    return pts


@dataclass
class BottleScene:
    scene_id: str
    yaw_deg: float


def build_bottle_points(yaw_deg: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Returns (points_world, body_mask, cap_mask), all (N,) aligned."""
    body_pts = _prism_surface_points(0, 0, BODY_CENTER_Z, BODY_R_M, BODY_H_M, n_h=6)
    cap_pts = _prism_surface_points(0, 0, CAP_CENTER_Z, CAP_R_M, CAP_H_M, n_h=3)

    points_local = np.concatenate([body_pts, cap_pts], axis=0)
    body_mask = np.concatenate([np.ones(len(body_pts), bool), np.zeros(len(cap_pts), bool)])
    cap_mask = ~body_mask

    # Same xy as the S2 cube / hammer pilots -- stays inside the region already
    # validated reachable against live Isaac.
    yaw = np.radians(yaw_deg)
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    points_world = points_local @ R.T + np.array([0.32, 0.0, 0.0])
    return points_world, body_mask, cap_mask


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
        BottleScene("bottle_yaw0", 0.0),
        BottleScene("bottle_yaw30", 30.0),
        BottleScene("bottle_yaw60", 60.0),
        BottleScene("bottle_yaw90", 90.0),
    ]

    rows = []
    for scene in scenes:
        points, body_mask, cap_mask = build_bottle_points(scene.yaw_deg)
        body_lo, body_hi = region_bounds(points, body_mask)
        cap_lo, cap_hi = region_bounds(points, cap_mask)

        b0 = run_baseline("B0_top_down_single_yaw", points, None, num_yaws=1)
        b1 = run_baseline("B1_multi_yaw_whole_object", points, None, num_yaws=8)
        b2 = run_baseline("B2_multi_yaw_body_only", points, body_mask, num_yaws=8)

        print(f"\n=== {scene.scene_id} (task=body, yaw={scene.yaw_deg}deg) ===")
        for result in (b0, b1, b2):
            best = result["best"]
            if best is None:
                verdict = "NO_ACCEPTED_CANDIDATE"
            else:
                tcp = np.array(best.tcp_position_m)
                in_body = in_bounds(tcp, body_lo, body_hi)
                in_cap = in_bounds(tcp, cap_lo, cap_hi) and not in_body
                verdict = "IN_BODY (correct)" if in_body else ("IN_CAP (wrong)" if in_cap else "AMBIGUOUS/BOUNDARY")
            print(f"  {result['baseline']:28s} accepted={result['n_accepted']}/{result['n_candidates']:2d} "
                  f"best_tcp={None if best is None else tuple(round(v,4) for v in best.tcp_position_m)} "
                  f"-> {verdict}")
            rows.append({"scene": scene.scene_id, **result,
                         "best_tcp": None if best is None else best.tcp_position_m, "verdict": verdict})

    n_b2_correct = sum(1 for r in rows if r["baseline"] == "B2_multi_yaw_body_only" and r["verdict"].startswith("IN_BODY"))
    n_b1_correct = sum(1 for r in rows if r["baseline"] == "B1_multi_yaw_whole_object" and r["verdict"].startswith("IN_BODY"))
    n_b0_correct = sum(1 for r in rows if r["baseline"] == "B0_top_down_single_yaw" and r["verdict"].startswith("IN_BODY"))
    print(f"\nPilot summary (4 scenes, NOT a formal benchmark): "
          f"B0 grasped the body {n_b0_correct}/4, B1 {n_b1_correct}/4, B2 {n_b2_correct}/4.")

    out_path = ROOT / "out" / "grasp_motion" / "s5_pilot" / "bottle_pilot_results.json"
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
