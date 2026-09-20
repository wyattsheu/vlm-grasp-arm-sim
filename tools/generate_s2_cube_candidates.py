#!/usr/bin/env python3
"""Generate a grasp_contract.py candidates JSON for the S2 pick_place scene's known
target_cube geometry (matches sim/scripts/run_robot129_ros_webrtc.py and
robot129_mtc_pick_place.cpp's scene layout -- see research/configs/scene_geometry.json,
the single source of truth for these numbers as of 2026-09-20). Ground-truth box
surface points, not a captured point cloud -- this is the S3-to-S2 wiring bridge, not a
perception test (S4 is where the points come from a real depth image).

Usage:
  /mnt/HDD4/wyattsheu/env_robot129_research/bin/python tools/generate_s2_cube_candidates.py \
      [out/grasp_motion/candidates/s2_cube_candidates.json]

The output is consumed by robot129_mtc_pick_place's candidates_path ROS param
(ros2_ws/src/robot129_tasks/launch/mtc_pick_place_sim.launch.py).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "research" / "src"))

import numpy as np

from mpg.grasp_candidates import generate_grasp_candidates
from mpg.grasp_contract import write_candidates_json

_geometry = json.loads((ROOT / "research" / "configs" / "scene_geometry.json").read_text())["cube"]
CUBE_X, CUBE_Y = _geometry["xy_default_m"]
CUBE_Z, CUBE_SIZE = _geometry["rest_z_m"], _geometry["size_m"]


def cube_surface_points(n_per_edge: int = 8) -> np.ndarray:
    half = CUBE_SIZE / 2.0
    lin = np.linspace(-half, half, n_per_edge)
    pts = []
    for a in lin:
        for b in lin:
            # four side faces, so edge-contact / antipodal detection sees the box silhouette
            pts.append([CUBE_X + half, CUBE_Y + b, CUBE_Z + a])
            pts.append([CUBE_X - half, CUBE_Y + b, CUBE_Z + a])
            pts.append([CUBE_X + b, CUBE_Y + half, CUBE_Z + a])
            pts.append([CUBE_X + b, CUBE_Y - half, CUBE_Z + a])
            # top/bottom faces so grasp_height_mode="region_midpoint" sees the full z extent
            pts.append([CUBE_X + a, CUBE_Y + b, CUBE_Z + half])
            pts.append([CUBE_X + a, CUBE_Y + b, CUBE_Z - half])
    return np.array(pts)


def main() -> int:
    out_path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "out/grasp_motion/candidates/s2_cube_candidates.json"
    points = cube_surface_points()

    # Same gripper/generation parameters as research/configs/grasp_generation.yaml.
    candidates = generate_grasp_candidates(
        points, frame_id="world", table_z_m=0.0,
        gripper_max_opening_m=0.0690, gripper_min_opening_m=0.006,
        pinch_offset_m=0.125, table_clearance_m=0.015,
        num_yaws=8, pregrasp_standoff_m=0.05,
        edge_band_fraction=0.12, min_edge_points_per_side=3,
        antipodal_angle_tolerance_deg=35.0, max_candidates=8,
    )
    accepted = [c for c in candidates if c.accepted]
    print(f"generated {len(candidates)} candidates, {len(accepted)} accepted")
    for c in candidates:
        status = "accepted" if c.accepted else f"rejected {c.rejection_reasons}"
        print(f"  {c.candidate_id}: {status} score={c.score:.4f}")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    write_candidates_json(out_path, candidates, scene_id="s2_pick_place_cube", object_id="target_cube")
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
