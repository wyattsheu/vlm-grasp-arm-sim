#!/usr/bin/env python3
"""Generate a grasp_contract.py candidates JSON for a --scene pick_place target's known
geometry (matches sim/scripts/run_robot129_ros_webrtc.py's spawn logic -- see
research/configs/scene_geometry.json for the default cube and
research/configs/grasp_targets.json for every other --target-object shape, both single
sources of truth). Ground-truth surface points, not a captured point cloud -- this is the
S3-to-S2 wiring bridge, not a perception test (S4 is where the points come from a real
depth image).

Usage:
  /mnt/HDD4/wyattsheu/env_robot129_research/bin/python tools/generate_s2_cube_candidates.py \
      [out/grasp_motion/candidates/s2_cube_candidates.json] [--target <id>]

--target (default cube_35, unchanged from before this flag existed): an id from
research/configs/grasp_targets.json, or the special value "cube_35" which uses
scene_geometry.json's cube (byte-identical to this script's original, --target-less
behavior). Only "box" and "cylinder" shapes are supported -- "sphere" (the `ball` target)
is REFUSED, not silently mishandled: generate_grasp_candidates()'s antipodal/edge-band
logic assumes a roughly-flat local surface at each sampled yaw, which a sphere's surface
doesn't have anywhere; grasp_targets.json's own note on `ball` already flags this as a
"needs a different strategy" stretch target, not a guaranteed drop-in.

The output is consumed by tools/run_curobo_grasp_demo.sh / robot129_mtc_pick_place's
candidates_path ROS param (ros2_ws/src/robot129_tasks/launch/mtc_pick_place_sim.launch.py).
"""
from __future__ import annotations

import argparse
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
GRASP_TARGETS_PATH = ROOT / "research" / "configs" / "grasp_targets.json"

# --target-object always spawns at the same xy as the original cube (see CUBE_XY_DEFAULT
# in sim/scripts/run_robot129_ros_webrtc.py -- only the shape/z/mass change per target,
# not the xy position), so CUBE_X/CUBE_Y apply to every target, not just cube_35.
SUPPORTED_SHAPES = ("box", "cylinder")


def cube_surface_points(center_xy: tuple[float, float], rest_z: float, size: float, n_per_edge: int = 8) -> np.ndarray:
    """This script's ORIGINAL point generator, unmodified (only parameterized on
    center_xy/rest_z/size instead of the module-level CUBE_X/CUBE_Y/CUBE_Z/CUBE_SIZE
    globals it used to close over) -- used for the cube_35 default so that path stays
    byte-for-byte identical to before --target existed. See box_surface_points() below
    for why a *different*, more general function handles every other box-shaped target
    rather than trying to generalize this one in place: an earlier attempt at that
    changed the (a, b) loop's iteration/interleaving order, which changed
    generate_grasp_candidates()'s point indices and downstream candidate scores even
    though the point SET was geometrically equivalent -- caught by a regression diff
    against the pre-existing output, 2026-09-24, not by inspection.
    """
    cx, cy = center_xy
    half = size / 2.0
    lin = np.linspace(-half, half, n_per_edge)
    pts = []
    for a in lin:
        for b in lin:
            # four side faces, so edge-contact / antipodal detection sees the box silhouette
            pts.append([cx + half, cy + b, rest_z + a])
            pts.append([cx - half, cy + b, rest_z + a])
            pts.append([cx + b, cy + half, rest_z + a])
            pts.append([cx + b, cy - half, rest_z + a])
            # top/bottom faces so grasp_height_mode="region_midpoint" sees the full z extent
            pts.append([cx + a, cy + b, rest_z + half])
            pts.append([cx + a, cy + b, rest_z - half])
    return np.array(pts)


def box_surface_points(center_xy: tuple[float, float], rest_z: float, size_xyz: tuple[float, float, float], n_per_edge: int = 8) -> np.ndarray:
    """General (non-cubic) box surface points, for grasp_targets.json shapes other than
    cube_35 (tall_block, flat_box) -- geometrically the same six-face sampling as
    cube_surface_points(), generalized to independent x/y/z half-extents. NOT used for
    cube_35 -- see that function's docstring for why keeping them separate matters.
    """
    cx, cy = center_xy
    sx, sy, sz = size_xyz
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    lin_x, lin_y, lin_z = np.linspace(-hx, hx, n_per_edge), np.linspace(-hy, hy, n_per_edge), np.linspace(-hz, hz, n_per_edge)
    pts = []
    for a in lin_z:
        for b in lin_y:
            # side faces along X, so edge-contact / antipodal detection sees the box silhouette
            pts.append([cx + hx, cy + b, rest_z + a])
            pts.append([cx - hx, cy + b, rest_z + a])
        for b in lin_x:
            pts.append([cx + b, cy + hy, rest_z + a])
            pts.append([cx + b, cy - hy, rest_z + a])
    # top/bottom faces so grasp_height_mode="region_midpoint" sees the full z extent
    for a in lin_x:
        for b in lin_y:
            pts.append([cx + a, cy + b, rest_z + hz])
            pts.append([cx + a, cy + b, rest_z - hz])
    return np.array(pts)


def cylinder_surface_points(center_xy: tuple[float, float], rest_z: float, radius_m: float, height_m: float, n_theta: int = 16, n_height: int = 6) -> np.ndarray:
    """Side wall (ring at several heights, so a yaw-sliced silhouette still looks like
    two roughly-parallel local surfaces the way a box side face does) plus top/bottom
    disks (same role as box_surface_points' top/bottom faces: give
    grasp_height_mode="region_midpoint" the full z extent).
    """
    cx, cy = center_xy
    half_h = height_m / 2.0
    thetas = np.linspace(0.0, 2.0 * np.pi, n_theta, endpoint=False)
    heights = np.linspace(-half_h, half_h, n_height)
    pts = []
    for theta in thetas:
        x, y = cx + radius_m * np.cos(theta), cy + radius_m * np.sin(theta)
        for h in heights:
            pts.append([x, y, rest_z + h])
    for r in np.linspace(0.0, radius_m, 4):
        for theta in thetas:
            x, y = cx + r * np.cos(theta), cy + r * np.sin(theta)
            pts.append([x, y, rest_z + half_h])
            pts.append([x, y, rest_z - half_h])
    return np.array(pts)


def surface_points_for_target(target_id: str) -> tuple[np.ndarray, str]:
    """Returns (surface points, object_id for write_candidates_json)."""
    if target_id == "cube_35":
        return cube_surface_points((CUBE_X, CUBE_Y), CUBE_Z, CUBE_SIZE), "target_cube"

    targets = json.loads(GRASP_TARGETS_PATH.read_text())["targets"]
    if target_id not in targets:
        raise SystemExit(f"unknown --target {target_id!r}, choices: {sorted(targets) + ['cube_35']}")
    spec = targets[target_id]
    shape = spec["shape"]
    if shape not in SUPPORTED_SHAPES:
        raise SystemExit(
            f"--target {target_id!r} has shape={shape!r}, which this generator does not support yet "
            f"(only {SUPPORTED_SHAPES}) -- see this script's module docstring for why 'sphere' is refused rather than guessed at."
        )
    if shape == "box":
        size_xyz = tuple(spec["size_m"])
        rest_z = size_xyz[2] / 2.0
        return box_surface_points((CUBE_X, CUBE_Y), rest_z, size_xyz), target_id
    radius_m, height_m = float(spec["radius_m"]), float(spec["height_m"])
    rest_z = height_m / 2.0
    return cylinder_surface_points((CUBE_X, CUBE_Y), rest_z, radius_m, height_m), target_id


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out_path", nargs="?", type=Path, default=ROOT / "out/grasp_motion/candidates/s2_cube_candidates.json")
    parser.add_argument("--target", default="cube_35")
    args = parser.parse_args()

    points, object_id = surface_points_for_target(args.target)

    # Same gripper/generation parameters as research/configs/grasp_generation.yaml --
    # unchanged across targets: the gripper itself doesn't change per object, only what
    # it's grasping. Every grasp_targets.json size was chosen to fit within
    # gripper_max_opening_m with margin (see that file's _provenance note).
    candidates = generate_grasp_candidates(
        points, frame_id="world", table_z_m=0.0,
        gripper_max_opening_m=0.0690, gripper_min_opening_m=0.006,
        pinch_offset_m=0.125, table_clearance_m=0.015,
        num_yaws=8, pregrasp_standoff_m=0.05,
        edge_band_fraction=0.12, min_edge_points_per_side=3,
        antipodal_angle_tolerance_deg=35.0, max_candidates=8,
    )
    accepted = [c for c in candidates if c.accepted]
    print(f"target={args.target} object_id={object_id}: generated {len(candidates)} candidates, {len(accepted)} accepted")
    for c in candidates:
        status = "accepted" if c.accepted else f"rejected {c.rejection_reasons}"
        print(f"  {c.candidate_id}: {status} score={c.score:.4f}")

    args.out_path.parent.mkdir(parents=True, exist_ok=True)
    # "s2_pick_place_cube" (unchanged) for the default target; a distinct scene_id per
    # other target so candidates files for different shapes are easy to tell apart at a
    # glance (e.g. in out/grasp_motion/candidates/), not because anything downstream
    # parses this string.
    scene_id = "s2_pick_place_cube" if args.target == "cube_35" else f"s2_pick_place_{args.target}"
    write_candidates_json(args.out_path, candidates, scene_id=scene_id, object_id=object_id)
    print(f"wrote {args.out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
