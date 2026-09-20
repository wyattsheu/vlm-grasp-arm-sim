#!/usr/bin/env python3
"""S0 offline reachability scan for top-down parallel-gripper grasps.

Pure numpy, no ROS/Isaac dependency. Computes forward kinematics for the
Robot 129 (Piper) arm from ros2_ws/src/robot129_description/urdf/robot129.urdf
joint origins (hardcoded below, verified against that file) and grid-searches
joints 2-5 (joint1 only rotates the whole solution about world z, so it does
not change reachability radius/height/approach-alignment) to find which
(radius from base, pinch height) pairs admit a top-down approach (gripper
approach axis ~= world -z) within the URDF joint limits.

This does NOT call Isaac or ROS; it is a design-time check to choose cube and
place positions for the S1-S2 pick-and-place scene. It is NOT a substitute
for verifying reachability with the actual MoveIt/KDL IK solver against the
live robot_description once the pipeline exists (S1/S2 acceptance).

Usage:
  PYTHONPATH= python3 tools/analyze_top_down_reach.py
Output:
  out/grasp_motion/s0/reach_scan.json
  Prints a short human-readable summary to stdout.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "out" / "grasp_motion" / "s0"
OUT.mkdir(parents=True, exist_ok=True)


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    """URDF rpy convention: R = Rz(y) * Ry(p) * Rx(r)."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def homog(rot: np.ndarray, trans: np.ndarray) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = trans
    return t


def rot_z(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# Fixed joint origins from robot129.urdf (verified file:line during S0 survey).
# Each entry: (xyz origin, rpy origin) of the joint frame relative to parent link.
J1_ORIGIN = (np.array([0, 0, 0.123]), (0, 0, 0))                       # :54
J2_ORIGIN = (np.array([0, 0, 0]), (1.5708, -0.1359, -3.1416))          # :83
J3_ORIGIN = (np.array([0.28503, 0, 0]), (0, 0, -1.7939))               # :112
J4_ORIGIN = (np.array([-0.021984, -0.25075, 0]), (1.5708, 0, 0))       # :141
J5_ORIGIN = (np.array([0, 0, 0]), (-1.5708, 0, 0))                     # :170
J6_ORIGIN = (np.array([8.8259e-05, -0.091, 0]), (1.5708, 0, 0))        # :199
# gripper_base is joint6_to_gripper_base fixed, identity (:227-231)
PINCH_OFFSET_Z = 0.125   # proposed pinch_center offset from gripper_base (see plan S0 finding #1)
TCP_OFFSET_Z = 0.180     # existing (misleading) URDF tcp offset (:307-311), kept for reference only

LIMITS = {
    "j2": (0.0, 3.14),        # :87
    "j3": (-2.967, 0.0),      # :116
    "j4": (-1.745, 1.745),    # :145
    "j5": (-1.22, 1.22),      # :174
}


def fk_pinch(j2, j3, j4, j5):
    """Return (world_xyz of pinch_center, world approach unit vector (gripper local +z))
    for joint1 = 0. Vectorized over numpy arrays of equal shape."""
    n = j2.shape
    T = np.tile(np.eye(4), n + (1, 1))

    def apply(T, origin, angle):
        xyz, rpy = origin
        r_fixed = rpy_matrix(*rpy)
        c, s = np.cos(angle), np.sin(angle)
        # batched Rz(angle)
        Rz = np.zeros(angle.shape + (3, 3))
        Rz[..., 0, 0] = c; Rz[..., 0, 1] = -s
        Rz[..., 1, 0] = s; Rz[..., 1, 1] = c
        Rz[..., 2, 2] = 1
        Jt = np.zeros(angle.shape + (4, 4))
        Jt[..., :3, :3] = np.einsum("ij,...jk->...ik", r_fixed, Rz)
        Jt[..., :3, 3] = xyz
        Jt[..., 3, 3] = 1
        return np.einsum("...ij,...jk->...ik", T, Jt)

    zero = np.zeros(n)
    T = apply(T, J1_ORIGIN, zero)       # joint1 = 0
    T = apply(T, J2_ORIGIN, j2)
    T = apply(T, J3_ORIGIN, j3)
    T = apply(T, J4_ORIGIN, j4)
    T = apply(T, J5_ORIGIN, j5)
    T = apply(T, J6_ORIGIN, zero)       # joint6 does not change position/approach axis
    # gripper_base fixed identity, then pinch offset along local z
    pinch_local = np.array([0, 0, PINCH_OFFSET_Z, 1.0])
    pinch_world = np.einsum("...ij,j->...i", T, pinch_local)[..., :3]
    approach_world = T[..., :3, 2]  # gripper local +z axis expressed in world
    return pinch_world, approach_world


def main():
    # Outer joints (2,3) looped in pure Python to bound peak memory; inner
    # joints (4,5) vectorized per outer step. n=41 per joint -> 41^4 ~= 2.8M
    # poses total, ~140k per outer step, comfortably small per batch.
    n = 41
    j2_vals = np.linspace(*LIMITS["j2"], n)
    j3_vals = np.linspace(*LIMITS["j3"], n)
    j4_vals = np.linspace(*LIMITS["j4"], n)
    j5_vals = np.linspace(*LIMITS["j5"], n)
    G4, G5 = np.meshgrid(j4_vals, j5_vals, indexing="ij")

    r_chunks, z_chunks = [], []
    for j2v in j2_vals:
        for j3v in j3_vals:
            G2 = np.full_like(G4, j2v)
            G3 = np.full_like(G4, j3v)
            pinch, approach = fk_pinch(G2, G3, G4, G5)
            x, y, z = pinch[..., 0], pinch[..., 1], pinch[..., 2]
            r = np.sqrt(x**2 + y**2)
            az = approach[..., 2]
            mask = az <= -0.98  # within ~11.5 deg of straight down
            if mask.any():
                r_chunks.append(r[mask])
                z_chunks.append(z[mask])

    if not r_chunks:
        raise SystemExit("No top-down solutions found in grid; loosen tolerance or check FK.")

    r_td = np.concatenate(r_chunks)
    z_td = np.concatenate(z_chunks)

    # Bin by height to report reachable radius range at representative heights.
    heights_of_interest = [0.0175, 0.03, 0.05, 0.07, 0.09, 0.12, 0.15]
    height_report = []
    for h in heights_of_interest:
        band = np.abs(z_td - h) < 0.01
        if band.any():
            rb = r_td[band]
            height_report.append({
                "height_m": h, "reachable": True,
                "r_min_m": float(rb.min()), "r_max_m": float(rb.max()),
                "n_solutions": int(band.sum()),
            })
        else:
            height_report.append({"height_m": h, "reachable": False})

    result = {
        "method": "grid_fk_scan_j2_j3_j4_j5_j1_marginalized",
        "grid_resolution_per_joint": n,
        "top_down_tolerance_deg": round(math.degrees(math.acos(0.98)), 2),
        "pinch_offset_from_gripper_base_z_m": PINCH_OFFSET_Z,
        "note": "joint1 only rotates the whole solution about world z; radius r and "
                "approach z-component are invariant under joint1, so it is fixed at 0 "
                "in the scan and any world azimuth is reachable via joint1 for a given r.",
        "overall_top_down_r_range_m": [float(r_td.min()), float(r_td.max())],
        "overall_top_down_z_range_m": [float(z_td.min()), float(z_td.max())],
        "by_height": height_report,
    }
    (OUT / "reach_scan.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
