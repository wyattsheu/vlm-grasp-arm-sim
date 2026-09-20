"""S3 geometric grasp candidate generator (grasp_motion_research_plan_20260918.md §5.1).

Pure numpy, no ROS/Isaac dependency -- generates parallel-gripper top-down grasp
candidates from a target object's point cloud (in a consistent world/base frame,
meters), independent of any perception pipeline. Ground-truth object geometry or a
segmented point cloud can both be fed in; this module only cares about points and an
optional task-region mask.

Scope, precisely: top-down approach only. This matches the S0 reachability finding
(docs/progress/grasp_motion_s0_inventory.md, tools/analyze_top_down_reach.py) that
Robot 129's joint5 limit makes top-down the only reliably reachable approach family
for this arm -- it is not a general-purpose antipodal grasp sampler. Yaw is sampled
around the vertical approach axis over a single flat support surface. This is the
"grasp candidates G" half of ZeroDex Sec.3.4's affordance-guided pipeline; it does not
itself do affordance-region extraction (the multi-view VLM voting of eq. 9-11) --
callers pass in whatever points they consider the graspable/task region
(task_region_mask), which could come from a frozen oracle mask (S4) or eventually
real affordance voting.

Poses are expressed at the "pinch point" (the world position midway between the
fingertips along the approach axis), matching the convention already established for
Robot 129's MTC pick-and-place task (robot129_tasks/src/robot129_mtc_pick_place.cpp
uses this exact offset as its MoveIt ComputeIK ik_frame, cross-validated there against
out/lesson_06/grasp_pose_search.json's empirical contact range). Positions and
quaternions use meters and xyzw, matching scene_bundle.transform_matrix elsewhere in
this package.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# ---- SE(3) / quaternion helpers ---------------------------------------------------


def quat_from_axis_angle(axis_xyz: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = np.asarray(axis_xyz, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        raise ValueError("zero-length rotation axis")
    axis = axis / norm
    s = math.sin(angle_rad / 2.0)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle_rad / 2.0)])


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product q1 * q2 (both xyzw). Applying the result to a vector is
    equivalent to rotating by q2 first, then q1 (matches R1 @ R2 @ v)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([-x, -y, -z, w])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("zero-norm quaternion")
    return q / n


def quat_rotate_vector(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate 3-vector v by unit quaternion q (xyzw)."""
    qv = np.array([v[0], v[1], v[2], 0.0])
    return quat_multiply(quat_multiply(q, qv), quat_conjugate(q))[:3]


def se3_compose(t1, q1, t2, q2):
    """Compose T1 * T2 (apply T2 first, then T1). Returns (translation, quat_xyzw)."""
    q1 = quat_normalize(np.asarray(q1, dtype=float))
    q2 = quat_normalize(np.asarray(q2, dtype=float))
    t1 = np.asarray(t1, dtype=float)
    t2 = np.asarray(t2, dtype=float)
    t = t1 + quat_rotate_vector(q1, t2)
    q = quat_normalize(quat_multiply(q1, q2))
    return t, q


def se3_inverse(t, q):
    q = quat_normalize(np.asarray(q, dtype=float))
    t = np.asarray(t, dtype=float)
    q_inv = quat_conjugate(q)
    t_inv = -quat_rotate_vector(q_inv, t)
    return t_inv, q_inv


# ---- top-down grasp orientation ----------------------------------------------------
# "Flip" quaternion: rotates the gripper's local +z (its approach axis, per the MTC
# pinch-frame convention) to point along world -z, local +x preserved -- a 180-degree
# rotation about the local/world x axis. This matches the orientation family actually
# found for Robot 129's own verified top-down grasp pose (see
# docs/progress/grasp_motion_progress_report.md S2: quat_xyzw ~= (0, 1, 0, 0)).
_TOP_DOWN_FLIP_QUAT = quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), math.pi)


def top_down_grasp_quaternion(yaw_rad: float) -> np.ndarray:
    """Quaternion for a top-down approach with closing-axis yaw about world z."""
    yaw_quat = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), yaw_rad)
    return quat_normalize(quat_multiply(yaw_quat, _TOP_DOWN_FLIP_QUAT))


REJECTION_REASONS = frozenset({
    "WIDTH_EXCEEDED",
    "WIDTH_TOO_NARROW",
    "TABLE_CLEARANCE",
    "UNKNOWN_GEOMETRY",
    "NON_ANTIPODAL_EDGE_CONTACT",
    "EMPTY_TASK_REGION",
})


@dataclass(frozen=True)
class GraspCandidate:
    candidate_id: str
    frame_id: str
    source: str                        # "top_down_yaw_grid"
    yaw_rad: float
    tcp_position_m: tuple              # pinch-point position, world/base frame
    tcp_quaternion_xyzw: tuple
    pregrasp_position_m: tuple         # retreated along approach axis
    pregrasp_quaternion_xyzw: tuple
    gripper_base_position_m: tuple     # tcp_position offset back by pinch_offset_m,
                                        # for direct comparison with the MTC/URDF frame
    approach_axis_world: tuple         # unit vector, world frame (always (0,0,-1) here)
    opening_width_m: float
    contact_point_indices: tuple       # indices into the input points array, both edges
    score: float
    rejection_reasons: tuple = ()

    @property
    def accepted(self) -> bool:
        return len(self.rejection_reasons) == 0


def _closing_axis(yaw_rad: float) -> np.ndarray:
    return np.array([math.cos(yaw_rad), math.sin(yaw_rad)])


def _empty_candidate(candidate_id: str, frame_id: str, yaw: float, reason: str) -> GraspCandidate:
    return GraspCandidate(
        candidate_id=candidate_id, frame_id=frame_id, source="top_down_yaw_grid",
        yaw_rad=float(yaw), tcp_position_m=(0.0, 0.0, 0.0),
        tcp_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        pregrasp_position_m=(0.0, 0.0, 0.0), pregrasp_quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        gripper_base_position_m=(0.0, 0.0, 0.0),
        approach_axis_world=(0.0, 0.0, -1.0), opening_width_m=0.0,
        contact_point_indices=(), score=float("inf"), rejection_reasons=(reason,),
    )


def generate_grasp_candidates(
    points_xyz: np.ndarray,
    *,
    frame_id: str,
    table_z_m: float,
    gripper_max_opening_m: float,
    gripper_min_opening_m: float,
    pinch_offset_m: float,
    table_clearance_m: float,
    num_yaws: int,
    pregrasp_standoff_m: float,
    task_region_mask: np.ndarray | None = None,
    yaw_start_rad: float = 0.0,
    edge_band_fraction: float = 0.12,
    min_edge_points_per_side: int = 3,
    antipodal_angle_tolerance_deg: float = 35.0,
    max_candidates: int | None = None,
    grasp_height_mode: str = "region_midpoint",
) -> list[GraspCandidate]:
    """Generate top-down parallel-gripper grasp candidates by sampling yaw around the
    world vertical axis. Every candidate is returned, accepted or not -- callers that
    only want feasible ones should filter on `.accepted`; keeping rejections (each
    with a reason) is what lets downstream code and evaluation tell "no candidate fits
    this object at this orientation" apart from "the generator produced nothing".

    points_xyz: (N, 3) object points in a consistent world/base frame, meters.
    task_region_mask: optional (N,) boolean array selecting the task-allowed contact
      region (e.g. the handle of a tool). If None, every point in points_xyz is
      treated as an allowed contact point -- appropriate for a plain rigid object like
      the S2 cube, not for task-conditioned grasping (research plan Sec.9's B2).
    grasp_height_mode: "region_midpoint" (default) places the pinch point halfway
      between the region's lowest and highest point -- correct for a parallel gripper
      enclosing an object shorter than the finger length, and cross-validated against
      the real MTC-planned/executed grasp height for the S2 cube (0.0175m, exactly the
      cube's mid-height, not its top surface). Requires points_xyz to carry both the
      top and bottom of the graspable extent (true of GT/mesh geometry; NOT true of a
      single wrist-camera view, which only ever sees the top surface). "region_top"
      places the pinch point at the region's highest point instead -- use this only
      when points_xyz is known to be a top-surface-only view and a separate object
      height estimate (see lifting.object_height_estimate) has not yet been combined
      in; it will otherwise grasp too high and miss real contact, as first observed
      when this module's own demo used top-face-only points against the real robot.
    """
    points = np.asarray(points_xyz, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points_xyz must be (N, 3), got {points.shape}")
    if not np.all(np.isfinite(points)):
        raise ValueError("points_xyz contains non-finite values")
    n_points = points.shape[0]

    if task_region_mask is None:
        region_mask = np.ones(n_points, dtype=bool)
    else:
        region_mask = np.asarray(task_region_mask, dtype=bool)
        if region_mask.shape != (n_points,):
            raise ValueError("task_region_mask must have shape (N,) matching points_xyz")
    region_indices = np.nonzero(region_mask)[0]

    candidates: list[GraspCandidate] = []
    # Sampling only [0, pi) is deliberate: a parallel gripper's closing axis is a
    # line, not a direction, so yaw and yaw+pi describe the same grasp.
    yaws = yaw_start_rad + np.linspace(0.0, math.pi, num_yaws, endpoint=False)

    for i, yaw in enumerate(yaws):
        candidate_id = f"G{i:03d}"

        if region_indices.size == 0:
            candidates.append(_empty_candidate(candidate_id, frame_id, yaw, "EMPTY_TASK_REGION"))
            continue

        region_points = points[region_indices]
        axis = _closing_axis(yaw)
        projections = region_points[:, 0] * axis[0] + region_points[:, 1] * axis[1]
        proj_min, proj_max = float(projections.min()), float(projections.max())
        opening_width = proj_max - proj_min
        center_along_axis = 0.5 * (proj_min + proj_max)

        # Recenter on the midpoint of the projected span, not the region centroid:
        # for an off-center task region, the grasp must straddle the region's own
        # extent along the closing axis, not wherever its centroid happens to sit.
        centroid_xy = region_points[:, :2].mean(axis=0)
        center_xy = centroid_xy + (center_along_axis - float(centroid_xy @ axis)) * axis
        if grasp_height_mode == "region_midpoint":
            grasp_z = float(0.5 * (region_points[:, 2].min() + region_points[:, 2].max()))
        elif grasp_height_mode == "region_top":
            grasp_z = float(region_points[:, 2].max())
        else:
            raise ValueError(f"unknown grasp_height_mode: {grasp_height_mode!r}")

        span = max(opening_width, 1e-9)
        band = span * edge_band_fraction
        low_mask = projections <= proj_min + band
        high_mask = projections >= proj_max - band
        low_indices = region_indices[low_mask]
        high_indices = region_indices[high_mask]

        reasons: list[str] = []
        if opening_width > gripper_max_opening_m:
            reasons.append("WIDTH_EXCEEDED")
        if opening_width < gripper_min_opening_m:
            reasons.append("WIDTH_TOO_NARROW")
        if grasp_z - table_z_m < table_clearance_m:
            reasons.append("TABLE_CLEARANCE")
        if low_indices.size < min_edge_points_per_side or high_indices.size < min_edge_points_per_side:
            reasons.append("UNKNOWN_GEOMETRY")
        else:
            low_centroid = points[low_indices, :2].mean(axis=0)
            high_centroid = points[high_indices, :2].mean(axis=0)
            edge_vector = high_centroid - low_centroid
            edge_norm = np.linalg.norm(edge_vector)
            if edge_norm < 1e-9:
                reasons.append("UNKNOWN_GEOMETRY")
            else:
                cos_angle = float(np.clip((edge_vector / edge_norm) @ axis, -1.0, 1.0))
                angle_deg = math.degrees(math.acos(abs(cos_angle)))
                if angle_deg > antipodal_angle_tolerance_deg:
                    reasons.append("NON_ANTIPODAL_EDGE_CONTACT")

        quat = top_down_grasp_quaternion(float(yaw))
        approach_axis_world = np.array([0.0, 0.0, -1.0])
        tcp_position = np.array([center_xy[0], center_xy[1], grasp_z])
        pregrasp_position = tcp_position - approach_axis_world * pregrasp_standoff_m
        gripper_base_position = tcp_position - approach_axis_world * pinch_offset_m

        # Lower score is better, and sorted ascending below: prefer the candidate
        # that uses the SMALLEST fraction of the gripper's max opening, i.e. the one
        # with the most closing margin left. (An earlier version of this formula
        # scored the opposite way -- lower score for LESS margin -- and would have
        # picked the riskiest orientation as "best"; caught by comparing the
        # generator's output against the already MTC-verified real grasp, which uses
        # the axis-aligned, most-margin orientation for the S2 cube.)
        score = opening_width / max(gripper_max_opening_m, 1e-9)

        candidates.append(GraspCandidate(
            candidate_id=candidate_id, frame_id=frame_id, source="top_down_yaw_grid",
            yaw_rad=float(yaw),
            tcp_position_m=tuple(tcp_position.tolist()),
            tcp_quaternion_xyzw=tuple(quat.tolist()),
            pregrasp_position_m=tuple(pregrasp_position.tolist()),
            pregrasp_quaternion_xyzw=tuple(quat.tolist()),
            gripper_base_position_m=tuple(gripper_base_position.tolist()),
            approach_axis_world=tuple(approach_axis_world.tolist()),
            opening_width_m=float(opening_width),
            contact_point_indices=tuple(int(v) for v in np.concatenate([low_indices, high_indices])),
            score=float(score),
            rejection_reasons=tuple(reasons),
        ))

    accepted = sorted((c for c in candidates if c.accepted), key=lambda c: c.score)
    rejected = [c for c in candidates if not c.accepted]
    ordered = accepted + rejected
    if max_candidates is not None:
        ordered = ordered[:max_candidates]
    return ordered
