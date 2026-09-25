"""Frame and quaternion conversions between robot129's grasp_candidate_v0 contract
and cuRoboV2's Pose/GoalToolPose types.

This module is deliberately narrow: it converts *representations* (quaternion
component order, external-tool axis conventions), not physics. It does not
try to silently repair upstream data -- see "KNOWN MISMATCH" below.

## Conventions this module bridges

- **grasp_candidate_v0** (research/src/mpg/grasp_contract.py): position in
  metres, ``quaternion_xyzw`` (SciPy/ROS order), frame_id "world" (identical
  to "base_link" -- ``world_to_base`` in the URDF is an identity fixed joint),
  tcp_frame "pinch_center".
- **cuRoboV2** ``Pose``: position in metres, ``quaternion`` in **wxyz** order
  (see ``curobo/examples/getting_started/motion_planning.py`` step 2: "Obstacle
  poses use the format ``[x, y, z, qw, qx, qy, qz]``").
- **GraspGen-X** (arXiv 2606.00998, github.com/NVlabs/GraspGenX): grasp poses
  have the gripper's approach axis on local **+Z** and the closing direction
  on local **+X** ("the gripper's approach axis is aligned with +Z in the
  base frame, with +X indicating the closing direction").

## robot129's real closing axis (verified against the URDF, not assumed)

joint7/joint8 (the two finger prismatic joints) are children of
``gripper_base`` with origins ``rpy="1.5708 0 0"`` / ``"1.5708 0 -3.1416"``
(robot129.urdf:255,284). Numerically driving joint7 from 0 -> 0.035 m and
re-expressing link7's/link8's origin in the ``gripper_base`` frame (see
``research/tests/test_curobo_frames.py::test_finger_axis_matches_urdf``, which
redoes this with yourdfpy so it breaks loudly if the URDF ever changes) gives:

    link7 (joint7=0 -> 0.035):  gripper_base-local  (0, -0.035, 0)
    link8 (joint8=0 -> -0.035): gripper_base-local  (0, +0.035, 0)

So the fingers separate along **gripper_base/pinch_center local Y**, not X.
``pinch_center`` (this module, and research/configs/curobo/robot129.yml) is a
zero-rotation offset of gripper_base along local +Z, so this holds for
pinch_center too.

## KNOWN MISMATCH -- not fixed here, flagged instead

``research/src/mpg/grasp_candidates.py``'s ``top_down_grasp_quaternion()``
puts the *scored* closing axis on local **+X** (``_closing_axis(yaw)`` follows
the same convention the quaternion construction produces). That is 90 degrees
off from the real finger travel axis (local Y) derived above. This was first
noted by exploration during this cuRobo integration (2026-09-24) but
``grasp_candidates.py`` is S3 scope (candidate generation), not S6/cuRobo
integration scope, so this module does **not** silently rotate incoming
``grasp_candidate_v0`` poses to compensate -- that would hide a bug that
belongs to the generator, and would double-correct if the generator is fixed
later. It only matters for asymmetric objects (e.g. the hammer handle, which
has never been executed through MTC either); it is invisible on the
symmetric cube. Flag for whoever owns grasp_candidates.py next.

The GraspGen-X conversion below (a *different*, external, explicitly-declared
convention) is a legitimate, documented rotation, not a workaround for the
mismatch above.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# Real closing axis of robot129's gripper, in gripper_base/pinch_center local
# coordinates -- see module docstring and test_finger_axis_matches_urdf.
ROBOT129_CLOSING_AXIS_LOCAL = np.array([0.0, 1.0, 0.0])

# GraspGen-X's declared closing axis (arXiv 2606.00998): local +X.
GRASPGENX_CLOSING_AXIS_LOCAL = np.array([1.0, 0.0, 0.0])

# GraspGen-X and robot129 agree on the approach axis: local +Z.
GRASP_APPROACH_AXIS_LOCAL = np.array([0.0, 0.0, 1.0])


# ---- quaternion order -----------------------------------------------------------

def xyzw_to_wxyz(q_xyzw) -> np.ndarray:
    """[x, y, z, w] -> [w, x, y, z]."""
    x, y, z, w = np.asarray(q_xyzw, dtype=float)
    return np.array([w, x, y, z])


def wxyz_to_xyzw(q_wxyz) -> np.ndarray:
    """[w, x, y, z] -> [x, y, z, w]."""
    w, x, y, z = np.asarray(q_wxyz, dtype=float)
    return np.array([x, y, z, w])


def quat_multiply_xyzw(q1_xyzw, q2_xyzw) -> np.ndarray:
    """Hamilton product q1 * q2, both and the result in xyzw order.

    Matches the "apply q2 first, then q1" convention used throughout
    research/src/mpg/grasp_candidates.py (se3_compose).
    """
    x1, y1, z1, w1 = np.asarray(q1_xyzw, dtype=float)
    x2, y2, z2, w2 = np.asarray(q2_xyzw, dtype=float)
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


# 90-degree rotation about local Z, xyzw. Rz(-90deg) maps local +X -> local
# -Y: composed *after* a GraspGen-X pose (local frame, right-multiply), it
# takes GraspGen-X's +X closing axis to (0, -1, 0) -- the same line as
# ROBOT129_CLOSING_AXIS_LOCAL (the axis is unsigned/symmetric: a parallel
# gripper's closing direction is a line, not a signed vector).
_RZ_NEG90_XYZW = np.array([0.0, 0.0, -np.sin(np.pi / 4), np.cos(np.pi / 4)])


def graspgenx_pose_to_robot129_pinch(position_m, quaternion_xyzw) -> tuple[np.ndarray, np.ndarray]:
    """Convert a GraspGen-X grasp pose (approach +Z, closing +X) into a pose
    expressed the way robot129's pinch_center expects it (approach +Z,
    closing local Y -- the real finger travel axis, see module docstring).

    Position is untouched (both conventions place the origin at the grasp
    point / tool frame origin). Returns (position_m, quaternion_xyzw).
    """
    position_m = np.asarray(position_m, dtype=float)
    q_out = quat_multiply_xyzw(quaternion_xyzw, _RZ_NEG90_XYZW)
    return position_m, q_out


# ---- grasp_candidate_v0 <-> cuRobo Pose ------------------------------------------

@dataclass(frozen=True)
class CuroboPoseXYZW:
    """Plain-numpy stand-in for curobo.types.Pose, used so this module has no
    hard dependency on the `curobo` package (frames.py's pure-math parts run
    fine under env_robot129_research; only plan_grasp.py needs env_robot129_curobo).
    Call .to_curobo_pose(device_cfg) to get a real curobo.types.Pose.
    """
    position_m: np.ndarray       # (3,)
    quaternion_wxyz: np.ndarray  # (4,)

    def to_curobo_pose(self, device_cfg=None):
        import torch
        from curobo.types import Pose

        kwargs = {} if device_cfg is None else {"device_cfg": device_cfg}
        return Pose(
            position=torch.tensor([self.position_m], dtype=torch.float32),
            quaternion=torch.tensor([self.quaternion_wxyz], dtype=torch.float32),
            **kwargs,
        )


def candidate_tcp_pose_to_curobo(candidate_dict: dict, pose_key: str = "tcp_pose") -> CuroboPoseXYZW:
    """grasp_candidate_v0[pose_key] (position_m + quaternion_xyzw, tcp_frame
    "pinch_center", frame_id "world") -> a pose ready for cuRobo's
    "pinch_center" tool frame in the "world"-rooted robot129.yml kinematics.

    No rotation is applied here -- see module docstring's KNOWN MISMATCH note.
    pose_key is "tcp_pose" or "pregrasp_pose" (grasp_contract.py's schema).
    """
    pose = candidate_dict[pose_key]
    position_m = np.asarray(pose["position_m"], dtype=float)
    quaternion_wxyz = xyzw_to_wxyz(pose["quaternion_xyzw"])
    return CuroboPoseXYZW(position_m=position_m, quaternion_wxyz=quaternion_wxyz)


def curobo_pose_to_xyzw(position_m, quaternion_wxyz) -> tuple[np.ndarray, np.ndarray]:
    """cuRobo (position, quaternion_wxyz) -> (position_m, quaternion_xyzw), for
    writing results back out in grasp_candidate_v0 / MTC sub_trajectories style.
    """
    return np.asarray(position_m, dtype=float), wxyz_to_xyzw(quaternion_wxyz)


# ---- named approach direction -> pinch_center orientation ------------------------

# CLI/YAML-friendly aliases for a world-frame approach direction. Distinct from
# mpg.urdf_fk.NAMED_WORLD_DIRECTIONS, which uses "+Z (up)"-style keys meant for
# *display* (classify_approach_direction's output), not for typing in a config file
# or on a command line. Shared by research/scripts/select_demo_poses.py and
# research/src/mpg/curobo_bridge/waypoints.py so a waypoint YAML's `approach: up`
# and select_demo_poses.py's `--direction-a up` mean the exact same vector.
NAMED_DIRECTION_ALIASES: dict[str, np.ndarray] = {
    "up": np.array([0.0, 0.0, 1.0]),
    "down": np.array([0.0, 0.0, -1.0]),
    "left": np.array([0.0, 1.0, 0.0]),
    "right": np.array([0.0, -1.0, 0.0]),
    "forward": np.array([1.0, 0.0, 0.0]),
    "backward": np.array([-1.0, 0.0, 0.0]),
}


def approach_direction_to_quat_xyzw(approach_world, closing_hint_world=(0.0, 0.0, 1.0)) -> np.ndarray:
    """Build a pinch_center orientation (quaternion_xyzw) whose local approach
    axis (+Z, GRASP_APPROACH_AXIS_LOCAL) points along `approach_world`, and
    whose local closing axis (+Y, ROBOT129_CLOSING_AXIS_LOCAL -- the real
    finger travel axis, see module docstring) is as close as possible to
    `closing_hint_world` while staying orthogonal to the approach direction.

    Used by research/scripts/select_demo_poses.py to turn a named direction
    ("gripper points up", "gripper points left") into a concrete orientation
    to feed cuRobo IK -- not measured or copied from any existing candidate.
    Only the repo's default axis convention (approach=local+Z, closing=local+Y)
    is implemented; this is not a general "rotate frame A onto frame B" utility.
    """
    z_world = np.asarray(approach_world, dtype=float)
    z_world = z_world / np.linalg.norm(z_world)
    hint = np.asarray(closing_hint_world, dtype=float)
    hint = hint / np.linalg.norm(hint)
    if abs(float(np.dot(hint, z_world))) > 0.999:
        # hint is (anti)parallel to the approach direction -- no orthogonal
        # component survives the projection below. Fall back to a hint that
        # is guaranteed non-parallel to z_world (world +X, or +Y if z_world
        # is itself close to +X).
        hint = np.array([1.0, 0.0, 0.0]) if abs(z_world[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    y_world = hint - float(np.dot(hint, z_world)) * z_world
    y_world = y_world / np.linalg.norm(y_world)
    x_world = np.cross(y_world, z_world)  # right-handed: X = Y x Z
    rotation = np.column_stack([x_world, y_world, z_world])  # local X,Y,Z axes expressed in world

    # Local import: keeps frames.py's top-level import list free of a
    # same-package dependency for the common case (candidate/pose conversion)
    # that doesn't need this function.
    from mpg.urdf_fk import matrix_to_quaternion_xyzw

    return matrix_to_quaternion_xyzw(rotation)
