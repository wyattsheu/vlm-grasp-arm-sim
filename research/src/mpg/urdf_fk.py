"""Pure-numpy forward kinematics for robot129.urdf.

No ROS, Isaac, or cuRobo dependency -- runs under any venv that has numpy
(env_robot129_research, env_robot129_ros, env_robot129_curobo, ...). This is
the FK engine `tools/verify_fk_consistency.py` and `tools/send_joint_cmd.py`
both build on top of (see their call sites for the cross-checks against Isaac
and cuRobo respectively that establish this module is correct, not just that
it runs).

Everything here works from the URDF's `<joint>` elements directly (origin
xyz/rpy, axis, type) -- it does not read SRDF, MoveIt config, or cuRobo's
robot129.yml, so it stays correct even when those derived files lag behind a
URDF edit.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# pinch_center is not a URDF link -- it is a virtual tool frame that
# research/configs/curobo/robot129.yml defines as a fixed, zero-rotation
# offset from gripper_base along local +Z (see
# mpg.curobo_bridge.frames module docstring, "KNOWN MISMATCH" section, and
# robot129.yml's kinematics.extra_links.pinch_center.fixed_transform). Kept
# here as the single numeric source so this module and frames.py agree
# without importing curobo.
PINCH_CENTER_OFFSET_FROM_GRIPPER_BASE_M = 0.125

DEFAULT_URDF_RELATIVE_PATH = "ros2_ws/src/robot129_description/urdf/robot129.urdf"

ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


def _rpy_to_matrix(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_angle_to_matrix(axis, angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    kx, ky, kz = a
    k = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> quaternion, xyzw order (grasp_candidate_v0 / ROS
    convention -- see mpg.curobo_bridge.frames module docstring).
    """
    m00, m01, m02 = rotation[0]
    m10, m11, m12 = rotation[1]
    m20, m21, m22 = rotation[2]
    trace = m00 + m11 + m22
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def rotate_vector_xyzw(quaternion_xyzw, vector) -> np.ndarray:
    """Rotate a 3-vector by a quaternion (xyzw), standard double-product
    formula. test_curobo_frames.py's `_rotate_vector_xyzw` re-derives this
    independently rather than importing it, so a bug here doesn't silently
    validate itself through that test.
    """
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=float)
    v = np.asarray(vector, dtype=float)
    qv = np.array([x, y, z])
    uv = np.cross(qv, v)
    uuv = np.cross(qv, uv)
    return v + 2.0 * (w * uv + uuv)


class UrdfChainFk:
    """Loads one URDF and caches its joint tree, so repeated FK calls (e.g.
    once per waypoint in a demo sequence) reparse the XML only once.
    """

    def __init__(self, urdf_path: Path | str):
        self.urdf_path = Path(urdf_path)
        root = ET.parse(self.urdf_path).getroot()
        self._joint_by_child: dict[str, ET.Element] = {
            j.find("child").attrib["link"]: j for j in root.findall("joint")
        }

    def _chain_to_root(self, target_link: str, root_link: str) -> list[ET.Element]:
        """Joints from root_link to target_link, root-to-tip order. Walks
        parent pointers from target_link back to root_link -- works for any
        target reachable by a single chain, true for every link in
        robot129.urdf (no closed kinematic loops).
        """
        chain: list[ET.Element] = []
        link = target_link
        seen: set[str] = set()
        while link != root_link:
            if link in seen:
                raise ValueError(f"cycle detected walking up from {target_link!r} to {root_link!r}")
            seen.add(link)
            joint = self._joint_by_child.get(link)
            if joint is None:
                raise ValueError(f"no joint has child link {link!r}; is {root_link!r} really an ancestor of it?")
            chain.append(joint)
            link = joint.find("parent").attrib["link"]
        chain.reverse()
        return chain

    def forward_kinematics(
        self, joint_values: dict[str, float], target_link: str, root_link: str = "base_link",
    ) -> np.ndarray:
        """4x4 homogeneous transform of target_link in root_link's frame.
        `joint_values`: {joint_name: angle_rad_or_distance_m}. Movable joints
        missing from the dict default to 0.0 (URDF's implicit zero pose) --
        callers that care about a specific pose should pass every joint
        explicitly rather than rely on this default.
        """
        chain = self._chain_to_root(target_link, root_link)
        transform = np.eye(4)
        for joint in chain:
            origin = joint.find("origin")
            xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()] if origin is not None else [0.0, 0.0, 0.0]
            rpy = [float(v) for v in origin.attrib.get("rpy", "0 0 0").split()] if origin is not None else [0.0, 0.0, 0.0]
            fixed = np.eye(4)
            fixed[:3, :3] = _rpy_to_matrix(rpy)
            fixed[:3, 3] = xyz
            transform = transform @ fixed

            joint_type = joint.attrib["type"]
            if joint_type in ("revolute", "continuous", "prismatic"):
                name = joint.attrib["name"]
                value = joint_values.get(name, 0.0)
                axis_el = joint.find("axis")
                axis = [float(v) for v in axis_el.attrib["xyz"].split()] if axis_el is not None else [1.0, 0.0, 0.0]
                moving = np.eye(4)
                if joint_type == "prismatic":
                    unit_axis = np.asarray(axis, dtype=float)
                    unit_axis = unit_axis / np.linalg.norm(unit_axis)
                    moving[:3, 3] = unit_axis * value
                else:
                    moving[:3, :3] = _axis_angle_to_matrix(axis, value)
                transform = transform @ moving
        return transform

    def link_pose(
        self, joint_values: dict[str, float], target_link: str, root_link: str = "base_link",
    ) -> tuple[np.ndarray, np.ndarray]:
        """(position_m, quaternion_xyzw) of target_link in root_link's frame."""
        transform = self.forward_kinematics(joint_values, target_link, root_link)
        return transform[:3, 3], matrix_to_quaternion_xyzw(transform[:3, :3])

    def pinch_center_pose(
        self,
        joint_values_6: list[float] | dict[str, float],
        root_link: str = "base_link",
        arm_joint_names: tuple[str, ...] = ARM_JOINT_NAMES,
    ) -> tuple[np.ndarray, np.ndarray]:
        """(position_m, quaternion_xyzw) of pinch_center = gripper_base +
        PINCH_CENTER_OFFSET_FROM_GRIPPER_BASE_M along gripper_base's local +Z
        (zero-rotation offset, so pinch_center's orientation equals
        gripper_base's -- see robot129.yml's extra_links.pinch_center).
        """
        if not isinstance(joint_values_6, dict):
            joint_values_6 = list(joint_values_6)
            if len(joint_values_6) != len(arm_joint_names):
                raise ValueError(f"expected {len(arm_joint_names)} joint values, got {len(joint_values_6)}")
            joint_values_6 = dict(zip(arm_joint_names, joint_values_6))
        transform_gripper_base = self.forward_kinematics(joint_values_6, "gripper_base", root_link)
        pinch_local = np.array([0.0, 0.0, PINCH_CENTER_OFFSET_FROM_GRIPPER_BASE_M, 1.0])
        pinch_world = transform_gripper_base @ pinch_local
        return pinch_world[:3], matrix_to_quaternion_xyzw(transform_gripper_base[:3, :3])


# ---- approach-axis classification, for reporting "gripper points up/left/..." ---

NAMED_WORLD_DIRECTIONS: dict[str, np.ndarray] = {
    "+X": np.array([1.0, 0.0, 0.0]),
    "-X": np.array([-1.0, 0.0, 0.0]),
    "+Y (left)": np.array([0.0, 1.0, 0.0]),
    "-Y (right)": np.array([0.0, -1.0, 0.0]),
    "+Z (up)": np.array([0.0, 0.0, 1.0]),
    "-Z (down)": np.array([0.0, 0.0, -1.0]),
}


def quaternion_angular_distance_xyzw(quaternion_a_xyzw, quaternion_b_xyzw) -> float:
    """Smallest rotation angle (radians) between two orientations, xyzw order.
    Uses |dot(a, b)| since q and -q represent the same rotation (double
    cover), so this never returns a spuriously large angle for two
    quaternions that differ only by an overall sign flip. Used by
    tools/send_joint_cmd.py and reactive.py's waypoint-arrival check to turn
    a quaternion pair into a single scalar error against a tolerance.
    """
    a = np.asarray(quaternion_a_xyzw, dtype=float)
    a = a / np.linalg.norm(a)
    b = np.asarray(quaternion_b_xyzw, dtype=float)
    b = b / np.linalg.norm(b)
    cos_half_angle = min(1.0, abs(float(np.dot(a, b))))
    return 2.0 * math.acos(cos_half_angle)


def classify_approach_direction(
    quaternion_xyzw, local_approach_axis=(0.0, 0.0, 1.0),
) -> tuple[str, float]:
    """World-frame direction the gripper's approach axis (local +Z by this
    repo's convention -- see frames.py's GRASP_APPROACH_AXIS_LOCAL) currently
    points, as the closest named world axis plus the angular error to it in
    degrees, e.g. ("+Z (up)", 2.1) meaning "approach ≈ +Z (up), off by 2.1°".
    """
    world_vec = rotate_vector_xyzw(quaternion_xyzw, local_approach_axis)
    world_vec = world_vec / np.linalg.norm(world_vec)
    best_name, best_cos = None, -2.0
    for name, axis in NAMED_WORLD_DIRECTIONS.items():
        cos_angle = float(np.dot(world_vec, axis))
        if cos_angle > best_cos:
            best_cos, best_name = cos_angle, name
    angle_deg = math.degrees(math.acos(float(np.clip(best_cos, -1.0, 1.0))))
    return best_name, angle_deg
