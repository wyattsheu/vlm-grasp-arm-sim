"""Tests for mpg.curobo_bridge.frames.

Runs under env_robot129_research (pure numpy/quaternion tests) *and* under
env_robot129_curobo (adds the yourdfpy URDF cross-check and a live cuRobo FK
check) -- see docs/dev_guide_paper_core_and_dashboard_plan.md §7 for exact
commands. Tests that need yourdfpy/curobo skip themselves (not fail) when
those packages aren't on the path, so this file works unmodified in both
venvs.
"""
from __future__ import annotations

import math
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np  # noqa: E402

from mpg.curobo_bridge.frames import (  # noqa: E402
    GRASP_APPROACH_AXIS_LOCAL,
    GRASPGENX_CLOSING_AXIS_LOCAL,
    ROBOT129_CLOSING_AXIS_LOCAL,
    approach_direction_to_quat_xyzw,
    candidate_tcp_pose_to_curobo,
    curobo_pose_to_xyzw,
    graspgenx_pose_to_robot129_pinch,
    quat_multiply_xyzw,
    wxyz_to_xyzw,
    xyzw_to_wxyz,
)

_URDF_PATH = (
    Path(__file__).resolve().parents[2]
    / "ros2_ws/src/robot129_description/urdf/robot129.urdf"
)


def _rotate_vector_xyzw(q_xyzw, v):
    """Rotate 3-vector v by quaternion q (xyzw), via the double quat product
    q * (v, 0) * q_conj -- kept local to the test so it doesn't depend on
    grasp_candidates.py's implementation of the same idea.
    """
    x, y, z, w = np.asarray(q_xyzw, dtype=float)
    vx, vy, vz = np.asarray(v, dtype=float)
    # standard quaternion-vector rotation formula
    qv = np.array([x, y, z])
    uv = np.cross(qv, [vx, vy, vz])
    uuv = np.cross(qv, uv)
    return np.array([vx, vy, vz]) + 2 * (w * uv + uuv)


class TestQuaternionOrder(unittest.TestCase):
    def test_xyzw_wxyz_roundtrip(self):
        q_xyzw = np.array([0.1, 0.2, 0.3, 0.9273618])
        q_xyzw = q_xyzw / np.linalg.norm(q_xyzw)
        q_wxyz = xyzw_to_wxyz(q_xyzw)
        self.assertTrue(np.allclose(wxyz_to_xyzw(q_wxyz), q_xyzw))

    def test_xyzw_to_wxyz_identity(self):
        # identity rotation, xyzw = (0,0,0,1)
        self.assertTrue(np.allclose(xyzw_to_wxyz([0, 0, 0, 1]), [1, 0, 0, 0]))


class TestCandidateToCurobo(unittest.TestCase):
    def test_candidate_tcp_pose_to_curobo(self):
        candidate = {
            "tcp_pose": {
                "position_m": [0.3, -0.05, 0.12],
                "quaternion_xyzw": [0.0, 1.0, 0.0, 0.0],  # top-down flip, no yaw
            }
        }
        pose = candidate_tcp_pose_to_curobo(candidate)
        self.assertTrue(np.allclose(pose.position_m, [0.3, -0.05, 0.12]))
        self.assertTrue(np.allclose(pose.quaternion_wxyz, [0.0, 0.0, 1.0, 0.0]))

    def test_curobo_pose_to_xyzw_roundtrip(self):
        pos = np.array([0.1, 0.2, 0.3])
        quat_wxyz = np.array([0.7071068, 0.7071068, 0.0, 0.0])
        pos_out, quat_xyzw = curobo_pose_to_xyzw(pos, quat_wxyz)
        self.assertTrue(np.allclose(pos_out, pos))
        self.assertTrue(np.allclose(xyzw_to_wxyz(quat_xyzw), quat_wxyz))


class TestGraspGenXConversion(unittest.TestCase):
    def test_approach_axis_unchanged(self):
        # identity orientation: GraspGen-X pose aligned with the pinch frame axes
        q_identity_xyzw = np.array([0.0, 0.0, 0.0, 1.0])
        _, q_out = graspgenx_pose_to_robot129_pinch([0, 0, 0], q_identity_xyzw)
        approach_out = _rotate_vector_xyzw(q_out, GRASP_APPROACH_AXIS_LOCAL)
        # approach axis (+Z) is shared between GraspGen-X and robot129, and the
        # correction is a rotation about local Z, so it must be unchanged.
        self.assertTrue(np.allclose(approach_out, GRASP_APPROACH_AXIS_LOCAL, atol=1e-6))

    def test_closing_axis_maps_to_robot129_y(self):
        q_identity_xyzw = np.array([0.0, 0.0, 0.0, 1.0])
        _, q_out = graspgenx_pose_to_robot129_pinch([0, 0, 0], q_identity_xyzw)
        closing_out = _rotate_vector_xyzw(q_out, GRASPGENX_CLOSING_AXIS_LOCAL)
        # GraspGen-X's local +X closing axis must land on the same *line* as
        # robot129's real closing axis (local Y) -- sign doesn't matter for a
        # symmetric parallel gripper.
        cos_angle = abs(float(np.dot(closing_out, ROBOT129_CLOSING_AXIS_LOCAL)))
        self.assertAlmostEqual(cos_angle, 1.0, places=6)

    def test_position_passthrough(self):
        pos_in = [0.4, -0.02, 0.15]
        pos_out, _ = graspgenx_pose_to_robot129_pinch(pos_in, [0, 0, 0, 1])
        self.assertTrue(np.allclose(pos_out, pos_in))


class TestFingerAxisMatchesUrdf(unittest.TestCase):
    """Independently re-derives ROBOT129_CLOSING_AXIS_LOCAL from the live URDF
    (not from frames.py's hardcoded constant) so a future URDF edit that moves
    the fingers breaks this test loudly instead of silently invalidating the
    constant. Needs yourdfpy -- only present in env_robot129_curobo; skips
    under env_robot129_research.
    """

    def setUp(self):
        try:
            import yourdfpy  # noqa: F401
        except ImportError:
            self.skipTest("yourdfpy not installed in this venv (see dev guide §7)")
        if not _URDF_PATH.exists():
            self.skipTest(f"URDF not found at {_URDF_PATH}")

    def test_finger_axis_matches_urdf(self):
        import yourdfpy

        urdf = yourdfpy.URDF.load(str(_URDF_PATH), load_meshes=False)

        def link7_pos_in_gripper_base(joint7_val):
            cfg = {
                "joint1": np.float64(0), "joint2": np.float64(0), "joint3": np.float64(0),
                "joint4": np.float64(0), "joint5": np.float64(0), "joint6": np.float64(0),
                "joint7": np.float64(joint7_val), "joint8": np.float64(-joint7_val),
            }
            urdf.update_cfg(cfg)
            T_gb = urdf.get_transform(frame_to="gripper_base", frame_from="world")
            T_link7 = urdf.get_transform(frame_to="link7", frame_from="world")
            T_rel = np.linalg.inv(T_gb) @ T_link7
            return T_rel[:3, 3]

        p0 = link7_pos_in_gripper_base(0.0)
        p1 = link7_pos_in_gripper_base(0.035)
        displacement = p1 - p0
        axis = displacement / np.linalg.norm(displacement)
        cos_angle = abs(float(np.dot(axis, ROBOT129_CLOSING_AXIS_LOCAL)))
        self.assertAlmostEqual(cos_angle, 1.0, places=4)


class TestPinchCenterFkMatchesUrdf(unittest.TestCase):
    """Cross-checks research/configs/curobo/robot129.yml's pinch_center
    extra_link (gripper_base + 0.125 m local Z) against an independent FK
    computation from the same URDF via yourdfpy. Skips under
    env_robot129_research (no yourdfpy).
    """

    def setUp(self):
        try:
            import yourdfpy  # noqa: F401
        except ImportError:
            self.skipTest("yourdfpy not installed in this venv (see dev guide §7)")
        if not _URDF_PATH.exists():
            self.skipTest(f"URDF not found at {_URDF_PATH}")

    def test_pinch_center_fk(self):
        import yourdfpy

        urdf = yourdfpy.URDF.load(str(_URDF_PATH), load_meshes=False)
        cfg = {
            "joint1": np.float64(0.0), "joint2": np.float64(1.2), "joint3": np.float64(-1.25),
            "joint4": np.float64(0.0), "joint5": np.float64(0.15), "joint6": np.float64(0.0),
            "joint7": np.float64(0.0345), "joint8": np.float64(-0.0345),
        }
        urdf.update_cfg(cfg)
        T_gb = urdf.get_transform(frame_to="gripper_base", frame_from="world")
        pinch_world = T_gb @ np.array([0, 0, 0.125, 1.0])

        # Same SRDF-home joint values, computed via cuRobo's own Kinematics --
        # see docs/progress/curobo_step1_robot_config.md for how this number
        # was first obtained.
        expected = np.array([0.395029392, -1.9620870261860546e-06, 0.453423842])
        self.assertTrue(
            np.allclose(pinch_world[:3], expected, atol=1e-3),
            f"pinch_center FK mismatch: yourdfpy={pinch_world[:3]} vs cuRobo={expected}",
        )


class TestApproachDirectionToQuat(unittest.TestCase):
    def test_approach_up_local_z_points_world_up(self):
        q_xyzw = approach_direction_to_quat_xyzw([0, 0, 1], closing_hint_world=[0, 1, 0])
        approach_out = _rotate_vector_xyzw(q_xyzw, GRASP_APPROACH_AXIS_LOCAL)
        self.assertTrue(np.allclose(approach_out, [0, 0, 1], atol=1e-6))

    def test_approach_left_local_z_points_world_y(self):
        # "left" == world +Y, matching mpg.urdf_fk.NAMED_WORLD_DIRECTIONS.
        q_xyzw = approach_direction_to_quat_xyzw([0, 1, 0], closing_hint_world=[0, 0, 1])
        approach_out = _rotate_vector_xyzw(q_xyzw, GRASP_APPROACH_AXIS_LOCAL)
        self.assertTrue(np.allclose(approach_out, [0, 1, 0], atol=1e-6))

    def test_closing_hint_is_honored_when_orthogonal(self):
        # closing_hint already orthogonal to approach -> closing axis should
        # land exactly on it (same line, sign may flip -- symmetric gripper).
        q_xyzw = approach_direction_to_quat_xyzw([0, 0, 1], closing_hint_world=[1, 0, 0])
        closing_out = _rotate_vector_xyzw(q_xyzw, ROBOT129_CLOSING_AXIS_LOCAL)
        cos_angle = abs(float(np.dot(closing_out, [1, 0, 0])))
        self.assertAlmostEqual(cos_angle, 1.0, places=6)

    def test_output_is_unit_quaternion(self):
        q_xyzw = approach_direction_to_quat_xyzw([0.3, -0.1, 0.9])
        self.assertAlmostEqual(float(np.linalg.norm(q_xyzw)), 1.0, places=6)

    def test_parallel_hint_falls_back_instead_of_dividing_by_zero(self):
        # closing_hint parallel to approach must not raise or produce NaN.
        q_xyzw = approach_direction_to_quat_xyzw([0, 0, 1], closing_hint_world=[0, 0, 1])
        self.assertFalse(np.any(np.isnan(q_xyzw)))
        self.assertAlmostEqual(float(np.linalg.norm(q_xyzw)), 1.0, places=6)
        approach_out = _rotate_vector_xyzw(q_xyzw, GRASP_APPROACH_AXIS_LOCAL)
        self.assertTrue(np.allclose(approach_out, [0, 0, 1], atol=1e-6))


if __name__ == "__main__":
    unittest.main()
