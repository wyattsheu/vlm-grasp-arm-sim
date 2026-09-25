"""Tests for mpg.urdf_fk. Pure numpy -- runs in every venv."""
from __future__ import annotations

import json
import math
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np  # noqa: E402

from mpg.urdf_fk import (  # noqa: E402
    UrdfChainFk,
    classify_approach_direction,
    matrix_to_quaternion_xyzw,
    quaternion_angular_distance_xyzw,
    rotate_vector_xyzw,
)

_REPO_ROOT = Path(__file__).resolve().parents[2]
_URDF_PATH = _REPO_ROOT / "ros2_ws/src/robot129_description/urdf/robot129.urdf"
_GRIPPER_PROBE_PATH = _REPO_ROOT / "out/lesson_06/gripper_probe.json"


def _quat_xyzw_to_matrix(q_xyzw) -> np.ndarray:
    x, y, z, w = np.asarray(q_xyzw, dtype=float)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


@unittest.skipUnless(_URDF_PATH.exists(), f"URDF not found at {_URDF_PATH}")
class TestUrdfChainFk(unittest.TestCase):
    def setUp(self):
        self.chain = UrdfChainFk(_URDF_PATH)

    def test_identity_pose_gripper_base_matches_manual_chain(self):
        # Zero joint values: link7's/link8's origins are pure translation
        # chains with no rotation contribution beyond fixed joints, so this
        # mainly checks _chain_to_root and matrix composition wire correctly.
        pos, quat = self.chain.link_pose({}, "gripper_base")
        self.assertEqual(pos.shape, (3,))
        self.assertEqual(quat.shape, (4,))
        self.assertAlmostEqual(float(np.linalg.norm(quat)), 1.0, places=6)

    def test_pinch_center_matches_srdf_home_cross_check(self):
        # Same SRDF-home values and expected cuRobo FK result independently
        # cross-checked in test_curobo_frames.py::TestPinchCenterFkMatchesUrdf
        # (via yourdfpy) -- this test re-derives the same number with this
        # module's own FK instead of yourdfpy, so the two independent
        # implementations must agree.
        home = [0.0, 1.2, -1.25, 0.0, 0.15, 0.0]
        pos, _quat = self.chain.pinch_center_pose(home)
        expected = np.array([0.395029392, -1.9620870261860546e-06, 0.453423842])
        self.assertTrue(
            np.allclose(pos, expected, atol=1e-3),
            f"pinch_center FK mismatch: urdf_fk={pos} vs cuRobo/yourdfpy={expected}",
        )

    @unittest.skipUnless(_GRIPPER_PROBE_PATH.exists(), "out/lesson_06/gripper_probe.json not present")
    def test_gripper_base_matches_isaac_measured_pose(self):
        # Cross-check against the same Isaac-measured ground truth
        # tools/verify_fk_consistency.py uses, via this module's FK instead of
        # that script's inline one -- both must land within the same 1mm /
        # 0.1deg tolerance the original script enforces.
        probe = json.loads(_GRIPPER_PROBE_PATH.read_text())
        sample = probe["measurements"]["open"]
        joint_values = dict(zip(probe["joint_names"], sample["joint_position"]))
        pos, quat = self.chain.link_pose(joint_values, "gripper_base")

        isaac = sample["bodies"]["gripper_base"]
        isaac_pos = np.array(isaac["position"])
        isaac_rot = _quat_xyzw_to_matrix(isaac["quaternion_xyzw"])
        translation_error = float(np.linalg.norm(pos - isaac_pos))
        our_rot = _quat_xyzw_to_matrix(quat)
        cos_angle = np.clip((np.trace(our_rot.T @ isaac_rot) - 1) / 2, -1, 1)
        rotation_error_rad = float(math.acos(cos_angle))

        self.assertLess(translation_error, 0.001, f"translation error {translation_error} m")
        self.assertLess(rotation_error_rad, math.radians(0.1), f"rotation error {math.degrees(rotation_error_rad)} deg")

    def test_unknown_target_link_raises(self):
        with self.assertRaises(ValueError):
            self.chain.link_pose({}, "not_a_real_link")


class TestMatrixQuaternionRoundtrip(unittest.TestCase):
    def test_identity(self):
        quat = matrix_to_quaternion_xyzw(np.eye(3))
        self.assertTrue(np.allclose(quat, [0, 0, 0, 1]))

    def test_90deg_about_z(self):
        c, s = math.cos(math.pi / 2), math.sin(math.pi / 2)
        rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        quat = matrix_to_quaternion_xyzw(rot)
        expected = np.array([0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
        self.assertTrue(np.allclose(quat, expected, atol=1e-6) or np.allclose(quat, -expected, atol=1e-6))


class TestClassifyApproachDirection(unittest.TestCase):
    def test_identity_quaternion_points_up(self):
        name, angle_deg = classify_approach_direction([0, 0, 0, 1])
        self.assertEqual(name, "+Z (up)")
        self.assertAlmostEqual(angle_deg, 0.0, places=6)

    def test_180deg_about_x_points_down(self):
        # Rotating local +Z by 180deg about X sends it to -Z ("down").
        q_xyzw = [1.0, 0.0, 0.0, 0.0]
        name, angle_deg = classify_approach_direction(q_xyzw)
        self.assertEqual(name, "-Z (down)")
        self.assertAlmostEqual(angle_deg, 0.0, places=4)

    def test_90deg_about_x_points_left_or_right(self):
        # Rotating local +Z by -90deg about X: rotate_vector_xyzw with
        # q=(sin(-45deg),0,0,cos(-45deg)) sends local +Z to world +Y ("left").
        half = math.pi / 4
        q_xyzw = [-math.sin(half), 0.0, 0.0, math.cos(half)]
        world_vec = rotate_vector_xyzw(q_xyzw, [0, 0, 1])
        name, angle_deg = classify_approach_direction(q_xyzw)
        self.assertLess(angle_deg, 1e-4)
        # sanity: whichever named axis it lands on must match the raw rotation
        self.assertTrue(np.allclose(world_vec, [0, 1, 0], atol=1e-6))
        self.assertEqual(name, "+Y (left)")


class TestQuaternionAngularDistance(unittest.TestCase):
    def test_identical_quaternions_zero_distance(self):
        q = [0.1, 0.2, 0.3, 0.9]
        q = list(np.asarray(q) / np.linalg.norm(q))
        self.assertAlmostEqual(quaternion_angular_distance_xyzw(q, q), 0.0, places=6)

    def test_sign_flip_is_zero_distance(self):
        q = [0.0, 0.0, 0.0, 1.0]
        neg_q = [0.0, 0.0, 0.0, -1.0]
        self.assertAlmostEqual(quaternion_angular_distance_xyzw(q, neg_q), 0.0, places=6)

    def test_90deg_about_z(self):
        identity = [0, 0, 0, 1.0]
        ninety_about_z = [0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
        angle = quaternion_angular_distance_xyzw(identity, ninety_about_z)
        self.assertAlmostEqual(angle, math.pi / 2, places=6)

    def test_180deg_about_x(self):
        identity = [0, 0, 0, 1.0]
        flip = [1.0, 0, 0, 0]
        angle = quaternion_angular_distance_xyzw(identity, flip)
        self.assertAlmostEqual(angle, math.pi, places=6)


if __name__ == "__main__":
    unittest.main()
