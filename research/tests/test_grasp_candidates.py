from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np  # noqa: E402

from mpg.grasp_candidates import (  # noqa: E402
    GraspCandidate,
    generate_grasp_candidates,
    quat_conjugate,
    quat_multiply,
    quat_normalize,
    quat_rotate_vector,
    se3_compose,
    se3_inverse,
    top_down_grasp_quaternion,
)
from mpg.grasp_contract import (  # noqa: E402
    GraspContractError,
    candidate_to_dict,
    read_candidates_json,
    validate_candidate_dict,
    write_candidates_json,
)

IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])


class QuaternionAlgebraTest(unittest.TestCase):
    def test_identity_quaternion_leaves_vector_unchanged(self) -> None:
        v = np.array([1.0, 2.0, 3.0])
        out = quat_rotate_vector(IDENTITY_QUAT, v)
        np.testing.assert_allclose(out, v, atol=1e-12)

    def test_multiply_by_conjugate_is_identity(self) -> None:
        q = quat_normalize(np.array([0.1, 0.4, -0.2, 0.9]))
        product = quat_multiply(q, quat_conjugate(q))
        np.testing.assert_allclose(product, IDENTITY_QUAT, atol=1e-9)

    def test_90deg_z_rotation_maps_x_to_y(self) -> None:
        from mpg.grasp_candidates import quat_from_axis_angle
        q = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), math.pi / 2)
        out = quat_rotate_vector(q, np.array([1.0, 0.0, 0.0]))
        np.testing.assert_allclose(out, [0.0, 1.0, 0.0], atol=1e-9)

    def test_180deg_x_rotation_maps_z_to_negative_z(self) -> None:
        from mpg.grasp_candidates import quat_from_axis_angle
        q = quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), math.pi)
        out = quat_rotate_vector(q, np.array([0.0, 0.0, 1.0]))
        np.testing.assert_allclose(out, [0.0, 0.0, -1.0], atol=1e-9)
        # and x is preserved by a rotation about the x axis
        out_x = quat_rotate_vector(q, np.array([1.0, 0.0, 0.0]))
        np.testing.assert_allclose(out_x, [1.0, 0.0, 0.0], atol=1e-9)

    def test_quaternion_composition_order_matches_matrix_convention(self) -> None:
        from mpg.grasp_candidates import quat_from_axis_angle
        # Rotate 90deg about z, then 90deg about the (now-rotated) local x is NOT the
        # same as applying them in world frame via left-multiplication; check the
        # documented convention instead: q_outer * q_inner rotates by q_inner first.
        q_z = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), math.pi / 2)
        q_x = quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), math.pi / 2)
        combined = quat_multiply(q_z, q_x)
        v = np.array([0.0, 0.0, 1.0])
        # apply q_x first, then q_z, matching the documented v' = R_outer @ R_inner @ v
        expected = quat_rotate_vector(q_z, quat_rotate_vector(q_x, v))
        actual = quat_rotate_vector(combined, v)
        np.testing.assert_allclose(actual, expected, atol=1e-9)


class Se3ComposeInverseTest(unittest.TestCase):
    def test_compose_then_inverse_round_trips_to_identity(self) -> None:
        from mpg.grasp_candidates import quat_from_axis_angle
        t = np.array([0.3, -0.1, 0.05])
        q = quat_from_axis_angle(np.array([0.2, 0.7, 0.1]), 1.1)
        t_inv, q_inv = se3_inverse(t, q)
        t_id, q_id = se3_compose(t, q, t_inv, q_inv)
        np.testing.assert_allclose(t_id, [0.0, 0.0, 0.0], atol=1e-9)
        np.testing.assert_allclose(np.abs(q_id), [0, 0, 0, 1], atol=1e-9)

    def test_compose_applies_second_transform_in_first_frame(self) -> None:
        # T1: pure translation along x by 1m. T2: pure translation along x by 0.5m.
        # T1 * T2 should translate the origin by 1.5m along x.
        t1, q1 = np.array([1.0, 0.0, 0.0]), IDENTITY_QUAT
        t2, q2 = np.array([0.5, 0.0, 0.0]), IDENTITY_QUAT
        t, q = se3_compose(t1, q1, t2, q2)
        np.testing.assert_allclose(t, [1.5, 0.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(q, IDENTITY_QUAT, atol=1e-12)


class TopDownGraspQuaternionTest(unittest.TestCase):
    def test_approach_axis_always_points_world_negative_z(self) -> None:
        for yaw in np.linspace(0, 2 * math.pi, 9):
            q = top_down_grasp_quaternion(float(yaw))
            approach = quat_rotate_vector(q, np.array([0.0, 0.0, 1.0]))
            np.testing.assert_allclose(approach, [0.0, 0.0, -1.0], atol=1e-9)

    def test_yaw_rotates_closing_axis_in_world_xy(self) -> None:
        # local +x should land at world (cos(yaw), sin(yaw), ~0) after accounting for
        # the flip; check yaw=0 and yaw=pi/2 explicitly against the actual convention.
        q0 = top_down_grasp_quaternion(0.0)
        x0 = quat_rotate_vector(q0, np.array([1.0, 0.0, 0.0]))
        q90 = top_down_grasp_quaternion(math.pi / 2)
        x90 = quat_rotate_vector(q90, np.array([1.0, 0.0, 0.0]))
        # the two closing-axis directions must be perpendicular in the world xy plane
        self.assertAlmostEqual(float(x0[:2] @ x90[:2]), 0.0, places=9)


def _rectangular_plate_points(half_x: float, half_y: float, z: float, n_per_side: int) -> np.ndarray:
    xs = np.linspace(-half_x, half_x, n_per_side)
    ys = np.linspace(-half_y, half_y, n_per_side)
    grid_x, grid_y = np.meshgrid(xs, ys)
    points = np.stack([grid_x.ravel(), grid_y.ravel(), np.full(grid_x.size, z)], axis=1)
    return points


class GenerateGraspCandidatesTest(unittest.TestCase):
    def setUp(self) -> None:
        # A 0.04m (x) x 0.02m (y) plate, top surface at z=0.05, table at z=0.
        self.points = _rectangular_plate_points(0.02, 0.01, 0.05, n_per_side=9)
        self.common = dict(
            frame_id="world", table_z_m=0.0, pinch_offset_m=0.125,
            table_clearance_m=0.01, num_yaws=2, pregrasp_standoff_m=0.05,
        )

    def test_narrow_axis_accepted_wide_axis_rejected_for_width(self) -> None:
        candidates = generate_grasp_candidates(
            self.points, gripper_max_opening_m=0.03, gripper_min_opening_m=0.0,
            **self.common,
        )
        self.assertEqual(len(candidates), 2)
        by_yaw = {round(c.yaw_rad, 6): c for c in candidates}
        wide = by_yaw[0.0]  # closing along x: spans ~0.04m > 0.03m max opening
        narrow = by_yaw[round(math.pi / 2, 6)]  # closing along y: spans ~0.02m
        self.assertIn("WIDTH_EXCEEDED", wide.rejection_reasons)
        self.assertFalse(wide.accepted)
        self.assertEqual(narrow.rejection_reasons, ())
        self.assertTrue(narrow.accepted)
        self.assertAlmostEqual(narrow.opening_width_m, 0.02, places=6)

    def test_best_candidate_prefers_most_closing_margin_not_least(self) -> None:
        # With both yaws accepted (a generous max opening), the axis with the
        # SMALLER opening width (more margin left before the gripper's max) must
        # sort first -- picking the candidate closest to the gripper's limit as
        # "best" would be the wrong, riskier choice.
        candidates = generate_grasp_candidates(
            self.points, gripper_max_opening_m=0.10, gripper_min_opening_m=0.0,
            **self.common,
        )
        self.assertTrue(all(c.accepted for c in candidates))
        self.assertLess(candidates[0].opening_width_m, candidates[1].opening_width_m)
        self.assertAlmostEqual(candidates[0].yaw_rad, math.pi / 2, places=6)

    def test_accepted_candidates_sorted_before_rejected(self) -> None:
        candidates = generate_grasp_candidates(
            self.points, gripper_max_opening_m=0.03, gripper_min_opening_m=0.0,
            **self.common,
        )
        accepted_flags = [c.accepted for c in candidates]
        # once a False appears, no True may follow
        self.assertNotIn((False, True), zip(accepted_flags, accepted_flags[1:]))

    def test_table_clearance_rejection(self) -> None:
        common = dict(self.common)
        common["table_clearance_m"] = 0.10  # plate top at z=0.05 < table_z(0)+0.10
        candidates = generate_grasp_candidates(
            self.points, gripper_max_opening_m=0.03, gripper_min_opening_m=0.0, **common,
        )
        for c in candidates:
            self.assertIn("TABLE_CLEARANCE", c.rejection_reasons)

    def test_empty_task_region_mask_produces_explicit_reason(self) -> None:
        mask = np.zeros(self.points.shape[0], dtype=bool)
        candidates = generate_grasp_candidates(
            self.points, gripper_max_opening_m=0.03, gripper_min_opening_m=0.0,
            task_region_mask=mask, **self.common,
        )
        for c in candidates:
            self.assertEqual(c.rejection_reasons, ("EMPTY_TASK_REGION",))

    def test_off_center_task_region_shifts_grasp_center(self) -> None:
        # Restrict the task region to only the +x half of the plate; the accepted
        # (narrow-axis) candidate's grasp center must follow that half, not the
        # whole plate's centroid.
        mask = self.points[:, 0] > 0.0
        candidates = generate_grasp_candidates(
            self.points, gripper_max_opening_m=0.03, gripper_min_opening_m=0.0,
            task_region_mask=mask, **self.common,
        )
        narrow = next(c for c in candidates if math.isclose(c.yaw_rad, math.pi / 2, abs_tol=1e-6))
        self.assertTrue(narrow.accepted)
        self.assertGreater(narrow.tcp_position_m[0], 0.0)

    def test_region_midpoint_height_mode_uses_vertical_center_not_top(self) -> None:
        # A 3D cube-like region spanning z in [0.0, 0.035] (matching the real S2 cube):
        # region_midpoint must place the pinch point at z=0.0175 (the cube's actual
        # verified MTC grasp height), not at the top surface z=0.035.
        half = 0.0175
        xs = np.linspace(-half, half, 4)
        zs = np.linspace(0.0, 0.035, 4)
        gx, gz = np.meshgrid(xs, zs)
        cube_side = np.stack([gx.ravel(), np.zeros(gx.size), gz.ravel()], axis=1)
        common = dict(self.common)
        common["table_clearance_m"] = 0.005
        candidates_mid = generate_grasp_candidates(
            cube_side, gripper_max_opening_m=0.05, gripper_min_opening_m=0.0,
            grasp_height_mode="region_midpoint", **common,
        )
        candidates_top = generate_grasp_candidates(
            cube_side, gripper_max_opening_m=0.05, gripper_min_opening_m=0.0,
            grasp_height_mode="region_top", **common,
        )
        self.assertAlmostEqual(candidates_mid[0].tcp_position_m[2], 0.0175, places=6)
        self.assertAlmostEqual(candidates_top[0].tcp_position_m[2], 0.035, places=6)

    def test_gripper_base_position_offset_from_tcp_along_approach_axis(self) -> None:
        candidates = generate_grasp_candidates(
            self.points, gripper_max_opening_m=0.03, gripper_min_opening_m=0.0,
            **self.common,
        )
        c = next(x for x in candidates if x.accepted)
        tcp = np.array(c.tcp_position_m)
        base = np.array(c.gripper_base_position_m)
        # approach axis is world -z, so gripper_base sits ABOVE tcp by pinch_offset_m
        np.testing.assert_allclose(base - tcp, [0.0, 0.0, self.common["pinch_offset_m"]], atol=1e-9)

    def test_rejects_malformed_points_shape(self) -> None:
        with self.assertRaises(ValueError):
            generate_grasp_candidates(
                np.zeros((5, 2)), gripper_max_opening_m=0.03, gripper_min_opening_m=0.0,
                **self.common,
            )

    def test_rejects_non_finite_points(self) -> None:
        bad = self.points.copy()
        bad[0, 0] = np.nan
        with self.assertRaises(ValueError):
            generate_grasp_candidates(
                bad, gripper_max_opening_m=0.03, gripper_min_opening_m=0.0, **self.common,
            )


class GraspContractTest(unittest.TestCase):
    def setUp(self) -> None:
        points = _rectangular_plate_points(0.02, 0.01, 0.05, n_per_side=9)
        self.candidates = generate_grasp_candidates(
            points, frame_id="world", table_z_m=0.0, gripper_max_opening_m=0.03,
            gripper_min_opening_m=0.0, pinch_offset_m=0.125, table_clearance_m=0.01,
            num_yaws=2, pregrasp_standoff_m=0.05,
        )

    def test_accepted_and_rejected_candidates_both_validate(self) -> None:
        for c in self.candidates:
            row = candidate_to_dict(c, scene_id="unit_test_scene")
            validate_candidate_dict(row)  # must not raise

    def test_accepted_candidate_has_no_rejection_reasons(self) -> None:
        accepted = next(c for c in self.candidates if c.accepted)
        row = candidate_to_dict(accepted, scene_id="unit_test_scene")
        self.assertEqual(row["rejection_reasons"], [])
        self.assertTrue(row["accepted"])

    def test_rejected_candidate_carries_a_reason(self) -> None:
        rejected = next(c for c in self.candidates if not c.accepted)
        row = candidate_to_dict(rejected, scene_id="unit_test_scene")
        self.assertGreater(len(row["rejection_reasons"]), 0)

    def test_invariant_violation_is_rejected_by_validator(self) -> None:
        row = candidate_to_dict(self.candidates[0], scene_id="s")
        row["accepted"] = True
        row["rejection_reasons"] = ["WIDTH_EXCEEDED"]
        with self.assertRaises(GraspContractError):
            validate_candidate_dict(row)

    def test_unknown_rejection_reason_is_rejected_by_validator(self) -> None:
        row = candidate_to_dict(self.candidates[0], scene_id="s")
        row["accepted"] = False
        row["rejection_reasons"] = ["NOT_A_REAL_REASON"]
        with self.assertRaises(GraspContractError):
            validate_candidate_dict(row)

    def test_write_then_read_round_trip(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "candidates.json"
            write_candidates_json(path, self.candidates, scene_id="unit_test_scene")
            rows = read_candidates_json(path)
            self.assertEqual(len(rows), len(self.candidates))
            raw = json.loads(path.read_text())
            self.assertEqual(raw["scene_id"], "unit_test_scene")


if __name__ == "__main__":
    unittest.main()
