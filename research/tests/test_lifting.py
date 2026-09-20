from __future__ import annotations

import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np  # noqa: E402

from mpg.lifting import (  # noqa: E402
    AxisAlignedBox,
    LiftingError,
    apply_transform,
    deproject,
    deproject_point_yx_norm1000,
    deproject_region,
    depth_window_median,
    fit_table_plane_ransac,
    object_height_estimate,
    project_point,
    refine_point,
    waypoint_corridor_height,
)

K_IDENTITY_LIKE = [100.0, 0.0, 50.0, 0.0, 100.0, 40.0, 0.0, 0.0, 1.0]  # fx,0,cx, 0,fy,cy, 0,0,1


class DepthWindowMedianTest(unittest.TestCase):
    def test_median_ignores_zero_pixels(self) -> None:
        depth = np.full((10, 10), 2.0, dtype=np.float32)
        depth[4, 4] = 0.0  # invalid
        depth[4, 5] = 0.0
        sample = depth_window_median(depth, 5, 5, window_k=3)
        self.assertAlmostEqual(sample.depth_m, 2.0)
        self.assertLess(sample.valid_fraction, 1.0)

    def test_window_clips_at_image_edge(self) -> None:
        depth = np.full((10, 10), 1.5, dtype=np.float32)
        sample = depth_window_median(depth, 0, 0, window_k=7)
        self.assertEqual(sample.n_total, 4 * 4)  # clipped to top-left quadrant of the window

    def test_even_window_k_rejected(self) -> None:
        depth = np.full((10, 10), 1.0, dtype=np.float32)
        with self.assertRaises(ValueError):
            depth_window_median(depth, 5, 5, window_k=4)

    def test_point_outside_image_raises(self) -> None:
        depth = np.full((10, 10), 1.0, dtype=np.float32)
        with self.assertRaises(LiftingError):
            depth_window_median(depth, 100, 100, window_k=3)

    def test_all_invalid_window_reports_zero_fraction(self) -> None:
        depth = np.zeros((10, 10), dtype=np.float32)
        sample = depth_window_median(depth, 5, 5, window_k=3)
        self.assertEqual(sample.valid_fraction, 0.0)
        self.assertTrue(np.isnan(sample.depth_m))


class DeprojectTest(unittest.TestCase):
    def test_principal_point_maps_to_zero_xy(self) -> None:
        point = deproject(50, 40, 2.0, K_IDENTITY_LIKE)
        np.testing.assert_allclose(point, [0.0, 0.0, 2.0])

    def test_offset_pixel_scales_by_depth_over_focal_length(self) -> None:
        point = deproject(60, 40, 2.0, K_IDENTITY_LIKE)  # 10px right of cx, fx=100
        np.testing.assert_allclose(point, [0.2, 0.0, 2.0])

    def test_non_positive_focal_length_rejected(self) -> None:
        bad_k = [0.0, 0.0, 50.0, 0.0, 100.0, 40.0, 0.0, 0.0, 1.0]
        with self.assertRaises(ValueError):
            deproject(50, 40, 1.0, bad_k)


class ProjectPointTest(unittest.TestCase):
    def test_inverts_deproject(self) -> None:
        point_cam = deproject(60, 45, 2.0, K_IDENTITY_LIKE)
        x_px, y_px = project_point(point_cam, K_IDENTITY_LIKE)
        self.assertAlmostEqual(x_px, 60.0, places=6)
        self.assertAlmostEqual(y_px, 45.0, places=6)

    def test_point_behind_camera_raises(self) -> None:
        with self.assertRaises(LiftingError):
            project_point(np.array([0.0, 0.0, -1.0]), K_IDENTITY_LIKE)


class DeprojectPointYxNorm1000Test(unittest.TestCase):
    def test_full_pipeline_uses_pixel_from_norm1000(self) -> None:
        depth = np.full((80, 100), 1.0, dtype=np.float32)
        point_cam, sample = deproject_point_yx_norm1000(
            (500, 500), depth, K_IDENTITY_LIKE, width=100, height=80,
            window_k=3, valid_fraction_min=0.5,
        )
        self.assertAlmostEqual(sample.depth_m, 1.0)
        self.assertEqual(point_cam.shape, (3,))

    def test_low_valid_fraction_raises(self) -> None:
        depth = np.zeros((80, 100), dtype=np.float32)
        depth[40, 50] = 1.0  # only the exact center pixel is valid
        with self.assertRaises(LiftingError):
            deproject_point_yx_norm1000(
                (500, 500), depth, K_IDENTITY_LIKE, width=100, height=80,
                window_k=7, valid_fraction_min=0.5,
            )


class DeprojectRegionTest(unittest.TestCase):
    def test_matches_single_pixel_deproject(self) -> None:
        depth = np.full((80, 100), 2.0, dtype=np.float32)
        points, rows, cols = deproject_region(depth, K_IDENTITY_LIKE)
        # pick out the principal point pixel (cx=50, cy=40) and compare
        # against the scalar deproject() for the exact same pixel.
        idx = np.nonzero((rows == 40) & (cols == 50))[0]
        self.assertEqual(idx.size, 1)
        np.testing.assert_allclose(points[idx[0]], deproject(50, 40, 2.0, K_IDENTITY_LIKE))

    def test_returns_every_valid_pixel_with_no_mask(self) -> None:
        depth = np.full((8, 10), 1.5, dtype=np.float32)
        points, rows, cols = deproject_region(depth, K_IDENTITY_LIKE)
        self.assertEqual(points.shape, (80, 3))
        self.assertEqual(rows.shape, (80,))
        self.assertEqual(cols.shape, (80,))

    def test_non_finite_and_non_positive_depth_always_excluded(self) -> None:
        depth = np.full((4, 4), 1.0, dtype=np.float32)
        depth[0, 0] = 0.0
        depth[1, 1] = -1.0
        depth[2, 2] = np.nan
        depth[3, 3] = np.inf
        points, rows, cols = deproject_region(depth, K_IDENTITY_LIKE)
        self.assertEqual(points.shape[0], 16 - 4)
        for r, c in [(0, 0), (1, 1), (2, 2), (3, 3)]:
            self.assertFalse(np.any((rows == r) & (cols == c)))

    def test_pixel_mask_restricts_output(self) -> None:
        depth = np.full((6, 6), 1.0, dtype=np.float32)
        mask = np.zeros((6, 6), dtype=bool)
        mask[2:4, 2:4] = True  # a 2x2 sub-region
        points, rows, cols = deproject_region(depth, K_IDENTITY_LIKE, pixel_mask=mask)
        self.assertEqual(points.shape[0], 4)
        for r, c in zip(rows, cols):
            self.assertTrue(2 <= r < 4 and 2 <= c < 4)

    def test_pixel_mask_wrong_shape_rejected(self) -> None:
        depth = np.full((6, 6), 1.0, dtype=np.float32)
        with self.assertRaises(ValueError):
            deproject_region(depth, K_IDENTITY_LIKE, pixel_mask=np.ones((3, 3), dtype=bool))

    def test_non_positive_focal_length_rejected(self) -> None:
        depth = np.full((4, 4), 1.0, dtype=np.float32)
        bad_k = [0.0, 0.0, 50.0, 0.0, 100.0, 40.0, 0.0, 0.0, 1.0]
        with self.assertRaises(ValueError):
            deproject_region(depth, bad_k)


class ApplyTransformTest(unittest.TestCase):
    def test_identity(self) -> None:
        np.testing.assert_allclose(apply_transform(np.eye(4), [1, 2, 3]), [1, 2, 3])

    def test_translation(self) -> None:
        t = np.eye(4)
        t[:3, 3] = [0.1, 0.2, 0.3]
        np.testing.assert_allclose(apply_transform(t, [0, 0, 0]), [0.1, 0.2, 0.3])

    def test_wrong_shape_rejected(self) -> None:
        with self.assertRaises(ValueError):
            apply_transform(np.eye(3), [0, 0, 0])


class FitTablePlaneRansacTest(unittest.TestCase):
    def _flat_table_points(self, rng, n=200, noise=0.0005):
        xy = rng.uniform(-0.3, 0.3, size=(n, 2))
        z = rng.normal(0.0, noise, size=n)
        return np.column_stack([xy, z])

    def test_recovers_flat_plane_with_up_hint(self) -> None:
        rng = np.random.default_rng(0)
        points = self._flat_table_points(rng)
        plane = fit_table_plane_ransac(
            points, distance_threshold_m=0.005, max_iterations=100,
            min_inlier_fraction=0.5, up_hint=np.array([0, 0, 1.0]), rng=rng,
        )
        np.testing.assert_allclose(np.abs(plane.normal), [0, 0, 1], atol=0.05)
        self.assertGreater(np.dot(plane.normal, [0, 0, 1]), 0)
        self.assertAlmostEqual(plane.offset, 0.0, delta=0.01)

    def test_robust_to_outliers(self) -> None:
        rng = np.random.default_rng(1)
        table = self._flat_table_points(rng, n=180)
        outliers = rng.uniform(-0.3, 0.3, size=(20, 3))
        outliers[:, 2] += 0.5  # clearly off the table plane
        points = np.vstack([table, outliers])
        plane = fit_table_plane_ransac(
            points, distance_threshold_m=0.005, max_iterations=200,
            min_inlier_fraction=0.5, up_hint=np.array([0, 0, 1.0]), rng=rng,
        )
        np.testing.assert_allclose(np.abs(plane.normal), [0, 0, 1], atol=0.05)

    def test_too_few_points_raises(self) -> None:
        with self.assertRaises(LiftingError):
            fit_table_plane_ransac(
                np.array([[0, 0, 0], [1, 0, 0]]),
                distance_threshold_m=0.01, max_iterations=10, min_inlier_fraction=0.5,
            )

    def test_insufficient_inliers_raises(self) -> None:
        rng = np.random.default_rng(2)
        # pure noise, no real plane structure
        points = rng.uniform(-1, 1, size=(50, 3))
        with self.assertRaises(LiftingError):
            fit_table_plane_ransac(
                points, distance_threshold_m=0.001, max_iterations=50,
                min_inlier_fraction=0.9, rng=rng,
            )


class ObjectHeightEstimateTest(unittest.TestCase):
    def test_height_matches_synthetic_bump(self) -> None:
        rng = np.random.default_rng(3)
        table = np.column_stack([rng.uniform(-0.2, 0.2, 100), rng.uniform(-0.2, 0.2, 100), np.zeros(100)])
        bump = np.column_stack([
            rng.uniform(-0.01, 0.01, 20), rng.uniform(-0.01, 0.01, 20), np.full(20, 0.05),
        ])
        points = np.vstack([table, bump])
        plane = fit_table_plane_ransac(
            table, distance_threshold_m=0.005, max_iterations=100,
            min_inlier_fraction=0.5, up_hint=np.array([0, 0, 1.0]), rng=rng,
        )
        height = object_height_estimate(points, plane, grasp_point_xyz=np.array([0, 0, 0.05]), radius_m=0.05)
        self.assertAlmostEqual(height, 0.05, delta=0.01)

    def test_no_nearby_points_raises(self) -> None:
        plane = type("P", (), {"normal": np.array([0, 0, 1.0]), "offset": 0.0})()
        with self.assertRaises(LiftingError):
            object_height_estimate(
                np.array([[1.0, 1.0, 1.0]]), plane, grasp_point_xyz=np.array([0, 0, 0]), radius_m=0.01
            )


class WaypointCorridorHeightTest(unittest.TestCase):
    def test_bump_inside_corridor_is_found(self) -> None:
        table = np.column_stack([np.linspace(-0.3, 0.3, 50), np.zeros(50), np.zeros(50)])
        bump = np.array([[0.0, 0.0, 0.1]])  # right on the segment, tall
        points = np.vstack([table, bump])
        plane = type("P", (), {"normal": np.array([0, 0, 1.0]), "offset": 0.0})()
        height = waypoint_corridor_height(
            points, plane, grasp_xy=np.array([-0.3, 0, 0]), release_xy=np.array([0.3, 0, 0]),
            corridor_r_m=0.05,
        )
        self.assertAlmostEqual(height, 0.1, delta=0.01)

    def test_bump_outside_corridor_is_ignored(self) -> None:
        table = np.column_stack([np.linspace(-0.3, 0.3, 50), np.zeros(50), np.zeros(50)])
        far_bump = np.array([[0.0, 1.0, 0.1]])  # 1m off the corridor
        points = np.vstack([table, far_bump])
        plane = type("P", (), {"normal": np.array([0, 0, 1.0]), "offset": 0.0})()
        height = waypoint_corridor_height(
            points, plane, grasp_xy=np.array([-0.3, 0, 0]), release_xy=np.array([0.3, 0, 0]),
            corridor_r_m=0.05,
        )
        self.assertAlmostEqual(height, 0.0, delta=0.01)

    def test_coincident_endpoints_raise(self) -> None:
        plane = type("P", (), {"normal": np.array([0, 0, 1.0]), "offset": 0.0})()
        with self.assertRaises(LiftingError):
            waypoint_corridor_height(
                np.zeros((5, 3)), plane, grasp_xy=np.array([0, 0, 0]),
                release_xy=np.array([0, 0, 0]), corridor_r_m=0.05,
            )


class RefinePointTest(unittest.TestCase):
    def test_non_colliding_point_is_unchanged(self) -> None:
        box = AxisAlignedBox(half_extent=np.array([0.02, 0.02, 0.02]))
        scene = np.array([[1.0, 1.0, 1.0]])  # far away
        point, refined = refine_point(np.array([0, 0, 0]), box, scene, radius_m=0.04, step_m=0.02)
        np.testing.assert_allclose(point, [0, 0, 0])
        self.assertFalse(refined)

    def test_vertical_offset_preferred_over_lateral(self) -> None:
        box = AxisAlignedBox(half_extent=np.array([0.01, 0.01, 0.01]))
        # a single scene point sitting exactly at the candidate -> collision
        scene = np.array([[0.0, 0.0, 0.0]])
        point, refined = refine_point(np.array([0, 0, 0]), box, scene, radius_m=0.04, step_m=0.02)
        self.assertTrue(refined)
        # vertical-first search: x and y should be unchanged
        self.assertAlmostEqual(point[0], 0.0)
        self.assertAlmostEqual(point[1], 0.0)
        self.assertNotAlmostEqual(point[2], 0.0)

    def test_fully_surrounded_box_raises(self) -> None:
        box = AxisAlignedBox(half_extent=np.array([0.005, 0.005, 0.005]))
        # Dense grid of points filling the whole search radius so every
        # candidate offset collides with something.
        grid = np.arange(-0.04, 0.041, 0.01)
        xx, yy, zz = np.meshgrid(grid, grid, grid)
        scene = np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])
        with self.assertRaises(LiftingError):
            refine_point(np.array([0, 0, 0]), box, scene, radius_m=0.04, step_m=0.02)


class AxisAlignedBoxTest(unittest.TestCase):
    def test_half_extent_from_points(self) -> None:
        points = np.array([[-1, -2, -3], [1, 2, 3]])
        box = AxisAlignedBox.from_points(points)
        np.testing.assert_allclose(box.half_extent, [1, 2, 3])

    def test_empty_points_raises(self) -> None:
        with self.assertRaises(LiftingError):
            AxisAlignedBox.from_points(np.zeros((0, 3)))


if __name__ == "__main__":
    unittest.main()
