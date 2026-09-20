from __future__ import annotations

import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np  # noqa: E402

from mpg.affordance_region import (  # noqa: E402
    AffordanceRegionError,
    affordance_bbox_to_pixel_mask,
    object_points_and_task_region_mask,
)
from mpg.grasp_candidates import generate_grasp_candidates  # noqa: E402
from mpg.schema import AffordanceRegion, PointStatus  # noqa: E402

K = [100.0, 0.0, 29.5, 0.0, 100.0, 19.5, 0.0, 0.0, 1.0]  # fx,0,cx, 0,fy,cy, 0,0,1
WIDTH, HEIGHT = 60, 40


def _norm(px: float, extent_minus_1: int) -> float:
    """Inverse of schema.pixel_from_norm1000's 'new' adapter, for building
    a bbox in normalized coordinates from a known pixel target -- avoids
    hand-picking norm values and hoping they round-trip correctly."""
    return px * 1000.0 / extent_minus_1


def _region(row_lo, col_lo, row_hi, col_hi) -> AffordanceRegion:
    return AffordanceRegion(
        status=PointStatus.LOCALIZED,
        description="test region",
        point_yx_norm1000=(
            _norm((row_lo + row_hi) / 2, HEIGHT - 1), _norm((col_lo + col_hi) / 2, WIDTH - 1),
        ),
        bbox_yx_norm1000=(
            _norm(row_lo, HEIGHT - 1), _norm(col_lo, WIDTH - 1),
            _norm(row_hi, HEIGHT - 1), _norm(col_hi, WIDTH - 1),
        ),
        reason_codes=[],
    )


def _unavailable_region() -> AffordanceRegion:
    return AffordanceRegion(
        status=PointStatus.UNAVAILABLE, description=None, point_yx_norm1000=None,
        bbox_yx_norm1000=None, reason_codes=["OCCLUDED"],
    )


class AffordanceBboxToPixelMaskTest(unittest.TestCase):
    def test_mask_covers_the_requested_pixel_range(self) -> None:
        region = _region(10, 20, 20, 40)
        mask = affordance_bbox_to_pixel_mask(region, width=WIDTH, height=HEIGHT)
        self.assertEqual(mask.shape, (HEIGHT, WIDTH))
        self.assertTrue(mask[15, 30])   # inside
        self.assertFalse(mask[5, 30])   # outside above
        self.assertFalse(mask[25, 30])  # outside below
        self.assertFalse(mask[15, 5])   # outside left

    def test_unavailable_region_raises(self) -> None:
        with self.assertRaises(AffordanceRegionError):
            affordance_bbox_to_pixel_mask(_unavailable_region(), width=WIDTH, height=HEIGHT)


class ObjectPointsAndTaskRegionMaskTest(unittest.TestCase):
    def setUp(self) -> None:
        # A flat "object" 5cm above the table, occupying rows 10-30 / cols
        # 20-40 of the image (a "bottle": rows 10-20 = cap, rows 20-30 =
        # body). Camera is 0.5m above the table, looking straight down, so
        # depth 0.45m everywhere on the object -> world z = 0.05m.
        self.depth = np.zeros((HEIGHT, WIDTH), dtype=float)
        self.object_mask = np.zeros((HEIGHT, WIDTH), dtype=bool)
        self.object_mask[10:30, 20:40] = True
        self.depth[self.object_mask] = 0.45

    def test_bbox_restricted_to_body_excludes_cap_rows(self) -> None:
        body_region = _region(20, 20, 30, 40)  # only the "body" half
        points_cam, task_mask = object_points_and_task_region_mask(
            self.depth, K, object_pixel_mask=self.object_mask, region=body_region,
            width=WIDTH, height=HEIGHT,
        )
        self.assertEqual(points_cam.shape[0], 20 * 20)  # whole object: 20 rows x 20 cols
        self.assertEqual(task_mask.shape[0], points_cam.shape[0])
        # exactly the "body" half (10 rows x 20 cols) should be marked allowed
        self.assertEqual(int(task_mask.sum()), 10 * 20)

    def test_bbox_covering_whole_object_marks_everything_allowed(self) -> None:
        whole_region = _region(10, 20, 30, 40)
        _, task_mask = object_points_and_task_region_mask(
            self.depth, K, object_pixel_mask=self.object_mask, region=whole_region,
            width=WIDTH, height=HEIGHT,
        )
        self.assertTrue(np.all(task_mask))

    def test_bbox_disjoint_from_object_raises(self) -> None:
        far_region = _region(0, 0, 5, 5)  # nowhere near the object mask
        with self.assertRaises(AffordanceRegionError):
            object_points_and_task_region_mask(
                self.depth, K, object_pixel_mask=self.object_mask, region=far_region,
                width=WIDTH, height=HEIGHT,
            )

    def test_unavailable_region_raises(self) -> None:
        with self.assertRaises(AffordanceRegionError):
            object_points_and_task_region_mask(
                self.depth, K, object_pixel_mask=self.object_mask,
                region=_unavailable_region(), width=WIDTH, height=HEIGHT,
            )

    def test_empty_object_mask_raises(self) -> None:
        whole_region = _region(10, 20, 30, 40)
        with self.assertRaises(AffordanceRegionError):
            object_points_and_task_region_mask(
                self.depth, K, object_pixel_mask=np.zeros((HEIGHT, WIDTH), dtype=bool),
                region=whole_region, width=WIDTH, height=HEIGHT,
            )

    def test_wrong_mask_shape_rejected(self) -> None:
        whole_region = _region(10, 20, 30, 40)
        with self.assertRaises(ValueError):
            object_points_and_task_region_mask(
                self.depth, K, object_pixel_mask=np.ones((5, 5), dtype=bool),
                region=whole_region, width=WIDTH, height=HEIGHT,
            )


class EndToEndAffordanceToGraspCandidateTest(unittest.TestCase):
    """The real point of this module: a VLM affordance box, over a depth
    image, produces a task_region_mask that generate_grasp_candidates can
    actually use to pick a grasp on the allowed part of the object only --
    the same "avoid the cap, grasp the body" behavior the ZeroDex
    breakdown doc's recommendation section describes, run end to end on
    synthetic data instead of asserted in prose."""

    def test_full_chain_prefers_the_affordance_restricted_region(self) -> None:
        depth = np.zeros((HEIGHT, WIDTH), dtype=float)
        object_mask = np.zeros((HEIGHT, WIDTH), dtype=bool)
        object_mask[10:30, 20:40] = True   # whole "bottle": cap (10-20) + body (20-30)
        depth[object_mask] = 0.45          # 5cm above the table, see setUp above

        body_region = _region(20, 20, 30, 40)  # VLM says: grasp the body, not the cap
        points_cam, task_mask = object_points_and_task_region_mask(
            depth, K, object_pixel_mask=object_mask, region=body_region,
            width=WIDTH, height=HEIGHT,
        )

        # Camera looks straight down at the table from 0.5m: camera X -> world X,
        # camera Y (image-down) -> world -Y, camera Z (forward/down) -> world -Z,
        # camera origin is 0.5m above table_z_m=0. See module docstring: this
        # transform is the CALLER's job, generate_grasp_candidates never sees
        # camera-frame points.
        camera_height_m = 0.5
        points_world = np.stack(
            [points_cam[:, 0], -points_cam[:, 1], camera_height_m - points_cam[:, 2]], axis=1
        )
        np.testing.assert_allclose(points_world[:, 2], 0.05, atol=1e-9)  # sanity: 5cm above table

        candidates = generate_grasp_candidates(
            points_world, frame_id="world", table_z_m=0.0,
            gripper_max_opening_m=0.10, gripper_min_opening_m=0.0,
            pinch_offset_m=0.125, table_clearance_m=0.01, num_yaws=4,
            pregrasp_standoff_m=0.05, task_region_mask=task_mask,
        )
        self.assertTrue(any(c.accepted for c in candidates))
        best = next(c for c in candidates if c.accepted)
        # The body occupies world y in roughly [-0.10, 0.0] (rows 20-30 of a
        # camera-down projection with cy=19.5); the excluded cap region sits
        # at more negative y (rows 10-20). The accepted candidate's grasp
        # center must fall within the body's own y-range, not the cap's.
        body_points_y = points_world[task_mask][:, 1]
        self.assertGreaterEqual(best.tcp_position_m[1], body_points_y.min() - 1e-6)
        self.assertLessEqual(best.tcp_position_m[1], body_points_y.max() + 1e-6)


if __name__ == "__main__":
    unittest.main()
