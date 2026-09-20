"""Bridge: a 2D grasp-affordance box (src/mpg/schema.AffordanceRegion, from
prompts/grasp_affordance.txt) + a depth image -> the (points_xyz,
task_region_mask) pair src/mpg/grasp_candidates.generate_grasp_candidates
expects. This is the piece that turns "the VLM said don't touch the
functional tip" into an actual 3D mask a grasp candidate can be checked
against, single-view (this workspace has one wrist camera, not ZeroDex's
multi-view rig -- see docs/progress/grasp_motion_progress_report.md
"我們的單相機情況" for why there is no cross-view voting here).

Frame warning, read before wiring this into anything: deproject_region()
(src/mpg/lifting.py) returns points in the CAMERA optical frame (X right,
Y down, Z forward), not world/base frame. generate_grasp_candidates()
assumes a WORLD/base frame (its top-down approach axis is a literal
world -z). The caller of object_points_and_task_region_mask() below MUST
transform its returned points_xyz into the world/base frame (via
src/mpg/scene_bundle.transform_matrix with a known T_base_camera) before
handing them to generate_grasp_candidates -- this module does not do that
transform itself, because camera extrinsics are not always available to a
pure geometry function and silently assuming an identity transform would
be a much worse bug than requiring the caller to do it explicitly.
"""
from __future__ import annotations

import numpy as np

from .lifting import deproject_region
from .schema import AffordanceRegion, PointStatus, pixel_from_norm1000


class AffordanceRegionError(ValueError):
    """Raised when an AffordanceRegion cannot be turned into a 3D mask --
    e.g. it is UNAVAILABLE (the VLM correctly abstained), or the resulting
    region is empty after intersecting with the object mask and depth
    validity. Carries enough to log a real reason, not just "no points"."""

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


def affordance_bbox_to_pixel_mask(
    region: AffordanceRegion, *, width: int, height: int, adapter: str = "new"
) -> np.ndarray:
    """Convert an AffordanceRegion's normalized bbox to a dense (H, W)
    boolean pixel mask, using the SAME pixel_from_norm1000 adapter as
    every other point in this codebase (never a second coordinate
    convention). Raises AffordanceRegionError if region.status is
    UNAVAILABLE -- there is no bbox to convert, and returning an empty or
    all-True mask instead would silently hide that the VLM abstained."""
    if region.status is not PointStatus.LOCALIZED:
        raise AffordanceRegionError(
            f"AffordanceRegion is {region.status.value}, not LOCALIZED "
            f"(reason_codes={region.reason_codes}); there is no bbox to use"
        )
    y1, x1, y2, x2 = region.bbox_yx_norm1000
    x1_px, y1_px = pixel_from_norm1000((y1, x1), width=width, height=height, adapter=adapter)
    x2_px, y2_px = pixel_from_norm1000((y2, x2), width=width, height=height, adapter=adapter)

    mask = np.zeros((height, width), dtype=bool)
    row_lo, row_hi = int(np.floor(min(y1_px, y2_px))), int(np.ceil(max(y1_px, y2_px)))
    col_lo, col_hi = int(np.floor(min(x1_px, x2_px))), int(np.ceil(max(x1_px, x2_px)))
    row_lo, row_hi = max(row_lo, 0), min(row_hi, height - 1)
    col_lo, col_hi = max(col_lo, 0), min(col_hi, width - 1)
    if row_lo > row_hi or col_lo > col_hi:
        raise AffordanceRegionError(f"bbox {region.bbox_yx_norm1000} maps outside the image bounds")
    mask[row_lo : row_hi + 1, col_lo : col_hi + 1] = True
    return mask


def object_points_and_task_region_mask(
    depth_m: np.ndarray,
    k_matrix: list[float],
    *,
    object_pixel_mask: np.ndarray,
    region: AffordanceRegion,
    width: int,
    height: int,
    adapter: str = "new",
) -> tuple[np.ndarray, np.ndarray]:
    """The full bridge: deproject every valid-depth pixel of the object
    (object_pixel_mask -- from GT segmentation, colour segmentation, or
    whatever produced it; this function does not create that mask) into
    camera-frame 3D points, then mark which of those points also fall
    inside the VLM's affordance bbox. Deliberately intersects the bbox
    with the object mask rather than trusting the bbox alone (a single-
    view box commonly includes background/table pixels for a non-
    rectangular object) -- this is the "AND, not OR" design recorded in
    the ZeroDex breakdown doc's recommendation section.

    Returns (points_xyz_cam (N, 3), task_region_mask (N,) bool), ready for
    src/mpg/grasp_candidates.generate_grasp_candidates(points_xyz,
    task_region_mask=...) ONCE points_xyz_cam has been transformed to
    world/base frame by the caller (see module docstring).

    Raises AffordanceRegionError if the object mask has no valid-depth
    pixels at all, or if none of them fall inside the affordance bbox
    (a real, reportable failure -- not silently treated as "grasp
    anywhere on the object").
    """
    if object_pixel_mask.shape != (height, width):
        raise ValueError(
            f"object_pixel_mask shape {object_pixel_mask.shape} must be (height, width)=({height}, {width})"
        )
    bbox_pixel_mask = affordance_bbox_to_pixel_mask(region, width=width, height=height, adapter=adapter)

    points_cam, row_idx, col_idx = deproject_region(depth_m, k_matrix, pixel_mask=object_pixel_mask)
    if points_cam.shape[0] == 0:
        raise AffordanceRegionError("object_pixel_mask has no pixels with valid depth")

    task_region_mask = bbox_pixel_mask[row_idx, col_idx]
    if not np.any(task_region_mask):
        raise AffordanceRegionError(
            "affordance bbox does not overlap any valid-depth object pixel "
            "(bbox intersect object mask is empty)"
        )
    return points_cam, task_region_mask
