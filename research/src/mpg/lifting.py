"""Phase 3: single-view RGB-D lifting to 3D + collision-aware refinement
(AGENTS.md Sec 5, Phase 3). Offline, pure numpy — no Open3D (this
environment cannot install new packages; see src/mpg/vlm/gemini.py's
docstring for why). The point-cloud screenshot AGENTS.md asks for is
therefore NOT implemented here; render_plan_overlay in viz.py covers the
2D side only. Every threshold lives in configs/default.yaml, not here
(coding standard: no magic numbers), and several are explicitly marked
UNVERIFIED there — this module treats them as inputs, never invents its
own default.

Every function here is designed to be unit-testable against a synthetic
depth image, per AGENTS.md Phase 3 task 1 ("Unit-test this; it is the
most common bug") — none of it has been run against a real captured
scene, because none exists yet (Phase 1 capture is still blocked on
physical scene setup). Treat every number this module produces on real
data as UNVERIFIED until it has been.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .schema import pixel_from_norm1000


class LiftingError(ValueError):
    """A geometric step could not be computed. Carries enough to write one
    row of a Phase 3 failures list without re-deriving the reason."""

    def __init__(self, step: str, reason: str):
        super().__init__(f"[{step}] {reason}")
        self.step = step
        self.reason = reason


@dataclass(frozen=True)
class DepthSample:
    depth_m: float
    valid_fraction: float
    n_valid: int
    n_total: int


def depth_window_median(
    depth_m: np.ndarray, x_px: float, y_px: float, *, window_k: int
) -> DepthSample:
    """Median of valid (finite, > 0) depth in a window_k x window_k window
    centered on (x_px, y_px), clipped to the image bounds. AGENTS.md Phase
    3 task 2: "Report the valid fraction. Fail the point if the valid
    fraction is below the threshold" — this function only measures; the
    caller decides pass/fail against configs/default.yaml
    lifting.depth.valid_fraction_min.
    """
    if window_k < 1 or window_k % 2 == 0:
        raise ValueError(f"window_k must be a positive odd integer, got {window_k}")
    h, w = depth_m.shape[:2]
    cx, cy = int(round(x_px)), int(round(y_px))
    half = window_k // 2
    x0, x1 = max(0, cx - half), min(w, cx + half + 1)
    y0, y1 = max(0, cy - half), min(h, cy + half + 1)
    if x0 >= x1 or y0 >= y1:
        raise LiftingError("depth_window", f"point ({x_px}, {y_px}) is outside the image")

    window = depth_m[y0:y1, x0:x1]
    n_total = window.size
    valid_mask = np.isfinite(window) & (window > 0)
    n_valid = int(valid_mask.sum())
    valid_fraction = n_valid / n_total if n_total else 0.0
    median = float(np.median(window[valid_mask])) if n_valid else float("nan")
    return DepthSample(depth_m=median, valid_fraction=valid_fraction, n_valid=n_valid, n_total=n_total)


def deproject(x_px: float, y_px: float, depth_m: float, k_matrix: list[float]) -> np.ndarray:
    """Pinhole deprojection to the camera optical frame: X right, Y down,
    Z forward (matches camera_info.json's frame_id convention already
    used by src/mpg/scene_bundle.py). k_matrix is the row-major 3x3
    CameraInfo K, as stored by scripts/capture_scene.py."""
    if len(k_matrix) != 9:
        raise ValueError("k_matrix must have 9 elements (row-major 3x3)")
    fx, _, cx = k_matrix[0], k_matrix[1], k_matrix[2]
    _, fy, cy = k_matrix[3], k_matrix[4], k_matrix[5]
    if fx <= 0 or fy <= 0:
        raise ValueError(f"fx/fy must be positive, got fx={fx} fy={fy}")
    x = (x_px - cx) * depth_m / fx
    y = (y_px - cy) * depth_m / fy
    return np.array([x, y, depth_m], dtype=float)


def deproject_point_yx_norm1000(
    point_yx_norm1000: tuple[float, float],
    depth_m: np.ndarray,
    k_matrix: list[float],
    *,
    width: int,
    height: int,
    window_k: int,
    valid_fraction_min: float,
    adapter: str = "new",
) -> tuple[np.ndarray, DepthSample]:
    """The full pixel -> camera-frame-3D step for one plan point: convert
    the normalized point to pixels (src/mpg/schema.pixel_from_norm1000,
    reused rather than reimplemented — AGENTS.md 0.1 item 4's (W-1)/(H-1)
    contract lives in exactly one place), sample the depth window at that
    single pixel, and deproject using the window's median depth."""
    x_px, y_px = pixel_from_norm1000(
        point_yx_norm1000, width=width, height=height, adapter=adapter
    )
    sample = depth_window_median(depth_m, x_px, y_px, window_k=window_k)
    if sample.valid_fraction < valid_fraction_min:
        raise LiftingError(
            "deproject",
            f"valid_fraction={sample.valid_fraction:.2f} < min={valid_fraction_min}",
        )
    point_cam = deproject(x_px, y_px, sample.depth_m, k_matrix)
    return point_cam, sample


def deproject_region(
    depth_m: np.ndarray, k_matrix: list[float], *, pixel_mask: np.ndarray | None = None
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Vectorized deproject() over many pixels at once, for a whole region
    rather than one plan point. Reuses the exact same pinhole model as
    deproject() (same fx/fy/cx/cy extraction) rather than a second
    implementation -- this is the array-shaped sibling of that function,
    not an independent one.

    pixel_mask: optional (H, W) boolean array restricting which pixels are
    considered at all (e.g. an affordance bbox intersected with an object
    segmentation mask, built by the caller -- see
    src/mpg/affordance_region.py). If None, every pixel in depth_m is a
    candidate. Pixels with non-finite or non-positive depth are always
    excluded, regardless of pixel_mask, the same validity rule deproject()
    and depth_window_median() apply elsewhere in this module.

    Returns (points_cam (N, 3), row_idx (N,), col_idx (N,)) for the N
    pixels that passed both the mask and the depth validity check, so a
    caller can trace any returned 3D point back to its source pixel.
    """
    if len(k_matrix) != 9:
        raise ValueError("k_matrix must have 9 elements (row-major 3x3)")
    fx, _, cx = k_matrix[0], k_matrix[1], k_matrix[2]
    _, fy, cy = k_matrix[3], k_matrix[4], k_matrix[5]
    if fx <= 0 or fy <= 0:
        raise ValueError(f"fx/fy must be positive, got fx={fx} fy={fy}")
    depth_m = np.asarray(depth_m, dtype=float)
    if depth_m.ndim != 2:
        raise ValueError(f"depth_m must be (H, W), got shape {depth_m.shape}")
    height, width = depth_m.shape

    valid = np.isfinite(depth_m) & (depth_m > 0)
    if pixel_mask is not None:
        pixel_mask = np.asarray(pixel_mask, dtype=bool)
        if pixel_mask.shape != depth_m.shape:
            raise ValueError(
                f"pixel_mask shape {pixel_mask.shape} must match depth_m shape {depth_m.shape}"
            )
        valid = valid & pixel_mask

    row_idx, col_idx = np.nonzero(valid)
    z = depth_m[row_idx, col_idx]
    x = (col_idx.astype(float) - cx) * z / fx
    y = (row_idx.astype(float) - cy) * z / fy
    points_cam = np.stack([x, y, z], axis=1)
    return points_cam, row_idx, col_idx


def project_point(point_xyz_cam: np.ndarray, k_matrix: list[float]) -> tuple[float, float]:
    """Pinhole projection, the inverse of deproject(): a camera-optical-frame
    3D point -> pixel (x_px, y_px). Used to draw things whose position is
    known in 3D (e.g. a grasp candidate's world-frame pose, transformed into
    the camera frame via apply_transform with the inverse extrinsics) back
    onto the RGB image for a static overlay -- no live 3D renderer needed."""
    if len(k_matrix) != 9:
        raise ValueError("k_matrix must have 9 elements (row-major 3x3)")
    fx, _, cx = k_matrix[0], k_matrix[1], k_matrix[2]
    _, fy, cy = k_matrix[3], k_matrix[4], k_matrix[5]
    x, y, z = point_xyz_cam
    if z <= 0:
        raise LiftingError("project_point", f"point is behind or on the camera plane, z={z}")
    x_px = fx * (x / z) + cx
    y_px = fy * (y / z) + cy
    return float(x_px), float(y_px)


def apply_transform(t_matrix_4x4: np.ndarray, point_xyz: np.ndarray) -> np.ndarray:
    """Apply a homogeneous 4x4 transform (e.g. T_base_camera, as validated
    by src/mpg/scene_bundle.validate_scene) to one point."""
    t = np.asarray(t_matrix_4x4, dtype=float)
    if t.shape != (4, 4):
        raise ValueError(f"t_matrix_4x4 must be 4x4, got {t.shape}")
    homogeneous = np.append(np.asarray(point_xyz, dtype=float), 1.0)
    return (t @ homogeneous)[:3]


@dataclass(frozen=True)
class PlaneFit:
    normal: np.ndarray  # unit vector, sign chosen per up_hint (see fit_table_plane_ransac)
    offset: float  # plane: normal . x = offset
    inlier_mask: np.ndarray

    def height_above(self, point_xyz: np.ndarray) -> float:
        """Signed distance from the plane along its normal — positive is
        the side the normal points to."""
        return float(np.dot(self.normal, point_xyz) - self.offset)


def fit_table_plane_ransac(
    points_xyz: np.ndarray,
    *,
    distance_threshold_m: float,
    max_iterations: int,
    min_inlier_fraction: float,
    up_hint: np.ndarray | None = None,
    rng: np.random.Generator | None = None,
) -> PlaneFit:
    """RANSAC plane fit for the table surface.

    up_hint: a unit vector this function will orient the fitted normal
    towards (dot(normal, up_hint) >= 0). AGENTS.md 0.1 item 6 / Sec 4.4
    of docs/data_and_evaluation.md: "Table normal 朝向由已確認的重力／base
    定義判定，不把 camera z 當上方" — so up_hint MUST come from a confirmed
    base-frame Z axis transformed into the camera frame, never assumed.
    If up_hint is None (no base transform available — the common case
    until AGENTS.md's base_link TF question is resolved), the returned
    normal's sign is arbitrary and MUST be treated as UNKNOWN orientation
    by the caller, not silently assumed to point "up".
    """
    points = np.asarray(points_xyz, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points_xyz must be (N, 3), got {points.shape}")
    n = points.shape[0]
    if n < 3:
        raise LiftingError("table_ransac", f"need at least 3 points, got {n}")

    rng = rng or np.random.default_rng()
    best_inliers: np.ndarray | None = None
    best_count = -1

    for _ in range(max_iterations):
        sample_idx = rng.choice(n, size=3, replace=False)
        p0, p1, p2 = points[sample_idx]
        v1, v2 = p1 - p0, p2 - p0
        normal = np.cross(v1, v2)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-9:
            continue  # degenerate (near-collinear) sample
        normal = normal / norm_len
        offset = float(np.dot(normal, p0))
        distances = np.abs(points @ normal - offset)
        inliers = distances <= distance_threshold_m
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_inliers = inliers
            best_normal, best_offset = normal, offset

    if best_inliers is None or best_count / n < min_inlier_fraction:
        raise LiftingError(
            "table_ransac",
            f"best inlier fraction {best_count / n if n else 0:.2f} < "
            f"min={min_inlier_fraction} after {max_iterations} iterations",
        )

    # Refit on all inliers for a less noisy normal (mean-centered SVD).
    inlier_points = points[best_inliers]
    centroid = inlier_points.mean(axis=0)
    _, _, vt = np.linalg.svd(inlier_points - centroid, full_matrices=False)
    normal = vt[-1]
    normal = normal / np.linalg.norm(normal)
    offset = float(np.dot(normal, centroid))

    if up_hint is not None:
        up_hint = np.asarray(up_hint, dtype=float)
        if np.dot(normal, up_hint) < 0:
            normal, offset = -normal, -offset

    return PlaneFit(normal=normal, offset=offset, inlier_mask=best_inliers)


def object_height_estimate(
    points_xyz: np.ndarray,
    plane: PlaneFit,
    grasp_point_xyz: np.ndarray,
    *,
    radius_m: float,
) -> float:
    """Height of the carried object above the table: max plane-normal
    height among scene points within radius_m of the grasp point, minus
    the table plane (i.e., 0 if nothing rises above it). AGENTS.md Phase 3
    task 4: "from the point cloud around the grasp point, top of object
    minus table plane."

    plane.normal orientation matters here — if it came from
    fit_table_plane_ransac with up_hint=None, this "height" is only valid
    up to an unknown sign and the caller must treat it as UNKNOWN, not use
    it downstream.
    """
    points = np.asarray(points_xyz, dtype=float)
    grasp = np.asarray(grasp_point_xyz, dtype=float)
    nearby_mask = np.linalg.norm(points - grasp, axis=1) <= radius_m
    if not nearby_mask.any():
        raise LiftingError("object_height", f"no scene points within {radius_m} m of grasp point")
    nearby = points[nearby_mask]
    heights = nearby @ plane.normal - plane.offset
    return float(max(0.0, heights.max()))


def waypoint_corridor_height(
    points_xyz: np.ndarray,
    plane: PlaneFit,
    grasp_xy: np.ndarray,
    release_xy: np.ndarray,
    *,
    corridor_r_m: float,
) -> float:
    """Max plane-normal height of scene points within corridor_r_m of the
    grasp->release segment (projected onto the plane). AGENTS.md Phase 3
    task 4, WAYPOINT rule: "Z = max scene height inside a corridor...
    along the segment grasp->release." The historical grasp_xy/release_xy
    names accept three-dimensional endpoints in the point cloud frame.
    Both endpoints are projected to the plane, including on tilted tables.
    """
    points = np.asarray(points_xyz, dtype=float)
    a = np.asarray(grasp_xy, dtype=float)
    b = np.asarray(release_xy, dtype=float)
    if a.shape != (3,) or b.shape != (3,):
        raise ValueError("corridor endpoints must be XYZ vectors in the point-cloud frame")
    a = a - (np.dot(a, plane.normal) - plane.offset) * plane.normal
    b = b - (np.dot(b, plane.normal) - plane.offset) * plane.normal
    seg = b - a
    seg_len_sq = float(np.dot(seg, seg))
    if seg_len_sq < 1e-12:
        raise LiftingError("waypoint_corridor", "grasp and release XY coincide; no corridor")

    # Project each point onto the segment's plane-tangent XY (drop the
    # plane-normal component), then find perpendicular distance to the
    # segment line (clamped to the segment, not the infinite line).
    xy = points - np.outer(points @ plane.normal - plane.offset, plane.normal)
    rel = xy - a
    t = np.clip((rel @ seg) / seg_len_sq, 0.0, 1.0)
    closest = np.outer(t, seg) + a
    perp_dist = np.linalg.norm(xy - closest, axis=1)
    in_corridor = perp_dist <= corridor_r_m
    if not in_corridor.any():
        raise LiftingError("waypoint_corridor", f"no scene points within {corridor_r_m} m of the corridor")
    heights = points[in_corridor] @ plane.normal - plane.offset
    return float(max(0.0, heights.max()))


@dataclass(frozen=True)
class AxisAlignedBox:
    half_extent: np.ndarray  # (3,), in the SAME frame as the points used to build it

    @staticmethod
    def from_points(points_xyz: np.ndarray) -> "AxisAlignedBox":
        points = np.asarray(points_xyz, dtype=float)
        if points.shape[0] == 0:
            raise LiftingError("object_box", "no points to bound")
        extent = (points.max(axis=0) - points.min(axis=0)) / 2.0
        return AxisAlignedBox(half_extent=extent)


def _collides(candidate_center: np.ndarray, box: AxisAlignedBox, scene_points: np.ndarray) -> bool:
    if scene_points.shape[0] == 0:
        return False
    lower = candidate_center - box.half_extent
    upper = candidate_center + box.half_extent
    inside = np.all((scene_points >= lower) & (scene_points <= upper), axis=1)
    return bool(inside.any())


def refine_point(
    candidate_xyz: np.ndarray,
    box: AxisAlignedBox,
    scene_points_xyz: np.ndarray,
    *,
    radius_m: float,
    step_m: float,
    support_plane: PlaneFit | None = None,
) -> tuple[np.ndarray, bool]:
    """ZeroDex Eq. 12 / App D.1, simplified (AGENTS.md Phase 3 task 5): if
    the candidate collides with scene_points_xyz (already excluding the
    object's own points — the caller's responsibility), search a local
    grid of +/-radius_m at step_m resolution, vertical offsets (z, this
    module's third axis) before lateral, and take the nearest
    collision-free point. Returns (point, refined) where refined=False
    means the input was already collision-free — NOT the same as
    "refine_failed", which the caller signals by catching LiftingError.
    """
    candidate = np.asarray(candidate_xyz, dtype=float)
    scene_points = np.asarray(scene_points_xyz, dtype=float)
    if not np.isfinite(radius_m) or radius_m < 0 or not np.isfinite(step_m) or step_m <= 0:
        raise ValueError("radius must be finite/nonnegative and step finite/positive")
    if candidate.shape != (3,) or not np.isfinite(candidate).all():
        raise ValueError("candidate must be a finite XYZ vector")
    if scene_points.ndim != 2 or scene_points.shape[1] != 3 or not np.isfinite(scene_points).all():
        raise ValueError("scene points must be finite (N,3)")
    if scene_points.shape[0] == 0:
        raise LiftingError("refine", "UNKNOWN: empty observed geometry")
    extent = np.asarray(box.half_extent, dtype=float)
    if extent.shape != (3,) or not np.isfinite(extent).all() or np.any(extent <= 0):
        raise ValueError("object box must have positive finite extent in all axes")

    def acceptable(point):
        if support_plane is not None:
            bottom_height = support_plane.height_above(point) - np.dot(np.abs(support_plane.normal), extent)
            if bottom_height < 0:
                return False
        return not _collides(point, box, scene_points)

    if acceptable(candidate):
        return candidate, False

    n_steps = int(np.floor(radius_m / step_m + 1e-12))
    offsets_1d = np.arange(-n_steps, n_steps + 1) * step_m

    # Vertical-first: check purely-vertical offsets before any lateral
    # search, per AGENTS.md Phase 3 task 5.
    for dz in sorted(offsets_1d, key=lambda value: (abs(value), value < 0)):
        if dz == 0:
            continue
        moved = candidate + np.array([0.0, 0.0, dz])
        if acceptable(moved):
            return moved, True

    best: np.ndarray | None = None
    best_dist = float("inf")
    for dz in offsets_1d:
        for dy in offsets_1d:
            for dx in offsets_1d:
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                moved = candidate + np.array([dx, dy, dz])
                if not acceptable(moved):
                    continue
                dist = dx * dx + dy * dy + dz * dz
                if dist < best_dist:
                    best_dist = dist
                    best = moved
    if best is None:
        raise LiftingError("refine", f"no collision-free point within radius={radius_m} m")
    return best, True
