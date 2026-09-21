"""Overlay visualization (AGENTS.md Sec 2 task 4): colored points, labels,
a legend, and a baseline-vs-new side-by-side. Pure PIL, no matplotlib
dependency (this environment cannot install new packages — see
src/mpg/vlm/gemini.py's docstring for why).

Colors match ZeroDex Fig. S1's convention: grasp red, waypoint blue,
release green. APPLY_ACTION/HOLD (tool-use, out of this sprint's executor
scope) get distinct colors too, so a tool-use plan doesn't look identical
to an unlabeled pick-and-place one.

An UNAVAILABLE point is never silently skipped — it is listed in a status
line under the image with its reason codes, so an overlay can't look like
a clean success when the model actually abstained (rule 15: no
false-green).
"""

from __future__ import annotations

import math

from PIL import Image, ImageDraw, ImageFont

from .schema import AffordanceRegion, LocatedStep, Plan, PointStatus, StepType, pixel_from_norm1000

STEP_COLORS: dict[StepType, tuple[int, int, int]] = {
    StepType.GRASP: (220, 40, 40),  # red
    StepType.WAYPOINT: (40, 90, 220),  # blue
    StepType.RELEASE: (30, 160, 70),  # green
    StepType.FUNCTIONAL_TIP: (0, 170, 170),  # teal
    StepType.APPLY_ACTION: (230, 140, 20),  # orange
    StepType.HOLD: (150, 60, 200),  # purple
}

_TRUETYPE_CANDIDATES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
)


def _load_font(size: int) -> ImageFont.ImageFont:
    for path in _TRUETYPE_CANDIDATES:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _scale_for(width: int, height: int) -> tuple[int, int, ImageFont.ImageFont]:
    """Point radius and label font scale with image size — a fixed pixel
    radius reads fine on a 4x4 synthetic test image and is invisible on a
    1280px real photo. Floors keep tiny images (unit tests) legible too."""
    short_side = min(width, height)
    radius = max(6, round(short_side * 0.018))
    font_size = max(11, round(short_side * 0.035))
    return radius, font_size, _load_font(font_size)


def _draw_point(
    draw: ImageDraw.ImageDraw, x: float, y: float, color: tuple[int, int, int], label: str,
    *, radius: int, font: ImageFont.ImageFont,
) -> None:
    r = radius
    draw.ellipse((x - r, y - r, x + r, y + r), outline=color, width=max(2, r // 3))
    draw.line((x - r * 1.6, y, x + r * 1.6, y), fill=color, width=max(2, r // 4))
    draw.line((x, y - r * 1.6, x, y + r * 1.6), fill=color, width=max(2, r // 4))
    # A light halo behind the text keeps a light-colored label legible over
    # a busy background, matching the paper's own boxed-label convention.
    text_xy = (x + r + 4, y - r - 4)
    draw.text(text_xy, label, fill=color, font=font, stroke_width=max(2, r // 5), stroke_fill=(255, 255, 255))


def _legend_and_status(
    width: int, located: tuple[LocatedStep, ...], *, font_size: int
) -> Image.Image:
    """A strip under the image: a color-keyed legend plus, when any step is
    UNAVAILABLE, an explicit line naming which step and why."""
    font = _load_font(font_size)
    line_h = round(font_size * 1.7)
    unavailable = [s for s in located if s.point_status is PointStatus.UNAVAILABLE]
    height = line_h * (1 + len(unavailable)) + font_size // 2
    strip = Image.new("RGB", (width, height), color=(255, 255, 255))
    draw = ImageDraw.Draw(strip)

    x = font_size // 3
    swatch = font_size
    # Show only the step types actually present in this plan, in
    # STEP_COLORS' fixed order, so a tool-use plan's legend (GRASP,
    # FUNCTIONAL_TIP, APPLY_ACTION, RELEASE/HOLD) doesn't silently show
    # the pick-mode legend (GRASP, WAYPOINT, RELEASE) instead.
    present_types = {s.spec.type for s in located}
    for step_type in STEP_COLORS:
        if step_type not in present_types:
            continue
        color = STEP_COLORS[step_type]
        draw.ellipse((x, 3, x + swatch, 3 + swatch), outline=color, width=2)
        draw.text((x + swatch + 4, 2), step_type.value, fill=color, font=font)
        x += swatch + 4 + draw.textlength(step_type.value, font=font) + swatch

    for i, step in enumerate(unavailable):
        y = line_h * (i + 1)
        reasons = ", ".join(step.reason_codes) or "no reason code given"
        draw.text(
            (font_size // 3, y),
            f"UNAVAILABLE: {step.spec.step_id} ({step.spec.type.value}) — {reasons}",
            fill=(180, 0, 0),
            font=font,
        )
    return strip


def render_plan_overlay(image: Image.Image, plan: Plan, *, adapter: str = "new") -> Image.Image:
    """Draw every LOCALIZED step's point on a copy of the image, with an
    explicit UNAVAILABLE status strip underneath — never a bare image that
    looks the same whether the model succeeded or abstained."""
    vis = image.convert("RGB").copy()
    draw = ImageDraw.Draw(vis)
    width, height = vis.size
    radius, font_size, font = _scale_for(width, height)

    for step in plan.steps:
        if step.point_status is not PointStatus.LOCALIZED:
            continue
        x_px, y_px = pixel_from_norm1000(
            step.point_yx_norm1000, width=width, height=height, adapter=adapter
        )
        color = STEP_COLORS.get(step.spec.type, (0, 0, 0))
        _draw_point(
            draw, x_px, y_px, color, f"{step.spec.type.value} ({step.spec.step_id})",
            radius=radius, font=font,
        )

    strip = _legend_and_status(width, plan.steps, font_size=font_size)
    out = Image.new("RGB", (width, height + strip.height), color=(255, 255, 255))
    out.paste(vis, (0, 0))
    out.paste(strip, (0, height))
    return out


def render_baseline_overlay(
    image: Image.Image,
    *,
    target_point_px: tuple[float, float] | None,
    destination_point_px: tuple[float, float] | None,
    target_label: str = "target",
    destination_label: str = "destination",
) -> Image.Image:
    """Baseline (prompts/baseline_copied.txt) draws exactly two points —
    no waypoint exists in that system, so there is nothing to omit."""
    vis = image.convert("RGB").copy()
    draw = ImageDraw.Draw(vis)
    width, height = vis.size
    radius, font_size, font = _scale_for(width, height)
    if target_point_px is not None:
        _draw_point(draw, *target_point_px, STEP_COLORS[StepType.GRASP], target_label, radius=radius, font=font)
    if destination_point_px is not None:
        _draw_point(draw, *destination_point_px, STEP_COLORS[StepType.RELEASE], destination_label, radius=radius, font=font)

    missing = []
    if target_point_px is None:
        missing.append("target not found")
    if destination_point_px is None:
        missing.append("destination not found")
    strip_h = round(font_size * 1.7) if missing else 4
    strip = Image.new("RGB", (width, strip_h), color=(255, 255, 255))
    if missing:
        ImageDraw.Draw(strip).text((4, 2), "; ".join(missing), fill=(180, 0, 0), font=font)
    out = Image.new("RGB", (width, height + strip_h), color=(255, 255, 255))
    out.paste(vis, (0, 0))
    out.paste(strip, (0, height))
    return out


def side_by_side(left: Image.Image, right: Image.Image, *, gap: int = 12, label_left: str = "baseline", label_right: str = "new") -> Image.Image:
    """Baseline and new outputs next to each other (AGENTS.md Sec 2 task 4)."""
    _, font_size, font = _scale_for(left.width, left.height)
    label_h = round(font_size * 1.5)
    h = max(left.height, right.height) + label_h
    w = left.width + gap + right.width
    out = Image.new("RGB", (w, h), color=(255, 255, 255))
    draw = ImageDraw.Draw(out)
    draw.text((4, 2), label_left, fill=(0, 0, 0), font=font)
    draw.text((left.width + gap + 4, 2), label_right, fill=(0, 0, 0), font=font)
    out.paste(left, (0, label_h))
    out.paste(right, (left.width + gap, label_h))
    return out


def render_affordance_overlay(
    image: Image.Image,
    region: "AffordanceRegion",
    *,
    caption_lines: tuple[str, ...] = (),
) -> Image.Image:
    """Draw one run's grasp-affordance VLM output on a copy of the image: the
    bbox (orange, per D.2's contact-region prompt) plus its center point, and
    an optional caption strip underneath for whatever the caller wants
    recorded alongside the image -- e.g. timestamp, object id, instruction,
    LOCALIZED/UNAVAILABLE status. Built for the per-run dashboard artifacts
    (research/scripts/render_run_dashboard.py), not for the live VLM-call
    loop, so it takes the already-parsed AffordanceRegion rather than a raw
    backend response.

    An UNAVAILABLE region draws no box (there is nothing to draw — rule 15,
    no false-green) but the caption still reports it, same convention as
    render_plan_overlay's status strip."""
    vis = image.convert("RGB").copy()
    draw = ImageDraw.Draw(vis)
    width, height = vis.size
    _, font_size, font = _scale_for(width, height)
    color = (230, 140, 20)  # orange, distinct from the GRASP/WAYPOINT/RELEASE palette

    if region.status is PointStatus.LOCALIZED and region.bbox_yx_norm1000 is not None:
        y1, x1, y2, x2 = region.bbox_yx_norm1000
        (px1, py1) = pixel_from_norm1000((y1, x1), width=width, height=height)
        (px2, py2) = pixel_from_norm1000((y2, x2), width=width, height=height)
        draw.rectangle((px1, py1, px2, py2), outline=color, width=max(3, font_size // 6))
        label = region.description or "affordance region"
        draw.text(
            (px1 + 2, max(0, py1 - font_size - 4)), label, fill=color, font=font,
            stroke_width=max(2, font_size // 8), stroke_fill=(255, 255, 255),
        )
        if region.point_yx_norm1000 is not None:
            cx, cy = pixel_from_norm1000(region.point_yx_norm1000, width=width, height=height)
            _draw_point(draw, cx, cy, color, "", radius=max(4, font_size // 3), font=font)

    lines = list(caption_lines)
    if region.status is not PointStatus.LOCALIZED:
        reasons = ", ".join(region.reason_codes) or "no reason code given"
        lines.append(f"UNAVAILABLE: {reasons}")
    if not lines:
        return vis

    line_h = round(font_size * 1.5)
    strip = Image.new("RGB", (width, line_h * len(lines) + font_size // 2), color=(255, 255, 255))
    sdraw = ImageDraw.Draw(strip)
    for i, line in enumerate(lines):
        fill = (180, 0, 0) if line.startswith("UNAVAILABLE") else (0, 0, 0)
        sdraw.text((font_size // 3, line_h * i), line, fill=fill, font=font)
    out = Image.new("RGB", (width, height + strip.height), color=(255, 255, 255))
    out.paste(vis, (0, 0))
    out.paste(strip, (0, height))
    return out


def render_candidate_ghosts(
    image: Image.Image,
    candidates: list[dict],
    *,
    k_matrix: list[float],
    t_camera_world: "np.ndarray",
    chosen_id: str | None = None,
    caption_lines: tuple[str, ...] = (),
) -> Image.Image:
    """Draw every grasp candidate as a semi-transparent gripper-shaped
    'ghost' (two finger blocks spanning the jaw opening, filled and
    translucent, not just a line), projected from its world-frame pose back
    into the camera it was captured from. Many overlapping ghosts are meant
    to read the way ZeroDex's own "Atomic-Action Alignment" figure does --
    a fan of translucent gripper/hand poses converging on the target -- not
    as an abstract vector diagram. This is a static-image stand-in for the
    RViz MarkerArray candidate visualization (see docs/progress's
    "候選決策視覺化" section), which only ever exists live in RViz on a
    machine with no display to screenshot it from.

    `candidates` is the list of dicts as written by grasp_contract's
    write_candidates_json / read back with read_candidates_json (each needs
    candidate_id, tcp_pose.position_m, yaw_rad, opening_width_m, and
    optionally rejection_reasons/accepted). `t_camera_world` is the 4x4
    inverse of the captured T_world_camera (world point -> camera-optical
    frame point), i.e. np.linalg.inv(tf["matrix"]).

    Chosen candidate: solid green, drawn last and fully opaque so it stands
    out from the pile. Every other accepted candidate: translucent grey.
    Rejected candidate: translucent red. A candidate whose points project
    behind the camera (z <= 0) is skipped, not silently drawn wrong."""
    import numpy as np

    from .lifting import apply_transform, project_point

    base = image.convert("RGBA")
    width, height = base.size
    _, font_size, font = _scale_for(width, height)

    # Real jaw spans/finger widths are often only ~1-2cm on a small object
    # seen top-down, projecting to a handful of pixels -- unreadable, and
    # every candidate's ghost piles onto the same few pixels since they
    # share a center. Each ghost's finger span is stretched (about its true
    # center, keeping its true on-screen orientation) to at least this many
    # pixels so the shapes stay legible; this is a visual exaggeration of
    # scale, not of position or angle, and is named as such in the caption
    # so it never reads as a real-size measurement.
    min_half_len_px = max(22.0, font_size * 1.3)
    finger_half_thickness_px = max(5.0, font_size * 0.35)

    def _draw_gripper_ghost(layer_draw, pa, pb, color, alpha, thick_px):
        """One candidate's ghost: two finger blocks (perpendicular to the
        closing axis, at each jaw-contact point) plus a thin bridge between
        them, all filled -- reads as 'a gripper about to close here', not a
        vector arrow."""
        pa = np.array(pa, dtype=float)
        pb = np.array(pb, dtype=float)
        closing_dir = pb - pa
        length = float(np.linalg.norm(closing_dir))
        closing_dir = closing_dir / length if length > 1e-6 else np.array([1.0, 0.0])
        across_dir = np.array([-closing_dir[1], closing_dir[0]])  # perpendicular, in-plane
        finger_half_len = max(min_half_len_px * 0.55, thick_px * 2)
        fill = (*color, alpha)
        for tip in (pa, pb):
            poly = [
                tuple(tip + across_dir * finger_half_len),
                tuple(tip - across_dir * finger_half_len),
            ]
            layer_draw.line(poly, fill=fill, width=int(finger_half_thickness_px * 2))
            layer_draw.ellipse(
                (tip[0] - finger_half_thickness_px, tip[1] - finger_half_thickness_px,
                 tip[0] + finger_half_thickness_px, tip[1] + finger_half_thickness_px),
                fill=fill,
            )
        layer_draw.line((tuple(pa), tuple(pb)), fill=fill, width=max(2, int(thick_px * 0.4)))

    items = []
    for cand in candidates:
        cid = cand["candidate_id"]
        pos = np.array(cand["tcp_pose"]["position_m"], dtype=float)
        yaw = float(cand["yaw_rad"])
        half_open = float(cand["opening_width_m"]) / 2.0
        closing_axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        jaw_a_world = pos + closing_axis * half_open
        jaw_b_world = pos - closing_axis * half_open

        try:
            jaw_a_cam = apply_transform(t_camera_world, jaw_a_world)
            jaw_b_cam = apply_transform(t_camera_world, jaw_b_world)
            pa = np.array(project_point(jaw_a_cam, k_matrix))
            pb = np.array(project_point(jaw_b_cam, k_matrix))
        except Exception:
            continue  # behind the camera or otherwise unprojectable -- skip, don't guess

        mid = (pa + pb) / 2
        direction = pb - pa
        length = float(np.linalg.norm(direction))
        direction = direction / length if length > 1e-6 else np.array([1.0, 0.0])
        if length / 2 < min_half_len_px:
            pa = mid - direction * min_half_len_px
            pb = mid + direction * min_half_len_px

        accepted = bool(cand.get("accepted", not cand.get("rejection_reasons")))
        is_chosen = chosen_id is not None and cid == chosen_id
        if is_chosen:
            color, alpha, thick_px = (30, 200, 60), 255, max(4, font_size // 5)
        elif accepted:
            color, alpha, thick_px = (170, 170, 175), 70, max(3, font_size // 7)
        else:
            color, alpha, thick_px = (220, 40, 40), 70, max(3, font_size // 7)

        score = cand.get("score")
        items.append((is_chosen, cid, tuple(pa), tuple(pb), color, alpha, thick_px, score, accepted))

    # Chosen candidate drawn last so it's fully opaque on top of the
    # translucent pile, not blended into it.
    items.sort(key=lambda it: it[0])
    detail_lines = []
    for i, (is_chosen, cid, pa, pb, color, alpha, thick_px, score, accepted) in enumerate(items):
        layer = Image.new("RGBA", base.size, (0, 0, 0, 0))
        layer_draw = ImageDraw.Draw(layer)
        _draw_gripper_ghost(layer_draw, pa, pb, color, alpha, thick_px)
        base = Image.alpha_composite(base, layer)
        tag = "CHOSEN" if is_chosen else ("accepted" if accepted else "rejected")
        score_txt = f"{score:.3f}" if isinstance(score, (int, float)) else "?"
        detail_lines.append(f"[{i}] {cid} score={score_txt} {tag}")

    vis = base.convert("RGB")
    draw = ImageDraw.Draw(vis)
    # Index tags after compositing, on the flattened image, so text stays
    # crisp (not alpha-blended) regardless of how many ghosts are underneath.
    for i, (is_chosen, cid, pa, pb, color, alpha, thick_px, score, accepted) in enumerate(items):
        mid = ((pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2)
        angle = (i / max(1, len(items))) * 2 * math.pi
        tx = mid[0] + math.cos(angle) * (min_half_len_px + 10)
        ty = mid[1] + math.sin(angle) * (min_half_len_px + 10) - font_size / 2
        label_color = color if is_chosen else (90, 90, 90) if accepted else (150, 20, 20)
        draw.text(
            (tx, ty), str(i), fill=label_color, font=font,
            stroke_width=max(1, font_size // 10), stroke_fill=(255, 255, 255),
        )

    lines = list(caption_lines)
    lines.append(f"{len(items)}/{len(candidates)} candidates projected onto camera view (gripper size exaggerated for legibility, not to true scale)")
    lines.extend(detail_lines)
    line_h = round(font_size * 1.5)
    strip = Image.new("RGB", (width, line_h * len(lines) + font_size // 2), color=(255, 255, 255))
    sdraw = ImageDraw.Draw(strip)
    for i, line in enumerate(lines):
        sdraw.text((font_size // 3, line_h * i), line, fill=(0, 0, 0), font=font)
    out = Image.new("RGB", (width, height + strip.height), color=(255, 255, 255))
    out.paste(vis, (0, 0))
    out.paste(strip, (0, height))
    return out
