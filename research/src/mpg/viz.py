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
    """Draw every grasp candidate's jaw span as a semi-transparent 'ghost'
    line on a copy of the image, projected from its world-frame pose back
    into the camera it was captured from -- a static-image stand-in for the
    RViz MarkerArray candidate visualization (see docs/progress's
    "候選決策視覺化" section), which only ever exists live in RViz on a
    machine with no display to screenshot it from.

    `candidates` is the list of dicts as written by grasp_contract's
    write_candidates_json / read back with read_candidates_json (each needs
    candidate_id, tcp_pose.position_m, yaw_rad, opening_width_m, and
    optionally rejection_reasons/accepted). `t_camera_world` is the 4x4
    inverse of the captured T_world_camera (world point -> camera-optical
    frame point), i.e. np.linalg.inv(tf["matrix"]).

    Chosen candidate: solid green. Any other accepted candidate: dashed
    grey (still legal, just not the one taken). Rejected candidate: dashed
    red. A candidate whose jaw-span points project behind the camera
    (z <= 0) is skipped, not silently drawn wrong."""
    import numpy as np

    from .lifting import apply_transform, project_point

    vis = image.convert("RGB").copy()
    draw = ImageDraw.Draw(vis)
    width, height = vis.size
    _, font_size, font = _scale_for(width, height)

    def _dashed_line(p0, p1, color, width_px):
        p0 = np.array(p0, dtype=float)
        p1 = np.array(p1, dtype=float)
        length = float(np.linalg.norm(p1 - p0))
        if length < 1e-6:
            return
        n_dashes = max(1, int(length // 6))
        for i in range(n_dashes):
            if i % 2 == 1:
                continue
            a = p0 + (p1 - p0) * (i / n_dashes)
            b = p0 + (p1 - p0) * (min(i + 1, n_dashes) / n_dashes)
            draw.line((tuple(a), tuple(b)), fill=color, width=width_px)

    # Real jaw spans are often only ~1-2cm on a small object seen top-down,
    # which projects to a few pixels -- unreadable, and every candidate's
    # ghost piles up on the same few pixels since they share a center. Each
    # ghost is stretched (about its true center, keeping its true on-screen
    # orientation) to at least this many pixels so it stays legible; this is
    # a visual exaggeration of scale, not of position or angle, and is named
    # as such in the caption so it never reads as a real-size measurement.
    min_half_len_px = max(18.0, font_size * 1.1)

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
            color, width_px, solid = (30, 200, 60), max(3, font_size // 6), True
        elif accepted:
            color, width_px, solid = (150, 150, 150), max(2, font_size // 8), False
        else:
            color, width_px, solid = (220, 40, 40), max(2, font_size // 8), False

        score = cand.get("score")
        items.append((is_chosen, cid, tuple(pa), tuple(pb), tuple(mid), color, width_px, solid, score, accepted))

    # Chosen candidate drawn last so it sits on top of the pile, not buried
    # under whichever unrelated candidate happened to come later in the list.
    items.sort(key=lambda it: it[0])
    # Full id/score/status text goes in the caption strip below, not on the
    # image: for a small top-down object every candidate's ghost lands on
    # nearly the same few pixels, and text labels there just pile into an
    # unreadable stack. Each ghost gets only a 1-2 digit index tag on the
    # image; the caption spells out what each index is.
    detail_lines = []
    for i, (is_chosen, cid, pa, pb, mid, color, width_px, solid, score, accepted) in enumerate(items):
        if solid:
            draw.line((pa, pb), fill=color, width=width_px)
        else:
            _dashed_line(pa, pb, color, width_px)
        draw.ellipse((mid[0] - 2, mid[1] - 2, mid[0] + 2, mid[1] + 2), fill=color)
        # Tag sits just past one jaw tip, offset around the shared center by
        # index so tags for candidates piled on the same pixel don't overlap.
        angle = (i / max(1, len(items))) * 2 * math.pi
        tx = mid[0] + math.cos(angle) * (min_half_len_px + 6)
        ty = mid[1] + math.sin(angle) * (min_half_len_px + 6) - font_size / 2
        draw.text(
            (tx, ty), str(i), fill=color, font=font,
            stroke_width=max(1, font_size // 10), stroke_fill=(255, 255, 255),
        )
        tag = "CHOSEN" if is_chosen else ("accepted" if accepted else "rejected")
        score_txt = f"{score:.3f}" if isinstance(score, (int, float)) else "?"
        detail_lines.append(f"[{i}] {cid} score={score_txt} {tag}")

    lines = list(caption_lines)
    lines.append(f"{len(items)}/{len(candidates)} candidates projected onto camera view (jaw span exaggerated for legibility, not to true scale)")
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
