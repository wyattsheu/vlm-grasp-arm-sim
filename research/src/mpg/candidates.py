"""Numbered-grid localization ablation inspired by MOKA IV-C, not a reproduction.

Source: https://arxiv.org/html/2403.03174v3#S4.SS3
No task labels are used to propose candidates. A grid can miss small objects;
proposal coverage must be measured separately from model selection accuracy.
"""
import json
import numpy as np
from PIL import Image, ImageDraw
from .schema import StageAPlan, ParseError, parse_stage_b_response


def propose_grid(width: int, height: int, *, rows: int, columns: int, seed: int,
                 depth_m: np.ndarray | None = None) -> list[dict]:
    if min(width, height, rows, columns) < 1 or rows > height or columns > width:
        raise ValueError("grid and image dimensions are invalid")
    if depth_m is not None and depth_m.shape != (height, width):
        raise ValueError("depth must match image dimensions")
    points = []
    for y in np.linspace(0, height-1, rows+2)[1:-1]:
        for x in np.linspace(0, width-1, columns+2)[1:-1]:
            if depth_m is not None:
                value = depth_m[int(round(y)), int(round(x))]
                if not np.isfinite(value) or value <= 0:
                    continue
            points.append((float(x), float(y)))
    ids = np.random.default_rng(seed).permutation(len(points))
    return [{"id": f"C{int(i):03d}", "pixel_xy": [x, y],
             "point_yx_norm1000": [y/max(height-1, 1)*1000, x/max(width-1, 1)*1000]}
            for i, (x, y) in zip(ids, points)]


def render_candidates(image: Image.Image, candidates: list[dict]) -> Image.Image:
    out = image.convert("RGB").copy()
    draw = ImageDraw.Draw(out)
    for row in candidates:
        x, y = row["pixel_xy"]
        draw.ellipse((x-3, y-3, x+3, y+3), fill="yellow", outline="black")
        draw.text((x+4, y), row["id"], fill="yellow", stroke_width=1, stroke_fill="black")
    return out


def candidate_prompt(stage_a: StageAPlan, candidates: list[dict]) -> str:
    return (
        "Localize this fixed plan using only the numbered yellow marks. "
        "Return a JSON list in the exact step order. Each entry must contain "
        "step_id, candidate_id, point_status, reason_codes. "
        "For a supported mark use point_status='localized' and reason_codes=[]. "
        "If no candidate is suitable, use candidate_id=null, point_status='unavailable', "
        "reason_codes=['UNCERTAIN_LOCALIZATION']. Do not invent a mark or change the plan.\n"
        + json.dumps(stage_a.to_json(), ensure_ascii=False)
        + "\nAllowed candidate IDs: " + json.dumps([row["id"] for row in candidates])
    )


def parse_candidates(raw: str, stage_a: StageAPlan, candidates: list[dict]):
    from .schema import strip_code_fence, parse_json_strict
    rows = parse_json_strict("candidate_b", strip_code_fence(raw))
    if not isinstance(rows, list):
        raise ParseError("candidate_b", "expected a list", raw)
    lookup = {row["id"]: row for row in candidates}
    located = []
    for row in rows:
        if not isinstance(row, dict):
            raise ParseError("candidate_b", "expected step object", raw)
        cid = row.get("candidate_id")
        if cid is not None and (not isinstance(cid, str) or cid not in lookup):
            raise ParseError("candidate_b", "unknown candidate ID", raw)
        located.append({**row, "point_yx_norm1000": lookup[cid]["point_yx_norm1000"] if cid else None})
    return parse_stage_b_response(json.dumps(located), stage_a)
