"""Molmo2 pointing backend — the L2 rung of docs/prompt_strategy.md Sec 5:
Qwen decides roles/plan semantics, Molmo2 localizes each reference point.

Molmo2 does NOT speak our JSON schema. It emits its own inline tag format,
so this module owns both the request shape and a parser for that format,
then the caller maps points into schema.LocatedStep. One request per
reference point (prompt_strategy.md Sec 5 item 4: Molmo2's multi-reference
ID correspondence is unverified, so never assume output order equals
semantic order).

Wire-format provenance (read-only, never executed — AGENTS.md forbids
running scripts under LOCAL_VLM_CODE):
  - prompt shape "Point to the {label}." and the JPEG-q92 encoding:
    /home/acm/robotic_agent/new_modle_test/stage2_molmo_pointing.py:33-51
  - output format `<points coords="<frame> <idx> <x> <y>">label</points>`
    with x/y scaled 0-1000 of image width/height, and the "There are none."
    not-found reply:
    /home/acm/robotic_agent/new_modle_test/molmo_point_parser.py:1-34
    (which in turn cites allenai/Molmo2-4B's own README point-QA example)
  - service: container local_pipeline_stage2_molmo2, vLLM on :8002,
    per /home/acm/robotic_agent/new_modle_test/HANDOFF_TO_INTEGRATOR.md:18

The not-found reply is the reason this pipeline was split in two: per that
handoff doc, "accurate models would hallucinate a point on a nonexistent
target, and models with a clean 'not found' behavior were inaccurate at
pointing." That maps directly onto our PointStatus.UNAVAILABLE.
"""

from __future__ import annotations

import base64
import io
import re
from pathlib import Path

import requests
from PIL import Image

from .base import BaseBackend

DEFAULT_MODEL = "allenai/Molmo2-4B"
DEFAULT_URL = "http://127.0.0.1:8002/v1/chat/completions"

_COORD_RE = re.compile(r'<(?:points|tracks).*? coords="([0-9\t:;, .]+)"/?>')
_FRAME_RE = re.compile(r'(?:^|\t|:|,|;)([0-9.]+) ([0-9. ]+)')
_POINTS_RE = re.compile(r"([0-9]+) ([0-9]{1,4}) ([0-9]{1,4})")

# Molmo2's literal not-found reply; matched case-insensitively on a
# stripped prefix so trailing prose does not defeat the check.
NOT_FOUND_PREFIX = "there are none"


def parse_molmo_points(text: str, *, width: int, height: int) -> list[tuple[float, float]]:
    """Pixel (x, y) points from Molmo2's inline tag output. Empty list means
    no parseable point — which, combined with is_not_found(), lets a caller
    tell 'model said it isn't there' apart from 'model emitted garbage'."""
    out: list[tuple[float, float]] = []
    for coord in _COORD_RE.finditer(text):
        for frame_grp in _FRAME_RE.finditer(coord.group(1)):
            for m in _POINTS_RE.finditer(frame_grp.group(2)):
                x = float(m.group(2)) / 1000 * width
                y = float(m.group(3)) / 1000 * height
                if 0 <= x <= width and 0 <= y <= height:
                    out.append((x, y))
    return out


def is_not_found(text: str) -> bool:
    return text.strip().lower().startswith(NOT_FOUND_PREFIX)


def point_prompt(label: str) -> str:
    return f"Point to the {label}."


class MolmoBackend(BaseBackend):
    backend_name = "local_molmo2"

    def __init__(
        self,
        *,
        cache_dir: Path,
        log_path: Path,
        model: str = DEFAULT_MODEL,
        url: str = DEFAULT_URL,
        call_cap: int | None = None,
        session: requests.Session | None = None,
        timeout_s: float = 120.0,
    ):
        self.model = model
        self._url = url
        self._session = session or requests.Session()
        self._timeout_s = timeout_s
        super().__init__(cache_dir=cache_dir, log_path=log_path, call_cap=call_cap)

    def _call(self, image: Image.Image, prompt: str) -> str:
        # JPEG q92 rather than PNG, matching the validated stage-2 client —
        # a different encoding is a different input to the model, so keep it
        # identical to what Rounds 15-17 measured.
        buf = io.BytesIO()
        image.convert("RGB").save(buf, format="JPEG", quality=92)
        b64 = base64.b64encode(buf.getvalue()).decode("ascii")
        payload = {
            "model": self.model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{b64}"},
                        },
                    ],
                }
            ],
            "max_tokens": 512,
            "temperature": 0.0,
        }
        response = self._session.post(self._url, json=payload, timeout=self._timeout_s)
        response.raise_for_status()
        data = response.json()
        try:
            return data["choices"][0]["message"]["content"] or ""
        except (KeyError, IndexError, TypeError) as exc:
            raise RuntimeError(f"unexpected Molmo2 response shape: keys={list(data)}") from exc
