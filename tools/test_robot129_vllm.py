#!/usr/bin/env python3
"""One text and one image request against the on-demand Robot 129 vLLM."""
from __future__ import annotations

import argparse
import base64
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import time

import requests

ROOT = Path(__file__).resolve().parents[1]


def parse_object(text: str) -> dict:
    clean = text.strip()
    clean = re.sub(r"^```(?:json)?\\s*", "", clean, flags=re.I)
    clean = re.sub(r"\\s*```$", "", clean)
    match = re.search(r"\{.*\}", clean, flags=re.S)
    if not match:
        raise ValueError(f"no JSON object in response: {text[:200]!r}")
    value = json.loads(match.group())
    if not isinstance(value, dict):
        raise ValueError("response JSON is not an object")
    return value


def post(url: str, payload: dict, timeout: float = 120.0) -> tuple[dict, float]:
    start = time.monotonic()
    response = requests.post(url, json=payload, timeout=timeout)
    response.raise_for_status()
    return response.json(), time.monotonic() - start


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", default="http://127.0.0.1:8001/v1")
    parser.add_argument("--model", default="qwen-vl")
    parser.add_argument(
        "--image",
        type=Path,
        default=ROOT / "out/ros_webrtc_robot129/viewport_probe.png",
    )
    args = parser.parse_args()
    model_info = requests.get(f"{args.base_url}/models", timeout=10).json()
    endpoint = f"{args.base_url}/chat/completions"

    text_data, text_latency = post(endpoint, {
        "model": args.model,
        "messages": [
            {"role": "system", "content": "Return only one JSON object. No markdown."},
            {"role": "user", "content": 'Extract this Robot 129 command: "grasp the red cube". Schema: {"action":"grasp|place|handover","label":"string"}'},
        ],
        "temperature": 0.0,
        "max_tokens": 96,
    })
    text_raw = text_data["choices"][0]["message"]["content"] or ""
    text_parsed = parse_object(text_raw)
    if text_parsed.get("action") not in {"grasp", "place", "handover"}:
        raise ValueError(f"unexpected action: {text_parsed!r}")

    image_bytes = args.image.read_bytes()
    mime = "image/png" if args.image.suffix.lower() == ".png" else "image/jpeg"
    encoded = base64.b64encode(image_bytes).decode("ascii")
    vision_data, vision_latency = post(endpoint, {
        "model": args.model,
        "messages": [{
            "role": "user",
            "content": [
                {"type": "text", "text": 'Inspect this simulation image. Return only JSON: {"robot_visible":true_or_false,"red_object_visible":true_or_false}.'},
                {"type": "image_url", "image_url": {"url": f"data:{mime};base64,{encoded}"}},
            ],
        }],
        "temperature": 0.0,
        "max_tokens": 96,
    }, timeout=180)
    vision_raw = vision_data["choices"][0]["message"]["content"] or ""
    vision_parsed = parse_object(vision_raw)
    if not {"robot_visible", "red_object_visible"}.issubset(vision_parsed):
        raise ValueError(f"missing vision keys: {vision_parsed!r}")

    report = {
        "status": "PASS",
        "scope": "VLLM_OPENAI_COMPATIBLE_TEXT_AND_IMAGE_SMOKE",
        "simulation_only": True,
        "base_url": args.base_url,
        "model": args.model,
        "model_info": model_info,
        "image": str(args.image),
        "text": {"latency_s": round(text_latency, 3), "raw": text_raw, "parsed": text_parsed},
        "vision": {"latency_s": round(vision_latency, 3), "raw": vision_raw, "parsed": vision_parsed},
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "control_command_sent": False,
    }
    output = ROOT / "out/vllm_robot129/smoke_test.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
