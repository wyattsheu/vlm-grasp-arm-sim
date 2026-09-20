#!/usr/bin/env python3
"""Interactively label target, destination, and obstacle bounding boxes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def _box(values: tuple[int, int, int, int]) -> list[int] | None:
    x, y, width, height = (int(value) for value in values)
    return [x, y, x + width - 1, y + height - 1] if width > 0 and height > 0 else None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scene_dir", type=Path)
    parser.add_argument("--obstacles", type=int, default=0)
    parser.add_argument("--annotator", required=True)
    args = parser.parse_args()
    try:
        import cv2
    except ImportError as exc:
        raise SystemExit("OpenCV Python is required for the interactive label tool") from exc
    image_path = args.scene_dir / "rgb.png"
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"cannot read {image_path}")

    target = _box(cv2.selectROI("target", image, showCrosshair=True, fromCenter=False))
    destination = _box(
        cv2.selectROI("destination", image, showCrosshair=True, fromCenter=False)
    )
    obstacles = []
    for index in range(args.obstacles):
        value = _box(cv2.selectROI(
            f"obstacle_{index}", image, showCrosshair=True, fromCenter=False
        ))
        if value is not None:
            obstacles.append({"instance_id": f"obstacle_{index}", "bbox_xyxy": value})
    cv2.destroyAllWindows()
    if target is None or destination is None:
        raise SystemExit("target and destination boxes are required")

    labels = {
        "annotation_version": "bbox_v1",
        "coordinate_convention": "pixel_xyxy_inclusive",
        "annotator": args.annotator,
        "instruction_validity": "VALID",
        "target": {
            "instance_id": "target_0", "bbox_xyxy": target,
            "visibility": "VISIBLE", "acceptable_grasp_regions": [],
        },
        "destination": {
            "instance_id": "destination_0", "bbox_xyxy": destination,
            "visibility": "VISIBLE", "acceptable_release_regions": [],
        },
        "obstacles": obstacles,
        "notes": "",
    }
    path = args.scene_dir / "labels.json"
    path.write_text(json.dumps(labels, indent=2) + "\n", encoding="utf-8")
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
