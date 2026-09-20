#!/usr/bin/env python3
"""Validate a captured SceneBundle without ROS."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK_DIR / "src"))

from mpg.scene_bundle import validate_scene


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scene_dir", type=Path)
    args = parser.parse_args()
    try:
        result = validate_scene(args.scene_dir)
    except Exception as exc:
        print(json.dumps({"valid": False, "error": str(exc)}))
        return 1
    print(json.dumps({"valid": True, **result}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
