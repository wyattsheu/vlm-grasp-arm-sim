#!/usr/bin/env python3
"""Per-run 'dashboard-lite' artifacts: no live web server, just three files
saved to disk after a run so you can open them and see what happened. This is
the simplified replacement for the live-websocket dashboard design in
docs/dev_guide_paper_core_and_dashboard_plan.md's original §3 -- kept as
static images because that is enough to *observe* a run afterward, and it
does not need a new always-on server process.

Produces, in --out-dir:
  vlm_overlay.png        RGB + affordance bbox (if any) + a caption strip
                          (timestamp, object, instruction, status)
  candidates_ghost.png   RGB + every grasp candidate's jaw span projected
                          back onto the camera view (green=chosen,
                          grey dashed=accepted-not-chosen, red dashed=rejected)
  summary.json           the same facts as the two captions, machine-readable

Video: if a recorded video.mp4 exists for this run (out/grasp_motion/sessions/
run_NNNN/video.mp4, or pass --video explicitly), it is copied alongside these
images rather than re-encoded -- this script only adds the two annotated
stills, it does not touch video.

Usage:
  python research/scripts/render_run_dashboard.py \
      --scene-bundle research/data/scenes/s4_live_wrist_capture_01 \
      --affordance-json out/grasp_motion/s4_live_test/affordance_region.json \
      --candidates-json out/grasp_motion/s4_live_test/s4_live_candidates.json \
      --chosen-id G000 \
      --out-dir out/grasp_motion/dashboard/s4_live_wrist_capture_01
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "research" / "src"))

from mpg.schema import AffordanceRegion, PointStatus  # noqa: E402
from mpg.viz import render_affordance_overlay, render_candidate_ghosts  # noqa: E402


def _load_affordance_region(path: Path) -> AffordanceRegion:
    d = json.loads(path.read_text())
    status = PointStatus(d["status"])
    bbox = d.get("affordance_bbox_yx_norm1000")
    point = d.get("affordance_point_yx_norm1000")
    return AffordanceRegion(
        status=status,
        description=d.get("affordance_description"),
        point_yx_norm1000=tuple(point) if point else None,
        bbox_yx_norm1000=tuple(bbox) if bbox else None,
        reason_codes=d.get("reason_codes", []),
    )


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scene-bundle", required=True, type=Path, help="dir with rgb.png, tf.json, camera_info.json")
    p.add_argument("--affordance-json", type=Path, default=None)
    p.add_argument("--candidates-json", type=Path, default=None, help="grasp_contract-format candidates JSON")
    p.add_argument("--chosen-id", default=None)
    p.add_argument("--instruction", default=None, help="overrides scene-bundle/instruction.txt if given")
    p.add_argument("--object-id", default=None)
    p.add_argument("--video", type=Path, default=None, help="optional recorded video.mp4 to copy alongside")
    p.add_argument("--out-dir", required=True, type=Path)
    args = p.parse_args()

    bundle = args.scene_bundle
    rgb = Image.open(bundle / "rgb.png").convert("RGB")
    camera_info = json.loads((bundle / "camera_info.json").read_text())
    k_matrix = camera_info["k"]
    tf = json.loads((bundle / "tf.json").read_text())
    t_world_camera = np.array(tf["matrix"])
    t_camera_world = np.linalg.inv(t_world_camera)

    instr_path = bundle / "instruction.txt"
    instruction = args.instruction or (instr_path.read_text().strip() if instr_path.is_file() else "(no instruction recorded)")

    timestamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "generated_at_utc": timestamp,
        "scene_bundle": str(bundle),
        "instruction": instruction,
        "object_id": args.object_id,
    }

    # --- VLM affordance overlay --------------------------------------------
    if args.affordance_json is not None:
        region = _load_affordance_region(args.affordance_json)
        caption = (
            f"time: {timestamp}",
            f"instruction: {instruction}",
            f"object: {args.object_id or '(unspecified)'}",
            f"status: {region.status.value}",
        )
        overlay = render_affordance_overlay(rgb, region, caption_lines=caption)
        overlay_path = args.out_dir / "vlm_overlay.png"
        overlay.save(overlay_path)
        summary["vlm_overlay"] = {
            "path": str(overlay_path),
            "status": region.status.value,
            "description": region.description,
        }
        print(f"wrote {overlay_path}")

    # --- candidate ghosts ----------------------------------------------------
    if args.candidates_json is not None:
        cand_doc = json.loads(args.candidates_json.read_text())
        candidates = cand_doc["candidates"] if "candidates" in cand_doc else cand_doc
        caption = (
            f"time: {timestamp}",
            f"scene: {cand_doc.get('scene_id', bundle.name)}",
            f"chosen: {args.chosen_id or '(none recorded)'}",
        )
        ghosts = render_candidate_ghosts(
            rgb, candidates, k_matrix=k_matrix, t_camera_world=t_camera_world,
            chosen_id=args.chosen_id, caption_lines=caption,
        )
        ghosts_path = args.out_dir / "candidates_ghost.png"
        ghosts.save(ghosts_path)
        summary["candidates_ghost"] = {
            "path": str(ghosts_path),
            "n_candidates": len(candidates),
            "chosen_id": args.chosen_id,
        }
        print(f"wrote {ghosts_path}")

    if args.video is not None and args.video.is_file():
        video_out = args.out_dir / "camera.mp4"
        shutil.copy2(args.video, video_out)
        summary["video"] = str(video_out)
        print(f"copied {args.video} -> {video_out}")

    (args.out_dir / "summary.json").write_text(json.dumps(summary, indent=2))
    print(f"wrote {args.out_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
