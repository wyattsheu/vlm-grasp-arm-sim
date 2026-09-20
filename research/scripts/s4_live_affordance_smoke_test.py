#!/usr/bin/env python3
"""S4 real end-to-end smoke test: real Isaac wrist-camera capture + real local VLM
affordance call + real depth deprojection + real world-frame transform + real grasp
candidate generation, against a scene bundle captured by capture_scene.py --best-effort
(see docs/progress/grasp_motion_progress_report.md's "S4" section for the capture
command and a real run's results). Requires tools/start_robot129_vllm.sh to be running.

Object mask: simple red-channel-dominant color threshold against the known cube color
(diffuse_color=(0.88,0.08,0.04) in sim/scripts/run_robot129_ros_webrtc.py) -- this is the
"GT mask or simple color segmentation" placeholder the affordance_region.py docstring and
progress report explicitly call out as still-needed; not a general segmentation model.
Swap this step out for a real segmenter (or a different scene's GT mask) to reuse this
script for anything other than the S2 red cube.

Usage:
  /mnt/HDD4/wyattsheu/env_robot129_research/bin/python \\
      research/scripts/s4_live_affordance_smoke_test.py [scene_id] [output_dir]
"""
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "research" / "src"))

from mpg.grounding import (
    GroundingFailure,
    NoGraspStepError,
    derive_grasp_hint,
    run_grasp_affordance,
    run_stage_a,
    run_stage_b,
)
from mpg.affordance_region import object_points_and_task_region_mask, AffordanceRegionError
from mpg.grasp_candidates import generate_grasp_candidates
from mpg.grasp_contract import write_candidates_json
from mpg.vlm.local import LocalQwenBackend

INSTRUCTION = "pick up the red cube and place it on the green pad"


def main() -> int:
    scene_id = sys.argv[1] if len(sys.argv) > 1 else "s4_live_wrist_capture_01"
    bundle = ROOT / "research" / "data" / "scenes" / scene_id
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / "out" / "grasp_motion" / "s4_live_test"
    out.mkdir(parents=True, exist_ok=True)
    return run(bundle, out)


def run(BUNDLE: Path, OUT: Path) -> int:
    rgb = np.array(Image.open(BUNDLE / "rgb.png").convert("RGB"))
    depth = np.load(BUNDLE / "depth.npy")
    camera_info = json.loads((BUNDLE / "camera_info.json").read_text())
    tf = json.loads((BUNDLE / "tf.json").read_text())
    height, width = camera_info["height"], camera_info["width"]
    k_matrix = camera_info["k"]
    T_world_camera = np.array(tf["matrix"])  # base_frame was "world" at capture time

    print(f"rgb shape={rgb.shape} depth shape={depth.shape} depth range=[{np.nanmin(depth):.3f},{np.nanmax(depth):.3f}]m")

    backend = LocalQwenBackend(
        cache_dir=OUT / "vlm_cache",
        log_path=OUT / "vlm_log.jsonl",
        model="qwen-vl",
        url="http://127.0.0.1:8001/v1/chat/completions",
        phase_id="s4-live-test",
    )
    image = Image.fromarray(rgb)

    # --- Step 0: stage_a/stage_b semantic localization, THEN derive the affordance
    # call's (grasp_object, grasp_hint) from what stage_b actually found -- rather than
    # this script inventing "red cube" / "pick up the red cube" as standalone literals
    # unconnected to any earlier grounding call, which is what every caller in this
    # repository did before derive_grasp_hint() existed (see grounding.py's docstring
    # on it, and docs/progress/grasp_motion_progress_report.md's 2026-09-20 revision
    # note for the audit finding that prompted this). If stage_a/stage_b can't find a
    # GRASP step (wrong instruction, object not visible, etc.), stop here honestly
    # rather than falling back to a hardcoded guess.
    try:
        stage_a_plan, info_stage_a = run_stage_a(backend, image, INSTRUCTION)
        print(f"stage_a: status={stage_a_plan.status} mode={stage_a_plan.mode} "
              f"target={stage_a_plan.target} n_calls={info_stage_a.n_calls}")
        located_steps, info_stage_b = run_stage_b(backend, image, stage_a_plan)
        for step in located_steps:
            print(f"  stage_b {step.spec.step_id} {step.spec.type.value}: "
                  f"{step.point_yx_norm1000} status={step.point_status.value}")
    except GroundingFailure as e:
        print(f"GroundingFailure: stage={e.stage} n_calls={e.n_calls}")
        print("raw_responses:", e.raw_responses)
        return 1

    try:
        grasp_object, grasp_hint = derive_grasp_hint(located_steps, instruction=INSTRUCTION)
    except NoGraspStepError as e:
        print(f"NoGraspStepError: {e} -- stopping here (this is a real, honest result, "
              f"not forced to succeed).")
        return 0
    print(f"derived from stage_b: grasp_object={grasp_object!r} grasp_hint={grasp_hint!r}")

    # --- Step 1: real local VLM call for the grasp affordance region, conditioned on
    # the grasp point stage_b just found (not a fresh independent guess) -----------
    try:
        region, call_info = run_grasp_affordance(
            backend, image, grasp_object=grasp_object, grasp_hint=grasp_hint,
            replicate_id="s4-live-test-01",
        )
    except GroundingFailure as e:
        print(f"GroundingFailure: stage={e.stage} n_calls={e.n_calls}")
        print("raw_responses:", e.raw_responses)
        return 1
    print(f"affordance region: status={region.status} description={region.description!r} "
          f"bbox_yx_norm1000={region.bbox_yx_norm1000} reason_codes={region.reason_codes} "
          f"n_calls={call_info.n_calls} parse_retries={call_info.parse_retries}")
    (OUT / "affordance_region.json").write_text(json.dumps(region.to_json(), indent=2))

    if region.status.value != "localized":
        print("VLM did not localize a graspable region -- stopping here (this is a real, "
              "honest result, not forced to succeed).")
        return 0

    # --- Step 2: object mask via simple color threshold (documented placeholder) -----
    # Cube diffuse_color=(0.88,0.08,0.04) -> vivid red/orange under the scene's lighting.
    r, g, b = rgb[..., 0].astype(int), rgb[..., 1].astype(int), rgb[..., 2].astype(int)
    object_mask = (r > 150) & (r - g > 40) & (r - b > 30)
    n_object_px = int(object_mask.sum())
    print(f"color-threshold object mask: {n_object_px} px")
    Image.fromarray((object_mask * 255).astype(np.uint8)).save(OUT / "object_mask.png")
    if n_object_px == 0:
        print("object mask is empty -- color threshold didn't find the cube, stopping.")
        return 1

    # --- Step 3: bbox ∩ object mask -> 3D task region mask (camera frame) ------------
    try:
        points_cam, task_mask = object_points_and_task_region_mask(
            depth, k_matrix, object_pixel_mask=object_mask, region=region,
            width=width, height=height,
        )
    except AffordanceRegionError as e:
        print(f"AffordanceRegionError: {e}")
        return 1
    print(f"object points: {points_cam.shape[0]}, task-region points: {int(task_mask.sum())}")

    # --- Step 4: camera frame -> world frame (T_world_camera from the captured TF) ---
    points_cam_h = np.concatenate([points_cam, np.ones((points_cam.shape[0], 1))], axis=1)
    points_world = (T_world_camera @ points_cam_h.T).T[:, :3]
    print(f"world-frame object points: x=[{points_world[:,0].min():.3f},{points_world[:,0].max():.3f}] "
          f"y=[{points_world[:,1].min():.3f},{points_world[:,1].max():.3f}] "
          f"z=[{points_world[:,2].min():.3f},{points_world[:,2].max():.3f}]")

    # --- Step 5: real grasp candidate generation (same config as S3/S2) --------------
    candidates = generate_grasp_candidates(
        points_world, frame_id="world", table_z_m=0.0,
        gripper_max_opening_m=0.0690, gripper_min_opening_m=0.006,
        pinch_offset_m=0.125, table_clearance_m=0.015,
        num_yaws=8, pregrasp_standoff_m=0.05,
        edge_band_fraction=0.12, min_edge_points_per_side=3,
        antipodal_angle_tolerance_deg=35.0, max_candidates=8,
        task_region_mask=task_mask,
    )
    accepted = [c for c in candidates if c.accepted]
    print(f"generated {len(candidates)} candidates, {len(accepted)} accepted")
    for c in candidates:
        status = "accepted" if c.accepted else f"rejected {c.rejection_reasons}"
        print(f"  {c.candidate_id}: {status} score={c.score:.4f} tcp={tuple(round(v,4) for v in c.tcp_position_m)}")

    out_path = OUT / "s4_live_candidates.json"
    write_candidates_json(out_path, candidates, scene_id=BUNDLE.name, object_id="red_cube")
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
