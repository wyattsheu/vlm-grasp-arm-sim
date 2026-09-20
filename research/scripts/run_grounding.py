#!/usr/bin/env python3
"""Isolated offline-image grounding with complete per-request artifacts.

Default replay makes NO model requests. Live local/Gemini are explicit options.
This script contains no robot interface or motion code.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time
import subprocess

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from PIL import Image
import yaml
from mpg.grounding import run_stage_a, run_stage_b, run_single_call
from mpg.schema import StageAPlan, Meta, merge_plan, PlanStatus, ParseError
from mpg.vlm.base import QueryResult
from mpg.viz import render_plan_overlay
from mpg.candidates import propose_grid, render_candidates, candidate_prompt, parse_candidates


def write(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n")


def source_identity():
    digest=hashlib.sha256()
    files=sorted([*ROOT.glob('src/**/*.py'),*ROOT.glob('scripts/*.py')])
    for path in files:
        digest.update(str(path.relative_to(ROOT)).encode()+b'\0'+path.read_bytes()+b'\0')
    head=subprocess.run(['git','rev-parse','HEAD'],cwd=ROOT,capture_output=True,text=True)
    return dict(git_head=head.stdout.strip() if head.returncode==0 else None,
                source_tree_sha256=digest.hexdigest())


class RecordedBackend:
    """Persist inputs and raw outputs even if downstream parsing fails."""
    def __init__(self, output, backend=None, replay=None):
        self.output, self.backend = output, backend
        self.replay = iter(replay) if replay is not None else None
        self.records = []

    def query(self, image, prompt, *, replicate_id=""):
        index = len(self.records)
        image_path = self.output / f"request_{index:03d}.png"
        image.save(image_path)
        (self.output/f"request_{index:03d}.txt").write_text(prompt)
        row = dict(index=index, replicate_id=replicate_id,
                   prompt_sha256=hashlib.sha256(prompt.encode()).hexdigest(),
                   image_sha256=hashlib.sha256(image_path.read_bytes()).hexdigest())
        self.records.append(row)
        try:
            if self.replay is not None:
                raw = next(self.replay)
                raw = raw if isinstance(raw, str) else json.dumps(raw)
                result = QueryResult(raw, 0., True, "REPLAY_NOT_INFERENCE")
            else:
                result = self.backend.query(image, prompt, replicate_id=replicate_id)
            row.update(asdict(result), status="returned", replay=self.replay is not None)
            (self.output/f"response_{index:03d}.txt").write_text(result.raw_text)
            return result
        except Exception as exc:
            row.update(status="failed", error_class=type(exc).__name__)
            raise
        finally:
            write(self.output/"requests.json", self.records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--mode", choices=["ab", "single", "candidates"], default="ab")
    parser.add_argument("--backend", choices=["replay", "local", "gemini"], default="replay")
    parser.add_argument("--replay", type=Path, help="JSON list of raw response strings or parsed responses")
    parser.add_argument("--stage-a", type=Path, help="Fixed Stage A checkpoint for paired localization ablation")
    parser.add_argument("--config", type=Path, default=ROOT/"configs/default.yaml")
    parser.add_argument("--replicate-id", required=True)
    parser.add_argument("--phase-id", default="phase2")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = (args.output or ROOT/"out/runs"/datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")).resolve()
    if not output.is_relative_to(ROOT):
        parser.error("output must be within WORK_DIR")
    if args.backend == "replay" and args.replay is None:
        parser.error("replay backend requires --replay")
    if args.mode == "single" and args.stage_a:
        parser.error("single-call does not accept a fixed Stage A")
    config = yaml.safe_load(args.config.read_text())
    image = Image.open(args.image).convert("RGB")
    if args.stage_a:
        checkpoint_input = args.stage_a.parent / "input.json"
        if not checkpoint_input.is_file():
            parser.error("fixed Stage A requires adjacent input.json provenance")
        checkpoint = json.loads(checkpoint_input.read_text())
        if (checkpoint.get("image_sha256") != hashlib.sha256(args.image.read_bytes()).hexdigest()
            or checkpoint.get("instruction") != args.instruction
            or checkpoint.get("scene_id") != args.scene_id):
            parser.error("fixed Stage A image/instruction/scene differs from current run")
    replay = json.loads(args.replay.read_text()) if args.replay else None
    if args.backend == "replay" and not isinstance(replay, list):
        parser.error("replay file must contain a list")
    output.mkdir(parents=True, exist_ok=False)
    write(output/"config.json", config)
    write(output/"input.json", {**{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
         "image_sha256": hashlib.sha256(args.image.read_bytes()).hexdigest(),
         "stage_a_sha256": hashlib.sha256(args.stage_a.read_bytes()).hexdigest() if args.stage_a else None,
         "image_size": list(image.size), "execution_status":"NOT_EXECUTED"})
    write(output/"source.json",source_identity())
    recorder = RecordedBackend(output, replay=replay if args.backend == "replay" else None)
    started = time.monotonic()
    retry_count = 0
    try:
        if args.backend != "replay":
            from mpg.vlm import LocalQwenBackend, GeminiBackend
            cls = LocalQwenBackend if args.backend == "local" else GeminiBackend
            recorder.backend = cls(cache_dir=ROOT/"cache", log_path=ROOT/"logs/vlm_calls.jsonl",
                                   phase_id=args.phase_id, **config["generation"])
        kwargs = dict(max_retries=config["parsing"]["max_format_retries"], replicate_id=args.replicate_id)
        if args.mode == "single":
            stage_a, located, info = run_single_call(recorder, image, args.instruction, **kwargs)
            retry_count += info.parse_retries
        else:
            if args.stage_a:
                stage_a = StageAPlan.from_json_text(args.stage_a.read_text())
            else:
                stage_a, info = run_stage_a(recorder, image, args.instruction, **kwargs)
                retry_count += info.parse_retries
            write(output/"stage_a.json", stage_a.to_json())
            if stage_a.status is not PlanStatus.READY:
                from mpg.schema import LocatedStep, PointStatus
                located = tuple(LocatedStep(spec, None, PointStatus.UNAVAILABLE, ["MODEL_ABSTAINED"]) for spec in stage_a.steps)
            elif args.mode == "ab":
                located, info = run_stage_b(recorder, image, stage_a, **kwargs)
                retry_count += info.parse_retries
            else:
                candidates = propose_grid(*image.size, **config["candidates"])
                write(output/"candidates.json", candidates)
                marked = render_candidates(image, candidates)
                marked.save(output/"candidates.png")
                prompt = candidate_prompt(stage_a, candidates)
                for attempt in range(kwargs["max_retries"]+1):
                    raw = recorder.query(marked, prompt, replicate_id=f"{args.replicate_id}:candidate{attempt}").raw_text
                    try:
                        located = parse_candidates(raw, stage_a, candidates)
                        break
                    except ParseError:
                        if attempt == kwargs["max_retries"]:
                            raise
                        retry_count += 1
                        prompt += "\nReturn only valid JSON with existing candidate IDs in the fixed step order."
        records = recorder.records
        n_calls = sum(not r.get("cache_hit", False) and not r.get("replay", False) for r in records)
        plan = merge_plan(scene_id=args.scene_id, instruction=args.instruction,
            backend=args.backend, stage_a_plan=stage_a, located_steps=located,
            meta=Meta(sum(r.get("latency_s",0.) for r in records), n_calls, retry_count))
        write(output/"plan.json", plan.to_json())
        render_plan_overlay(image, plan).save(output/"overlay.png")
        write(output/"result.json", {"status":"PARSED", "semantic_status":stage_a.status.value,
            "localization_status":"COMPLETE" if located and all(s.point_yx_norm1000 is not None for s in located) else "INCOMPLETE",
            "executor_supported":not stage_a.unsupported_by_executor,
            "geometry_status":"NOT_RUN", "ik_status":"NOT_RUN", "execution_status":"NOT_EXECUTED",
            "replay":args.backend == "replay", "wall_latency_s":time.monotonic()-started,
            "actual_calls":n_calls, "cache_hits":sum(r.get("cache_hit",False) and not r.get("replay",False) for r in records)})
        print(output)
        return 0
    except Exception as exc:
        write(output/"result.json", {"status":"FAILED", "error_class":type(exc).__name__,
            "replay":args.backend == "replay",
            "geometry_status":"NOT_RUN", "execution_status":"NOT_EXECUTED", "wall_latency_s":time.monotonic()-started})
        print(f"Run failed ({type(exc).__name__}); artifacts: {output}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
