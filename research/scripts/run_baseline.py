#!/usr/bin/env python3
"""Two single-point requests, original system/user prompts and W/H coordinates.

Independent adapter based on read-only gemini_client.py:160-218,
source commit 583ada7af4d4c52420d82edac34cdec919228712. Compared with legacy,
malformed/nonfinite values raise a typed run failure instead of crashing or
becoming None; this difference is recorded, not called exact reproduction.
"""
import argparse
import json
import math
import re
from pathlib import Path
import time
import hashlib
from run_grounding import ROOT, RecordedBackend, write, source_identity
from PIL import Image
import yaml
from mpg.viz import render_baseline_overlay
from mpg.schema import pixel_from_norm1000


def parse_point(raw):
    text=re.sub(r'```(?:json)?','',raw).strip().rstrip('`').strip()
    try:
        d=json.loads(text)
    except json.JSONDecodeError:
        match=re.search(r'\{.*\}',text,re.DOTALL)
        if not match:
            raise ValueError('baseline invalid JSON')
        d=json.loads(match.group())
    if not isinstance(d,dict):
        raise ValueError('baseline requires an object')
    if d.get('action') is None or d.get('point') is None:
        return None
    if d['action'] not in ('grasp','handover','place'):
        raise ValueError('baseline unknown action')
    p=d['point']
    if not isinstance(p,list) or len(p)!=2:
        raise ValueError('baseline invalid point')
    p=[float(x) for x in p]
    if not all(math.isfinite(x) for x in p):
        raise ValueError('baseline nonfinite point')
    return p


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--image',type=Path,required=True)
    p.add_argument('--pick-instruction',required=True)
    p.add_argument('--place-instruction',required=True)
    p.add_argument('--scene-id',required=True)
    p.add_argument('--replicate-id',required=True)
    p.add_argument('--backend',choices=['replay','local','gemini'],default='replay')
    p.add_argument('--replay',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--phase-id',default='phase2')
    a=p.parse_args()
    out=a.output.resolve()
    if not out.is_relative_to(ROOT):
        p.error('output must be in WORK_DIR')
    if a.backend=='replay' and a.replay is None:
        p.error('replay requires --replay')
    image=Image.open(a.image).convert('RGB')
    text=(ROOT/'prompts/baseline_copied.txt').read_text()
    system,template=text.split('=== SYSTEM PROMPT (verbatim) ===',1)[1].split('=== USER TEMPLATE (verbatim) ===')
    system,template=system.strip(),template.strip()
    config=yaml.safe_load((ROOT/'configs/default.yaml').read_text())
    out.mkdir(parents=True,exist_ok=False)
    (out/'system_prompt.txt').write_text(system)
    write(out/'config.json',config)
    write(out/'source.json',source_identity())
    write(out/'input.json',{'scene_id':a.scene_id,'image_size':list(image.size),'coordinate_adapter':'legacy',
        'image_sha256':hashlib.sha256(a.image.read_bytes()).hexdigest(),
        'pick_instruction':a.pick_instruction,'place_instruction':a.place_instruction,
        'replicate_id':a.replicate_id,'backend':a.backend,
        'baseline_deviations':['REST transport instead of SDK','malformed/nonfinite responses recorded as failures',
            'pick/place instruction decomposition supplied explicitly by experimenter']})
    recorder=RecordedBackend(out)
    start=time.monotonic()
    try:
        if a.backend=='replay':
            recorder.replay=iter(json.loads(a.replay.read_text()))
        else:
            from mpg.vlm import LocalQwenBackend,GeminiBackend
            cls=LocalQwenBackend if a.backend=='local' else GeminiBackend
            recorder.backend=cls(cache_dir=ROOT/'cache',log_path=ROOT/'logs/vlm_calls.jsonl',
                system_prompt=system,phase_id=a.phase_id,**config['generation'])
        points=[]
        for i,instruction in enumerate((a.pick_instruction,a.place_instruction)):
            result=recorder.query(image,template.replace('{instruction}',instruction),replicate_id=f'{a.replicate_id}:baseline{i}')
            points.append(parse_point(result.raw_text))
        steps=[{'step_id':f'b{i}','type':kind,'point_yx_norm1000':pt,
                'point_status':'localized' if pt is not None else 'unavailable'}
               for i,(kind,pt) in enumerate(zip(('GRASP','RELEASE'),points))]
        write(out/'plan.json',{'scene_id':a.scene_id,'stage_a':{'status':'ready' if all(pt is not None for pt in points) else 'abstain'},
            'steps':steps,'backend':a.backend,'schema_version':'baseline_two_points_v1'})
        pixels=[pixel_from_norm1000(pt,width=image.width,height=image.height,adapter='legacy') if pt is not None else None for pt in points]
        render_baseline_overlay(image,target_point_px=pixels[0],destination_point_px=pixels[1]).save(out/'overlay.png')
        write(out/'result.json',{'status':'PARSED','replay':a.backend=='replay','execution_status':'NOT_EXECUTED',
            'wall_latency_s':time.monotonic()-start,'actual_calls':sum(not r.get('cache_hit',False) for r in recorder.records)})
        print(out)
        return 0
    except Exception as exc:
        write(out/'result.json',{'status':'FAILED','error_class':type(exc).__name__,'replay':a.backend=='replay',
             'execution_status':'NOT_EXECUTED'})
        print(f'Baseline failed ({type(exc).__name__}): {out}')
        return 2


if __name__=='__main__':
    raise SystemExit(main())
