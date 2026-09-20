#!/usr/bin/env python3
"""Evaluate an explicit expected-run manifest; missing/failed runs stay in denominators.

Manifest: [{"run_dir":"...", "labels":"...", "group":"ab", "scene_id":"s1"}].
Paths are relative to the manifest. Outputs are only coarse bbox hit metrics,
never grasp success, collision validity, or independent 3D accuracy.
"""
import argparse
import json
from pathlib import Path
import sys
import math
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from mpg.schema import pixel_from_norm1000


def hit(point, bbox):
    if bbox is None:
        return None
    if len(bbox) != 4 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in bbox):
        raise ValueError("invalid bbox")
    x0,y0,x1,y1 = bbox
    if x1 < x0 or y1 < y0:
        raise ValueError("reversed bbox")
    return bool(point is not None and x0 <= point[0] <= x1 and y0 <= point[1] <= y1)


def evaluate(manifest_path):
    entries = json.loads(manifest_path.read_text())
    if not isinstance(entries,list) or not entries:
        raise ValueError("expected non-empty manifest list")
    rows=[]
    seen=set()
    for entry in entries:
        run=(manifest_path.parent/entry['run_dir']).resolve()
        if run in seen:
            raise ValueError("duplicate run directory in manifest")
        seen.add(run)
        labels=json.loads((manifest_path.parent/entry['labels']).read_text())
        row=dict(scene_id=entry['scene_id'], group=entry['group'], parsed=False,
                 semantic_ready=False, replay=None, status="MISSING", target_hit=None,destination_hit=None)
        for role in ('target','destination'):
            # Annotated failures score false, unannotated points remain unknown.
            row[role+'_hit']=hit(None, labels.get(role,{}).get('bbox_xyxy'))
        if (run/'result.json').exists():
            result=json.loads((run/'result.json').read_text())
            row['status']=result['status']
            row['replay']=result.get('replay')
            if result['status']=='PARSED':
                plan=json.loads((run/'plan.json').read_text())
                inp=json.loads((run/'input.json').read_text())
                if plan['scene_id'] != entry['scene_id']:
                    raise ValueError("manifest/plan scene_id mismatch")
                row['parsed']=True
                row['semantic_ready']=plan['stage_a']['status']=='ready'
                w,h=inp['image_size']
                for role,kind in [('target','GRASP'),('destination','RELEASE')]:
                    points=[s for s in plan['steps'] if s['type']==kind]
                    p=None
                    if points and points[0]['point_status']=='localized':
                        p=pixel_from_norm1000(points[0]['point_yx_norm1000'],width=w,height=h,adapter=inp.get('coordinate_adapter','new'))
                    row[role+'_hit']=hit(p,labels.get(role,{}).get('bbox_xyxy'))
        rows.append(row)
    groups={}
    for group in sorted(set(r['group'] for r in rows)):
        selected=[r for r in rows if r['group']==group]
        modes={r['replay'] for r in selected if r['replay'] is not None}
        if modes == {True, False}:
            raise ValueError('cannot aggregate replay and real inference in the same group')
        summary=dict(expected_runs=len(selected), distinct_scenes=len(set(r['scene_id'] for r in selected)),
                     parsed=sum(r['parsed'] for r in selected),missing=sum(r['status']=='MISSING' for r in selected))
        for role in ('target','destination'):
            values=[r[role+'_hit'] for r in selected if r[role+'_hit'] is not None]
            summary[role+'_bbox_hit']={"successes":sum(values),"annotated_denominator":len(values),
                                      "rate":sum(values)/len(values) if values else None}
        groups[group]=summary
    return dict(metric_scope="REPLAY_SOFTWARE_ONLY" if any(r['replay'] for r in rows) else "COARSE_2D_ONLY", rows=rows,groups=groups,
                warning="Replay is software verification, not model accuracy. Repeats are clustered by scene.")


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not a.output.resolve().is_relative_to(ROOT):
        p.error('output must be inside WORK_DIR')
    result=evaluate(a.manifest.resolve())
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(a.output)
