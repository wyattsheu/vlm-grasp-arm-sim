#!/usr/bin/env python3
"""Synthetic acceptance artifacts; NO model inference or robot operation.

Fixtures use hand-written responses to verify serialization/orchestration only.
Any apparent accuracy from these fixtures is NOT an experimental model result.
"""
from pathlib import Path
import sys
import json
import subprocess
from datetime import datetime, timezone
import numpy as np
from PIL import Image, ImageDraw
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(ROOT/'tests'))
from test_grounding import _stage_a_json, _stage_b_json
from mpg.candidates import propose_grid
from mpg.scene_bundle import sha256_file


def main():
    (ROOT/'logs').mkdir(parents=True, exist_ok=True)
    output=ROOT/'out/validation'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    scene=output/'synthetic_scene'
    scene.mkdir(parents=True)
    def write(path,data): path.write_text(json.dumps(data,indent=2)+'\n')
    image=Image.new('RGB',(320,240),'gray')
    draw=ImageDraw.Draw(image)
    draw.rectangle((90,70,150,150),fill='red')
    draw.rectangle((195,70,250,150),fill='green')
    image.save(scene/'rgb.png')
    Image.fromarray(np.full((240,320),1000,np.uint16)).save(scene/'depth.png')
    write(scene/'camera_info.json',dict(width=320,height=240,k=[300,0,160,0,300,120,0,0,1],
                                      d=[0,0,0,0,0],depth_scale_m=.001,frame_id='synthetic_optical'))
    write(scene/'tf.json',dict(status='UNAVAILABLE',matrix=None,base_frame='synthetic_base',camera_frame='synthetic_optical'))
    write(scene/'joint_states.json',dict(names=['synthetic_joint'],positions=[0]))
    (scene/'instruction.txt').write_text('put the red object in the green region\n')
    names=['rgb.png','depth.png','camera_info.json','tf.json','joint_states.json','instruction.txt']
    write(scene/'manifest.json',dict(capture_complete=True,synthetic=True,sha256={n:sha256_file(scene/n) for n in names}))
    stage_a=_stage_a_json()
    stage_b=_stage_b_json()
    write(output/'ab.json',[stage_a,stage_b])
    single={**stage_a,'steps':[{**a,**b} for a,b in zip(stage_a['steps'],stage_b)]}
    write(output/'single.json',[single])
    write(output/'stage_a.json',stage_a)
    grid=propose_grid(320,240,rows=6,columns=8,seed=129)
    write(output/'candidates_replay.json',[[dict(step_id=s['step_id'],candidate_id=grid[i]['id'],point_status='localized',reason_codes=[])
                                           for i,s in enumerate(stage_a['steps'])]])
    write(output/'baseline.json',[dict(action='grasp',point=[412,388],label='red object'),
                                 dict(action='place',point=[455,690],label='green region')])
    logs=[]
    def run(args):
        result=subprocess.run([sys.executable,*map(str,args)],cwd=ROOT,text=True,capture_output=True)
        logs.append(dict(command=[sys.executable,*map(str,args)],exit_code=result.returncode,stdout=result.stdout,stderr=result.stderr))
        if result.returncode: raise RuntimeError(result.stderr or result.stdout)
    common=['--image',scene/'rgb.png','--instruction','put red object in green region','--scene-id','synthetic',
            '--replicate-id','acceptance0']
    for mode,replay in [('ab','ab.json'),('single','single.json'),('candidates','candidates_replay.json')]:
        args=[ROOT/'scripts/run_grounding.py',*common,'--mode',mode,'--replay',output/replay,'--output',output/mode]
        if mode=='candidates': args+=['--stage-a',output/'ab/stage_a.json']
        run(args)
    run([ROOT/'scripts/run_baseline.py','--image',scene/'rgb.png','--pick-instruction','grasp red object',
        '--place-instruction','place in green region','--scene-id','synthetic','--replicate-id','acceptance0',
        '--replay',output/'baseline.json','--output',output/'baseline'])
    run([ROOT/'scripts/run_lifting.py','--scene',scene,'--plan',output/'ab/plan.json','--output',output/'surfaces.json'])
    write(output/'labels.json',dict(target=dict(bbox_xyxy=[90,70,150,150]),destination=dict(bbox_xyxy=[195,70,250,150])))
    entries=[dict(run_dir=mode,labels='labels.json',group=mode,scene_id='synthetic') for mode in ('ab','single','candidates','baseline')]
    write(output/'evaluation_manifest.json',entries)
    run([ROOT/'scripts/run_eval.py','--manifest',output/'evaluation_manifest.json','--output',output/'evaluation.json'])
    for mode in ('ab','single','candidates','baseline'):
        result=json.loads((output/mode/'result.json').read_text())
        assert result['replay'] and result['actual_calls']==0
    surfaces=json.loads((output/'surfaces.json').read_text())
    assert all(s['surface_base_m'] is None and s['geometry_status']=='UNKNOWN' for s in surfaces['steps'])
    write(output/'acceptance.json',dict(status='PASS',scope='SYNTHETIC_REPLAY_ONLY',actual_model_calls=0,robot_commands=0,commands=logs))
    write(ROOT/'logs/offline_acceptance_20260912.json',dict(status='PASS',artifact_dir=str(output),
        scope='SYNTHETIC_REPLAY_ONLY',actual_model_calls=0,robot_commands=0,commands_run=len(logs)))
    print(output)


if __name__=='__main__':
    main()
