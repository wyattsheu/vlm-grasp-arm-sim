#!/usr/bin/env python3
"""Run MPG grounding replay and 2D-to-3D lifting on an actual Isaac RGB-D SceneBundle."""
import json, os, subprocess, sys
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
BUNDLE=ROOT.parent
sys.path.insert(0,str(ROOT/'src'))
from mpg.scene_bundle import validate_scene

def write(path,value): path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def norm(y,x,h,w): return [round(y/h*1000,3),round(x/w*1000,3)]

def main():
 scene=BUNDLE/'out/integrated_demo/scene_bundle'; properties=validate_scene(scene)
 rgb=np.asarray(Image.open(scene/'rgb.png').convert('RGB')); depth=np.load(scene/'depth.npy').squeeze(); h,w=depth.shape
 score=rgb[:,:,0].astype(int)-np.maximum(rgb[:,:,1],rgb[:,:,2]).astype(int)
 ys,xs=np.where(score>40)
 if len(xs)<100: raise RuntimeError('red cube segmentation did not produce enough pixels')
 grasp=(float(ys.mean()),float(xs.mean()))
 waypoint=(max(40.0,grasp[0]-90.0), min(w-40.0,grasp[1]+100.0))
 release=(0.75*h,0.70*w)
 for label,(y,x) in [('grasp',grasp),('waypoint',waypoint),('release',release)]:
  if not np.isfinite(depth[int(y),int(x)]) or depth[int(y),int(x)]<=0: raise RuntimeError(f'{label} depth invalid')
 stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
 out=ROOT/'out/robot129_sim_replay'/stamp; out.mkdir(parents=True)
 stage_a={
  'schema_version':'0','view_description':'Robot 129 holds a red cube above a simulated ground plane.',
  'mode':'pick','target':{'name':'red cube','visibility':'visible'},
  'destination':{'name':'release area on the ground','visibility':'visible'},'tool':None,
  'status':'ready','reason_codes':[],
  'steps':[
   {'step_id':'s0','type':'GRASP','desc':'red cube center','reference_kind':'grasp_contact','geometric_meaning':'visible center of the simulated red cube'},
   {'step_id':'s1','type':'WAYPOINT','desc':'free-space image anchor','reference_kind':'transit_anchor','geometric_meaning':'visible transit anchor above and right of the cube'},
   {'step_id':'s2','type':'RELEASE','desc':'ground release area','reference_kind':'destination_support','geometric_meaning':'visible simulated ground support point'}]}
 stage_b=[
  {'step_id':'s0','point_yx_norm1000':norm(*grasp,h,w),'point_status':'localized','reason_codes':[]},
  {'step_id':'s1','point_yx_norm1000':norm(*waypoint,h,w),'point_status':'localized','reason_codes':[]},
  {'step_id':'s2','point_yx_norm1000':norm(*release,h,w),'point_status':'localized','reason_codes':[]}]
 replay=out/'replay.json'; write(replay,[stage_a,stage_b])
 run=out/'grounding'
 command=[sys.executable,str(ROOT/'scripts/run_grounding.py'),'--image',str(scene/'rgb.png'),
  '--instruction',(scene/'instruction.txt').read_text().strip(),'--scene-id','robot129_isaac_capture_220',
  '--mode','ab','--backend','replay','--replay',str(replay),'--replicate-id','robot129sim0','--output',str(run)]
 subprocess.run(command,cwd=ROOT,check=True)
 surfaces=out/'surfaces.json'
 subprocess.run([sys.executable,str(ROOT/'scripts/run_lifting.py'),'--scene',str(scene),'--plan',str(run/'plan.json'),'--output',str(surfaces)],cwd=ROOT,check=True)
 lifting=json.loads(surfaces.read_text()); steps=lifting['steps']
 checks={'scene_bundle_valid':True,'tf_available':properties['tf_status']=='AVAILABLE','red_pixels_over_100':len(xs)>100,
  'three_localized_steps':len(steps)==3,'all_camera_points_measured':all(s.get('surface_camera_m') is not None for s in steps),
  'all_base_points_transformed':all(s.get('surface_base_m') is not None for s in steps),'actual_model_calls_zero':True,'robot_commands_zero':True}
 report={'status':'PASS' if all(checks.values()) else 'FAIL','scope':'ISAAC_RGBD_REPLAY_SOFTWARE_ONLY',
  'scene':str(scene),'grounding_run':str(run),'overlay':str(run/'overlay.png'),'surfaces':str(surfaces),
  'red_bbox_xyxy':[int(xs.min()),int(ys.min()),int(xs.max()),int(ys.max())],
  'points_yx_norm1000':{s['step_id']:s['point_yx_norm1000'] for s in stage_b},
  'actual_model_calls':0,'robot_commands':0,'checks':checks}
 write(out/'acceptance.json',report); write(ROOT/'out/robot129_sim_replay/latest.json',{'run_dir':str(out),'status':report['status']})
 print(json.dumps(report,indent=2)); return 0 if report['status']=='PASS' else 1
if __name__=='__main__': raise SystemExit(main())
