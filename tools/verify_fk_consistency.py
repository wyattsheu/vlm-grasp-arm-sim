#!/usr/bin/env python3
"""Compare URDF FK against the Isaac-measured gripper_base pose from Lesson 6."""
import json,math,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
def rpy(v):
 r,p,y=v; cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
 return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
def axis_angle(axis,q):
 a=np.asarray(axis,float);a/=np.linalg.norm(a);K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
 return np.eye(3)+math.sin(q)*K+(1-math.cos(q))*(K@K)
def quat_xyzw(q):
 x,y,z,w=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
urdf=ET.parse(ROOT/'ros2_ws/src/robot129_description/urdf/robot129.urdf').getroot(); joints={j.find('child').attrib['link']:j for j in urdf.findall('joint')}
probe=json.loads((ROOT/'out/lesson_06/gripper_probe.json').read_text()); sample=probe['measurements']['open']; q=dict(zip(probe['joint_names'],sample['joint_position']))
T=np.eye(4)
for child in ['base_link','link1','link2','link3','link4','link5','link6','gripper_base']:
 j=joints[child]; o=j.find('origin'); xyz=[float(x) for x in o.attrib.get('xyz','0 0 0').split()]; rp=[float(x) for x in o.attrib.get('rpy','0 0 0').split()]
 A=np.eye(4);A[:3,:3]=rpy(rp);A[:3,3]=xyz;T=T@A
 if j.attrib['type']=='revolute':
  B=np.eye(4);B[:3,:3]=axis_angle([float(x) for x in j.find('axis').attrib['xyz'].split()],q[j.attrib['name']]);T=T@B
isaac=sample['bodies']['gripper_base']; p=np.array(isaac['position']); R=quat_xyzw(isaac['quaternion_xyzw']); trans=float(np.linalg.norm(T[:3,3]-p)); angle=float(math.acos(np.clip((np.trace(T[:3,:3].T@R)-1)/2,-1,1)))
checks={'translation_under_1mm':trans<.001,'rotation_under_0_1deg':angle<math.radians(.1)}
report={'status':'PASS' if all(checks.values()) else 'FAIL','link':'gripper_base','joint_state_source':'out/lesson_06/gripper_probe.json','urdf_position_m':T[:3,3].tolist(),'isaac_position_m':p.tolist(),'translation_error_m':trans,'rotation_error_rad':angle,'rotation_error_deg':math.degrees(angle),'checks':checks}
out=ROOT/'out/lesson_05/fk_consistency.json';out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));raise SystemExit(0 if report['status']=='PASS' else 1)
