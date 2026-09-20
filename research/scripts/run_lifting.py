#!/usr/bin/env python3
"""Measured surface lifting only: no grasp offsets, TCP, collision or IK claims."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from mpg.scene_bundle import validate_scene, _read_depth, sha256_file
from mpg.lifting import deproject_point_yx_norm1000, apply_transform, LiftingError


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not a.output.resolve().is_relative_to(ROOT):
        p.error('output must be within WORK_DIR')
    properties=validate_scene(a.scene)
    source=json.loads((a.plan.parent/'input.json').read_text())
    if source['image_sha256']!=sha256_file(a.scene/'rgb.png'):
        p.error('plan image hash differs from SceneBundle RGB')
    if source.get('coordinate_adapter')=='legacy':
        p.error('baseline W/H lifting is not implemented; compare 2D only')
    camera=json.loads((a.scene/'camera_info.json').read_text())
    if any(abs(float(x))>1e-12 for x in camera.get('d',[])):
        p.error('nonzero distortion: rectification/deprojection adapter required')
    if not np.isfinite(camera['k']).all():
        p.error('nonfinite camera matrix')
    depth,_=_read_depth(a.scene)
    if np.issubdtype(depth.dtype,np.floating) and camera['depth_scale_m'] != 1.0:
        p.error('floating depth requires explicit meter units (depth_scale_m=1)')
    if not camera.get('frame_id'):
        p.error('camera optical frame_id is required')
    depth_m=depth.astype(float)*camera['depth_scale_m']
    plan=json.loads(a.plan.read_text())
    tf=json.loads((a.scene/'tf.json').read_text())
    matrix=np.asarray(tf['matrix']) if tf.get('matrix') is not None else None
    if matrix is not None and (tf.get('camera_frame') != camera['frame_id'] or not tf.get('base_frame')):
        p.error('transform frames do not match CameraInfo')
    if matrix is not None and not np.isfinite(matrix).all():
        p.error('nonfinite transform')
    config=yaml.safe_load((ROOT/'configs/default.yaml').read_text())['lifting']['depth']
    lifted=[]
    for step in plan['steps']:
        row=dict(step_id=step['step_id'],type=step['type'],surface_camera_m=None,surface_base_m=None,
                 camera_frame=camera.get('frame_id'),base_frame=tf.get('base_frame'),
                 geometry_status='UNKNOWN',ik_status='NOT_RUN',execution_status='NOT_EXECUTED')
        if step['point_status']!='localized':
            row.update(lifting_status='UNAVAILABLE',reason='NO_LOCALIZED_POINT')
        else:
            try:
                point,sample=deproject_point_yx_norm1000(step['point_yx_norm1000'],depth_m,camera['k'],
                    width=properties['width'],height=properties['height'],window_k=config['window_k'],
                    valid_fraction_min=config['valid_fraction_min'])
                if not np.isfinite(point).all():
                    raise LiftingError('deproject','nonfinite point')
                row.update(lifting_status='MEASURED_SURFACE_ONLY',surface_camera_m=point.tolist(),
                           depth_valid_fraction=sample.valid_fraction)
                if matrix is not None:
                    row['surface_base_m']=apply_transform(matrix,point).tolist()
            except (ValueError,LiftingError) as exc:
                row.update(lifting_status='FAILED',reason=str(exc))
        lifted.append(row)
    result=dict(scene_id=plan['scene_id'],steps=lifted,depth_scale_m=camera['depth_scale_m'],
        image_sha256=source['image_sha256'],point_semantics='observed_surface_not_free_space_or_TCP',
        transform_status=properties['tf_status'],config=config)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(a.output)


if __name__=='__main__':
    main()
