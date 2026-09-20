#!/usr/bin/env python3
"""Numerically validate an active Robot129 camera calibration triplet."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
DEFAULTS = (
    ROOT / 'calibration/camera_intrinsics.yaml',
    ROOT / 'calibration/extrinsics.yaml',
    ROOT / 'calibration/camera_registration.yaml',
)


def load_yaml(path):
    with path.open() as stream:
        return yaml.safe_load(stream) or {}


def validate(intrinsics, extrinsics, registration):
    checks = {}

    def record(name, condition, detail=None):
        checks[name] = {'pass': bool(condition)}
        if detail is not None:
            checks[name]['detail'] = detail

    record('all_status_verified', all(item.get('status') == 'VERIFIED' for item in
                                      (intrinsics, extrinsics, registration)))
    color = intrinsics.get('streams', {}).get('color', {})
    depth = intrinsics.get('streams', {}).get('aligned_depth_to_color', {})
    width, height = color.get('width'), color.get('height')
    record('positive_image_dimensions', isinstance(width, int) and width > 0 and
           isinstance(height, int) and height > 0)
    record('aligned_depth_same_pixel_grid',
           depth.get('width') == width and depth.get('height') == height)
    scale = depth.get('depth_scale_m')
    record('positive_depth_scale', isinstance(scale, (int, float)) and scale > 0)

    k_raw = intrinsics.get('camera_info', {}).get('k')
    try:
        k = np.asarray(k_raw, dtype=float).reshape(3, 3)
        k_ok = np.isfinite(k).all() and k[0, 0] > 0 and k[1, 1] > 0 and k[2, 2] != 0
    except (TypeError, ValueError):
        k = None
        k_ok = False
    record('K_is_finite_3x3_with_positive_focal_length', k_ok)
    principal_ok = bool(k_ok and 0 <= k[0, 2] < width and 0 <= k[1, 2] < height)
    record('principal_point_inside_image', principal_ok)

    transform = extrinsics.get('verified_transform', {})
    matrix_raw = transform.get('matrix_4x4_row_major')
    try:
        matrix = np.asarray(matrix_raw, dtype=float).reshape(4, 4)
        finite = np.isfinite(matrix).all()
    except (TypeError, ValueError):
        matrix = None
        finite = False
    record('extrinsic_is_finite_4x4', finite)
    if finite:
        rotation = matrix[:3, :3]
        ortho_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3)))
        determinant = float(np.linalg.det(rotation))
        record('extrinsic_homogeneous_bottom_row',
               np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8), matrix[3].tolist())
        record('rotation_is_SO3', ortho_error < 1e-3 and abs(determinant - 1.0) < 1e-3,
               {'orthonormality_error': ortho_error, 'determinant': determinant})
    else:
        record('extrinsic_homogeneous_bottom_row', False)
        record('rotation_is_SO3', False)
    record('transform_direction_explicit', transform.get('direction') == 'T_parent_child')

    frames = registration.get('frames', {})
    info_frame = intrinsics.get('camera_info', {}).get('frame_id')
    record('camera_info_uses_color_optical_frame',
           bool(info_frame) and info_frame == frames.get('color_optical_frame'))
    record('extrinsic_child_matches_optical_frame',
           transform.get('child_frame') == frames.get('color_optical_frame'))
    record('extrinsic_parent_matches_registration',
           transform.get('parent_frame') == frames.get('parent_link'))
    streams = registration.get('streams', {})
    record('registration_dimensions_match_intrinsics',
           streams.get('width') == width and streams.get('height') == height)

    validation = registration.get('validation', {})
    metric_names = ('mean_reprojection_error_px', 'p95_reprojection_error_px',
                    'depth_scale_relative_error', 'known_point_3d_error_m')
    metrics = [validation.get(name) for name in metric_names]
    record('validation_metrics_finite_nonnegative', all(
        isinstance(value, (int, float)) and np.isfinite(value) and value >= 0
        for value in metrics))

    failed = [name for name, result in checks.items() if not result['pass']]
    return {'status': 'PASS' if not failed else 'FAIL', 'checks': checks,
            'failed_checks': failed}


def self_test():
    intrinsics = {
        'status': 'VERIFIED',
        'streams': {'color': {'width': 640, 'height': 480},
                    'aligned_depth_to_color': {'width': 640, 'height': 480,
                                               'depth_scale_m': 0.001}},
        'camera_info': {'frame_id': 'cam_optical',
                        'k': [600, 0, 320, 0, 600, 240, 0, 0, 1]},
    }
    extrinsics = {'status': 'VERIFIED', 'verified_transform': {
        'parent_frame': 'tool0', 'child_frame': 'cam_optical',
        'direction': 'T_parent_child',
        'matrix_4x4_row_major': np.eye(4).tolist()}}
    registration = {
        'status': 'VERIFIED',
        'frames': {'parent_link': 'tool0', 'color_optical_frame': 'cam_optical'},
        'streams': {'width': 640, 'height': 480},
        'validation': {'mean_reprojection_error_px': 0.2,
                       'p95_reprojection_error_px': 0.5,
                       'depth_scale_relative_error': 0.01,
                       'known_point_3d_error_m': 0.004},
    }
    good = validate(intrinsics, extrinsics, registration)
    extrinsics['verified_transform']['matrix_4x4_row_major'][0][0] = 2.0
    bad = validate(intrinsics, extrinsics, registration)
    passed = good['status'] == 'PASS' and bad['status'] == 'FAIL'
    return {'status': 'PASS' if passed else 'FAIL',
            'valid_fixture': good['status'], 'invalid_rotation_fixture': bad['status']}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--intrinsics', type=Path, default=DEFAULTS[0])
    parser.add_argument('--extrinsics', type=Path, default=DEFAULTS[1])
    parser.add_argument('--registration', type=Path, default=DEFAULTS[2])
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        report = self_test()
    else:
        paths = (args.intrinsics, args.extrinsics, args.registration)
        missing = [str(path) for path in paths if not path.is_file()]
        report = ({'status': 'NOT_READY', 'missing_files': missing} if missing else
                  validate(*(load_yaml(path) for path in paths)))
    output = ROOT / 'out/readiness/camera_calibration_validation.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['status'] == 'PASS' else 2


if __name__ == '__main__':
    sys.exit(main())
