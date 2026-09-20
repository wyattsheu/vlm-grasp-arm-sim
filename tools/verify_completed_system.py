#!/usr/bin/env python3
"""Aggregate reproducible Robot 129 server-side simulation acceptance artifacts."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
LATEST = json.loads((ROOT / 'research/out/robot129_sim_replay/latest.json').read_text())
REPORTS = {
    'lesson_03_description': ROOT / 'out/lesson_03/urdf_validation.json',
    'lesson_05_usd_import': ROOT / 'sim/assets/robot129/import_report.json',
    'lesson_05_fk': ROOT / 'out/lesson_05/fk_consistency.json',
    'lesson_06_physx_grasp': ROOT / 'out/lesson_06/physics_grasp/physics_grasp_report.json',
    'lesson_08_ros_isolation': ROOT / 'out/lesson_08/ros_isolation.json',
    'lesson_08_ros_isaac': ROOT / 'out/lesson_08/ros_isaac_roundtrip.json',
    'lesson_08_ros2_control': ROOT / 'out/lesson_08/ros2_control_mock.json',
    'lesson_08_controller_guards': ROOT / 'out/lesson_08/ros2_control_guards.json',
    'lesson_09_moveit': ROOT / 'out/lesson_09/moveit_fixed_plan.json',
    'lesson_09_mtc': ROOT / 'out/lesson_09/mtc_plan.json',
    'lesson_10_mpg_replay': Path(LATEST['run_dir']) / 'acceptance.json',
    'lesson_11_thor_loopback': ROOT / 'out/lesson_11/thor_adapter_loopback.json',
    'webrtc_server_ready': ROOT / 'out/webrtc_robot129/acceptance.json',
    'final_video': ROOT / 'out/final_demo/video_report.json',
}
ARTIFACTS = {
    'robot129_xacro': ROOT / 'ros2_ws/src/robot129_description/urdf/robot129.urdf.xacro',
    'robot129_usd': ROOT / 'sim/assets/robot129/robot129/robot129.usda',
    'scene_rgb': ROOT / 'out/integrated_demo/scene_bundle/rgb.png',
    'scene_depth': ROOT / 'out/integrated_demo/scene_bundle/depth.npy',
    'final_video': ROOT / 'out/final_demo/robot129_full_simulation.mp4',
    'contact_sheet': ROOT / 'out/final_demo/contact_sheet.png',
}


def main():
    checks = {}
    summaries = {}
    for name, path in REPORTS.items():
        exists = path.is_file()
        data = json.loads(path.read_text()) if exists else {}
        checks[f'{name}_report_exists'] = exists
        checks[f'{name}_pass'] = data.get('status') == 'PASS'
        summaries[name] = {'path': str(path.relative_to(ROOT)), 'status': data.get('status', 'MISSING')}
    for name, path in ARTIFACTS.items():
        checks[f'{name}_exists_nonempty'] = path.is_file() and path.stat().st_size > 0
    video = ARTIFACTS['final_video']
    report = {
        'status': 'PASS' if all(checks.values()) else 'FAIL',
        'scope': 'ROBOT129_SERVER_SIDE_SIMULATION_ACCEPTANCE',
        'simulation_only': True,
        'checks': checks,
        'reports': summaries,
        'video_sha256': hashlib.sha256(video.read_bytes()).hexdigest() if video.exists() else None,
        'hardware_commands': 0,
        'not_run': {
            'live_vllm_model': 'No vLLM/transformers install and no model cache; requires a separately approved large install/download.',
            'thor_connected_rehearsal': 'Thor host/network was not connected; server-local adapter loopback passed.',
            'real_robot': 'CAN, Piper SDK, RealSense USB and production ROS domain remain disconnected.',
            'calibration': 'Real wrist-camera extrinsics/intrinsics and measured controller/contact parameters remain unverified.',
        },
    }
    output = ROOT / 'out/final_acceptance.json'
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
