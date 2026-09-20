#!/usr/bin/env python3
"""Report Robot 129 layer readiness without accepting empty or template data."""
import argparse
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]

TARGETS = {
    'vlm': {
        'path': 'environment/vlm_source_inventory.yaml',
        'status': {'VERIFIED', 'READY'},
        'required': [
            'versions.vllm', 'versions.transformers', 'versions.pytorch',
            'models.qwen.exact_revision', 'models.qwen.served_model_name',
            'models.molmo.exact_revision', 'models.molmo.served_model_name',
            'serving.dtype', 'serving.max_model_len', 'serving.limit_mm_per_prompt',
            'api_contract.point_coordinate_convention', 'api_contract.timeout_s',
            'security.bind_address', 'security.network_scope',
            'provenance.live_source_commit', 'provenance.installed_artifact_hash',
        ],
    },
    'thor': {
        'path': 'deployment/thor_rehearsal.yaml',
        'status': {'APPROVED', 'READY'},
        'required': [
            'thor.ip_on_test_network', 'thor.ros_distro', 'thor.rmw_implementation',
            'thor.source_commit', 'thor.build_command', 'thor.launch_command',
            'server.ip_on_test_network', 'server.adapter_endpoint',
            'network.isolated_vlan_or_direct_link', 'network.allowed_thor_to_server_ports',
            'network.dds_config_path', 'clock.maximum_offset_ms',
            'safety.can_devices_absent_from_container',
            'safety.realsense_devices_absent_from_container',
            'safety.production_domain_not_129_confirmed',
            'rollback.stop_commands', 'rollback.owner',
        ],
    },
    'camera': {
        'files': {
            'calibration/camera_intrinsics.yaml': [
                'device.exact_model_and_revision', 'streams.color.width',
                'streams.color.height', 'streams.color.fps', 'streams.color.encoding',
                'streams.aligned_depth_to_color.width',
                'streams.aligned_depth_to_color.height',
                'streams.aligned_depth_to_color.encoding',
                'streams.aligned_depth_to_color.depth_scale_m',
                'camera_info.frame_id', 'camera_info.distortion_model',
                'camera_info.k', 'camera_info.d', 'camera_info.r', 'camera_info.p',
                'provenance.capture_id', 'provenance.captured_at',
                'provenance.source_host', 'provenance.calibration_file_hash', 'review.reviewed_by',
                'review.reviewed_at',
            ],
            'calibration/extrinsics.yaml': [
                'verified_transform.parent_frame', 'verified_transform.child_frame',
                'verified_transform.direction',
                'verified_transform.matrix_4x4_row_major',
                'required_resolution.calibration_method',
                'required_resolution.calibrated_at',
                'required_resolution.translation_error_m',
                'required_resolution.rotation_error_rad',
                'required_resolution.validation_dataset',
                'review.raw_dataset_sha256', 'review.reviewed_by',
                'review.reviewed_at',
            ],
            'calibration/camera_registration.yaml': [
                'camera_id', 'hardware_revision_ref', 'frames.parent_link',
                'frames.camera_link', 'frames.color_optical_frame',
                'frames.base_frame', 'streams.color_topic',
                'streams.aligned_depth_topic', 'streams.camera_info_topic',
                'streams.width', 'streams.height', 'streams.fps',
                'synchronization.method',
                'synchronization.maximum_rgb_depth_skew_ms',
                'synchronization.maximum_joint_age_ms',
                'validation.mean_reprojection_error_px',
                'validation.p95_reprojection_error_px',
                'validation.known_point_3d_error_m',
                'validation.validation_dataset', 'validation.accepted_by',
                'validation.reviewed_at',
            ],
        },
        'status': {'VERIFIED'},
    },
    'hardware': {
        'files': {
            'calibration/hardware_revision.yaml': [
                'recorded_at', 'recorded_by', 'robot.exact_model',
                'robot.hardware_revision', 'arm.firmware_version',
                'arm.joint_positive_direction_verified',
                'gripper.exact_model', 'gripper.hardware_revision',
                'gripper.total_opening_width_m',
                'provenance.evidence_files', 'provenance.reviewed_by',
                'provenance.reviewed_at',
            ],
            'calibration/physical_measurements.yaml': [
                'measurement_date', 'measured_by', 'tools_used',
                'robot.piper_model', 'robot.hardware_revision',
                'end_effector.flange_to_tcp', 'end_effector.tcp_definition',
                'camera_mount.flange_to_mount', 'uncertainty.linear_m',
                'uncertainty.angular_rad', 'provenance.evidence_files',
                'provenance.reviewed_by', 'provenance.reviewed_at',
            ],
            'calibration/gripper_mapping.yaml': [
                'standard_sim_interface.command_joint',
                'standard_sim_interface.controller_type',
                'standard_sim_interface.command_represents',
                'standard_sim_interface.total_opening_width_m_at_open',
                'standard_sim_interface.total_opening_width_m_at_closed',
                'validation.open_pose', 'validation.closed_pose',
                'validation.fingertip_distance_measurement',
                'provenance.raw_data_paths', 'provenance.reviewed_by',
                'provenance.reviewed_at',
            ],
            'calibration/controller_characterization.yaml': [
                'captured_at', 'operator', 'hardware_revision_ref',
                'software.source_commit', 'software.installed_artifact_hash',
                'software.controller_node', 'software.update_rate_hz',
                'interfaces.command_topic_or_action', 'interfaces.feedback_topic',
                'interfaces.joint_names', 'safety.velocity_limit_fraction',
                'safety.watchdog_timeout_s', 'metrics.command_to_feedback_latency_ms',
                'metrics.tracking_rmse_per_joint', 'metrics.peak_error_per_joint',
                'completion_policy.position_tolerance_per_joint',
                'completion_policy.stable_duration_s',
                'provenance.raw_data_paths', 'provenance.reviewed_by',
                'provenance.reviewed_at',
            ],
            'calibration/contact_characterization.yaml': [
                'captured_at', 'operator', 'hardware_revision_ref',
                'gripper_mapping_ref', 'contact_signal.type',
                'contact_signal.topic_or_api_field', 'contact_signal.sample_rate_hz',
                'objects', 'trials.total', 'trials.success_count',
                'metrics.minimum_reliable_contact_signal',
                'metrics.slip_threshold_m', 'metrics.repeatability_rate',
                'provenance.raw_data_paths', 'provenance.reviewed_by',
                'provenance.reviewed_at',
            ],
        },
        'status': {'VERIFIED'},
    },
}


def lookup(data, dotted):
    cur = data
    for part in dotted.split('.'):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur


def nonempty(value):
    return value is not None and value != '' and value != [] and value != {}


def inspect_file(rel, accepted, required=()):
    path = ROOT / rel
    if not path.is_file():
        return {'ready': False, 'path': rel, 'reason': 'MISSING'}
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except Exception as exc:
        return {'ready': False, 'path': rel, 'reason': f'INVALID_YAML: {exc}'}
    status = str(data.get('status', 'MISSING_STATUS'))
    missing = [key for key in required if not nonempty(lookup(data, key))]
    ready = status in accepted and not missing
    reason = None
    if status not in accepted:
        reason = 'STATUS_NOT_VERIFIED'
    elif missing:
        reason = 'REQUIRED_FIELDS_MISSING'
    return {
        'ready': ready,
        'path': rel,
        'status': status,
        'missing_fields': missing,
        'reason': reason,
    }


def inspect_target(spec):
    if 'files' in spec:
        files = [inspect_file(path, spec['status'], required)
                 for path, required in spec['files'].items()]
        return {'ready': all(item['ready'] for item in files), 'files': files}
    return inspect_file(spec['path'], spec['status'], spec['required'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--require', choices=['simulation', *TARGETS], default='simulation')
    args = parser.parse_args()
    sim_path = ROOT / 'out/final_acceptance.json'
    sim = json.loads(sim_path.read_text()) if sim_path.is_file() else {}
    layers = {
        'simulation': {
            'ready': sim.get('status') == 'PASS',
            'path': 'out/final_acceptance.json',
            'status': sim.get('status', 'MISSING'),
        }
    }
    layers.update({name: inspect_target(spec) for name, spec in TARGETS.items()})
    report = {
        'status': 'PASS' if layers[args.require]['ready'] else 'NOT_READY',
        'required_target': args.require,
        'simulation_only_ready': layers['simulation']['ready'],
        'real_robot_ready': layers['camera']['ready'] and layers['hardware']['ready'],
        'thor_connected_ready': layers['thor']['ready'],
        'live_vlm_ready': layers['vlm']['ready'],
        'layers': layers,
        'rule': 'A status label alone is insufficient: templates, blank fields, and legacy candidates never count as verified.',
    }
    output = ROOT / 'out/readiness/development_readiness.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['status'] == 'PASS' else 2


if __name__ == '__main__':
    sys.exit(main())
