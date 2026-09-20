"""Shared Robot 129 joint model constants.

Single source of truth for the adapter and executor so client-side validation
cannot silently drift from ros2_ws/src/robot129_description/urdf/robot129.urdf and
sim/scripts/run_robot129_ros_webrtc.py's own LIMITS array. If the URDF changes,
update this file and re-verify against docs/progress/grasp_motion_s0_inventory.md.
"""
from __future__ import annotations

ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINT_NAMES = ["joint7"]  # joint8 is a URDF mimic (-joint7); never commanded directly.

# (lower, upper) in radians for revolute joints, meters for the prismatic gripper joint.
# robot129.urdf:58,87,116,145,174,203,259
JOINT_LIMITS = {
    "joint1": (-2.618, 2.618),
    "joint2": (0.0, 3.14),
    "joint3": (-2.967, 0.0),
    "joint4": (-1.745, 1.745),
    "joint5": (-1.22, 1.22),
    "joint6": (-2.0944, 2.0944),
    "joint7": (0.0, 0.035),
}

ARM_COMMAND_TOPIC = "/robot129_sim/arm_controller/joint_trajectory"
GRIPPER_COMMAND_TOPIC = "/robot129_sim/gripper_controller/joint_trajectory"
JOINT_STATES_TOPIC = "/robot129_sim/joint_states"
TRAJECTORY_EVENTS_TOPIC = "/robot129_sim/trajectory_events"
SCENE_STATE_TOPIC = "/robot129_sim/scene_state"
CONTACT_TOPICS = {
    "link7": "/robot129_sim/contacts/link7",
    "link8": "/robot129_sim/contacts/link8",
}
OBJECT_POSE_TOPIC = "/robot129_sim/objects/target_cube/pose"
RESET_SCENE_SERVICE = "/robot129_sim/reset_scene"
RECORDING_SERVICE = "/robot129_sim/recording"

ARM_ACTION_NAME = "/robot129_sim/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION_NAME = "/robot129_sim/gripper_controller/follow_joint_trajectory"

# Observed on this shared-GPU deployment (S1 smoke test, 2026-09-18): the live Isaac
# runner sustains roughly 19 Hz against its nominal 120 Hz physics rate, i.e. a
# real-time factor around 0.16, consistent with the historically measured ~4.3-4.6 Hz
# wrist-camera rate (both publish every 4th physics frame). This is NOT a fixed
# constant -- it depends on GPU contention with other users -- so timeouts must be
# derived from it via a configurable minimum assumed real-time factor, never a fixed
# few-second value tuned for an idle GPU.
DEFAULT_MIN_REALTIME_FACTOR = 0.08  # conservative floor; measured ~0.16 on a busy GPU
DEFAULT_TIMEOUT_MARGIN_S = 10.0
