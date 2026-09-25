"""Persistent ROS 2 controlled Robot 129 simulation with an interactive WebRTC viewport.

S1 grasp/motion changes (2026-09-18, see docs/progress/grasp_motion_s0_inventory.md and
the plan at grasp_motion research handoff) on top of the original marker-only server:

- ``--scene pick_place`` spawns a dynamic (real-gravity, real-friction) cube on the floor
  plus a visual place-zone marker, and activates contact sensors on link7/link8, so a
  FollowJointTrajectory adapter + MoveIt/MTC can attempt a real physical pick-and-place.
  ``--scene marker`` (default) is byte-for-byte the original behaviour: visual-only red
  cube, viewport red-pixel probe, no physics object, no new scene topics/services beyond
  the additive ones below (which publish nothing meaningful without a cube).
- The trajectory callback now accepts a MoveIt-style first point at ``time_from_start=0``
  (previously rejected outright) as long as it matches the current commanded target
  within tolerance, and interpolates with linear (or cubic Hermite when velocities are
  present) segments sampled on simulation time instead of smoothstep-on-wallclock.
- New topics: ``objects/target_cube/pose`` (PoseStamped, pick_place only), ``contacts/{link7,link8}``
  (WrenchStamped, pick_place only), ``joint_command`` (JointState of the commanded target),
  ``trajectory_events`` (String JSON mirroring the ACCEPT/REJECT/COMPLETE log lines so a
  separate adapter process can observe them), ``scene_state`` (String JSON, transient-local).
- New services: ``reset_scene`` (std_srvs/Trigger, reads ROS parameters under ``reset.*``)
  and ``recording`` (std_srvs/SetBool) which starts/stops periodic overview-camera frame
  capture to ``out/grasp_motion/sessions/run_%04d/`` and, on stop, launches ffmpeg to
  encode the frames (non-blocking).
- ``--record-only`` skips the interactive-viewport / WebRTC-probe path entirely (no port
  49100 usage) and uses an offscreen ``Camera`` render product instead, matching the
  existing convention in ``verify_robot129_physics_grasp.py``'s non-realtime path.

Everything added is opt-in via new CLI flags; the default invocation (no new flags) is
unchanged from before this edit.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Run Robot 129 under isolated ROS 2 trajectory control.")
parser.add_argument("--bundle", type=Path, required=True)
parser.add_argument("--warmup-frames", type=int, default=90)
parser.add_argument(
    "--scene", choices=["marker", "pick_place", "pick_place_hammer", "pick_place_counter", "dynamic_stick"], default="marker",
    help="marker (default, unchanged original behaviour), pick_place (dynamic cube + "
         "contacts), pick_place_hammer (S5 pilot object: same pick_place wiring, but "
         "the graspable body is an elongated handle-shaped box, with a second purely "
         "kinematic 'head' box riding along for a two-part shape a real camera can see), or "
         "pick_place_counter (real-robot layout: raised pedestal with a can-like target and "
         "place pad on top, HOME = the real robot's observe pose; meant to be driven by "
         "tools/run_real_stack_task.sh, which runs the unmodified real mm_actions grasp/place code), or "
         "dynamic_stick (no graspable object; a kinematic rod sways slowly beside the "
         "arm, NOT included in any planner's known geometry -- only visible via "
         "live depth -- for testing research/src/mpg/curobo_bridge/reactive.py's live obstacle "
         "avoidance; publishes /robot129_sim/contacts/{link} WrenchStamped for every monitored "
         "link as ground-truth collision detection).",
)
parser.add_argument(
    "--wrist-camera-model", choices=["auto", "urdf_nominal", "real_calib"], default="auto",
    help="urdf_nominal: the URDF's camera_mount (looks along gripper_base +X, perpendicular to "
         "the grasp direction) with the old 41x31 deg nominal lens. real_calib: the real robot's "
         "hand-eye calibration (ee_T_cam in references/upstream/.../actions/base_action.py; looks "
         "along the grasp direction) with D435-like 640x480 fx=fy=615 intrinsics. auto (default): "
         "real_calib for pick_place_counter, urdf_nominal for every older scene so the tools "
         "validated against them keep their view.",
)
parser.add_argument(
    "--scene-manifest", default="",
    help="Optional path to a scene_manifest_v1 JSON (see "
         "docs/dev_guide_paper_core_and_dashboard_plan.md §5) listing extra static "
         "clutter objects (primitive boxes or NVIDIA-asset USD props) to spawn around "
         "the --scene target object. Only additive to --scene pick_place/pick_place_hammer; "
         "does not change the grasp target, floor, or place marker. Every manifest object "
         "is spawned kinematic/static (never falls, never physically interacts) -- this is "
         "a visual/distractor clutter layer, not new graspable targets, and deliberately "
         "does not touch the table_z_m=0.0 assumption baked into "
         "research/src/mpg/grasp_candidates.py and the MTC collision setup.",
)
parser.add_argument("--demo-targets", choices=["none", "static_multi", "dynamic_bar"], default="none",
                    help="Spawn visual-only target cubes for the two official-style cuRobo demos")
parser.add_argument(
    "--wrist-camera-pitch-deg", type=float, default=0.0,
    help="Opt-in correction for the wrist camera's mount angle (see the "
         "WRIST_CAMERA_PITCH_CORRECTION_DEG comment below for the full empirical "
         "calibration story). 0.0 (default) keeps the straight-down view every VLM/"
         "grasp result this session was validated against. Negative values tilt toward "
         "world -X (brings the place-pad into frame at an oblique angle closer to real "
         "Robot 129 photos, but also starts showing the arm's own hardware at the bottom "
         "of frame and moves the cube off-center) -- try -25 for a first look.",
)
parser.add_argument(
    "--record-only", action="store_true",
    help="Skip the interactive viewport / WebRTC probe path; use an offscreen recording "
         "camera instead. Launch without --livestream when using this flag.",
)
parser.add_argument(
    "--scene-camera", choices=["off", "pole"], default="off",
    help="off (default, unchanged behaviour): only the wrist camera exists. pole: also "
         "spawn a second fixed RGB-D camera on a thin static pole behind/beside the arm "
         "base (research/configs/scene_camera.yaml, status ROUGH_FROM_PHOTO -- see the "
         "dev guide §7.4 for how these numbers were obtained and how to replace them "
         "once measured). Publishes /robot129_sim/scene_camera/{color,aligned_depth_to_"
         "color}/... and a world->scene_camera_color_optical_frame TF, alongside the "
         "existing wrist camera topics -- purely additive, does not touch the wrist "
         "camera or any --scene-camera off behaviour.",
)
parser.add_argument("--record-fps", type=float, default=10.0)
parser.add_argument("--seed", type=int, default=0, help="initial value for the reset.seed ROS parameter")
parser.add_argument(
    "--stick", choices=["swing", "none", "insert_bar"], default="swing",
    help="--scene dynamic_stick only. swing (default): slow local sinusoidal sway "
         "beside the arm. none: spawn no rod and no contact "
         "sensors at all -- use this with --scene-manifest to test a purely STATIC "
         "obstacle (e.g. a cylinder) instead, without an unrelated moving rod also in frame. "
         "insert_bar: park a horizontal bar offstage until /robot129_sim/insert_bar_obstacle is called.",
)
# These defaults define a 3 cm total stroke over a 12 s cycle.
parser.add_argument("--stick-period-s", type=float, default=12.0, help="--scene dynamic_stick --stick swing only; default 12 s for a slow local sway")
parser.add_argument("--stick-amplitude-m", type=float, default=0.015, help="--scene dynamic_stick --stick swing only; default 1.5 cm either side of its resting position")
parser.add_argument(
    "--target-object", default="cube_35",
    help="Which grasp target to spawn for --scene pick_place/pick_place_hammer/"
         "pick_place_counter, by id in research/configs/grasp_targets.json (see "
         "docs/dev_guide_paper_core_and_dashboard_plan.md §8). Default 'cube_35' is the "
         "original 3.5cm cube -- unchanged behaviour. Ignored for pick_place_hammer "
         "(the hammer shape is not in that file) and for --scene marker/dynamic_stick.",
)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True
launcher = AppLauncher(args)
app = launcher.app

import carb
import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.sensors import Camera, CameraCfg, ContactSensor, ContactSensorCfg
from isaaclab.sim import SimulationContext

JOINT_NAMES = [f"joint{i}" for i in range(1, 9)]
ARM_NAMES = JOINT_NAMES[:6]
GRIPPER_NAMES = ["joint7"]
ACTIVE_NAMES = JOINT_NAMES[:7]
LIMITS = np.array(
    [
        [-2.618, 2.618],
        [0.0, 3.140],
        [-2.967, 0.0],
        [-1.745, 1.745],
        [-1.220, 1.220],
        [-2.094, 2.094],
        [0.0, 0.035],
        [-0.035, 0.0],
    ],
    dtype=np.float64,
)
# Per-joint tolerance for accepting a MoveIt-style t=0 first trajectory point: it must be
# within this distance of the currently commanded target (radians for revolute joints,
# meters for the prismatic gripper joint) or it is rejected, never silently snapped.
T0_TOLERANCE = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.003], dtype=np.float64)
HOME = np.array([0.0, 1.20, -1.25, 0.0, 0.15, 0.0, 0.035, -0.035], dtype=np.float64)
WRIST_OPTICAL_PARENT = (
    "/World/Robot/Geometry/world/base_link/link1/link2/link3/link4/link5/link6/"
    "gripper_base/camera_mount/camera_link/camera_color_optical_frame"
)
WRIST_CAMERA_PRIM = WRIST_OPTICAL_PARENT + "/sensor"
WRIST_FRAME = "camera_color_optical_frame"
# --wrist-camera-model urdf_nominal only (real_calib ignores this; see wrist_camera_offset()).
# CORRECTION 2026-09-23: this comment used to claim the URDF camera is coaxial with the
# gripper's reach axis. It is not: camera_link +X (the optical axis) = gripper_base +X,
# perpendicular to the grasp direction (+Z). It only looked "straight down" because the
# gripper points horizontally forward at HOME. Real Robot 129 photos supplied by the user 2026-09-21 show the physical
# camera mounted at a real angle off that axis, not coaxial with it -- that file already
# carries an explicit SIMULATION_NOMINAL_UNVERIFIED_ON_HARDWARE warning for exactly this.
# Re-importing the URDF (tools/import_robot129_usd.sh) to fix camera_mount_rpy_rad
# properly would touch the whole robot USD; this constant instead corrects only the
# Camera sensor's own offset (CameraCfg.OffsetCfg below), which both the rendered image
# and the published TF derive from (wrist_camera.data.pos_w/quat_w_ros), so image and TF
# stay consistent with each other without an USD re-import.
#
# Calibrated empirically 2026-09-21 by bisecting against real captures (not guessed):
# rotation is about the optical frame's own local X axis. 0 deg = straight down (the
# behavior every VLM/grasp result all session was validated against). +90 deg = dead
# level, facing world +X (away from the arm's own base -- renders as empty background,
# nothing at table height is in frame when perfectly level from 0.4m up). -90 deg = dead
# level facing world -X (toward the arm's own base/column -- fills the frame with the
# robot's own structure). Around -25 deg starts bringing the green place-pad into frame
# at a genuine oblique angle closer to the real robot's photos, but ALSO starts showing
# a sliver of the robot's own hardware at the bottom of frame, and moves the cube out of
# center. This is a real trade-off, not a bug to fix further blind: the whole S3-S5 VLM/
# candidate-generation pipeline this session validated assumed close to the straight-down
# view, so changing this default would need everything downstream re-validated against
# the new angle, not just a prettier-looking single frame. Left at 0.0 (matches all
# existing validated results); pass --wrist-camera-pitch-deg to try the tilted view.
WRIST_CAMERA_PITCH_CORRECTION_DEG = args.wrist_camera_pitch_deg
RGB_TOPIC = "/robot129_sim/camera/color/image_raw"
DEPTH_TOPIC = "/robot129_sim/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/robot129_sim/camera/aligned_depth_to_color/camera_info"

# --scene-camera pole: a second, fixed (non-wrist) RGB-D camera. See
# research/configs/scene_camera.yaml for the full provenance note -- status
# ROUGH_FROM_PHOTO, not a measured extrinsic. Loaded unconditionally (like
# _geometry below) even when --scene-camera off, so the numbers are always
# available for e.g. cuRobo's world model to read as a known-obstacle pole
# without needing the camera itself enabled.
SCENE_CAMERA_ENABLED = args.scene_camera == "pole"
_SCENE_CAMERA_PATH = args.bundle.resolve() / "research" / "configs" / "scene_camera.yaml"
_scene_camera_doc = yaml.safe_load(_SCENE_CAMERA_PATH.read_text())
_scene_camera_cfg = _scene_camera_doc["pole_camera"]
SCENE_CAMERA_OFFSET_FROM_BASE_M = tuple(float(v) for v in _scene_camera_cfg["offset_from_base_m"])
SCENE_CAMERA_PITCH_DEG = float(_scene_camera_cfg["pitch_deg"])
SCENE_CAMERA_YAW_DEG = float(_scene_camera_cfg.get("yaw_deg", 0.0))
SCENE_CAMERA_INTRINSICS = _scene_camera_cfg["intrinsics"]
SCENE_CAMERA_POLE_CFG = _scene_camera_cfg["pole"]
SCENE_CAMERA_PRIM = "/World/ScenePoleCamera"
SCENE_CAMERA_FRAME = "scene_camera_color_optical_frame"
SCENE_RGB_TOPIC = "/robot129_sim/scene_camera/color/image_raw"
SCENE_DEPTH_TOPIC = "/robot129_sim/scene_camera/aligned_depth_to_color/image_raw"
SCENE_CAMERA_INFO_TOPIC = "/robot129_sim/scene_camera/aligned_depth_to_color/camera_info"

# "Global view" (dashboard window 2, docs/dev_guide_paper_core_and_dashboard_plan.md §8):
# the existing overview_camera (previously used only internally for --record's video, see
# its CameraCfg comment below) also published live to ROS. .get(...) with the exact
# previous hardcoded values as defaults, so a scene_camera.yaml missing this whole section
# still behaves exactly as before it existed.
_overview_camera_cfg = _scene_camera_doc.get("overview_camera", {})
OVERVIEW_CAMERA_EYE_M = tuple(float(v) for v in _overview_camera_cfg.get("eye_m", [1.05, 0.95, 0.85]))
OVERVIEW_CAMERA_TARGET_XY_M = tuple(float(v) for v in _overview_camera_cfg.get("target_xy_m", [0.30, -0.05]))
OVERVIEW_CAMERA_TARGET_Z_BASE_M = float(_overview_camera_cfg.get("target_z_base_m", 0.05))
OVERVIEW_RGB_TOPIC = "/robot129_sim/overview_camera/color/image_raw"
OVERVIEW_CAMERA_INFO_TOPIC = "/robot129_sim/overview_camera/camera_info"
OVERVIEW_CAMERA_FRAME = "overview_camera_optical_frame"

# S0 chosen scene layout (docs/progress/grasp_motion_s0_inventory.md, cross-validated FK).
# All object geometry below is loaded from research/configs/scene_geometry.json, the
# single source of truth introduced 2026-09-20 after an audit found the hammer
# dimensions here had silently drifted from research/scripts/s5_pilot_hammer.py's copy
# (handle height 0.030 here vs 0.040 there, head 0.045 vs 0.050) despite a comment in
# this file claiming they matched -- the offline pilot's candidates were computed for a
# taller object than Isaac actually spawns. Isaac's spawned geometry is authoritative;
# every other reader (the pilot script, tools/generate_s2_cube_candidates.py, and
# robot129_mtc_pick_place.cpp's collision-proxy constants) must load this same file
# rather than redeclare the numbers.
_GEOMETRY_PATH = args.bundle.resolve() / "research" / "configs" / "scene_geometry.json"
_geometry = json.loads(_GEOMETRY_PATH.read_text())

CUBE_SIZE_M = _geometry["cube"]["size_m"]
CUBE_XY_DEFAULT = tuple(_geometry["cube"]["xy_default_m"])
CUBE_DROP_Z = _geometry["cube"]["drop_z_m"]  # spawn slightly above the floor (z=0) and let gravity settle it
CUBE_REST_Z = _geometry["cube"]["rest_z_m"]  # cube half-size; resting height once settled on the floor
CUBE_MASS_KG = _geometry["cube"]["mass_kg"]
PLACE_XY_DEFAULT = tuple(_geometry["place"]["xy_default_m"])
SETTLE_STEPS = 90

COUNTER_SCENE = args.scene == "pick_place_counter"
TABLE_Z = 0.0
TARGET_SIZE_XYZ = (CUBE_SIZE_M, CUBE_SIZE_M, CUBE_SIZE_M)
if COUNTER_SCENE:
    _counter = _geometry["counter"]
    PEDESTAL_CENTER_XY = tuple(_counter["pedestal_center_xy_m"])
    PEDESTAL_SIZE_M = tuple(_counter["pedestal_size_m"])
    TABLE_Z = PEDESTAL_SIZE_M[2]
    TARGET_SIZE_XYZ = tuple(_counter["target_size_m"])
    CUBE_XY_DEFAULT = tuple(_counter["target_xy_default_m"])
    PLACE_XY_DEFAULT = tuple(_counter["place_xy_default_m"])
    CUBE_DROP_Z = TABLE_Z + TARGET_SIZE_XYZ[2] / 2.0 + 0.005
    HOME = np.array(list(_counter["observe_joints_rad"]) + [0.035, -0.035], dtype=np.float64)

WRIST_CAMERA_MODEL = args.wrist_camera_model
if WRIST_CAMERA_MODEL == "auto":
    WRIST_CAMERA_MODEL = "real_calib" if COUNTER_SCENE else "urdf_nominal"
# Real robot hand-eye calibration, verbatim from references/upstream/mm_system/main_ws/src/
# mm_actions/mm_actions/actions/base_action.py convert_camera_to_base(): camera optical frame
# expressed in the rtb Piper end-effector frame, which is piper_gripper_base + (0, 0, 0.12)
# (Tsaimingchun14/robotics-toolbox-python@8d8c0c3 models/URDF/Piper.py, grippers[0].tool).
REAL_EE_T_CAM = np.array([
    [0.12045728, 0.99241666, 0.02447911, -0.07102005],
    [-0.99265611, 0.12068956, -0.0082389, 0.02413094],
    [-0.0111308, -0.0233069, 0.99966639, -0.09727718],
    [0.0, 0.0, 0.0, 1.0],
])
REAL_EE_TOOL_Z_M = 0.12

# S5 pilot object (docs/progress/grasp_motion_progress_report.md "S5" section): a
# two-part hammer, realized here as two REAL prims so a live camera can actually see
# and segment it. The handle is a real dynamic RigidObject spawned at the "target_cube"
# prim path/topic (every existing pick_place tool -- MTC's hardcoded collision object,
# adapter, contact sensors -- keeps working unmodified, treating the handle as "the
# graspable object" exactly like the cube); the head is a SEPARATE, purely kinematic
# body that is not wired into any of that existing machinery -- every frame its pose is
# set to the handle's live pose composed with a fixed local offset, so it visually and
# physically (for camera/depth capture) rides along with the handle. It does not
# participate in MTC collision avoidance or grasp candidate execution this round -- see
# the progress report's honest scope note for why that's deliberately out of scope here.
_hammer = _geometry["hammer"]
HAMMER_HANDLE_SIZE_M = (_hammer["handle"]["len_m"], _hammer["handle"]["width_m"], _hammer["handle"]["height_m"])
HAMMER_HEAD_SIZE_M = (_hammer["head"]["len_m"], _hammer["head"]["width_m"], _hammer["head"]["height_m"])
# From handle center to head center, in the handle's own local frame. MUST clear
# handle_half_x (0.07) + head_half_x (0.025) = 0.095 or the two boxes overlap at spawn;
# update_hammer_head() (which would otherwise correct this) only starts running once the
# ROS publish loop begins, AFTER the ~90-step settle phase runs with physics live -- an
# overlapping kinematic (infinite-mass) head shoved the dynamic handle across the whole
# scene during exactly that settle phase the first time this was tried (observed
# 2026-09-19: handle ended up at world x=-1.07, over a meter from its x=0.32 spawn).
# 0.12 clears that minimum with margin.
HAMMER_HEAD_LOCAL_OFFSET_M = tuple(_hammer["head_local_offset_m"])

# --scene dynamic_stick: place the rod at the static cone's A/B corridor, then sway
# locally. The former y=0 center intersected the home gripper (FK: gripper_base
# x=0.27, y=0, z=0.455), so reducing amplitude alone still caused contact.
INSERT_BAR_MODE = args.stick == "insert_bar"
_bar_cfg = json.loads((args.bundle.resolve() / "research/configs/scenes/official_dynamic_bar.json").read_text())
BAR_SIZE_XYZ = tuple(_bar_cfg["size_m"])
BAR_CENTER_XYZ = tuple(_bar_cfg["center_m"])
BAR_PARKED_XYZ = tuple(_bar_cfg["parked_center_m"])
STICK_SIZE_XYZ = BAR_SIZE_XYZ if INSERT_BAR_MODE else (0.03, 0.03, 0.22)
STICK_CENTER_X = BAR_CENTER_XYZ[0] if INSERT_BAR_MODE else 0.2054028008
STICK_CENTER_Y = BAR_CENTER_XYZ[1] if INSERT_BAR_MODE else 0.3580777478
STICK_CENTER_Z = BAR_CENTER_XYZ[2] if INSERT_BAR_MODE else 0.11
# At the default 1.5 cm amplitude the rod stays in y=0.335..0.365; its peak
# lateral speed is about 0.8 cm/s, versus about 23 cm/s before this change.
STICK_SWEEP_Y_AMPLITUDE_M = args.stick_amplitude_m
STICK_SWEEP_PERIOD_S = args.stick_period_s
STICK_MONITORED_LINKS = ["link1", "link2", "link3", "link4", "link5", "link6", "gripper_base", "link7", "link8"]
STICK_FRAME = "dynamic_stick"
STICK_ENABLED = args.stick in ("swing", "insert_bar")

keep_running = True


def request_stop(_signum, _frame):
    global keep_running
    keep_running = False


def set_live_camera(viewport, eye=(0.92, 0.78, 0.72), target=(0.27, 0.0, 0.36)):
    from omni.kit.viewport.utility.camera_state import ViewportCameraState
    from pxr import Gf

    path = viewport.get_active_camera() or "/OmniverseKit_Persp"
    state = ViewportCameraState(path, viewport)
    state.set_position_world(Gf.Vec3d(*eye), False)
    state.set_target_world(Gf.Vec3d(*target), True)
    return str(path)


def _hue_degrees(rgb_0_255: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Per-pixel hue (degrees, [0,360)) and chroma (max-min channel) for an [...,3] array.
    Hue is undefined where chroma is ~0 (gray/black/white) -- caller masks those out
    rather than trusting the (arbitrary) value this returns for them.
    """
    r, g, b = rgb_0_255[..., 0], rgb_0_255[..., 1], rgb_0_255[..., 2]
    maxc = np.maximum(np.maximum(r, g), b)
    minc = np.minimum(np.minimum(r, g), b)
    delta = maxc - minc
    safe_delta = np.where(delta == 0, 1.0, delta)
    is_r_max = maxc == r
    is_g_max = (maxc == g) & ~is_r_max
    is_b_max = ~is_r_max & ~is_g_max
    hue = np.zeros_like(maxc)
    hue = np.where(is_r_max, ((g - b) / safe_delta) % 6.0, hue)
    hue = np.where(is_g_max, ((b - r) / safe_delta) + 2.0, hue)
    hue = np.where(is_b_max, ((r - g) / safe_delta) + 4.0, hue)
    return np.mod(hue * 60.0, 360.0), delta


def capture_viewport_probe(root: Path, viewport, step_once, target_color_rgb=None) -> dict:
    """Boot-time WebRTC acceptance check: capture one viewport frame and confirm it's
    actually a real render (not blank/corrupted), plus -- when `target_color_rgb` is given
    ((r,g,b), each 0-1, matching whatever visual_material the scene's target/obstacle was
    actually spawned with, see main()'s `probe_target_color_rgb`) -- that a plausible
    fraction of the frame is close to that color, roughly confirming the expected object
    is actually in view.

    `target_color_rgb=None` (used for scenes/configs with no single guaranteed-colored
    object, e.g. --scene dynamic_stick --stick none with no --scene-manifest) skips the
    color check and only checks luminance. This replaces a hardcoded "red" check that
    silently broke the moment a scene's target could be a non-red color (--target-object)
    or nonexistent (dynamic_stick without a stick or manifest) -- found live 2026-09-24.
    """
    from omni.kit.viewport.utility import capture_viewport_to_file
    from PIL import Image

    state_dir = root / "out/ros_webrtc_robot129"
    state_dir.mkdir(parents=True, exist_ok=True)
    image_path = state_dir / "viewport_probe.png"
    report_path = state_dir / "viewport_probe.json"
    image_path.unlink(missing_ok=True)
    capture_viewport_to_file(viewport, file_path=str(image_path))
    for _ in range(120):
        step_once()
        if image_path.is_file() and image_path.stat().st_size > 1024:
            break
    try:
        with Image.open(image_path) as image:
            rgb = np.asarray(image.convert("RGB"), dtype=np.float32)
        luma = 0.2126 * rgb[..., 0] + 0.7152 * rgb[..., 1] + 0.0722 * rgb[..., 2]
        valid = float(luma.mean()) > 5 and float(luma.std()) > 3
        report = {
            "resolution": [int(rgb.shape[1]), int(rgb.shape[0])],
            "mean_luminance": round(float(luma.mean()), 3),
            "std_luminance": round(float(luma.std()), 3),
            "image": str(image_path),
        }
        if target_color_rgb is not None:
            # HUE match, not raw RGB distance: a real live test (2026-09-24, an orange
            # (0.85,0.35,0.10) cylinder, confirmed by eye to be correctly rendered and
            # clearly in frame) found this RTX renderer's PBR lighting shifts a flat
            # PreviewSurfaceCfg diffuse_color a LOT in brightness/saturation under this
            # scene's dome light (rendered patch mean (230,201,133) vs raw material
            # (217,89,26) -- Euclidean RGB distance ~155, nowhere near a naive threshold),
            # while shifting hue comparatively little (~22 deg in that same test). Only
            # trust hue where the pixel is actually saturated (chroma > 20) -- gray/white/
            # black pixels have noisy, meaningless hue.
            target = np.asarray(target_color_rgb, dtype=np.float32) * 255.0
            target_hue, target_chroma = _hue_degrees(target[np.newaxis, :])
            pixel_hue, pixel_chroma = _hue_degrees(rgb)
            saturated = pixel_chroma > 20.0
            if float(target_chroma[0]) < 5.0:
                # Target itself is ~gray (e.g. a white/gray manifest object) -- hue is
                # meaningless for it too; fall back to plain brightness closeness.
                matches = np.linalg.norm(rgb - target, axis=-1) < 40.0
            else:
                hue_diff = np.abs(pixel_hue - float(target_hue[0]))
                hue_diff = np.minimum(hue_diff, 360.0 - hue_diff)
                matches = saturated & (hue_diff < 35.0)
            target_fraction = float(matches.mean())
            valid = valid and target_fraction > 0.0002
            report["target_color_rgb"] = [round(float(v), 3) for v in target_color_rgb]
            report["target_color_fraction"] = round(target_fraction, 6)
        report["status"] = "PASS" if valid else "FAIL"
    except Exception as exc:
        report = {"status": "FAIL", "reason": f"{type(exc).__name__}: {exc}", "image": str(image_path)}
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    return report


def trajectory_seconds(point) -> float:
    return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9


def hermite(p0, v0, p1, v1, t, span):
    if span <= 1e-9:
        return p1
    s = t / span
    h00 = 2 * s**3 - 3 * s**2 + 1
    h10 = s**3 - 2 * s**2 + s
    h01 = -2 * s**3 + 3 * s**2
    h11 = s**3 - s**2
    return h00 * p0 + h10 * span * v0 + h01 * p1 + h11 * span * v1


def sample_plan(plan, sim_time: float):
    """Sample a plan at simulation time ``sim_time`` (seconds since plan start).

    Linear interpolation between waypoints, or cubic Hermite when every waypoint carries
    velocities. Time is measured on simulation time (frame_count * physics_dt), not the
    wall clock, so playback speed tracks the physics step rate rather than render stalls.
    """
    elapsed = sim_time - plan["started"]
    prev_t, prev_p, prev_v = 0.0, plan["initial"], plan["initial_velocity"]
    for t, p, v in zip(plan["times"], plan["points"], plan["velocities"]):
        if elapsed <= t:
            span = t - prev_t
            if plan["has_velocity"]:
                return hermite(prev_p, prev_v, p, v, elapsed - prev_t, span), False
            fraction = 1.0 if span <= 1e-9 else (elapsed - prev_t) / span
            fraction = max(0.0, min(1.0, fraction))
            return prev_p + (p - prev_p) * fraction, False
        prev_t, prev_p, prev_v = t, p, v
    return plan["points"][-1], True


def yaw_to_quat_xyzw(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def _matrix_to_quat_xyzw(m: np.ndarray):
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2.0, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2.0, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2.0, m[1, 0] - m[0, 1])
    return (x, y, z, w)


def wrist_camera_offset():
    """(pos, rot_xyzw) of the camera sensor relative to its parent prim, the URDF's
    camera_color_optical_frame (gripper_base -> camera_mount xyz 0.055 0 0.075 -> optical
    rpy -pi/2 0 -pi/2; see robot129.urdf). With convention="ros" an identity offset means
    "the sensor's optical axes are the parent's optical axes"."""
    if WRIST_CAMERA_MODEL == "urdf_nominal":
        half = math.radians(WRIST_CAMERA_PITCH_CORRECTION_DEG) / 2.0
        return (0.0, 0.0, 0.0), (math.sin(half), 0.0, 0.0, math.cos(half))
    gb_T_parent = np.eye(4)
    gb_T_parent[:3, :3] = _rpy_matrix(-math.pi / 2, 0.0, -math.pi / 2)
    gb_T_parent[:3, 3] = (0.055, 0.0, 0.075)
    gb_T_ee = np.eye(4)
    gb_T_ee[2, 3] = REAL_EE_TOOL_Z_M
    parent_T_cam = np.linalg.inv(gb_T_parent) @ gb_T_ee @ REAL_EE_T_CAM
    return tuple(float(v) for v in parent_T_cam[:3, 3]), _matrix_to_quat_xyzw(parent_T_cam[:3, :3])


def spawn_clutter_from_manifest(manifest_path: str) -> list:
    """--scene-manifest support (see the arg's help text and docs/dev_guide_
    paper_core_and_dashboard_plan.md §5): spawn each listed object as a
    kinematic prop by default -- a distractor clutter layer, not a new
    graspable target. Returns a list of dicts (NOT bare RigidObject handles,
    since 2026-09-21's capture-back feature needs each object's manifest
    metadata alongside its live handle to write a manifest entry back out):
    {"id", "kind", "physics", "prim_path", plus the kind-specific fields
    (size_m/color_rgb for primitive_box, usd_path/scale for usd_asset), and
    "rigid_obj" -- ONLY for kind="primitive_box", see the crash note below.

    kind="primitive_box": a flat-shaded box, no external asset needed.
    kind="primitive_cylinder" / "primitive_cone" / "primitive_sphere" (added for the
    obstacle-avoidance demo scenarios, docs/dev_guide_paper_core_and_dashboard_plan.md
    §8): same flat-shaded-no-asset approach as primitive_box, fields "radius_m" (all
    three) and "height_m" (cylinder/cone only; sphere has none). IsaacLab's
    CylinderCfg/ConeCfg spawn with their axis along local +Z (USD convention), i.e.
    upright for yaw_rad=0 -- exactly "a vertical rod/cone standing on the floor" without
    any extra rotation math, which is what every demo scenario that uses these wants.
    kind="usd_asset": loads usd_path as-is (NVIDIA's Isaac asset CDN paths,
    e.g. Isaac/Props/YCB/Axis_Aligned/025_mug.usd, verified reachable
    2026-09-20 -- see the dev guide for the checked list). Isaac fetches and
    locally caches USD files referenced this way; the first spawn of a given
    asset may take longer while it downloads.

    KNOWN LIMITATION, confirmed by direct testing 2026-09-21, not worked
    around (it is a native Isaac Sim crash, not something fixable from this
    script): touching a usd_asset-spawned RigidObject's Python handle a
    SECOND time -- either by keeping the original handle alive past this
    function, or by freshly re-wrapping the same prim_path later with
    RigidObject(RigidObjectCfg(prim_path=..., spawn=None)) -- crashes Isaac
    with no Python traceback (signature: "Plugin interface for a client:
    omni.hydratexture.plugin was already released" + "Unexpected reference
    count of 2 for UsdStage ... while being closed"), whether that second
    touch happens moments later during sim.reset() or tens of seconds later
    from inside a ROS service callback. kind="primitive_box" (plain authored
    geometry, not a referenced/instanced external stage) has neither problem
    -- both retaining its handle and re-wrapping it later are fine. So:
    primitive_box keeps "rigid_obj" in its record and reports its true LIVE
    pose on capture; usd_asset does not, and capture_scene_manifest instead
    re-emits its ORIGINAL manifest-spawn pose unchanged (it is documented as
    a static/kinematic visual backdrop by default anyway -- see physics=
    below -- so this only matters if you set physics="dynamic" on one, in
    which case capture will not reflect where it actually settled).

    physics="static" (default): kinematic, disable_gravity -- never falls,
    never gets pushed, only useful as a fixed visual/collision backdrop.
    physics="dynamic": real gravity + mass (mass_kg field, default 0.05) --
    the object can fall, be knocked over, or be pushed by the arm, same as
    the pick_place target cube's own RigidBodyPropertiesCfg. Requested
    2026-09-21 alongside the capture-back feature so a WebRTC-dragged object
    can actually be picked up too, not just moved once by hand.
    """
    manifest = json.loads(Path(manifest_path).read_text())
    spawned = []
    for i, obj in enumerate(manifest.get("objects", [])):
        obj_id = obj.get("id", f"clutter_{i}")
        prim_path = f"/World/Clutter/{obj_id}"
        xy = obj.get("xy_m", [0.0, 0.0])
        z = float(obj.get("z_m", 0.05))
        yaw = float(obj.get("yaw_rad", 0.0))
        kind = obj.get("kind", "primitive_box")
        physics = obj.get("physics", "static")
        dynamic = physics == "dynamic"
        rigid_props = sim_utils.RigidBodyPropertiesCfg(
            kinematic_enabled=not dynamic, disable_gravity=not dynamic,
        )
        mass_props = sim_utils.MassPropertiesCfg(mass=float(obj.get("mass_kg", 0.05))) if dynamic else None

        record = {"id": obj_id, "kind": kind, "physics": physics}
        if kind == "primitive_box":
            size = tuple(obj.get("size_m", [0.05, 0.05, 0.05]))
            color = tuple(obj.get("color_rgb", [0.5, 0.5, 0.5]))
            spawn_cfg = sim_utils.CuboidCfg(
                size=size, rigid_props=rigid_props, mass_props=mass_props,
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, metallic=0.05),
                # Needed for a ContactSensor filtered against this prim to report anything
                # (see obstacle_prim_paths in main(), added 2026-09-24 for the
                # static_cylinder/p2p_cone demo scenarios) -- harmless for manifest uses
                # that never filter a sensor against this object either.
                activate_contact_sensors=True,
            )
            record["size_m"] = list(size)
            record["color_rgb"] = list(color)
        elif kind in ("primitive_cylinder", "primitive_cone", "primitive_sphere"):
            radius = float(obj.get("radius_m", 0.025))
            color = tuple(obj.get("color_rgb", [0.5, 0.5, 0.5]))
            common = dict(
                rigid_props=rigid_props, mass_props=mass_props,
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, metallic=0.05),
                activate_contact_sensors=True,  # see the primitive_box branch's comment above
            )
            if kind == "primitive_sphere":
                spawn_cfg = sim_utils.SphereCfg(radius=radius, **common)
                record["radius_m"] = radius
            else:
                height = float(obj.get("height_m", 0.1))
                cfg_cls = sim_utils.CylinderCfg if kind == "primitive_cylinder" else sim_utils.ConeCfg
                spawn_cfg = cfg_cls(radius=radius, height=height, **common)
                record["radius_m"] = radius
                record["height_m"] = height
            record["color_rgb"] = list(color)
        elif kind == "usd_asset":
            usd_path = obj["usd_path"]
            scale = tuple(obj.get("scale", [1.0, 1.0, 1.0]))
            spawn_cfg = sim_utils.UsdFileCfg(
                usd_path=usd_path, scale=scale,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
            )
            record["usd_path"] = usd_path
            record["scale"] = list(scale)
            # capture_scene_manifest re-emits this unchanged for usd_asset objects --
            # see the crash note above for why it can't safely re-read a live pose.
            record["spawn_xy_m"] = [xy[0], xy[1]]
            record["spawn_z_m"] = z
            record["spawn_yaw_rad"] = yaw
        else:
            print(f"[ROBOT129 ROS WEBRTC] SCENE_MANIFEST skipping {obj_id}: unknown kind={kind!r}", flush=True)
            continue

        rigid_obj = RigidObject(
            RigidObjectCfg(
                prim_path=prim_path, spawn=spawn_cfg,
                init_state=RigidObjectCfg.InitialStateCfg(
                    # InitialStateCfg.rot is (x, y, z, w) in this IsaacLab version.
                    pos=(xy[0], xy[1], z), rot=yaw_to_quat_xyzw(yaw),
                ),
            )
        )
        record["prim_path"] = prim_path
        if kind in ("primitive_box", "primitive_cylinder", "primitive_cone", "primitive_sphere"):
            # All plain authored (not referenced/instanced external-stage) geometry --
            # same "retaining the handle is safe" case the crash note above documents
            # for primitive_box specifically, on the same grounds (not a usd_asset).
            record["rigid_obj"] = rigid_obj
        else:
            del rigid_obj  # see the docstring above -- must not be retained for kind="usd_asset"
        spawned.append(record)
        print(f"[ROBOT129 ROS WEBRTC] SCENE_MANIFEST spawned {obj_id} kind={kind} physics={physics} at xy={xy}", flush=True)
    return spawned


def quat_wxyz_to_yaw(w: float, x: float, y: float, z: float) -> float:
    """Yaw (rotation about world Z) from a wxyz quaternion, ignoring any
    roll/pitch component -- correct only for objects that stayed upright
    (true for a dragged-in-WebRTC object that wasn't also tipped over; the
    manifest schema has no roll/pitch fields to round-trip those anyway)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def main() -> int:
    if int(os.environ.get("ROS_DOMAIN_ID", "-1")) != 129:
        print("[ROBOT129 ROS WEBRTC] FAIL - ROS_DOMAIN_ID must be 129", flush=True)
        return 2

    root = args.bundle.resolve()
    pick_place = args.scene in ("pick_place", "pick_place_hammer", "pick_place_counter")
    hammer_mode = args.scene == "pick_place_hammer"
    settings = carb.settings.get_settings()
    settings.set("/rtx/background/source/type", 2)
    settings.set("/rtx/background/source/color", (0.055, 0.065, 0.080))
    # Demo scenes render at 15 Hz while physics remains 120 Hz. Three RTX camera
    # products plus WebRTC at 30 Hz made RTF collapse under shared-GPU load; changing
    # render_interval reduces rendering work without changing physics/contact timing.
    render_interval = 8 if args.demo_targets != "none" else 4
    sim = SimulationContext(sim_utils.SimulationCfg(device=args.device, dt=1 / 120, render_interval=render_interval))

    def step_sim():
        app.update()
        app.update()
        sim.step()

    floor = sim_utils.CuboidCfg(
        size=(4.0, 4.0, 0.05),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.11, 0.12, 0.14), roughness=0.88),
    )
    floor.func("/World/DeepGrayFloor", floor, translation=(0.0, 0.0, -0.025))
    light = sim_utils.DomeLightCfg(intensity=2800, color=(0.82, 0.84, 0.88), visible_in_primary_ray=False)
    light.func("/World/Light", light)

    cube = None
    left_contact = None
    right_contact = None
    clutter_objects: list = []  # populated below iff --scene-manifest was given; read by the
    # capture_scene_manifest service (declared later in main()) to write live poses back out.
    stick = None
    stick_contacts: dict = {}
    dynamic_stick_mode = args.scene == "dynamic_stick"
    # target_drop_z: default here so handle_reset_scene (declared later, for every scene,
    # not just pick_place) always has a value to close over; overwritten below inside
    # `if pick_place:` to match whichever --target-object shape was actually spawned --
    # respawning a tall_block at the cube's old CUBE_DROP_Z would drop it from inside the
    # floor or several cm above it depending on shape, not gently settle it.
    target_drop_z = CUBE_DROP_Z
    # capture_viewport_probe()'s boot-time acceptance check (WebRTC path only -- see its
    # call site) needs to know what color to actually look for: it used to hardcode "red",
    # which silently broke the moment a scene could have a non-red target (--target-object,
    # added 2026-09-24) or no guaranteed-colored target at all (dynamic_stick with
    # --stick none and no manifest) -- found live 2026-09-24 running
    # `--scene dynamic_stick --stick none --scene-manifest ...` over WebRTC for the first
    # time (VIEWPORT_PROBE FAIL, red_target_fraction=0.0, on an otherwise-correct render).
    # None means "don't color-check, just confirm the frame isn't blank".
    probe_target_color_rgb = None
    if pick_place:
        # Dynamic cube: real gravity from the start, realistic (not exaggerated) friction.
        # Unlike sim/scripts/verify_robot129_physics_grasp.py this spawns ON the floor and
        # is never made kinematic or gravity-disabled: "picked up" here means real PhysX
        # contact lift, not a floating pre-placed prop.
        if COUNTER_SCENE:
            # Static collider only (like the floor), not a rigid body: the real counter
            # never moves, and nothing here needs its pose.
            pedestal = sim_utils.CuboidCfg(
                size=PEDESTAL_SIZE_M,
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.62, 0.55, 0.45), roughness=0.8),
            )
            pedestal.func(
                "/World/Pedestal", pedestal,
                translation=(PEDESTAL_CENTER_XY[0], PEDESTAL_CENTER_XY[1], TABLE_Z / 2.0),
            )
        # --target-object (see the arg's help text and research/configs/grasp_targets.json):
        # only applies to plain pick_place. hammer_mode and COUNTER_SCENE each already fully
        # control their own target geometry through separate, pre-existing paths (HAMMER_*
        # / _counter["target_size_m"]) and ignore this flag -- unchanged from before it existed.
        target_shape = "box"
        target_size_xyz = HAMMER_HANDLE_SIZE_M if hammer_mode else TARGET_SIZE_XYZ
        target_radius_m = target_height_m = None
        target_color = (0.55, 0.35, 0.12) if hammer_mode else (0.88, 0.08, 0.04)  # brown handle vs red cube
        target_mass_kg = CUBE_MASS_KG
        target_drop_z = CUBE_DROP_Z
        if not hammer_mode and not COUNTER_SCENE and args.target_object != "cube_35":
            grasp_targets = json.loads((root / "research" / "configs" / "grasp_targets.json").read_text())["targets"]
            if args.target_object not in grasp_targets:
                print(
                    f"[ROBOT129 ROS WEBRTC] FAIL - unknown --target-object {args.target_object!r}, "
                    f"choices: {sorted(grasp_targets)}", flush=True,
                )
                return 2
            spec = grasp_targets[args.target_object]
            target_shape = spec["shape"]
            target_color = tuple(spec["color_rgb"])
            target_mass_kg = float(spec["mass_kg"])
            if target_shape == "box":
                target_size_xyz = tuple(spec["size_m"])
                rest_z = target_size_xyz[2] / 2.0
            elif target_shape == "cylinder":
                target_radius_m = float(spec["radius_m"])
                target_height_m = float(spec["height_m"])
                rest_z = target_height_m / 2.0
            elif target_shape == "sphere":
                target_radius_m = float(spec["radius_m"])
                rest_z = target_radius_m
            else:
                print(
                    f"[ROBOT129 ROS WEBRTC] FAIL - grasp_targets.json {args.target_object!r} "
                    f"has unknown shape {target_shape!r}", flush=True,
                )
                return 2
            # Same settle clearance the original cube used: drop_z_m (0.03) - rest_z_m
            # (0.0175) = 0.0125 above resting height, per research/configs/scene_geometry.json.
            target_drop_z = rest_z + 0.0125
        probe_target_color_rgb = target_color

        target_common_kwargs = dict(
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=False, disable_gravity=False, max_depenetration_velocity=0.5
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=target_mass_kg),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.8, dynamic_friction=0.6, restitution=0.0,
                friction_combine_mode="average", restitution_combine_mode="min",
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=target_color, metallic=0.05),
            activate_contact_sensors=True,
        )
        if target_shape == "cylinder":
            target_spawn_cfg = sim_utils.CylinderCfg(radius=target_radius_m, height=target_height_m, **target_common_kwargs)
        elif target_shape == "sphere":
            target_spawn_cfg = sim_utils.SphereCfg(radius=target_radius_m, **target_common_kwargs)
        else:
            target_spawn_cfg = sim_utils.CuboidCfg(size=target_size_xyz, **target_common_kwargs)

        cube = RigidObject(
            RigidObjectCfg(
                prim_path="/World/TargetCube",
                spawn=target_spawn_cfg,
                init_state=RigidObjectCfg.InitialStateCfg(pos=(CUBE_XY_DEFAULT[0], CUBE_XY_DEFAULT[1], target_drop_z)),
            )
        )
        hammer_head = None
        if hammer_mode:
            # Purely kinematic (never touched by PhysX dynamics or MTC/adapter/contact
            # code) -- see the module-level HAMMER_* comment for why. Spawned resting
            # near the handle's expected settled position; its pose is overwritten every
            # publish tick in update_hammer_head() below once the handle has settled.
            hammer_head = RigidObject(
                RigidObjectCfg(
                    prim_path="/World/HammerHead",
                    spawn=sim_utils.CuboidCfg(
                        size=HAMMER_HEAD_SIZE_M,
                        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
                        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.20, 0.20, 0.22), metallic=0.3),
                    ),
                    init_state=RigidObjectCfg.InitialStateCfg(
                        pos=(CUBE_XY_DEFAULT[0] + HAMMER_HEAD_LOCAL_OFFSET_M[0], CUBE_XY_DEFAULT[1], CUBE_DROP_Z)
                    ),
                )
            )
        place_marker = sim_utils.CuboidCfg(
            size=(0.06, 0.06, 0.002),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.10, 0.85, 0.20), metallic=0.0),
        )
        place_marker.func(
            "/World/PlaceZoneMarker", place_marker,
            translation=(PLACE_XY_DEFAULT[0], PLACE_XY_DEFAULT[1], TABLE_Z + 0.001),
        )
        if args.scene_manifest:
            clutter_objects = spawn_clutter_from_manifest(args.scene_manifest)
        left_contact = ContactSensor(
            ContactSensorCfg(
                prim_path="/World/Robot/link7", update_period=0, history_length=1,
                filter_prim_paths_expr=["/World/TargetCube"], force_threshold=0.01,
            )
        )
        right_contact = ContactSensor(
            ContactSensorCfg(
                prim_path="/World/Robot/link8", update_period=0, history_length=1,
                filter_prim_paths_expr=["/World/TargetCube"], force_threshold=0.01,
            )
        )
    elif dynamic_stick_mode:
        # A kinematic RigidObject with contact sensing. Swing mode drives a
        # vertical rod locally in world Y and reactive.py detects it from depth.
        # insert_bar mode parks a horizontal rod offstage until the ROS service
        # moves it into the corridor; official_style_demos.py then updates its
        # MotionPlanner world after confirming the rod's TF.
        # --stick none (STICK_ENABLED False): skip the rod -- for the static_cylinder demo
        # scenario, which wants a --scene-manifest obstacle in this same contact-sensor-
        # and-camera-wired scene WITHOUT an unrelated moving rod also in frame. See the
        # --stick arg's help text.
        if STICK_ENABLED:
            stick = RigidObject(
                RigidObjectCfg(
                    prim_path="/World/DynamicStick",
                    spawn=sim_utils.CuboidCfg(
                        size=STICK_SIZE_XYZ,
                        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
                        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
                        visual_material=sim_utils.PreviewSurfaceCfg(
                            diffuse_color=(0.90, 0.08, 0.08) if INSERT_BAR_MODE else (0.85, 0.55, 0.15), metallic=0.1
                        ),
                        activate_contact_sensors=True,
                    ),
                    init_state=RigidObjectCfg.InitialStateCfg(
                        pos=BAR_PARKED_XYZ if INSERT_BAR_MODE else (STICK_CENTER_X, STICK_CENTER_Y, STICK_CENTER_Z)
                    ),
                )
            )
            # The bar begins parked offstage, so a boot-time red-pixel probe
            # would fail before the insertion service is called.
            probe_target_color_rgb = None if INSERT_BAR_MODE else (0.85, 0.55, 0.15)
        if args.scene_manifest:
            # Additive static/dynamic clutter (e.g. a cylinder or cone obstacle for the
            # static_cylinder / p2p_cone demo scenarios) -- same spawner pick_place uses,
            # opened up to this scene 2026-09-24 (previously pick_place*-only, see the
            # spawn_clutter_from_manifest call site's original "only called for
            # pick_place* scenes" note in the dev guide, now stale).
            clutter_objects = spawn_clutter_from_manifest(args.scene_manifest)
            if probe_target_color_rgb is None and clutter_objects:
                # No stick to color-check against, but the first manifest object works
                # just as well as a "did the scene actually render" signal.
                probe_target_color_rgb = tuple(clutter_objects[0].get("color_rgb", (0.5, 0.5, 0.5)))

        # Ground-truth contact sensing (docs/dev_guide_paper_core_and_dashboard_plan.md
        # §8's "0 N contact force" pass criterion): filtered against BOTH the stick (if
        # enabled) and every manifest clutter object, not just the stick -- the
        # static_cylinder scenario has no stick at all, and previously had no contact
        # sensor covering its obstacle either. One WrenchStamped per monitored link is
        # still published either way; see publish_stick_and_contacts() below for how it
        # no longer requires `stick is not None`.
        obstacle_prim_paths = (["/World/DynamicStick"] if STICK_ENABLED else []) + [
            rec["prim_path"] for rec in clutter_objects
        ]
        if obstacle_prim_paths:
            stick_contacts = {
                link: ContactSensor(
                    ContactSensorCfg(
                        prim_path=f"/World/Robot/{link}", update_period=0, history_length=1,
                        filter_prim_paths_expr=obstacle_prim_paths, force_threshold=0.01,
                    )
                )
                for link in STICK_MONITORED_LINKS
            }
    else:
        # Original marker-only scene: unchanged from before this edit.
        marker = sim_utils.CuboidCfg(
            size=(0.035, 0.035, 0.035),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.88, 0.08, 0.04), metallic=0.05),
        )
        marker.func("/World/TargetMarker", marker, translation=(0.38, 0.0, 0.0175))
        probe_target_color_rgb = (0.88, 0.08, 0.04)  # matches the marker's own visual_material above

    if args.demo_targets != "none":
        if args.demo_targets == "static_multi":
            targets_doc = yaml.safe_load((root / "research/configs/demo/official_static_multi.yaml").read_text())
            marker_positions = [(entry["name"], entry["position_m"]) for entry in targets_doc["waypoints"]]
        else:
            targets_doc = yaml.safe_load((root / "research/configs/demo/official_dynamic_bar_targets.yaml").read_text())
            marker_positions = [(entry["name"], entry["position_m"]) for entry in targets_doc["waypoints"]]
        for label, xyz in marker_positions:
            marker_cfg = sim_utils.CuboidCfg(
                size=(0.030, 0.030, 0.030) if args.demo_targets == "dynamic_bar" else (0.035, 0.035, 0.035),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.08, 0.45, 0.95), metallic=0.05),
            )
            marker_cfg.func(f"/World/DemoTargets/{label}", marker_cfg, translation=tuple(xyz))

    robot = Articulation(
        ArticulationCfg(
            prim_path="/World/Robot",
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(root / "sim/assets/robot129/robot129/robot129.usda"),
                activate_contact_sensors=True,
            ),
            actuators={
                "arm": ImplicitActuatorCfg(
                    joint_names_expr=["joint[1-6]"], effort_limit_sim=100, stiffness=800, damping=80
                ),
                "gripper": ImplicitActuatorCfg(
                    joint_names_expr=["joint[78]"], effort_limit_sim=10, stiffness=2000, damping=100
                ),
            },
        )
    )
    demo_camera_period = 1.0 / 15.0 if args.demo_targets != "none" else 1.0 / 30.0
    demo_camera_height = 360 if args.demo_targets != "none" else 480
    demo_camera_width = 480 if args.demo_targets != "none" else 640
    camera_publish_interval = 8 if args.demo_targets != "none" else 4
    wrist_camera = Camera(
        CameraCfg(
            prim_path=WRIST_CAMERA_PRIM,
            update_period=demo_camera_period,
            height=demo_camera_height,
            width=demo_camera_width,
            data_types=["rgb", "distance_to_image_plane"],
            update_latest_camera_pose=True,
            offset=CameraCfg.OffsetCfg(
                pos=wrist_camera_offset()[0],
                rot=wrist_camera_offset()[1],  # (x, y, z, w)
                convention="ros",
            ),
            spawn=sim_utils.PinholeCameraCfg(
                # real_calib: fx = fy = 615 px at 640x480 (D435 color, nominal -- the real
                # node reads the live camera_info, so only the FOV matters for the sim).
                focal_length=28.0 if WRIST_CAMERA_MODEL == "urdf_nominal" else 615.0 * 20.955 / 640.0,
                horizontal_aperture=20.955,
                clipping_range=(0.05, 5.0),
            ),
        )
    )
    # Was `if args.record_only:` -- an offscreen recording camera, independent of the
    # interactive `viewport` set up below, is what the /robot129_sim/recording service
    # (handle_recording -> the sim-loop capture block) actually reads frames from. Gating
    # it on record_only made it mutually exclusive with --livestream (viewport is set
    # exactly when NOT record_only), so no WebRTC session could ever produce a video: the
    # recording service would report "started", capture zero frames, and never finalize
    # a video.mp4 -- silently, since handle_recording's stop path only skips writing
    # anything when frame_index==0 rather than reporting an error. Found 2026-09-20
    # running tools/run_grasp_demo_live.sh with --record for the first time; every prior
    # successful recording had been made via start_robot129_grasp_sim.sh (--record-only,
    # headless, no viewport). Always creating this camera fixes both paths at once and
    # costs one small offscreen render product even when nothing is being recorded.
    overview_camera = Camera(
        CameraCfg(
            prim_path="/World/RecordingCamera",
            update_period=demo_camera_period if args.demo_targets != "none" else 0,
            height=demo_camera_height, width=demo_camera_width,
            data_types=["rgb"], background_color=(0.055, 0.065, 0.080),
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=28, horizontal_aperture=20.955, clipping_range=(0.05, 5)
            ),
        )
    )

    scene_camera = None
    if SCENE_CAMERA_ENABLED:
        # Pole: visual + static collision, standing in for the real aluminum-extrusion
        # pole in the user's photo (not the cart -- see scene_camera.yaml's header).
        # Runs from the base-mount plane (z=0) up to the camera height, centered under
        # the camera's xy so it visually/physically supports it.
        pole_half = SCENE_CAMERA_POLE_CFG["half_extent_m"]
        pole_height = SCENE_CAMERA_OFFSET_FROM_BASE_M[2]
        pole = sim_utils.CuboidCfg(
            size=(pole_half * 2.0, pole_half * 2.0, pole_height),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=tuple(SCENE_CAMERA_POLE_CFG["color_rgb"]), roughness=0.6
            ),
        )
        pole.func(
            "/World/ScenePole", pole,
            translation=(SCENE_CAMERA_OFFSET_FROM_BASE_M[0], SCENE_CAMERA_OFFSET_FROM_BASE_M[1], pole_height / 2.0),
        )
        scene_camera = Camera(
            CameraCfg(
                prim_path=SCENE_CAMERA_PRIM,
                update_period=demo_camera_period,
                height=demo_camera_height if args.demo_targets != "none" else SCENE_CAMERA_INTRINSICS["height"],
                width=demo_camera_width if args.demo_targets != "none" else SCENE_CAMERA_INTRINSICS["width"],
                data_types=["rgb", "distance_to_image_plane"],
                update_latest_camera_pose=True,
                # No parent prim (unlike wrist_camera, which nests under the robot's own
                # USD hierarchy) -- a bare /World prim, pose set below via
                # set_world_poses_from_view, same pattern as overview_camera.
                spawn=sim_utils.PinholeCameraCfg(
                    focal_length=SCENE_CAMERA_INTRINSICS["focal_length_mm"],
                    horizontal_aperture=SCENE_CAMERA_INTRINSICS["horizontal_aperture_mm"],
                    clipping_range=tuple(SCENE_CAMERA_INTRINSICS["clipping_range_m"]),
                ),
            )
        )

    sim.reset()
    if overview_camera is not None:
        overview_eye = (1.30, 1.20, 1.10) if args.demo_targets != "none" else OVERVIEW_CAMERA_EYE_M
        overview_target = ((0.20, 0.0, 0.40) if args.demo_targets == "dynamic_bar" else (0.20, 0.22, 0.38)) if args.demo_targets != "none" else (
            OVERVIEW_CAMERA_TARGET_XY_M[0], OVERVIEW_CAMERA_TARGET_XY_M[1], OVERVIEW_CAMERA_TARGET_Z_BASE_M + TABLE_Z
        )
        overview_camera.set_world_poses_from_view(
            eyes=torch.tensor([list(overview_eye)], device=sim.device),
            targets=torch.tensor([list(overview_target)], device=sim.device),
        )
    if scene_camera is not None:
        # Aim from the configured pole-top position toward the A/B corridor using
        # the authored yaw while keeping the specified pitch. This is a simulation
        # framing choice, not a calibrated physical camera transform.
        # set_world_poses_from_view derives the camera's
        # orientation from eye->target, and .data.quat_w_ros (used for publishing, see
        # publish_camera_frame) is computed from the live prim transform regardless of
        # how that transform was set -- same mechanism overview_camera already relies on.
        eye = np.array(SCENE_CAMERA_OFFSET_FROM_BASE_M, dtype=np.float64)
        pitch = math.radians(SCENE_CAMERA_PITCH_DEG)
        yaw = math.radians(SCENE_CAMERA_YAW_DEG)
        forward = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), -math.sin(pitch)])
        target = (np.array([0.20, 0.0, 0.40], dtype=np.float64)
                  if args.demo_targets == "dynamic_bar" else eye + forward)
        scene_camera.set_world_poses_from_view(
            eyes=torch.tensor([eye.tolist()], dtype=torch.float32, device=sim.device),
            targets=torch.tensor([target.tolist()], dtype=torch.float32, device=sim.device),
        )
    home_tensor = torch.tensor(HOME, dtype=torch.float32, device=sim.device).unsqueeze(0)
    robot.write_joint_position_to_sim_index(position=home_tensor)
    robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(home_tensor))
    target = HOME.copy()
    plans = {"arm": None, "gripper": None, "combined": None}
    counters = {"accepted": 0, "rejected": 0, "published_states": 0}

    viewport = None
    if not args.record_only:
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport is None:
            print("[ROBOT129 ROS WEBRTC] FAIL - active viewport unavailable", flush=True)
            return 3
        if args.demo_targets != "none":
            set_live_camera(viewport, eye=(1.30, 1.20, 1.10), target=(0.20, 0.22, 0.38))
        else:
            set_live_camera(viewport)

    stick_step_count = {"n": 0}  # local sim-time source independent of the ROS-loop
    bar_state = {"inserted": False, "inserted_sim_time_s": None}
    # frame_state dict below (which doesn't exist yet during the warmup phase, but
    # advance() runs there too) -- see the STICK_* module comment.

    def advance():
        if stick is not None:
            # Formula-driven pose (not physics-driven like the hammer head follows the
            # cube), so update it every physics step for smooth motion and reliable
            # contact detection, not just once per ROS publish tick.
            stick_time_s = stick_step_count["n"] * (1.0 / 120.0)
            stick_step_count["n"] += 1
            y = STICK_CENTER_Y + STICK_SWEEP_Y_AMPLITUDE_M * math.sin(2.0 * math.pi * stick_time_s / STICK_SWEEP_PERIOD_S)
            xyz = (BAR_CENTER_XYZ if bar_state["inserted"] else BAR_PARKED_XYZ) if INSERT_BAR_MODE else (STICK_CENTER_X, y, STICK_CENTER_Z)
            stick_pose = torch.tensor(
                [[*xyz, 1.0, 0.0, 0.0, 0.0]], dtype=torch.float32, device=sim.device,
            )
            stick.write_root_pose_to_sim_index(root_pose=stick_pose, env_ids=None)
            stick.write_root_velocity_to_sim_index(
                root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=sim.device), env_ids=None,
            )
        robot.write_data_to_sim()
        step_sim()
        dt = sim.get_physics_dt()
        robot.update(dt)
        wrist_camera.update(dt)
        if cube is not None:
            cube.update(dt)
        if stick is not None:
            stick.update(dt)
        if left_contact is not None:
            left_contact.update(dt)
            right_contact.update(dt)
        for sensor in stick_contacts.values():
            sensor.update(dt)
        if overview_camera is not None:
            overview_camera.update(dt)
        if scene_camera is not None:
            scene_camera.update(dt)

    print(f"[ROBOT129 ROS WEBRTC] WARMING_UP - {args.warmup_frames} rendered frames", flush=True)
    try:
        for warmup_index in range(args.warmup_frames):
            robot.set_joint_position_target_index(target=torch.tensor(target, dtype=torch.float32, device=sim.device).unsqueeze(0))
            advance()
    except BaseException as exc:
        print(f"[ROBOT129 ROS WEBRTC] WARMUP_EXCEPTION index={warmup_index} type={type(exc).__name__} value={exc!r}", flush=True)
        raise

    if cube is not None:
        # Let gravity settle the cube from its drop height onto the floor before READY.
        for _ in range(SETTLE_STEPS):
            robot.set_joint_position_target_index(target=torch.tensor(target, dtype=torch.float32, device=sim.device).unsqueeze(0))
            advance()

    if viewport is not None:
        if args.demo_targets != "none":
            set_live_camera(viewport, eye=(1.30, 1.20, 1.10), target=(0.20, 0.22, 0.38))
        else:
            set_live_camera(viewport)
        for _ in range(15):
            advance()
        probe = capture_viewport_probe(root, viewport, advance, probe_target_color_rgb)
        print(f"[ROBOT129 ROS WEBRTC] VIEWPORT_PROBE {probe['status']} - {probe}", flush=True)
        if probe["status"] != "PASS":
            return 4
    else:
        # record-only path: validate the offscreen recording camera instead of the
        # interactive viewport, matching verify_robot129_physics_grasp.py's non-realtime
        # capture pattern rather than depending on a livestream session that was never
        # requested.
        for _ in range(15):
            advance()
        rgb = overview_camera.data.output["rgb"].torch[0, ..., :3].detach().cpu().numpy().astype(np.float32)
        luma = 0.2126 * rgb[..., 0] + 0.7152 * rgb[..., 1] + 0.0722 * rgb[..., 2]
        probe_ok = float(luma.mean()) > 5 and float(luma.std()) > 3
        print(f"[ROBOT129 ROS WEBRTC] RECORDING_CAMERA_PROBE {'PASS' if probe_ok else 'FAIL'}", flush=True)
        if not probe_ok:
            return 4

    print("[ROBOT129 ROS WEBRTC] VIEWPORT_READY - loading ROS 2 Python runtime", flush=True)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped, TransformStamped, WrenchStamped
    from sensor_msgs.msg import CameraInfo, Image, JointState
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, Trigger
    from tf2_msgs.msg import TFMessage
    from trajectory_msgs.msg import JointTrajectory

    rclpy.init()
    node = rclpy.create_node("isaac_articulation_bridge", namespace="/robot129_sim")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    state_pub = node.create_publisher(JointState, "/robot129_sim/joint_states", 10)
    # Real PiPER driver interface (piper_ros, as used by references/upstream mm_actions):
    # feedback = joints 1-6 + "gripper" total opening in metres; the sim's two fingers each
    # travel 0-0.035 m, so opening = 2 * joint7 (0-0.07 m; the real gripper reaches 0.1).
    piper_feedback_pub = node.create_publisher(JointState, "/robot129_sim/piper/joint_states_feedback", 10)
    command_pub = node.create_publisher(JointState, "/robot129_sim/joint_command", 10)
    events_pub = node.create_publisher(String, "/robot129_sim/trajectory_events", 10)
    latched_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1,
        reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    scene_state_pub = node.create_publisher(String, "/robot129_sim/scene_state", latched_qos)
    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    rgb_pub = node.create_publisher(Image, RGB_TOPIC, sensor_qos)
    depth_pub = node.create_publisher(Image, DEPTH_TOPIC, sensor_qos)
    info_pub = node.create_publisher(CameraInfo, CAMERA_INFO_TOPIC, sensor_qos)
    tf_pub = node.create_publisher(TFMessage, "/tf", 10)
    # Global view (dashboard window 2): unconditional, like overview_camera itself --
    # previously this camera only fed the --record video, now also live on ROS.
    overview_rgb_pub = node.create_publisher(Image, OVERVIEW_RGB_TOPIC, sensor_qos)
    overview_info_pub = node.create_publisher(CameraInfo, OVERVIEW_CAMERA_INFO_TOPIC, sensor_qos)
    latest_camera_dir = root / "out/ros_webrtc_robot129/wrist_camera"
    latest_camera_dir.mkdir(parents=True, exist_ok=True)

    # --scene-camera pole publishers, same pattern as the wrist camera's, on their own
    # topics/frame so both cameras can be subscribed to simultaneously. None when
    # --scene-camera off (SCENE_CAMERA_ENABLED False), matching scene_camera itself.
    scene_rgb_pub = scene_depth_pub = scene_info_pub = None
    latest_scene_camera_dir = None
    if SCENE_CAMERA_ENABLED:
        scene_rgb_pub = node.create_publisher(Image, SCENE_RGB_TOPIC, sensor_qos)
        scene_depth_pub = node.create_publisher(Image, SCENE_DEPTH_TOPIC, sensor_qos)
        scene_info_pub = node.create_publisher(CameraInfo, SCENE_CAMERA_INFO_TOPIC, sensor_qos)
        latest_scene_camera_dir = root / "out/ros_webrtc_robot129/scene_camera"
        latest_scene_camera_dir.mkdir(parents=True, exist_ok=True)

    cube_pose_pub = None
    contact_pubs = None
    if pick_place:
        cube_pose_pub = node.create_publisher(PoseStamped, "/robot129_sim/objects/target_cube/pose", sensor_qos)
        contact_pubs = {
            "link7": node.create_publisher(WrenchStamped, "/robot129_sim/contacts/link7", sensor_qos),
            "link8": node.create_publisher(WrenchStamped, "/robot129_sim/contacts/link8", sensor_qos),
        }

    stick_contact_pubs = None
    if dynamic_stick_mode:
        stick_contact_pubs = {
            link: node.create_publisher(WrenchStamped, f"/robot129_sim/contacts/{link}", sensor_qos)
            for link in STICK_MONITORED_LINKS
        }

    def publish_event(kind: str, key: str, **fields):
        payload = {"kind": kind, "channel": key, "stamp_sim_time": frame_state["sim_time"]}
        payload.update(fields)
        msg = String()
        msg.data = json.dumps(payload)
        events_pub.publish(msg)

    def publish_camera_frame(camera, frame_id, rgb_publisher, depth_publisher, info_publisher, stamp):
        """Shared publish path for any Camera sensor with data_types=["rgb",
        "distance_to_image_plane"]: RGB image, depth image (32FC1, metres, along the
        optical axis -- NOT range), CameraInfo, and a world->frame_id TF. Used by both
        the wrist camera (frame_id=WRIST_FRAME, unchanged behaviour/topics from before
        --scene-camera existed) and the --scene-camera pole camera
        (frame_id=SCENE_CAMERA_FRAME, its own topics) -- see SCENE_CAMERA_* constants
        and publish_wrist_camera/publish_scene_camera below.
        """
        rgb = camera.data.output["rgb"].torch[0, ..., :3].detach().cpu().numpy().astype(np.uint8)
        depth = camera.data.output["distance_to_image_plane"].torch[0].detach().cpu().numpy().astype(np.float32)
        depth = np.squeeze(depth)
        intrinsics = camera.data.intrinsic_matrices.torch[0].detach().cpu().numpy()
        position = camera.data.pos_w.torch[0].detach().cpu().numpy()
        quaternion = camera.data.quat_w_ros.torch[0].detach().cpu().numpy()

        rgb_msg = Image()
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = frame_id
        rgb_msg.height, rgb_msg.width = rgb.shape[:2]
        rgb_msg.encoding = "rgb8"
        rgb_msg.is_bigendian = False
        rgb_msg.step = int(rgb.shape[1] * 3)
        rgb_msg.data = rgb.tobytes()
        rgb_publisher.publish(rgb_msg)

        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = frame_id
        depth_msg.height, depth_msg.width = depth.shape
        depth_msg.encoding = "32FC1"
        depth_msg.is_bigendian = False
        depth_msg.step = int(depth.shape[1] * 4)
        depth_msg.data = depth.tobytes()
        depth_publisher.publish(depth_msg)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.height, info.width = depth.shape
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = intrinsics.reshape(-1).tolist()
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            float(intrinsics[0, 0]), 0.0, float(intrinsics[0, 2]), 0.0,
            0.0, float(intrinsics[1, 1]), float(intrinsics[1, 2]), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        info_publisher.publish(info)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = frame_id
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        tf_pub.publish(TFMessage(transforms=[transform]))
        return rgb, depth, intrinsics, position, quaternion

    def publish_wrist_camera(stamp):
        return publish_camera_frame(wrist_camera, WRIST_FRAME, rgb_pub, depth_pub, info_pub, stamp)

    def publish_scene_camera(stamp):
        return publish_camera_frame(
            scene_camera, SCENE_CAMERA_FRAME, scene_rgb_pub, scene_depth_pub, scene_info_pub, stamp
        )

    def publish_overview_camera(stamp):
        """RGB + CameraInfo only (dashboard window 2, docs/dev_guide_paper_core_and_
        dashboard_plan.md §8) -- no depth, no TF. overview_camera's CameraCfg has no
        `update_latest_camera_pose=True` (only affects .data.pos_w/quat_w_ros, which
        this function doesn't touch, so it doesn't need to change), and its pose is
        fixed at startup by set_world_poses_from_view() above, so a TF would be
        redundant with a single well-known static transform anyway -- not published,
        to keep this camera genuinely cheaper than the wrist/scene RGB-D pair, matching
        why it's polled at a lower rate in the main loop (frame % 8, not frame % 4).
        """
        rgb = overview_camera.data.output["rgb"].torch[0, ..., :3].detach().cpu().numpy().astype(np.uint8)
        intrinsics = overview_camera.data.intrinsic_matrices.torch[0].detach().cpu().numpy()

        rgb_msg = Image()
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = OVERVIEW_CAMERA_FRAME
        rgb_msg.height, rgb_msg.width = rgb.shape[:2]
        rgb_msg.encoding = "rgb8"
        rgb_msg.is_bigendian = False
        rgb_msg.step = int(rgb.shape[1] * 3)
        rgb_msg.data = rgb.tobytes()
        overview_rgb_pub.publish(rgb_msg)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = OVERVIEW_CAMERA_FRAME
        info.height, info.width = rgb.shape[:2]
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = intrinsics.reshape(-1).tolist()
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            float(intrinsics[0, 0]), 0.0, float(intrinsics[0, 2]), 0.0,
            0.0, float(intrinsics[1, 1]), float(intrinsics[1, 2]), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        overview_info_pub.publish(info)

    def update_hammer_head():
        """Drive the kinematic head to the handle's live pose composed with a fixed
        local offset -- see the HAMMER_* module comment for why this is kinematic
        follow rather than a true compound rigid body. xyzw quaternion throughout,
        matching every other pose in this script."""
        if hammer_head is None:
            return
        pos = cube.data.root_pos_w.torch[0].detach().cpu().numpy()
        quat = cube.data.root_quat_w.torch[0].detach().cpu().numpy()  # xyzw
        qx, qy, qz, qw = quat
        # standard quaternion-rotate-vector: v' = q * v * q^-1, expanded closed-form.
        offset = np.array(HAMMER_HEAD_LOCAL_OFFSET_M, dtype=np.float64)
        qvec = np.array([qx, qy, qz])
        uv = np.cross(qvec, offset)
        uuv = np.cross(qvec, uv)
        rotated_offset = offset + 2.0 * (qw * uv + uuv)
        head_pos = pos + rotated_offset
        head_pose = torch.tensor(
            [[head_pos[0], head_pos[1], head_pos[2], qx, qy, qz, qw]], dtype=torch.float32, device=sim.device
        )
        hammer_head.write_root_pose_to_sim_index(root_pose=head_pose, env_ids=None)

    def publish_object_and_contacts(stamp):
        if cube is None:
            return
        update_hammer_head()
        pos = cube.data.root_pos_w.torch[0].detach().cpu().numpy()
        quat = cube.data.root_quat_w.torch[0].detach().cpu().numpy()  # xyzw
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = (float(v) for v in pos)
        (pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
         pose_msg.pose.orientation.z, pose_msg.pose.orientation.w) = (float(v) for v in quat)
        cube_pose_pub.publish(pose_msg)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = "target_cube"
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = (
            float(v) for v in pos
        )
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = (float(v) for v in quat)
        tf_pub.publish(TFMessage(transforms=[transform]))

        for name, sensor in (("link7", left_contact), ("link8", right_contact)):
            raw = sensor.data.normal_force_matrix_w
            force = np.zeros(3, dtype=np.float64) if raw is None else raw.torch[0].detach().cpu().numpy().reshape(-1)[:3]
            wrench = WrenchStamped()
            wrench.header.stamp = stamp
            wrench.header.frame_id = name
            wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z = (float(v) for v in force)
            contact_pubs[name].publish(wrench)

    def publish_stick_and_contacts(stamp):
        """--scene dynamic_stick only. Publishes the stick's own TF (for offline
        analysis/visualization; NOT fed to any planner -- see reactive.py's module
        docstring), if the stick actually exists (--stick swing, the default) -- but the
        per-link WrenchStamped contact publish below runs whenever ANY obstacle contact
        sensor was created (stick and/or manifest clutter, see obstacle_prim_paths
        above), independent of whether the stick specifically is present. static_cylinder
        (--stick none + --scene-manifest) has no stick to publish a TF for, but still
        needs its own 0-contact-force ground truth published -- same pattern
        publish_object_and_contacts uses for the pick_place cube's link7/link8.
        """
        if stick is not None:
            pos = stick.data.root_pos_w.torch[0].detach().cpu().numpy()
            quat = stick.data.root_quat_w.torch[0].detach().cpu().numpy()  # xyzw
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "world"
            transform.child_frame_id = STICK_FRAME
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = (
                float(v) for v in pos
            )
            (transform.transform.rotation.x, transform.transform.rotation.y,
             transform.transform.rotation.z, transform.transform.rotation.w) = (float(v) for v in quat)
            tf_pub.publish(TFMessage(transforms=[transform]))

        for name, sensor in stick_contacts.items():
            raw = sensor.data.normal_force_matrix_w
            force = np.zeros(3, dtype=np.float64) if raw is None else raw.torch[0].detach().cpu().numpy().reshape(-1)[:3]
            wrench = WrenchStamped()
            wrench.header.stamp = stamp
            wrench.header.frame_id = name
            wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z = (float(v) for v in force)
            stick_contact_pubs[name].publish(wrench)

    def save_latest_camera(out_dir, frame_id, camera_prim, calibration_tag, attached_to, rgb, depth, intrinsics, position, quaternion):
        """Shared "latest frame to disk" path for any camera, atomic-replace style (so a
        concurrent reader, e.g. tools/robot129_camera_viewer.py or research/scripts/
        capture_scene.py, never sees a half-written file). Used by both
        save_latest_wrist (unchanged output layout: out/ros_webrtc_robot129/wrist_camera/)
        and save_latest_scene_camera (out/ros_webrtc_robot129/scene_camera/).
        """
        from PIL import Image as PILImage

        def atomic_text(name, payload):
            final = out_dir / name
            temporary = out_dir / (name + ".tmp")
            temporary.write_text(payload)
            os.replace(temporary, final)

        rgb_final = out_dir / "rgb.png"
        rgb_temporary = out_dir / "rgb.png.tmp"
        PILImage.fromarray(rgb).save(rgb_temporary, format="PNG")
        os.replace(rgb_temporary, rgb_final)

        depth_final = out_dir / "depth.npy"
        depth_temporary = out_dir / "depth.npy.tmp"
        with depth_temporary.open("wb") as stream:
            np.save(stream, depth)
        os.replace(depth_temporary, depth_final)

        atomic_text(
            "camera_info.json",
            json.dumps(
                {
                    "width": int(rgb.shape[1]),
                    "height": int(rgb.shape[0]),
                    "k": intrinsics.reshape(-1).tolist(),
                    "d": [0.0] * 5,
                    "depth_scale_m": 1.0,
                    "frame_id": frame_id,
                    "depth_units": "meter",
                    "depth_definition": "optical_axis_z",
                    "calibration": calibration_tag,
                    "physical_calibration_verified": False,
                },
                indent=2,
            ) + "\n",
        )
        atomic_text(
            "tf.json",
            json.dumps(
                {
                    "status": "AVAILABLE",
                    "translation_xyz_m": position.tolist(),
                    "quaternion_xyzw": quaternion.tolist(),
                    "base_frame": "world",
                    "camera_frame": frame_id,
                    "camera_prim": camera_prim,
                    "attached_to": attached_to,
                    "physical_wrist_extrinsic_verified": False,
                },
                indent=2,
            ) + "\n",
        )

    def save_latest_wrist(rgb, depth, intrinsics, position, quaternion):
        save_latest_camera(
            latest_camera_dir, WRIST_FRAME, WRIST_CAMERA_PRIM, "ISAAC_WRIST_CAMERA_NOMINAL",
            WRIST_OPTICAL_PARENT, rgb, depth, intrinsics, position, quaternion,
        )

    def save_latest_scene_camera(rgb, depth, intrinsics, position, quaternion):
        save_latest_camera(
            latest_scene_camera_dir, SCENE_CAMERA_FRAME, SCENE_CAMERA_PRIM, "ISAAC_SCENE_CAMERA_ROUGH_FROM_PHOTO",
            "/World (standalone, not attached to the robot)", rgb, depth, intrinsics, position, quaternion,
        )

    def make_callback(key: str, expected_names: list[str], indices: list[int]):
        tol = T0_TOLERANCE[indices]

        def callback(msg):
            nonlocal target
            reason = None
            times: list[float] = []
            points: list[np.ndarray] = []
            velocities: list[np.ndarray] = []
            has_velocity = False

            if list(msg.joint_names) != expected_names:
                reason = f"joint_names must be {expected_names}"
            elif not msg.points:
                reason = "trajectory has no points"
            else:
                raw_times = [trajectory_seconds(point) for point in msg.points]
                raw_points_msgs = list(msg.points)

                if raw_times and raw_times[0] == 0.0:
                    first = raw_points_msgs[0]
                    if len(first.positions) != len(indices):
                        reason = "t=0 first point position count does not match joint_names"
                    else:
                        first_pos = np.asarray(first.positions, dtype=np.float64)
                        current = target[indices]
                        if np.all(np.abs(first_pos - current) <= tol):
                            raw_times = raw_times[1:]
                            raw_points_msgs = raw_points_msgs[1:]
                        else:
                            reason = (
                                "t=0 first point must match the current commanded target "
                                f"within tolerance {tol.tolist()}"
                            )

                if reason is None and not raw_points_msgs:
                    reason = "trajectory has no points after dropping the matched t=0 first point"
                elif reason is None:
                    if any(t <= 0 or t > 120 for t in raw_times) or any(b <= a for a, b in zip(raw_times, raw_times[1:])):
                        reason = "time_from_start must be strictly increasing in (0, 120] seconds"
                    elif any(len(point.positions) != len(indices) for point in raw_points_msgs):
                        reason = "position count does not match joint_names"
                    else:
                        points = [np.asarray(point.positions, dtype=np.float64) for point in raw_points_msgs]
                        if not all(np.isfinite(point).all() for point in points):
                            reason = "positions must be finite"
                        elif any(
                            np.any(point < LIMITS[indices, 0]) or np.any(point > LIMITS[indices, 1])
                            for point in points
                        ):
                            reason = "position violates Robot 129 joint limits"
                        else:
                            has_vels = [len(point.velocities) == len(indices) for point in raw_points_msgs]
                            if any(has_vels) and not all(has_vels):
                                reason = "velocities must be provided for all points or none"
                            else:
                                has_velocity = all(has_vels) and len(has_vels) > 0
                                if has_velocity:
                                    velocities = [
                                        np.asarray(point.velocities, dtype=np.float64) for point in raw_points_msgs
                                    ]
                                    if not all(np.isfinite(v).all() for v in velocities):
                                        reason = "velocities must be finite"
                                if reason is None:
                                    times = raw_times

            if reason is not None:
                counters["rejected"] += 1
                print(f"[ROBOT129 ROS WEBRTC] REJECT {key} - {reason}", flush=True)
                publish_event("REJECT", key, reason=reason)
                return

            if not has_velocity:
                velocities = [np.zeros(len(indices), dtype=np.float64) for _ in points]
            plans[key] = {
                "started": frame_state["sim_time"],
                "initial": target[indices].copy(),
                "initial_velocity": np.zeros(len(indices), dtype=np.float64),
                "times": times,
                "points": points,
                "velocities": velocities,
                "has_velocity": has_velocity,
                "indices": indices,
            }
            if key == "combined":
                plans["arm"] = None
                plans["gripper"] = None
            else:
                plans["combined"] = None
            counters["accepted"] += 1
            print(
                f"[ROBOT129 ROS WEBRTC] ACCEPT {key} sequence={counters['accepted']} duration={times[-1]:.3f}s "
                f"points={len(points)} has_velocity={has_velocity}",
                flush=True,
            )
            publish_event("ACCEPT", key, duration_s=times[-1], n_points=len(points), has_velocity=has_velocity)

        return callback

    def handle_piper_joint_cmd(msg):
        # Streaming position command, same shape the real driver consumes: the real
        # mm_actions servo loop publishes one JointState per 25 ms and expects the arm to
        # track it directly, so there is no trajectory interpolation here. A streamed
        # command supersedes any active JointTrajectory plan.
        positions = dict(zip(msg.name, msg.position))
        missing = [n for n in ARM_NAMES if n not in positions]
        values = [positions.get(n, 0.0) for n in ARM_NAMES]
        if missing or not np.all(np.isfinite(values)):
            print(f"[ROBOT129 ROS WEBRTC] REJECT piper_cmd - missing={missing} or non-finite", flush=True)
            return
        for key in plans:
            plans[key] = None
        target[:6] = np.clip(values, LIMITS[:6, 0], LIMITS[:6, 1])
        if "gripper" in positions and math.isfinite(positions["gripper"]):
            target[6] = float(np.clip(positions["gripper"] / 2.0, LIMITS[6, 0], LIMITS[6, 1]))

    subscriptions = [
        node.create_subscription(JointState, "/robot129_sim/piper/joint_cmd", handle_piper_joint_cmd, 10),
        node.create_subscription(
            JointTrajectory,
            "/robot129_sim/arm_controller/joint_trajectory",
            make_callback("arm", ARM_NAMES, list(range(6))),
            10,
        ),
        node.create_subscription(
            JointTrajectory,
            "/robot129_sim/gripper_controller/joint_trajectory",
            make_callback("gripper", GRIPPER_NAMES, [6]),
            10,
        ),
        node.create_subscription(
            JointTrajectory,
            "/robot129_sim/joint_trajectory",
            make_callback("combined", ACTIVE_NAMES, list(range(7))),
            10,
        ),
    ]

    # --- reset_scene / recording services (declared regardless of scene, harmless no-ops
    # for the cube-less marker scene beyond re-homing the arm). ---
    node.declare_parameter("reset.seed", args.seed)
    node.declare_parameter("reset.cube_xy", list(CUBE_XY_DEFAULT))
    node.declare_parameter("reset.cube_yaw", 0.0)
    node.declare_parameter("reset.place_xy", list(PLACE_XY_DEFAULT))
    scene_revision = {"value": 0}
    # Resume the run_id counter from whatever sessions/ already has, not always -1 (so
    # the first recording of THIS process becomes run_0000): a fresh Isaac process always
    # started counting at -1 before, so any restart silently overwrote run_0000 (and
    # run_0001, etc.) from a PREVIOUS process's recordings -- real evidence from an
    # earlier session was lost this way on 2026-09-20 before this fix existed.
    sessions_dir = root / "out/grasp_motion/sessions"
    existing_run_ids = [
        int(p.name[len("run_"):]) for p in sessions_dir.glob("run_[0-9][0-9][0-9][0-9]")
        if p.name[len("run_"):].isdigit()
    ] if sessions_dir.is_dir() else []
    recording_state = {
        "active": False, "run_id": max(existing_run_ids, default=-1), "frame_index": 0,
        "wrist_frame_index": 0, "scene_frame_index": 0, "dir": None,
        "last_saved": 0.0, "wrist_last_saved": 0.0, "scene_last_saved": 0.0,
    }
    frame_state = {"sim_time": 0.0, "count": 0}

    def publish_scene_state():
        payload = {
            "revision": scene_revision["value"],
            "scene": args.scene,
            "target_object": args.target_object if pick_place and not hammer_mode and not COUNTER_SCENE else None,
            "seed": node.get_parameter("reset.seed").value,
            "cube_xy": node.get_parameter("reset.cube_xy").value,
            "cube_yaw": node.get_parameter("reset.cube_yaw").value,
            "place_xy": node.get_parameter("reset.place_xy").value,
            "sim_time_s": frame_state["sim_time"],
        }
        msg = String()
        msg.data = json.dumps(payload)
        scene_state_pub.publish(msg)

    def handle_reset_scene(_request, response):
        if any(plans.values()):
            response.success = False
            response.message = "BUSY: a trajectory plan is active, reject reset until idle"
            return response
        rng = np.random.default_rng(int(node.get_parameter("reset.seed").value))
        cube_xy = list(node.get_parameter("reset.cube_xy").value)
        cube_yaw = float(node.get_parameter("reset.cube_yaw").value)
        nonlocal target
        target = HOME.copy()
        robot.write_joint_position_to_sim_index(
            position=torch.tensor(HOME, dtype=torch.float32, device=sim.device).unsqueeze(0)
        )
        robot.write_joint_velocity_to_sim_index(
            velocity=torch.zeros((1, 8), dtype=torch.float32, device=sim.device)
        )
        if cube is not None:
            qx, qy, qz, qw = yaw_to_quat_xyzw(cube_yaw)
            # target_drop_z, not module-level CUBE_DROP_Z: matches whichever
            # --target-object shape was actually spawned (see its assignment above).
            pose = torch.tensor(
                [[cube_xy[0], cube_xy[1], target_drop_z, qx, qy, qz, qw]], dtype=torch.float32, device=sim.device
            )
            cube.write_root_pose_to_sim_index(root_pose=pose, env_ids=None)
            cube.write_root_velocity_to_sim_index(
                root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=sim.device), env_ids=None
            )
            # hammer_mode only: the kinematic head is otherwise only re-derived from the
            # handle's pose on the next ROS publish tick (update_hammer_head(), called
            # from publish_object_and_contacts()) -- without this call it would sit at
            # its stale pre-reset pose for however long that tick takes to arrive,
            # visually and physically detached from the handle it just snapped back to.
            update_hammer_head()
        scene_revision["value"] += 1
        publish_scene_state()
        response.success = True
        response.message = json.dumps({
            "revision": scene_revision["value"],
            "seed": int(node.get_parameter("reset.seed").value),
            "cube_xy": cube_xy,
            "cube_yaw": cube_yaw,
        })
        print(f"[ROBOT129 ROS WEBRTC] RESET_SCENE revision={scene_revision['value']}", flush=True)
        return response

    def start_recording():
        recording_state["run_id"] += 1
        run_dir = root / "out/grasp_motion/sessions" / f"run_{recording_state['run_id']:04d}"
        (run_dir / "frames").mkdir(parents=True, exist_ok=True)
        # Third-person (fixed overview_camera) into frames/ -> video.mp4, as before.
        # Also record first-person -- the actual wrist_camera the VLM judges the scene
        # from -- into wrist_frames/ -> wrist_video.mp4, so a run's video answers "what
        # did the gripper's own camera see, and what did it do about it" instead of only
        # "what did an external observer see." Added 2026-09-21 on request: until now
        # only the third-person view was ever recorded.
        (run_dir / "wrist_frames").mkdir(parents=True, exist_ok=True)
        # Third stream, --scene-camera pole only: scene_frames/ -> scene_video.mp4, the
        # fixed back/pole viewpoint. Same opt-in pattern as wrist_frames above.
        if SCENE_CAMERA_ENABLED:
            (run_dir / "scene_frames").mkdir(parents=True, exist_ok=True)
        recording_state["dir"] = run_dir
        recording_state["frame_index"] = 0
        recording_state["wrist_frame_index"] = 0
        recording_state["scene_frame_index"] = 0
        recording_state["active"] = True
        recording_state["last_saved"] = -1.0
        recording_state["wrist_last_saved"] = -1.0
        recording_state["scene_last_saved"] = -1.0
        (run_dir / "manifest.json").write_text(json.dumps({
            "run_id": recording_state["run_id"],
            "started_sim_time_s": frame_state["sim_time"],
            "scene": args.scene,
            "fps": args.record_fps,
            "status": "RECORDING",
        }, indent=2))

    def stop_recording():
        recording_state["active"] = False
        run_dir = recording_state["dir"]
        if run_dir is None or recording_state["frame_index"] == 0:
            return
        manifest_path = run_dir / "manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["status"] = "ENCODING"
        manifest["frame_count"] = recording_state["frame_index"]
        manifest["stopped_sim_time_s"] = frame_state["sim_time"]
        manifest_path.write_text(json.dumps(manifest, indent=2))
        video_path = run_dir / "video.mp4"
        log_path = run_dir / "ffmpeg.log"
        with log_path.open("wb") as log_file:
            subprocess.Popen(
                [
                    "ffmpeg", "-y", "-framerate", str(args.record_fps),
                    "-i", str(run_dir / "frames" / "frame_%06d.png"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                    str(video_path),
                ],
                stdout=log_file, stderr=subprocess.STDOUT,
            )
        wrist_video_path = run_dir / "wrist_video.mp4"
        wrist_log_path = run_dir / "wrist_ffmpeg.log"
        wrist_frame_count = recording_state.get("wrist_frame_index", 0)
        if wrist_frame_count > 0:
            with wrist_log_path.open("wb") as log_file:
                subprocess.Popen(
                    [
                        "ffmpeg", "-y", "-framerate", str(args.record_fps),
                        "-i", str(run_dir / "wrist_frames" / "frame_%06d.png"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                        str(wrist_video_path),
                    ],
                    stdout=log_file, stderr=subprocess.STDOUT,
                )
        manifest["wrist_frame_count"] = wrist_frame_count
        scene_video_path = run_dir / "scene_video.mp4"
        scene_frame_count = recording_state.get("scene_frame_index", 0)
        if SCENE_CAMERA_ENABLED and scene_frame_count > 0:
            scene_log_path = run_dir / "scene_ffmpeg.log"
            with scene_log_path.open("wb") as log_file:
                subprocess.Popen(
                    [
                        "ffmpeg", "-y", "-framerate", str(args.record_fps),
                        "-i", str(run_dir / "scene_frames" / "frame_%06d.png"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                        str(scene_video_path),
                    ],
                    stdout=log_file, stderr=subprocess.STDOUT,
                )
            manifest["scene_frame_count"] = scene_frame_count
        manifest_path.write_text(json.dumps(manifest, indent=2))
        print(f"[ROBOT129 ROS WEBRTC] RECORDING_STOP run_id={recording_state['run_id']} "
              f"frames={recording_state['frame_index']} -> {video_path} "
              f"wrist_frames={wrist_frame_count} -> {wrist_video_path} "
              f"scene_frames={scene_frame_count} -> {scene_video_path if SCENE_CAMERA_ENABLED else 'N/A'}", flush=True)

    def handle_recording(request, response):
        if request.data and not recording_state["active"]:
            start_recording()
            response.success = True
            response.message = f"recording started run_id={recording_state['run_id']}"
        elif not request.data and recording_state["active"]:
            stop_recording()
            response.success = True
            response.message = f"recording stopped run_id={recording_state['run_id']}"
        else:
            response.success = True
            response.message = "no-op: recording already in requested state"
        return response

    # Reverse of --scene-manifest: freeze the CURRENT live pose of every clutter object
    # (including anything you just dragged in the WebRTC viewport with Shift+drag) back
    # into a manifest JSON, so a drag-to-arrange session becomes something you can
    # actually pass to the next --scene-manifest run instead of evaporating when this
    # process exits. Requested 2026-09-21 alongside the physics="dynamic" option above.
    # Trigger has no request fields, so the output path is a parameter, following the
    # same pattern reset_scene's cube_xy/cube_yaw params already use in this file --
    # set it with `ros2 param set` before calling, or just take the default.
    node.declare_parameter(
        "capture.output_path", str(root / "research" / "configs" / "scenes" / "captured_manifest.json"),
    )

    def handle_capture_scene_manifest(request, response):
        if not clutter_objects:
            response.success = False
            response.message = "no clutter objects to capture -- this process wasn't started with --scene-manifest"
            return response
        out_path = Path(node.get_parameter("capture.output_path").value)
        objects_out = []
        for rec in clutter_objects:
            entry = {"id": rec["id"], "kind": rec["kind"], "physics": rec["physics"]}
            if rec["kind"] in ("primitive_box", "primitive_cylinder", "primitive_cone", "primitive_sphere"):
                # Safe to read a true live pose: see the spawn_clutter_from_manifest
                # docstring -- only usd_asset's referenced/instanced prims crash Isaac
                # when their RigidObject handle is touched a second time.
                rigid_obj = rec["rigid_obj"]
                # .data is timestamp-cached and only refreshes after update(); advance()
                # never updates clutter objects, so without this it returns a stale pose.
                rigid_obj.update(sim.get_physics_dt())
                pos = rigid_obj.data.root_pos_w.torch[0].detach().cpu().numpy()
                qx, qy, qz, qw = (float(v) for v in rigid_obj.data.root_quat_w.torch[0].detach().cpu().numpy())
                entry["xy_m"] = [float(pos[0]), float(pos[1])]
                entry["z_m"] = float(pos[2])
                entry["yaw_rad"] = quat_wxyz_to_yaw(qw, qx, qy, qz)
                entry["color_rgb"] = rec["color_rgb"]
                if rec["kind"] == "primitive_box":
                    entry["size_m"] = rec["size_m"]
                elif rec["kind"] == "primitive_sphere":
                    entry["radius_m"] = rec["radius_m"]
                else:
                    entry["radius_m"] = rec["radius_m"]
                    entry["height_m"] = rec["height_m"]
            elif rec["kind"] == "usd_asset":
                # No live re-read (see docstring): re-emits the pose it was spawned at.
                # Only loses accuracy if this object also had physics="dynamic" and moved.
                entry["xy_m"] = rec["spawn_xy_m"]
                entry["z_m"] = rec["spawn_z_m"]
                entry["yaw_rad"] = rec["spawn_yaw_rad"]
                entry["usd_path"] = rec["usd_path"]
                entry["scale"] = rec["scale"]
            objects_out.append(entry)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps({
            "schema_version": "scene_manifest_v1",
            "_provenance": f"captured live from a running --scene-manifest session via "
                            f"/robot129_sim/capture_scene_manifest at sim_time={frame_state['sim_time']:.2f}s "
                            f"-- positions reflect whatever was last dragged/settled, not the original manifest.",
            "objects": objects_out,
        }, indent=2))
        response.success = True
        response.message = f"captured {len(objects_out)} object(s) -> {out_path}"
        print(f"[ROBOT129 ROS WEBRTC] CAPTURE_SCENE_MANIFEST {response.message}", flush=True)
        return response

    def handle_insert_bar(_request, response):
        if not INSERT_BAR_MODE or stick is None:
            response.success = False
            response.message = "launch with --scene dynamic_stick --stick insert_bar"
            return response
        bar_state["inserted"] = True
        bar_state["inserted_sim_time_s"] = float(frame_state["sim_time"])
        response.success = True
        response.message = json.dumps({"sim_time_s": bar_state["inserted_sim_time_s"], "center_m": BAR_CENTER_XYZ, "size_m": BAR_SIZE_XYZ})
        print(f"[ROBOT129 ROS WEBRTC] INSERT_BAR {response.message}", flush=True)
        return response

    reset_service = node.create_service(Trigger, "/robot129_sim/reset_scene", handle_reset_scene)
    insert_bar_service = node.create_service(Trigger, "/robot129_sim/insert_bar_obstacle", handle_insert_bar)
    recording_service = node.create_service(SetBool, "/robot129_sim/recording", handle_recording)
    capture_manifest_service = node.create_service(
        Trigger, "/robot129_sim/capture_scene_manifest", handle_capture_scene_manifest
    )
    publish_scene_state()

    print("[ROBOT129 ROS WEBRTC] READY", flush=True)
    print("[ROBOT129 ROS WEBRTC] domain=129 namespace=/robot129_sim hardware_drivers=0", flush=True)
    print(f"[ROBOT129 ROS WEBRTC] scene={args.scene}", flush=True)
    print("[ROBOT129 ROS WEBRTC] command=/robot129_sim/arm_controller/joint_trajectory", flush=True)
    print("[ROBOT129 ROS WEBRTC] command=/robot129_sim/gripper_controller/joint_trajectory", flush=True)
    print("[ROBOT129 ROS WEBRTC] feedback=/robot129_sim/joint_states", flush=True)
    print(f"[ROBOT129 ROS WEBRTC] wrist_rgb={RGB_TOPIC}", flush=True)
    print(f"[ROBOT129 ROS WEBRTC] wrist_depth={DEPTH_TOPIC}", flush=True)
    print(f"[ROBOT129 ROS WEBRTC] wrist_info={CAMERA_INFO_TOPIC}", flush=True)
    print(f"[ROBOT129 ROS WEBRTC] wrist_camera_prim={WRIST_CAMERA_PRIM}", flush=True)
    if SCENE_CAMERA_ENABLED:
        print(f"[ROBOT129 ROS WEBRTC] scene_rgb={SCENE_RGB_TOPIC}", flush=True)
        print(f"[ROBOT129 ROS WEBRTC] scene_depth={SCENE_DEPTH_TOPIC}", flush=True)
        print(f"[ROBOT129 ROS WEBRTC] scene_info={SCENE_CAMERA_INFO_TOPIC}", flush=True)
        print(f"[ROBOT129 ROS WEBRTC] scene_camera_prim={SCENE_CAMERA_PRIM}", flush=True)
    if pick_place:
        print("[ROBOT129 ROS WEBRTC] object_pose=/robot129_sim/objects/target_cube/pose", flush=True)
        print("[ROBOT129 ROS WEBRTC] contacts=/robot129_sim/contacts/link7,/robot129_sim/contacts/link8", flush=True)
    if dynamic_stick_mode:
        print(f"[ROBOT129 ROS WEBRTC] stick_tf={STICK_FRAME}", flush=True)
        print(f"[ROBOT129 ROS WEBRTC] contacts={','.join('/robot129_sim/contacts/' + link for link in STICK_MONITORED_LINKS)}", flush=True)
    print("[ROBOT129 ROS WEBRTC] services=/robot129_sim/reset_scene,/robot129_sim/recording", flush=True)

    frame = 0
    while keep_running and app.is_running():
        # Update sim_time before spin_once so a trajectory accepted inside this frame's
        # callback records the correct "started" timestamp, not the previous frame's.
        frame_state["sim_time"] = frame / 120.0
        sim_time = frame_state["sim_time"]
        executor.spin_once(timeout_sec=0.0)
        for key in ("combined", "arm", "gripper"):
            plan = plans[key]
            if plan is None:
                continue
            values, done = sample_plan(plan, sim_time)
            target[plan["indices"]] = values
            if done:
                plans[key] = None
                print(f"[ROBOT129 ROS WEBRTC] COMPLETE {key}", flush=True)
                publish_event("COMPLETE", key)
        target[7] = -target[6]
        robot.set_joint_position_target_index(
            target=torch.tensor(target, dtype=torch.float32, device=sim.device).unsqueeze(0)
        )
        advance()
        frame += 1
        if frame % 4 == 0:
            stamp = node.get_clock().now().to_msg()
            state = JointState()
            state.header.stamp = stamp
            state.name = JOINT_NAMES
            state.position = robot.data.joint_pos.torch[0].detach().cpu().tolist()
            state.velocity = robot.data.joint_vel.torch[0].detach().cpu().tolist()
            state_pub.publish(state)
            piper_fb = JointState()
            piper_fb.header.stamp = stamp
            piper_fb.name = ARM_NAMES + ["gripper"]
            piper_fb.position = list(state.position[:6]) + [2.0 * state.position[6]]
            piper_feedback_pub.publish(piper_fb)
            counters["published_states"] += 1

            command_state = JointState()
            command_state.header.stamp = stamp
            command_state.name = JOINT_NAMES
            command_state.position = target.tolist()
            command_pub.publish(command_state)

            if pick_place:
                publish_object_and_contacts(stamp)
            if dynamic_stick_mode:
                publish_stick_and_contacts(stamp)

            if frame % camera_publish_interval == 0:
                try:
                    rgb, depth, intrinsics, position, quaternion = publish_wrist_camera(stamp)
                    counters["published_rgbd"] = counters.get("published_rgbd", 0) + 1
                    if frame % 120 == 0:
                        save_latest_wrist(rgb, depth, intrinsics, position, quaternion)
                except Exception as exc:
                    counters["camera_errors"] = counters.get("camera_errors", 0) + 1
                    if counters["camera_errors"] <= 3:
                        print(f"[ROBOT129 ROS WEBRTC] CAMERA_ERROR {type(exc).__name__}: {exc}", flush=True)

                if SCENE_CAMERA_ENABLED:
                    try:
                        s_rgb, s_depth, s_intrinsics, s_position, s_quaternion = publish_scene_camera(stamp)
                        counters["published_scene_rgbd"] = counters.get("published_scene_rgbd", 0) + 1
                        if frame % 120 == 0:
                            save_latest_scene_camera(s_rgb, s_depth, s_intrinsics, s_position, s_quaternion)
                    except Exception as exc:
                        counters["scene_camera_errors"] = counters.get("scene_camera_errors", 0) + 1
                        if counters["scene_camera_errors"] <= 3:
                            print(f"[ROBOT129 ROS WEBRTC] SCENE_CAMERA_ERROR {type(exc).__name__}: {exc}", flush=True)

        if frame % 8 == 0:
            # Global view (dashboard window 2): its own, coarser throttle -- half the
            # wrist/scene RGB-D cameras' rate, RGB-only, independent of the frame % 4
            # block above (deliberately not nested inside it: this camera has nothing to
            # do with joint-state/contact publishing cadence).
            try:
                publish_overview_camera(node.get_clock().now().to_msg())
                counters["published_overview_rgb"] = counters.get("published_overview_rgb", 0) + 1
            except Exception as exc:
                counters["overview_camera_errors"] = counters.get("overview_camera_errors", 0) + 1
                if counters["overview_camera_errors"] <= 3:
                    print(f"[ROBOT129 ROS WEBRTC] OVERVIEW_CAMERA_ERROR {type(exc).__name__}: {exc}", flush=True)

        if recording_state["active"] and overview_camera is not None:
            period = 1.0 / max(args.record_fps, 0.1)
            if sim_time - recording_state["last_saved"] >= period:
                recording_state["last_saved"] = sim_time
                try:
                    from PIL import Image as PILImage

                    frame_rgb = overview_camera.data.output["rgb"].torch[0, ..., :3].detach().cpu().numpy().astype(np.uint8)
                    frame_path = recording_state["dir"] / "frames" / f"frame_{recording_state['frame_index']:06d}.png"
                    PILImage.fromarray(frame_rgb).save(frame_path)
                    recording_state["frame_index"] += 1
                except Exception as exc:
                    print(f"[ROBOT129 ROS WEBRTC] RECORD_FRAME_ERROR {type(exc).__name__}: {exc}", flush=True)

        if recording_state["active"] and wrist_camera is not None:
            # Same recording toggle, same run_dir, a second stream: the gripper's own
            # first-person view (wrist_camera), captured into wrist_frames/. Shares
            # record_fps with the third-person stream so the two videos stay comparable
            # frame-for-frame even though they're written by two independent throttles.
            period = 1.0 / max(args.record_fps, 0.1)
            if sim_time - recording_state["wrist_last_saved"] >= period:
                recording_state["wrist_last_saved"] = sim_time
                try:
                    from PIL import Image as PILImage

                    wrist_rgb = wrist_camera.data.output["rgb"].torch[0, ..., :3].detach().cpu().numpy().astype(np.uint8)
                    wrist_frame_path = recording_state["dir"] / "wrist_frames" / f"frame_{recording_state['wrist_frame_index']:06d}.png"
                    PILImage.fromarray(wrist_rgb).save(wrist_frame_path)
                    recording_state["wrist_frame_index"] += 1
                except Exception as exc:
                    print(f"[ROBOT129 ROS WEBRTC] WRIST_RECORD_FRAME_ERROR {type(exc).__name__}: {exc}", flush=True)

        if recording_state["active"] and SCENE_CAMERA_ENABLED and scene_camera is not None:
            # Third stream: the fixed back/pole view, captured into scene_frames/. Same
            # pattern and record_fps as the wrist stream above.
            period = 1.0 / max(args.record_fps, 0.1)
            if sim_time - recording_state["scene_last_saved"] >= period:
                recording_state["scene_last_saved"] = sim_time
                try:
                    from PIL import Image as PILImage

                    scene_rgb = scene_camera.data.output["rgb"].torch[0, ..., :3].detach().cpu().numpy().astype(np.uint8)
                    scene_frame_path = recording_state["dir"] / "scene_frames" / f"frame_{recording_state['scene_frame_index']:06d}.png"
                    PILImage.fromarray(scene_rgb).save(scene_frame_path)
                    recording_state["scene_frame_index"] += 1
                except Exception as exc:
                    print(f"[ROBOT129 ROS WEBRTC] SCENE_RECORD_FRAME_ERROR {type(exc).__name__}: {exc}", flush=True)

        time.sleep(1 / 120)

    if recording_state["active"]:
        stop_recording()

    state_dir = root / "out/ros_webrtc_robot129"
    (state_dir / "session.json").write_text(
        json.dumps(
            {
                "status": "STOPPED",
                "simulation_only": True,
                "ros_domain_id": 129,
                "namespace": "/robot129_sim",
                "scene": args.scene,
                "accepted_commands": counters["accepted"],
                "rejected_commands": counters["rejected"],
                "published_joint_states": counters["published_states"],
                "published_rgbd": counters.get("published_rgbd", 0),
                "camera_errors": counters.get("camera_errors", 0),
                "scene_revision": scene_revision["value"],
                "wrist_camera_prim": WRIST_CAMERA_PRIM,
                "rgb_topic": RGB_TOPIC,
                "depth_topic": DEPTH_TOPIC,
                "camera_info_topic": CAMERA_INFO_TOPIC,
                "scene_camera_enabled": SCENE_CAMERA_ENABLED,
                "scene_camera_prim": SCENE_CAMERA_PRIM if SCENE_CAMERA_ENABLED else None,
                "scene_rgb_topic": SCENE_RGB_TOPIC if SCENE_CAMERA_ENABLED else None,
                "scene_depth_topic": SCENE_DEPTH_TOPIC if SCENE_CAMERA_ENABLED else None,
                "scene_camera_info_topic": SCENE_CAMERA_INFO_TOPIC if SCENE_CAMERA_ENABLED else None,
                "published_scene_rgbd": counters.get("published_scene_rgbd", 0),
                "scene_camera_errors": counters.get("scene_camera_errors", 0),
                "hardware_drivers": 0,
            },
            indent=2,
        )
        + "\n"
    )
    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    try:
        result = main()
    finally:
        app.close()
    raise SystemExit(result)
