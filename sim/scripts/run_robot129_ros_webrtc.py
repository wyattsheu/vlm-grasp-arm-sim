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

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Run Robot 129 under isolated ROS 2 trajectory control.")
parser.add_argument("--bundle", type=Path, required=True)
parser.add_argument("--warmup-frames", type=int, default=90)
parser.add_argument(
    "--scene", choices=["marker", "pick_place", "pick_place_hammer"], default="marker",
    help="marker (default, unchanged original behaviour), pick_place (dynamic cube + "
         "contacts), or pick_place_hammer (S5 pilot object: same pick_place wiring, but "
         "the graspable body is an elongated handle-shaped box, with a second purely "
         "kinematic 'head' box riding along for a two-part shape a real camera can see).",
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
parser.add_argument("--record-fps", type=float, default=10.0)
parser.add_argument("--seed", type=int, default=0, help="initial value for the reset.seed ROS parameter")
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
# ros2_ws/src/robot129_description/config/nominal_frames.yaml's camera_mount_rpy_rad
# (baked into the imported USD this prim chain lives in) is [0,0,0] -- the sim camera
# was, until this constant existed, aligned exactly with the gripper's own reach axis
# (gripper_to_tcp is along local +Z), i.e. looking straight down the approach direction
# at HOME. Real Robot 129 photos supplied by the user 2026-09-21 show the physical
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


def capture_viewport_probe(root: Path, viewport, step_once) -> dict:
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
        red = (rgb[..., 0] > 130) & (rgb[..., 0] > rgb[..., 1] * 1.35) & (rgb[..., 0] > rgb[..., 2] * 1.35)
        valid = float(luma.mean()) > 5 and float(luma.std()) > 3 and float(red.mean()) > 0.0002
        report = {
            "status": "PASS" if valid else "FAIL",
            "resolution": [int(rgb.shape[1]), int(rgb.shape[0])],
            "mean_luminance": round(float(luma.mean()), 3),
            "std_luminance": round(float(luma.std()), 3),
            "red_target_fraction": round(float(red.mean()), 6),
            "image": str(image_path),
        }
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


def spawn_clutter_from_manifest(manifest_path: str) -> list:
    """--scene-manifest support (see the arg's help text and docs/dev_guide_
    paper_core_and_dashboard_plan.md §5): spawn each listed object as a static,
    kinematic prop -- a distractor clutter layer, not a new graspable target.
    Returns the list of spawned RigidObject handles (nothing currently reads
    them back; kept so a future manifest field, e.g. per-object pose reset on
    reset_scene, has something to hold onto without re-deriving prim paths).

    kind="primitive_box": a flat-shaded box, no external asset needed.
    kind="usd_asset": loads usd_path as-is (NVIDIA's Isaac asset CDN paths,
    e.g. Isaac/Props/YCB/Axis_Aligned/025_mug.usd, verified reachable
    2026-09-20 -- see the dev guide for the checked list). Isaac fetches and
    locally caches USD files referenced this way; the first spawn of a given
    asset may take longer while it downloads.
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

        if kind == "primitive_box":
            size = tuple(obj.get("size_m", [0.05, 0.05, 0.05]))
            color = tuple(obj.get("color_rgb", [0.5, 0.5, 0.5]))
            spawn_cfg = sim_utils.CuboidCfg(
                size=size,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, metallic=0.05),
            )
        elif kind == "usd_asset":
            usd_path = obj["usd_path"]
            scale = tuple(obj.get("scale", [1.0, 1.0, 1.0]))
            spawn_cfg = sim_utils.UsdFileCfg(
                usd_path=usd_path, scale=scale,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
            )
        else:
            print(f"[ROBOT129 ROS WEBRTC] SCENE_MANIFEST skipping {obj_id}: unknown kind={kind!r}", flush=True)
            continue

        rigid_obj = RigidObject(
            RigidObjectCfg(
                prim_path=prim_path, spawn=spawn_cfg,
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=(xy[0], xy[1], z), rot=yaw_to_quat_xyzw_wxyz(yaw),
                ),
            )
        )
        spawned.append(rigid_obj)
        print(f"[ROBOT129 ROS WEBRTC] SCENE_MANIFEST spawned {obj_id} kind={kind} at xy={xy}", flush=True)
    return spawned


def yaw_to_quat_xyzw_wxyz(yaw: float):
    """RigidObjectCfg.InitialStateCfg.rot wants (w, x, y, z), unlike the
    ROS-facing yaw_to_quat_xyzw() above which returns (x, y, z, w) -- kept as
    a separate small helper rather than a shared one to avoid silently
    reordering the ROS-facing convention everything else in this file uses."""
    half = yaw / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def main() -> int:
    if int(os.environ.get("ROS_DOMAIN_ID", "-1")) != 129:
        print("[ROBOT129 ROS WEBRTC] FAIL - ROS_DOMAIN_ID must be 129", flush=True)
        return 2

    root = args.bundle.resolve()
    pick_place = args.scene in ("pick_place", "pick_place_hammer")
    hammer_mode = args.scene == "pick_place_hammer"
    settings = carb.settings.get_settings()
    settings.set("/rtx/background/source/type", 2)
    settings.set("/rtx/background/source/color", (0.055, 0.065, 0.080))
    sim = SimulationContext(sim_utils.SimulationCfg(device=args.device, dt=1 / 120, render_interval=4))

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
    if pick_place:
        # Dynamic cube: real gravity from the start, realistic (not exaggerated) friction.
        # Unlike sim/scripts/verify_robot129_physics_grasp.py this spawns ON the floor and
        # is never made kinematic or gravity-disabled: "picked up" here means real PhysX
        # contact lift, not a floating pre-placed prop.
        cube_size_xyz = HAMMER_HANDLE_SIZE_M if hammer_mode else (CUBE_SIZE_M, CUBE_SIZE_M, CUBE_SIZE_M)
        cube_color = (0.55, 0.35, 0.12) if hammer_mode else (0.88, 0.08, 0.04)  # brown handle vs red cube
        cube = RigidObject(
            RigidObjectCfg(
                prim_path="/World/TargetCube",
                spawn=sim_utils.CuboidCfg(
                    size=cube_size_xyz,
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(
                        kinematic_enabled=False, disable_gravity=False, max_depenetration_velocity=0.5
                    ),
                    mass_props=sim_utils.MassPropertiesCfg(mass=CUBE_MASS_KG),
                    collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
                    physics_material=sim_utils.RigidBodyMaterialCfg(
                        static_friction=0.8, dynamic_friction=0.6, restitution=0.0,
                        friction_combine_mode="average", restitution_combine_mode="min",
                    ),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=cube_color, metallic=0.05),
                    activate_contact_sensors=True,
                ),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(CUBE_XY_DEFAULT[0], CUBE_XY_DEFAULT[1], CUBE_DROP_Z)),
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
            translation=(PLACE_XY_DEFAULT[0], PLACE_XY_DEFAULT[1], 0.001),
        )
        if args.scene_manifest:
            spawn_clutter_from_manifest(args.scene_manifest)
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
    else:
        # Original marker-only scene: unchanged from before this edit.
        marker = sim_utils.CuboidCfg(
            size=(0.035, 0.035, 0.035),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.88, 0.08, 0.04), metallic=0.05),
        )
        marker.func("/World/TargetMarker", marker, translation=(0.38, 0.0, 0.0175))

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
    wrist_camera = Camera(
        CameraCfg(
            prim_path=WRIST_CAMERA_PRIM,
            update_period=1.0 / 30.0,
            height=480,
            width=640,
            data_types=["rgb", "distance_to_image_plane"],
            update_latest_camera_pose=True,
            offset=CameraCfg.OffsetCfg(
                pos=(0.0, 0.0, 0.0),
                # Rotation about the camera's own local X ("right") axis -- pitches
                # the optical forward axis away from the gripper's reach direction.
                # (x, y, z, w) -- CameraCfg.OffsetCfg.rot's documented order (NOT w,x,y,z).
                rot=(
                    math.sin(math.radians(WRIST_CAMERA_PITCH_CORRECTION_DEG) / 2.0),
                    0.0, 0.0,
                    math.cos(math.radians(WRIST_CAMERA_PITCH_CORRECTION_DEG) / 2.0),
                ),
                convention="ros",
            ),
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=28.0,
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
            prim_path="/World/RecordingCamera", update_period=0, height=480, width=640,
            data_types=["rgb"], background_color=(0.055, 0.065, 0.080),
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=28, horizontal_aperture=20.955, clipping_range=(0.05, 5)
            ),
        )
    )

    sim.reset()
    if overview_camera is not None:
        overview_camera.set_world_poses_from_view(
            eyes=torch.tensor([[1.05, 0.95, 0.85]], device=sim.device),
            targets=torch.tensor([[0.30, -0.05, 0.05]], device=sim.device),
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
        set_live_camera(viewport)

    def advance():
        robot.write_data_to_sim()
        step_sim()
        dt = sim.get_physics_dt()
        robot.update(dt)
        wrist_camera.update(dt)
        if cube is not None:
            cube.update(dt)
        if left_contact is not None:
            left_contact.update(dt)
            right_contact.update(dt)
        if overview_camera is not None:
            overview_camera.update(dt)

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
        set_live_camera(viewport)
        for _ in range(15):
            advance()
        probe = capture_viewport_probe(root, viewport, advance)
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
    latest_camera_dir = root / "out/ros_webrtc_robot129/wrist_camera"
    latest_camera_dir.mkdir(parents=True, exist_ok=True)

    cube_pose_pub = None
    contact_pubs = None
    if pick_place:
        cube_pose_pub = node.create_publisher(PoseStamped, "/robot129_sim/objects/target_cube/pose", sensor_qos)
        contact_pubs = {
            "link7": node.create_publisher(WrenchStamped, "/robot129_sim/contacts/link7", sensor_qos),
            "link8": node.create_publisher(WrenchStamped, "/robot129_sim/contacts/link8", sensor_qos),
        }

    def publish_event(kind: str, key: str, **fields):
        payload = {"kind": kind, "channel": key, "stamp_sim_time": frame_state["sim_time"]}
        payload.update(fields)
        msg = String()
        msg.data = json.dumps(payload)
        events_pub.publish(msg)

    def publish_wrist_camera(stamp):
        rgb = wrist_camera.data.output["rgb"].torch[0, ..., :3].detach().cpu().numpy().astype(np.uint8)
        depth = wrist_camera.data.output["distance_to_image_plane"].torch[0].detach().cpu().numpy().astype(np.float32)
        depth = np.squeeze(depth)
        intrinsics = wrist_camera.data.intrinsic_matrices.torch[0].detach().cpu().numpy()
        position = wrist_camera.data.pos_w.torch[0].detach().cpu().numpy()
        quaternion = wrist_camera.data.quat_w_ros.torch[0].detach().cpu().numpy()

        rgb_msg = Image()
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = WRIST_FRAME
        rgb_msg.height, rgb_msg.width = rgb.shape[:2]
        rgb_msg.encoding = "rgb8"
        rgb_msg.is_bigendian = False
        rgb_msg.step = int(rgb.shape[1] * 3)
        rgb_msg.data = rgb.tobytes()
        rgb_pub.publish(rgb_msg)

        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = WRIST_FRAME
        depth_msg.height, depth_msg.width = depth.shape
        depth_msg.encoding = "32FC1"
        depth_msg.is_bigendian = False
        depth_msg.step = int(depth.shape[1] * 4)
        depth_msg.data = depth.tobytes()
        depth_pub.publish(depth_msg)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = WRIST_FRAME
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
        info_pub.publish(info)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = WRIST_FRAME
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        tf_pub.publish(TFMessage(transforms=[transform]))
        return rgb, depth, intrinsics, position, quaternion

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

    def save_latest_wrist(rgb, depth, intrinsics, position, quaternion):
        from PIL import Image as PILImage

        def atomic_text(name, payload):
            final = latest_camera_dir / name
            temporary = latest_camera_dir / (name + ".tmp")
            temporary.write_text(payload)
            os.replace(temporary, final)

        rgb_final = latest_camera_dir / "rgb.png"
        rgb_temporary = latest_camera_dir / "rgb.png.tmp"
        PILImage.fromarray(rgb).save(rgb_temporary, format="PNG")
        os.replace(rgb_temporary, rgb_final)

        depth_final = latest_camera_dir / "depth.npy"
        depth_temporary = latest_camera_dir / "depth.npy.tmp"
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
                    "frame_id": WRIST_FRAME,
                    "depth_units": "meter",
                    "depth_definition": "optical_axis_z",
                    "calibration": "ISAAC_WRIST_CAMERA_NOMINAL",
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
                    "camera_frame": WRIST_FRAME,
                    "camera_prim": WRIST_CAMERA_PRIM,
                    "attached_to": WRIST_OPTICAL_PARENT,
                    "physical_wrist_extrinsic_verified": False,
                },
                indent=2,
            ) + "\n",
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

    subscriptions = [
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
        "wrist_frame_index": 0, "dir": None, "last_saved": 0.0, "wrist_last_saved": 0.0,
    }
    frame_state = {"sim_time": 0.0, "count": 0}

    def publish_scene_state():
        payload = {
            "revision": scene_revision["value"],
            "scene": args.scene,
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
            pose = torch.tensor(
                [[cube_xy[0], cube_xy[1], CUBE_DROP_Z, qx, qy, qz, qw]], dtype=torch.float32, device=sim.device
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
        recording_state["dir"] = run_dir
        recording_state["frame_index"] = 0
        recording_state["wrist_frame_index"] = 0
        recording_state["active"] = True
        recording_state["last_saved"] = -1.0
        recording_state["wrist_last_saved"] = -1.0
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
        manifest_path.write_text(json.dumps(manifest, indent=2))
        print(f"[ROBOT129 ROS WEBRTC] RECORDING_STOP run_id={recording_state['run_id']} "
              f"frames={recording_state['frame_index']} -> {video_path} "
              f"wrist_frames={wrist_frame_count} -> {wrist_video_path}", flush=True)

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

    reset_service = node.create_service(Trigger, "/robot129_sim/reset_scene", handle_reset_scene)
    recording_service = node.create_service(SetBool, "/robot129_sim/recording", handle_recording)
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
    if pick_place:
        print("[ROBOT129 ROS WEBRTC] object_pose=/robot129_sim/objects/target_cube/pose", flush=True)
        print("[ROBOT129 ROS WEBRTC] contacts=/robot129_sim/contacts/link7,/robot129_sim/contacts/link8", flush=True)
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
            counters["published_states"] += 1

            command_state = JointState()
            command_state.header.stamp = stamp
            command_state.name = JOINT_NAMES
            command_state.position = target.tolist()
            command_pub.publish(command_state)

            if pick_place:
                publish_object_and_contacts(stamp)

            try:
                rgb, depth, intrinsics, position, quaternion = publish_wrist_camera(stamp)
                counters["published_rgbd"] = counters.get("published_rgbd", 0) + 1
                if frame % 120 == 0:
                    save_latest_wrist(rgb, depth, intrinsics, position, quaternion)
            except Exception as exc:
                counters["camera_errors"] = counters.get("camera_errors", 0) + 1
                if counters["camera_errors"] <= 3:
                    print(f"[ROBOT129 ROS WEBRTC] CAMERA_ERROR {type(exc).__name__}: {exc}", flush=True)

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
