"""Verify a dynamic Robot 129 two-finger grasp with PhysX contact forces."""

import argparse
import json
from pathlib import Path
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--bundle", type=Path, required=True)
parser.add_argument("--frames", type=int, default=300)
parser.add_argument("--realtime", action="store_true", help="pace rendered frames for a live WebRTC demo")
parser.add_argument("--preroll-seconds", type=float, default=0.0)
parser.add_argument("--hold-seconds", type=float, default=0.0)
parser.add_argument("--warmup-frames", type=int, default=90)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = not args.realtime
launcher = AppLauncher(args)
app = launcher.app

import carb
import numpy as np
import torch
from PIL import Image, ImageDraw
import omni.usd

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.sensors import Camera, CameraCfg, ContactSensor, ContactSensorCfg
from isaaclab.sim import SimulationContext


def smooth(a, b, t):
    t = max(0.0, min(1.0, t))
    t = 3 * t * t - 2 * t * t * t
    return a + (b - a) * t


def set_live_viewport_camera(viewport, eye, target) -> None:
    """Set the actual Perspective camera streamed by WebRTC."""
    from omni.kit.viewport.utility.camera_state import ViewportCameraState
    from pxr import Gf

    camera_path = viewport.get_active_camera() or "/OmniverseKit_Persp"
    camera_state = ViewportCameraState(camera_path, viewport)
    camera_state.set_position_world(Gf.Vec3d(*map(float, eye)), False)
    camera_state.set_target_world(Gf.Vec3d(*map(float, target)), True)
    print(
        f"[ROBOT129 WEBRTC] CAMERA - path={camera_path} "
        f"eye={tuple(round(float(v), 3) for v in camera_state.position_world)}",
        flush=True,
    )


def verify_live_viewport(root: Path, viewport, step_once, label: str = "viewport_probe", require_target: bool = True) -> bool:
    """Capture the exact LDR viewport streamed by WebRTC and reject blank frames."""
    from omni.kit.viewport.utility import capture_viewport_to_file

    state = root / "out/webrtc_robot129"
    state.mkdir(parents=True, exist_ok=True)
    image_path = state / f"{label}.png"
    report_path = state / f"{label}.json"
    image_path.unlink(missing_ok=True)
    report_path.unlink(missing_ok=True)

    if viewport is None:
        report = {"status": "FAIL", "reason": "active viewport is unavailable"}
        report_path.write_text(json.dumps(report, indent=2) + "\n")
        print("[ROBOT129 WEBRTC] VIEWPORT_PROBE FAIL - active viewport unavailable", flush=True)
        return False

    capture_viewport_to_file(viewport, file_path=str(image_path))
    for _ in range(120):
        step_once()
        if image_path.is_file() and image_path.stat().st_size > 1024:
            break

    try:
        with Image.open(image_path) as captured:
            rgb = np.asarray(captured.convert("RGB"), dtype=np.float32)
        luminance = 0.2126 * rgb[..., 0] + 0.7152 * rgb[..., 1] + 0.0722 * rgb[..., 2]
        mean_luma = float(luminance.mean())
        std_luma = float(luminance.std())
        p01, p99 = (float(value) for value in np.percentile(luminance, [1, 99]))
        non_black_fraction = float((luminance > 5.0).mean())
        red_target = (rgb[..., 0] > 130.0) & (rgb[..., 0] > rgb[..., 1] * 1.35) & (rgb[..., 0] > rgb[..., 2] * 1.35)
        red_target_fraction = float(red_target.mean())
        valid_frame = mean_luma > 5.0 and std_luma > 3.0 and (p99 - p01) > 10.0
        passed = valid_frame and (not require_target or red_target_fraction > 0.0002)
        report = {
            "status": "PASS" if passed else "FAIL",
            "image": str(image_path),
            "resolution": [int(rgb.shape[1]), int(rgb.shape[0])],
            "mean_luminance": round(mean_luma, 3),
            "std_luminance": round(std_luma, 3),
            "p01_luminance": round(p01, 3),
            "p99_luminance": round(p99, 3),
            "non_black_fraction": round(non_black_fraction, 6),
            "red_target_fraction": round(red_target_fraction, 6),
            "target_visibility_required": require_target,
            "active_camera": str(viewport.get_active_camera()),
        }
    except Exception as exc:
        passed = False
        report = {
            "status": "FAIL",
            "reason": f"viewport capture unavailable: {type(exc).__name__}: {exc}",
            "image": str(image_path),
        }

    report_path.write_text(json.dumps(report, indent=2) + "\n")
    metrics = " ".join(
        f"{key}={value}"
        for key, value in report.items()
        if key in {"mean_luminance", "std_luminance", "p01_luminance", "p99_luminance"}
    )
    print(f"[ROBOT129 WEBRTC] VIEWPORT_PROBE {report['status']} - {metrics}", flush=True)
    return passed


def phase(frame):
    if frame < 45:
        return "HOME", "Place dynamic cube at measured grasp pose"
    if frame < 105:
        return "GRASP", "Close fingers; then release cube to PhysX"
    if frame < 190:
        return "LIFT", "Lift using friction and two-finger contact"
    if frame < 225:
        return "WAYPOINT", "Hold payload without pose attachment"
    if frame < 270:
        return "RELEASE", "Open fingers and let gravity act"
    return "RETREAT", "Keep robot clear of released payload"


def main() -> int:
    root = args.bundle.resolve()
    run = root / "out/lesson_06/physics_grasp"
    frames_dir = run / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    settings = carb.settings.get_settings()
    settings.set("/rtx/background/source/type", 2)
    settings.set("/rtx/background/source/color", (0.055, 0.065, 0.080))
    sim = SimulationContext(sim_utils.SimulationCfg(device=args.device, dt=1 / 120, render_interval=2))

    def step_sim():
        # This server's known-good WebRTC path requires explicit Kit event-loop
        # pumping. It delivers rendered frames and processes remote mouse input.
        # Pump before the physics step so the following step refreshes tensor views.
        if args.realtime:
            app.update()
            app.update()
        sim.step()

    floor_cfg = sim_utils.CuboidCfg(
        size=(4.0, 4.0, 0.05),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(0.11, 0.12, 0.14), metallic=0.0, roughness=0.88
        ),
    )
    floor_cfg.func("/World/DeepGrayFloor", floor_cfg, translation=(0.0, 0.0, -0.025))
    light_cfg = sim_utils.DomeLightCfg(
        intensity=2800, color=(0.82, 0.84, 0.88), visible_in_primary_ray=False
    )
    light_cfg.func("/World/Light", light_cfg)
    robot = Articulation(ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(root / "sim/assets/robot129/robot129/robot129.usda"), activate_contact_sensors=True
        ),
        actuators={
            "arm": ImplicitActuatorCfg(joint_names_expr=["joint[1-6]"], effort_limit_sim=100, stiffness=800, damping=80),
            "gripper": ImplicitActuatorCfg(joint_names_expr=["joint[78]"], effort_limit_sim=10, stiffness=2000, damping=100),
        },
    ))
    cube = RigidObject(RigidObjectCfg(
        prim_path="/World/TargetCube",
        spawn=sim_utils.CuboidCfg(
            size=(0.035, 0.035, 0.035),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                kinematic_enabled=False, disable_gravity=True, max_depenetration_velocity=0.5
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.08),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=4.0, dynamic_friction=3.0, restitution=0.0,
                friction_combine_mode="max", restitution_combine_mode="min"
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.88, 0.08, 0.04), metallic=0.05),
            activate_contact_sensors=True,
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.38, 0.0, 0.45)),
    ))
    left = ContactSensor(ContactSensorCfg(
        prim_path="/World/Robot/link7", update_period=0, history_length=1,
        filter_prim_paths_expr=["/World/TargetCube"], force_threshold=0.01,
    ))
    right = ContactSensor(ContactSensorCfg(
        prim_path="/World/Robot/link8", update_period=0, history_length=1,
        filter_prim_paths_expr=["/World/TargetCube"], force_threshold=0.01,
    ))
    camera = None
    if not args.realtime:
        camera = Camera(CameraCfg(
            prim_path="/World/RecordingCamera", update_period=0, height=480, width=640,
            data_types=["rgb"], background_color=(0.055, 0.065, 0.080),
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=28, horizontal_aperture=20.955, clipping_range=(0.05, 5)
            ),
        ))
    sim.reset()
    # WebRTC streams the active Kit viewport; its camera is configured below
    # through ViewportCameraState so remote orbit/pan/zoom remains available.
    if camera is not None:
        camera.set_world_poses_from_view(
            eyes=torch.tensor([[1.15, 1.05, 0.90]], device=sim.device),
            targets=torch.tensor([[0.25, 0.0, 0.40]], device=sim.device),
        )
    if args.realtime:
        from omni.kit.viewport.utility import get_active_viewport
        viewport = get_active_viewport()
        active_camera = None if viewport is None else viewport.get_active_camera()
        print(f"[ROBOT129 WEBRTC] VIEWPORT - camera={active_camera} sensor_render_product=disabled", flush=True)
        if viewport is not None:
            set_live_viewport_camera(viewport, eye=(0.92, 0.78, 0.72), target=(0.27, 0.0, 0.36))
    home_arm = torch.tensor([0.0, 1.20, -1.25, 0.0, 0.15, 0.0], device=sim.device)
    lift_arm = torch.tensor([0.0, 1.35, -1.75, 0.0, 0.15, 0.0], device=sim.device)
    open_fingers = torch.tensor([0.035, -0.035], device=sim.device)
    hold_fingers = torch.tensor([0.0, 0.0], device=sim.device)
    max_force = [0.0, 0.0]
    trace = []
    gravity_enable_frame = 105
    gravity_attr = omni.usd.get_context().get_stage().GetPrimAtPath("/World/TargetCube").GetAttribute(
        "physxRigidBody:disableGravity"
    )

    if args.realtime:
        robot.set_joint_position_target_index(target=torch.cat((home_arm, open_fingers)).unsqueeze(0))
        print(f"[ROBOT129 WEBRTC] WARMING_UP - {args.warmup_frames} rendered frames", flush=True)
        # Force shader compilation, the camera render product, Fabric/USD synchronization,
        # and WebRTC geometry upload before telling the client that input is ready.
        for _ in range(max(0, args.warmup_frames)):
            robot.write_data_to_sim()
            step_sim()
            dt = sim.get_physics_dt()
            robot.update(dt); cube.update(dt); left.update(dt); right.update(dt)
            if camera is not None:
                camera.update(dt)

        def step_live_viewport():
            robot.write_data_to_sim()
            step_sim()
            dt = sim.get_physics_dt()
            robot.update(dt); cube.update(dt); left.update(dt); right.update(dt)

        # Kit can reset the implicit Perspective camera while its viewport and
        # livestream target finish initializing. Re-apply the teaching view after
        # warmup, then let RTX accumulate several frames before validation.
        set_live_viewport_camera(viewport, eye=(0.92, 0.78, 0.72), target=(0.27, 0.0, 0.36))
        for _ in range(15):
            step_live_viewport()

        print("[ROBOT129 WEBRTC] VERIFYING_VIEWPORT - capturing streamed LDR frame", flush=True)
        if not verify_live_viewport(root, viewport, step_live_viewport):
            return 2
        print("[ROBOT129 WEBRTC] READY - warmup complete; interactive preroll", flush=True)
        for _ in range(max(0, round(args.preroll_seconds * 30))):
            robot.write_data_to_sim()
            step_sim()
            dt = sim.get_physics_dt()
            robot.update(dt); cube.update(dt); left.update(dt); right.update(dt)
            if camera is not None:
                camera.update(dt)
            time.sleep(1 / 30)

    for frame in range(args.frames):
        name, description = phase(frame)
        if name == "HOME":
            arm, fingers = home_arm, open_fingers
        elif name == "GRASP":
            arm = home_arm
            fingers = smooth(open_fingers, hold_fingers, (frame - 45) / 50)
        elif name in {"LIFT", "WAYPOINT"}:
            arm = smooth(home_arm, lift_arm, (frame - 105) / 75)
            fingers = hold_fingers
        elif name == "RELEASE":
            arm = lift_arm
            fingers = smooth(hold_fingers, open_fingers, (frame - 225) / 35)
        else:
            arm, fingers = lift_arm, open_fingers
        target = torch.cat((arm, fingers)).unsqueeze(0)
        robot.set_joint_position_target_index(target=target)
        robot.write_data_to_sim()
        if frame == gravity_enable_frame:
            gravity_attr.Set(False)
        step_sim()
        dt = sim.get_physics_dt()
        robot.update(dt)
        cube.update(dt)
        left.update(dt)
        right.update(dt)
        if camera is not None:
            camera.update(dt)
        forces = []
        for idx, sensor in enumerate((left, right)):
            raw = sensor.data.normal_force_matrix_w
            value = 0.0 if raw is None else float(torch.linalg.vector_norm(raw.torch).item())
            max_force[idx] = max(max_force[idx], value)
            forces.append(value)
        cube_pos = cube.data.root_pos_w.torch[0].detach().cpu().tolist()
        trace.append({"frame": frame, "phase": name, "cube_position": cube_pos, "left_N": forces[0], "right_N": forces[1]})

        if camera is not None:
            rgb = camera.data.output["rgb"].torch[0, ..., :3].detach().cpu().numpy()
            image = Image.fromarray(rgb)
            draw = ImageDraw.Draw(image, "RGBA")
            colors = {"GRASP": (220, 35, 35, 235), "WAYPOINT": (35, 90, 230, 235), "RELEASE": (25, 160, 65, 235)}
            color = colors.get(name, (30, 30, 30, 215))
            draw.rectangle((0, 0, 640, 74), fill=(255, 255, 255, 225))
            draw.rectangle((12, 10, 142, 48), fill=color)
            draw.text((24, 20), name, fill=(255, 255, 255, 255))
            draw.text((158, 12), "ROBOT 129 | PHYSX DYNAMIC GRASP", fill=(0, 0, 0, 255))
            draw.text((158, 32), description, fill=(45, 45, 45, 255))
            draw.text((12, 55), f"contact L={forces[0]:.2f} N  R={forces[1]:.2f} N   cube z={cube_pos[2]:.3f} m", fill=(0, 0, 0, 255))
            image.save(frames_dir / f"frame_{frame:04d}.png")
        if args.realtime:
            time.sleep(1 / 30)

    if args.realtime:
        print("[ROBOT129 WEBRTC] VERIFYING_SUSTAINED_VIEWPORT - capture after demo", flush=True)
        if not verify_live_viewport(root, viewport, step_live_viewport, label="viewport_probe_after_demo", require_target=False):
            return 3
        print("[ROBOT129 WEBRTC] COMPLETE - holding final scene", flush=True)
        remaining = None if args.hold_seconds < 0 else max(0, round(args.hold_seconds * 30))
        while app.is_running() and (remaining is None or remaining > 0):
            robot.write_data_to_sim(); step_sim()
            dt = sim.get_physics_dt()
            robot.update(dt); cube.update(dt); left.update(dt); right.update(dt)
            if camera is not None:
                camera.update(dt)
            if remaining is not None:
                remaining -= 1
            time.sleep(1 / 30)

    z_at_release = trace[gravity_enable_frame]["cube_position"][2]
    z_before_lift = trace[104]["cube_position"][2]
    z_peak = max(item["cube_position"][2] for item in trace[105:225])
    z_final = trace[-1]["cube_position"][2]
    checks = {
        "left_contact_over_0_1N": max_force[0] > 0.1,
        "right_contact_over_0_1N": max_force[1] > 0.1,
        "payload_lift_over_0_03m": z_peak - z_before_lift > 0.03,
        "payload_falls_after_release": z_peak - z_final > 0.03,
        "finite_positions": bool(np.isfinite([item["cube_position"] for item in trace]).all()),
    }
    report = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "simulation_only": True,
        "kinematic_attachment": False,
        "cube_dynamic_from_start": True,
        "gravity_enabled_after_frame": gravity_enable_frame,
        "cube_mass_kg": 0.08,
        "max_contact_force_N": {"link7": max_force[0], "link8": max_force[1]},
        "z_m": {"physics_release": z_at_release, "before_lift": z_before_lift, "peak": z_peak, "final": z_final},
        "checks": checks,
        "trace": trace,
    }
    (run / "physics_grasp_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "trace"}, indent=2))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    try:
        result = main()
    finally:
        app.close()
    raise SystemExit(result)
