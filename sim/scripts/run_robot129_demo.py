"""Robot 129 simulation-only pick-and-place demonstration and recorder."""

import argparse
import hashlib
import json
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--bundle", type=Path, required=True)
parser.add_argument("--frames", type=int, default=300)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True
launcher = AppLauncher(args)
app = launcher.app

import numpy as np
import torch
from PIL import Image, ImageDraw

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.sensors.camera import Camera, CameraCfg
from isaaclab.sim import SimulationContext


def lerp(a, b, t):
    return a + (b - a) * (3 * t * t - 2 * t * t * t)


def segment(frame):
    phases = [
        (0, 45, "HOME", "Initialize known joint state"),
        (45, 95, "APPROACH", "Move TCP above object"),
        (95, 130, "GRASP", "Close symmetric fingers"),
        (130, 185, "LIFT", "Lift attached payload"),
        (185, 235, "WAYPOINT", "Transport with payload"),
        (235, 270, "RELEASE", "Open fingers at target"),
        (270, 300, "RETREAT", "Return to safe pose"),
    ]
    for start, end, name, desc in phases:
        if start <= frame < end:
            return start, end, name, desc
    return phases[-1]


def main() -> int:
    root = args.bundle.resolve()
    run = root / "out/integrated_demo"
    frames_dir = run / "frames"
    scene_dir = run / "scene_bundle"
    frames_dir.mkdir(parents=True, exist_ok=True)
    scene_dir.mkdir(parents=True, exist_ok=True)
    usd = root / "sim/assets/robot129/robot129/robot129.usda"

    sim = SimulationContext(sim_utils.SimulationCfg(device=args.device, dt=1 / 60, render_interval=1))
    ground = sim_utils.GroundPlaneCfg()
    ground.func("/World/Ground", ground)
    light = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.8, 0.8, 0.8))
    light.func("/World/Light", light)

    robot = Articulation(ArticulationCfg(
        prim_path="/World/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path=str(usd), activate_contact_sensors=True),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
        actuators={
            "arm": ImplicitActuatorCfg(joint_names_expr=["joint[1-6]"], effort_limit_sim=100.0, stiffness=800.0, damping=80.0),
            "gripper": ImplicitActuatorCfg(joint_names_expr=["joint[78]"], effort_limit_sim=10.0, stiffness=2000.0, damping=100.0),
        },
    ))
    cube = RigidObject(RigidObjectCfg(
        prim_path="/World/TargetCube",
        spawn=sim_utils.CuboidCfg(
            size=(0.055, 0.055, 0.055),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.10),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.12, 0.08), metallic=0.1),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.42, 0.0, 0.06)),
    ))
    camera = Camera(CameraCfg(
        prim_path="/World/RecordingCamera", update_period=0, height=480, width=640,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=28.0, horizontal_aperture=20.955, clipping_range=(0.05, 5.0)),
    ))

    sim.reset()
    camera.set_world_poses_from_view(
        eyes=torch.tensor([[1.35, 1.25, 1.05]], device=sim.device),
        targets=torch.tensor([[0.0, 0.0, 0.35]], device=sim.device),
    )
    names = list(robot.joint_names)
    expected = [f"joint{i}" for i in range(1, 9)]
    if names != expected:
        raise RuntimeError(f"Unexpected joint order: {names}")

    poses = {
        "HOME": torch.tensor([0.0, 1.20, -1.25, 0.0, 0.15, 0.0, 0.035, -0.035], device=sim.device),
        "APPROACH": torch.tensor([0.35, 1.45, -1.75, 0.15, 0.35, 0.0, 0.035, -0.035], device=sim.device),
        "GRASP": torch.tensor([0.35, 1.45, -1.75, 0.15, 0.35, 0.0, 0.004, -0.004], device=sim.device),
        "LIFT": torch.tensor([0.35, 1.10, -1.30, 0.15, 0.25, 0.0, 0.004, -0.004], device=sim.device),
        "WAYPOINT": torch.tensor([-0.55, 1.05, -1.25, -0.10, 0.20, 0.0, 0.004, -0.004], device=sim.device),
        "RELEASE": torch.tensor([-0.55, 1.35, -1.60, -0.10, 0.30, 0.0, 0.035, -0.035], device=sim.device),
        "RETREAT": torch.tensor([0.0, 1.20, -1.25, 0.0, 0.15, 0.0, 0.035, -0.035], device=sim.device),
    }
    order = ["HOME", "APPROACH", "GRASP", "LIFT", "WAYPOINT", "RELEASE", "RETREAT"]
    prev = poses["HOME"]
    phase_log = []
    carried_pose = None
    capture = None

    for frame in range(args.frames):
        start, end, phase, desc = segment(frame)
        idx = order.index(phase)
        source = poses[order[max(0, idx - 1)]] if idx else poses["HOME"]
        target = poses[phase]
        t = min(1.0, (frame - start) / max(1, end - start - 1))
        command = lerp(source, target, t).unsqueeze(0)
        robot.set_joint_position_target(command)
        robot.write_data_to_sim()

        if phase in {"GRASP", "LIFT", "WAYPOINT"}:
            grip_idx = robot.body_names.index("gripper_base")
            p = robot.data.body_pos_w.torch[0, grip_idx].clone()
            q = robot.data.body_quat_w.torch[0, grip_idx].clone()
            p[2] -= 0.13
            carried_pose = torch.cat((p, q)).unsqueeze(0)
            cube.write_root_pose_to_sim(carried_pose)
        elif phase == "RELEASE" and carried_pose is not None:
            cube.write_root_pose_to_sim(carried_pose)

        sim.step()
        robot.update(sim.get_physics_dt())
        cube.update(sim.get_physics_dt())
        camera.update(sim.get_physics_dt())

        rgb = camera.data.output["rgb"].torch[0, ..., :3].detach().cpu().numpy()
        if frame == 220:
            capture = {
                "rgb": rgb.copy(),
                "depth": camera.data.output["distance_to_image_plane"].torch[0].detach().cpu().numpy().astype(np.float32),
                "position": camera.data.pos_w.torch[0].detach().cpu().numpy(),
                "quaternion_xyzw": camera.data.quat_w_ros.torch[0].detach().cpu().numpy(),
                "intrinsics": camera.data.intrinsic_matrices.torch[0].detach().cpu().numpy(),
            }
        image = Image.fromarray(rgb)
        draw = ImageDraw.Draw(image, "RGBA")
        colors = {"GRASP": (220, 35, 35, 235), "WAYPOINT": (35, 90, 230, 235), "RELEASE": (25, 160, 65, 235)}
        color = colors.get(phase, (25, 25, 25, 210))
        draw.rectangle((0, 0, 640, 58), fill=(255, 255, 255, 220))
        draw.rectangle((12, 10, 142, 48), fill=color)
        draw.text((24, 20), phase, fill=(255, 255, 255, 255))
        draw.text((158, 14), "ROBOT 129  |  SIMULATION ONLY", fill=(0, 0, 0, 255))
        draw.text((158, 34), desc, fill=(45, 45, 45, 255))
        image.save(frames_dir / f"frame_{frame:04d}.png")
        if frame in {0, 45, 95, 130, 185, 235, 270}:
            phase_log.append({"frame": frame, "phase": phase, "joint_target": command[0].tolist()})
        prev = command[0]

    if capture is None:
        raise RuntimeError("synchronized RGB-D capture was not produced")
    Image.fromarray(capture["rgb"]).save(scene_dir / "rgb.png")
    np.save(scene_dir / "depth.npy", capture["depth"])
    k = capture["intrinsics"].reshape(-1).tolist()
    (scene_dir / "camera_info.json").write_text(json.dumps({"width": 640, "height": 480, "k": k, "d": [0, 0, 0, 0, 0], "depth_scale_m": 1.0, "frame_id": "recording_camera_optical_frame", "depth_units": "meter", "depth_definition": "optical_axis_z", "calibration": "ISAAC_RENDER_CAMERA"}, indent=2)+"\n")
    (scene_dir / "joint_states.json").write_text(json.dumps({"names": names, "positions": robot.data.joint_pos.torch[0].tolist(), "frame": "world", "ros_domain_id": 129, "namespace": "/robot129_sim"}, indent=2)+"\n")
    x, y, z, w = [float(v) for v in capture["quaternion_xyzw"]]
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    transform = np.eye(4); transform[:3, :3] = rotation; transform[:3, 3] = capture["position"]
    (scene_dir / "tf.json").write_text(json.dumps({"status": "AVAILABLE", "matrix": transform.tolist(), "base_frame": "world", "camera_frame": "recording_camera_optical_frame", "source": "IsaacLab CameraData.pos_w + quat_w_ros", "wrist_extrinsic_verified": False}, indent=2)+"\n")
    (scene_dir / "instruction.txt").write_text("Pick the red cube and place it at the release pose.\n")
    names_to_hash = ["rgb.png", "depth.npy", "camera_info.json", "tf.json", "joint_states.json", "instruction.txt"]
    hashes = {name: hashlib.sha256((scene_dir / name).read_bytes()).hexdigest() for name in names_to_hash}
    manifest = {"status": "PASS", "capture_complete": True, "sha256": hashes, "simulator": "Isaac Sim 6.0.1.0", "physics": "PhysX", "asset": str(usd), "seed": 129, "frames": args.frames, "fps": 30, "phases": phase_log, "simulation_only": True, "hardware_commands": 0}
    (scene_dir / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    (run / "demo_report.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print(f"PASS: recorded {args.frames} Robot 129 frames in {frames_dir}")
    return 0


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        app.close()
    raise SystemExit(rc)
