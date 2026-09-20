"""Search a small deterministic pose grid for two-finger PhysX contact."""

import argparse
import json
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--bundle", type=Path, required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
launcher = AppLauncher(args)
app = launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.sim import SimulationContext


def main() -> int:
    root = args.bundle.resolve()
    sim = SimulationContext(sim_utils.SimulationCfg(device=args.device, dt=1 / 120, render_interval=8))
    sim_utils.GroundPlaneCfg().func("/World/Ground", sim_utils.GroundPlaneCfg())
    robot = Articulation(
        ArticulationCfg(
            prim_path="/World/Robot",
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(root / "sim/assets/robot129/robot129/robot129.usda"),
                activate_contact_sensors=True,
            ),
            actuators={
                "arm": ImplicitActuatorCfg(joint_names_expr=["joint[1-6]"], effort_limit_sim=100, stiffness=800, damping=80),
                "gripper": ImplicitActuatorCfg(joint_names_expr=["joint[78]"], effort_limit_sim=10, stiffness=2000, damping=100),
            },
        )
    )
    cube = RigidObject(
        RigidObjectCfg(
            prim_path="/World/TargetCube",
            spawn=sim_utils.CuboidCfg(
                size=(0.035, 0.035, 0.035),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                mass_props=sim_utils.MassPropertiesCfg(mass=0.08),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.2, dynamic_friction=1.0),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.1, 0.05)),
                activate_contact_sensors=True,
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.40, 0.0, 0.45)),
        )
    )
    left = ContactSensor(ContactSensorCfg(
        prim_path="/World/Robot/link7", update_period=0.0, history_length=1,
        filter_prim_paths_expr=["/World/TargetCube"], force_threshold=0.01,
    ))
    right = ContactSensor(ContactSensorCfg(
        prim_path="/World/Robot/link8", update_period=0.0, history_length=1,
        filter_prim_paths_expr=["/World/TargetCube"], force_threshold=0.01,
    ))
    sim.reset()
    arm = torch.tensor([[0.0, 1.20, -1.25, 0.0, 0.15, 0.0]], device=sim.device)

    def step(target, count):
        maxima = [0.0, 0.0]
        for _ in range(count):
            robot.set_joint_position_target_index(target=target)
            robot.write_data_to_sim()
            sim.step(render=False)
            robot.update(sim.get_physics_dt())
            cube.update(sim.get_physics_dt())
            left.update(sim.get_physics_dt())
            right.update(sim.get_physics_dt())
            for i, sensor in enumerate((left, right)):
                force = sensor.data.normal_force_matrix_w
                if force is not None:
                    maxima[i] = max(maxima[i], float(torch.linalg.vector_norm(force.torch).item()))
        return maxima

    opened = torch.cat((arm, torch.tensor([[0.035, -0.035]], device=sim.device)), dim=1)
    closed = torch.cat((arm, torch.tensor([[0.003, -0.003]], device=sim.device)), dim=1)
    step(opened, 120)
    results = []
    for x in (0.35, 0.38, 0.41, 0.44, 0.47):
        for y in (-0.025, 0.0, 0.025):
            for z in (0.435, 0.45, 0.465):
                step(opened, 45)
                pose = torch.tensor([[x, y, z, 1.0, 0.0, 0.0, 0.0]], device=sim.device)
                cube.write_root_pose_to_sim_index(root_pose=pose)
                cube.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=sim.device))
                force = step(closed, 60)
                results.append({"position": [x, y, z], "left_N": force[0], "right_N": force[1], "min_N": min(force)})
    results.sort(key=lambda item: item["min_N"], reverse=True)
    report = {"status": "PASS", "cube_size_m": 0.035, "top_candidates": results[:12], "all_candidates": results}
    out = root / "out/lesson_06/grasp_pose_search.json"
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report["top_candidates"], indent=2))
    return 0


if __name__ == "__main__":
    try:
        result = main()
    finally:
        app.close()
    raise SystemExit(result)
