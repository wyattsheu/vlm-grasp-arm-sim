"""Measure Robot 129 gripper body poses at open and closed commands."""

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
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sim import SimulationContext


def main() -> int:
    root = args.bundle.resolve()
    sim = SimulationContext(sim_utils.SimulationCfg(device=args.device, dt=1 / 120, render_interval=4))
    sim_utils.GroundPlaneCfg().func("/World/Ground", sim_utils.GroundPlaneCfg())
    robot = Articulation(
        ArticulationCfg(
            prim_path="/World/Robot",
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(root / "sim/assets/robot129/robot129/robot129.usda"),
                activate_contact_sensors=True,
            ),
            init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
            actuators={
                "arm": ImplicitActuatorCfg(
                    joint_names_expr=["joint[1-6]"], effort_limit_sim=100.0, stiffness=800.0, damping=80.0
                ),
                "gripper": ImplicitActuatorCfg(
                    joint_names_expr=["joint[78]"], effort_limit_sim=10.0, stiffness=2000.0, damping=100.0
                ),
            },
        )
    )
    sim.reset()
    arm = torch.tensor([[0.0, 1.20, -1.25, 0.0, 0.15, 0.0]], device=sim.device)
    records = {}
    for label, fingers in (("open", (0.035, -0.035)), ("closed", (0.0, 0.0))):
        target = torch.cat((arm, torch.tensor([fingers], device=sim.device)), dim=1)
        for _ in range(180):
            robot.set_joint_position_target_index(target=target)
            robot.write_data_to_sim()
            sim.step(render=False)
            robot.update(sim.get_physics_dt())
        item = {"joint_position": robot.data.joint_pos.torch[0].tolist(), "bodies": {}}
        for body in ("gripper_base", "link7", "link8"):
            idx = robot.body_names.index(body)
            item["bodies"][body] = {
                "position": robot.data.body_pos_w.torch[0, idx].tolist(),
                "quaternion_xyzw": robot.data.body_quat_w.torch[0, idx].tolist(),
            }
        records[label] = item
    report = {
        "status": "PASS",
        "body_names": list(robot.body_names),
        "joint_names": list(robot.joint_names),
        "measurements": records,
    }
    out = root / "out/lesson_06/gripper_probe.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    try:
        result = main()
    finally:
        app.close()
    raise SystemExit(result)
