"""Verify ROS 2 domain-129 trajectory commands drive the Isaac articulation."""
import argparse
import json
import time
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument('--bundle', type=Path, required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
launcher = AppLauncher(args)
app = launcher.app

import torch
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sim import SimulationContext


def main():
    root = args.bundle.resolve()
    sim = SimulationContext(sim_utils.SimulationCfg(device=args.device, dt=1/120, render_interval=8))
    sim_utils.GroundPlaneCfg().func('/World/Ground', sim_utils.GroundPlaneCfg())
    robot = Articulation(ArticulationCfg(
        prim_path='/World/Robot',
        spawn=sim_utils.UsdFileCfg(usd_path=str(root/'sim/assets/robot129/robot129/robot129.usda')),
        actuators={
            'arm': ImplicitActuatorCfg(joint_names_expr=['joint[1-6]'], effort_limit_sim=100, stiffness=800, damping=80),
            'gripper': ImplicitActuatorCfg(joint_names_expr=['joint[78]'], effort_limit_sim=10, stiffness=2000, damping=100),
        },
    ))
    sim.reset()
    names = [f'joint{i}' for i in range(1,9)]
    start = torch.tensor([0.0,1.20,-1.25,0.0,0.15,0.0,0.035,-0.035], device=sim.device)
    goal = torch.tensor([0.25,1.35,-1.55,0.10,0.20,0.0,0.020,-0.020], device=sim.device)
    target = start.clone()
    command_received = False
    rejected_commands = 0
    received_129 = []
    received_0 = []

    ctx129, ctx0 = Context(), Context()
    rclpy.init(context=ctx129, domain_id=129)
    rclpy.init(context=ctx0, domain_id=0)
    bridge = rclpy.create_node('isaac_articulation_bridge', namespace='/robot129_sim', context=ctx129)
    client = rclpy.create_node('trajectory_acceptance_client', namespace='/robot129_sim', context=ctx129)
    probe = rclpy.create_node('production_domain_probe', namespace='/robot129_sim', context=ctx0)

    def on_command(msg):
        nonlocal command_received, target, rejected_commands
        if list(msg.joint_names) != names or not msg.points or len(msg.points[-1].positions) != 8:
            rejected_commands += 1
            return
        target = torch.tensor(msg.points[-1].positions, device=sim.device)
        command_received = True

    command_topic = '/robot129_sim/joint_trajectory'
    state_topic = '/robot129_sim/joint_states'
    bridge.create_subscription(JointTrajectory, command_topic, on_command, 10)
    state_pub = bridge.create_publisher(JointState, state_topic, 10)
    command_pub = client.create_publisher(JointTrajectory, command_topic, 10)
    client.create_subscription(JointState, state_topic, lambda m: received_129.append(list(m.position)), 10)
    probe.create_subscription(JointState, state_topic, lambda m: received_0.append(list(m.position)), 10)
    ex129 = SingleThreadedExecutor(context=ctx129); ex129.add_node(bridge); ex129.add_node(client)
    ex0 = SingleThreadedExecutor(context=ctx0); ex0.add_node(probe)

    robot.write_joint_position_to_sim_index(position=start.unsqueeze(0))
    robot.write_joint_velocity_to_sim_index(velocity=torch.zeros((1,8),device=sim.device))
    for step in range(420):
        if 50 <= step < 90:
            msg=JointTrajectory(); msg.joint_names=names
            point=JointTrajectoryPoint(); point.positions=goal.tolist(); point.time_from_start.sec=1
            msg.points=[point]; command_pub.publish(msg)
        ex129.spin_once(timeout_sec=0.0)
        ex0.spin_once(timeout_sec=0.0)
        robot.set_joint_position_target_index(target=target.unsqueeze(0))
        robot.write_data_to_sim(); sim.step(render=False); robot.update(sim.get_physics_dt())
        state=JointState(); state.name=names; state.position=robot.data.joint_pos.torch[0].detach().cpu().tolist()
        state.header.stamp=bridge.get_clock().now().to_msg(); state_pub.publish(state)
        ex129.spin_once(timeout_sec=0.0); ex0.spin_once(timeout_sec=0.0)
        if step < 120: time.sleep(0.01)
    final=robot.data.joint_pos.torch[0].detach().cpu()
    error=float(torch.max(torch.abs(final-goal.cpu())).item())
    checks={
        'command_received': command_received,
        'joint_state_roundtrip': len(received_129)>10,
        'domain_0_received_zero': len(received_0)==0,
        'actual_articulation_error_under_0_01_rad_m': error<0.01,
        'no_command_rejection': rejected_commands==0,
    }
    report={
        'status':'PASS' if all(checks.values()) else 'FAIL', 'simulation_only':True,
        'ros_domain_id':129, 'namespace':'/robot129_sim', 'command_topic':command_topic,
        'state_topic':state_topic, 'received_joint_states_domain_129':len(received_129),
        'received_joint_states_domain_0':len(received_0), 'max_final_error':error,
        'goal':goal.tolist(), 'final':final.tolist(), 'checks':checks, 'hardware_drivers':0,
    }
    out=root/'out/lesson_08/ros_isaac_roundtrip.json'; out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps(report,indent=2))
    for n in (bridge,client,probe): n.destroy_node()
    ex129.shutdown(); ex0.shutdown(); ctx129.shutdown(); ctx0.shutdown()
    return 0 if report['status']=='PASS' else 1

if __name__=='__main__':
    try: code=main()
    finally: app.close()
    raise SystemExit(code)
