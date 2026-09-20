#!/usr/bin/env python3
"""Request and validate one collision-aware fixed joint-space MoveIt plan."""
import json, math, sys, time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.msg import CollisionObject
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

root=Path(__file__).resolve().parents[1]
rclpy.init(domain_id=129)
node=rclpy.create_node('robot129_moveit_acceptance', namespace='/robot129_sim')
client=ActionClient(node,MoveGroup,'/move_action')
if not client.wait_for_server(timeout_sec=30):
 print('FAIL: /move_action unavailable'); rclpy.shutdown(); raise SystemExit(2)
start=[0.0,1.20,-1.25,0.0,0.15,0.0,0.035,-0.035]
target=[0.25,1.35,-1.55,0.10,0.20,0.0]
goal=MoveGroup.Goal(); req=goal.request
req.group_name='arm'; req.num_planning_attempts=5; req.allowed_planning_time=5.0
req.pipeline_id='ompl'; req.planner_id='RRTConnect'
req.max_velocity_scaling_factor=0.2; req.max_acceleration_scaling_factor=0.2
req.start_state.joint_state=JointState(name=[f'joint{i}' for i in range(1,9)],position=start)
req.start_state.is_diff=False
constraints=Constraints(name='fixed_joint_goal')
for i,value in enumerate(target,1):
 constraints.joint_constraints.append(JointConstraint(joint_name=f'joint{i}',position=value,tolerance_above=0.001,tolerance_below=0.001,weight=1.0))
req.goal_constraints=[constraints]
obstacle=CollisionObject(); obstacle.header.frame_id='base_link'; obstacle.id='side_obstacle'
box=SolidPrimitive(); box.type=SolidPrimitive.BOX; box.dimensions=[0.10,0.10,0.20]
pose=Pose(); pose.position.x=0.55; pose.position.y=0.35; pose.position.z=0.10; pose.orientation.w=1.0
obstacle.primitives=[box]; obstacle.primitive_poses=[pose]; obstacle.operation=CollisionObject.ADD
goal.planning_options.planning_scene_diff.world.collision_objects=[obstacle]
goal.planning_options.planning_scene_diff.is_diff=True; goal.planning_options.plan_only=True
future=client.send_goal_async(goal); rclpy.spin_until_future_complete(node,future,timeout_sec=10)
handle=future.result()
if handle is None or not handle.accepted:
 print('FAIL: goal rejected'); rclpy.shutdown(); raise SystemExit(3)
rf=handle.get_result_async(); rclpy.spin_until_future_complete(node,rf,timeout_sec=20)
wrapped=rf.result(); result=None if wrapped is None else wrapped.result
points=[] if result is None else list(result.planned_trajectory.joint_trajectory.points)
final=[] if not points else list(points[-1].positions)
err=math.inf if len(final)<6 else max(abs(a-b) for a,b in zip(final[:6],target))
code=None if result is None else result.error_code.val
checks={'move_action_available':True,'goal_accepted':handle.accepted,'moveit_success_code':code==MoveItErrorCodes.SUCCESS,'trajectory_has_multiple_points':len(points)>1,'final_goal_error_under_0_01_rad':err<0.01,'collision_object_in_request':True}
report={'status':'PASS' if all(checks.values()) else 'FAIL','simulation_only':True,'ros_domain_id':129,'group':'arm','pipeline':'ompl','planner':'RRTConnect','start':start,'target':target,'trajectory_points':len(points),'final_goal_error_rad':err,'moveit_error_code':code,'planning_time_s':None if result is None else result.planning_time,'collision_object':{'id':'side_obstacle','frame':'base_link','size_m':[.1,.1,.2],'position_m':[.55,.35,.1]},'checks':checks,'hardware_commands':0}
out=root/'out/lesson_09/moveit_fixed_plan.json'; out.parent.mkdir(parents=True,exist_ok=True); out.write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps(report,indent=2))
node.destroy_node(); rclpy.shutdown(); raise SystemExit(0 if report['status']=='PASS' else 1)
