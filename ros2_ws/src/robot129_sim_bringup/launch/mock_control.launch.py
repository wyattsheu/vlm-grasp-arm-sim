from pathlib import Path
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
 urdf=Path(get_package_share_directory('robot129_description'))/'urdf/robot129.urdf.xacro'
 description=xacro.process_file(str(urdf)).toxml()
 config=str(Path(get_package_share_directory('robot129_sim_bringup'))/'config/ros2_controllers.yaml')
 common={'robot_description':description,'use_sim_time':False}
 return LaunchDescription([
  Node(package='robot_state_publisher',executable='robot_state_publisher',namespace='robot129_sim',parameters=[common],output='screen'),
  Node(package='controller_manager',executable='ros2_control_node',namespace='robot129_sim',parameters=[common,config],output='screen'),
  TimerAction(period=2.0,actions=[Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','--controller-manager','/robot129_sim/controller_manager'])]),
  TimerAction(period=3.0,actions=[Node(package='controller_manager',executable='spawner',arguments=['arm_controller','--controller-manager','/robot129_sim/controller_manager'])]),
  TimerAction(period=4.0,actions=[Node(package='controller_manager',executable='spawner',arguments=['gripper_controller','--controller-manager','/robot129_sim/controller_manager'])]),
 ])
