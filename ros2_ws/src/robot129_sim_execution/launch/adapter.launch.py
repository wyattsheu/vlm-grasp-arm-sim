"""Launch the Robot 129 grasp/motion FollowJointTrajectory adapter.

This does not launch Isaac or MoveIt; it only starts the adapter node that bridges
between them. Start the live Isaac runner (tools/start_robot129_grasp_sim.sh) first.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robot129_sim_execution")
    params_file = os.path.join(share, "config", "adapter_params.yaml")
    return LaunchDescription([
        Node(
            package="robot129_sim_execution",
            executable="adapter_node",
            name="robot129_grasp_motion_adapter",
            namespace="/robot129_sim",
            output="screen",
            parameters=[params_file],
        ),
    ])
