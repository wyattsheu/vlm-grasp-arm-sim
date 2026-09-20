"""move_group in the /robot129_sim namespace, so its relative topics/actions
(joint_states, arm_controller/follow_joint_trajectory, ...) resolve to the live Isaac
runner's actual names instead of the root namespace used by move_group.launch.py
(which move_group.launch.py deliberately keeps unchanged for the existing plan-only
fixed-target regression in tools/verify_moveit_fixed_plan.sh).

Grasp/motion S2: move_group -> robot129_sim_execution adapter -> live Isaac.
"""
from launch import LaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    config = (
        MoveItConfigsBuilder("robot129", package_name="robot129_moveit_config")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )
    inner = generate_move_group_launch(config)
    return LaunchDescription([
        GroupAction([
            PushRosNamespace("robot129_sim"),
            *inner.entities,
        ]),
    ])
