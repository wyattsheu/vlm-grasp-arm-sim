"""Plan (not execute) the S2 MTC pick-and-place task against the LIVE Isaac robot
state. Namespaced under /robot129_sim so the node's relative `joint_states`
subscription resolves to the real /robot129_sim/joint_states published by
sim/scripts/run_robot129_ros_webrtc.py, not a fake state.

Also starts move_group in the same namespace: MTC's CurrentState stage acquires its
initial PlanningScene via the /get_planning_scene service (observed: without a running
move_group this times out after 3s with "Failed to acquire current PlanningScene",
even though ongoing state updates would otherwise come straight from joint_states).
This move_group is plan-only supporting infrastructure for MTC's CurrentState stage,
not a substitute for one -- it is not used to plan or execute anything itself here.

Does not touch Isaac; start it first with tools/start_robot129_grasp_sim.sh --scene pick_place.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    config = (
        MoveItConfigsBuilder("robot129", package_name="robot129_moveit_config")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )
    report_path_arg = DeclareLaunchArgument(
        "report_path",
        default_value="/mnt/HDD4/wyattsheu/handoff/robot129_pro6000_sim_20260913/out/grasp_motion/mtc_pick_place.json",
    )
    # Empty by default -- preserves the original single-hardcoded-candidate S2 behavior.
    # Pass a grasp_contract.py JSON export (research/src/mpg/grasp_contract.py) to try
    # multiple S3-generated candidates in score order instead.
    candidates_path_arg = DeclareLaunchArgument("candidates_path", default_value="")
    max_candidates_arg = DeclareLaunchArgument("max_candidates", default_value="8")
    # Place-side collision-aware position refinement (ZeroDex Sec 3.4 eq. 12 stand-in,
    # see robot129_mtc_pick_place.cpp's PlaceOffsetCandidate comment): xy step size for
    # the fixed nominal-plus-4-neighbor search grid around the nominal place pose.
    place_offset_step_arg = DeclareLaunchArgument("place_offset_step_m", default_value="0.015")
    # Test-only (default false): inject a collision obstacle at the nominal place point
    # to verify the place-offset fallthrough loop actually falls through to a neighbor.
    debug_block_place_arg = DeclareLaunchArgument("debug_block_place_nominal", default_value="false")
    move_group_launch = generate_move_group_launch(config)
    mtc = Node(
        package="robot129_tasks",
        executable="robot129_mtc_pick_place",
        output="screen",
        parameters=[config.to_dict(), {
            "report_path": LaunchConfiguration("report_path"),
            "candidates_path": LaunchConfiguration("candidates_path"),
            "max_candidates": LaunchConfiguration("max_candidates"),
            "place_offset_step_m": LaunchConfiguration("place_offset_step_m"),
            "debug_block_place_nominal": LaunchConfiguration("debug_block_place_nominal"),
        }],
    )
    mtc_delayed = TimerAction(period=6.0, actions=[mtc])
    return LaunchDescription([
        report_path_arg,
        candidates_path_arg,
        max_candidates_arg,
        place_offset_step_arg,
        debug_block_place_arg,
        GroupAction([
            PushRosNamespace("robot129_sim"),
            *move_group_launch.entities,
            mtc_delayed,
        ]),
        RegisterEventHandler(OnProcessExit(
            target_action=mtc,
            on_exit=[EmitEvent(event=Shutdown(reason="MTC pick_place planning complete"))],
        )),
    ])
