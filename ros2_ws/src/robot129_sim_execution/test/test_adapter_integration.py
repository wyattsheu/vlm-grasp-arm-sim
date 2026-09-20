"""Integration tests for the FollowJointTrajectory adapter against a fake runner.

Runs the real AdapterNode and a FakeRunner in-process over real DDS on an isolated
ROS_DOMAIN_ID (chosen far from the live deployment's domain 129) so these tests never
touch the live Isaac simulation. Covers the runbook batch A-C acceptance criteria:
normal execution, joint-name permutation, busy-reject, cancel/hold, timeout, and
stale-feedback handling.

Run with:
  cd ros2_ws && colcon test --packages-select robot129_sim_execution
or directly:
  PYTHONPATH=... python3 -m pytest ros2_ws/src/robot129_sim_execution/test -v
"""
from __future__ import annotations

import os
import threading
import time

import pytest

os.environ.setdefault("ROS_DOMAIN_ID", "219")  # isolated from the live deployment (129)

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot129_sim_execution import robot_model as rm
from robot129_sim_execution.adapter_node import AdapterNode
from fake_runner import FakeRunner


def make_goal(names, target, duration_s):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(names)
    point = JointTrajectoryPoint()
    point.positions = list(target)
    total_ns = int(duration_s * 1e9)
    point.time_from_start.sec = total_ns // 1_000_000_000
    point.time_from_start.nanosec = total_ns % 1_000_000_000
    goal.trajectory.points = [point]
    return goal


def wait_for(future, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if future.done():
            return future.result()
        time.sleep(0.02)
    raise TimeoutError(f"future did not complete within {timeout_s}s")


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def system(ros_context):
    runner = FakeRunner()
    adapter = AdapterNode()
    adapter.set_parameters([
        rclpy.parameter.Parameter("min_realtime_factor", rclpy.Parameter.Type.DOUBLE, 1.0),
        rclpy.parameter.Parameter("timeout_margin_s", rclpy.Parameter.Type.DOUBLE, 1.0),
        rclpy.parameter.Parameter("stale_feedback_timeout_s", rclpy.Parameter.Type.DOUBLE, 0.4),
        rclpy.parameter.Parameter("log_dir", rclpy.Parameter.Type.STRING, "/tmp/robot129_adapter_test_logs"),
    ])
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(runner)
    executor.add_node(adapter)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    arm_client = ActionClient(runner, FollowJointTrajectory, rm.ARM_ACTION_NAME)
    gripper_client = ActionClient(runner, FollowJointTrajectory, rm.GRIPPER_ACTION_NAME)
    assert arm_client.wait_for_server(timeout_sec=5.0)
    assert gripper_client.wait_for_server(timeout_sec=5.0)
    # Let discovery settle and at least one joint_state through before tests start.
    time.sleep(0.5)

    yield {"runner": runner, "adapter": adapter, "arm": arm_client, "gripper": gripper_client}

    executor.shutdown()
    runner.destroy_node()
    adapter.destroy_node()


def test_normal_execution_succeeds(system):
    goal = make_goal(rm.ARM_JOINT_NAMES, [0.1, 0.2, -0.1, 0.05, 0.05, 0.0], 0.3)
    send_future = system["arm"].send_goal_async(goal)
    goal_handle = wait_for(send_future, 5.0)
    assert goal_handle.accepted, "normal goal must be accepted"
    result = wait_for(goal_handle.get_result_async(), 5.0)
    assert result.result.error_code == 0, result.result.error_string
    assert result.status == 4  # STATUS_SUCCEEDED


def test_joint_name_permutation_succeeds(system):
    canonical_target = [0.1, 0.2, -0.1, 0.05, 0.05, 0.0]  # matches rm.ARM_JOINT_NAMES order
    reversed_names = list(reversed(rm.ARM_JOINT_NAMES))
    reversed_target = list(reversed(canonical_target))  # same per-joint values, reversed order
    goal = make_goal(reversed_names, reversed_target, 0.3)
    send_future = system["arm"].send_goal_async(goal)
    goal_handle = wait_for(send_future, 5.0)
    assert goal_handle.accepted, "permuted-but-complete joint set must still be accepted"
    result = wait_for(goal_handle.get_result_async(), 5.0)
    assert result.result.error_code == 0, result.result.error_string


def test_unknown_joint_rejected(system):
    goal = make_goal(["joint1", "joint2", "joint3", "joint4", "joint5", "not_a_joint"],
                      [0.0] * 6, 0.3)
    send_future = system["arm"].send_goal_async(goal)
    goal_handle = wait_for(send_future, 5.0)
    assert not goal_handle.accepted, "unknown joint name must be rejected at goal-accept time"


def test_missing_joint_rejected(system):
    goal = make_goal(rm.ARM_JOINT_NAMES[:5], [0.0] * 5, 0.3)
    send_future = system["arm"].send_goal_async(goal)
    goal_handle = wait_for(send_future, 5.0)
    assert not goal_handle.accepted, "incomplete joint set must be rejected"


def test_out_of_limits_rejected(system):
    target = [0.0, 0.2, -0.1, 0.05, 0.05, 0.0]
    target[1] = 999.0  # joint2 limit is [0, 3.14]
    goal = make_goal(rm.ARM_JOINT_NAMES, target, 0.3)
    send_future = system["arm"].send_goal_async(goal)
    goal_handle = wait_for(send_future, 5.0)
    assert not goal_handle.accepted, "out-of-limit position must be rejected"


def test_busy_rejects_second_goal(system):
    goal = make_goal(rm.ARM_JOINT_NAMES, [0.1, 0.2, -0.1, 0.05, 0.05, 0.0], 1.5)
    first = wait_for(system["arm"].send_goal_async(goal), 5.0)
    assert first.accepted
    second_goal = make_goal(rm.ARM_JOINT_NAMES, [0.0, 0.1, -0.05, 0.0, 0.0, 0.0], 0.3)
    second = wait_for(system["arm"].send_goal_async(second_goal), 5.0)
    assert not second.accepted, "a goal on a busy channel must be rejected, not queued"
    # Drain the first goal so later tests see an idle channel.
    wait_for(first.get_result_async(), 5.0)


def test_cancel_holds_at_measured_position(system):
    goal = make_goal(rm.ARM_JOINT_NAMES, [0.3, 0.3, -0.3, 0.1, 0.1, 0.0], 3.0)
    goal_handle = wait_for(system["arm"].send_goal_async(goal), 5.0)
    assert goal_handle.accepted
    time.sleep(0.3)
    cancel_future = goal_handle.cancel_goal_async()
    wait_for(cancel_future, 5.0)
    result = wait_for(goal_handle.get_result_async(), 5.0)
    assert result.status == 5  # STATUS_CANCELED
    assert "CANCELED" in result.result.error_string
    # The channel must be free again immediately after cancellation.
    followup = make_goal(rm.ARM_JOINT_NAMES, [0.0, 0.05, -0.05, 0.0, 0.0, 0.0], 0.3)
    followup_handle = wait_for(system["arm"].send_goal_async(followup), 5.0)
    assert followup_handle.accepted, "channel must not stay busy after a cancel"
    wait_for(followup_handle.get_result_async(), 5.0)


def test_timeout_when_runner_never_completes(system):
    system["runner"].silently_ignore_next["gripper"] = True
    goal = make_goal(rm.GRIPPER_JOINT_NAMES, [0.02], 0.2)
    goal_handle = wait_for(system["gripper"].send_goal_async(goal), 5.0)
    assert goal_handle.accepted, "the adapter itself must accept a well-formed goal"
    result = wait_for(goal_handle.get_result_async(), 8.0)
    assert result.status == 6  # STATUS_ABORTED
    assert "TIMEOUT" in result.result.error_string or result.result.error_code == -5


def test_stale_feedback_aborts(system):
    system["runner"].publish_joint_states = False
    try:
        goal = make_goal(rm.ARM_JOINT_NAMES, [0.2, 0.2, -0.2, 0.05, 0.05, 0.0], 5.0)
        goal_handle = wait_for(system["arm"].send_goal_async(goal), 5.0)
        assert goal_handle.accepted
        result = wait_for(goal_handle.get_result_async(), 5.0)
        assert result.status == 6  # STATUS_ABORTED
        assert "STALE_FEEDBACK" in result.result.error_string
    finally:
        system["runner"].publish_joint_states = True
