#!/usr/bin/env python3
import json
from pathlib import Path
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / 'out/lesson_08/ros2_control_guards.json'
JOINTS = [f'joint{i}' for i in range(1, 7)]
ACTION = '/robot129_sim/arm_controller/follow_joint_trajectory'


def wait(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result() if future.done() else None


def goal(positions, seconds):
    msg = FollowJointTrajectory.Goal()
    msg.trajectory.joint_names = JOINTS
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    whole = int(seconds)
    point.time_from_start = Duration(sec=whole, nanosec=int((seconds - whole) * 1e9))
    msg.trajectory.points = [point]
    msg.goal_time_tolerance = Duration(sec=1)
    return msg


def main():
    rclpy.init()
    node = rclpy.create_node('robot129_control_guard_acceptance', namespace='/robot129_sim')
    client = ActionClient(node, FollowJointTrajectory, ACTION)
    checks = {}
    detail = {}
    try:
        checks['action_server_available'] = client.wait_for_server(timeout_sec=12.0)
        if not checks['action_server_available']:
            raise RuntimeError(f'action server unavailable: {ACTION}')

        first_handle = wait(node, client.send_goal_async(goal([0.1, 1.0, -1.0, 0.0, 0.1, 0.0], 0.8)), 3.0)
        checks['known_goal_accepted'] = bool(first_handle and first_handle.accepted)
        first_result = wait(node, first_handle.get_result_async(), 4.0) if first_handle else None
        checks['known_goal_succeeded'] = bool(first_result and first_result.status == GoalStatus.STATUS_SUCCEEDED and first_result.result.error_code == 0)
        detail['known_goal_status'] = first_result.status if first_result else None

        long_handle = wait(node, client.send_goal_async(goal([0.5, 1.4, -1.5, 0.2, 0.3, 0.1], 8.0)), 3.0)
        checks['long_goal_accepted'] = bool(long_handle and long_handle.accepted)
        watchdog_started = time.monotonic()
        time.sleep(0.25)
        cancel_response = wait(node, long_handle.cancel_goal_async(), 3.0) if long_handle else None
        detail['watchdog_cancel_after_s'] = time.monotonic() - watchdog_started
        checks['watchdog_cancel_requested'] = bool(cancel_response and cancel_response.goals_canceling)
        canceled_result = wait(node, long_handle.get_result_async(), 4.0) if long_handle else None
        checks['canceled_goal_reports_canceled'] = bool(canceled_result and canceled_result.status == GoalStatus.STATUS_CANCELED)
        detail['canceled_goal_status'] = canceled_result.status if canceled_result else None

        recovery_handle = wait(node, client.send_goal_async(goal([0.0, 0.8, -0.8, 0.0, 0.0, 0.0], 0.8)), 3.0)
        checks['recovery_goal_accepted'] = bool(recovery_handle and recovery_handle.accepted)
        recovery_result = wait(node, recovery_handle.get_result_async(), 4.0) if recovery_handle else None
        checks['controller_recovers_after_cancel'] = bool(recovery_result and recovery_result.status == GoalStatus.STATUS_SUCCEEDED and recovery_result.result.error_code == 0)
        detail['recovery_goal_status'] = recovery_result.status if recovery_result else None
    except Exception as exc:
        detail['error'] = str(exc)
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()

    report = {
        'status': 'PASS' if checks and all(checks.values()) else 'FAIL',
        'simulation_only': True,
        'ros_domain_id': 129,
        'namespace': '/robot129_sim',
        'action': ACTION,
        'watchdog_policy': 'cancel an unfinished 8 s goal after 0.25 s, then require a recovery goal',
        'checks': checks,
        'detail': detail,
        'hardware_drivers': 0,
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
