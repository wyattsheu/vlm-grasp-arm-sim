#!/usr/bin/env python3
"""FollowJointTrajectory adapter: MoveIt/MTC (or any control_msgs action client) ->
live Isaac Robot 129 simulation.

Why this exists (docs/progress/grasp_motion_s0_inventory.md): the live Isaac runner
(sim/scripts/run_robot129_ros_webrtc.py) only understands raw JointTrajectory topics.
It has no action server, no cancel semantics of its own, and signals completion only
through a machine-readable ``trajectory_events`` topic (added alongside this adapter).
This node is a SEPARATE ROS process -- not embedded in the Isaac process, to avoid
loading this environment's control_msgs typesupport into Isaac's bundled rclpy and to
keep goal-tracking latency independent of render stalls -- that exposes two proper
FollowJointTrajectory action servers (arm, gripper), validates and forwards goals,
and determines success/failure/cancellation from the runner's real feedback.

Design points from the plan (grasp_motion_research_plan_20260918.md S1 / runbook A-C):
  - joint_names may arrive in any order; unknown/duplicate/missing/non-finite/
    out-of-limit values are rejected with a specific reason, never silently coerced.
  - one active goal per channel (arm, gripper); a new goal while busy is REJECTED at
    goal-accept time, not accepted-then-aborted (no preemption in this first version).
  - this adapter must be the only publisher on the target command topic; if another
    node (e.g. the mock ros2_control controller_manager) is also publishing, the goal
    is rejected rather than silently racing two writers.
  - cancel publishes an immediate hold-at-measured-position command and only reports
    CANCELED once that hold has actually been observed in joint_states, not on the
    mere act of publishing it.
  - a stale JointState feed (measured via the wall/steady clock, not simulation time)
    aborts an in-flight goal rather than hanging forever.
  - accepting a goal does not mean success; success requires the runner's COMPLETE
    event for that channel AND the final measured position within goal tolerance.
    The gripper additionally treats "stopped short of the commanded position while in
    contact with the target object" as a distinct, explicitly reported success mode
    (a fully closed goal against a real object is physically impossible), never a
    silent PASS.
  - every goal is logged (goal id, scene revision at accept time, robot state
    timestamp, result, cancel events) to out/grasp_motion/adapter/events.jsonl.

Because the live Isaac runner's actual sustained frame rate has been measured at
~19 Hz against its nominal 120 Hz physics rate (real-time factor ~0.16, comparable to
the historically measured ~4.3-4.6 Hz wrist-camera rate) rather than the nominal rate,
execution timeouts are derived from the planned trajectory duration divided by a
configurable minimum assumed real-time factor, not a fixed few-second constant.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path
from threading import Lock

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot129_sim_execution import robot_model as rm

# control_msgs/action/FollowJointTrajectory.action error codes.
SUCCESSFUL = 0
INVALID_GOAL = -1
INVALID_JOINTS = -2
OLD_HEADER_TIMESTAMP = -3
PATH_TOLERANCE_VIOLATED = -4
GOAL_TOLERANCE_VIOLATED = -5


class ChannelConfig:
    def __init__(self, name, joint_names, command_topic, action_name, tolerance):
        self.name = name
        self.joint_names = joint_names
        self.command_topic = command_topic
        self.action_name = action_name
        self.tolerance = tolerance  # dict joint_name -> abs tolerance


class ChannelState:
    def __init__(self):
        self.busy = False
        self.lock = Lock()
        self.last_event = None       # most recent trajectory_events payload for this channel
        self.last_event_seq = 0
        self.consumed_seq = 0


class AdapterNode(Node):
    def __init__(self):
        super().__init__("robot129_grasp_motion_adapter")

        self.declare_parameter("min_realtime_factor", rm.DEFAULT_MIN_REALTIME_FACTOR)
        self.declare_parameter("timeout_margin_s", rm.DEFAULT_TIMEOUT_MARGIN_S)
        self.declare_parameter("stale_feedback_timeout_s", 3.0)
        # 0.03, not 0.02 -- see config/adapter_params.yaml's comment: two independent
        # real executions both landed joint6 just outside 0.02 on the RRTConnect
        # "connect to place" move.
        self.declare_parameter("arm_goal_tolerance_rad", 0.03)
        self.declare_parameter("gripper_goal_tolerance_m", 0.003)
        self.declare_parameter("gripper_contact_force_n", 0.3)
        self.declare_parameter("hold_duration_s", 0.2)
        self.declare_parameter("log_dir", "out/grasp_motion/adapter")

        arm_tol = float(self.get_parameter("arm_goal_tolerance_rad").value)
        grip_tol = float(self.get_parameter("gripper_goal_tolerance_m").value)
        self.channels = {
            "arm": ChannelConfig(
                "arm", rm.ARM_JOINT_NAMES, rm.ARM_COMMAND_TOPIC, rm.ARM_ACTION_NAME,
                {n: arm_tol for n in rm.ARM_JOINT_NAMES},
            ),
            "gripper": ChannelConfig(
                "gripper", rm.GRIPPER_JOINT_NAMES, rm.GRIPPER_COMMAND_TOPIC, rm.GRIPPER_ACTION_NAME,
                {n: grip_tol for n in rm.GRIPPER_JOINT_NAMES},
            ),
        }
        self.state = {key: ChannelState() for key in self.channels}

        cg = ReentrantCallbackGroup()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
        )

        self.joint_state_lock = Lock()
        self.latest_joint_state = None       # dict joint_name -> position
        self.latest_joint_state_time = None  # steady-clock time.monotonic() of receipt
        self.create_subscription(JointState, rm.JOINT_STATES_TOPIC, self._on_joint_state, sensor_qos, callback_group=cg)

        self.contact_lock = Lock()
        self.latest_contact_force = {"link7": 0.0, "link8": 0.0}
        for link, topic in rm.CONTACT_TOPICS.items():
            self.create_subscription(
                WrenchStamped, topic,
                (lambda msg, link=link: self._on_contact(link, msg)),
                sensor_qos, callback_group=cg,
            )

        self.scene_lock = Lock()
        self.latest_scene_revision = None
        # The runner publishes scene_state as transient-local (latched) so a late
        # subscriber still gets the last known revision. Matching durability here is
        # required, not optional: a plain-volatile subscriber connects fine but never
        # receives the latched replay, only messages published after it subscribed
        # (observed: scene_revision stayed null for a whole session with no resets).
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, rm.SCENE_STATE_TOPIC, self._on_scene_state, latched_qos, callback_group=cg)

        self.event_seq_counter = 0
        self.create_subscription(String, rm.TRAJECTORY_EVENTS_TOPIC, self._on_event, 10, callback_group=cg)

        self.publishers_by_channel = {
            key: self.create_publisher(JointTrajectory, cfg.command_topic, 10)
            for key, cfg in self.channels.items()
        }

        # Resolved lazily in _log(), not here: reading the parameter now would freeze
        # in whatever value was declared/overridden at construction time, which is
        # fine for a normal launch (overrides apply before __init__ runs) but not for
        # tests that call set_parameters() on an already-constructed node.
        self._log_lock = Lock()
        self._goal_counter = 0

        self.action_servers = {}
        for key, cfg in self.channels.items():
            self.action_servers[key] = ActionServer(
                self, FollowJointTrajectory, cfg.action_name,
                execute_callback=(lambda gh, key=key: self._execute(key, gh)),
                goal_callback=(lambda goal_request, key=key: self._goal_callback(key, goal_request)),
                cancel_callback=self._cancel_callback,
                callback_group=cg,
            )
            self.get_logger().info(f"FollowJointTrajectory action server ready: {cfg.action_name}")

    # ---- subscriptions ----

    def _on_joint_state(self, msg: JointState):
        with self.joint_state_lock:
            self.latest_joint_state = dict(zip(msg.name, msg.position))
            self.latest_joint_state_time = time.monotonic()

    def _on_contact(self, link, msg: WrenchStamped):
        force = math.sqrt(msg.wrench.force.x**2 + msg.wrench.force.y**2 + msg.wrench.force.z**2)
        with self.contact_lock:
            self.latest_contact_force[link] = force

    def _on_scene_state(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        with self.scene_lock:
            self.latest_scene_revision = payload.get("revision")

    def _on_event(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        channel = payload.get("channel")
        if channel not in self.state:
            return
        st = self.state[channel]
        with st.lock:
            st.last_event = payload
            st.last_event_seq += 1

    # ---- logging ----

    def _log(self, record: dict):
        record["stamp_wall"] = time.time()
        log_path = Path(self.get_parameter("log_dir").value) / "events.jsonl"
        with self._log_lock:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            with log_path.open("a") as f:
                f.write(json.dumps(record) + "\n")

    # ---- validation / reordering ----

    def _validate_and_reorder(self, cfg: ChannelConfig, goal: FollowJointTrajectory.Goal):
        """Return (reordered_points, reason). reason is None on success."""
        names = list(goal.trajectory.joint_names)
        expected = set(cfg.joint_names)
        if len(names) != len(cfg.joint_names):
            return None, f"expected {len(cfg.joint_names)} joint_names, got {len(names)}"
        if len(set(names)) != len(names):
            return None, "duplicate joint_names in goal"
        if set(names) != expected:
            missing = expected - set(names)
            unknown = set(names) - expected
            return None, f"joint_names mismatch: missing={sorted(missing)} unknown={sorted(unknown)}"

        # Permutation: index in the goal's order for each canonical joint name.
        goal_index = {name: i for i, name in enumerate(names)}
        perm = [goal_index[name] for name in cfg.joint_names]

        if not goal.trajectory.points:
            return None, "trajectory has no points"

        reordered_points = []
        for point in goal.trajectory.points:
            if len(point.positions) != len(cfg.joint_names):
                return None, "a trajectory point has the wrong number of positions"
            positions = [point.positions[i] for i in perm]
            if not all(math.isfinite(v) for v in positions):
                return None, "non-finite position in trajectory"
            for name, value in zip(cfg.joint_names, positions):
                lo, hi = rm.JOINT_LIMITS[name]
                if not (lo - 1e-6 <= value <= hi + 1e-6):
                    return None, f"{name}={value} outside limits [{lo}, {hi}]"
            new_point = JointTrajectoryPoint()
            new_point.positions = positions
            if point.velocities and len(point.velocities) == len(cfg.joint_names):
                new_point.velocities = [point.velocities[i] for i in perm]
            if point.accelerations and len(point.accelerations) == len(cfg.joint_names):
                new_point.accelerations = [point.accelerations[i] for i in perm]
            new_point.time_from_start = point.time_from_start
            reordered_points.append(new_point)
        return reordered_points, None

    # ---- action server callbacks ----

    def _goal_callback(self, key: str, goal_request):
        cfg = self.channels[key]
        st = self.state[key]
        with st.lock:
            if st.busy:
                self.get_logger().warn(f"[{key}] REJECT goal: busy")
                return GoalResponse.REJECT

        publisher_count = self.count_publishers(cfg.command_topic)
        if publisher_count != 1:
            self.get_logger().error(
                f"[{key}] REJECT goal: expected exactly 1 writer on {cfg.command_topic}, "
                f"found {publisher_count} (another controller may be active)"
            )
            return GoalResponse.REJECT

        points, reason = self._validate_and_reorder(cfg, goal_request)
        if reason is not None:
            self.get_logger().warn(f"[{key}] REJECT goal: {reason}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _hold_command(self, key: str) -> bool:
        """Publish a short hold-at-measured-position trajectory. Returns True if a
        measured position was available to hold at."""
        cfg = self.channels[key]
        with self.joint_state_lock:
            js = dict(self.latest_joint_state) if self.latest_joint_state else None
        if js is None or not all(n in js for n in cfg.joint_names):
            return False
        hold_duration = float(self.get_parameter("hold_duration_s").value)
        msg = JointTrajectory()
        msg.joint_names = list(cfg.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [js[n] for n in cfg.joint_names]
        total_ns = max(1, int(hold_duration * 1e9))
        point.time_from_start.sec = total_ns // 1_000_000_000
        point.time_from_start.nanosec = total_ns % 1_000_000_000
        msg.points = [point]
        self.publishers_by_channel[key].publish(msg)
        return True

    def _execute(self, key: str, goal_handle):
        cfg = self.channels[key]
        st = self.state[key]
        goal = goal_handle.request

        points, reason = self._validate_and_reorder(cfg, goal)
        with self.scene_lock:
            scene_revision = self.latest_scene_revision
        self._goal_counter += 1
        goal_id = self._goal_counter
        with self.joint_state_lock:
            state_stamp = self.latest_joint_state_time

        if reason is not None:
            # Should not happen (goal_callback already validated), defensive only.
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = INVALID_GOAL
            result.error_string = reason
            self._log({"goal_id": goal_id, "channel": key, "scene_revision": scene_revision,
                       "robot_state_timestamp": state_stamp, "result": "ABORTED", "reason": reason})
            return result

        with st.lock:
            st.busy = True
            st.last_event = None
        self._log({"goal_id": goal_id, "channel": key, "scene_revision": scene_revision,
                   "robot_state_timestamp": state_stamp, "result": "STARTED",
                   "n_points": len(points), "duration_s": _point_seconds(points[-1])})

        try:
            fwd = JointTrajectory()
            fwd.joint_names = list(cfg.joint_names)
            fwd.points = points
            self.publishers_by_channel[key].publish(fwd)

            duration = _point_seconds(points[-1])
            min_rtf = float(self.get_parameter("min_realtime_factor").value)
            margin = float(self.get_parameter("timeout_margin_s").value)
            timeout_s = duration / max(min_rtf, 1e-3) + margin
            stale_timeout = float(self.get_parameter("stale_feedback_timeout_s").value)

            deadline = time.monotonic() + timeout_s
            result = FollowJointTrajectory.Result()
            outcome = None
            reason_str = ""

            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    held = self._hold_command(key)
                    time.sleep(max(0.1, float(self.get_parameter("hold_duration_s").value)))
                    goal_handle.canceled()
                    result.error_code = SUCCESSFUL
                    result.error_string = "CANCELED: held at measured position" if held else \
                        "CANCELED: no measured position was available to hold"
                    self._log({"goal_id": goal_id, "channel": key, "scene_revision": scene_revision,
                               "result": "CANCELED", "held": held})
                    return result

                with self.joint_state_lock:
                    last_state_time = self.latest_joint_state_time
                if last_state_time is None or (time.monotonic() - last_state_time) > stale_timeout:
                    outcome, reason_str = "STALE_FEEDBACK", (
                        f"STALE_FEEDBACK: no joint_states for over {stale_timeout}s while goal active"
                    )
                    break

                with st.lock:
                    event = st.last_event
                if event is not None and event.get("kind") == "REJECT":
                    outcome, reason_str = "RUNNER_REJECT", event.get("reason", "runner rejected trajectory")
                    break
                if event is not None and event.get("kind") == "COMPLETE":
                    outcome = "RUNNER_COMPLETE"
                    break

                time.sleep(0.05)
            else:
                outcome, reason_str = "TIMEOUT", (
                    f"no COMPLETE/REJECT event within {timeout_s:.1f}s "
                    f"(planned duration {duration:.2f}s, min_realtime_factor {min_rtf})"
                )

            if outcome == "RUNNER_REJECT":
                goal_handle.abort()
                result.error_code = INVALID_GOAL
                result.error_string = reason_str
                self._log({"goal_id": goal_id, "channel": key, "result": "ABORTED", "reason": reason_str})
                return result
            if outcome == "STALE_FEEDBACK":
                goal_handle.abort()
                result.error_code = PATH_TOLERANCE_VIOLATED
                result.error_string = reason_str
                self._log({"goal_id": goal_id, "channel": key, "result": "ABORTED", "reason": reason_str})
                return result
            if outcome == "TIMEOUT":
                goal_handle.abort()
                result.error_code = GOAL_TOLERANCE_VIOLATED
                result.error_string = reason_str
                self._log({"goal_id": goal_id, "channel": key, "result": "ABORTED", "reason": reason_str})
                return result

            # outcome == "RUNNER_COMPLETE": verify final tolerance against measured state.
            final_target = dict(zip(cfg.joint_names, points[-1].positions))
            with self.joint_state_lock:
                measured = dict(self.latest_joint_state) if self.latest_joint_state else {}
            errors = {n: abs(measured.get(n, math.nan) - final_target[n]) for n in cfg.joint_names}
            within_tol = all(
                math.isfinite(errors[n]) and errors[n] <= cfg.tolerance[n] for n in cfg.joint_names
            )

            if within_tol:
                goal_handle.succeed()
                result.error_code = SUCCESSFUL
                result.error_string = "OK"
                self._log({"goal_id": goal_id, "channel": key, "result": "SUCCEEDED", "errors": errors})
                return result

            if key == "gripper":
                with self.contact_lock:
                    forces = dict(self.latest_contact_force)
                min_force = float(self.get_parameter("gripper_contact_force_n").value)
                stalled_on_object = forces["link7"] >= min_force and forces["link8"] >= min_force
                if stalled_on_object:
                    goal_handle.succeed()
                    result.error_code = SUCCESSFUL
                    result.error_string = (
                        f"OK (stalled_on_object): joint7 error {errors['joint7']:.4f}m exceeds tolerance "
                        f"but link7/link8 contact forces {forces} indicate the gripper is holding an object"
                    )
                    self._log({"goal_id": goal_id, "channel": key, "result": "SUCCEEDED_STALLED_ON_OBJECT",
                               "errors": errors, "contact_forces": forces})
                    return result

            goal_handle.abort()
            result.error_code = GOAL_TOLERANCE_VIOLATED
            result.error_string = f"final position error {errors} exceeds tolerance {cfg.tolerance}"
            self._log({"goal_id": goal_id, "channel": key, "result": "ABORTED", "errors": errors})
            return result
        finally:
            with st.lock:
                st.busy = False


def _point_seconds(point: JointTrajectoryPoint) -> float:
    return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = AdapterNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
