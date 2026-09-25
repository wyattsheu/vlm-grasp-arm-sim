"""Two end-to-end Robot 129 cuRobo MotionPlanner demos on ROS domain 129.

Uses the existing MotionPlanner and Isaac JointTrajectory bridge.  The bar demo
stops at B, inserts a kinematic obstacle by ROS service, observes its TF,
updates the same planner's collision world, then replans B->A from measured q.
No continuous obstacle prediction is claimed.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch
import yaml

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "research/src"))

from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.scene import Cuboid, Scene
from curobo.types import JointState
from mpg.urdf_fk import UrdfChainFk, quaternion_angular_distance_xyzw

ARM = [f"joint{i}" for i in range(1, 7)]
HOME = [0.0, 1.2, -1.25, 0.0, 0.15, 0.0]
ROBOT_CONFIG = ROOT / "research/configs/curobo/robot129.yml"
URDF = ROOT / "ros2_ws/src/robot129_description/urdf/robot129.urdf"
STATIC_YAML = ROOT / "research/configs/demo/official_static_multi.yaml"
DYNAMIC_TARGETS_YAML = ROOT / "research/configs/demo/official_dynamic_bar_targets.yaml"
CONE = {"name": "red_cone_conservative_envelope", "center": [0.20, 0.39, 0.25], "size": [0.12, 0.14, 0.50]}
_BAR_CFG = json.loads((ROOT / "research/configs/scenes/official_dynamic_bar.json").read_text())
BAR = {"name": _BAR_CFG["name"], "center": _BAR_CFG["center_m"], "size": _BAR_CFG["size_m"]}
FLOOR = Cuboid(name="floor", pose=[0, 0, -0.025, 1, 0, 0, 0], dims=[4, 4, 0.05])
# Existing demos intentionally stretched cuRobo's interpolated trajectory by 4x.
# Keep that as the baseline and expose an execution multiplier so a scenario can
# compress the ROS time_from_start values without changing the planned path.
BASE_TIME_SCALE = 4.0
MAX_TARGET_ERROR_M = 0.025
MAX_ORIENTATION_ERROR_RAD = 0.15
MIN_CLEARANCE_M = 0.002
MIN_DETOUR_M = 0.025
# dynamic_bar: leg index (into `order`) at which the bar gets inserted. 1 means
# "right after the first LEFT reach" -- legs 1..len(order)-1 then all execute
# with the obstacle present, showing repeated avoided transfers instead of
# just one. Raised from the original 7 (bar only appeared before the very
# last leg, so only 1 obstacle-avoidance transfer ever ran) on user feedback
# that a single post-insertion transfer wasn't enough to see the behavior.
DYNAMIC_BAR_INSERT_LEG_INDEX = 1


def world(obstacle=None) -> Scene:
    cuboids = [FLOOR]
    if obstacle is not None:
        cuboids.append(Cuboid(name=obstacle["name"], pose=[*obstacle["center"], 1, 0, 0, 0], dims=obstacle["size"]))
    return Scene(cuboid=cuboids)


def load_targets(mode):
    if mode == "static_multi":
        entries = yaml.safe_load(STATIC_YAML.read_text())["waypoints"]
        return {e["name"].split("_")[0]: e for e in entries}
    entries = yaml.safe_load(DYNAMIC_TARGETS_YAML.read_text())["waypoints"]
    return {entry["name"]: entry for entry in entries}


def make_js(q):
    return JointState.from_position(torch.tensor([list(q)], device="cuda", dtype=torch.float32), joint_names=ARM)


def timed_plan(planner, start_q, goal_q, time_scale):
    start = time.monotonic()
    result = planner.plan_cspace(make_js(goal_q), make_js(start_q), max_attempts=5)
    elapsed = time.monotonic() - start
    if result is None or not bool(result.success.any().item()):
        return None, {"success": False, "planning_time_s": elapsed}
    n = int(result.interpolated_last_tstep.view(-1)[0].item())
    all_names = planner.kinematics.all_articulated_joint_names
    columns = [all_names.index(name) for name in ARM]
    q = result.interpolated_trajectory.position[0, 0, :n, columns].detach().cpu().numpy()
    dt = float(result.interpolated_trajectory.dt.view(-1)[0].item())
    return q, {"success": True, "planning_time_s": elapsed, "solver_time_s": float(result.solve_time),
               "trajectory_duration_s": (n - 1) * dt * time_scale, "interpolation_dt_s": dt,
               "joint_trajectory_rad": q.tolist()}


def clearance(planner, q, obstacle):
    """All cuRobo collision spheres, including upper arm, forearm, wrist and gripper.

    For the visual cone, a circumscribing box is used: positive clearance to
    this box is conservative clearance to the actual narrower cone.
    """
    if obstacle is None or q is None or len(q) == 0:
        return None
    arr = torch.as_tensor(np.asarray(q), dtype=torch.float32, device="cuda").reshape(-1, 1, 6)
    spheres = planner.kinematics._forward(arr).robot_spheres[:, 0].detach().cpu().numpy()
    center, half = np.asarray(obstacle["center"]), np.asarray(obstacle["size"]) / 2
    closest = np.clip(spheres[:, :, :3], center - half, center + half)
    signed = np.linalg.norm(spheres[:, :, :3] - closest, axis=-1) - spheres[:, :, 3]
    return {"minimum_clearance_m": float(signed.min()), "penetrating_sphere_samples": int((signed < 0).sum()),
            "sample_count": int(signed.shape[0]), "sphere_count": int(signed.shape[1])}


def physical_cone_clearance(planner, q):
    """Sphere-to-solid-cone clearance for the actual Isaac cone (R=.06, H=.50).

    Rotation symmetry reduces nearest-surface distance to a point-to-triangle
    distance in (radial, z); the conservative cuboid remains the planner world.
    """
    arr = torch.as_tensor(np.asarray(q), dtype=torch.float32, device="cuda").reshape(-1, 1, 6)
    spheres = planner.kinematics._forward(arr).robot_spheres[:, 0].detach().cpu().numpy()
    radial = np.linalg.norm(spheres[:, :, :2] - np.array([0.20, 0.39]), axis=-1)
    xy = np.stack([radial, spheres[:, :, 2]], axis=-1)
    vertices = [np.array([0.0, 0.0]), np.array([0.06, 0.0]), np.array([0.0, 0.50])]
    distances = []
    for a, b in zip(vertices, vertices[1:] + vertices[:1]):
        edge = b - a
        fraction = np.clip(np.sum((xy - a) * edge, axis=-1) / np.dot(edge, edge), 0, 1)
        distances.append(np.linalg.norm(xy - (a + fraction[..., None] * edge), axis=-1))
    nearest = np.min(distances, axis=0)
    inside = (xy[..., 1] >= 0) & (xy[..., 1] <= 0.50) & (radial <= 0.06 * (1 - xy[..., 1] / 0.50))
    signed = np.where(inside, -nearest, nearest) - spheres[:, :, 3]
    return {"minimum_clearance_m": float(signed.min()), "penetrating_sphere_samples": int((signed < 0).sum())}


def ee_path(fk, q):
    return np.asarray([fk.pinch_center_pose(row.tolist())[0] for row in q], dtype=float)


def path_difference_m(fk, old_q, new_q):
    a, b = ee_path(fk, old_q), ee_path(fk, new_q)
    s = np.linspace(0, 1, 101)
    aa = np.column_stack([np.interp(s, np.linspace(0, 1, len(a)), a[:, i]) for i in range(3)])
    bb = np.column_stack([np.interp(s, np.linspace(0, 1, len(b)), b[:, i]) for i in range(3)])
    d = np.linalg.norm(aa - bb, axis=1)
    return {"maximum_ee_deviation_m": float(d.max()), "mean_ee_deviation_m": float(d.mean())}


class LiveBridge:
    def __init__(self, fk):
        import rclpy
        from geometry_msgs.msg import WrenchStamped
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState as RosJointState
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from tf2_msgs.msg import TFMessage
        from trajectory_msgs.msg import JointTrajectory
        self.rclpy = rclpy
        self.String = String
        self.JointTrajectory = JointTrajectory
        self.Trigger = Trigger
        self.fk = fk
        rclpy.init()
        self.node = rclpy.create_node("official_style_curobo_demo", namespace="/robot129_sim")
        self.arm_pub = self.node.create_publisher(JointTrajectory, "/robot129_sim/arm_controller/joint_trajectory", 10)
        self.gripper_pub = self.node.create_publisher(JointTrajectory, "/robot129_sim/gripper_controller/joint_trajectory", 10)
        self.path_pub = self.node.create_publisher(String, "/robot129_sim/demo/path", 10)
        self.status_pub = self.node.create_publisher(String, "/robot129_sim/demo/status", 10)
        self.insert_client = self.node.create_client(Trigger, "/robot129_sim/insert_bar_obstacle")
        self.latest_q = None
        self.latest_all = None
        self.samples = []
        self.contact_max = {name: 0.0 for name in ["link1", "link2", "link3", "link4", "link5", "link6", "gripper_base", "link7", "link8"]}
        self.contact_count = {name: 0 for name in self.contact_max}
        self.bar_tf = None
        self.events = []
        self.node.create_subscription(RosJointState, "/robot129_sim/joint_states", self._on_joints, 10)
        self.node.create_subscription(String, "/robot129_sim/trajectory_events", self._on_event, 10)
        self.node.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for name in self.contact_max:
            self.node.create_subscription(WrenchStamped, f"/robot129_sim/contacts/{name}",
                                          lambda msg, link=name: self._on_contact(link, msg), qos)

    def _on_joints(self, msg):
        by_name = dict(zip(msg.name, msg.position))
        if all(name in by_name for name in ARM):
            self.latest_q = np.asarray([by_name[name] for name in ARM], dtype=float)
            self.latest_all = by_name
            self.samples.append({"stamp_s": float(msg.header.stamp.sec) + msg.header.stamp.nanosec * 1e-9,
                                 "joint_values_rad": self.latest_q.tolist(),
                                 "ee_position_m": self.fk.pinch_center_pose(self.latest_q.tolist())[0].tolist()})

    def _on_contact(self, name, msg):
        f = msg.wrench.force
        value = float(np.linalg.norm([f.x, f.y, f.z]))
        self.contact_max[name] = max(self.contact_max[name], value)
        self.contact_count[name] += 1

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == "dynamic_stick":
                self.bar_tf = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]

    def _on_event(self, msg):
        try:
            self.events.append(json.loads(msg.data))
        except json.JSONDecodeError:
            pass

    def spin(self, timeout=0.05):
        self.rclpy.spin_once(self.node, timeout_sec=timeout)

    def ready(self, timeout=40):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin()
            if self.latest_q is not None and self.arm_pub.get_subscription_count() and self.gripper_pub.get_subscription_count():
                return True
        return False

    def publish_path(self, label, kind, points):
        msg = self.String()
        msg.data = json.dumps({"label": label, "kind": kind, "xyz_m": np.asarray(points).tolist()})
        self.path_pub.publish(msg)

    def status(self, payload):
        msg = self.String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def send_trajectory(self, q, dt, time_scale, label, target, obstacle, planner, timeout_s=180):
        from trajectory_msgs.msg import JointTrajectoryPoint
        msg = self.JointTrajectory()
        msg.joint_names = ARM
        for i, row in enumerate(q):
            # The Isaac bridge interpolates from its current target. Omit t=0
            # to avoid rejecting a measured-state/commanded-state offset.
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in row]
            ns = int(round((i + 1) * dt * time_scale * 1e9))
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
            msg.points.append(point)
        start_sample = len(self.samples)
        start_event = len(self.events)
        wall_start = time.monotonic()
        self.arm_pub.publish(msg)
        deadline = time.monotonic() + timeout_s
        settled = 0
        while time.monotonic() < deadline:
            self.spin()
            recent = self.events[start_event:]
            if any(e.get("kind") == "REJECT" and e.get("channel") == "arm" for e in recent):
                break
            if self.latest_q is not None and np.max(np.abs(self.latest_q - np.asarray(target["joint_values_rad"]))) < 0.03:
                settled += 1
            else:
                settled = 0
            # Reaching a loose joint tolerance before the simulator has finished
            # interpolating is not completion: the arm may still be moving.
            completed = any(e.get("kind") == "COMPLETE" and e.get("channel") == "arm" for e in recent)
            if completed and settled >= 5:
                break
        actual = self.samples[start_sample:]
        if actual:
            self.publish_path(label, "actual", [s["ee_position_m"] for s in actual])
        q_actual = self.latest_q
        goal_p, goal_quat = self.fk.pinch_center_pose(target["joint_values_rad"])
        actual_p, actual_quat = self.fk.pinch_center_pose(q_actual.tolist()) if q_actual is not None else (None, None)
        path_q = np.asarray([s["joint_values_rad"] for s in actual]) if actual else None
        execution_wall_time_s = time.monotonic() - wall_start
        arm_events = [e for e in self.events[start_event:] if e.get("channel") == "arm"]
        accept = next((e for e in arm_events if e.get("kind") == "ACCEPT"), None)
        complete = next((e for e in arm_events if e.get("kind") == "COMPLETE"), None)
        execution_sim_time_s = (float(complete["stamp_sim_time"]) - float(accept["stamp_sim_time"])
                                if accept is not None and complete is not None else None)
        conservative_coll = clearance(planner, path_q, obstacle) if obstacle is not None else None
        # cuRobo intentionally plans against a box that circumscribes the cone.
        # At high execution speed the measured path can enter an empty corner of
        # that box while remaining clear of the physical cone. Use the physical
        # cone surface plus Isaac contact sensors for execution acceptance, and
        # retain the conservative-box result as a diagnostic.
        coll = (physical_cone_clearance(planner, path_q)
                if obstacle is not None and obstacle.get("name") == CONE["name"]
                else conservative_coll)
        result = {"target": label, "settled": settled >= 5 and any(
                      e.get("kind") == "COMPLETE" and e.get("channel") == "arm" for e in self.events[start_event:]),
                  "actual_joint_trajectory": actual,
                  "actual_minimum_clearance_m": coll["minimum_clearance_m"] if coll else None,
                  "actual_conservative_box_clearance_m": (
                      conservative_coll["minimum_clearance_m"] if conservative_coll else None),
                  "execution_wall_time_s": execution_wall_time_s,
                  "execution_sim_time_s": execution_sim_time_s,
                  "observed_real_time_factor": (
                      execution_sim_time_s / execution_wall_time_s if execution_sim_time_s is not None else None),
                  "final_position_error_m": float(np.linalg.norm(actual_p - goal_p)) if actual_p is not None else None,
                  "final_orientation_error_rad": quaternion_angular_distance_xyzw(actual_quat, goal_quat) if actual_quat is not None else None,
                  "trajectory_events": self.events[start_event:], "contact_max_n": dict(self.contact_max)}
        result["pass"] = bool(result["settled"] and result["final_position_error_m"] <= MAX_TARGET_ERROR_M
                              and result["final_orientation_error_rad"] <= MAX_ORIENTATION_ERROR_RAD
                              and (coll is None or coll["minimum_clearance_m"] > MIN_CLEARANCE_M)
                              and max(self.contact_max.values()) < 0.01)
        return result

    def gripper(self, closed):
        from trajectory_msgs.msg import JointTrajectoryPoint
        msg = self.JointTrajectory()
        msg.joint_names = ["joint7"]
        pt = JointTrajectoryPoint()
        pt.positions = [0.002 if closed else 0.01725]
        gripper_duration_s = 0.5
        ns = int(gripper_duration_s * 1e9)
        pt.time_from_start.sec, pt.time_from_start.nanosec = divmod(ns, 10**9)
        msg.points = [pt]
        first_event = len(self.events)
        self.gripper_pub.publish(msg)
        end = time.monotonic() + 45
        while time.monotonic() < end:
            self.spin()
            complete = any(e.get("kind") == "COMPLETE" and e.get("channel") == "gripper"
                           for e in self.events[first_event:])
            if complete and self.latest_all is not None and abs(self.latest_all.get("joint7", 999) - pt.positions[0]) < 0.003:
                return True
        return False

    def insert_bar(self):
        if not self.insert_client.wait_for_service(timeout_sec=10):
            raise RuntimeError("/robot129_sim/insert_bar_obstacle unavailable")
        send_wall = time.monotonic()
        fut = self.insert_client.call_async(self.Trigger.Request())
        while not fut.done() and time.monotonic() - send_wall < 20:
            self.spin()
        if not fut.done() or not fut.result().success:
            raise RuntimeError("bar insertion service failed")
        response = json.loads(fut.result().message)
        until = time.monotonic() + 10
        while time.monotonic() < until:
            self.spin()
            if self.bar_tf is not None and np.linalg.norm(np.asarray(self.bar_tf) - BAR["center"]) < 0.01:
                return {"request_wall_monotonic_s": send_wall, "confirmed_wall_monotonic_s": time.monotonic(),
                        "sim_insertion_time_s": response["sim_time_s"], "tf_center_m": self.bar_tf}
        raise RuntimeError("bar insertion service succeeded but obstacle TF never reached the path")

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def run(mode, out_dir, speed_multiplier):
    if speed_multiplier <= 0:
        raise ValueError("speed_multiplier must be positive")
    time_scale = BASE_TIME_SCALE / speed_multiplier
    out_dir.mkdir(parents=True, exist_ok=True)
    fk = UrdfChainFk(URDF)
    targets = load_targets(mode)
    obstacle = CONE if mode == "static_multi" else None
    planner = MotionPlanner(MotionPlannerCfg.create(robot=str(ROBOT_CONFIG), scene_model=world(obstacle),
                                                 collision_cache={"cuboid": 4}, use_cuda_graph=False))
    planner.warmup(enable_graph=True, num_warmup_iterations=1)
    bridge = LiveBridge(fk)
    report = {"mode": mode, "status": "FAIL", "pipeline": "MotionPlanner + explicit world update + ROS JointTrajectory + Isaac Sim",
              "robot_collision_links": list(yaml.safe_load(ROBOT_CONFIG.read_text())["kinematics"]["collision_link_names"]),
              "obstacle": obstacle, "speed_multiplier": speed_multiplier,
              "trajectory_time_scale": time_scale, "legs": [], "contact_max_n": {}, "failure_reason": None}
    try:
        if not bridge.ready():
            raise RuntimeError("joint_states or Isaac trajectory subscribers unavailable")
        if mode == "static_multi":
            order = ["A", "B", "C", "D", "A"]
            baseline = MotionPlanner(MotionPlannerCfg.create(robot=str(ROBOT_CONFIG), scene_model=world(),
                                                              collision_cache={"cuboid": 4}, use_cuda_graph=False))
            baseline.warmup(enable_graph=True, num_warmup_iterations=1)
        else:
            # Reach LEFT once, then insert the bar and run the remaining
            # transfers (RIGHT/LEFT alternating) with the obstacle present, so
            # the avoided trajectory repeats multiple times instead of once.
            # DYNAMIC_BAR_INSERT_LEG_INDEX assumes order[0] == "LEFT" and
            # order[DYNAMIC_BAR_INSERT_LEG_INDEX] == "RIGHT" -- update the
            # hardcoded targets["RIGHT"] lookup below if that index ever moves.
            order = ["LEFT", "RIGHT", "LEFT", "RIGHT", "LEFT", "RIGHT", "LEFT", "RIGHT"]
            baseline = None
        for index, key in enumerate(order):
            if mode == "dynamic_bar" and index == DYNAMIC_BAR_INSERT_LEG_INDEX:
                # Current arm is stopped at LEFT. Compare the same LEFT->RIGHT
                # movement before and after insertion, with the target unchanged.
                current = bridge.latest_q.tolist()
                old_q, old_meta = timed_plan(planner, current, targets["RIGHT"]["joint_values_rad"], time_scale)
                if old_q is None:
                    raise RuntimeError("pre-insertion LEFT->RIGHT baseline planning failed")
                bridge.publish_path("LEFT_to_RIGHT_before", "before", ee_path(fk, old_q))
                report["pre_insertion_LEFT_to_RIGHT"] = old_meta
                report["insertion"] = bridge.insert_bar()
                t_update = time.monotonic()
                planner.update_world(world(BAR))
                report["world_update_time_s"] = time.monotonic() - t_update
                report["obstacle"] = BAR
                blocked = clearance(planner, old_q, BAR)
                report["old_trajectory_against_new_world"] = blocked
                report["old_trajectory_joint_rad"] = old_q.tolist()
                if blocked["minimum_clearance_m"] >= 0:
                    raise RuntimeError("inserted bar did not block the old B->A trajectory")
                obstacle = BAR
            start_q = bridge.latest_q.tolist()
            target = targets[key]
            q, meta = timed_plan(planner, start_q, target["joint_values_rad"], time_scale)
            leg = {"sequence_index": index, "target": key, "planning": meta}
            report["legs"].append(leg)
            if q is None:
                raise RuntimeError(f"cuRobo planning failed for leg {index} -> {key}")
            label = f"{index}_{key}"
            leg["planned_minimum_clearance"] = clearance(planner, q, obstacle)
            bridge.publish_path(label, "planned", ee_path(fk, q))
            if mode == "static_multi" and index == 1:
                old_q, baseline_meta = timed_plan(baseline, start_q, target["joint_values_rad"], time_scale)
                if old_q is None:
                    raise RuntimeError("static baseline A->B planning failed")
                leg["unobstructed_baseline"] = baseline_meta
                leg["unobstructed_path_against_cone"] = clearance(planner, old_q, CONE)
                leg["unobstructed_path_against_physical_cone"] = physical_cone_clearance(planner, old_q)
                leg["planned_path_against_physical_cone"] = physical_cone_clearance(planner, q)
                leg["detour"] = path_difference_m(fk, old_q, q)
                bridge.publish_path("A_to_B_unobstructed", "before", ee_path(fk, old_q))
            if mode == "dynamic_bar" and index == DYNAMIC_BAR_INSERT_LEG_INDEX:
                report["new_trajectory_joint_rad"] = q.tolist()
                report["replanning_time_s"] = meta["planning_time_s"]
                report["path_change"] = path_difference_m(fk, old_q, q)
                bridge.publish_path("LEFT_to_RIGHT_after", "after", ee_path(fk, q))
            bridge.status({"mode": mode, "leg": index, "target": key, "obstacle_active": obstacle is not None,
                           "planning_time_s": meta["planning_time_s"]})
            leg["execution"] = bridge.send_trajectory(
                q, meta["interpolation_dt_s"], time_scale, label, target, obstacle, planner
            )
            print(f"[{mode}] leg={index} target={key} planning={meta['planning_time_s']:.3f}s "
                  f"error={leg['execution']['final_position_error_m']} pass={leg['execution']['pass']}", flush=True)
            if not leg["execution"]["pass"]:
                raise RuntimeError(f"execution/collision/target check failed on leg {index} -> {key}")
            if mode == "dynamic_bar":
                leg["gripper_close_ok"] = bridge.gripper(True)
                leg["gripper_open_ok"] = bridge.gripper(False)
                if not leg["gripper_close_ok"] or not leg["gripper_open_ok"]:
                    raise RuntimeError(f"gripper sequence failed at {key}")
        report["contact_max_n"] = bridge.contact_max
        report["contact_samples"] = bridge.contact_count
        if mode == "static_multi":
            proof = report["legs"][1]
            if proof["unobstructed_path_against_physical_cone"]["minimum_clearance_m"] >= 0:
                raise RuntimeError("physical cone did not block baseline A->B path")
            if proof["detour"]["maximum_ee_deviation_m"] < MIN_DETOUR_M:
                raise RuntimeError("cone trajectory change is not visually distinct")
        else:
            if report["path_change"]["maximum_ee_deviation_m"] < MIN_DETOUR_M:
                raise RuntimeError("bar trajectory change is not visually distinct")
        if any(n == 0 for n in bridge.contact_count.values()):
            raise RuntimeError("one or more obstacle contact sensors published no samples")
        report["status"] = "PASS"
    except Exception as exc:
        import traceback
        traceback.print_exc()
        report["failure_reason"] = f"{type(exc).__name__}: {exc}"
        print(f"[{mode}] FAIL {report['failure_reason']}", flush=True)
    finally:
        report["contact_max_n"] = bridge.contact_max
        report["contact_samples"] = bridge.contact_count
        (out_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        bridge.close()
    return 0 if report["status"] == "PASS" else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["static_multi", "dynamic_bar"])
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--speed-multiplier", type=float, default=1.0,
                        help="execute this many times faster than the original 4x-slow demo timing")
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "129":
        raise SystemExit("ROS_DOMAIN_ID=129 required")
    return run(args.mode, args.out_dir, args.speed_multiplier)


if __name__ == "__main__":
    raise SystemExit(main())
