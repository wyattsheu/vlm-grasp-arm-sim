#!/usr/bin/env python3
"""Run the REAL robot's mm_actions grasp/place code, unmodified, against the Isaac sim.

The action logic (VLM decision -> depth patch -> camera_to_base via the real hand-eye
calibration -> rtb Piper position-only IK -> QP servo -> close-until-tight -> return) is
imported at runtime from references/upstream/mm_system/main_ws/src/mm_actions, which is
deliberately not tracked in git (licence unconfirmed, see README). Nothing here copies or
patches that code; this file only rebuilds the thin ROS shell of the real
mm_actions_node.py (which needs the lab-only mm_interface action and cv_bridge) and maps
its topics onto the sim:

  real                                         sim
  /camera/color/image_raw                      /robot129_sim/camera/color/image_raw
  /camera/aligned_depth_to_color/image_raw     /robot129_sim/camera/aligned_depth_to_color/image_raw
    (16UC1 mm)                                   (32FC1 m -> converted to 16UC1 mm here)
  /camera/aligned_depth_to_color/camera_info   /robot129_sim/camera/aligned_depth_to_color/camera_info
  joint_states_feedback                        /robot129_sim/piper/joint_states_feedback
  /joint_states (command)                      /robot129_sim/piper/joint_cmd

is_holding_tightly() follows the real node's use_force_grasp=false path exactly (the real
arm has no force sensor): "holding" once the commanded width reaches grasp_close_width.
So, as on the real robot, "grasp complete" does not prove the object was picked up; the
report's sim_ground_truth block (target pose before/after, from the sim) is the check.

Usage: tools/run_real_stack_task.sh "grasp the red block"
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
import pkgutil
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
UPSTREAM = ROOT / "references/upstream/mm_system/main_ws/src/mm_actions"
sys.path.insert(0, str(UPSTREAM))

import rclpy  # noqa: E402
import rerun as rr  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from message_filters import ApproximateTimeSynchronizer, Subscriber  # noqa: E402
from PIL import Image as PILImage, ImageDraw  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, JointState  # noqa: E402

import mm_actions.actions as upstream_actions  # noqa: E402
from mm_actions.actions.base_action import BaseAction  # noqa: E402
from mm_actions.stage_log import Stage  # noqa: E402

# references/upstream/mm_system/main_ws/ed305_arm_pose.py OBSERVE (= grasp.py's return pose).
OBSERVE = [0.0, 0.2, -0.6, 0.0, 0.8, 0.0]
GRIPPER_OPEN = 0.1


def image_to_numpy(msg: Image) -> np.ndarray:
    if msg.encoding in ("rgb8", "bgr8"):
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return arr[..., ::-1].copy() if msg.encoding == "bgr8" else arr.copy()
    if msg.encoding in ("rgba8", "bgra8"):
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 4)[..., :3]
        return arr[..., ::-1].copy() if msg.encoding == "bgra8" else arr.copy()
    if msg.encoding == "32FC1":
        return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()
    if msg.encoding == "16UC1":
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).copy()
    raise ValueError(f"unsupported encoding {msg.encoding}")


def depth_m_to_realsense_mm(depth_m: np.ndarray) -> np.ndarray:
    """RealSense aligned depth is 16UC1 millimetres with 0 = no reading; the real
    camera_2d_to_3d() divides by 1000 and drops zeros via its 0.1-3.0 m window."""
    out = np.where(np.isfinite(depth_m) & (depth_m > 0), depth_m * 1000.0, 0.0)
    return np.clip(np.round(out), 0, 65535).astype(np.uint16)


class SimMmActionsNode(Node):
    def __init__(self, grasp_close_width: float):
        super().__init__("sim_mm_actions")
        self._grasp_close_width = grasp_close_width
        self._last_gripper_cmd = None
        self._latest_image = None
        self._latest_joint_state = None
        self._target_pose = None

        # The sim publishes camera and object topics best-effort (sensor QoS).
        qos = qos_profile_sensor_data
        color = Subscriber(self, Image, "/robot129_sim/camera/color/image_raw", qos_profile=qos)
        depth = Subscriber(self, Image, "/robot129_sim/camera/aligned_depth_to_color/image_raw", qos_profile=qos)
        info = Subscriber(self, CameraInfo, "/robot129_sim/camera/aligned_depth_to_color/camera_info", qos_profile=qos)
        self._sync = ApproximateTimeSynchronizer([color, depth, info], queue_size=30, slop=0.05)
        self._sync.registerCallback(self._synced_image_cb)
        self.create_subscription(JointState, "/robot129_sim/piper/joint_states_feedback", self._joint_cb, 1)
        self.create_subscription(PoseStamped, "/robot129_sim/objects/target_cube/pose", self._target_cb, qos)
        self._cmd_pub = self.create_publisher(JointState, "/robot129_sim/piper/joint_cmd", 10)

    def _synced_image_cb(self, img_msg, depth_msg, info_msg):
        self._latest_image = {
            "rgb": image_to_numpy(img_msg),
            "depth": depth_m_to_realsense_mm(image_to_numpy(depth_msg)),
            "intrinsics": {"fx": info_msg.k[0], "fy": info_msg.k[4], "cx": info_msg.k[2], "cy": info_msg.k[5]},
            "stamp": time.time(),
        }

    def _joint_cb(self, msg):
        self._latest_joint_state = list(msg.position)

    def _target_cb(self, msg):
        p = msg.pose.position
        self._target_pose = [p.x, p.y, p.z]

    # --- the callbacks the real BaseAction receives ---
    def get_image(self):
        return self._latest_image

    def get_joint_state(self):
        return self._latest_joint_state

    def is_holding_tightly(self) -> bool:
        return self._last_gripper_cmd is not None and self._last_gripper_cmd <= self._grasp_close_width

    def publish_arm_cmd(self, q, gripper=None):
        msg = JointState()
        q = list(q)
        assert len(q) == 6, f"Expected 6 joint commands for the arm, got {len(q)}"
        msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        msg.position = [float(v) for v in q]
        if gripper is not None:
            msg.position.append(float(gripper))
            msg.name.append("gripper")
            self._last_gripper_cmd = float(gripper)
        msg.header.stamp = self.get_clock().now().to_msg()
        self._cmd_pub.publish(msg)


def load_actions():
    """Same discovery as the real mm_actions_node._load_actions()."""
    actions = {}
    for _, name, _ in pkgutil.iter_modules(upstream_actions.__path__):
        module = importlib.import_module(f"mm_actions.actions.{name}")
        if getattr(module, "ACTION_NAME", None) and getattr(module, "ACTION_CLASS", None) is not None:
            actions[module.ACTION_NAME] = module.ACTION_CLASS
    return actions


def upstream_revision():
    """Which mm_system code ran: the sim imports whatever branch is checked out there
    (e.g. a local fix not yet pushed), so every report says which one."""
    import subprocess

    def git(*a):
        return subprocess.run(["git", "-C", str(UPSTREAM), *a], capture_output=True,
                              text=True).stdout.strip()
    return {"branch": git("branch", "--show-current"), "commit": git("rev-parse", "--short", "HEAD"),
            "dirty": bool(git("status", "--porcelain", "--", "."))}


def make_vlm_client():
    backend = os.getenv("VLM_BACKEND", "local").strip().lower()
    if backend == "gemini":
        from mm_actions.reasoning.gemini_client import GeminiRoboticsClient

        api_key = os.getenv("GOOGLE_API_KEY") or os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise SystemExit("VLM_BACKEND=gemini needs GOOGLE_API_KEY or GEMINI_API_KEY in the environment")
        # GEMINI_MODEL overrides the upstream default (gemini-robotics-er-2-preview) through the
        # constructor argument the real client already exposes; unset keeps the real behaviour.
        return backend, GeminiRoboticsClient(api_key=api_key, model=os.getenv("GEMINI_MODEL") or None)
    if backend == "local":
        from mm_actions.reasoning.local_pipeline_client import LocalPipelineClient

        return backend, LocalPipelineClient()
    raise SystemExit(f'VLM_BACKEND={backend!r} not recognized - use "local" or "gemini"')


def wait_for(predicate, timeout_s, what):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if predicate():
            return
        time.sleep(0.05)
    raise SystemExit(f"timed out after {timeout_s:.0f}s waiting for {what}")


def save_vlm_debug(rgb, decision, path: Path):
    img = PILImage.fromarray(rgb)
    draw = ImageDraw.Draw(img)
    if decision is None:
        draw.text((10, 10), "NOT FOUND", fill=(255, 0, 0))
    else:
        x, y = decision.point
        draw.ellipse([x - 12, y - 12, x + 12, y + 12], outline=(0, 255, 0), width=3)
        draw.line([x - 18, y, x + 18, y], fill=(0, 255, 0), width=3)
        draw.line([x, y - 18, x, y + 18], fill=(0, 255, 0), width=3)
        draw.text((10, 10), f"{decision.action} | {decision.label}", fill=(0, 255, 0))
    img.save(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("instruction")
    parser.add_argument("--grasp-close-width", type=float, default=0.032,
                        help="real node param grasp_close_width: object width minus 0.5-1 cm "
                             "(RUNBOOK sec 6). Default fits the 4 cm counter target.")
    parser.add_argument("--skip-observe", action="store_true",
                        help="do not move to OBSERVE first (the real flow runs ed305_arm_pose.py by hand)")
    parser.add_argument("--vlm-retries", type=int, default=None,
                        help="extra decide_task attempts when the VLM returns nothing (5 s, 10 s, ... backoff); "
                             "default 4 for gemini (503 overloads), 0 for local (None there means Molmo2 "
                             "found no target, which a retry at temperature 0 just repeats)")
    args = parser.parse_args()
    if args.vlm_retries is None:
        args.vlm_retries = 4 if os.getenv("VLM_BACKEND", "local").strip().lower() == "gemini" else 0

    if not UPSTREAM.is_dir():
        raise SystemExit(f"missing {UPSTREAM} (real-robot code is kept out of git; copy it in first)")

    out_dir = ROOT / "out/grasp_motion/real_stack" / datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    rr.init("sim_mm_actions", spawn=False)
    rr.save(str(out_dir / "rerun.rrd"))

    rclpy.init()
    node = SimMmActionsNode(args.grasp_close_width)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    report = {"instruction": args.instruction, "grasp_close_width": args.grasp_close_width,
              "mm_system": upstream_revision()}
    try:
        wait_for(lambda: node.get_joint_state() is not None, 15, "piper/joint_states_feedback (is the sim up?)")
        wait_for(lambda: node.get_image() is not None, 15, "synced RGB-D + camera_info")
        wait_for(lambda: node._target_pose is not None, 5, "target pose")
        report["sim_ground_truth"] = {"target_xyz_before": node._target_pose}

        if not args.skip_observe:
            helper = BaseAction(node.get_image, node.get_joint_state, node.is_holding_tightly,
                                node.publish_arm_cmd, grasp_close_width=args.grasp_close_width)
            _obs = Stage("OBSERVE_POSE", print).start(f"q={OBSERVE}")
            helper.move_arm_to_joint_state(OBSERVE + [GRIPPER_OPEN])
            time.sleep(1.5)
            _obs.ok()

        t_req = time.time()
        wait_for(lambda: node.get_image()["stamp"] > t_req, 10, "a frame newer than the observe move")
        image = node.get_image()
        joint_state_at_image = node.get_joint_state()
        PILImage.fromarray(image["rgb"]).save(out_dir / "vlm_input.png")

        backend, client = make_vlm_client()
        report["vlm_backend"] = backend
        _obs_task = Stage("ARM_TASK", print).start(f'command="{args.instruction}"')
        _obs_vlm = Stage("VLM", print).start(f"backend={backend}")
        # The real client swallows API errors (e.g. 503 "high demand" on the preview model) and
        # returns None, same as "target not visible". Retry a few times with backoff so a
        # transient overload doesn't fail the run; temperature is 0, so a genuine "not visible"
        # just repeats and costs a few extra calls.
        decision = None
        for attempt in range(1, args.vlm_retries + 2):
            decision = client.decide_task(image_rgb=image["rgb"], instruction=args.instruction)
            if decision is not None or attempt > args.vlm_retries:
                break
            delay = 5 * attempt
            print(f"[VLM] RETRY | attempt={attempt + 1}/{args.vlm_retries + 1} in {delay}s")
            time.sleep(delay)
        report["vlm_attempts"] = attempt
        save_vlm_debug(image["rgb"], decision, out_dir / "vlm_debug.png")
        if decision is None:
            _obs_vlm.fail("NO_DECISION", "API error or target not visible")
            _obs_task.fail("VLM_FAILED")
            report.update(success=False, message="VLM could not determine a task")
            return 1
        _obs_vlm.ok(f"action={decision.action} | label={decision.label} | "
                    f"pixel=({decision.point[0]:.1f}, {decision.point[1]:.1f})")
        report["decision"] = {"action": decision.action, "label": decision.label,
                              "pixel_xy": [float(v) for v in decision.point]}

        action_cls = load_actions().get(decision.action)
        if action_cls is None:
            _obs_task.fail("UNKNOWN_ACTION", f"action={decision.action}")
            report.update(success=False, message=f"unknown action: {decision.action}")
            return 1
        action = action_cls(node.get_image, node.get_joint_state, node.is_holding_tightly,
                            node.publish_arm_cmd, image, decision.point, joint_state_at_image,
                            grasp_close_width=args.grasp_close_width)
        success, message = action.run()
        (_obs_task.ok if success else _obs_task.fail)(f"action={decision.action} | message={message}")
        time.sleep(1.0)
        report.update(success=bool(success), message=message)
        report["sim_ground_truth"]["target_xyz_after"] = node._target_pose
        before, after = report["sim_ground_truth"]["target_xyz_before"], node._target_pose
        report["sim_ground_truth"]["target_lift_m"] = after[2] - before[2]
        return 0 if success else 1
    finally:
        (out_dir / "report.json").write_text(json.dumps(report, indent=2))
        print(f"[SIM_MM_ACTIONS] report={out_dir / 'report.json'}", flush=True)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
