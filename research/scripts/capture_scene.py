#!/usr/bin/env python3
"""Capture exactly one subscriber-only synchronized RGB-D SceneBundle.

This program creates subscriptions and a TF listener only. It deliberately
contains no ROS publisher, service client, action client, or motion interface.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image as PilImage

WORK_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK_DIR / "src"))

from mpg.scene_bundle import sha256_file, transform_matrix, validate_scene, write_json


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def decode_color(message: Any) -> np.ndarray:
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
    encoding = message.encoding.lower()
    if encoding not in channels:
        raise ValueError(f"unsupported color encoding: {message.encoding}")
    count = channels[encoding]
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    image = rows[:, : message.width * count].reshape(message.height, message.width, count)
    if encoding in ("bgr8", "bgra8"):
        image = image[..., [2, 1, 0] + ([3] if count == 4 else [])]
    return np.ascontiguousarray(image[..., :3])


def decode_depth(message: Any) -> np.ndarray:
    encoding = message.encoding.lower()
    formats = {
        "16uc1": np.dtype(">u2" if message.is_bigendian else "<u2"),
        "mono16": np.dtype(">u2" if message.is_bigendian else "<u2"),
        "32fc1": np.dtype(">f4" if message.is_bigendian else "<f4"),
    }
    if encoding not in formats:
        raise ValueError(f"unsupported depth encoding: {message.encoding}")
    dtype = formats[encoding]
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    packed = np.ascontiguousarray(rows[:, : message.width * dtype.itemsize])
    return packed.view(dtype).reshape(message.height, message.width).astype(dtype.newbyteorder("="))


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--scene-id", required=True, help="Example: S1_01")
    result.add_argument("--instruction", required=True)
    result.add_argument("--base-frame", default="base_link")
    result.add_argument(
        "--allow-missing-tf",
        action="store_true",
        help="Capture a typed 2D-only bundle when T_base_camera is unavailable",
    )
    result.add_argument("--depth-scale-m", required=True, type=float,
                        help="Meters per raw depth unit; use 1.0 for 32FC1 meters")
    result.add_argument("--output-root", type=Path, default=WORK_DIR / "data" / "scenes")
    result.add_argument("--color-topic", default="/camera/color/image_raw")
    result.add_argument("--depth-topic", default="/camera/aligned_depth_to_color/image_raw")
    result.add_argument("--camera-info-topic",
                        default="/camera/aligned_depth_to_color/camera_info")
    result.add_argument("--joint-topic", default="/joint_states_feedback")
    result.add_argument("--sync-slop-ms", type=float, default=30.0)
    result.add_argument("--joint-max-age-ms", type=float, default=100.0)
    result.add_argument("--timeout-s", type=float, default=20.0)
    result.add_argument(
        "--best-effort", action="store_true",
        help="Subscribe with BEST_EFFORT reliability instead of the default RELIABLE. "
             "Needed for Isaac Sim's simulated camera publishers (sensor_qos there uses "
             "BEST_EFFORT, matching typical real depth-camera driver QoS elsewhere in ROS "
             "2, just not what this script defaulted to) -- without this the subscriber "
             "silently receives nothing and the capture times out with no useful error "
             "beyond the QoS-incompatibility WARN rclpy logs. Off by default so the "
             "existing real-camera capture path is unchanged.",
    )
    return result


def run(args: argparse.Namespace) -> Path:
    if not args.scene_id.replace("_", "").isalnum():
        raise ValueError("scene-id may contain only letters, digits, and underscore")
    if args.depth_scale_m <= 0 or not np.isfinite(args.depth_scale_m):
        raise ValueError("depth-scale-m must be finite and positive")
    output_root = args.output_root.resolve()
    allowed_root = (WORK_DIR / "data" / "scenes").resolve()
    if output_root != allowed_root:
        raise ValueError(f"output-root is fixed to writable workspace path {allowed_root}")
    final_dir = output_root / args.scene_id
    staging = output_root / f".{args.scene_id}.partial"
    if final_dir.exists() or staging.exists():
        raise FileExistsError(f"scene already exists or has partial data: {args.scene_id}")
    staging.mkdir(parents=True)

    try:
        import message_filters
        import rclpy
        from builtin_interfaces.msg import Time as TimeMessage
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.time import Time
        from sensor_msgs.msg import CameraInfo, Image, JointState
        from tf2_ros import Buffer, TransformException, TransformListener
    except ImportError as exc:
        shutil.rmtree(staging, ignore_errors=True)
        raise RuntimeError("ROS 2 Python environment is required to capture") from exc

    class SnapshotNode(Node):
        def __init__(self) -> None:
            super().__init__("mpg_subscriber_only_capture")
            self.done = False
            self.error: Exception | None = None
            self.latest_joint: tuple[Any, int] | None = None
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=(
                    ReliabilityPolicy.BEST_EFFORT if args.best_effort else ReliabilityPolicy.RELIABLE
                ),
            )
            self.joint_sub = self.create_subscription(
                JointState, args.joint_topic, self.on_joint, qos
            )
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.color_sub = message_filters.Subscriber(
                self, Image, args.color_topic, qos_profile=qos
            )
            self.depth_sub = message_filters.Subscriber(
                self, Image, args.depth_topic, qos_profile=qos
            )
            self.info_sub = message_filters.Subscriber(
                self, CameraInfo, args.camera_info_topic, qos_profile=qos
            )
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.color_sub, self.depth_sub, self.info_sub],
                queue_size=20,
                slop=args.sync_slop_ms / 1000.0,
            )
            self.sync.registerCallback(self.on_images)

        def on_joint(self, message: Any) -> None:
            self.latest_joint = (message, time.time_ns())

        def on_images(self, color: Any, depth: Any, info: Any) -> None:
            if self.done:
                return
            try:
                if self.latest_joint is None:
                    return
                joint, joint_rx_ns = self.latest_joint
                color_ns, depth_ns, info_ns = (
                    stamp_ns(color.header.stamp),
                    stamp_ns(depth.header.stamp),
                    stamp_ns(info.header.stamp),
                )
                joint_ns = stamp_ns(joint.header.stamp)
                reference_ns = color_ns
                joint_age_ms = (
                    abs(reference_ns - joint_ns) / 1e6 if joint_ns > 0
                    else abs(time.time_ns() - joint_rx_ns) / 1e6
                )
                if joint_age_ms > args.joint_max_age_ms:
                    return
                camera_frame = color.header.frame_id or info.header.frame_id
                if not camera_frame:
                    raise ValueError("camera frame_id is empty")
                ros_stamp = TimeMessage(
                    sec=int(color.header.stamp.sec),
                    nanosec=int(color.header.stamp.nanosec),
                )
                tf_msg = None
                tf_error = None
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        args.base_frame,
                        camera_frame,
                        Time.from_msg(ros_stamp),
                        timeout=Duration(seconds=0.2),
                    )
                except TransformException as exc:
                    if not args.allow_missing_tf:
                        return
                    tf_error = str(exc)

                rgb = decode_color(color)
                depth_array = decode_depth(depth)
                if rgb.shape[:2] != depth_array.shape[:2]:
                    raise ValueError("aligned RGB and depth dimensions differ")
                PilImage.fromarray(rgb, mode="RGB").save(staging / "rgb.png")
                if depth_array.dtype == np.uint16:
                    PilImage.fromarray(depth_array).save(staging / "depth.png")
                    depth_name = "depth.png"
                else:
                    np.save(staging / "depth.npy", depth_array.astype(np.float32),
                            allow_pickle=False)
                    depth_name = "depth.npy"

                write_json(staging / "camera_info.json", {
                    "width": int(info.width), "height": int(info.height),
                    "k": list(info.k), "d": list(info.d), "r": list(info.r),
                    "p": list(info.p), "distortion_model": info.distortion_model,
                    "binning_x": int(info.binning_x), "binning_y": int(info.binning_y),
                    "frame_id": info.header.frame_id,
                    "header_stamp_ns": info_ns,
                    "depth_scale_m": float(args.depth_scale_m),
                })
                if tf_msg is not None:
                    transform = tf_msg.transform
                    translation = [transform.translation.x, transform.translation.y,
                                   transform.translation.z]
                    quaternion = [transform.rotation.x, transform.rotation.y,
                                  transform.rotation.z, transform.rotation.w]
                    tf_data = {
                        "status": "AVAILABLE",
                        "direction": "T_base_camera",
                        "base_frame": args.base_frame,
                        "camera_frame": camera_frame,
                        "header_stamp_ns": stamp_ns(tf_msg.header.stamp),
                        "translation_xyz_m": translation,
                        "quaternion_xyzw": quaternion,
                        "matrix": transform_matrix(translation, quaternion),
                        "source": "tf2 lookup at RGB header timestamp",
                    }
                else:
                    tf_data = {
                        "status": "UNAVAILABLE",
                        "direction": "T_base_camera",
                        "base_frame": args.base_frame,
                        "camera_frame": camera_frame,
                        "header_stamp_ns": None,
                        "matrix": None,
                        "source": "tf2 lookup at RGB header timestamp",
                        "error": tf_error,
                        "restriction": "2D_ONLY_UNTIL_EXTRINSIC_IS_RECONSTRUCTED",
                    }
                write_json(staging / "tf.json", tf_data)
                write_json(staging / "joint_states.json", {
                    "frame_id": joint.header.frame_id,
                    "header_stamp_ns": joint_ns,
                    "received_at_unix_ns": joint_rx_ns,
                    "names": list(joint.name),
                    "positions": list(joint.position),
                    "velocities": list(joint.velocity),
                    "efforts": list(joint.effort),
                    "position_unit": "radian_except_gripper_as_published",
                })
                (staging / "instruction.txt").write_text(
                    args.instruction.strip() + "\n", encoding="utf-8"
                )
                files = ["rgb.png", depth_name, "camera_info.json", "tf.json",
                         "joint_states.json", "instruction.txt"]
                manifest = {
                    "schema_version": "scene_bundle_v1",
                    "scene_id": args.scene_id,
                    "capture_id": f"{args.scene_id}-{time.time_ns()}",
                    "capture_complete": True,
                    "topics": {
                        "color": args.color_topic, "depth": args.depth_topic,
                        "camera_info": args.camera_info_topic, "joint_states": args.joint_topic,
                    },
                    "encodings": {"color": color.encoding, "depth": depth.encoding},
                    "timestamps_ns": {
                        "rgb": color_ns, "depth": depth_ns, "camera_info": info_ns,
                        "joint_state": joint_ns,
                        "tf": (
                            stamp_ns(tf_msg.header.stamp) if tf_msg is not None else None
                        ),
                    },
                    "delta_from_rgb_ms": {
                        "depth": (depth_ns - color_ns) / 1e6,
                        "camera_info": (info_ns - color_ns) / 1e6,
                        "joint_state": (
                            (joint_ns - color_ns) / 1e6 if joint_ns > 0 else None
                        ),
                    },
                    "capture_config": {
                        "sync_slop_ms": args.sync_slop_ms,
                        "joint_max_age_ms": args.joint_max_age_ms,
                        "depth_scale_m": args.depth_scale_m,
                    },
                    "geometry_3d_ready": tf_msg is not None,
                    "sha256": {name: sha256_file(staging / name) for name in files},
                }
                write_json(staging / "manifest.json", manifest)
                validate_scene(staging)
                os.replace(staging, final_dir)
                self.done = True
            except Exception as exc:
                self.error = exc
                self.done = True

    rclpy.init()
    node = SnapshotNode()
    deadline = time.monotonic() + args.timeout_s
    try:
        while not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.error:
            raise node.error
        if not node.done:
            raise TimeoutError(
                "no valid synchronized snapshot before timeout; check camera, TF, and scale"
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if staging.exists():
            shutil.rmtree(staging)
    return final_dir


def main() -> int:
    args = parser().parse_args()
    try:
        output = run(args)
    except Exception as exc:
        print(f"capture failed: {exc}", file=sys.stderr)
        return 1
    print(json.dumps({"status": "captured", "scene_dir": str(output)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
