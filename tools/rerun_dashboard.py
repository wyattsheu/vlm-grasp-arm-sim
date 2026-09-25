#!/usr/bin/env python3
"""Four-window Rerun dashboard for the obstacle-avoidance demo
(docs/dev_guide_paper_core_and_dashboard_plan.md §8):

  window 1  world/points + world/obstacle_voxels   -- fused depth point cloud from the
                                                        wrist + rear cameras, plus cuRobo's
                                                        live-detected ESDF obstacle voxels
                                                        (from reactive.py, if it's running)
  window 2  cameras/overview                        -- fixed global view
  window 3  cameras/rear                             -- rear/pole camera
  window 4  cameras/wrist                            -- wrist ("top") camera
  + a bottom tab row: joints (commanded vs actual), contacts, status (scene/reactive JSON)

Pure ROS SUBSCRIBER -- never publishes, never sends a command, matching this repo's
existing "the dashboard doesn't control anything" convention (see research/scripts/
capture_scene.py's own docstring: "creates subscriptions ... deliberately contains no
publisher"). Run under env_robot129_realstack, which is layered on env_robot129_ros (so
it has BOTH rclpy and rerun_sdk together) -- see the dev guide for exact commands.

Binds only to localhost by default (AGENTS.md: opening a network service needs
confirmation first). View it via an SSH tunnel:
    ssh -L 9090:localhost:9090 -L 9876:localhost:9876 <this-machine>
then open http://localhost:9090/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A9876%2Fproxy
in a browser. This process being separate from Isaac
means it does not affect whether Isaac is *also* streaming its own interactive viewport
over WebRTC (port 49100) -- the two are independent and can run at the same time; see
the dev guide's §8.7 for the combined "WebRTC + dashboard" launch sequence.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.parse
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "research" / "src"))

from mpg.urdf_fk import ARM_JOINT_NAMES, UrdfChainFk, classify_approach_direction, rotate_vector_xyzw  # noqa: E402
from mpg.curobo_bridge.waypoints import load_waypoints_from_ab_poses, load_waypoints_from_yaml  # noqa: E402

URDF_PATH = ROOT / "ros2_ws" / "src" / "robot129_description" / "urdf" / "robot129.urdf"
ARM_NAMES = list(ARM_JOINT_NAMES)
ALL_NAMES = [f"joint{i}" for i in range(1, 9)]
DEFAULT_APPLICATION_ID = "robot129_demo"
BAR_CFG = json.loads((ROOT / "research/configs/scenes/official_dynamic_bar.json").read_text())


def quat_xyzw_to_matrix(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def decode_rgb_image(msg) -> np.ndarray:
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
    count = channels.get(msg.encoding, 3)
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    image = rows[:, : msg.width * count].reshape(msg.height, msg.width, count)[..., :3]
    if msg.encoding in ("bgr8", "bgra8"):
        image = image[..., ::-1]
    return np.ascontiguousarray(image)


def decode_depth_image(msg) -> np.ndarray:
    return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()


def decode_obstacle_voxels(msg) -> tuple[np.ndarray, np.ndarray]:
    """Matches mpg.ros_pointcloud.build_pointcloud2's exact field layout (x,y,z float32 +
    r,g,b uint8 + 1 pad byte, point_step=16) -- this decoder is only correct for messages
    that module produced, not a generic PointCloud2 reader.
    """
    n = msg.width
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)
    dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1"), ("_pad", "u1")])
    arr = np.frombuffer(msg.data, dtype=dtype, count=n)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=-1)
    rgb = np.stack([arr["r"], arr["g"], arr["b"]], axis=-1)
    return xyz, rgb


def deproject_depth_to_world(depth, rgb, k_flat, cam_pos, cam_quat_xyzw, stride, max_range_m):
    """Same pinhole convention as research/src/mpg/lifting.py's deproject() (X right,
    Y down, Z forward in the camera optical frame), vectorized over a strided pixel
    grid here for speed rather than calling that per-pixel -- not imported from there
    because deproject() is scalar-per-call, not built for a full image.
    """
    h, w = depth.shape
    ys, xs = np.mgrid[0:h:stride, 0:w:stride]
    d = depth[ys, xs]
    valid = np.isfinite(d) & (d > 0.05) & (d < max_range_m)
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)
    # Index down to the valid subset BEFORE the arithmetic, not after -- doing the
    # multiply on the full (inf/nan-containing) array first and masking afterward is
    # numerically harmless (those entries get discarded either way) but throws a
    # RuntimeWarning on every single call (confirmed live, 2026-09-24).
    fx, cx, fy, cy = k_flat[0], k_flat[2], k_flat[4], k_flat[5]
    xs_v, ys_v, d_v = xs[valid], ys[valid], d[valid]
    x = (xs_v - cx) * d_v / fx
    y = (ys_v - cy) * d_v / fy
    pts_cam = np.stack([x, y, d_v], axis=-1)
    rot = quat_xyzw_to_matrix(cam_quat_xyzw)
    pts_world = pts_cam @ rot.T + np.asarray(cam_pos, dtype=np.float64)
    colors = rgb[ys, xs][valid] if rgb is not None else np.full((pts_world.shape[0], 3), 180, dtype=np.uint8)
    return pts_world.astype(np.float32), colors


def build_blueprint() -> rrb.Blueprint:
    top = rrb.Grid(
        contents=[
            rrb.Spatial3DView(name="1. Goals & Sensed Obstacle (3D)", origin="world"),
            rrb.Spatial2DView(name="2. Global View", origin="cameras/overview"),
            rrb.Spatial2DView(name="3. Rear Camera", origin="cameras/rear"),
            rrb.Spatial2DView(name="4. Wrist Camera", origin="cameras/wrist"),
        ],
        grid_columns=2,
    )
    return rrb.Blueprint(top, collapse_panels=True)


class Dashboard:
    def __init__(self, recordings: list[rr.RecordingStream], point_stride: int, max_range_m: float,
                 waypoints_path: Path | None = None, obstacle: str = "none"):
        self.recordings = recordings
        self.point_stride = point_stride
        self.max_range_m = max_range_m
        self.urdf = UrdfChainFk(URDF_PATH)
        self.state = {
            "wrist_depth": None, "wrist_info": None, "wrist_pose": None, "wrist_rgb": None,
            "rear_depth": None, "rear_info": None, "rear_pose": None,
        }
        self.last_points_log_s = 0.0
        self.last_voxels_log_s = 0.0
        self.obstacle = obstacle
        self.bar_inserted = False
        self._show_demo_markers(waypoints_path)
        detection_text = (
            "Waiting for depth observations. Orange points are sensed obstacle samples; "
            "no ideal cone model is drawn."
            if obstacle == "cone_multi" else
            "Known collision geometry is used for planning. Gray/blue/green/red lines show "
            "baseline/planned/actual/replanned paths."
            if obstacle == "bar" else
            "Waiting for sensed obstacle voxels."
        )
        self._log_static("status/detection", rr.TextDocument(detection_text))

    def _log(self, path: str, *entities) -> None:
        for rec in self.recordings:
            rr.log(path, *entities, recording=rec)

    def _log_static(self, path: str, *entities) -> None:
        for rec in self.recordings:
            rr.log(path, *entities, static=True, recording=rec)

    def _show_demo_markers(self, waypoints_path: Path | None) -> None:
        if waypoints_path is not None:
            import yaml
            doc = yaml.safe_load(waypoints_path.read_text())
            waypoints = (load_waypoints_from_ab_poses(waypoints_path)
                         if isinstance(doc, dict) and doc.get("schema_version") == "demo_ab_poses_v1"
                         else load_waypoints_from_yaml(waypoints_path))
            positions = np.asarray([w.position_m for w in waypoints])
            vectors = np.asarray([rotate_vector_xyzw(w.quaternion_xyzw, [0, 0, 0.10]) for w in waypoints])
            # Rerun labels have a fixed screen-space font size. Short names avoid
            # covering the arm and obstacle while preserving target identity.
            labels = [w.name.split("_")[0] for w in waypoints]
            colors = [[0, 220, 90] if i == 0 else [255, 70, 70] if i == len(waypoints) - 1
                      else [40, 150, 255] for i in range(len(waypoints))]
            self._log_static("world/demo/targets", rr.Points3D(positions, labels=labels,
                             show_labels=True, colors=colors, radii=0.018))
            self._log_static("world/demo/approach", rr.Arrows3D(origins=positions,
                             vectors=vectors, colors=colors, radii=0.004))
            if len(positions) > 1:
                self._log_static("world/demo/target_order", rr.LineStrips3D([positions],
                                 colors=[[40, 150, 255]], radii=0.002))
        # Legacy cone demo still shows its known model. static_multi deliberately
        # omits this geometry: its orange obstacle points come only from RGB-D depth.
        if self.obstacle == "cone":
            manifest_path = "obstacle_cone_ab.json"
            manifest = json.loads((ROOT / "research/configs/scenes" / manifest_path).read_text())
            cone = manifest["objects"][0]
            x, y = cone["xy_m"]
            z, radius, height = cone["z_m"], cone["radius_m"], cone["height_m"]
            theta = np.linspace(0, 2 * np.pi, 25)
            base = np.column_stack((x + radius * np.cos(theta), y + radius * np.sin(theta),
                                    np.full(theta.shape, z - height / 2)))
            apex = np.array([x, y, z + height / 2])
            strips = [base] + [np.stack((base[i], apex)) for i in (0, 6, 12, 18)]
            cone_color = [230, 30, 200]
            self._log_static("world/demo/cone", rr.LineStrips3D(strips, colors=[cone_color], radii=0.004))
        legend = ("Targets: A/B/C/D. Orange: sensed obstacle depth. "
                  "Gray: direct path. Blue: planned. Green: actual. Contacts must remain 0 N."
                  if self.obstacle == "cone_multi" else
                  "Blue points: targets. Red bar: inserted obstacle. Gray/blue/green/red: "
                  "baseline/planned/actual/replanned paths. Contacts must remain 0 N."
                  if self.obstacle == "bar" else
                  "Green: start. Blue: intermediate. Red: end. Cyan: current gripper direction. "
                  "Orange: sensed occupied voxels. Gray: raw depth background.")
        self._log_static("status/legend", rr.TextDocument(legend))

    def _set_time(self, t_seconds: float) -> None:
        for rec in self.recordings:
            rr.set_time("sim_time", timestamp=t_seconds, recording=rec)

    # -- camera images (windows 2/3/4) ------------------------------------------------
    def on_wrist_rgb(self, msg):
        self._set_time(_stamp_s(msg.header.stamp))
        rgb = decode_rgb_image(msg)
        self.state["wrist_rgb"] = rgb
        self._log("cameras/wrist", rr.Image(rgb).compress(jpeg_quality=85))

    def on_rear_rgb(self, msg):
        self._set_time(_stamp_s(msg.header.stamp))
        self._log("cameras/rear", rr.Image(decode_rgb_image(msg)).compress(jpeg_quality=85))

    def on_overview_rgb(self, msg):
        self._set_time(_stamp_s(msg.header.stamp))
        self._log("cameras/overview", rr.Image(decode_rgb_image(msg)).compress(jpeg_quality=85))

    # -- depth + tf + info, fused into world/points (window 1) ------------------------
    def on_wrist_depth(self, msg):
        self.state["wrist_depth"] = decode_depth_image(msg)
        self._maybe_log_points()

    def on_wrist_info(self, msg):
        self.state["wrist_info"] = np.asarray(msg.k, dtype=np.float64)

    def on_rear_depth(self, msg):
        self.state["rear_depth"] = decode_depth_image(msg)
        self._maybe_log_points()

    def on_rear_info(self, msg):
        self.state["rear_info"] = np.asarray(msg.k, dtype=np.float64)

    def on_tf(self, msg):
        for t in msg.transforms:
            pose = ([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z],
                     [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
            if t.child_frame_id == "camera_color_optical_frame":
                self.state["wrist_pose"] = pose
            elif t.child_frame_id == "scene_camera_color_optical_frame":
                self.state["rear_pose"] = pose
            elif t.child_frame_id == "dynamic_stick" and self.obstacle in ("stick", "bar"):
                if self.obstacle == "bar" and pose[0][0] > 1.0:
                    continue  # parked offstage, before the insertion service
                if self.obstacle == "bar":
                    # The TF only gates when sensing should begin. Do not draw the
                    # authored box; the 3D view shows depth-derived points below.
                    self.bar_inserted = True
                else:
                    self._log("world/demo/stick", rr.Boxes3D(centers=[pose[0]], sizes=[[0.03, 0.03, 0.22]],
                              colors=[[230, 30, 200, 80]], labels=["MOVING ROD"], show_labels=False))

    def _maybe_log_points(self, min_period_s: float = 0.5):
        now = time.monotonic()
        if now - self.last_points_log_s < min_period_s:
            return
        pts_all, colors_all = [], []
        for depth_key, info_key, pose_key, rgb in (
            ("wrist_depth", "wrist_info", "wrist_pose", self.state["wrist_rgb"]),
            ("rear_depth", "rear_info", "rear_pose", None),
        ):
            depth, info, pose = self.state[depth_key], self.state[info_key], self.state[pose_key]
            if depth is None or info is None or pose is None:
                continue
            pts, colors = deproject_depth_to_world(depth, rgb, info, pose[0], pose[1], self.point_stride, self.max_range_m)
            if pts.shape[0]:
                pts_all.append(pts)
                colors_all.append(colors)
        if not pts_all:
            return
        self.last_points_log_s = now
        points = np.concatenate(pts_all)
        background = np.tile(np.array([140, 150, 160, 35], dtype=np.uint8), (len(points), 1))
        self._log("world/points", rr.Points3D(points, colors=background, radii=0.001))
        if self.obstacle == "cone_multi" or (self.obstacle == "bar" and self.bar_inserted):
            # Sensor-derived visualization only. The ROI selects depth returns near
            # the physical obstacle and rejects the floor; it does not draw or sample
            # the cuRobo collision geometry used internally for planning.
            if self.obstacle == "cone_multi":
                center = np.array([0.20, 0.39, 0.25])
                half = np.array([0.10, 0.11, 0.31])
            else:
                center = np.asarray(BAR_CFG["center_m"], dtype=float)
                half = np.asarray(BAR_CFG["size_m"], dtype=float) / 2 + np.array([0.04, 0.04, 0.04])
            mask = (np.all(np.abs(points - center) < half, axis=1) & (points[:, 2] > 0.04))
            detected = points[mask]
            orange = np.tile(np.array([255, 140, 0], dtype=np.uint8), (len(detected), 1))
            self._log("world/detected_obstacle", rr.Points3D(detected, colors=orange, radii=0.005))
            self._log("status/detection", rr.TextDocument(
                f"Detected obstacle depth points: {len(detected)}"))

    # -- reactive.py topics (window 1 continued + status tab) -------------------------
    def on_obstacle_voxels(self, msg):
        self._set_time(_stamp_s(msg.header.stamp))
        xyz, rgb = decode_obstacle_voxels(msg)
        if self.obstacle in ("cone", "stick", "cone_multi", "bar") and len(xyz):
            center = (np.array(BAR_CFG["center_m"]) if self.obstacle == "bar" else
                      np.array([0.20, 0.39, 0.25]) if self.obstacle == "cone_multi" else
                      np.array([0.2054028008, 0.3580777478, 0.11]))
            width_y = 0.23 if self.obstacle == "bar" else 0.16
            mask = (np.abs(xyz[:, 0] - center[0]) < 0.16) & (np.abs(xyz[:, 1] - center[1]) < width_y) & (xyz[:, 2] > 0.02) & (xyz[:, 2] < 0.60)
            xyz = xyz[mask]
        orange = np.tile(np.array([255, 140, 0], dtype=np.uint8), (len(xyz), 1))
        self._log("world/obstacle_voxels", rr.Points3D(xyz, colors=orange, radii=0.008))
        self._log("status/detection", rr.TextDocument(
            f"Sensed occupied voxels near obstacle: {len(xyz)}" if self.obstacle != "none"
            else f"Sensed occupied voxels: {len(xyz)}"))

    def on_goal(self, msg):
        self._set_time(_stamp_s(msg.header.stamp))
        p, q = msg.pose.position, msg.pose.orientation
        self._log(
            "world/goal",
            rr.Transform3D(
                translation=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w], axis_length=0.08,
            ),
        )

    def on_reactive_status(self, msg):
        try:
            doc = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._log("status/reactive", rr.TextDocument(json.dumps(doc, indent=2), media_type=rr.MediaType.MARKDOWN))

    def on_demo_path(self, msg):
        try:
            doc = json.loads(msg.data)
            points = np.asarray(doc["xyz_m"], dtype=float).reshape(-1, 3)
        except (ValueError, KeyError, TypeError):
            return
        if len(points) < 2:
            return
        kind = doc.get("kind", "planned")
        color = {"planned": [40, 165, 255], "actual": [0, 230, 100],
                 "before": [150, 150, 150], "after": [255, 70, 40]}.get(kind, [255, 255, 255])
        name = "".join(c if c.isalnum() or c == "_" else "_" for c in str(doc.get("label", kind)))
        self._log(f"world/demo/trajectories/{name}_{kind}", rr.LineStrips3D([points], colors=[color], radii=0.004))
        self._log("status/demo_path", rr.TextDocument(f"{name}: {kind}, {len(points)} points"))

    def on_demo_status(self, msg):
        self._log("status/demo", rr.TextDocument(msg.data, media_type=rr.MediaType.MARKDOWN))

    def on_scene_state(self, msg):
        try:
            doc = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._log("status/scene", rr.TextDocument(json.dumps(doc, indent=2), media_type=rr.MediaType.MARKDOWN))

    # -- arm pose in world/robot (joint values are not plotted) -----------------------
    def on_joint_states(self, msg):
        self._set_time(_stamp_s(msg.header.stamp))
        pos_by_name = dict(zip(msg.name, msg.position))
        if all(n in pos_by_name for n in ARM_NAMES):
            q6 = [pos_by_name[n] for n in ARM_NAMES]
            pos, quat = self.urdf.pinch_center_pose(q6)
            direction, angle_deg = classify_approach_direction(quat)
            self._log("world/robot/pinch_center", rr.Transform3D(translation=pos.tolist(), quaternion=quat.tolist(), axis_length=0.06))
            direction_vec = rotate_vector_xyzw(quat, [0, 0, 0.10])
            self._log("world/robot/approach", rr.Arrows3D(origins=[pos], vectors=[direction_vec],
                      colors=[[0, 230, 230]], labels=["EE"], show_labels=False))
            self._log("status/gripper", rr.TextDocument(f"approach ~= {direction} (off by {angle_deg:.1f} deg)\npinch_position_m = {np.round(pos, 4).tolist()}"))

def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--web-port", type=int, default=9090)
    parser.add_argument("--grpc-port", type=int, default=9876)
    parser.add_argument("--bind", default="localhost", help="AGENTS.md: keep this localhost and use an SSH tunnel, don't expose it directly")
    parser.add_argument("--open-browser", action="store_true", help="try to open a local browser (usually pointless on a headless server -- use the SSH tunnel instead)")
    parser.add_argument("--save-rrd", type=Path, default=None, help="also save a .rrd recording; default out/demo/dashboard/<timestamp>/dashboard.rrd")
    parser.add_argument("--no-save", action="store_true", help="live-serve only, don't write a .rrd file")
    parser.add_argument("--point-stride", type=int, default=4, help="pixel stride when deprojecting depth to world points (higher = fewer points, faster)")
    parser.add_argument("--max-range-m", type=float, default=2.5)
    parser.add_argument("--duration-s", type=float, default=None, help="exit after this many wall-clock seconds; default runs until Ctrl-C")
    parser.add_argument("--waypoints", type=Path, default=None, help="show labeled 6D goal points and approach arrows")
    parser.add_argument("--obstacle", choices=["none", "cone", "stick", "cone_multi", "bar"], default="none")
    args = parser.parse_args()

    if args.bind not in ("localhost", "127.0.0.1"):
        print("REFUSED: --bind must be localhost/127.0.0.1 -- use an SSH tunnel to view remotely (see module docstring), "
              "not a directly-exposed port (AGENTS.md: opening an external network service needs confirmation first).", flush=True)
        return 2

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    # NOT recordings[0].serve_web(...) (the one-call convenience wrapper): that call
    # doesn't hand back the gRPC connect URI, and opening the bare http://host:port/ it
    # implies (with open_browser=False, which this script always uses) shows the web
    # viewer's own "no recording connected" welcome screen instead of this dashboard --
    # confirmed live 2026-09-24 (a person opened exactly that bare URL and saw the
    # generic "Visualize multimodal data / Log data with the Rerun SDK..." landing page,
    # not any window/pane from this dashboard). serve_grpc() + serve_web_viewer() are the
    # same two calls serve_web() makes internally, done separately here so the returned
    # URI can be embedded in the browser URL as `?url=<connect-uri>` -- confirmed by
    # reading the served viewer HTML's own inline script (it parses exactly
    # `new URLSearchParams(window.location.search).getAll("url")`; no query param means
    # no connection means the welcome screen).
    live_recording = rr.RecordingStream(application_id=DEFAULT_APPLICATION_ID)
    connect_uri = live_recording.serve_grpc(grpc_port=args.grpc_port, default_blueprint=build_blueprint())
    rr.serve_web_viewer(web_port=args.web_port, open_browser=args.open_browser, connect_to=connect_uri)
    recordings = [live_recording]
    viewer_url = f"http://{args.bind}:{args.web_port}/?url={urllib.parse.quote(connect_uri, safe='')}"
    print(f"[rerun_dashboard] serving web viewer -- open this exact URL (not the bare host:port): {viewer_url}", flush=True)

    if not args.no_save:
        save_path = args.save_rrd or (ROOT / "out" / "demo" / "dashboard" / time.strftime("%Y%m%d_%H%M%S") / "dashboard.rrd")
        save_path.parent.mkdir(parents=True, exist_ok=True)
        file_rec = rr.RecordingStream(application_id=DEFAULT_APPLICATION_ID)
        file_rec.save(str(save_path))
        recordings.append(file_rec)
        print(f"[rerun_dashboard] also saving to {save_path}", flush=True)

    dashboard = Dashboard(recordings, args.point_stride, args.max_range_m, args.waypoints, args.obstacle)

    rclpy.init()
    node = Node("rerun_dashboard", namespace="/robot129_sim")
    sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    latched_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    node.create_subscription(Image, "/robot129_sim/camera/color/image_raw", dashboard.on_wrist_rgb, sensor_qos)
    node.create_subscription(Image, "/robot129_sim/camera/aligned_depth_to_color/image_raw", dashboard.on_wrist_depth, sensor_qos)
    node.create_subscription(CameraInfo, "/robot129_sim/camera/aligned_depth_to_color/camera_info", dashboard.on_wrist_info, sensor_qos)
    node.create_subscription(Image, "/robot129_sim/scene_camera/color/image_raw", dashboard.on_rear_rgb, sensor_qos)
    node.create_subscription(Image, "/robot129_sim/scene_camera/aligned_depth_to_color/image_raw", dashboard.on_rear_depth, sensor_qos)
    node.create_subscription(CameraInfo, "/robot129_sim/scene_camera/aligned_depth_to_color/camera_info", dashboard.on_rear_info, sensor_qos)
    node.create_subscription(Image, "/robot129_sim/overview_camera/color/image_raw", dashboard.on_overview_rgb, sensor_qos)
    node.create_subscription(TFMessage, "/tf", dashboard.on_tf, 10)
    node.create_subscription(JointState, "/robot129_sim/joint_states", dashboard.on_joint_states, 10)
    node.create_subscription(String, "/robot129_sim/scene_state", dashboard.on_scene_state, latched_qos)
    node.create_subscription(PointCloud2, "/robot129_sim/reactive/obstacle_voxels", dashboard.on_obstacle_voxels, sensor_qos)
    node.create_subscription(PoseStamped, "/robot129_sim/reactive/goal", dashboard.on_goal, sensor_qos)
    node.create_subscription(String, "/robot129_sim/reactive/status", dashboard.on_reactive_status, sensor_qos)
    node.create_subscription(String, "/robot129_sim/demo/path", dashboard.on_demo_path, 10)
    node.create_subscription(String, "/robot129_sim/demo/status", dashboard.on_demo_status, 10)
    print("[rerun_dashboard] subscribed, spinning (Ctrl-C to stop)", flush=True)
    t_start = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if args.duration_s is not None and time.monotonic() - t_start > args.duration_s:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("[rerun_dashboard] stopped", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
