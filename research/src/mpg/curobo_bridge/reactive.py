"""Live reactive obstacle avoidance for robot129, following the method in cuRobo's own
``curobo/examples/reference/live_volumetric_mapping_mpc.py`` reference example: fuse
live depth into a TSDF, compute an ESDF, and run cuRobo's MPC against it every control
tick so the arm reacts to obstacles it has never been told about in advance.

Architecture note -- a deliberate deviation from "lockstep inside the Isaac process":
the approved plan's first draft assumed this would run inside the same Python process as
sim/scripts/run_robot129_ros_webrtc.py (avoiding ROS transport latency). Building that
required either installing cuRobo into the shared IsaacLab venv (real risk: other
projects on this machine depend on its pinned torch/warp versions, and AGENTS.md already
forbids touching it without asking) or a `--target`-installed PYTHONPATH injection that
would still run inside Isaac's own process and could destabilize it in ways hard to test
in isolation. Instead this runs as an ORDINARY separate ROS 2 node, in
env_robot129_curobo, subscribing to the existing camera/joint_states topics and
publishing to the existing streaming command topic
(/robot129_sim/piper/joint_cmd -- see handle_piper_joint_cmd() in
sim/scripts/run_robot129_ros_webrtc.py: "no trajectory interpolation ... a streamed
command supersedes any active JointTrajectory plan", exactly what MPC output needs).
This matches every other process boundary already in this repo (Isaac / ROS / research
venvs are always separate processes talking over ROS or files, never sharing an
interpreter) and is also a closer match to how this would eventually run against real
hardware (a perception+planning process talking to a robot driver over ROS), which the
plan already flagged as the natural next step. The cost is ROS pub/sub latency instead of
zero-copy tensors; see docs/progress/curobo_step4_reactive.md for the measured impact.

Must run under env_robot129_curobo (needs `curobo`), NOT env_robot129_research or the
IsaacLab venv. See docs/dev_guide_paper_core_and_dashboard_plan.md §7.8 for exact
commands.

## A real, non-obvious cuRobo bug found while building this (fixed here, not upstream)

``curobo.types.CameraObservation.depth_to_meter`` defaults to ``0.001`` (i.e. it assumes
raw millimetre depth, the common RealSense uint16 convention). robot129's sim
already publishes metre-scale ``32FC1`` depth (see ``publish_camera_frame`` in
sim/scripts/run_robot129_ros_webrtc.py). Leaving the default silently multiplies every
depth value by 0.001, collapsing all deprojected points to within millimetres of the
camera. This produced two different, equally wrong, silent failures during development
(caught 2026-09-24 by cross-checking against a hand-labelled real frame, not by inspecting
this field directly -- see docs/progress/curobo_step4_reactive.md for the exact repro):
the wrist camera (mounted centimetres from the gripper) flagged 100% of pixels as
"robot"; the scene camera (~0.4 m away) flagged 0%. Every ``CameraObservation`` built in
this module passes ``depth_to_meter=1.0`` explicitly -- see ``_camera_observation()``.

## Other rough edges found and worked around here

- ``RobotSegmenter.from_robot_file()`` resolves its path against cuRobo's own bundled
  ``content/configs/robot/`` directory, not the CWD or an absolute path -- use
  ``RobotSegmenter(Kinematics(...), ops_dtype=torch.float32)`` directly instead (see
  below for the second reason this constructor is needed).
- ``RobotSegmenter``'s default ``ops_dtype=torch.bfloat16`` crashes immediately
  (``robot_spheres: expected dtype torch.float32, got torch.bfloat16``) against
  robot129.yml's float32 collision spheres -- pass ``ops_dtype=torch.float32`` explicitly.

## Extracting obstacle points for visualization (dashboard --scene-camera pole window 1)

``VoxelGrid.get_occupied_voxels()`` (the method on the object ``Mapper.compute_esdf()``
returns) is NOT what to call here: it needs ``.xyzr_tensor`` populated, which
``compute_esdf()`` leaves ``None``, and even after calling ``create_xyzr_tensor()``
yourself, ``feature_tensor`` comes back as the unflattened ``(nx, ny, nz)`` grid, so
``get_occupied_voxels()``'s internal ``xyzr[:, 3] = self.feature_tensor`` raises a shape
mismatch (confirmed empirically, not from reading the source alone -- see
docs/dev_guide_paper_core_and_dashboard_plan.md). The dense grid also always contains a
``10000.0`` sentinel for never-observed voxels (standard cuRobo ESDF sign convention:
negative = inside an obstacle, positive = free/outside, confirmed against
``curobo/examples/getting_started/volumetric_mapping.py``'s own slice visualization,
"blue=inside, white=zero, red=outside"), so a naive threshold on the dense grid would
paint the ENTIRE unobserved region as "occupied" -- exactly backwards for a dashboard
meant to show what the camera has actually detected.

The correct call, matching what cuRobo's own ``volumetric_mapping.py`` example uses for
its point-cloud visualization, is the block-sparse TSDF integrator's own extraction:
``mapper.integrator.extract_occupied_voxels(surface_only=...)`` -> ``(centers, colors)``,
wrapped here as ``LiveEsdfReactiveController.occupied_voxels()``. It only returns voxels
the integrator actually allocated (i.e. actually observed near a surface), so there is no
sentinel-filtering to get wrong, and an empty scene correctly returns empty tensors rather
than a giant false-positive blob.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO_ROOT / "research" / "src"))

import numpy as np
import torch

from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.model_predictive_control import ModelPredictiveControl, ModelPredictiveControlCfg
from curobo.perception import FilterDepth, Mapper, MapperCfg, RobotSegmenter
from curobo.scene import Cuboid, Scene
from curobo.types import CameraObservation, GoalToolPose, JointState, Pose

from mpg.curobo_bridge.waypoints import WaypointSequencer, load_waypoints_from_ab_poses, load_waypoints_from_yaml
from mpg.ros_pointcloud import build_pointcloud2
from mpg.urdf_fk import UrdfChainFk, quaternion_angular_distance_xyzw

DEFAULT_ROBOT_CONFIG = REPO_ROOT / "research" / "configs" / "curobo" / "robot129.yml"
DEFAULT_URDF = REPO_ROOT / "ros2_ws" / "src" / "robot129_description" / "urdf" / "robot129.urdf"
ARM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# sim/scripts/run_robot129_ros_webrtc.py steps physics at dt=1/120 and publishes
# /robot129_sim/joint_states once every 4 physics frames (`if frame % 4 == 0`), so each
# joint_states message is exactly 4/120 s of SIM time regardless of how fast or slow the
# simulator is running in wall-clock terms. reactive_node() uses the message count as its
# clock and runs one MPC step (of this same optimization_dt) per message -- see its
# docstring's "Lockstep with sim time" section for why.
SIM_DT_PER_JOINT_STATE = 4.0 / 120.0


def xyzw_to_wxyz(quat_xyzw) -> list[float]:
    x, y, z, w = quat_xyzw
    return [w, x, y, z]


def _camera_observation(depth_m: np.ndarray, rgb: np.ndarray | None, intrinsics_3x3: np.ndarray,
                         position_xyz, quaternion_xyzw, device: str) -> CameraObservation:
    """Build a batched (leading camera dim) CameraObservation with depth already in
    metres -- see module docstring's depth_to_meter warning. Every camera-observation
    construction in this module MUST go through this function, not CameraObservation()
    directly, so that fix can't be silently missed in a second call site.
    """
    depth = torch.from_numpy(np.ascontiguousarray(depth_m)).to(device=device, dtype=torch.float32)
    depth = torch.nan_to_num(depth, nan=5.0, posinf=5.0, neginf=0.0).unsqueeze(0)
    if rgb is None:
        rgb = np.zeros((*depth_m.shape, 3), dtype=np.uint8)
    rgb_t = torch.from_numpy(np.ascontiguousarray(rgb)).to(device=device, dtype=torch.uint8).unsqueeze(0)
    intrinsics = torch.from_numpy(np.ascontiguousarray(intrinsics_3x3)).to(device=device, dtype=torch.float32).unsqueeze(0)
    quat_wxyz = xyzw_to_wxyz(quaternion_xyzw)
    pose = Pose(
        position=torch.tensor([position_xyz], device=device, dtype=torch.float32),
        quaternion=torch.tensor([quat_wxyz], device=device, dtype=torch.float32),
    )
    return CameraObservation(
        name="camera", depth_image=depth, rgb_image=rgb_t, pose=pose, intrinsics=intrinsics,
        depth_to_meter=1.0,
    )


class LiveEsdfReactiveController:
    """Segment -> integrate -> ESDF -> MPC, one camera frame and one control tick at a
    time. No ROS/Isaac dependency (see reactive_node() below for the ROS wrapper) so this
    class alone can be unit-tested with synthetic or replayed real frames -- see
    research/tests/test_reactive_controller.py.
    """

    def __init__(
        self,
        robot_config: Path = DEFAULT_ROBOT_CONFIG,
        grid_center=(0.3, 0.0, 0.3),
        extent_xyz=(1.2, 1.2, 1.0),
        voxel_size: float = 0.02,
        esdf_voxel_size: float = 0.03,
        distance_threshold: float = 0.05,
        known_cuboids: list[Cuboid] | None = None,
        device: str = "cuda",
        optimization_dt: float = SIM_DT_PER_JOINT_STATE,
    ):
        self.device = device
        self.robot_config = str(robot_config)
        self.optimization_dt = optimization_dt
        self._ik = None  # built lazily by solve_goal_ik()
        kin_cfg = KinematicsCfg.from_robot_yaml_file(self.robot_config)
        self._kinematics = Kinematics(kin_cfg)
        # ops_dtype=torch.float32 -- see module docstring, default bfloat16 crashes.
        self._segmenter = RobotSegmenter(self._kinematics, distance_threshold=distance_threshold, ops_dtype=torch.float32)
        self._mapper = Mapper(MapperCfg(
            voxel_size=voxel_size, esdf_voxel_size=esdf_voxel_size,
            extent_meters_xyz=tuple(extent_xyz), extent_esdf_meters_xyz=tuple(extent_xyz),
            grid_center=torch.tensor(list(grid_center), dtype=torch.float32),
            truncation_distance=voxel_size * 4.0, depth_minimum_distance=0.15, depth_maximum_distance=3.0,
            minimum_tsdf_weight=1.0, decay_factor=0.3, num_cameras=1, device=device,
        ))
        self._filter_depth: FilterDepth | None = None  # built lazily, needs image_shape
        self._known_cuboids = known_cuboids or []
        self._mpc: ModelPredictiveControl | None = None
        self._voxel_grid = None
        self._current_state: JointState | None = None
        self._integrated_frame_count = 0

    def integrate_camera(self, depth_m: np.ndarray, rgb: np.ndarray | None, intrinsics_3x3: np.ndarray,
                          position_xyz, quaternion_xyzw, joint_state_6) -> None:
        """One camera frame: segment the robot's own body out of the depth (using
        joint_state_6, the CURRENT arm joint values, not the pose the frame happened to
        be rendered at -- an MPC control loop's segmentation must track the live state,
        unlike a one-shot capture), filter, integrate into the running TSDF.
        """
        obs = _camera_observation(depth_m, rgb, intrinsics_3x3, position_xyz, quaternion_xyzw, self.device)
        js = JointState.from_position(
            torch.tensor([joint_state_6], device=self.device, dtype=torch.float32), joint_names=ARM_JOINT_NAMES,
        )
        _mask, filtered_depth = self._segmenter.get_robot_mask(obs, js)
        if self._filter_depth is None:
            self._filter_depth = FilterDepth(image_shape=tuple(depth_m.shape), device=self.device)
        filtered_depth, _valid = self._filter_depth(filtered_depth)
        obs_filtered = CameraObservation(
            name=obs.name, depth_image=filtered_depth, rgb_image=obs.rgb_image, pose=obs.pose,
            intrinsics=obs.intrinsics, depth_to_meter=1.0,
        )
        self._mapper.integrate(obs_filtered)
        self._integrated_frame_count += 1

    def compute_esdf(self):
        self._voxel_grid = self._mapper.compute_esdf()
        return self._voxel_grid

    def occupied_voxels(self, surface_only: bool = True) -> tuple[np.ndarray, np.ndarray]:
        """World-frame occupied voxel centers ``[N, 3]`` and per-voxel colors
        ``[N, 3]`` (uint8) -- see module docstring's "Extracting obstacle
        points" section for why this goes through
        ``mapper.integrator.extract_occupied_voxels()`` rather than
        ``VoxelGrid.get_occupied_voxels()``. Call after at least one
        ``compute_esdf()``. Returns two ``(0, 3)`` arrays, not an error, when
        nothing has been observed yet -- callers (reactive_node()'s
        PointCloud2 publisher) should treat that as "nothing to draw this
        tick", not a failure.
        """
        centers, colors = self._mapper.integrator.extract_occupied_voxels(surface_only=surface_only)
        if centers is None or centers.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.uint8)
        return centers.detach().float().cpu().numpy(), colors.detach().cpu().numpy().astype(np.uint8)

    def ensure_mpc(self) -> ModelPredictiveControl:
        """Build the MPC solver the first time it's needed -- must happen AFTER at
        least one compute_esdf() call, matching cuRobo's own reference example
        (LiveEsdfMpc is constructed only once the first voxel_grid exists). Subsequent
        compute_esdf() calls refresh the SAME aliased tensor the MPC's collision
        checker already references (this is cuRobo's own documented mechanism, not
        something this module manages) -- so ensure_mpc() is only ever called once.
        """
        if self._mpc is not None:
            return self._mpc
        if self._voxel_grid is None:
            raise RuntimeError("call compute_esdf() at least once before ensure_mpc()")
        scene = Scene(voxel=[self._voxel_grid], cuboid=self._known_cuboids)
        cfg = ModelPredictiveControlCfg.create(
            robot=self.robot_config, scene_model=scene, use_cuda_graph=True,
            optimization_dt=self.optimization_dt, interpolation_steps=4, optimizer_collision_activation_distance=0.01,
        )
        self._mpc = ModelPredictiveControl(cfg)
        current_state = JointState.from_position(
            self._mpc.default_joint_position.clone().unsqueeze(0), joint_names=self._mpc.joint_names,
        )
        current_state.velocity = torch.zeros_like(current_state.position)
        current_state.acceleration = torch.zeros_like(current_state.position)
        self._mpc.setup(current_state)
        self._current_state = current_state
        return self._mpc

    @property
    def tool_frame(self) -> str:
        return self.ensure_mpc().tool_frames[0]

    def sync_state(self, joint_state_6, joint_velocity_6=None) -> None:
        """Overwrite the internal warm-start state with a freshly measured one (e.g.
        after a ROS joint_states message arrives) instead of relying purely on the
        previous optimize_action_sequence() output -- keeps MPC tracking the real robot
        instead of drifting from it under ROS latency.
        """
        mpc = self.ensure_mpc()
        pos = torch.tensor([joint_state_6], device=self.device, dtype=torch.float32)
        vel = torch.tensor([joint_velocity_6], device=self.device, dtype=torch.float32) if joint_velocity_6 is not None else torch.zeros_like(pos)
        self._current_state = JointState.from_position(pos, joint_names=mpc.joint_names)
        self._current_state.velocity = vel
        self._current_state.acceleration = torch.zeros_like(pos)

    def set_goal(self, goal_position_xyz, goal_quaternion_xyzw) -> bool:
        mpc = self.ensure_mpc()
        mpc.enable_tool_pose_tracking()
        # np.asarray(...) before torch.tensor(): goal_position_xyz is a numpy array (e.g.
        # a waypoints.Waypoint.position_m), and wrapping a numpy array directly in a
        # Python list before torch.tensor() hits torch's slow per-element construction
        # path -- confirmed live 2026-09-24 (UserWarning: "Creating a tensor from a list
        # of numpy.ndarrays is extremely slow"), harmless to correctness but wasteful
        # every single control tick.
        target_pose = Pose(
            position=torch.tensor(np.asarray([goal_position_xyz], dtype=np.float32), device=self.device, dtype=torch.float32),
            quaternion=torch.tensor(np.asarray([xyzw_to_wxyz(goal_quaternion_xyzw)], dtype=np.float32), device=self.device, dtype=torch.float32),
        )
        goal_tool_poses = GoalToolPose.from_poses(
            {self.tool_frame: target_pose}, ordered_tool_frames=mpc.tool_frames, num_goalset=1,
        )
        return bool(mpc.update_goal_tool_poses(goal_tool_poses, run_ik=False))

    def solve_goal_ik(self, goal_position_xyz, goal_quaternion_xyzw, near_q, return_seeds: int = 16) -> list[float] | None:
        """Global (many-seed) IK for a goal pose, returning the successful solution
        closest in joint space to `near_q` (the configuration the arm is in now), or
        None. Collision world = the known cuboids only (floor, pole), NOT the live ESDF:
        the goal configuration only has to be valid at the goal, and live-obstacle
        avoidance on the way there is the MPC's job.

        Why this exists: a Cartesian-only MPC goal is a local optimization, and from HOME
        it slides into whichever IK branch is nearest -- for the demo A/B poses that
        branch pins joint5 at its -1.22 rad limit and stalls 5 cm (A) / 24 cm (B) short,
        forever. Confirmed 2026-09-24 with a pure kinematic rollout (no Isaac, no
        latency), so it is the goal formulation, not the plumbing. MPC's own
        update_goal_tool_poses(run_ik=True) doesn't help: its IK is seeded ONLY from the
        current state (seed_config = current_state, one seed), which finds that same
        near-but-wrong branch.
        """
        if self._ik is None:
            self._ik = InverseKinematics(InverseKinematicsCfg.create(
                robot=self.robot_config, scene_model=Scene(cuboid=self._known_cuboids),
                num_seeds=64, position_tolerance=0.003, orientation_tolerance=0.03,
            ))
        pose = Pose(
            position=torch.tensor(np.asarray([goal_position_xyz], dtype=np.float32), device=self.device),
            quaternion=torch.tensor(np.asarray([xyzw_to_wxyz(goal_quaternion_xyzw)], dtype=np.float32), device=self.device),
        )
        tool = self._ik.tool_frames[0]
        result = self._ik.solve_pose(GoalToolPose.from_poses({tool: pose}, ordered_tool_frames=[tool]), return_seeds=return_seeds)
        success = result.success.reshape(-1).detach().cpu().numpy()
        solutions = result.solution.reshape(-1, len(ARM_JOINT_NAMES)).detach().cpu().numpy()
        if not success.any():
            return None
        candidates = solutions[success]
        best = candidates[np.argmin(np.abs(candidates - np.asarray(near_q, dtype=np.float64)[None, :]).sum(axis=1))]
        return best.tolist()

    def set_waypoint_goal(self, goal_position_xyz, goal_quaternion_xyzw, near_q, joint_goal=None) -> list[float] | None:
        """Set a new target ONCE per waypoint (not every tick): select a joint-space
        goal (`joint_goal`, or solve_goal_ik() nearest to `near_q`) and enable MPC
        joint-position tracking, which keeps the MPC in the chosen IK branch. The
        measured 6D FK pose remains the waypoint acceptance criterion.
        Kinematic rollout check (2026-09-24): HOME->A and A->B both converge to within
        0.002 rad of the IK solution in ~50 MPC steps (1.7 s sim), including the 3.5 rad
        joint1 swing between A and B. Returns the joint goal used, or None if IK failed
        (then only the Cartesian goal is set -- the old, local behaviour).
        """
        mpc = self.ensure_mpc()
        if joint_goal is None:
            joint_goal = self.solve_goal_ik(goal_position_xyz, goal_quaternion_xyzw, near_q)
        if joint_goal is None:
            # No usable IK solution: retain the old Cartesian-only fallback.
            self.set_goal(goal_position_xyz, goal_quaternion_xyzw)
            return None
        goal_js = JointState.from_position(
            torch.tensor([list(joint_goal)], device=self.device, dtype=torch.float32), joint_names=mpc.joint_names,
        )
        mpc.update_goal_state(goal_js)
        # Once an IK solution is selected, tracking both its joint state and the tool
        # pose can make two local costs pull toward different IK branches.  Track the
        # chosen joint state only; the waypoint sequencer still evaluates the measured
        # pinch-center 6D FK error before it declares the point reached.
        mpc.disable_tool_pose_tracking()
        mpc.enable_joint_position_tracking()
        return list(joint_goal)

    def current_position(self) -> list[float]:
        """The MPC's own rollout joint positions (what it last commanded)."""
        return self._current_state.position.reshape(-1).detach().cpu().tolist()

    def step(self) -> list[float] | None:
        """One MPC control step against the CURRENT (already-integrated) ESDF and the
        last goal set via set_goal(). Returns the next 6 arm joint positions, or None if
        the solver produced no usable action (caller should hold the previous command,
        not command zeros).
        """
        mpc = self.ensure_mpc()
        result = mpc.optimize_action_sequence(self._current_state)
        if result.action_sequence is None or result.action_sequence.position.shape[1] == 0:
            return None
        next_position = result.action_sequence.position[:, -1, :]
        self._current_state = JointState.from_position(next_position.clone(), joint_names=mpc.joint_names)
        self._current_state.velocity = result.action_sequence.velocity[:, -1, :]
        self._current_state.acceleration = result.action_sequence.acceleration[:, -1, :]
        return next_position.squeeze(0).detach().cpu().tolist()


def _known_world_cuboids(include_pedestal: bool, scene_geometry: dict, include_pole: bool, scene_camera: dict | None) -> list[Cuboid]:
    """Same known-geometry cuboids plan_grasp.py's _build_scene() uses, duplicated in
    miniature here rather than imported: plan_grasp.py's version also handles the
    pick_place_counter pedestal selection logic that doesn't apply to a live reactive
    session (there's no "scene name" here, just whatever's actually in front of the
    camera), so sharing the exact function would drag in unrelated arguments.
    """
    cuboids = [Cuboid(name="floor", pose=[0.0, 0.0, -0.025, 1.0, 0.0, 0.0, 0.0], dims=[4.0, 4.0, 0.05])]
    if include_pole and scene_camera is not None:
        pole = scene_camera["pole_camera"]
        ox, oy, oz = pole["offset_from_base_m"]
        half = pole["pole"]["half_extent_m"]
        cuboids.append(Cuboid(name="scene_pole", pose=[ox, oy, oz / 2.0, 1.0, 0.0, 0.0, 0.0], dims=[half * 2.0, half * 2.0, oz]))
    return cuboids


def reactive_node() -> int:
    """ROS 2 node: subscribes to the scene camera's depth/info + /tf + joint_states,
    runs LiveEsdfReactiveController, publishes to /robot129_sim/piper/joint_cmd. See
    module docstring for why this is a separate process rather than running inside
    Isaac. Only the scene (pole) camera is wired up in this first version -- the wrist
    camera could be added the same way (a second integrate_camera() call per tick) but
    is left out here: it is mounted centimetres from the gripper, so at HOME most of its
    view is the robot's own hardware even after segmentation, and validating that
    second camera's contribution needs its own real-frame check (see
    docs/progress/curobo_step4_reactive.md) which hasn't been done yet.

    Two goal-driving modes:
    - Default (no --waypoints): the original --goal-a/--goal-b pair, alternated purely
      by --switch-period-s wall-clock timer, both sharing --goal-quaternion-xyzw. This
      is UNCHANGED from before this option existed.
    - --waypoints <path>: drives a mpg.curobo_bridge.waypoints.WaypointSequencer
      instead -- advances to the next waypoint only once position/orientation error is
      within tolerance and held, or a per-waypoint timeout elapses (see that module).
      <path> is either a dedicated waypoints.yaml (mpg.curobo_bridge.waypoints.
      load_waypoints_from_yaml's schema) or a research/scripts/select_demo_poses.py
      ab_poses.yaml (auto-detected via its schema_version field, so both the p2p_cone
      demo and tools/send_joint_cmd.py's --sequence can point at the SAME file). Without
      --loop, the node stops itself once the sequencer reports all_done, and writes a
      reactive_report.json (see --report-out).

    New publishers (all namespaced under /robot129_sim/reactive/, all best-effort,
    depth-1 QoS -- these are dashboard/debug topics, not control-critical):
    - goal (geometry_msgs/PoseStamped): the currently active target pose, world frame.
    - status (std_msgs/String, JSON): waypoint name/phase/errors (if --waypoints) or
      goal_index (if not), plus running stats -- what tools/rerun_dashboard.py's status
      panel reads.
    - obstacle_voxels (sensor_msgs/PointCloud2): LiveEsdfReactiveController.
      occupied_voxels(), throttled to roughly 2 Hz wall-clock -- what dashboard window 1
      draws as the live-detected obstacle model. See mpg.ros_pointcloud.build_pointcloud2's
      docstring for the exact (non-PCL-packed) field layout.
    """
    import json

    import rclpy
    import yaml
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image, JointState as RosJointState, PointCloud2
    from std_msgs.msg import Header, String as RosString
    from tf2_msgs.msg import TFMessage

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_ROBOT_CONFIG)
    parser.add_argument("--goal-a", type=float, nargs=3, default=[0.30, 0.15, 0.35])
    parser.add_argument("--goal-b", type=float, nargs=3, default=[0.30, -0.15, 0.35])
    parser.add_argument("--goal-quaternion-xyzw", type=float, nargs=4, default=[0.0, 1.0, 0.0, 0.0])
    parser.add_argument("--switch-period-s", type=float, default=8.0)
    parser.add_argument("--control-rate-hz", type=float, default=5.0, help="DEPRECATED, ignored: the loop now steps once per /joint_states (sim time)")
    parser.add_argument("--esdf-every-n-frames", type=int, default=1, help="DEPRECATED, ignored: use --esdf-period-s")
    parser.add_argument("--include-pole", action="store_true")
    parser.add_argument("--grid-center", type=float, nargs=3, default=[0.3, 0.0, 0.3])
    parser.add_argument("--extent", type=float, nargs=3, default=[1.2, 1.2, 1.0])
    parser.add_argument("--duration-s", type=float, default=None, help="exit after this many seconds (sim time via wall clock); default runs until Ctrl-C")
    parser.add_argument("--waypoints", type=Path, default=None, help="waypoints.yaml or an ab_poses.yaml from select_demo_poses.py -- see reactive_node()'s docstring")
    parser.add_argument("--loop", action="store_true", help="only meaningful with --waypoints: keep cycling instead of stopping once all waypoints are reached")
    parser.add_argument("--waypoint-hold-s", type=float, default=1.5, help="only used for an ab_poses.yaml input, which has no per-point hold_s of its own")
    parser.add_argument("--waypoint-timeout-s", type=float, default=20.0, help="only used for an ab_poses.yaml input, which has no per-point timeout_s of its own")
    parser.add_argument("--position-tolerance-m", type=float, default=0.01)
    parser.add_argument("--orientation-tolerance-rad", type=float, default=0.0872665, help="~5 deg")
    parser.add_argument("--report-out", type=Path, default=None, help="only used with --waypoints; default out/demo/reactive/<timestamp>/reactive_report.json")
    parser.add_argument("--resync-threshold-rad", type=float, default=0.3,
                        help="DEPRECATED, ignored: command tracking now gates every MPC advance")
    parser.add_argument("--esdf-period-s", type=float, default=0.25, help="sim seconds between depth integrations / ESDF updates")
    parser.add_argument("--max-catchup-steps", type=int, default=3,
                        help="DEPRECATED, ignored: MPC advances only after Isaac tracks the prior command")
    parser.add_argument("--command-tolerance-rad", type=float, default=0.02,
                        help="advance MPC once every measured joint is this close to the previous command")
    parser.add_argument("--no-joint-goal", action="store_true",
                        help="debug: Cartesian-only goals (the old behaviour that stalls in the wrong IK branch)")
    args = parser.parse_args(sys.argv[1:])

    scene_geometry = json.loads((REPO_ROOT / "research" / "configs" / "scene_geometry.json").read_text())
    scene_camera_cfg = None
    if args.include_pole:
        scene_camera_cfg = yaml.safe_load((REPO_ROOT / "research" / "configs" / "scene_camera.yaml").read_text())
    cuboids = _known_world_cuboids(False, scene_geometry, args.include_pole, scene_camera_cfg)

    controller = LiveEsdfReactiveController(robot_config=args.robot_config, grid_center=tuple(args.grid_center), extent_xyz=tuple(args.extent), known_cuboids=cuboids)

    sequencer = None
    if args.waypoints is not None:
        peek = yaml.safe_load(args.waypoints.read_text())
        if isinstance(peek, dict) and peek.get("schema_version") == "demo_ab_poses_v1":
            waypoints = load_waypoints_from_ab_poses(args.waypoints, hold_s=args.waypoint_hold_s, timeout_s=args.waypoint_timeout_s)
        else:
            waypoints = load_waypoints_from_yaml(args.waypoints)
        sequencer = WaypointSequencer(
            waypoints, position_tolerance_m=args.position_tolerance_m,
            orientation_tolerance_rad=args.orientation_tolerance_rad, loop=args.loop,
        )
        print(f"[curobo_reactive] loaded {len(waypoints)} waypoint(s) from {args.waypoints}: {[w.name for w in waypoints]}", flush=True)
    urdf_chain = UrdfChainFk(DEFAULT_URDF)

    rclpy.init()
    node = Node("curobo_reactive_controller", namespace="/robot129_sim")
    sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

    state = {"depth": None, "info": None, "pose": None, "joint_state": None, "joint_velocity": None, "js_count": 0}

    def on_depth(msg):
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        state["depth"] = arr.copy()

    def on_info(msg):
        state["info"] = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def on_tf(msg):
        for t in msg.transforms:
            if t.child_frame_id == "scene_camera_color_optical_frame":
                state["pose"] = (
                    [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z],
                    [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w],
                )

    def on_joint_states(msg):
        names = list(msg.name)
        pos_by_name = dict(zip(names, msg.position))
        vel_by_name = dict(zip(names, msg.velocity)) if msg.velocity else {}
        if all(n in pos_by_name for n in ARM_JOINT_NAMES):
            state["joint_state"] = [pos_by_name[n] for n in ARM_JOINT_NAMES]
            state["joint_velocity"] = [vel_by_name.get(n, 0.0) for n in ARM_JOINT_NAMES]
            state["js_count"] += 1  # the sim clock -- see SIM_DT_PER_JOINT_STATE

    node.create_subscription(Image, "/robot129_sim/scene_camera/aligned_depth_to_color/image_raw", on_depth, sensor_qos)
    node.create_subscription(CameraInfo, "/robot129_sim/scene_camera/aligned_depth_to_color/camera_info", on_info, sensor_qos)
    node.create_subscription(TFMessage, "/tf", on_tf, 50)
    node.create_subscription(RosJointState, "/robot129_sim/joint_states", on_joint_states, 50)
    cmd_pub = node.create_publisher(RosJointState, "/robot129_sim/piper/joint_cmd", 10)
    goal_pub = node.create_publisher(PoseStamped, "/robot129_sim/reactive/goal", sensor_qos)
    status_pub = node.create_publisher(RosString, "/robot129_sim/reactive/status", sensor_qos)
    voxel_pub = node.create_publisher(PointCloud2, "/robot129_sim/reactive/obstacle_voxels", sensor_qos)

    print("[curobo_reactive] waiting for first depth + info + tf + joint_states ...", flush=True)
    t_wait_start = time.monotonic()
    while rclpy.ok() and time.monotonic() - t_wait_start < 30.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if all(state[k] is not None for k in ("depth", "info", "pose", "joint_state")):
            break
    if any(state[k] is None for k in ("depth", "info", "pose", "joint_state")):
        print("[curobo_reactive] FAIL: did not receive all required topics within 30s", flush=True)
        return 1

    goals = [(args.goal_a, args.goal_quaternion_xyzw), (args.goal_b, args.goal_quaternion_xyzw)]
    goal_index = 0

    # ---- Lockstep with sim time -------------------------------------------------------
    # The previous loop ran at a fixed 5 Hz wall clock and, every tick, reset the MPC to
    # the MEASURED joints (arm already settled, velocity ~0) and took ONE 0.03 s step. An
    # MPC starting from rest moves ~0.0009 rad in its first 0.03 s, so the arm crept
    # (20 s -> 4 cm of progress) and visibly twitched on the stepwise targets; confirmed
    # 2026-09-24 by replaying that exact loop kinematically (0.399 m -> 0.359 m over 100
    # ticks), matching the live runs' ~0.41 m final error. Now: the MPC rolls out its OWN
    # state like cuRobo's reactive_control example, but only asks for the next MPC step
    # after Isaac has tracked the previous command.  Waypoint timeouts and the
    # goal-a/goal-b switch period are in sim seconds.
    controller.integrate_camera(state["depth"], None, state["info"], state["pose"][0], state["pose"][1], state["joint_state"])
    controller.compute_esdf()
    controller.ensure_mpc()
    controller.sync_state(state["joint_state"], [0.0] * len(ARM_JOINT_NAMES))
    state["depth"] = None

    def sim_time_s() -> float:
        return state["js_count"] * SIM_DT_PER_JOINT_STATE

    processed_js = state["js_count"]
    last_esdf_sim = sim_time_s()
    last_switch_sim = sim_time_s()
    active_target_key = None
    joint_goal = None
    pending_command = None
    t_loop_start = time.monotonic()
    voxel_publish_period_s = 0.5  # sim seconds (~2 Hz at real-time), see module docstring's publisher list
    last_voxel_publish = 0.0
    stats = {"steps": 0, "none_steps": 0, "esdf_updates": 1, "resyncs": 0, "ik_failures": 0, "sim_time_s": 0.0}
    stopped_reason = "duration_s elapsed" if args.duration_s is not None else "Ctrl-C"
    print("[curobo_reactive] starting control loop" + (" (waypoints mode)" if sequencer else " (goal-a/goal-b timer mode)")
          + " -- lockstep with /joint_states (sim time)", flush=True)
    while rclpy.ok():
        for _ in range(40):  # drain every queued callback, not one per pass
            rclpy.spin_once(node, timeout_sec=0.0)
        n_new = state["js_count"] - processed_js
        if n_new <= 0:
            rclpy.spin_once(node, timeout_sec=0.005)
            continue
        processed_js = state["js_count"]
        now = sim_time_s()
        stats["sim_time_s"] = now
        if args.duration_s is not None and time.monotonic() - t_loop_start > args.duration_s:
            break
        if sequencer is None and now - last_switch_sim > args.switch_period_s:
            goal_index = 1 - goal_index
            last_switch_sim = now
            print(f"[curobo_reactive] switching to goal {goal_index} (sim t={now:.1f}s)", flush=True)

        if state["depth"] is not None and now - last_esdf_sim >= args.esdf_period_s:
            controller.integrate_camera(
                state["depth"], None, state["info"], state["pose"][0], state["pose"][1], state["joint_state"],
            )
            controller.compute_esdf()
            stats["esdf_updates"] += 1
            last_esdf_sim = now
            state["depth"] = None  # consumed -- wait for a genuinely new frame

        waypoint_status = None
        if sequencer is not None:
            target_position, target_quaternion = sequencer.current.position_m, sequencer.current.quaternion_xyzw
            target_key = (sequencer.current.name, sequencer.history.__len__())
        else:
            target_position, target_quaternion = goals[goal_index]
            target_key = ("goal", goal_index)
        if target_key != active_target_key:
            active_target_key = target_key
            pending_command = None
            if args.no_joint_goal:
                controller.set_goal(target_position, target_quaternion)
                joint_goal = None
            else:
                # Prefer a joint solution stored by select_demo_poses.py.  It has
                # already been FK-checked and gives deterministic A/B branches.
                # Generic pose5 YAML has no stored joints, so it still uses the
                # many-seed IK fallback in set_waypoint_goal().
                stored_joint_goal = sequencer.current.joint_values_rad if sequencer is not None else None
                joint_goal = controller.set_waypoint_goal(
                    target_position, target_quaternion, near_q=controller.current_position(),
                    joint_goal=stored_joint_goal,
                )
                if joint_goal is None:
                    stats["ik_failures"] += 1
            print(f"[curobo_reactive] new target {target_key[0]} joint_goal="
                  f"{None if joint_goal is None else np.round(joint_goal, 3).tolist()} (sim t={now:.1f}s)", flush=True)

        measured = np.asarray(state["joint_state"], dtype=np.float64)
        current_position, current_quaternion = urdf_chain.pinch_center_pose(state["joint_state"])
        position_error_m = float(np.linalg.norm(current_position - np.asarray(target_position, dtype=float)))
        orientation_error_rad = quaternion_angular_distance_xyzw(current_quaternion, target_quaternion)

        # Do not advance the MPC rollout until Isaac has tracked its previous
        # position command.  Advancing once per incoming joint-state message made the
        # predicted state outrun the physical servo on large motions; repeatedly
        # resyncing then restarted the solver and caused the visible twitching.
        command_tracking_error_rad = 0.0
        if pending_command is not None:
            command_tracking_error_rad = float(np.max(np.abs(measured - np.asarray(pending_command))))
        if pending_command is None or command_tracking_error_rad <= args.command_tolerance_rad:
            stepped = controller.step()
            stats["steps"] += 1
            if stepped is None:
                stats["none_steps"] += 1
            else:
                pending_command = stepped
                command_tracking_error_rad = float(np.max(np.abs(measured - np.asarray(pending_command))))
        if pending_command is not None:
            cmd = RosJointState()
            cmd.name = ARM_JOINT_NAMES
            cmd.position = pending_command
            cmd_pub.publish(cmd)

        if sequencer is not None:
            waypoint_status = sequencer.update(position_error_m, orientation_error_rad, now)
            if waypoint_status.advanced:
                tag = "TIMEOUT" if waypoint_status.timed_out else "REACHED"
                print(f"[curobo_reactive] {tag} waypoint (now: {waypoint_status.name}, phase={waypoint_status.phase}, "
                      f"sim t={now:.1f}s, pos_err={position_error_m:.4f} m, rot_err={orientation_error_rad:.3f} rad)", flush=True)

        goal_msg = PoseStamped()
        goal_msg.header.stamp = node.get_clock().now().to_msg()
        goal_msg.header.frame_id = "world"
        goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z = [float(v) for v in target_position]
        gx, gy, gz, gw = [float(v) for v in target_quaternion]
        goal_msg.pose.orientation.x, goal_msg.pose.orientation.y, goal_msg.pose.orientation.z, goal_msg.pose.orientation.w = gx, gy, gz, gw
        goal_pub.publish(goal_msg)

        status_doc = {
            "mode": "waypoints" if sequencer is not None else "goal_ab",
            "position_error_m": position_error_m, "orientation_error_rad": orientation_error_rad,
            "steps": stats["steps"], "none_steps": stats["none_steps"], "esdf_updates": stats["esdf_updates"],
            "resyncs": stats["resyncs"], "sim_time_s": now, "joint_goal": joint_goal,
            "command_tracking_error_rad": command_tracking_error_rad,
        }
        if sequencer is not None:
            status_doc.update({
                "waypoint_index": waypoint_status.index, "waypoint_name": waypoint_status.name,
                "phase": waypoint_status.phase, "time_in_phase_s": waypoint_status.time_in_phase_s,
                "all_done": waypoint_status.all_done,
            })
        else:
            status_doc["goal_index"] = goal_index
        status_msg = RosString()
        status_msg.data = json.dumps(status_doc)
        status_pub.publish(status_msg)

        if now - last_voxel_publish >= voxel_publish_period_s:
            last_voxel_publish = now
            centers, colors = controller.occupied_voxels()
            cloud_header = Header()
            cloud_header.stamp = node.get_clock().now().to_msg()
            cloud_header.frame_id = "world"
            voxel_pub.publish(build_pointcloud2(cloud_header, centers, colors))

        if sequencer is not None and sequencer.done and not args.loop:
            stopped_reason = "all waypoints reached"
            break

    print(f"[curobo_reactive] STOPPED ({stopped_reason}) stats={stats}", flush=True)

    if sequencer is not None:
        report = {
            "status": "PASS" if sequencer.done and not any(h["timed_out"] for h in sequencer.history) else "FAIL",
            "waypoint_names": [w.name for w in waypoints],
            "loop": args.loop,
            "history": sequencer.history,
            "stats": stats,
        }
        report_out = args.report_out
        if report_out is None:
            report_out = REPO_ROOT / "out" / "demo" / "reactive" / time.strftime("%Y%m%d_%H%M%S") / "reactive_report.json"
        report_out.parent.mkdir(parents=True, exist_ok=True)
        report_out.write_text(json.dumps(report, indent=2))
        print(f"[curobo_reactive] wrote {report_out} (status={report['status']})", flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(reactive_node())
