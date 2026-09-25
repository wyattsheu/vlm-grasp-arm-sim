"""Tests for mpg.curobo_bridge.reactive.LiveEsdfReactiveController.

Runs only under env_robot129_curobo (needs `curobo`) -- skips itself under
env_robot129_research, matching test_curobo_frames.py's pattern. See
docs/dev_guide_paper_core_and_dashboard_plan.md §7.8.

Uses a REAL captured (depth, camera_info, tf, joint_states) bundle from a live Isaac
run (research/tests/fixtures/scene_camera_home/), not synthetic depth -- a synthetic
constant-depth frame produced two different degenerate results during development (see
reactive.py's module docstring on CameraObservation.depth_to_meter) that a real frame
caught and a synthetic one did not.
"""
from __future__ import annotations

import json
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

FIXTURE_DIR = Path(__file__).resolve().parent / "fixtures" / "scene_camera_home"


class TestLiveEsdfReactiveController(unittest.TestCase):
    def setUp(self):
        try:
            import curobo  # noqa: F401
        except ImportError:
            self.skipTest("curobo not installed in this venv (see dev guide §7.8)")
        if not FIXTURE_DIR.exists():
            self.skipTest(f"fixture bundle not found at {FIXTURE_DIR}")

    def _load_fixture(self):
        import numpy as np

        depth = np.load(FIXTURE_DIR / "depth.npy")
        info = json.loads((FIXTURE_DIR / "camera_info.json").read_text())
        tf = json.loads((FIXTURE_DIR / "tf.json").read_text())
        js = json.loads((FIXTURE_DIR / "joint_states.json").read_text())
        intrinsics = np.array(info["k"], dtype=np.float64).reshape(3, 3)
        name_to_pos = dict(zip(js["name"], js["position"]))
        arm_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        q6 = [name_to_pos[n] for n in arm_joint_names]
        return depth, intrinsics, tf["translation_xyz_m"], tf["quaternion_xyzw"], q6

    def test_segmentation_flags_robot_pixels_not_zero_not_all(self):
        """Regression test for the depth_to_meter bug (see reactive.py docstring): a
        wrong scale factor makes this either 0% (points collapse away from the robot)
        or 100% (points collapse onto the camera, which is "close" to everything under
        a generous threshold). A real frame with the robot visibly in view should land
        strictly between those extremes.
        """
        from mpg.curobo_bridge.reactive import DEFAULT_ROBOT_CONFIG, _camera_observation
        from curobo.kinematics import Kinematics, KinematicsCfg
        from curobo.perception import RobotSegmenter
        from curobo.types import JointState
        import torch

        depth, intrinsics, position, quaternion_xyzw, q6 = self._load_fixture()
        kin = Kinematics(KinematicsCfg.from_robot_yaml_file(str(DEFAULT_ROBOT_CONFIG)))
        seg = RobotSegmenter(kin, distance_threshold=0.05, ops_dtype=torch.float32)
        obs = _camera_observation(depth, None, intrinsics, position, quaternion_xyzw, "cuda")
        js = JointState.from_position(
            torch.tensor([q6], device="cuda", dtype=torch.float32),
            joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        mask, _filtered = seg.get_robot_mask(obs, js)
        frac = mask.float().mean().item()
        self.assertGreater(frac, 0.01, "expected some robot pixels flagged (camera sees the gripper)")
        self.assertLess(frac, 0.5, "expected most of the frame to NOT be the robot")

    def test_occupied_voxels_returns_valid_shapes_without_crashing(self):
        """Regression test for a real shape-mismatch bug found while building
        this (see reactive.py module docstring, "Extracting obstacle points"):
        calling VoxelGrid.get_occupied_voxels() directly raises a RuntimeError
        because compute_esdf()'s feature_tensor is unflattened. This fixture
        frame (pole camera at HOME) is documented (dev guide §7.4) to see no
        surfaces within the default grid extent, so this only asserts shape
        correctness and graceful-empty behavior, NOT a nonzero count -- a
        positive-count check needs a live Isaac run with an actual obstacle
        in view (dynamic_stick / static cylinder scenario), not this fixture.
        """
        import numpy as np

        from mpg.curobo_bridge.reactive import LiveEsdfReactiveController

        depth, intrinsics, position, quaternion_xyzw, q6 = self._load_fixture()
        controller = LiveEsdfReactiveController(grid_center=(0.3, 0.0, 0.3), extent_xyz=(1.2, 1.2, 1.0))
        controller.integrate_camera(depth, None, intrinsics, position, quaternion_xyzw, q6)
        controller.compute_esdf()

        centers, colors = controller.occupied_voxels()
        self.assertEqual(centers.ndim, 2)
        self.assertEqual(centers.shape[1], 3)
        self.assertEqual(colors.ndim, 2)
        self.assertEqual(colors.shape[1], 3)
        self.assertEqual(centers.shape[0], colors.shape[0])
        self.assertEqual(centers.dtype, np.float32)
        self.assertEqual(colors.dtype, np.uint8)

    def test_full_pipeline_runs_and_produces_an_action(self):
        from mpg.curobo_bridge.reactive import LiveEsdfReactiveController

        depth, intrinsics, position, quaternion_xyzw, q6 = self._load_fixture()
        controller = LiveEsdfReactiveController(grid_center=(0.3, 0.0, 0.3), extent_xyz=(1.2, 1.2, 1.0))
        controller.integrate_camera(depth, None, intrinsics, position, quaternion_xyzw, q6)
        voxel_grid = controller.compute_esdf()
        self.assertIsNotNone(voxel_grid.feature_tensor)

        controller.ensure_mpc()
        controller.sync_state(q6)
        updated = controller.set_goal([0.3, 0.15, 0.35], [0.0, 1.0, 0.0, 0.0])
        self.assertTrue(updated)
        next_positions = controller.step()
        self.assertIsNotNone(next_positions)
        self.assertEqual(len(next_positions), 6)


if __name__ == "__main__":
    unittest.main()
