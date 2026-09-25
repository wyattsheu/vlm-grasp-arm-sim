"""Tests for mpg.curobo_bridge.waypoints. Pure numpy/yaml -- no curobo needed."""
from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np  # noqa: E402
import yaml  # noqa: E402

from mpg.curobo_bridge.waypoints import (  # noqa: E402
    Waypoint,
    WaypointSequencer,
    load_waypoints_from_ab_poses,
    load_waypoints_from_yaml,
)

_REPO_ROOT = Path(__file__).resolve().parents[2]
_AB_POSES_PATH = _REPO_ROOT / "research" / "configs" / "demo" / "ab_poses.yaml"


def _wp(name, hold_s=1.0, timeout_s=15.0):
    return Waypoint(name=name, position_m=np.zeros(3), quaternion_xyzw=np.array([0, 0, 0, 1.0]), hold_s=hold_s, timeout_s=timeout_s)


class TestSingleWaypoint(unittest.TestCase):
    def test_advances_to_done_after_hold(self):
        seq = WaypointSequencer([_wp("A", hold_s=1.0)], position_tolerance_m=0.01, orientation_tolerance_rad=0.1)
        s0 = seq.update(position_error_m=0.001, orientation_error_rad=0.01, now_s=0.0)
        self.assertEqual(s0.phase, "holding")
        self.assertFalse(s0.all_done)
        s1 = seq.update(position_error_m=0.001, orientation_error_rad=0.01, now_s=0.5)
        self.assertFalse(s1.all_done)
        s2 = seq.update(position_error_m=0.001, orientation_error_rad=0.01, now_s=1.1)
        self.assertTrue(s2.advanced)
        self.assertTrue(s2.all_done)
        self.assertEqual(s2.phase, "done")

    def test_done_is_sticky(self):
        seq = WaypointSequencer([_wp("A", hold_s=0.1)], position_tolerance_m=0.01, orientation_tolerance_rad=0.1)
        seq.update(0.001, 0.01, 0.0)
        seq.update(0.001, 0.01, 0.2)
        self.assertTrue(seq.done)
        # further updates, even with garbage error values, must not un-done it
        s = seq.update(999.0, 999.0, 100.0)
        self.assertTrue(s.all_done)
        self.assertFalse(s.advanced)


class TestTwoWaypoints(unittest.TestCase):
    def test_advances_in_order_not_done_until_last(self):
        seq = WaypointSequencer([_wp("A", hold_s=0.5), _wp("B", hold_s=0.5)], position_tolerance_m=0.01, orientation_tolerance_rad=0.1)
        self.assertEqual(seq.current.name, "A")
        seq.update(0.001, 0.01, 0.0)
        s = seq.update(0.001, 0.01, 0.6)
        self.assertTrue(s.advanced)
        self.assertFalse(s.all_done)
        self.assertEqual(seq.current.name, "B")
        seq.update(0.001, 0.01, 0.7)
        s2 = seq.update(0.001, 0.01, 1.3)
        self.assertTrue(s2.advanced)
        self.assertTrue(s2.all_done)

    def test_history_records_each_arrival(self):
        seq = WaypointSequencer([_wp("A", hold_s=0.1), _wp("B", hold_s=0.1)], position_tolerance_m=0.01, orientation_tolerance_rad=0.1)
        seq.update(0.001, 0.01, 0.0)
        seq.update(0.001, 0.01, 0.2)  # A -> B
        seq.update(0.001, 0.01, 0.3)
        seq.update(0.001, 0.01, 0.5)  # B -> done
        names = [h["name"] for h in seq.history]
        self.assertEqual(names, ["A", "B"])
        self.assertTrue(all(not h["timed_out"] for h in seq.history))


class TestHoldTimerResets(unittest.TestCase):
    def test_losing_tolerance_mid_hold_restarts_hold(self):
        seq = WaypointSequencer([_wp("A", hold_s=1.0)], position_tolerance_m=0.01, orientation_tolerance_rad=0.1)
        seq.update(0.001, 0.01, 0.0)     # enters holding at t=0
        seq.update(0.001, 0.01, 0.9)     # still holding, 0.9s in -- not yet 1.0s
        s = seq.update(5.0, 5.0, 1.0)    # tolerance LOST just before hold would complete
        self.assertEqual(s.phase, "approaching")
        self.assertFalse(s.advanced)
        # re-enter tolerance -- hold must restart from here, not resume from 0.9s
        seq.update(0.001, 0.01, 1.1)
        s2 = seq.update(0.001, 0.01, 1.9)  # only 0.8s since re-entry -- must NOT have advanced yet
        self.assertFalse(s2.advanced)
        s3 = seq.update(0.001, 0.01, 2.2)  # now 1.1s since re-entry (t=1.1) -- should advance
        self.assertTrue(s3.advanced)


class TestTimeout(unittest.TestCase):
    def test_never_in_tolerance_forces_advance_after_timeout(self):
        seq = WaypointSequencer([_wp("A", timeout_s=2.0), _wp("B")], position_tolerance_m=0.01, orientation_tolerance_rad=0.1)
        seq.update(1.0, 1.0, 0.0)
        s = seq.update(1.0, 1.0, 2.1)
        self.assertTrue(s.advanced)
        self.assertTrue(s.timed_out)
        self.assertEqual(seq.current.name, "B")
        self.assertTrue(seq.history[0]["timed_out"])


class TestLoop(unittest.TestCase):
    def test_loop_wraps_back_to_first_waypoint(self):
        seq = WaypointSequencer([_wp("A", hold_s=0.1), _wp("B", hold_s=0.1)], position_tolerance_m=0.01, orientation_tolerance_rad=0.1, loop=True)
        seq.update(0.001, 0.01, 0.0)
        seq.update(0.001, 0.01, 0.2)  # A -> B
        self.assertEqual(seq.current.name, "B")
        seq.update(0.001, 0.01, 0.3)
        s = seq.update(0.001, 0.01, 0.5)  # B -> wraps to A
        self.assertTrue(s.advanced)
        self.assertFalse(s.all_done)
        self.assertEqual(seq.current.name, "A")

    def test_no_loop_stops_at_last(self):
        seq = WaypointSequencer([_wp("A", hold_s=0.1), _wp("B", hold_s=0.1)], position_tolerance_m=0.01, orientation_tolerance_rad=0.1, loop=False)
        seq.update(0.001, 0.01, 0.0)
        seq.update(0.001, 0.01, 0.2)
        seq.update(0.001, 0.01, 0.3)
        s = seq.update(0.001, 0.01, 0.5)
        self.assertTrue(s.all_done)


class TestLoadWaypointsFromYaml(unittest.TestCase):
    def test_approach_alias_and_explicit_quaternion(self):
        doc = {
            "waypoints": [
                {"name": "A", "position_m": [0.25, 0.35, 0.58], "approach": "up", "hold_s": 1.5},
                {"name": "B", "position_m": [0.05, 0.35, 0.35], "quaternion_xyzw": [0.0, 1.0, 0.0, 0.0], "timeout_s": 20.0},
            ]
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "waypoints.yaml"
            path.write_text(yaml.safe_dump(doc))
            waypoints = load_waypoints_from_yaml(path)
        self.assertEqual(len(waypoints), 2)
        self.assertEqual(waypoints[0].name, "A")
        self.assertAlmostEqual(waypoints[0].hold_s, 1.5)
        self.assertTrue(np.allclose(waypoints[0].position_m, [0.25, 0.35, 0.58]))
        # "up" approach must be a unit quaternion whose local +Z maps to world +Z
        self.assertAlmostEqual(float(np.linalg.norm(waypoints[0].quaternion_xyzw)), 1.0, places=6)
        self.assertEqual(waypoints[1].name, "B")
        self.assertTrue(np.allclose(waypoints[1].quaternion_xyzw, [0.0, 1.0, 0.0, 0.0]))
        self.assertAlmostEqual(waypoints[1].timeout_s, 20.0)

    def test_optional_joint_goal_is_preserved(self):
        joints = [0.1, 0.2, -0.3, 0.4, -0.5, 0.6]
        doc = {"waypoints": [{
            "name": "A", "position_m": [0.25, 0.35, 0.58],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "joint_values_rad": joints,
        }]}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "waypoints.yaml"
            path.write_text(yaml.safe_dump(doc))
            waypoint = load_waypoints_from_yaml(path)[0]
        self.assertTrue(np.allclose(waypoint.joint_values_rad, joints))

    def test_unknown_approach_raises(self):
        doc = {"waypoints": [{"name": "A", "position_m": [0, 0, 0], "approach": "sideways"}]}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "waypoints.yaml"
            path.write_text(yaml.safe_dump(doc))
            with self.assertRaises(ValueError):
                load_waypoints_from_yaml(path)

    def test_missing_orientation_raises(self):
        doc = {"waypoints": [{"name": "A", "position_m": [0, 0, 0]}]}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "waypoints.yaml"
            path.write_text(yaml.safe_dump(doc))
            with self.assertRaises(ValueError):
                load_waypoints_from_yaml(path)

    def test_empty_waypoints_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "waypoints.yaml"
            path.write_text(yaml.safe_dump({"waypoints": []}))
            with self.assertRaises(ValueError):
                load_waypoints_from_yaml(path)


class TestLoadWaypointsFromAbPoses(unittest.TestCase):
    def test_synthetic_ab_poses_schema(self):
        doc = {
            "schema_version": "demo_ab_poses_v1",
            "points": {
                "A": {"name": "A", "position_m": [0.25, 0.35, 0.58], "quaternion_xyzw": [0, 0, 0, 1]},
                "B": {"name": "B", "position_m": [0.05, 0.35, 0.35], "quaternion_xyzw": [0, 0.7071, 0.7071, 0]},
            },
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "ab_poses.yaml"
            path.write_text(yaml.safe_dump(doc))
            waypoints = load_waypoints_from_ab_poses(path, hold_s=2.0)
        self.assertEqual([w.name for w in waypoints], ["A", "B"])
        self.assertTrue(np.allclose(waypoints[0].position_m, [0.25, 0.35, 0.58]))
        self.assertAlmostEqual(waypoints[0].hold_s, 2.0)

    def test_wrong_schema_version_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.yaml"
            path.write_text(yaml.safe_dump({"schema_version": "something_else", "points": {}}))
            with self.assertRaises(ValueError):
                load_waypoints_from_ab_poses(path)

    @unittest.skipUnless(_AB_POSES_PATH.exists(), "research/configs/demo/ab_poses.yaml not generated yet (run select_demo_poses.py)")
    def test_real_ab_poses_file_loads(self):
        # End-to-end check against the actual Phase 0 output, not just a
        # synthetic fixture -- confirms select_demo_poses.py's schema and
        # this loader haven't drifted apart.
        waypoints = load_waypoints_from_ab_poses(_AB_POSES_PATH)
        self.assertEqual(len(waypoints), 2)
        self.assertEqual(waypoints[0].name, "A")
        self.assertEqual(waypoints[1].name, "B")
        for w in waypoints:
            self.assertAlmostEqual(float(np.linalg.norm(w.quaternion_xyzw)), 1.0, places=5)
            self.assertEqual(w.joint_values_rad.shape, (6,))


if __name__ == "__main__":
    unittest.main()
