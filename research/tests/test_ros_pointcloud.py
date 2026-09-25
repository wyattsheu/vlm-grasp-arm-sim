"""Tests for mpg.ros_pointcloud.build_pointcloud2.

Needs rclpy/sensor_msgs -- present in env_robot129_ros (and env_robot129_curobo
once ROS is sourced on top of it), NOT in env_robot129_research. Decodes the
built message with sensor_msgs_py.point_cloud2.read_points, an INDEPENDENT
decoder from our own struct.pack_into call, so a real field-offset/point_step
bug would show up as a decode mismatch, not just round-trip with itself.
"""
from __future__ import annotations

import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))


class TestBuildPointCloud2(unittest.TestCase):
    def setUp(self):
        try:
            import rclpy  # noqa: F401
            from sensor_msgs_py import point_cloud2  # noqa: F401
            from std_msgs.msg import Header  # noqa: F401
        except ImportError:
            self.skipTest("rclpy/sensor_msgs_py not installed in this venv (needs env_robot129_ros)")

    def test_roundtrip_via_independent_decoder(self):
        import numpy as np
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header

        from mpg.ros_pointcloud import build_pointcloud2

        points = np.array([[0.1, 0.2, 0.3], [-1.0, 2.5, 0.0], [0.0, 0.0, 0.0]], dtype=np.float32)
        colors = np.array([[255, 0, 0], [0, 255, 0], [10, 20, 30]], dtype=np.uint8)
        header = Header()
        header.frame_id = "world"

        msg = build_pointcloud2(header, points, colors)
        self.assertEqual(msg.header.frame_id, "world")
        self.assertEqual(msg.width, 3)
        self.assertEqual(msg.height, 1)
        self.assertEqual(len(msg.data), msg.point_step * 3)

        decoded = list(point_cloud2.read_points(msg, field_names=["x", "y", "z", "r", "g", "b"]))
        self.assertEqual(len(decoded), 3)
        for i, row in enumerate(decoded):
            self.assertAlmostEqual(float(row["x"]), float(points[i, 0]), places=5)
            self.assertAlmostEqual(float(row["y"]), float(points[i, 1]), places=5)
            self.assertAlmostEqual(float(row["z"]), float(points[i, 2]), places=5)
            self.assertEqual(int(row["r"]), int(colors[i, 0]))
            self.assertEqual(int(row["g"]), int(colors[i, 1]))
            self.assertEqual(int(row["b"]), int(colors[i, 2]))

    def test_empty_pointcloud(self):
        import numpy as np
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header

        from mpg.ros_pointcloud import build_pointcloud2

        header = Header()
        header.frame_id = "world"
        msg = build_pointcloud2(header, np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.uint8))
        self.assertEqual(msg.width, 0)
        self.assertEqual(len(msg.data), 0)
        decoded = list(point_cloud2.read_points(msg, field_names=["x", "y", "z"]))
        self.assertEqual(len(decoded), 0)


if __name__ == "__main__":
    unittest.main()
