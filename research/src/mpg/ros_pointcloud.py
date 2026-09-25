"""Manual sensor_msgs/PointCloud2 construction: x, y, z (FLOAT32) + r, g, b
(UINT8, one field each -- NOT PCL's packed-float rgb convention).

Split out of research/src/mpg/curobo_bridge/reactive.py (which imports curobo
unconditionally at module level, so it can't be imported for a quick test
under env_robot129_ros) so research/tests/test_ros_pointcloud.py can build a
real message and decode it with sensor_msgs_py.point_cloud2.read_points --
an independent decoder, not just re-parsing our own struct.pack_into output --
under whichever venv actually has rclpy/sensor_msgs (env_robot129_ros), no
curobo needed. reactive.py imports build_pointcloud2 from here.
"""
from __future__ import annotations

import struct

import numpy as np

POINT_STEP = 16  # 3 x float32 (12 bytes) + 3 x uint8 (3 bytes) + 1 byte padding


def build_pointcloud2(header, points_xyz: np.ndarray, colors_rgb: np.ndarray):
    """header: an already-populated std_msgs/Header (stamp + frame_id set by
    the caller). points_xyz: [N, 3] float. colors_rgb: [N, 3] uint8-ish.
    """
    from sensor_msgs.msg import PointCloud2, PointField

    n = points_xyz.shape[0]
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="r", offset=12, datatype=PointField.UINT8, count=1),
        PointField(name="g", offset=13, datatype=PointField.UINT8, count=1),
        PointField(name="b", offset=14, datatype=PointField.UINT8, count=1),
    ]
    data = bytearray(POINT_STEP * n)
    xyz = np.asarray(points_xyz, dtype=np.float32)
    rgb = np.asarray(colors_rgb, dtype=np.uint8)
    for i in range(n):
        struct.pack_into("<fffBBBx", data, i * POINT_STEP, xyz[i, 0], xyz[i, 1], xyz[i, 2], rgb[i, 0], rgb[i, 1], rgb[i, 2])

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = POINT_STEP
    msg.row_step = POINT_STEP * n
    msg.data = bytes(data)
    msg.is_dense = True
    return msg
