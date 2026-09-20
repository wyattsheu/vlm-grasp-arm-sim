#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
isaac="/mnt/HDD4/wyattsheu/IsaacLab"
rosroot="$isaac/.venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy"
export ROS_DISTRO=jazzy ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$rosroot/rclpy" LD_LIBRARY_PATH="$rosroot/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$isaac/.venv/bin/python" "$root/tools/verify_ros_isolation.py"
