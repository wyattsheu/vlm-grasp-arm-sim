#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
micro=/mnt/HDD4/wyattsheu/tools/micromamba/micromamba
envroot=/mnt/HDD4/wyattsheu/env_robot129_ros
exec "$micro" run -p "$envroot" bash -lc '
source /mnt/HDD4/wyattsheu/env_robot129_ros/setup.bash
source "'$root'/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec ros2 "$@"
' _ "$@"
