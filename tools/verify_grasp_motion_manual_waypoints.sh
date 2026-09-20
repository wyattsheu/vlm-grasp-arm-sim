#!/usr/bin/env bash
# Runs tools/verify_grasp_motion_manual_waypoints.py against the already-running
# pick_place sim (start it first with tools/start_robot129_grasp_sim.sh --scene pick_place).
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
/mnt/HDD4/wyattsheu/tools/micromamba/micromamba run -p /mnt/HDD4/wyattsheu/env_robot129_ros bash -lc "
source /mnt/HDD4/wyattsheu/env_robot129_ros/setup.bash
export ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 '$root/tools/verify_grasp_motion_manual_waypoints.py' \"\$@\"
" _ "$@"
