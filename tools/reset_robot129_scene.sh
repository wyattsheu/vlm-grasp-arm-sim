#!/usr/bin/env bash
# Put the running sim back to its start state: arm to HOME (OBSERVE pose in
# pick_place_counter), target object back on its default spot, velocities zeroed.
# Usage: bash tools/reset_robot129_scene.sh
# Calls /robot129_sim/reset_scene (std_srvs/Trigger); refused while a trajectory is running.
set -euo pipefail
micro=/mnt/HDD4/wyattsheu/tools/micromamba/micromamba
rosenv=/mnt/HDD4/wyattsheu/env_robot129_ros
exec "$micro" run -p "$rosenv" bash -c '
source "'"$rosenv"'/setup.bash"
export ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 service call /robot129_sim/reset_scene std_srvs/srv/Trigger
'
