#!/usr/bin/env bash
# Run a Python script with the Robot 129 ROS 2 environment already set up, so
# it can `import robot129_control_api` without dealing with ROS/Isaac Sim
# environment details.
#
# Prerequisite: the simulator must already be running --
#   bash tools/start_robot129_ros_webrtc.sh
#
# Usage:
#   bash tools/run_with_robot129_control.sh my_algorithm.py [args...]
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
micro=/mnt/HDD4/wyattsheu/tools/micromamba/micromamba
envroot=/mnt/HDD4/wyattsheu/env_robot129_ros

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <python_script.py> [args...]" >&2
  exit 2
fi

exec "$micro" run -p "$envroot" bash -lc '
source /mnt/HDD4/wyattsheu/env_robot129_ros/setup.bash
source "'$root'/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="'$root'/tools${PYTHONPATH:+:$PYTHONPATH}"
exec python3 "$@"
' _ "$@"
