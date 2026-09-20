#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
micro=/mnt/HDD4/wyattsheu/tools/micromamba/micromamba
envroot=/mnt/HDD4/wyattsheu/env_robot129_ros
mkdir -p "$root/out/lesson_08"
exec "$micro" run -p "$envroot" bash -lc '
set -eo pipefail
source /mnt/HDD4/wyattsheu/env_robot129_ros/setup.bash
source "'$root'/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim
setsid timeout --signal=TERM --kill-after=3s 30s ros2 launch robot129_sim_bringup mock_control.launch.py > "'$root'/out/lesson_08/ros2_control_guards.log" 2>&1 &
launcher=$!
cleanup() { kill -TERM -- -"$launcher" 2>/dev/null || true; wait "$launcher" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
python "'$root'/tools/verify_ros2_control_guards.py"
'
