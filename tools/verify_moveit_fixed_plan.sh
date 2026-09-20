#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
envroot=/mnt/HDD4/wyattsheu/env_robot129_ros
micro=/mnt/HDD4/wyattsheu/tools/micromamba/micromamba
mkdir -p "$root/out/lesson_09"
exec "$micro" run -p "$envroot" bash -lc '
set -o pipefail
source /mnt/HDD4/wyattsheu/env_robot129_ros/setup.bash
source "'$root'/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim
ros2 launch robot129_moveit_config move_group.launch.py allow_trajectory_execution:=false > "'$root'/out/lesson_09/move_group.log" 2>&1 &
pid=$!
cleanup() {
  kill -TERM "$pid" 2>/dev/null || true
  for _ in 1 2 3 4 5; do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.2
  done
  kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
python "'$root'/tools/verify_moveit_fixed_plan.py"
'
