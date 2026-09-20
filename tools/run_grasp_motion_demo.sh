#!/usr/bin/env bash
# Chains the three commands docs/progress/grasp_motion_progress_report.md's "如何自己
# 重現" section previously documented as three separate manual steps run in two
# different Python environments: (1) generate grasp candidates, (2) plan with MTC
# against the live Isaac scene, (3) start the adapter and execute the resulting plan.
#
# Requires an Isaac grasp/motion scene ALREADY running -- this script does not start or
# stop Isaac itself, matching every other tool in this directory (see
# tools/start_robot129_grasp_sim.sh --scene pick_place first). Requires
# ros2_ws/{robot129_moveit_config,robot129_tasks,robot129_sim_execution} already built.
#
# Usage:
#   tools/run_grasp_motion_demo.sh [--candidates-path PATH] [--max-candidates N]
#                                   [--record] [--out-dir DIR]
#
# --candidates-path: defaults to generating a fresh S2 cube candidates JSON via
#   tools/generate_s2_cube_candidates.py. Pass an existing file (e.g. an S4 live-VLM
#   candidates JSON) to skip that generation step.
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
micro=/mnt/HDD4/wyattsheu/tools/micromamba/micromamba
ros_env=/mnt/HDD4/wyattsheu/env_robot129_ros
research_python=/mnt/HDD4/wyattsheu/env_robot129_research/bin/python
scratch="$(mktemp -d)"
trap 'rm -rf "$scratch"' EXIT

candidates_path=""
max_candidates=8
record_flag=""
out_dir=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --candidates-path) candidates_path="$2"; shift 2 ;;
    --max-candidates) max_candidates="$2"; shift 2 ;;
    --record) record_flag="--record"; shift ;;
    --out-dir) out_dir="$2"; shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 1 ;;
  esac
done

# Reset FIRST, before planning: MTC's grasp candidates (whether freshly generated or
# passed in) assume the cube sits at its nominal scene_geometry.json position, and the
# executor's own reset_scene call (which it always does, right before executing) moves
# the arm back to HOME right before running the plan. Planning against whatever state a
# PREVIOUS demo cycle left the scene in (arm at the place pose, cube already moved) and
# only resetting afterwards, inside the executor, produces a plan whose very first point
# no longer matches reality once that reset runs -- discovered 2026-09-20 by running this
# wrapper twice in a row against the same live Isaac process without this reset-first
# step; the executor rejected the first arm segment outright ("t=0 first point must
# match the current commanded target"). Resetting here makes the executor's own reset a
# no-op, not a second state change the plan wasn't computed against.
echo "[0/3] resetting scene before planning"
cat > "$scratch/reset.sh" <<EOF
set -eo pipefail
source "$ros_env/setup.bash"
source "$root/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 service call /robot129_sim/reset_scene std_srvs/srv/Trigger
sleep 2
EOF
"$micro" run -p "$ros_env" bash "$scratch/reset.sh"

if [[ -z "$candidates_path" ]]; then
  candidates_path="$root/out/grasp_motion/candidates/s2_cube_candidates.json"
  echo "[1/3] generating candidates -> $candidates_path"
  "$research_python" "$root/tools/generate_s2_cube_candidates.py" "$candidates_path"
else
  echo "[1/3] using existing candidates: $candidates_path"
fi

plan_path="$root/out/grasp_motion/mtc_pick_place_demo.json"
echo "[2/3] planning with MTC against live Isaac -> $plan_path"
cat > "$scratch/plan.sh" <<EOF
set -eo pipefail
source "$ros_env/setup.bash"
source "$root/ros2_ws/install/setup.bash"
export LD_LIBRARY_PATH="\$CONDA_PREFIX/lib:\$LD_LIBRARY_PATH"
export ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp
timeout 60 ros2 launch robot129_tasks mtc_pick_place_sim.launch.py \
  report_path:="$plan_path" \
  candidates_path:="$candidates_path" \
  max_candidates:=$max_candidates
EOF
"$micro" run -p "$ros_env" bash "$scratch/plan.sh"

status="$(python3 -c "import json; print(json.load(open('$plan_path'))['status'])")"
if [[ "$status" != "PASS" ]]; then
  echo "FAIL: MTC planning did not PASS (status=$status), not executing. See $plan_path" >&2
  exit 2
fi

echo "[3/3] starting adapter and executing plan"
adapter_log="$root/out/grasp_motion/adapter/demo_adapter.log"
mkdir -p "$(dirname "$adapter_log")"
cat > "$scratch/adapter.sh" <<EOF
source "$ros_env/setup.bash"
source "$root/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec ros2 run robot129_sim_execution adapter_node \
  --ros-args --params-file "$root/ros2_ws/src/robot129_sim_execution/config/adapter_params.yaml" \
  -p log_dir:="$root/out/grasp_motion/adapter" \
  -r __ns:=/robot129_sim
EOF
"$micro" run -p "$ros_env" bash "$scratch/adapter.sh" > "$adapter_log" 2>&1 &
adapter_pid=$!
# kill "$adapter_pid" alone is NOT enough: micromamba run does not exec-replace itself,
# so the actual adapter_node process is a grandchild that survives its parent being
# killed -- discovered 2026-09-20 when a leftover adapter_node from an earlier run of
# this script (killed this way, apparently successfully) was still registered as an
# action server on the next run, causing the new adapter's goals to be silently
# rejected ("more than one action server for the action ..."). pkill -f is the same
# fallback tools/stop_robot129_grasp_sim.sh already uses for the identical reason.
cleanup() {
  kill "$adapter_pid" 2>/dev/null || true
  pkill -f "robot129_sim_execution.*adapter_node.*--ros-args" 2>/dev/null || true
}
trap 'cleanup; rm -rf "$scratch"' EXIT
sleep 3
if ! pgrep -f "robot129_sim_execution.*adapter_node.*--ros-args" > /dev/null; then
  echo "FAIL: adapter_node exited before executing anything; see $adapter_log" >&2
  exit 3
fi

exec_out_dir="${out_dir:-$root/out/grasp_motion/mtc_execution/$(date -u +%Y%m%dT%H%M%S)_demo}"
cat > "$scratch/exec.sh" <<EOF
set -eo pipefail
source "$ros_env/setup.bash"
source "$root/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run robot129_sim_execution run_grasp_motion \
  --plan "$plan_path" $record_flag \
  --out-dir "$exec_out_dir"
EOF
exec_status=0
"$micro" run -p "$ros_env" bash "$scratch/exec.sh" || exec_status=$?

echo "report: $exec_out_dir/report.json"
exit "$exec_status"
