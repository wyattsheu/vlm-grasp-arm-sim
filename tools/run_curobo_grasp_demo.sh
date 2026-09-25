#!/usr/bin/env bash
# cuRobo equivalent of tools/run_grasp_motion_demo.sh: (1) generate grasp candidates,
# (2) plan with cuRoboV2 against research/configs/curobo/robot129.yml (a static world
# model, not the live Isaac scene -- see step [2/3] below), (3) start the adapter and
# execute the resulting plan through the SAME unmodified run_grasp_motion.py /
# FollowJointTrajectory adapter MTC uses, so results are directly A/B-comparable.
#
# Requires an Isaac grasp/motion scene ALREADY running (tools/start_robot129_grasp_sim.sh
# --scene pick_place first) and ros2_ws/{robot129_sim_execution} already built. Does NOT
# require robot129_moveit_config/robot129_tasks (no MTC/MoveIt involved).
#
# Usage:
#   tools/run_curobo_grasp_demo.sh [--candidates-path PATH] [--scene pick_place|pick_place_hammer|pick_place_counter]
#                                   [--include-pole] [--out-dir DIR]
#
# --candidates-path: defaults to generating a fresh S2 cube candidates JSON via
#   tools/generate_s2_cube_candidates.py (only valid for --scene pick_place/pick_place_hammer;
#   pass an existing candidates file for pick_place_counter or any other object).
# --include-pole: add the back-camera pole (research/configs/scene_camera.yaml) as a
#   known obstacle in cuRobo's world model -- only pass this if Isaac was ALSO launched
#   with --scene-camera pole, otherwise cuRobo will avoid an obstacle that isn't really
#   there.
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
micro=/mnt/HDD4/wyattsheu/tools/micromamba/micromamba
ros_env=/mnt/HDD4/wyattsheu/env_robot129_ros
research_python=/mnt/HDD4/wyattsheu/env_robot129_research/bin/python
curobo_python=/mnt/HDD4/wyattsheu/env_robot129_curobo/bin/python
scratch="$(mktemp -d)"
trap 'rm -rf "$scratch"' EXIT

candidates_path=""
scene="pick_place"
include_pole=""
out_dir=""
target="cube_35"
# --target (2026-09-24, docs/dev_guide_paper_core_and_dashboard_plan.md §8's `grasp`
# demo scenario): only meaningful for --scene pick_place with no --candidates-path (it's
# forwarded to tools/generate_s2_cube_candidates.py's own --target, which is the only
# thing that reads it -- see that script for supported ids/shapes). Does NOT change what
# Isaac actually spawns; you still need to have started Isaac itself with the matching
# `--target-object` (sim/scripts/run_robot129_ros_webrtc.py), this script has no way to
# check that from here.
while [[ $# -gt 0 ]]; do
  case "$1" in
    --candidates-path) candidates_path="$2"; shift 2 ;;
    --scene) scene="$2"; shift 2 ;;
    --include-pole) include_pole="--include-pole"; shift ;;
    --out-dir) out_dir="$2"; shift 2 ;;
    --target) target="$2"; shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 1 ;;
  esac
done

# Reset FIRST, before planning -- same reasoning as run_grasp_motion_demo.sh: candidates
# assume the object sits at its nominal scene_geometry.json position, and resetting
# afterwards (inside the executor) would invalidate the plan's first point.
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
  if [[ "$scene" != "pick_place" && "$scene" != "pick_place_hammer" ]]; then
    echo "FAIL: --scene $scene needs an explicit --candidates-path (no known-geometry generator for it)" >&2
    exit 1
  fi
  candidates_path="$root/out/grasp_motion/candidates/curobo_${scene}_${target}_candidates.json"
  echo "[1/3] generating candidates (target=$target) -> $candidates_path"
  "$research_python" "$root/tools/generate_s2_cube_candidates.py" "$candidates_path" --target "$target"
else
  echo "[1/3] using existing candidates: $candidates_path"
fi

plan_path="$root/out/grasp_motion/curobo/demo_plan.json"
report_path="$root/out/grasp_motion/curobo/demo_plan_report.json"
echo "[2/3] planning with cuRobo -> $plan_path"
CUDA_VISIBLE_DEVICES="$(nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader,nounits | awk -F, '{gsub(/ /,""); print ($2*100+$3/100) "," $1}' | sort -n | head -1 | cut -d, -f2)" \
  "$curobo_python" "$root/research/src/mpg/curobo_bridge/plan_grasp.py" \
  --candidates "$candidates_path" --scene "$scene" $include_pole \
  --out "$plan_path" --report "$report_path"

status="$(python3 -c "import json; print(json.load(open('$plan_path'))['status'])")"
if [[ "$status" != "PASS" ]]; then
  echo "FAIL: cuRobo planning did not PASS (status=$status), not executing. See $report_path" >&2
  cat "$report_path" >&2
  exit 2
fi

echo "[3/3] starting adapter and executing plan"
adapter_log="$root/out/grasp_motion/adapter/curobo_demo_adapter.log"
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
# See run_grasp_motion_demo.sh's identical comment: micromamba run does not exec-replace
# itself, so killing $adapter_pid alone leaves the real adapter_node process (a
# grandchild) running and registered as an action server, breaking the next run.
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

exec_out_dir="${out_dir:-$root/out/grasp_motion/curobo_execution/$(date -u +%Y%m%dT%H%M%S)_demo}"
cat > "$scratch/exec.sh" <<EOF
set -eo pipefail
source "$ros_env/setup.bash"
source "$root/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run robot129_sim_execution run_grasp_motion \
  --plan "$plan_path" \
  --out-dir "$exec_out_dir"
EOF
exec_status=0
"$micro" run -p "$ros_env" bash "$scratch/exec.sh" || exec_status=$?

echo "planning report: $report_path"
echo "execution report: $exec_out_dir/report.json"
exit "$exec_status"
