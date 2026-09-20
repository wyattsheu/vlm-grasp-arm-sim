#!/usr/bin/env bash
# One command for the "VLM overlay + candidate ghosts" dashboard-lite images
# (docs/dev_guide_paper_core_and_dashboard_plan.md §3): reset scene -> capture a fresh
# wrist-camera snapshot -> real VLM grounding + affordance + candidate generation ->
# render the two static images. Requires an Isaac grasp/motion scene ALREADY running
# (tools/start_robot129_grasp_sim.sh or tools/start_robot129_ros_webrtc.sh --scene
# pick_place first), same precondition as tools/run_grasp_motion_demo.sh.
#
# Starts the local vLLM server itself if it isn't already up, and only stops it again
# on exit if this script is the one that started it -- running the steps by hand one at
# a time (as documented before this script existed) had a real race where a manual
# "stop vllm" step could land while an earlier "start vllm" step was still mid-load,
# killing it with SIGTERM 10s into loading (found 2026-09-20).
#
# Usage:
#   tools/run_grasp_dashboard.sh [scene_id] [instruction] [object_id]
#   tools/run_grasp_dashboard.sh                                    # all defaults
#   tools/run_grasp_dashboard.sh my_run_01 "pick up the red cube and place it on the green pad" red_cube
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
isaac=/mnt/HDD4/wyattsheu/IsaacLab
rosroot="$isaac/.venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy"
ros_env=/mnt/HDD4/wyattsheu/env_robot129_ros
micro=/mnt/HDD4/wyattsheu/tools/micromamba/micromamba
research_python=/mnt/HDD4/wyattsheu/env_robot129_research/bin/python

scene_id="${1:-dashboard_$(date -u +%Y%m%dT%H%M%SZ)}"
instruction="${2:-pick up the red cube and place it on the green pad}"
object_id="${3:-red_cube}"

vllm_dir="$root/out/grasp_motion/dashboard_runs/$scene_id"
mkdir -p "$vllm_dir"
scratch="$(mktemp -d)"

vllm_started_by_us=0
cleanup() {
  rm -rf "$scratch"
  if [[ "$vllm_started_by_us" == 1 ]]; then
    echo "[cleanup] stopping the vLLM server this run started"
    bash "$root/tools/stop_robot129_vllm.sh" || true
  fi
}
trap cleanup EXIT

if ! curl -sf --max-time 2 http://127.0.0.1:8001/v1/models > /dev/null 2>&1; then
  echo "[0/4] starting local vLLM (not already running)"
  bash "$root/tools/start_robot129_vllm.sh"
  vllm_started_by_us=1
else
  echo "[0/4] local vLLM already running, reusing it"
fi

echo "[1/4] resetting scene"
cat > "$scratch/reset.sh" <<EOF
set -eo pipefail
source "$ros_env/setup.bash"
source "$root/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 service call /robot129_sim/reset_scene std_srvs/srv/Trigger
sleep 1
EOF
"$micro" run -p "$ros_env" bash "$scratch/reset.sh"

echo "[2/4] capturing a fresh wrist-camera snapshot -> research/data/scenes/$scene_id"
export ROS_DISTRO=jazzy ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$rosroot/rclpy${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$rosroot/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$research_python" "$root/research/scripts/capture_scene.py" \
  --scene-id "$scene_id" \
  --instruction "$instruction" \
  --base-frame world \
  --depth-scale-m 1.0 \
  --color-topic /robot129_sim/camera/color/image_raw \
  --depth-topic /robot129_sim/camera/aligned_depth_to_color/image_raw \
  --camera-info-topic /robot129_sim/camera/aligned_depth_to_color/camera_info \
  --joint-topic /robot129_sim/joint_states \
  --best-effort

echo "[3/4] real VLM grounding + affordance + candidate generation"
"$research_python" "$root/research/scripts/s4_live_affordance_smoke_test.py" "$scene_id" "$vllm_dir"

chosen_id="$(python3 -c "
import json
d = json.load(open('$vllm_dir/s4_live_candidates.json'))
accepted = [c for c in d['candidates'] if not c.get('rejection_reasons')]
print(accepted[0]['candidate_id'] if accepted else '')
")"
if [[ -z "$chosen_id" ]]; then
  echo "FAIL: no accepted candidate to highlight as chosen" >&2
  exit 1
fi

echo "[4/4] rendering dashboard images (chosen=$chosen_id)"
dashboard_dir="$root/out/grasp_motion/dashboard/$scene_id"
"$research_python" "$root/research/scripts/render_run_dashboard.py" \
  --scene-bundle "$root/research/data/scenes/$scene_id" \
  --affordance-json "$vllm_dir/affordance_region.json" \
  --candidates-json "$vllm_dir/s4_live_candidates.json" \
  --chosen-id "$chosen_id" \
  --object-id "$object_id" \
  --out-dir "$dashboard_dir"

echo "done: $dashboard_dir/vlm_overlay.png"
echo "      $dashboard_dir/candidates_ghost.png"
