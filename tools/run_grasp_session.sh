#!/usr/bin/env bash
# The single command: type this, the arm moves for real, and when it's done you have
# a video AND the annotated images in one folder. Combines everything built so far:
#   1. real VLM grounding (stage_a/b) + affordance region + grasp candidates
#      (tools/run_grasp_dashboard.sh's steps 0-3, factored out below)
#   2. feeds those VLM-derived candidates into MTC planning + real execution + video
#      recording (tools/run_grasp_motion_demo.sh, via --candidates-path)
#   3. renders vlm_overlay.png / semantic_points.png / candidates_ghost.png against
#      the SAME run's data, and copies the recorded video alongside them
#
# Requires an Isaac grasp/motion scene ALREADY running (tools/start_robot129_grasp_sim.sh
# or tools/start_robot129_ros_webrtc.sh --scene pick_place first).
#
# Usage:
#   tools/run_grasp_session.sh [scene_id] [instruction] [object_id] [--backend qwen|gemini]
#   tools/run_grasp_session.sh                                    # all defaults, qwen (free)
#
# --backend gemini: real per-call cost. Reads GEMINI_API_KEY from the environment
#   only (AGENTS.md rule 13) -- export it yourself first, this script never accepts
#   it as an argument. See tools/run_grasp_dashboard.sh's header for why you'd want
#   this (free local Qwen3-VL-8B hallucinating scene layout once clutter is added).
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
isaac=/mnt/HDD4/wyattsheu/IsaacLab
rosroot="$isaac/.venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy"
ros_env=/mnt/HDD4/wyattsheu/env_robot129_ros
micro=/mnt/HDD4/wyattsheu/tools/micromamba/micromamba
research_python=/mnt/HDD4/wyattsheu/env_robot129_research/bin/python

backend="qwen"
positional=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --backend) backend="$2"; shift 2 ;;
    *) positional+=("$1"); shift ;;
  esac
done
scene_id="${positional[0]:-session_$(date -u +%Y%m%dT%H%M%SZ)}"
instruction="${positional[1]:-pick up the red cube and place it on the green pad}"
object_id="${positional[2]:-red_cube}"

if [[ "$backend" == "gemini" && -z "${GEMINI_API_KEY:-}" ]]; then
  echo "FAIL: --backend gemini requires GEMINI_API_KEY set in the environment (not passed as an argument, not guessed)." >&2
  exit 2
fi

session_dir="$root/out/grasp_motion/runs/$scene_id"
mkdir -p "$session_dir"
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

if [[ "$backend" == "gemini" ]]; then
  echo "[1/5] backend=gemini, no local vLLM needed"
elif ! curl -sf --max-time 2 http://127.0.0.1:8001/v1/models > /dev/null 2>&1; then
  echo "[1/5] starting local vLLM (not already running)"
  bash "$root/tools/start_robot129_vllm.sh"
  vllm_started_by_us=1
else
  echo "[1/5] local vLLM already running, reusing it"
fi

echo "[2/5] resetting scene + capturing a fresh wrist-camera snapshot"
cat > "$scratch/reset.sh" <<EOF
set -eo pipefail
source "$ros_env/setup.bash"
source "$root/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 service call /robot129_sim/reset_scene std_srvs/srv/Trigger
sleep 1
EOF
"$micro" run -p "$ros_env" bash "$scratch/reset.sh"

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

echo "[3/5] real VLM grounding + affordance + candidate generation (backend=$backend)"
"$research_python" "$root/research/scripts/s4_live_affordance_smoke_test.py" "$scene_id" "$session_dir" --backend "$backend"
candidates_path="$session_dir/s4_live_candidates.json"
if [[ ! -f "$candidates_path" ]]; then
  echo "FAIL: no candidates produced by the VLM step (see output above -- likely GroundingFailure or NoGraspStepError)" >&2
  exit 1
fi

echo "[4/5] planning with MTC against these VLM-derived candidates + real execution + recording"
plan_path="$session_dir/mtc_plan.json"
cat > "$scratch/plan.sh" <<EOF
set -eo pipefail
source "$ros_env/setup.bash"
source "$root/ros2_ws/install/setup.bash"
export LD_LIBRARY_PATH="\$CONDA_PREFIX/lib:\$LD_LIBRARY_PATH"
export ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp
timeout 60 ros2 launch robot129_tasks mtc_pick_place_sim.launch.py \
  report_path:="$plan_path" \
  candidates_path:="$candidates_path" \
  max_candidates:=8
EOF
"$micro" run -p "$ros_env" bash "$scratch/plan.sh"

plan_status="$(python3 -c "import json; print(json.load(open('$plan_path'))['status'])")"
if [[ "$plan_status" != "PASS" ]]; then
  echo "FAIL: MTC planning did not PASS (status=$plan_status) for these VLM candidates -- not executing. See $plan_path" >&2
  exit 2
fi
chosen_id="$(python3 -c "import json; print(json.load(open('$plan_path'))['chosen_grasp_candidate_id'])")"

adapter_log="$session_dir/adapter.log"
cat > "$scratch/adapter.sh" <<EOF
source "$ros_env/setup.bash"
source "$root/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec ros2 run robot129_sim_execution adapter_node \
  --ros-args --params-file "$root/ros2_ws/src/robot129_sim_execution/config/adapter_params.yaml" \
  -p log_dir:="$session_dir" \
  -r __ns:=/robot129_sim
EOF
"$micro" run -p "$ros_env" bash "$scratch/adapter.sh" > "$adapter_log" 2>&1 &
adapter_pid=$!
adapter_cleanup() {
  kill "$adapter_pid" 2>/dev/null || true
  pkill -f "robot129_sim_execution.*adapter_node.*--ros-args" 2>/dev/null || true
}
trap 'adapter_cleanup; cleanup' EXIT
sleep 3
if ! pgrep -f "robot129_sim_execution.*adapter_node.*--ros-args" > /dev/null; then
  echo "FAIL: adapter_node exited before executing anything; see $adapter_log" >&2
  exit 3
fi

cat > "$scratch/exec.sh" <<EOF
set -eo pipefail
source "$ros_env/setup.bash"
source "$root/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run robot129_sim_execution run_grasp_motion \
  --plan "$plan_path" --record \
  --out-dir "$session_dir"
EOF
exec_status=0
"$micro" run -p "$ros_env" bash "$scratch/exec.sh" || exec_status=$?
adapter_cleanup
trap cleanup EXIT

echo "[5/5] rendering dashboard images"
video_path="$(ls -t "$root"/out/grasp_motion/sessions/run_*/video.mp4 2>/dev/null | head -1 || true)"
"$research_python" "$root/research/scripts/render_run_dashboard.py" \
  --scene-bundle "$root/research/data/scenes/$scene_id" \
  --affordance-json "$session_dir/affordance_region.json" \
  --located-steps-json "$session_dir/located_steps.json" \
  --candidates-json "$candidates_path" \
  --chosen-id "$chosen_id" \
  --object-id "$object_id" \
  ${video_path:+--video "$video_path"} \
  --out-dir "$session_dir"
# render_run_dashboard.py's --video only takes one file (-> camera.mp4, third-person).
# The first-person wrist_video.mp4 (2026-09-21: same recording toggle now captures both,
# see sim/scripts/run_robot129_ros_webrtc.py's start_recording()) is copied here directly.
wrist_video_path="$(ls -t "$root"/out/grasp_motion/sessions/run_*/wrist_video.mp4 2>/dev/null | head -1 || true)"
if [[ -n "$wrist_video_path" ]]; then
  cp "$wrist_video_path" "$session_dir/wrist_camera.mp4"
  echo "copied $wrist_video_path -> $session_dir/wrist_camera.mp4"
fi

echo
echo "=== session folder: $session_dir ==="
ls "$session_dir"
exit "$exec_status"
