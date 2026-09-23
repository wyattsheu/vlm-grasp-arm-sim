#!/usr/bin/env bash
# Run one task through the REAL robot's mm_actions code against the sim.
# Usage:
#   bash tools/start_robot129_ros_webrtc.sh --scene pick_place_counter   # once
#   bash tools/run_real_stack_task.sh "grasp the red block"
#   bash tools/reset_robot129_scene.sh                                 # put block + arm back
#   bash tools/stop_robot129_ros_webrtc.sh                             # shut the sim down
#   VLM_BACKEND=gemini bash tools/run_real_stack_task.sh "..."           # Gemini API instead
# VLM_BACKEND defaults to local, same as the real mm_actions_node / scripts/run_grasp.sh: this
# script starts the Qwen + Molmo2 vLLM engines (~1-2 min), runs the task, and stops them on
# exit (also on failure / Ctrl-C) so they hold no GPU memory while idle. For several runs back
# to back, KEEP_LOCAL_ENGINES=1 leaves them up; stop later with stop_robot129_local_engines.sh.
# Extra args go to sim/scripts/sim_mm_actions_node.py (--grasp-close-width, --skip-observe, --vlm-retries).
#   GEMINI_MODEL=gemini-2.5-flash bash tools/run_real_stack_task.sh "..."   # if the default preview model is overloaded (503)
#
# Needs /mnt/HDD4/wyattsheu/env_robot129_realstack: a venv on top of env_robot129_ros with
# the real stack's pinned deps (references/upstream/mm_system/main_ws/requirements.txt:
# the lab's roboticstoolbox fork with the Piper model, numpy 1.26.4, qpsolvers, quadprog,
# google-genai, rerun-sdk). It is put first on PYTHONPATH because the ROS setup.bash
# prepends env_robot129_ros's own site-packages (numpy 2.x, which the fork's compiled
# modules cannot load).
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
micro=/mnt/HDD4/wyattsheu/tools/micromamba/micromamba
rosenv=/mnt/HDD4/wyattsheu/env_robot129_ros
venv=/mnt/HDD4/wyattsheu/env_robot129_realstack
[[ -x "$venv/bin/python" ]] || { echo "missing $venv (see header of this script)" >&2; exit 2; }
[[ $# -ge 1 ]] || { echo "usage: $0 \"<instruction>\" [--grasp-close-width W] [--skip-observe]" >&2; exit 2; }
export VLM_BACKEND="${VLM_BACKEND:-local}"
if [[ "$VLM_BACKEND" == local ]]; then
  # The unchanged LocalPipelineClient reads these; the sim engines live on :8010/:8012
  # because :8000/:8002 are taken on this machine (see start_robot129_local_engines.sh).
  export STAGE1_BASE_URL="${STAGE1_BASE_URL:-http://127.0.0.1:8010/v1}"
  export STAGE2_BASE_URL="${STAGE2_BASE_URL:-http://127.0.0.1:8012/v1}"
  [[ "${KEEP_LOCAL_ENGINES:-0}" == 1 ]] || trap 'bash "$root/tools/stop_robot129_local_engines.sh"' EXIT
  bash "$root/tools/start_robot129_local_engines.sh" || exit 3
elif [[ -z "${GOOGLE_API_KEY:-}${GEMINI_API_KEY:-}" && -f "$HOME/.config/robot129_gemini.env" ]]; then
  set -a; source "$HOME/.config/robot129_gemini.env"; set +a
fi
"$micro" run -p "$rosenv" bash -c '
source "'"$rosenv"'/setup.bash"
export ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONNOUSERSITE=1
export PYTHONPATH="'"$venv"'/lib/python3.12/site-packages:${PYTHONPATH:-}"
exec "'"$venv"'/bin/python" -W ignore "'"$root"'/sim/scripts/sim_mm_actions_node.py" "$@"
' _ "$@"
