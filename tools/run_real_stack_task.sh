#!/usr/bin/env bash
# Run one task through the REAL robot's mm_actions code against the sim.
# Usage:
#   bash tools/start_robot129_ros_webrtc.sh --scene pick_place_counter   # once
#   bash tools/run_real_stack_task.sh "grasp the red block"
#   VLM_BACKEND=local bash tools/run_real_stack_task.sh "..."            # real local pipeline
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
if [[ -z "${GOOGLE_API_KEY:-}${GEMINI_API_KEY:-}" && -f "$HOME/.config/robot129_gemini.env" ]]; then
  set -a; source "$HOME/.config/robot129_gemini.env"; set +a
fi
exec "$micro" run -p "$rosenv" bash -c '
source "'"$rosenv"'/setup.bash"
export ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTHONNOUSERSITE=1
export PYTHONPATH="'"$venv"'/lib/python3.12/site-packages:${PYTHONPATH:-}"
exec "'"$venv"'/bin/python" -W ignore "'"$root"'/sim/scripts/sim_mm_actions_node.py" "$@"
' _ "$@"
