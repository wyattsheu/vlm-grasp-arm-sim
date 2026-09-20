#!/usr/bin/env bash
# One command: get a live WebRTC view of the arm AND run the full grasp+motion
# demo against it, back to back. Composes three pieces that already exist and
# are each independently tested -- this script is just the glue:
#   tools/stop_any_webrtc.sh          -- frees port 49100 (any owner, not just ours)
#   tools/start_robot129_ros_webrtc.sh -- starts Isaac with WebRTC streaming on
#   tools/run_grasp_motion_demo.sh     -- reset -> candidates -> MTC plan -> execute
#
# This WILL interrupt whatever WebRTC session (this project's or anyone else's,
# e.g. parcel-forge) is currently holding port 49100 -- that is the whole point
# (see docs/dev_guide_paper_core_and_dashboard_plan.md §4). Do not run this if
# someone else's WebRTC session on this ports is actively in use.
#
# Usage:
#   tools/run_grasp_demo_live.sh [scene] [-- run_grasp_motion_demo.sh args...]
#   tools/run_grasp_demo_live.sh pick_place
#   tools/run_grasp_demo_live.sh pick_place_hammer -- --record
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"

scene="${1:-pick_place}"
[[ $# -gt 0 ]] && shift
if [[ "${1:-}" == "--" ]]; then shift; fi
demo_args=("$@")

echo "[1/3] freeing WebRTC port 49100 (stopping any current session, ours or not)"
bash "$root/tools/stop_any_webrtc.sh"

echo "[2/3] starting Isaac with live WebRTC streaming (scene=$scene)"
bash "$root/tools/start_robot129_ros_webrtc.sh" --scene "$scene"
echo "WebRTC: 140.113.203.85:49100 (signal) / 47998 (stream)"

echo "[3/3] running the full grasp+motion demo against it"
bash "$root/tools/run_grasp_motion_demo.sh" "${demo_args[@]}"
