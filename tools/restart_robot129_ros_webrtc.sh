#!/usr/bin/env bash
# Guarantee a clean slate: stop any running Robot 129 ROS/WebRTC session (if
# any) and start a fresh one. start_robot129_ros_webrtc.sh alone is a no-op
# when a session is already running, so a previous session's arm pose /
# gripper state / scene objects would otherwise carry over into your new
# script. This always relaunches Isaac Sim, so the robot comes back up at
# HOME and scene objects reset to their spawn state.
#
# Usage: same args as start_robot129_ros_webrtc.sh, e.g.
#   bash tools/restart_robot129_ros_webrtc.sh --scene pick_place
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
bash "$root/tools/stop_robot129_ros_webrtc.sh"
bash "$root/tools/start_robot129_ros_webrtc.sh" "$@"
