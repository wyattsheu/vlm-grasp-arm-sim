#!/usr/bin/env bash
# One-command demo: make sure Isaac Sim + ROS bridge + WebRTC are running,
# then drive the arm/gripper through robot129_control_api.
#
# Watch it live over WebRTC while this runs:
#   Server IP: 140.113.203.85   Signaling port: 49100   Stream port: 47998
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$root"
bash tools/start_robot129_ros_webrtc.sh
bash tools/run_with_robot129_control.sh tools/demo_robot129_control_api.py
