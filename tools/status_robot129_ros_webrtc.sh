#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/ros_webrtc_robot129"
if [[ -f "$state/server.pid" ]] && kill -0 "$(cat "$state/server.pid")" 2>/dev/null; then
  echo "RUNNING: PID=$(cat "$state/server.pid"), GPU=$(cat "$state/gpu.txt" 2>/dev/null || echo unknown)"
  ss -lntup 2>/dev/null | grep -E ':(49100|47998)\b' || true
  grep -E '\[ROBOT129 ROS WEBRTC\] (VIEWPORT_PROBE|READY|ACCEPT|REJECT|COMPLETE)' "$state/server.log" | tail -n 12 || true
else
  echo "STOPPED: Robot 129 ROS WebRTC 未執行"
fi
