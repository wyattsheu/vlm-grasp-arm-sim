#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/ros_webrtc_robot129"
if [[ ! -f "$state/server.pid" ]]; then
  echo "STOPPED: 無 PID file"
  exit 0
fi
pid="$(cat "$state/server.pid")"
if kill -0 "$pid" 2>/dev/null; then
  kill -TERM -- -"$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 15); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 1
  done
fi
rm -f "$state/server.pid"
echo "STOPPED: Robot 129 ROS WebRTC"
