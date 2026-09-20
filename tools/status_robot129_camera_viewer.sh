#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/camera_viewer"
if [[ -f "$state/viewer.pid" ]] && kill -0 "$(cat "$state/viewer.pid")" 2>/dev/null; then
  echo "RUNNING: PID=$(cat "$state/viewer.pid") http://127.0.0.1:8090"
  curl -fsS http://127.0.0.1:8090/stats
  echo
else
  echo "STOPPED: Robot 129 wrist RGB-D viewer"
fi
