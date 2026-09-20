#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/camera_viewer"
if [[ -f "$state/viewer.pid" ]]; then
  pid="$(cat "$state/viewer.pid")"
  kill "$pid" 2>/dev/null || true
  rm -f "$state/viewer.pid"
fi
echo "STOPPED: Robot 129 wrist RGB-D viewer"
