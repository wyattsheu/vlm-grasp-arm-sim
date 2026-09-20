#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/grasp_motion/live"
if [[ -f "$state/server.pid" ]] && kill -0 "$(cat "$state/server.pid")" 2>/dev/null; then
  echo "RUNNING pid=$(cat "$state/server.pid") gpu=$(cat "$state/gpu.txt" 2>/dev/null || echo '?')"
else
  echo "STOPPED"
fi
tail -n 20 "$state/server.log" 2>/dev/null || true
