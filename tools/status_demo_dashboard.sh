#!/usr/bin/env bash
set -uo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/demo/dashboard_live"
if [[ -f "$state/server.pid" ]] && kill -0 "$(cat "$state/server.pid")" 2>/dev/null; then
  echo "RUNNING pid=$(cat "$state/server.pid")"
else
  echo "STOPPED"
fi
tail -n 20 "$state/server.log" 2>/dev/null || true
