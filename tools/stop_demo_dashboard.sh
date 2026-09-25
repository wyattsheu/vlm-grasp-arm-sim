#!/usr/bin/env bash
set -uo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/demo/dashboard_live"
pattern="tools/rerun_dashboard.py"

if [[ -f "$state/server.pid" ]]; then
  pid="$(cat "$state/server.pid")"
  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM -- -"$pid" 2>/dev/null
    kill -TERM "$pid" 2>/dev/null
  fi
fi
pkill -TERM -f "$pattern" 2>/dev/null

for _ in $(seq 1 10); do
  pgrep -f "$pattern" >/dev/null 2>&1 || break
  sleep 1
done
if pgrep -f "$pattern" >/dev/null 2>&1; then
  pkill -KILL -f "$pattern" 2>/dev/null
  sleep 1
fi

rm -f "$state/server.pid"
if pgrep -f "$pattern" >/dev/null 2>&1; then
  echo "FAILED to stop; still running:" >&2
  pgrep -af "$pattern" >&2
  exit 1
fi
echo "stopped"
