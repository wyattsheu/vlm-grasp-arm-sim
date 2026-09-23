#!/usr/bin/env bash
# Stop the two local VLM engines started by start_robot129_local_engines.sh (weights stay on disk).
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/local_engines_robot129"
for name in stage1_qwen stage2_molmo2; do
  pidf="$state/$name.pid"
  [[ -f "$pidf" ]] || { echo "STOPPED: $name（無 PID file）"; continue; }
  pid="$(cat "$pidf")"
  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || break; sleep 1; done
    kill -0 "$pid" 2>/dev/null && kill -KILL -- "-$pid" 2>/dev/null || true
  fi
  rm -f "$pidf"
  echo "STOPPED: $name"
done
