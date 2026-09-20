#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/vllm_robot129"
if [[ ! -f "$state/server.pid" ]]; then
  echo "STOPPED: no vLLM PID file"
  exit 0
fi
pid="$(cat "$state/server.pid")"
if kill -0 "$pid" 2>/dev/null; then
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 1
  done
  if kill -0 "$pid" 2>/dev/null; then
    kill -KILL -- "-$pid" 2>/dev/null || true
  fi
fi
python3 -c 'from pathlib import Path; Path("'"$state/server.pid"'").unlink(missing_ok=True)'
echo "STOPPED: Robot 129 vLLM，模型 cache 保留"
nvidia-smi --query-gpu=index,memory.used,utilization.gpu --format=csv,noheader,nounits
