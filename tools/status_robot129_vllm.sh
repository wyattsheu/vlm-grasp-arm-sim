#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/vllm_robot129"
port="${ROBOT129_VLLM_PORT:-8001}"
if [[ -f "$state/server.pid" ]] && kill -0 "$(cat "$state/server.pid")" 2>/dev/null; then
  echo "RUNNING: PID=$(cat "$state/server.pid"), GPU=$(cat "$state/gpu.txt" 2>/dev/null || echo unknown)"
  curl -fsS --max-time 3 "http://127.0.0.1:$port/v1/models" || true
  echo
else
  echo "STOPPED: Robot 129 vLLM 未執行；模型 cache 仍在磁碟"
fi
nvidia-smi --query-gpu=index,memory.used,utilization.gpu --format=csv,noheader,nounits
