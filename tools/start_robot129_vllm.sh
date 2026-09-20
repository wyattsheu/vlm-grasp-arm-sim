#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/vllm_robot129"
venv=/mnt/HDD4/wyattsheu/env_robot129_vllm
hf_home=/mnt/HDD4/wyattsheu/models/robot129_vllm/huggingface
revision=0c351dd01ed87e9c1b53cbc748cba10e6187ff3b
model="$hf_home/hub/models--Qwen--Qwen3-VL-8B-Instruct/snapshots/$revision"
port="${ROBOT129_VLLM_PORT:-8001}"
util="${ROBOT129_VLLM_GPU_MEMORY_UTILIZATION:-0.30}"
max_len="${ROBOT129_VLLM_MAX_MODEL_LEN:-8192}"
mkdir -p "$state"

if [[ -f "$state/server.pid" ]] && kill -0 "$(cat "$state/server.pid")" 2>/dev/null; then
  echo "RUNNING: Robot 129 vLLM PID=$(cat "$state/server.pid")"
  bash "$root/tools/status_robot129_vllm.sh"
  exit 0
fi
[[ -x "$venv/bin/vllm" ]] || { echo "FAIL: vLLM environment missing: $venv" >&2; exit 2; }
[[ -f "$model/config.json" ]] || { echo "FAIL: model snapshot missing: $model" >&2; exit 3; }
if ss -lnt 2>/dev/null | grep -q ":$port "; then
  echo "FAIL: local port $port is already in use" >&2
  ss -lntp 2>/dev/null | grep ":$port " >&2 || true
  exit 4
fi

gpu_row="$(nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader,nounits | awk -F, '{gsub(/ /,""); print ($2*100+$3/100) "," $1 "," $2 "," $3}' | sort -n | head -1)"
IFS=',' read -r _ gpu gpu_util memory <<< "$gpu_row"
export CUDA_VISIBLE_DEVICES="$gpu"
export HF_HOME="$hf_home" HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
export TOKENIZERS_PARALLELISM=false VLLM_WORKER_MULTIPROC_METHOD=spawn
export VLLM_USE_FLASHINFER_SAMPLER=0
: > "$state/server.log"
cd "$root"
nohup setsid "$venv/bin/vllm" serve "$model" \
  --host 127.0.0.1 --port "$port" \
  --served-model-name qwen-vl decision-qwen \
  --dtype bfloat16 \
  --gpu-memory-utilization "$util" \
  --max-model-len "$max_len" \
  --max-num-seqs 1 \
  --limit-mm-per-prompt '{"image":1,"video":0}' \
  --generation-config vllm \
  --trust-remote-code \
  > "$state/server.log" 2>&1 &
pid=$!
echo "$pid" > "$state/server.pid"
echo "$gpu" > "$state/gpu.txt"
echo "啟動中：PID=$pid，實體 GPU=$gpu，原 utilization=${gpu_util}%，VRAM=${memory} MiB"
echo "限制：GPU memory utilization=$util，max model length=$max_len，max sequences=1"

for tick in $(seq 1 60); do
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "FAIL: vLLM process exited" >&2
    tail -n 120 "$state/server.log" >&2
    exit 1
  fi
  if curl -fsS --max-time 3 "http://127.0.0.1:$port/v1/models" > "$state/models.json"; then
    echo "PASS: Robot 129 vLLM ready at http://127.0.0.1:$port/v1"
    echo "served names: qwen-vl, decision-qwen"
    exit 0
  fi
  if (( tick % 3 == 0 )); then
    echo "模型載入中：$((tick * 5)) 秒"
  fi
  sleep 5
done
echo "FAIL: vLLM did not become ready within 300 seconds" >&2
tail -n 120 "$state/server.log" >&2
kill -TERM -- "-$pid" 2>/dev/null || true
exit 1
