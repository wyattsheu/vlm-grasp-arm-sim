#!/usr/bin/env bash
# Start the real robot's two-stage local VLM pipeline (VLM_BACKEND=local) for the sim:
#   stage1  Qwen/Qwen3-VL-4B-Instruct  text-only action + label   -> :8010
#   stage2  allenai/Molmo2-4B          pointing / "not found"     -> :8012
# The real robot runs these as docker containers local_pipeline_stage1_qwen (:8000) and
# local_pipeline_stage2_molmo2 (:8002) from new_modle_test/production_client/
# launch_local_engines.sh, which is not in any repo. On this machine :8000/:8002 already
# belong to other programs, so the sim uses :8010/:8012 and tools/run_real_stack_task.sh
# points the unchanged LocalPipelineClient at them through STAGE1_BASE_URL/STAGE2_BASE_URL
# (the env vars that client already reads). Served model names match the real ones exactly,
# because the client sends them in every request.
#
# Normally not called by hand: tools/run_real_stack_task.sh starts both engines for each
# VLM_BACKEND=local run and stops them when the run ends, so they hold no GPU memory while idle.
# By hand: bash tools/start_robot129_local_engines.sh / bash tools/stop_robot129_local_engines.sh
# Weights: bash tools/download_robot129_vllm_models.sh legacy-mm
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/local_engines_robot129"
venv=/mnt/HDD4/wyattsheu/env_robot129_vllm
hf_home=/mnt/HDD4/wyattsheu/models/robot129_vllm/huggingface
mkdir -p "$state"
[[ -x "$venv/bin/vllm" ]] || { echo "FAIL: vLLM environment missing: $venv" >&2; exit 2; }

# Memory fractions are of the whole 96 GB card and sized from measured use (Qwen: 8.1 GiB
# weights + 0.9 GiB activations): other users' jobs leave only ~16-24 GB free per card, and
# vLLM refuses to start if the fraction exceeds what is free at that moment.
# name|model id|revision|port|gpu memory fraction|max model len|extra args
engines=(
  "stage1_qwen|Qwen/Qwen3-VL-4B-Instruct|ebb281ec70b05090aa6165b016eac8ec08e71b17|${STAGE1_PORT:-8010}|${STAGE1_GPU_UTIL:-0.12}|4096|--limit-mm-per-prompt {\"image\":0,\"video\":0}"
  "stage2_molmo2|allenai/Molmo2-4B|042abfa7a38879a376cec03d949eff0aefaa0600|${STAGE2_PORT:-8012}|${STAGE2_GPU_UTIL:-0.15}|8192|--limit-mm-per-prompt {\"image\":1,\"video\":0}"
)

# GPU with the most free memory; other users' jobs share these cards, so never assume.
pick_gpu() {
  nvidia-smi --query-gpu=index,memory.total,memory.used --format=csv,noheader,nounits \
    | awk -F, '{gsub(/ /,""); print ($2-$3) "," $1}' | sort -t, -k1 -nr | head -1 | cut -d, -f2
}

export HF_HOME="$hf_home" HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
export TOKENIZERS_PARALLELISM=false VLLM_WORKER_MULTIPROC_METHOD=spawn VLLM_USE_FLASHINFER_SAMPLER=0

started=()
for row in "${engines[@]}"; do
  IFS='|' read -r name model rev port util max_len extra <<< "$row"
  pidf="$state/$name.pid"
  if [[ -f "$pidf" ]] && kill -0 "$(cat "$pidf")" 2>/dev/null; then
    echo "RUNNING: $name PID=$(cat "$pidf") port=$port"
    continue
  fi
  snap="$hf_home/hub/models--${model//\//--}/snapshots/$rev"
  [[ -f "$snap/config.json" ]] || { echo "FAIL: $model weights missing ($snap); run tools/download_robot129_vllm_models.sh legacy-mm" >&2; exit 3; }
  if ss -lnt 2>/dev/null | grep -q ":$port "; then
    echo "FAIL: port $port already in use" >&2; ss -lntp 2>/dev/null | grep ":$port " >&2 || true; exit 4
  fi
  gpu="$(pick_gpu)"
  : > "$state/$name.log"
  # shellcheck disable=SC2086
  CUDA_VISIBLE_DEVICES="$gpu" nohup setsid "$venv/bin/vllm" serve "$snap" \
    --host 127.0.0.1 --port "$port" --served-model-name "$model" \
    --dtype bfloat16 --gpu-memory-utilization "$util" --max-model-len "$max_len" \
    --max-num-seqs 1 --trust-remote-code $extra \
    > "$state/$name.log" 2>&1 &
  echo "$!" > "$pidf"
  echo "$gpu" > "$state/$name.gpu"
  echo "啟動中：$name（$model）GPU=$gpu port=$port memory=$util"
  started+=("$name|$port")
  # vLLM's startup memory profiling races with another process allocating on the same
  # card (DEVLOG 2026-08-28); load one engine at a time so they never profile together.
  for tick in $(seq 1 90); do
    kill -0 "$(cat "$pidf")" 2>/dev/null || { echo "FAIL: $name exited" >&2; tail -n 60 "$state/$name.log" >&2; rm -f "$pidf"; exit 1; }
    curl -fsS --max-time 3 "http://127.0.0.1:$port/health" >/dev/null 2>&1 && break
    (( tick % 6 == 0 )) && echo "  $name 載入中：$((tick * 5)) 秒"
    sleep 5
  done
  curl -fsS --max-time 3 "http://127.0.0.1:$port/health" >/dev/null 2>&1 \
    || { echo "FAIL: $name not healthy after 450 s" >&2; tail -n 60 "$state/$name.log" >&2; exit 1; }
  echo "PASS: $name ready at http://127.0.0.1:$port/v1"
done
echo "STAGE1_BASE_URL=http://127.0.0.1:${STAGE1_PORT:-8010}/v1 STAGE2_BASE_URL=http://127.0.0.1:${STAGE2_PORT:-8012}/v1"
