#!/usr/bin/env bash
set -euo pipefail

bundle_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
isaac_root="/mnt/HDD4/wyattsheu/IsaacLab"
state_dir="$bundle_root/out/lesson_02/webrtc"
pid_file="$state_dir/server.pid"
log_file="$state_dir/server.log"

mkdir -p "$state_dir"
if [[ -f "$pid_file" ]] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
  echo "WebRTC Demo 已在執行，PID=$(cat "$pid_file")"
  "$bundle_root/tools/status_lesson_02_webrtc.sh"
  exit 0
fi

if pgrep -u "$(id -u)" -f "$isaac_root/.venv/bin/python[3]" >/dev/null 2>&1; then
  echo "STOP: 本帳號已有另一個 Isaac Python process。" >&2
  pgrep -a -u "$(id -u)" -f "$isaac_root/.venv/bin/python[3]" >&2 || true
  exit 3
fi

gpu_row="$(nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader,nounits \
  | awk -F, '{gsub(/ /, ""); score=($2*100)+($3/100); print score "," $1 "," $2 "," $3}' \
  | sort -t, -k1,1n | head -n1 | cut -d, -f2-)"
IFS=',' read -r gpu_index gpu_util gpu_memory_mib <<< "$gpu_row"

export CUDA_VISIBLE_DEVICES="$gpu_index"
export OMNI_KIT_ACCEPT_EULA=Y
export OMP_NUM_THREADS=4
export LD_PRELOAD="$isaac_root/.venv/lib/python3.12/site-packages/nvidia/cuda_runtime/lib/libcudart.so.12${LD_PRELOAD:+:$LD_PRELOAD}"

: > "$log_file"
cd "$isaac_root"
nohup uv run --no-sync python "$bundle_root/lessons/lesson_02/cartpole_webrtc_demo.py" \
  --livestream 2 --device cuda:0 > "$log_file" 2>&1 &
demo_pid=$!
echo "$demo_pid" > "$pid_file"
printf '%s\n' "$gpu_index" > "$state_dir/gpu.txt"

echo "啟動中：PID=$demo_pid，實體 GPU=$gpu_index，utilization=${gpu_util}%，VRAM=${gpu_memory_mib} MiB"
for _ in $(seq 1 45); do
  if ! kill -0 "$demo_pid" 2>/dev/null; then
    echo "FAIL: WebRTC Demo 提前結束。" >&2
    tail -n 80 "$log_file" >&2
    exit 5
  fi
  if grep -q '\[WEBRTC DEMO\] READY' "$log_file" \
    && ss -lnt 2>/dev/null | grep -q ':49100 '; then
    echo "PASS: WebRTC Cartpole Demo 已就緒。"
    echo "WebRTC signal port: 49100；stream port: 47998"
    echo "Log: $log_file"
    exit 0
  fi
  sleep 1
done

echo "FAIL: 45 秒內未同時出現 READY 與 TCP 49100 listening。" >&2
tail -n 80 "$log_file" >&2
exit 6
