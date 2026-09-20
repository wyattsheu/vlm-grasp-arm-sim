#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
isaac=/mnt/HDD4/wyattsheu/IsaacLab
state="$root/out/webrtc_robot129"
mkdir -p "$state"
if [[ -f "$state/server.pid" ]] && kill -0 "$(cat "$state/server.pid")" 2>/dev/null; then
  echo "Robot 129 WebRTC 已在執行，PID=$(cat "$state/server.pid")"
  exit 0
fi
if ss -lnt 2>/dev/null | grep -q ':49100 '; then
  echo "STOP: WebRTC signaling port 49100 已被其他程序占用。" >&2
  ss -lntp 2>/dev/null | grep ':49100 ' >&2 || true
  exit 4
fi
gpu_row="$(nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader,nounits | awk -F, '{gsub(/ /,""); print ($2*100+$3/100) "," $1 "," $2 "," $3}' | sort -n | head -1)"
IFS=',' read -r _ gpu util memory <<< "$gpu_row"
export CUDA_VISIBLE_DEVICES="$gpu" OMNI_KIT_ACCEPT_EULA=Y OMP_NUM_THREADS=4
export ISAAC_LAB_ENABLE_ISAAC_RTX_PER_ENV_SCENE_PARTITION=0
public_ip="${ROBOT129_WEBRTC_PUBLIC_IP:-140.113.203.85}"
export LD_PRELOAD="$isaac/.venv/lib/python3.12/site-packages/nvidia/cuda_runtime/lib/libcudart.so.12${LD_PRELOAD:+:$LD_PRELOAD}"
: > "$state/server.log"
cd "$isaac"
nohup setsid uv run --no-sync python "$root/sim/scripts/verify_robot129_physics_grasp.py" \
  --bundle "$root" --device cuda:0 --frames 300 --livestream 2 \
  --kit_args "--/app/window/width=1280 --/app/window/height=720 --/exts/omni.kit.livestream.app/primaryStream/targetFps=30 --/exts/omni.kit.livestream.app/primaryStream/publicIp=$public_ip --/exts/omni.kit.livestream.app/primaryStream/allowDynamicResize=false --/rtx/hydra/readTransformsFromFabricInRenderDelegate=0 --/renderer/multiGpu/enabled=false" \
  --realtime --warmup-frames 90 --preroll-seconds 10 --hold-seconds -1 > "$state/server.log" 2>&1 &
pid=$!
echo "$pid" > "$state/server.pid"
echo "$gpu" > "$state/gpu.txt"
cleanup_failed() {
  kill -TERM -- -"$pid" 2>/dev/null || true
  rm -f "$state/server.pid"
}
echo "初始化中 [1/2]：載入 Isaac Sim，PID=$pid，GPU=$gpu，原始 utilization=${util}%，VRAM=${memory} MiB"
echo "WebRTC 網路：public IP=$public_ip，固定 1280x720@30fps"
phase=loading
for _ in $(seq 1 60); do
  if ! kill -0 "$pid" 2>/dev/null; then tail -n 100 "$state/server.log"; cleanup_failed; exit 1; fi
  if grep -q '\[Fatal\].*livestream\|NVST_R_' "$state/server.log"; then
    echo "FAIL: livestream extension 回報 Fatal。" >&2
    tail -n 80 "$state/server.log" >&2
    cleanup_failed
    exit 1
  fi
  if [[ "$phase" == loading ]] && grep -q '\[ROBOT129 WEBRTC\] WARMING_UP' "$state/server.log"; then
    echo "初始化中 [2/2]：RTX shader、viewport 與 WebRTC geometry 暖機"
    phase=warming
  fi
  if [[ "$phase" == warming ]] && grep -q '\[ROBOT129 WEBRTC\] VERIFYING_VIEWPORT' "$state/server.log"; then
    echo "驗證中：截取 WebRTC 實際 viewport，排除全黑或純色畫面"
    phase=verifying
  fi
  if grep -q '\[ROBOT129 WEBRTC\] READY' "$state/server.log" && ss -lnt 2>/dev/null | grep -q ':49100 '; then
    cat > "$state/acceptance.json" <<JSON
{
  "status": "PASS",
  "scope": "WEBRTC_SERVER_READY",
  "simulation_only": true,
  "gpu_index": $gpu,
  "public_ip": "$public_ip",
  "kit_event_updates_per_step": 2,
  "signal_port": 49100,
  "stream_port": 47998,
  "framebuffer": "1280x720@30fps",
  "dynamic_resize": false,
  "viewport_probe": "out/webrtc_robot129/viewport_probe.png",
  "warmup_frames_before_ready": 90,
  "sensor_render_product_enabled": false,
  "replicator_required_for_live_view": false,
  "fabric_render_delegate_reads": false,
  "per_env_rtx_scene_partition": false,
  "viewport_eye": [0.92, 0.78, 0.72],
  "viewport_target": [0.27, 0.0, 0.36],
  "background": "deep_gray",
  "external_client_connected": false,
  "hardware_drivers": 0
}
JSON
    echo "PASS: Robot 129 WebRTC 已就緒，signal port 49100，stream port 47998"
    echo "log=$state/server.log"
    exit 0
  fi
  sleep 1
done
echo "FAIL: 60 秒內未就緒" >&2
tail -n 100 "$state/server.log" >&2
cleanup_failed
exit 1
