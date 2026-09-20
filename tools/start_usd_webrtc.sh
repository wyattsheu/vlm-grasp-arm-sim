#!/usr/bin/env bash
# Generic USD WebRTC viewer -- load any .usd/.usda/.usdc file, auto-frame the
# camera, add a deep-gray floor + grid + 10cm reference cube + world axes,
# print an inspection report, and stream it over WebRTC.
#
# Usage:
#   bash tools/start_usd_webrtc.sh --usd /absolute/path/model.usd [--physics]
#
# Then connect a WebRTC client to:
#   Server IP:      140.113.203.85 (or ROBOT129_WEBRTC_PUBLIC_IP)
#   Signaling port: 49100
#   Stream port:    47998
#
# Stop with: bash tools/stop_usd_webrtc.sh
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
isaac=/mnt/HDD4/wyattsheu/IsaacLab
state="$root/out/webrtc_usd_viewer"
mkdir -p "$state"

usd_path=""
extra_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --usd) usd_path="$2"; shift 2 ;;
    --physics) extra_args+=(--physics); shift ;;
    *) extra_args+=("$1"); shift ;;
  esac
done
if [[ -z "$usd_path" ]]; then
  echo "USAGE: start_usd_webrtc.sh --usd /absolute/path/model.usd [--physics]" >&2
  exit 2
fi
if [[ "$usd_path" != /* ]]; then
  echo "STOP: --usd must be an absolute path (got: $usd_path)" >&2
  exit 2
fi
if [[ ! -f "$usd_path" ]]; then
  echo "STOP: file not found: $usd_path" >&2
  exit 2
fi

if [[ -f "$state/server.pid" ]] && kill -0 "$(cat "$state/server.pid")" 2>/dev/null; then
  echo "USD WebRTC viewer 已在執行，PID=$(cat "$state/server.pid")。先 stop_usd_webrtc.sh 再重開一個不同的模型。"
  exit 0
fi
if ss -lnt 2>/dev/null | grep -q ':49100 '; then
  echo "STOP: WebRTC signaling port 49100 已被其他程序占用（可能是 Robot 129 viewer 還在跑）。" >&2
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
nohup setsid uv run --no-sync python "$root/sim/scripts/view_usd_webrtc.py" \
  --usd "$usd_path" --out-dir "$state" "${extra_args[@]}" \
  --livestream 2 --hold-seconds -1 \
  --kit_args "--/app/window/width=1280 --/app/window/height=720 --/exts/omni.kit.livestream.app/primaryStream/targetFps=30 --/exts/omni.kit.livestream.app/primaryStream/publicIp=$public_ip --/exts/omni.kit.livestream.app/primaryStream/allowDynamicResize=false --/rtx/hydra/readTransformsFromFabricInRenderDelegate=0 --/renderer/multiGpu/enabled=false --/app/useFabricSceneDelegate=0 --enable omni.physx.ui" \
  > "$state/server.log" 2>&1 &
pid=$!
echo "$pid" > "$state/server.pid"
echo "$gpu" > "$state/gpu.txt"
echo "$usd_path" > "$state/usd_path.txt"
cleanup_failed() {
  kill -TERM -- -"$pid" 2>/dev/null || true
  rm -f "$state/server.pid"
}
echo "初始化中：載入 $usd_path，PID=$pid，GPU=$gpu，原始 utilization=${util}%，VRAM=${memory} MiB"
echo "WebRTC 網路：public IP=$public_ip，1280x720@30fps"
for _ in $(seq 1 90); do
  if ! kill -0 "$pid" 2>/dev/null; then tail -n 100 "$state/server.log"; cleanup_failed; exit 1; fi
  if grep -q '\[Fatal\].*livestream\|NVST_R_\|FATAL - USD file not found' "$state/server.log"; then
    echo "FAIL: 初始化失敗。" >&2
    tail -n 80 "$state/server.log" >&2
    cleanup_failed
    exit 1
  fi
  if grep -q '\[USD WEBRTC\] ---- END REPORT ----' "$state/server.log"; then
    echo "--- 模型檢查報告 ---"
    grep '\[USD WEBRTC\]' "$state/server.log" | sed -n '/INSPECTION REPORT/,/END REPORT/p'
  fi
  if grep -q '\[USD WEBRTC\] READY' "$state/server.log" && ss -lnt 2>/dev/null | grep -q ':49100 '; then
    echo "PASS: USD WebRTC viewer 已就緒，signal port 49100，stream port 47998"
    echo "檔案: $usd_path"
    echo "screenshot: $state/screenshot.png"
    echo "report: $state/report.json"
    echo "log=$state/server.log"
    exit 0
  fi
  sleep 1
done
echo "FAIL: 90 秒內未就緒" >&2
tail -n 100 "$state/server.log" >&2
cleanup_failed
exit 1
