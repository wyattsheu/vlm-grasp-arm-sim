#!/usr/bin/env bash
# Headless (no WebRTC/livestream, no port 49100 usage) launcher for the grasp/motion
# pick-and-place scene. Unlike tools/start_robot129_ros_webrtc.sh this does not require
# stopping any other WebRTC session and writes its own state directory so it can run
# alongside one.
#
# Usage:
#   bash tools/start_robot129_grasp_sim.sh [--scene marker|pick_place] [--seed N] [extra args...]
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
isaac=/mnt/HDD4/wyattsheu/IsaacLab
rosroot="$isaac/.venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy"
state="$root/out/grasp_motion/live"
mkdir -p "$state"

if [[ -f "$state/server.pid" ]] && kill -0 "$(cat "$state/server.pid")" 2>/dev/null; then
  echo "Grasp/motion sim already running, PID=$(cat "$state/server.pid")"
  exit 0
fi

gpu_row="$(nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader,nounits | awk -F, '{gsub(/ /,""); print ($2*100+$3/100) "," $1 "," $2 "," $3}' | sort -n | head -1)"
IFS=',' read -r _ gpu util memory <<< "$gpu_row"
export CUDA_VISIBLE_DEVICES="$gpu" OMNI_KIT_ACCEPT_EULA=Y OMP_NUM_THREADS=4
export ISAAC_LAB_ENABLE_ISAAC_RTX_PER_ENV_SCENE_PARTITION=0
export ROS_DISTRO=jazzy ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$rosroot/rclpy${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$rosroot/lib:$isaac/.venv/lib/python3.12/site-packages/nvidia/cuda_runtime/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export LD_PRELOAD="$isaac/.venv/lib/python3.12/site-packages/nvidia/cuda_runtime/lib/libcudart.so.12${LD_PRELOAD:+:$LD_PRELOAD}"

: > "$state/server.log"
cd "$isaac"
nohup setsid uv run --no-sync python "$root/sim/scripts/run_robot129_ros_webrtc.py" \
  --bundle "$root" --device cuda:0 --record-only "$@" \
  > "$state/server.log" 2>&1 &
pid=$!
echo "$pid" > "$state/server.pid"
echo "$gpu" > "$state/gpu.txt"

cleanup_failed() {
  kill -TERM -- -"$pid" 2>/dev/null || true
  rm -f "$state/server.pid"
}
echo "初始化中：grasp/motion headless sim，PID=$pid，GPU=$gpu，utilization=${util}%，VRAM=${memory} MiB"
for _ in $(seq 1 120); do
  if ! kill -0 "$pid" 2>/dev/null; then
    tail -n 150 "$state/server.log"
    cleanup_failed
    exit 1
  fi
  if grep -q '\[ROBOT129 ROS WEBRTC\] READY' "$state/server.log" 2>/dev/null; then
    echo "READY"
    exit 0
  fi
  sleep 1
done
echo "TIMEOUT waiting for READY" >&2
tail -n 150 "$state/server.log" >&2
cleanup_failed
exit 1
