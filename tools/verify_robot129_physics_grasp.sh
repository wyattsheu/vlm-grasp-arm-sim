#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
isaac="/mnt/HDD4/wyattsheu/IsaacLab"
gpu_row="$(nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader,nounits | awk -F, '{gsub(/ /,""); print ($2*100+$3/100) "," $1}' | sort -n | head -1)"
gpu="${gpu_row#*,}"
export CUDA_VISIBLE_DEVICES="$gpu" OMNI_KIT_ACCEPT_EULA=Y OMP_NUM_THREADS=4
export LD_PRELOAD="$isaac/.venv/lib/python3.12/site-packages/nvidia/cuda_runtime/lib/libcudart.so.12${LD_PRELOAD:+:$LD_PRELOAD}"
cd "$isaac"
uv run --no-sync python "$root/sim/scripts/verify_robot129_physics_grasp.py" --bundle "$root" --device cuda:0 --frames 300
ffmpeg -y -framerate 60 -i "$root/out/lesson_06/physics_grasp/frames/frame_%04d.png" -c:v libx264 -pix_fmt yuv420p -movflags +faststart "$root/out/lesson_06/physics_grasp/robot129_physics_grasp.mp4"
