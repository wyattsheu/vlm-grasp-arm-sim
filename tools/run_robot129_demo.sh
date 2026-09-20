#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
isaac="/mnt/HDD4/wyattsheu/IsaacLab"
gpu_row="$(nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader,nounits | awk -F, '{gsub(/ /,""); print ($2*100+$3/100) "," $1}' | sort -n | head -1)"
gpu="${gpu_row#*,}"
export CUDA_VISIBLE_DEVICES="$gpu" OMNI_KIT_ACCEPT_EULA=Y OMP_NUM_THREADS=4
export LD_PRELOAD="$isaac/.venv/lib/python3.12/site-packages/nvidia/cuda_runtime/lib/libcudart.so.12${LD_PRELOAD:+:$LD_PRELOAD}"
cd "$isaac"
uv run --no-sync python "$root/sim/scripts/run_robot129_demo.py" --bundle "$root" --device cuda:0 --frames 300
ffmpeg -y -framerate 30 -i "$root/out/integrated_demo/frames/frame_%04d.png" -c:v libx264 -pix_fmt yuv420p -movflags +faststart "$root/out/integrated_demo/robot129_simulation_demo.mp4"
ffprobe -v error -show_entries format=duration,size -of json "$root/out/integrated_demo/robot129_simulation_demo.mp4"
