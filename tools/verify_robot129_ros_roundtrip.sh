#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
isaac="/mnt/HDD4/wyattsheu/IsaacLab"
rosroot="$isaac/.venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy"
gpu_row="$(nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader,nounits | awk -F, '{gsub(/ /,""); print ($2*100+$3/100) "," $1}' | sort -n | head -1)"
gpu="${gpu_row#*,}"
export CUDA_VISIBLE_DEVICES="$gpu" OMNI_KIT_ACCEPT_EULA=Y OMP_NUM_THREADS=4
export ROS_DISTRO=jazzy ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$rosroot/rclpy" LD_LIBRARY_PATH="$rosroot/lib:$isaac/.venv/lib/python3.12/site-packages/nvidia/cuda_runtime/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export LD_PRELOAD="$isaac/.venv/lib/python3.12/site-packages/nvidia/cuda_runtime/lib/libcudart.so.12${LD_PRELOAD:+:$LD_PRELOAD}"
cd "$isaac"
uv run --no-sync python "$root/sim/scripts/verify_robot129_ros_roundtrip.py" --bundle "$root" --device cuda:0
