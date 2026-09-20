#!/usr/bin/env bash
set -euo pipefail

bundle_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
isaac_root="/mnt/HDD4/wyattsheu/IsaacLab"
lesson_script="$bundle_root/lessons/lesson_02/cartpole_articulation.py"

if [[ ! -x "$isaac_root/.venv/bin/python" ]]; then
  echo "FAIL: Isaac Python is missing: $isaac_root/.venv/bin/python" >&2
  exit 2
fi

if pgrep -u "$(id -u)" -f "$isaac_root/.venv/bin/python[3]" >/dev/null 2>&1; then
  echo "STOP: an Isaac process from this account is already running." >&2
  pgrep -a -u "$(id -u)" -f "$isaac_root/.venv/bin/python[3]" >&2 || true
  exit 3
fi

gpu_row="$(nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader,nounits \
  | awk -F, '{gsub(/ /, ""); score=($2*100)+($3/100); print score "," $1 "," $2 "," $3}' \
  | sort -t, -k1,1n | head -n1 | cut -d, -f2-)"
IFS=',' read -r gpu_index gpu_util gpu_memory_mib <<< "$gpu_row"
gpu_index="${gpu_index//[[:space:]]/}"
gpu_util="${gpu_util//[[:space:]]/}"
gpu_memory_mib="${gpu_memory_mib//[[:space:]]/}"

echo "Selected GPU $gpu_index: utilization=${gpu_util}%, memory=${gpu_memory_mib} MiB"
echo "Policy: use the least-loaded GPU without waiting for an idle threshold."

run_id="$(date -u +%Y%m%dT%H%M%SZ)"
run_dir="$bundle_root/out/lesson_02/$run_id"
mkdir -p "$run_dir"

export CUDA_VISIBLE_DEVICES="$gpu_index"
export OMNI_KIT_ACCEPT_EULA=Y
export OMP_NUM_THREADS=4
export LD_PRELOAD="$isaac_root/.venv/lib/python3.12/site-packages/nvidia/cuda_runtime/lib/libcudart.so.12${LD_PRELOAD:+:$LD_PRELOAD}"

echo "Running official Cartpole articulation walkthrough."
echo "Artifacts will be written to: $run_dir"
cd "$isaac_root"
set +e
uv run --no-sync python "$lesson_script" \
  --output-dir "$run_dir" \
  --steps 240 \
  --device cuda:0 2>&1 | tee "$run_dir/console.log"
sim_status=${PIPESTATUS[0]}
set -e

if (( sim_status != 0 )); then
  echo "FAIL: Isaac lesson process exited with status $sim_status." >&2
  exit "$sim_status"
fi

if [[ ! -s "$run_dir/cartpole.png" ]]; then
  echo "FAIL: viewport capture was not written: $run_dir/cartpole.png" >&2
  exit 5
fi

if ! grep -q '"status": "PASS"' "$run_dir/articulation_observation.json"; then
  echo "FAIL: articulation report did not record PASS." >&2
  exit 6
fi

echo "$run_dir" > "$bundle_root/out/lesson_02/latest_run.txt"
echo "PASS: Lesson 2 runtime completed."
echo "Open this image: $run_dir/cartpole.png"
echo "Read this report: $run_dir/articulation_observation.json"
