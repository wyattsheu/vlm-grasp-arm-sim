#!/usr/bin/env bash
set -euo pipefail
profile="${1:-unified}"
source /mnt/HDD4/wyattsheu/env_robot129_vllm/bin/activate
export HF_HOME=/mnt/HDD4/wyattsheu/models/robot129_vllm/huggingface
case "$profile" in
  unified)
    hf download Qwen/Qwen3-VL-8B-Instruct --revision 0c351dd01ed87e9c1b53cbc748cba10e6187ff3b
    ;;
  legacy-mm)
    hf download Qwen/Qwen3-VL-4B-Instruct
    hf download allenai/Molmo2-4B
    ;;
  *)
    echo "usage: $0 unified|legacy-mm" >&2
    exit 2
    ;;
esac
