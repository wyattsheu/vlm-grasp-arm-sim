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
    # Same models as the real robot's local_pipeline_client.py (stage1 / stage2);
    # revisions pinned to what this cache first resolved (2026-09-15).
    hf download Qwen/Qwen3-VL-4B-Instruct --revision ebb281ec70b05090aa6165b016eac8ec08e71b17
    hf download allenai/Molmo2-4B --revision 042abfa7a38879a376cec03d949eff0aefaa0600
    ;;
  *)
    echo "usage: $0 unified|legacy-mm" >&2
    exit 2
    ;;
esac
