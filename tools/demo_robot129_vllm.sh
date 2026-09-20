#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
cleanup() { bash "$root/tools/stop_robot129_vllm.sh"; }
trap cleanup EXIT INT TERM
bash "$root/tools/start_robot129_vllm.sh"
source /mnt/HDD4/wyattsheu/env_robot129_vllm/bin/activate
python "$root/tools/test_robot129_vllm.py" "$@"
cleanup
trap - EXIT INT TERM
echo "PASS: vLLM smoke test 完成且服務已停止，不再占用 GPU。"
