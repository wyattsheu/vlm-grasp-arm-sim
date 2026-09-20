#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
cleanup() { bash "$root/tools/stop_robot129_vllm.sh"; }
trap cleanup EXIT INT TERM
bash "$root/tools/start_robot129_vllm.sh"
source /mnt/HDD4/wyattsheu/env_robot129_research/bin/activate
export PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$root/research/src"
export ROBOT129_LOCAL_VLLM_MODEL=qwen-vl
export ROBOT129_LOCAL_VLLM_URL=http://127.0.0.1:8001/v1/chat/completions
run_stamp="$(date -u +%Y%m%dT%H%M%SZ)"
run_dir="$root/research/out/runs/${run_stamp}_robot129_qwen"
instruction="${ROBOT129_VLM_INSTRUCTION:-Pick the red cube and place it on the visible light-colored grid surface.}"
python "$root/research/scripts/run_grounding.py" \
  --image "$root/out/integrated_demo/scene_bundle/rgb.png" \
  --instruction "$instruction" \
  --scene-id robot129_isaac_live_qwen \
  --mode ab --backend local \
  --replicate-id "robot129-live-$run_stamp" \
  --phase-id robot129-live \
  --output "$run_dir"
python - "$run_dir" "$root/out/vllm_robot129/live_grounding_acceptance.json" <<'PY'
import json
from pathlib import Path
import sys
run = Path(sys.argv[1])
output = Path(sys.argv[2])
result = json.loads((run / 'result.json').read_text())
plan = json.loads((run / 'plan.json').read_text())
report = {
    'status': 'PASS' if result.get('status') == 'PARSED' and result.get('actual_calls', 0) > 0 else 'FAIL',
    'scope': 'LIVE_QWEN_VLM_GROUNDING_NO_EXECUTION',
    'simulation_only': True,
    'run_dir': str(run),
    'result': result,
    'semantic_status': result.get('semantic_status'),
    'localization_status': result.get('localization_status'),
    'step_count': len(plan.get('steps', [])),
    'execution_status': result.get('execution_status'),
    'robot_commands': 0,
    'overlay': str(run / 'overlay.png'),
}
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(report, ensure_ascii=False, indent=2))
raise SystemExit(0 if report['status'] == 'PASS' else 1)
PY
cleanup
trap - EXIT INT TERM
echo "PASS: live grounding 完成且 vLLM 已停止，不再占用 GPU。"
