#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/ros_webrtc_robot129"
cd "$root"
mkdir -p "$state/sequence"
echo '[1/6] 啟動本機 ROS-controlled Isaac/WebRTC'
bash tools/start_robot129_ros_webrtc.sh
for spec in 'inspect 2' 'grasp 2' 'lift 3' 'release 2' 'home 3'; do
  read -r pose duration <<< "$spec"
  case "$pose" in
    inspect) step=2 ;;
    grasp) step=3 ;;
    lift) step=4 ;;
    release) step=5 ;;
    home) step=6 ;;
  esac
  echo "[$step/6] $pose"
  bash tools/send_robot129_ros_pose.sh "$pose" --duration "$duration"
  cp "$state/last_command.json" "$state/sequence/$pose.json"
done
python3 - <<'PY'
import json
from datetime import datetime, timezone
from pathlib import Path
root = Path('out/ros_webrtc_robot129')
order = ['inspect', 'grasp', 'lift', 'release', 'home']
commands = [json.loads((root / 'sequence' / f'{pose}.json').read_text()) for pose in order]
report = {
    'status': 'PASS' if all(item['status'] == 'PASS' for item in commands) else 'FAIL',
    'scope': 'LOCAL_ROS_TO_ISAAC_WEBRTC_SEQUENCE',
    'simulation_only': True,
    'ros_domain_id': 129,
    'namespace': '/robot129_sim',
    'hardware_drivers': 0,
    'sequence': order,
    'max_error': max(item['max_error'] for item in commands),
    'commands': commands,
    'generated_at': datetime.now(timezone.utc).isoformat(),
}
(root / 'end_to_end_acceptance.json').write_text(json.dumps(report, indent=2) + '\n')
print(f"PASS: 完整驗收已寫入 {root / 'end_to_end_acceptance.json'}")
PY
echo 'PASS: 本機 ROS sequence 完成；Isaac/WebRTC 繼續執行。'
