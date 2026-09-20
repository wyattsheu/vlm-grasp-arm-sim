#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
micro=/mnt/HDD4/wyattsheu/tools/micromamba/micromamba
envroot=/mnt/HDD4/wyattsheu/env_robot129_ros
mkdir -p "$root/out/lesson_08"
"$micro" run -p "$envroot" bash -lc '
source /mnt/HDD4/wyattsheu/env_robot129_ros/setup.bash
source "'$root'/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim
mkdir -p "'$root'/out/lesson_08"
timeout --signal=INT --kill-after=3s 18s ros2 launch robot129_sim_bringup mock_control.launch.py > "'$root'/out/lesson_08/ros2_control.log" 2>&1 &
launcher=$!
cleanup() { kill -TERM "$launcher" 2>/dev/null || true; wait "$launcher" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
result=""
for i in $(seq 1 12); do
  result=$(timeout 2s ros2 control list_controllers --controller-manager /robot129_sim/controller_manager 2>/dev/null || true)
  if printf "%s" "$result" | grep -q "joint_state_broadcaster.*active" && printf "%s" "$result" | grep -q "arm_controller.*active" && printf "%s" "$result" | grep -q "gripper_controller.*active"; then break; fi
  sleep 1
done
export ROBOT129_CONTROLLERS="$result"
set +e
python - <<"PY"
import json,os
from pathlib import Path
text=os.environ.get("ROBOT129_CONTROLLERS","")
expected=["joint_state_broadcaster","arm_controller","gripper_controller"]
checks={name:any(name in line and "active" in line for line in text.splitlines()) for name in expected}
report={"status":"PASS" if all(checks.values()) else "FAIL","simulation_only":True,"hardware_plugin":"mock_components/GenericSystem","ros_domain_id":129,"controller_manager":"/robot129_sim/controller_manager","controllers":text.splitlines(),"checks":checks,"hardware_drivers":0}
Path("'$root'/out/lesson_08/ros2_control_mock.json").write_text(json.dumps(report,indent=2)+"\n")
print(json.dumps(report,indent=2))
raise SystemExit(0 if report["status"]=="PASS" else 1)
PY
rc=$?
set -e
exit "$rc"
'
