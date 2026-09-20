#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
micro=/mnt/HDD4/wyattsheu/tools/micromamba/micromamba
envroot=/mnt/HDD4/wyattsheu/env_robot129_ros
mkdir -p "$root/out/lesson_09"
exec "$micro" run -p "$envroot" bash -lc '
set -eo pipefail
source /mnt/HDD4/wyattsheu/env_robot129_ros/setup.bash
source "'$root'/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim
export LD_LIBRARY_PATH=/mnt/HDD4/wyattsheu/env_robot129_ros/lib:${LD_LIBRARY_PATH:-}
rm -f "'$root'/out/lesson_09/mtc_plan.json"
timeout --signal=TERM --kill-after=3s 35s ros2 launch robot129_tasks mtc_plan.launch.py > "'$root'/out/lesson_09/mtc.log" 2>&1
python - <<"PY"
import json
from pathlib import Path
p=Path("'$root'/out/lesson_09/mtc_plan.json")
if not p.exists(): raise SystemExit("MTC report was not produced")
r=json.loads(p.read_text())
print(json.dumps(r,indent=2))
raise SystemExit(0 if r.get("status")=="PASS" else 1)
PY
'
