#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/camera_viewer"
mkdir -p "$state"
if [[ -f "$state/viewer.pid" ]] && kill -0 "$(cat "$state/viewer.pid")" 2>/dev/null; then
  echo "RUNNING: PID=$(cat "$state/viewer.pid") http://127.0.0.1:8090"
  exit 0
fi
if [[ ! -f "$root/out/ros_webrtc_robot129/wrist_camera/rgb.png" ]]; then
  echo "STOP: 先執行 bash tools/start_robot129_ros_webrtc.sh" >&2
  exit 2
fi
nohup python3 "$root/tools/robot129_camera_viewer.py" --bind 127.0.0.1 --port 8090 \
  > "$state/viewer.log" 2>&1 &
pid=$!
echo "$pid" > "$state/viewer.pid"
for _ in $(seq 1 20); do
  if curl -fsS http://127.0.0.1:8090/stats > "$state/latest_stats.json"; then
    echo "PASS: wrist RGB-D viewer 已在 http://127.0.0.1:8090"
    echo "從個人電腦建立 SSH tunnel: ssh -N -L 8090:127.0.0.1:8090 wyattsheu@140.113.203.85"
    exit 0
  fi
  sleep 0.25
done
cat "$state/viewer.log" >&2
exit 1
