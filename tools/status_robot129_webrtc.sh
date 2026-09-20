#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/webrtc_robot129"
if [[ -f "$state/server.pid" ]] && kill -0 "$(cat "$state/server.pid")" 2>/dev/null; then
  echo "RUNNING: PID=$(cat "$state/server.pid"), GPU=$(cat "$state/gpu.txt" 2>/dev/null || echo unknown)"
  ss -lntup 2>/dev/null | grep -E ':(49100|47998)\b' || true
  if [[ -f "$state/viewport_probe.json" ]]; then
    python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print("VIEWPORT: {} {} target={:.6f} camera={}".format(d["status"], d.get("resolution"), d.get("red_target_fraction", 0), d.get("active_camera")))' "$state/viewport_probe.json"
  fi
  grep -E '\[ROBOT129 WEBRTC\] (WARMING_UP|VERIFYING_VIEWPORT|VIEWPORT_PROBE|READY|VERIFYING_SUSTAINED_VIEWPORT|COMPLETE)' "$state/server.log" | tail -n 8 || true
else
  echo "STOPPED: Robot 129 WebRTC 未執行"
fi
