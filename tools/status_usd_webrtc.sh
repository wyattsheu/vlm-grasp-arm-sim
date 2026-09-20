#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/webrtc_usd_viewer"
if [[ -f "$state/server.pid" ]] && kill -0 "$(cat "$state/server.pid")" 2>/dev/null; then
  echo "RUNNING: PID=$(cat "$state/server.pid"), GPU=$(cat "$state/gpu.txt" 2>/dev/null || echo unknown)"
  echo "USD: $(cat "$state/usd_path.txt" 2>/dev/null || echo unknown)"
  ss -lntup 2>/dev/null | grep -E ':(49100|47998)\b' || true
  echo "--- report.json ---"
  [[ -f "$state/report.json" ]] && cat "$state/report.json" || echo "(尚未產生)"
else
  echo "STOPPED: USD WebRTC viewer 未執行"
fi
