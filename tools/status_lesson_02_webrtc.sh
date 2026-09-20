#!/usr/bin/env bash
set -euo pipefail
bundle_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state_dir="$bundle_root/out/lesson_02/webrtc"
pid_file="$state_dir/server.pid"

if [[ -f "$pid_file" ]] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
  pid="$(cat "$pid_file")"
  echo "RUNNING: PID=$pid, GPU=$(cat "$state_dir/gpu.txt" 2>/dev/null || echo unknown)"
  ss -lntup 2>/dev/null | grep -E ':(49100|47998)\b' || true
  tail -n 12 "$state_dir/server.log"
else
  echo "STOPPED: Lesson 2 WebRTC Demo 未執行。"
fi

