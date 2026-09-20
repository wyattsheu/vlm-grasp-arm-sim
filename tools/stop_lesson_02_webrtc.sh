#!/usr/bin/env bash
set -euo pipefail
bundle_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state_dir="$bundle_root/out/lesson_02/webrtc"
pid_file="$state_dir/server.pid"

if [[ ! -f "$pid_file" ]] || ! kill -0 "$(cat "$pid_file")" 2>/dev/null; then
  echo "Lesson 2 WebRTC Demo 已停止。"
  rm -f "$pid_file"
  exit 0
fi

pid="$(cat "$pid_file")"
kill -TERM "$pid"
for _ in $(seq 1 20); do
  if ! kill -0 "$pid" 2>/dev/null; then
    rm -f "$pid_file"
    echo "PASS: WebRTC Demo 已正常停止。"
    exit 0
  fi
  sleep 1
done

echo "STOP: PID $pid 未在 20 秒內結束，未強制 kill。" >&2
exit 4
