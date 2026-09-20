#!/usr/bin/env bash
# Counterpart to tools/start_robot129_ros_webrtc.sh (and the other start_*_webrtc.sh /
# start_robot129_grasp_sim.sh scripts): frees WebRTC signaling port 49100 no matter
# which of this project's launchers holds it, AND no matter what unrelated tool holds it
# instead -- e.g. a parcel-forge `pf view` session, or anything else on this shared
# machine. The individual stop_*.sh scripts only know how to stop the one server they
# started (via their own out/.../server.pid); they silently no-op if a DIFFERENT program
# is squatting on the port, which is exactly the situation that motivated this script
# (2026-09-20: parcel-forge's viewer held 49100 while Robot 129 was running headless
# --record-only, so the WebRTC client kept showing parcel-forge's stale scene).
#
# Usage:
#   bash tools/stop_any_webrtc.sh
set -uo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"

echo "[1/2] stopping this project's own tracked WebRTC servers"
# Deliberately NOT stop_robot129_grasp_sim.sh: that launcher runs Isaac headless
# (--record-only), never opens port 49100, and is the sim the grasp/motion demo
# pipeline (tools/run_grasp_motion_demo.sh) needs kept alive. Stopping it here would
# silently kill an unrelated, still-needed simulation every time someone just wants
# their WebRTC *view* back -- learned the hard way 2026-09-20 when an earlier version
# of this script included it and took down the live demo sim as a side effect.
for stopper in stop_robot129_ros_webrtc.sh stop_robot129_webrtc.sh stop_usd_webrtc.sh stop_lesson_02_webrtc.sh; do
  if [[ -x "$root/tools/$stopper" ]]; then
    echo "  -> $stopper"
    "$root/tools/$stopper" || true
  fi
done

echo "[2/2] checking whether port 49100 is still held by something else"
found_any=0
for _ in $(seq 1 5); do
  line="$(ss -tlnp 2>/dev/null | grep ':49100 ' || true)"
  if [[ -z "$line" ]]; then
    break
  fi
  found_any=1
  pid="$(echo "$line" | grep -oP 'pid=\K[0-9]+' | head -1)"
  if [[ -z "$pid" ]]; then
    echo "STOP: port 49100 is occupied but the PID could not be parsed from:" >&2
    echo "  $line" >&2
    exit 4
  fi
  cmd="$(ps -o cmd= -p "$pid" 2>/dev/null || echo '<gone>')"
  echo "  port 49100 held by PID=$pid (owner process group), cmd: $cmd"
  echo "  -- this is not one of this project's tracked servers; killing its process group"
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
  if [[ -n "$pgid" ]]; then
    kill -TERM -- "-$pgid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  else
    kill -TERM "$pid" 2>/dev/null || true
  fi
  for _ in $(seq 1 10); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 1
  done
  if kill -0 "$pid" 2>/dev/null; then
    echo "  still alive after SIGTERM, sending SIGKILL"
    kill -KILL -- "-$pgid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    sleep 1
  fi
done

if ss -tlnp 2>/dev/null | grep -q ':49100 '; then
  echo "FAILED: port 49100 is still occupied:" >&2
  ss -tlnp 2>/dev/null | grep ':49100 ' >&2
  exit 1
fi

if [[ "$found_any" == 1 ]]; then
  echo "STOPPED: all WebRTC sessions on port 49100 (tracked and untracked)"
else
  echo "STOPPED: no tracked server was running and port 49100 was already free"
fi
