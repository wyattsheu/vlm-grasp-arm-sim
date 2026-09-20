#!/usr/bin/env bash
# Observed 2026-09-18: the Isaac/Kit child process sometimes ends up in a different
# process group than the launcher's `setsid` group (Kit apps commonly detach their own
# session for signal isolation), so `kill -TERM -- -"$pid"` on the recorded launcher pid
# can return success while the actual GPU process survives. This stop script therefore
# always also matches on the script's command line as a second, independent kill path,
# and only reports "stopped" once nothing matching is left, not just after signalling.
set -uo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/grasp_motion/live"
pattern="sim/scripts/run_robot129_ros_webrtc.py --bundle $root"

if [[ -f "$state/server.pid" ]]; then
  pid="$(cat "$state/server.pid")"
  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM -- -"$pid" 2>/dev/null
    kill -TERM "$pid" 2>/dev/null
  fi
fi
pkill -TERM -f "$pattern" 2>/dev/null

for _ in $(seq 1 15); do
  pgrep -f "$pattern" >/dev/null 2>&1 || break
  sleep 1
done

if pgrep -f "$pattern" >/dev/null 2>&1; then
  pkill -KILL -f "$pattern" 2>/dev/null
  sleep 1
fi

rm -f "$state/server.pid"
if pgrep -f "$pattern" >/dev/null 2>&1; then
  echo "FAILED to stop; still running:" >&2
  pgrep -af "$pattern" >&2
  exit 1
fi
echo "stopped"
