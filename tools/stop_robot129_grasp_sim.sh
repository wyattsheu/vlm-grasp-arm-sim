#!/usr/bin/env bash
# Observed 2026-09-18: the Isaac/Kit child process sometimes ends up in a different
# process group than the launcher's `setsid` group (Kit apps commonly detach their own
# session for signal isolation), so `kill -TERM -- -"$pid"` on the recorded launcher pid
# can return success while the actual GPU process survives. This stop script therefore
# always also matches on the script's command line as a second, independent kill path,
# and only reports "stopped" once nothing matching is left, not just after signalling.
#
# 2026-09-24 real bug, found live: the pattern used to be just
# "sim/scripts/run_robot129_ros_webrtc.py --bundle $root", which is a substring of BOTH
# this script's own --record-only invocation AND start_robot129_ros_webrtc.sh's
# --livestream invocation (same underlying python script, different flags) -- so calling
# THIS stop script also killed a live, independently-started WebRTC session that had
# nothing to do with the headless one this script is meant to manage. Narrowed to
# require --record-only (only start_robot129_grasp_sim.sh's own invocation has that
# flag) so this can no longer cross-kill a --livestream session.
set -uo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/grasp_motion/live"
pattern="sim/scripts/run_robot129_ros_webrtc.py --bundle $root --device cuda:0 --record-only"

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
