#!/usr/bin/env bash
# Launches tools/rerun_dashboard.py against whichever Robot 129 Isaac session is already
# running (headless via start_robot129_grasp_sim.sh, or live via start_robot129_ros_webrtc.sh
# -- both publish the same ROS topics this dashboard subscribes to, so either works, and
# it can run alongside a live WebRTC session with no conflict; see
# docs/dev_guide_paper_core_and_dashboard_plan.md §8.7).
#
# Usage: bash tools/start_demo_dashboard.sh [extra tools/rerun_dashboard.py args...]
#   e.g. bash tools/start_demo_dashboard.sh --duration-s 90
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
state="$root/out/demo/dashboard_live"
mkdir -p "$state"
pattern="tools/rerun_dashboard.py"

if [[ -f "$state/server.pid" ]] && kill -0 "$(cat "$state/server.pid")" 2>/dev/null; then
  echo "Rerun dashboard 已在執行，PID=$(cat "$state/server.pid")"
  exit 0
fi
if pgrep -f "$pattern" >/dev/null 2>&1; then
  echo "Rerun dashboard 已在執行（沒有 server.pid，但 process 存在）：$(pgrep -af "$pattern" | head -1)"
  exit 0
fi

# Only inject our own default --save-rrd when the caller didn't already pass one --
# passing both unconditionally (a real bug, found live 2026-09-24 via
# tools/run_demo_scenario.sh, which always passes its own --save-rrd) doesn't crash
# argparse (last value wins) but is confusing and makes the process's own command line
# lie about where it's actually saving.
save_rrd_args=(--save-rrd "$state/dashboard.rrd")
web_port=9090
grpc_port=9876
prev=""
for arg in "$@"; do
  if [[ "$arg" == "--save-rrd" || "$arg" == "--no-save" ]]; then
    save_rrd_args=()
  fi
  if [[ "$prev" == "--web-port" ]]; then
    web_port="$arg"
  fi
  if [[ "$prev" == "--grpc-port" ]]; then
    grpc_port="$arg"
  fi
  prev="$arg"
done

: > "$state/server.log"
nohup setsid bash -lc "
  source /mnt/HDD4/wyattsheu/env_robot129_ros/setup.bash
  source '$root/ros2_ws/install/setup.bash'
  export ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  cd '$root'
  exec /mnt/HDD4/wyattsheu/env_robot129_realstack/bin/python3 tools/rerun_dashboard.py \"\$@\"
" -- "${save_rrd_args[@]}" "$@" > "$state/server.log" 2>&1 &

# Do NOT trust $! here. Same class of bug tools/stop_robot129_grasp_sim.sh already
# documents for Isaac/Kit ("the child process sometimes ends up in a different process
# group than the launcher's setsid group"): when setsid itself has to fork (because the
# calling process is already a process group leader, which it usually is inside a
# backgrounded `nohup setsid ... &` job), the PID the shell captures via $! is the
# short-lived setsid wrapper, NOT the grandchild that actually execs into python3 and
# keeps running. Confirmed live 2026-09-24, and non-deterministically in BOTH directions:
# one run had `kill -0 "$!"` report dead within 3s while the dashboard was correctly up
# and serving; another had it report alive for the full timeout while the log never
# showed readiness. `pgrep -f` against the real command line is robust to which PID ends
# up owning the process, matching the pattern the stop_*.sh scripts already use.
#
# Readiness is checked by polling the web port with `ss`, NOT by grepping server.log for
# rerun_dashboard.py's own "serving web viewer" print -- confirmed live 2026-09-24 that
# the print is NOT a reliable signal here: in repeated runs the port was already open
# (confirmed with `ss`) while server.log stayed completely empty for the entire polling
# window, even though the process was genuinely up and serving. Root cause not fully
# pinned down (suspected: rerun's serve_web() spawns its own server thread/native code
# that interacts with the process's stdout in a way that breaks the buffered write this
# script's print(..., flush=True) depends on) -- the port binding itself is a strictly
# more reliable "is it actually reachable" signal anyway, so this sidesteps the mystery
# rather than chasing it further.
# The native web-viewer startup has occasionally hung after binding gRPC but
# before binding the web port. Fail this optional visualization step promptly;
# run_demo_scenario.sh retries once after Isaac settles.
# Shared training jobs can delay Rerun's native web server even after the Python
# process starts. Allow up to 60 seconds; the dashboard remains optional to motion
# execution, and run_demo_scenario still retries after Isaac is ready.
for _ in $(seq 1 120); do
  if ss -lnt 2>/dev/null | grep -q ":${web_port} "; then
    real_pid="$(pgrep -f "$pattern" | head -1)"
    echo "${real_pid:-unknown}" > "$state/server.pid"
    echo "READY -- web viewer listening on port $web_port (pid=${real_pid:-unknown})"
    # The bare http://localhost:$web_port/ shows the Rerun web viewer's own "no
    # recording connected" welcome screen, NOT this dashboard -- it needs a
    # `?url=<gRPC connect URI>` query param to auto-connect (confirmed live 2026-09-24:
    # a person opened the bare URL this script used to print and saw exactly that
    # generic landing page). tools/rerun_dashboard.py prints the full, correct URL
    # itself -- wait a little (same log-write-delay caveat as above) and relay it
    # rather than reconstructing it here a second way.
    real_url=""
    for _ in $(seq 1 20); do
      real_url="$(grep -o 'http://[^ ]*url=[^ ]*' "$state/server.log" 2>/dev/null | tail -1)"
      [[ -n "$real_url" ]] && break
      sleep 0.5
    done
    if [[ -n "$real_url" ]]; then
      echo "本機直接看：瀏覽器開 $real_url  （不要只開 http://localhost:$web_port，那會看到 Rerun 的空白歡迎畫面，不是這個 dashboard）"
      echo "遠端看：先 ssh -L $web_port:localhost:$web_port -L $grpc_port:localhost:$grpc_port 到這台機器，再瀏覽器開同一個網址"
    else
      echo "本機直接看：瀏覽器開 http://localhost:$web_port/?url=rerun%2Bhttp%3A%2F%2F127.0.0.1%3A${grpc_port}%2Fproxy"
      echo "遠端看：先 ssh -L $web_port:localhost:$web_port -L $grpc_port:localhost:$grpc_port 到這台機器，再瀏覽器開同一個網址"
    fi
    if [[ -s "$state/server.log" ]]; then
      grep 'also saving to' "$state/server.log" 2>/dev/null || true
    fi
    exit 0
  fi
  if ! pgrep -f "$pattern" >/dev/null 2>&1; then
    echo "FAILED: dashboard process exited before becoming ready" >&2
    tail -n 60 "$state/server.log" >&2
    rm -f "$state/server.pid"
    exit 1
  fi
  sleep 0.5
done
echo "TIMEOUT waiting for dashboard's web port to open" >&2
tail -n 60 "$state/server.log" >&2
pkill -f "$pattern" 2>/dev/null || true
rm -f "$state/server.pid"
exit 1
