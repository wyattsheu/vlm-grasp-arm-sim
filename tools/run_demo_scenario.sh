#!/usr/bin/env bash
# One-command orchestration for the five obstacle-avoidance demo scenarios
# (docs/dev_guide_paper_core_and_dashboard_plan.md §8): starts Isaac with the right
# scene/params, starts the dashboard, starts recording, runs the scenario's driver
# (reactive.py / run_curobo_grasp_demo.sh / send_joint_cmd.py), stops recording, collects
# video/rrd/report artifacts into out/demo/<scenario>/<timestamp>/, tears everything down.
#
# Usage:
#   tools/run_demo_scenario.sh <static_multi|dynamic_bar|pose5|p2p_cone|stick_ab|stick|static_cylinder|grasp|joints> [options]
#
# Options:
#   --webrtc            Launch Isaac with the interactive WebRTC viewport (port 49100)
#                        instead of headless --record-only. Either way the dashboard
#                        works the same -- see the dev guide §8.8 for viewing both at once.
#   --target ID          grasp scenario only: research/configs/grasp_targets.json id
#                        (default cube_35). Also accepted (ignored) for other scenarios.
#   --duration-s N        stick/static_cylinder only: how long reactive.py's goal-a/
#                        goal-b loop runs, in wall-clock seconds (default 60).
#   --waypoints PATH      pose5 only: editable YAML of five 6D pinch-center goals.
#   --out-dir DIR         override the default out/demo/<scenario>/<timestamp>/ location.
#   --keep-running        don't tear Isaac/dashboard down at the end (useful if you want
#                        to keep looking at the WebRTC/dashboard view after the scripted
#                        part finishes).
#
# Switching scenarios needs a fresh Isaac process (~1-2 min boot) -- this script always
# stops whatever was running first, it does not try to reuse an existing session.
#
# What this script does NOT do: automatically grade the pass/fail criteria in the dev
# guide's scenario table (e.g. "contact force stayed 0 N the whole time", "voxels tracked
# the obstacle within 3cm") -- those need a human looking at the recorded video/dashboard,
# or a separate analysis pass over the collected artifacts. This script's job is
# orchestration and artifact collection, not scoring.
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
micro=/mnt/HDD4/wyattsheu/tools/micromamba/micromamba
ros_env=/mnt/HDD4/wyattsheu/env_robot129_ros
ros_python=/mnt/HDD4/wyattsheu/env_robot129_ros/bin/python3
curobo_python=/mnt/HDD4/wyattsheu/env_robot129_curobo/bin/python

scenario="${1:-}"
if [[ -z "$scenario" ]]; then
  sed -n '2,25p' "$0" >&2
  exit 2
fi
shift

use_webrtc=0
target="cube_35"
duration_s=60
waypoints_path="$root/research/configs/demo/five_pose_waypoints.yaml"
out_dir=""
keep_running=0
dashboard_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --webrtc) use_webrtc=1; shift ;;
    --target) target="$2"; shift 2 ;;
    --duration-s) duration_s="$2"; shift 2 ;;
    --waypoints) waypoints_path="$2"; shift 2 ;;
    --out-dir) out_dir="$2"; shift 2 ;;
    --keep-running) keep_running=1; shift ;;
    -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

case "$scenario" in
  static_multi)
    sim_args=(--scene dynamic_stick --stick none --scene-manifest "$root/research/configs/scenes/official_static_cone.json" --scene-camera pole --demo-targets static_multi)
    dashboard_args+=(--waypoints "$root/research/configs/demo/official_static_multi.yaml" --obstacle cone_multi)
    ;;
  dynamic_bar)
    sim_args=(--scene dynamic_stick --stick insert_bar --scene-camera pole --demo-targets dynamic_bar)
    dashboard_args+=(--waypoints "$root/research/configs/demo/official_dynamic_bar_targets.yaml" --obstacle bar)
    ;;
  pose5)
    sim_args=(--scene dynamic_stick --stick none --scene-camera pole)
    dashboard_args+=(--waypoints "$waypoints_path" --obstacle none)
    ;;
  stick_ab)
    sim_args=(--scene dynamic_stick --scene-camera pole)
    dashboard_args+=(--waypoints "$root/research/configs/demo/ab_poses.yaml" --obstacle stick)
    ;;
  stick)
    sim_args=(--scene dynamic_stick --scene-camera pole)
    dashboard_args+=(--obstacle stick)
    ;;
  static_cylinder)
    sim_args=(--scene dynamic_stick --stick none --scene-manifest "$root/research/configs/scenes/obstacle_static_cylinder.json" --scene-camera pole)
    ;;
  p2p_cone)
    sim_args=(--scene dynamic_stick --stick none --scene-manifest "$root/research/configs/scenes/obstacle_cone_ab.json" --scene-camera pole)
    dashboard_args+=(--waypoints "$root/research/configs/demo/ab_poses.yaml" --obstacle cone)
    ;;
  grasp)
    sim_args=(--scene pick_place --target-object "$target")
    ;;
  joints)
    sim_args=(--scene pick_place)
    ;;
  *)
    echo "unknown scenario: $scenario (expected static_multi|dynamic_bar|pose5|p2p_cone|stick_ab|stick|static_cylinder|grasp|joints)" >&2
    exit 2
    ;;
esac
if [[ "$scenario" == "pose5" ]]; then
  waypoints_path="$(realpath "$waypoints_path")"
  dashboard_args=(--waypoints "$waypoints_path" --obstacle none)
  PYTHONPATH="$root/research/src" "$ros_python" - "$waypoints_path" <<'PY'
import sys
from mpg.curobo_bridge.waypoints import load_waypoints_from_yaml
points = load_waypoints_from_yaml(sys.argv[1])
if len(points) != 5:
    raise SystemExit(f"pose5 requires exactly five 6D goals; got {len(points)}")
PY
fi

run_out_dir="${out_dir:-$root/out/demo/$scenario/$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "$run_out_dir"
echo "[run_demo_scenario] scenario=$scenario out_dir=$run_out_dir webrtc=$use_webrtc"
echo "[run_demo_scenario] isaac args: ${sim_args[*]}"

least_busy_gpu() {
  nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader,nounits \
    | awk -F, '{gsub(/ /,""); print ($2*100+$3/100) "," $1}' | sort -n | head -1 | cut -d, -f2
}

ros_call() {
  # $1 = service path, $2 = srv type, $3 = request yaml
  "$micro" run -p "$ros_env" bash -lc "
    source '$ros_env/setup.bash'
    source '$root/ros2_ws/install/setup.bash'
    export ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    ros2 service call '$1' '$2' '$3'
  "
}

echo "[1/6] clean slate: stopping any previous session"
bash "$root/tools/stop_demo_dashboard.sh" >/dev/null 2>&1 || true
bash "$root/tools/stop_any_webrtc.sh" >/dev/null 2>&1 || true
bash "$root/tools/stop_robot129_grasp_sim.sh" >/dev/null 2>&1 || true

start_dashboard_for_run() {
  bash "$root/tools/start_demo_dashboard.sh" --save-rrd "$run_out_dir/dashboard.rrd" "${dashboard_args[@]}"
}

# Rerun's native web server has intermittently stalled when first launched
# immediately after Isaac starts. For these new tests it can subscribe before
# Isaac exists, and will discover the ROS topics when they appear.
dashboard_prestarted=0
if [[ "$scenario" == "static_multi" || "$scenario" == "dynamic_bar" ]]; then
  echo "[1/6] starting Rerun before Isaac to avoid native viewer startup contention"
  if start_dashboard_for_run; then dashboard_prestarted=1; fi
fi

echo "[2/6] starting Isaac"
if [[ "$use_webrtc" == "1" ]]; then
  bash "$root/tools/start_robot129_ros_webrtc.sh" "${sim_args[@]}"
else
  bash "$root/tools/start_robot129_grasp_sim.sh" "${sim_args[@]}"
fi

echo "[3/6] starting dashboard"
# Non-fatal: the dashboard is a visualization convenience, not part of what makes a
# scenario PASS/FAIL (that's the driver's own exit status + the recorded artifacts) --
# if it fails to start, warn and keep going with recording + the driver anyway.
if [[ "$dashboard_prestarted" == "0" ]] && ! start_dashboard_for_run; then
  echo "[run_demo_scenario] retrying dashboard after Isaac startup settles"
  sleep 2
  start_dashboard_for_run \
    || echo "WARNING: dashboard failed to start -- continuing without it (see out/demo/dashboard_live/server.log)"
fi

echo "[4/6] start recording"
ros_call /robot129_sim/recording std_srvs/srv/SetBool "{data: true}"

# reactive.py needs BOTH env_robot129_curobo (for curobo/torch) and rclpy (only in
# env_robot129_ros) at once -- neither venv alone has both. Same fix as
# docs/dev_guide_paper_core_and_dashboard_plan.md §7.8/§8.3's documented invocation:
# layer env_robot129_ros's site-packages onto PYTHONPATH rather than `source`-ing its
# setup.bash (that would fight env_robot129_curobo's own already-active venv). Found
# live 2026-09-24 running this exact script without this fix: `ModuleNotFoundError: No
# module named 'rclpy'` -- easy to miss because a bare `python reactive.py --help`-style
# smoke test never reaches the `import rclpy` line inside reactive_node().
ros_site_packages="/mnt/HDD4/wyattsheu/env_robot129_ros/lib/python3.12/site-packages"

driver_status=0
case "$scenario" in
  static_multi|dynamic_bar)
    echo "[5/6] running cuRobo MotionPlanner official-style $scenario test"
    gpu="$(least_busy_gpu)"
    speed_args=(--speed-multiplier 4)
    if [[ "$scenario" == "static_multi" ]]; then
      speed_args=(--speed-multiplier 8)
    fi
    CUDA_VISIBLE_DEVICES="$gpu" PYTHONPATH="$root/research/src:$ros_site_packages" \
      ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      "$curobo_python" "$root/research/src/mpg/curobo_bridge/official_style_demos.py" \
      "$scenario" --out-dir "$run_out_dir" "${speed_args[@]}" \
      > "$run_out_dir/demo.log" 2>&1 || driver_status=$?
    if [[ "$driver_status" == "0" ]]; then
      "$ros_python" - "$run_out_dir/report.json" <<'PY' || driver_status=$?
import json, sys
from pathlib import Path
report = json.loads(Path(sys.argv[1]).read_text())
if report.get("status") != "PASS":
    raise SystemExit("official-style demo report is not PASS")
PY
    fi
    ;;
  pose5|p2p_cone|stick_ab)
    if [[ "$scenario" == "pose5" ]]; then
      path="$waypoints_path"
    else
      path="$root/research/configs/demo/ab_poses.yaml"
    fi
    echo "[5/6] running cuRobo MPC through $(basename "$path")"
    gpu="$(least_busy_gpu)"
    CUDA_VISIBLE_DEVICES="$gpu" PYTHONPATH="$root/research/src:$ros_site_packages" \
      ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      "$curobo_python" "$root/research/src/mpg/curobo_bridge/reactive.py" \
      --include-pole --waypoints "$path" --report-out "$run_out_dir/reactive_report.json" \
      > "$run_out_dir/reactive.log" 2>&1 || driver_status=$?
    if [[ "$driver_status" == "0" ]]; then
      "$ros_python" - "$run_out_dir/reactive_report.json" <<'PY' || driver_status=$?
import json, sys
from pathlib import Path
path = Path(sys.argv[1])
if not path.exists() or json.loads(path.read_text()).get("status") != "PASS":
    raise SystemExit("cuRobo waypoint report missing or FAIL")
PY
    fi
    ;;
  stick|static_cylinder)
    echo "[5/6] running reactive.py (goal-a/goal-b loop) for ${duration_s}s wall-clock"
    gpu="$(least_busy_gpu)"
    CUDA_VISIBLE_DEVICES="$gpu" PYTHONPATH="$root/research/src:$ros_site_packages" \
      ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      "$curobo_python" "$root/research/src/mpg/curobo_bridge/reactive.py" \
      --include-pole --duration-s "$duration_s" \
      > "$run_out_dir/reactive.log" 2>&1 || driver_status=$?
    ;;
  grasp)
    echo "[5/6] running run_curobo_grasp_demo.sh --target $target"
    bash "$root/tools/run_curobo_grasp_demo.sh" --target "$target" --include-pole \
      --out-dir "$run_out_dir/execution" > "$run_out_dir/grasp.log" 2>&1 || driver_status=$?
    ;;
  joints)
    echo "[5/6] running send_joint_cmd.py --sequence joint_sequence.yaml"
    "$micro" run -p "$ros_env" bash -lc "
      source '$ros_env/setup.bash'
      source '$root/ros2_ws/install/setup.bash'
      export ROS_DOMAIN_ID=129 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      '$ros_python' '$root/tools/send_joint_cmd.py' \
        --sequence '$root/research/configs/demo/joint_sequence.yaml' \
        --out '$run_out_dir/joint_response.json'
    " > "$run_out_dir/joints.log" 2>&1 || driver_status=$?
    ;;
esac
case "$scenario" in
  grasp) driver_log=grasp.log ;;
  joints) driver_log=joints.log ;;
  static_multi|dynamic_bar) driver_log=demo.log ;;
  *) driver_log=reactive.log ;;
esac
echo "[5/6] driver exit status: $driver_status (log: $run_out_dir/$driver_log)"

echo "[6/6] stop recording, collect outputs, tear down"
ros_call /robot129_sim/recording std_srvs/srv/SetBool "{data: false}" || true
echo "waiting for ffmpeg to finish encoding..."
sleep 8

session_dir="$(ls -d "$root"/out/grasp_motion/sessions/run_* 2>/dev/null | sort | tail -1)"
if [[ -n "$session_dir" ]]; then
  for f in video.mp4 wrist_video.mp4 scene_video.mp4 manifest.json; do
    if [[ -f "$session_dir/$f" ]]; then
      cp "$session_dir/$f" "$run_out_dir/"
    fi
  done
  echo "collected recording from $session_dir"
else
  echo "WARNING: no recording session directory found under out/grasp_motion/sessions/"
fi

if [[ "$keep_running" == "1" ]]; then
  # Finalize the per-run recording before entering the open-ended inspection
  # session.  Leaving the file-backed dashboard alive made dashboard.rrd grow
  # forever (one overnight static_multi session reached 9.5 GB).  Restart the
  # same dashboard in live-only mode so the WebRTC/Rerun inspection URLs stay
  # available without modifying the finished artifact.
  echo "[run_demo_scenario] --keep-running: finalizing dashboard.rrd, then keeping Isaac + live-only dashboard up"
  bash "$root/tools/stop_demo_dashboard.sh" || true
  bash "$root/tools/start_demo_dashboard.sh" --no-save "${dashboard_args[@]}" || \
    echo "WARNING: live-only dashboard restart failed; Isaac/WebRTC is still running" >&2
else
  bash "$root/tools/stop_demo_dashboard.sh" || true
  if [[ "$use_webrtc" == "1" ]]; then
    bash "$root/tools/stop_robot129_ros_webrtc.sh" || true
  else
    bash "$root/tools/stop_robot129_grasp_sim.sh" || true
  fi
fi

status_word="PASS"
[[ "$driver_status" != "0" ]] && status_word="FAIL"
echo "DONE scenario=$scenario driver_status=$status_word out_dir=$run_out_dir"
exit "$driver_status"
