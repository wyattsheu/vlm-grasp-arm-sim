#!/usr/bin/env bash
# Usage: start_robot129_ros_webrtc.sh [--scene marker|pick_place|pick_place_hammer|pick_place_counter]
#        [--scene-manifest <json>] [--wrist-camera-model auto|urdf_nominal|real_calib]
# Defaults to marker (sim/scripts/run_robot129_ros_webrtc.py's own default) if omitted.
#
# Bug fixed 2026-09-20: this script used to take no arguments at all and silently
# ignore anything passed to it (including --scene), always launching the python
# script's "marker" default -- a scene with a purely visual red marker and no
# graspable physics object. Nobody hit this until today, because every prior real
# grasp-motion PASS was run against start_robot129_grasp_sim.sh (headless), which
# already forwarded "$@" correctly; two live-WebRTC executions via this script both
# failed physical_grasp_success even though MTC planning and the joint trajectory
# both reported success, because /robot129_sim/objects/target_cube/pose never
# existed to report a rest pose from -- the marker scene has no such object.
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
isaac=/mnt/HDD4/wyattsheu/IsaacLab
rosroot="$isaac/.venv/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy"
state="$root/out/ros_webrtc_robot129"
mkdir -p "$state"

scene="marker"
scene_manifest=""
wrist_camera_model="auto"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --scene) scene="$2"; shift 2 ;;
    --scene-manifest) scene_manifest="$2"; shift 2 ;;
    --wrist-camera-model) wrist_camera_model="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [[ -f "$state/server.pid" ]] && kill -0 "$(cat "$state/server.pid")" 2>/dev/null; then
  echo "Robot 129 ROS WebRTC 已在執行，PID=$(cat "$state/server.pid")"
  "$root/tools/status_robot129_ros_webrtc.sh"
  exit 0
fi
if [[ -f "$root/out/webrtc_robot129/server.pid" ]] && kill -0 "$(cat "$root/out/webrtc_robot129/server.pid")" 2>/dev/null; then
  echo "停止自動抓取 WebRTC Demo，改由 ROS 控制。"
  "$root/tools/stop_robot129_webrtc.sh"
fi
if ss -lnt 2>/dev/null | grep -q ':49100 '; then
  echo "STOP: WebRTC signaling port 49100 已被其他程序占用。" >&2
  ss -lntp 2>/dev/null | grep ':49100 ' >&2 || true
  exit 4
fi

gpu_row="$(nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader,nounits | awk -F, '{gsub(/ /,""); print ($2*100+$3/100) "," $1 "," $2 "," $3}' | sort -n | head -1)"
IFS=',' read -r _ gpu util memory <<< "$gpu_row"
public_ip="${ROBOT129_WEBRTC_PUBLIC_IP:-140.113.203.85}"
export CUDA_VISIBLE_DEVICES="$gpu" OMNI_KIT_ACCEPT_EULA=Y OMP_NUM_THREADS=4
export ISAAC_LAB_ENABLE_ISAAC_RTX_PER_ENV_SCENE_PARTITION=0
export ROS_DISTRO=jazzy ROS_DOMAIN_ID=129 ROS_NAMESPACE=/robot129_sim RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$rosroot/rclpy${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$rosroot/lib:$isaac/.venv/lib/python3.12/site-packages/nvidia/cuda_runtime/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export LD_PRELOAD="$isaac/.venv/lib/python3.12/site-packages/nvidia/cuda_runtime/lib/libcudart.so.12${LD_PRELOAD:+:$LD_PRELOAD}"

: > "$state/server.log"
cd "$isaac"
nohup setsid uv run --no-sync python "$root/sim/scripts/run_robot129_ros_webrtc.py" \
  --bundle "$root" --device cuda:0 --livestream 2 --warmup-frames 90 --scene "$scene" \
  ${scene_manifest:+--scene-manifest "$scene_manifest"} --wrist-camera-model "$wrist_camera_model" \
  --kit_args "--/app/window/width=1280 --/app/window/height=720 --/exts/omni.kit.livestream.app/primaryStream/targetFps=30 --/exts/omni.kit.livestream.app/primaryStream/publicIp=$public_ip --/exts/omni.kit.livestream.app/primaryStream/allowDynamicResize=false --/rtx/hydra/readTransformsFromFabricInRenderDelegate=0 --/renderer/multiGpu/enabled=false" \
  > "$state/server.log" 2>&1 &
pid=$!
echo "$pid" > "$state/server.pid"
echo "$gpu" > "$state/gpu.txt"

cleanup_failed() {
  kill -TERM -- -"$pid" 2>/dev/null || true
  rm -f "$state/server.pid"
}
echo "初始化中：ROS-controlled Isaac Sim，PID=$pid，GPU=$gpu，utilization=${util}%，VRAM=${memory} MiB"
echo "隔離：ROS_DOMAIN_ID=129，namespace=/robot129_sim，hardware drivers=0"
notified=0
for _ in $(seq 1 90); do
  if ! kill -0 "$pid" 2>/dev/null; then
    tail -n 100 "$state/server.log"
    cleanup_failed
    exit 1
  fi
  if [[ "$notified" == 0 ]] && grep -q '\[ROBOT129 ROS WEBRTC\] WARMING_UP' "$state/server.log"; then
    echo "暖機中：RTX、WebRTC、ROS node"
    notified=1
  fi
  if grep -q '\[ROBOT129 ROS WEBRTC\] READY' "$state/server.log" && ss -lnt 2>/dev/null | grep -q ':49100 '; then
    cat > "$state/acceptance.json" <<JSON
{
  "status": "PASS",
  "scope": "ROS_CONTROLLED_WEBRTC_SIMULATION",
  "simulation_only": true,
  "gpu_index": $gpu,
  "public_ip": "$public_ip",
  "ros_domain_id": 129,
  "namespace": "/robot129_sim",
  "signal_port": 49100,
  "stream_port": 47998,
  "framebuffer": "1280x720@30fps",
  "command_topics": [
    "/robot129_sim/arm_controller/joint_trajectory",
    "/robot129_sim/gripper_controller/joint_trajectory"
  ],
  "feedback_topic": "/robot129_sim/joint_states",
  "wrist_camera": {
    "frame_id": "camera_color_optical_frame",
    "rgb_topic": "/robot129_sim/camera/color/image_raw",
    "depth_topic": "/robot129_sim/camera/aligned_depth_to_color/image_raw",
    "camera_info_topic": "/robot129_sim/camera/aligned_depth_to_color/camera_info",
    "rgb_encoding": "rgb8",
    "depth_encoding": "32FC1",
    "depth_units": "meter",
    "physical_extrinsic_verified": false
  },
  "hardware_drivers": 0
}
JSON
    echo "PASS: ROS-controlled Robot 129 已就緒；現在保持 HOME，等待 ROS 命令。"
    echo "WebRTC=$public_ip:49100 stream=47998"
    echo "發送示例：bash tools/send_robot129_ros_pose.sh inspect"
    exit 0
  fi
  sleep 1
done
echo "FAIL: 90 秒內未就緒" >&2
tail -n 120 "$state/server.log" >&2
cleanup_failed
exit 1
