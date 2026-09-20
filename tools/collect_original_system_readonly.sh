#!/usr/bin/env bash
# Run this on Robot 129 / Thor. It never starts/stops containers, ROS nodes,
# CAN, RealSense, or models, and it intentionally does not collect environment secrets.
set -euo pipefail
out="${1:-robot129_source_inventory_$(date -u +%Y%m%dT%H%M%SZ).txt}"
ros_probe="${2:-no-ros}"
umask 077
{
  echo '# Robot 129 source inventory (read-only)'
  date -u +captured_at=%Y-%m-%dT%H:%M:%SZ
  hostname
  uname -a
  uname -m
  cat /etc/os-release 2>/dev/null || true
  cat /etc/nv_tegra_release 2>/dev/null || true
  df -h /
  free -h
  echo '# Tool versions'
  python3 --version 2>&1 || true
  nvcc --version 2>&1 | tail -n 5 || true
  docker --version 2>&1 || true
  ros2 --version 2>&1 || true
  echo '# ROS variable names and non-secret values'
  env | grep -E '^(ROS_DISTRO|ROS_DOMAIN_ID|RMW_IMPLEMENTATION)=' | sort || true
  echo '# Containers: names, images, state only'
  docker ps -a --format '{{.Names}}|{{.Image}}|{{.Status}}' 2>&1 || true
  for c in mm_container local_pipeline_stage1_qwen local_pipeline_stage2_molmo2; do
    echo "## container=$c"
    docker inspect --format 'image={{.Config.Image}} image_id={{.Image}} arch={{.Platform}}' "$c" 2>&1 || true
    docker inspect --format '{{range .Mounts}}mount={{.Source}} -> {{.Destination}} rw={{.RW}}{{println}}{{end}}' "$c" 2>&1 || true
  done
  echo '# VLM model endpoints; no inference request'
  timeout 5s curl -fsS http://127.0.0.1:8000/v1/models 2>&1 || true
  timeout 5s curl -fsS http://127.0.0.1:8002/v1/models 2>&1 || true
  echo '# Known source revisions'
  for repo in /home/wyattsheu/workspaces/robotic_agent/mm_system /home/acm/robotic_agent/robotic_system /home/acm/robotic/robotic_system; do
    echo "repo=$repo"
    git -C "$repo" rev-parse HEAD 2>&1 || true
    git -C "$repo" status --short --untracked-files=no 2>&1 || true
  done
  if [[ "$ros_probe" == '--ros-readonly' ]]; then
    echo '# ROS graph probe explicitly requested; this joins the current ROS domain read-only'
    timeout 8s ros2 node list 2>&1 || true
    timeout 8s ros2 topic list -t 2>&1 || true
    for topic in /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info /joint_states_feedback; do
      echo "topic=$topic"
      timeout 8s ros2 topic info -v "$topic" 2>&1 || true
    done
  else
    echo '# ROS graph skipped. Re-run with second argument --ros-readonly only after approving production-domain discovery.'
  fi
} > "$out"
echo "WROTE $out"
echo 'Review and redact host paths before transferring; secret values were not requested.'
