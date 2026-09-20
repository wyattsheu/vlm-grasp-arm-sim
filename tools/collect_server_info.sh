#!/usr/bin/env bash
set -u

bundle_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
output_path="${1:-$bundle_root/environment/server_environment.txt}"

{
  echo "Robot 129 PRO 6000 server environment inventory"
  echo "captured_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo
  echo "[architecture]"
  uname -m
  uname -sr
  echo
  echo "[processor]"
  if command -v lscpu >/dev/null 2>&1; then
    lscpu | sed -n -E '/^(Architecture|CPU\(s\)|On-line CPU\(s\) list|Model name|Socket\(s\)|Core\(s\) per socket|Thread\(s\) per core):/p'
  else
    echo "lscpu UNAVAILABLE"
  fi
  echo
  echo "[os-release]"
  if [[ -r /etc/os-release ]]; then sed -n '1,40p' /etc/os-release; else echo "UNAVAILABLE"; fi
  echo
  echo "[glibc]"
  if command -v ldd >/dev/null 2>&1; then ldd --version 2>&1 | sed -n '1p'; else echo "ldd UNAVAILABLE"; fi
  echo
  echo "[gpu]"
  if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
  else
    echo "nvidia-smi UNAVAILABLE"
  fi
  echo
  echo "[memory]"
  free -h 2>&1 || true
  echo
  echo "[workspace-disk]"
  df -h "$bundle_root" 2>&1 || true
  echo
  echo "[python]"
  command -v python3 2>/dev/null || true
  python3 --version 2>&1 || true
  echo
  echo "[uv]"
  if command -v uv >/dev/null 2>&1; then
    command -v uv
    uv --version 2>&1
  else
    echo "uv UNAVAILABLE"
  fi
  echo
  echo "[docker]"
  docker --version 2>&1 || echo "docker UNAVAILABLE"
  if command -v docker >/dev/null 2>&1; then
    if docker info >/dev/null 2>&1; then
      docker version --format 'daemon_version={{.Server.Version}}' 2>&1
    else
      echo "docker daemon UNAVAILABLE to current user"
    fi
  fi
  echo
  echo "[apptainer-singularity]"
  apptainer --version 2>&1 || echo "apptainer UNAVAILABLE"
  singularity --version 2>&1 || echo "singularity UNAVAILABLE"
  echo
  echo "[nvidia-container-toolkit]"
  nvidia-ctk --version 2>&1 || echo "nvidia-ctk UNAVAILABLE"
  nvidia-container-cli --version 2>&1 || echo "nvidia-container-cli UNAVAILABLE"
  echo
  echo "[ros2]"
  if command -v ros2 >/dev/null 2>&1; then
    command -v ros2
    printenv ROS_DISTRO 2>/dev/null || echo "ROS_DISTRO not set"
    printenv RMW_IMPLEMENTATION 2>/dev/null || echo "RMW_IMPLEMENTATION not set"
  else
    echo "ros2 UNAVAILABLE in current shell"
  fi
  echo
  echo "[isaac-sim]"
  command -v isaac-sim.sh 2>/dev/null || echo "isaac-sim.sh UNAVAILABLE in PATH"
} > "$output_path"

echo "Wrote $output_path"
echo "This inventory intentionally does not dump environment variables or credentials."
