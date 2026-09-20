#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
exec /mnt/HDD4/wyattsheu/env_robot129_research/bin/python \
  "$root/tools/validate_camera_calibration.py" "$@"
