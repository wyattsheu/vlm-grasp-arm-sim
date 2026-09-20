#!/usr/bin/env bash
set -euo pipefail
root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$root"
for path in README.md AGENTS.md LEARNING_PATH.md provenance/SHA256SUMS robot/vendor/piper_description/urdf/piper_description.urdf; do
  [[ -f "$path" ]] || { echo "MISSING: $path" >&2; exit 1; }
done
if find robot/vendor robot/reference_moveit_config provenance -type l -print -quit | grep -q .; then
  echo "FAIL: immutable source areas contain symlinks" >&2
  exit 1
fi
sha256sum --check provenance/SHA256SUMS
echo "PASS: handoff source checksums match; build/install symlinks were excluded intentionally"
