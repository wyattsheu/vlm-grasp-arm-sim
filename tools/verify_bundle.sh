#!/usr/bin/env bash
set -euo pipefail

bundle_root="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$bundle_root"

required=(
  README.md
  AGENTS.md
  START_PROMPT_FOR_CODEX_OR_CLAUDE.md
  LEARNING_PATH.md
  provenance/SHA256SUMS
  provenance/PIPER_LICENSE
  robot/vendor/piper_description/urdf/piper_description.urdf
  robot/vendor/piper_description/meshes/base_link.STL
  research/src/mpg/schema.py
  research/scripts/validate_offline_pipeline.py
)

for path in "${required[@]}"; do
  if [[ ! -f "$path" ]]; then
    echo "MISSING: $path" >&2
    exit 1
  fi
done

if find . -type l -print -quit | grep -q .; then
  echo "FAIL: bundle contains symlinks" >&2
  find . -type l -print >&2
  exit 1
fi

if find . -name .env -o -name '*.pem' -o -name '*.key' | grep -q .; then
  echo "FAIL: possible secret-bearing file found" >&2
  exit 1
fi

sha256sum --check provenance/SHA256SUMS
echo "PASS: required files, no symlinks, no obvious secret files, all checksums match"

