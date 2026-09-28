#!/usr/bin/env bash
# Fetch upstream packages into src/ (not part of this git overlay).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
mkdir -p src

if command -v vcs >/dev/null 2>&1; then
  vcs import --input "$ROOT/hsr.repos" src
  exit 0
fi

clone() {
  local name="$1" url="$2" branch="$3"
  if [[ -d "src/${name}/.git" ]]; then
    git -C "src/${name}" fetch origin
    git -C "src/${name}" checkout "$branch"
    git -C "src/${name}" pull --ff-only origin "$branch" || true
    return
  fi
  git clone -b "$branch" "$url" "src/${name}"
}

clone hsrb_common https://github.com/hsr-project/hsrb_common.git humble
clone hsrb_rosnav https://github.com/hsr-project/hsrb_rosnav.git humble
clone tmc_database https://github.com/hsr-project/tmc_database.git humble
