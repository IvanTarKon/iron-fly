#!/usr/bin/env bash
# Restore the exact upstream stack, without updating existing checkouts.
set -euo pipefail
cd "$(dirname "$0")/.."
sim=vendor/Hexapod-Simulation
pin=fc03ebf8b222031c40dc003c95f96d7f96302cf9
if [ ! -e "$sim" ]; then
  mkdir -p vendor
  git clone --no-checkout https://github.com/ggldnl/Hexapod-Simulation.git "$sim"
  git -C "$sim" checkout --detach "$pin"
fi
if [ "$(git -C "$sim" rev-parse HEAD)" != "$pin" ]; then
  echo 'Existing Simulation has a different version; refusing to change it.' >&2
  exit 1
fi
git -C "$sim" submodule update --init --recursive
fw="$sim/Hexapod-Firmware"
patch_file="$PWD/patches/0001-reject-whole-pose.patch"
if git -C "$fw" apply --check "$patch_file" 2>/dev/null; then
  git -C "$fw" apply "$patch_file"
elif ! git -C "$fw" apply --reverse --check "$patch_file" 2>/dev/null; then
  echo 'Firmware patch conflicts with existing changes; stopping.' >&2
  exit 1
fi
"${PYTHON:-python3}" scripts/verify_sources.py
