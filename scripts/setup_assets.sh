#!/usr/bin/env bash
# Fetch the MuJoCo Menagerie models this project depends on and link their mesh
# files into assets/ (where the scene XMLs look for them via meshdir="../assets").
#
# Both mujoco_menagerie/ and assets/ are gitignored, so a fresh clone must run
# this once before the simulation can load. Idempotent: safe to re-run.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

MODELS=(universal_robots_ur5e robotiq_2f85)

# 1. Sparse-clone only the two models we use (keeps the checkout small).
if [ ! -d mujoco_menagerie/.git ]; then
  echo "Cloning mujoco_menagerie (sparse: ${MODELS[*]})..."
  git clone --depth 1 --filter=blob:none --sparse \
    https://github.com/google-deepmind/mujoco_menagerie.git
  git -C mujoco_menagerie sparse-checkout set "${MODELS[@]}"
fi

# 2. Flatten both models' mesh assets into assets/ (ur5e .obj + robotiq .stl).
mkdir -p assets
for d in "${MODELS[@]}"; do
  for f in "$ROOT/mujoco_menagerie/$d/assets/"*; do
    ln -sf "$f" "assets/$(basename "$f")"
  done
done

# 3. Convenience symlink kept for compatibility.
ln -sfn mujoco_menagerie/robotiq_2f85/assets robotiq_assets

echo "assets/ ready ($(find assets -maxdepth 1 -type l | wc -l) mesh symlinks)"
