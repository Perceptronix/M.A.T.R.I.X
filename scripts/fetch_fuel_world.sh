#!/usr/bin/env bash
# Fetches the Rubicon Gazebo Fuel world model and places it where
# ugv_sim's launch file (GZ_SIM_RESOURCE_PATH) expects it:
#   src/ugv_sim/worlds/fuel/rubicon_world/models/Rubicon/
#
# This content is NOT committed to git (meshes/rubicon.dae alone is ~190MB,
# over GitHub's 100MB per-file limit). It is a verbatim copy of the public
# model at https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rubicon, so we
# just re-download it with `gz fuel download` instead of vendoring it.
#
# Usage: run once after cloning (and again any time the directory below is
# missing), from anywhere:
#   bash scripts/fetch_fuel_world.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$REPO_ROOT/src/ugv_sim/worlds/fuel/rubicon_world/models/Rubicon"
FUEL_URL="https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rubicon"
CACHE_BASE="$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/rubicon"

if [ -d "$DEST" ] && [ -n "$(ls -A "$DEST" 2>/dev/null)" ]; then
    echo "[fetch_fuel_world] $DEST already populated -- nothing to do."
    echo "[fetch_fuel_world] delete it first if you want to re-fetch."
    exit 0
fi

command -v gz >/dev/null 2>&1 || {
    echo "[fetch_fuel_world] ERROR: 'gz' (Gazebo CLI) not found on PATH."
    echo "  Install Gazebo Harmonic / ros-jazzy-ros-gz first, then re-run this script."
    exit 1
}

echo "[fetch_fuel_world] Downloading Rubicon model from Gazebo Fuel..."
gz fuel download -u "$FUEL_URL"

# gz fuel caches by version under .../rubicon/<N>/ -- pick the newest one.
LATEST_VERSION_DIR="$(ls -d "$CACHE_BASE"/*/ 2>/dev/null | sort -V | tail -n1)"
if [ -z "${LATEST_VERSION_DIR:-}" ]; then
    echo "[fetch_fuel_world] ERROR: download appeared to succeed but no cached"
    echo "  model directory was found under $CACHE_BASE"
    exit 1
fi

mkdir -p "$DEST"
cp -r "${LATEST_VERSION_DIR%/}"/. "$DEST"/
echo "[fetch_fuel_world] Done. Installed to: $DEST"
