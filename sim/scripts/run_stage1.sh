#!/usr/bin/env bash
# Stage 1: generate the sar_building world + mission, then launch PX4 SITL in it.
# Usage (from anywhere):  bash sim/scripts/run_stage1.sh [path/to/PX4-Autopilot]
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4="${1:-$HOME/PX4-Autopilot}"
mkdir -p "$REPO/results/stage1"
LOG="$REPO/results/stage1/run_log.txt"
exec > >(tee "$LOG") 2>&1

echo "== system =="; lsb_release -d; echo "ROS_DISTRO=$ROS_DISTRO"
echo "PX4: $PX4 ($(cd "$PX4" && git describe --tags))"
gz sim --versions 2>/dev/null | head -1

echo "== stopping any running sim =="
pkill -f 'bin/px4' || true; pkill -f 'gz sim' || true; sleep 3

echo "== generate world + mission =="
python3 "$REPO/sim/worlds/make_stage1.py" --px4 "$PX4"

echo "== launch PX4 SITL (sar_building, x500_depth) =="
echo "   In QGroundControl: Plan > Open > sim/missions/stage1_obstacle_mission.plan > Upload > Fly"
echo "   Remember: NAV_ACC_RAD must be 0.5 (bash sim/scripts/set_params.sh)"
cd "$PX4"
PX4_GZ_WORLD=sar_building make px4_sitl gz_x500_depth
