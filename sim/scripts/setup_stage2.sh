#!/usr/bin/env bash
# Stage 2 one-time setup: build px4_msgs (matching THIS PX4 checkout) + lostlink_planner in ~/lostlink_ws.
# Usage: bash sim/scripts/setup_stage2.sh [path/to/PX4-Autopilot]
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4="${1:-$HOME/PX4-Autopilot}"
WS="$HOME/lostlink_ws"
mkdir -p "$REPO/results/stage2"
exec > >(tee "$REPO/results/stage2/setup_log.txt") 2>&1
source /opt/ros/humble/setup.bash
mkdir -p "$WS/src"

echo "== px4_msgs =="
if [ ! -d "$WS/src/px4_msgs" ]; then
  if [ -d "$HOME/px4_ros2_ws/src/px4_msgs" ]; then cp -r "$HOME/px4_ros2_ws/src/px4_msgs" "$WS/src/"
  else git clone https://github.com/PX4/px4_msgs.git "$WS/src/px4_msgs"; fi
fi
# message definitions must match the PX4 firmware exactly (v1.16+ uses versioned topics)
rm -f "$WS/src/px4_msgs/msg/"*.msg
cp "$PX4"/msg/*.msg "$WS/src/px4_msgs/msg/"
[ -d "$PX4/msg/versioned" ] && cp "$PX4"/msg/versioned/*.msg "$WS/src/px4_msgs/msg/"
if [ -d "$PX4/srv" ]; then mkdir -p "$WS/src/px4_msgs/srv"; rm -f "$WS/src/px4_msgs/srv/"*.srv; cp "$PX4"/srv/*.srv "$WS/src/px4_msgs/srv/"; fi
echo "$(ls "$WS/src/px4_msgs/msg" | wc -l) message files copied from $PX4"

echo "== lostlink_planner (linked from repo) =="
ln -sfn "$REPO/ros2_ws/src/lostlink_planner" "$WS/src/lostlink_planner"

echo "== build (px4_msgs takes a few minutes) =="
cd "$WS"
colcon build --symlink-install --packages-select px4_msgs lostlink_planner --event-handlers console_cohesion- 2>&1 | tail -20
source install/setup.bash

echo "== checks =="
python3 -c "from px4_msgs.msg import VehicleStatus, VehicleOdometry, TrajectorySetpoint, VehicleCommand; print('px4_msgs OK - VehicleStatus version', VehicleStatus.MESSAGE_VERSION)"
python3 -c "import lostlink_planner.core; print('lostlink_planner OK')"
python3 -c "from gz.transport13 import Node; from gz.msgs10.image_pb2 import Image; print('gz-transport OK')"
echo "SETUP DONE"
