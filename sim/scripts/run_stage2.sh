#!/usr/bin/env bash
# Stage 2: fly the sar_building mission in OFFBOARD with (or without) depth-camera avoidance.
# Usage: bash sim/scripts/run_stage2.sh [avoid|noavoid] [path/to/PX4-Autopilot]
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODE="${1:-avoid}"; PX4="${2:-$HOME/PX4-Autopilot}"; WS="$HOME/lostlink_ws"
NAME="run_$(date +%m%d_%H%M)_$MODE"; RUN="$REPO/results/stage2/$NAME"
mkdir -p "$RUN"
exec > >(tee "$RUN/console.txt") 2>&1
source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"
export PYTHONDONTWRITEBYTECODE=1

echo "== clean start =="
pkill -f 'bin/px4'; pkill -f 'gz sim'; pkill -f MicroXRCEAgent; sleep 3
python3 "$REPO/sim/worlds/make_stage1.py" --px4 "$PX4" > /dev/null
touch "$RUN/.start"

echo "== PX4 + Gazebo (GUI) + agent + QGroundControl =="
export PX4_GZ_FOLLOW_OFFSET_X=-7 PX4_GZ_FOLLOW_OFFSET_Y=-7 PX4_GZ_FOLLOW_OFFSET_Z=6
( cd "$PX4" && PX4_SYS_AUTOSTART=4002 PX4_SIM_MODEL=gz_x500_depth PX4_GZ_WORLD=sar_building \
  setsid ./build/px4_sitl_default/bin/px4 -d < /dev/null > "$RUN/px4.log" 2>&1 & )
( setsid MicroXRCEAgent udp4 -p 8888 < /dev/null > /tmp/lostlink_agent.log 2>&1 & )
pgrep -f QGroundControl > /dev/null || ( setsid "$HOME/Applications/QGroundControl.AppImage" < /dev/null > /dev/null 2>&1 & )
for i in $(seq 1 60); do grep -aq "Ready for takeoff" "$RUN/px4.log" && break; sleep 2; done
grep -aq "Ready for takeoff" "$RUN/px4.log" && echo "PX4 ready" || echo "WARNING: PX4 not ready after 120 s"
sleep 5

echo "== mission node (avoid=$MODE) =="
AV=true; [ "$MODE" = "noavoid" ] && AV=false
timeout 900 ros2 run lostlink_planner avoid_mission --ros-args \
  -p plan_file:="$REPO/sim/missions/stage1_obstacle_mission.plan" -p avoid:=$AV -p log_dir:="$RUN"
sleep 3

echo "== flight log + report =="
LOG=$(find "$PX4/build/px4_sitl_default" -name "*.ulg" -newer "$RUN/.start" | sort | tail -1)
echo "log: $LOG"
mkdir -p "$REPO/_archive"; cp "$LOG" "$REPO/_archive/stage2_$NAME.ulg"
python3 "$REPO/sim/scripts/flight_report.py" "$LOG" --stage "stage2/$NAME"
echo "RUN FINISHED: $NAME"
