#!/usr/bin/env bash
# Stage 3: same mission, but GPS is jammed at the doorway; the drone continues on simulated VIO with drift.
# Usage: bash sim/scripts/run_stage3.sh <drift_pct> [seed] [gps_deny_east_m] [PX4 path]
#   e.g. bash sim/scripts/run_stage3.sh 2 0        (2 % drift, seed 0, jam at East > 14 m)
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DRIFT="${1:-2}"; SEED="${2:-0}"; DENY="${3:-14.0}"; PX4="${4:-$HOME/PX4-Autopilot}"; WS="$HOME/lostlink_ws"
NAME="run_$(date +%m%d_%H%M)_d${DRIFT}_s${SEED}"; RUN="$REPO/results/stage3/$NAME"; mkdir -p "$RUN"
exec > >(tee "$RUN/console.txt") 2>&1
source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"
export PYTHONDONTWRITEBYTECODE=1
B="$PX4/build/px4_sitl_default/bin"; P(){ ( cd "$PX4/build/px4_sitl_default/rootfs" && "$B/px4-$1" "${@:2}" ); }

echo "== clean start (drift ${DRIFT}% seed ${SEED}, GPS jammed at East > ${DENY} m) =="
for i in 1 2 3 4 5; do
  pkill -9 -f 'bin/px4'; pkill -9 -f 'gz sim'; pkill -9 -f MicroXRCEAgent; pkill -9 -f 'lib/lostlink_planner/'
  sleep 2; pgrep -f 'gz sim|bin/px4|lib/lostlink_planner/' > /dev/null || break
done
pgrep -af 'gz sim|bin/px4|lib/lostlink_planner/' && echo "WARNING: old processes still alive" || echo "clean"
sleep 2
python3 "$REPO/sim/worlds/make_stage1.py" --px4 "$PX4" > /dev/null
touch "$RUN/.start"
export PX4_GZ_FOLLOW_OFFSET_X=-7 PX4_GZ_FOLLOW_OFFSET_Y=-7 PX4_GZ_FOLLOW_OFFSET_Z=6
( cd "$PX4" && PX4_SYS_AUTOSTART=4002 PX4_SIM_MODEL=gz_x500_depth PX4_GZ_WORLD=sar_building \
  setsid ./build/px4_sitl_default/bin/px4 -d < /dev/null > "$RUN/px4.log" 2>&1 & )
( setsid MicroXRCEAgent udp4 -p 8888 < /dev/null > /tmp/lostlink_agent.log 2>&1 & )
pgrep -f QGroundControl > /dev/null || ( setsid "$HOME/Applications/QGroundControl.AppImage" < /dev/null > /dev/null 2>&1 & )
for i in $(seq 1 60); do grep -aq "Ready for takeoff" "$RUN/px4.log" && break; sleep 2; done
grep -aq "Ready for takeoff" "$RUN/px4.log" && echo "PX4 ready" || echo "WARNING: PX4 not ready after 120 s"

echo "== Stage 3 parameters =="
P param set SYS_FAILURE_EN 1     # allow failure injection (GPS jamming)
P param set EKF2_EV_CTRL 5       # fuse external vision: horizontal position (1) + velocity (4)
P param set EKF2_HGT_REF 0       # height from barometer (GPS height disappears when jammed)
P param set EKF2_EV_DELAY 0
P param set NAV_DLL_ACT 0        # no automatic RTL on ground-station link loss (Stage 4 handles comms loss itself)
P param set EKF2_GPS_CTRL 7

echo "== simulated VIO =="
( setsid ros2 run lostlink_planner sim_vio --ros-args -p drift_pct:=$(python3 -c "print(float('$DRIFT'))") -p seed:=$SEED < /dev/null > "$RUN/sim_vio.log" 2>&1 & )
sleep 15
P listener estimator_status_flags 1 | grep -E "cs_gnss_pos|cs_ev_pos|cs_ev_vel"

echo "== mission node =="
timeout 1500 ros2 run lostlink_planner avoid_mission --ros-args \
  -p plan_file:="$REPO/sim/missions/stage1_obstacle_mission.plan" -p avoid:=true -p log_dir:="$RUN" \
  -p gps_deny_east:=$DENY -p max_mission_s:=900.0
sleep 3
echo "== estimator after flight =="; P listener estimator_status_flags 1 | grep -E "cs_gnss_pos|cs_ev_pos|cs_inertial_dead"
pkill -f 'lostlink_planner/sim_vio'
LOG=$(find "$PX4/build/px4_sitl_default" -name "*.ulg" -newer "$RUN/.start" | sort | tail -1)
mkdir -p "$REPO/_archive"; cp "$LOG" "$REPO/_archive/stage3_$NAME.ulg"; echo "log: $LOG"
echo "RUN FINISHED: $NAME"
