#!/usr/bin/env bash
# Stage 5: like Stage 4 (GPS + radio link jammed at the entrance, drifting VIO), but in a GENERATED multi-room building.
# Usage: bash sim/scripts/run_stage5.sh <policy> <drift_pct> <building_seed> [r_hat] [PX4 path]
#   policy: stage4 (full) | map_reloc | stage3 | rtl_link        e.g.  bash sim/scripts/run_stage5.sh stage4 5 0
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
POLICY="${1:-stage4}"; DRIFT="${2:-5}"; BSEED="${3:-0}"; SEED="$BSEED"; DENY="14.0"; RHAT="${4:-0.05}"
PX4="${5:-$HOME/PX4-Autopilot}"; WS="$HOME/lostlink_ws"
WORLD="lostlink_b${BSEED}"; SPEC="$REPO/sim/worlds/generated/building_${BSEED}.json"
NAME="run_$(date +%m%d_%H%M)_b${BSEED}_${POLICY}_d${DRIFT}"; RUN="$REPO/results/stage5/$NAME"; mkdir -p "$RUN"
exec > >(tee "$RUN/console.txt") 2>&1
source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"
export PYTHONDONTWRITEBYTECODE=1
B="$PX4/build/px4_sitl_default/bin"; P(){ ( cd "$PX4/build/px4_sitl_default/rootfs" && "$B/px4-$1" "${@:2}" ); }
f(){ python3 -c "print(float('$1'))"; }

echo "== clean start (building ${BSEED}, policy ${POLICY}, drift ${DRIFT}% seed ${SEED}, GPS+link jammed at East > ${DENY} m, r_hat ${RHAT}) =="
for i in 1 2 3 4 5; do
  pkill -9 -f 'bin/px4'; pkill -9 -f 'gz sim'; pkill -9 -f MicroXRCEAgent; pkill -9 -f 'lib/lostlink_planner/'
  sleep 2; pgrep -f 'gz sim|bin/px4|lib/lostlink_planner/' > /dev/null || break
done
pgrep -af 'gz sim|bin/px4|lib/lostlink_planner/' && echo "WARNING: old processes still alive" || echo "clean"
sleep 2
python3 "$REPO/sim/worlds/make_building.py" --seed "$BSEED" --px4 "$PX4"
touch "$RUN/.start"
export PX4_GZ_FOLLOW_OFFSET_X=-7 PX4_GZ_FOLLOW_OFFSET_Y=-7 PX4_GZ_FOLLOW_OFFSET_Z=6
( cd "$PX4" && PX4_SYS_AUTOSTART=4002 PX4_SIM_MODEL=gz_x500_depth PX4_GZ_WORLD=$WORLD \
  setsid ./build/px4_sitl_default/bin/px4 -d < /dev/null > "$RUN/px4.log" 2>&1 & )
( setsid MicroXRCEAgent udp4 -p 8888 < /dev/null > /tmp/lostlink_agent.log 2>&1 & )
pgrep -f QGroundControl > /dev/null || ( setsid "$HOME/Applications/QGroundControl.AppImage" < /dev/null > /dev/null 2>&1 & )
for i in $(seq 1 60); do grep -aq "Ready for takeoff" "$RUN/px4.log" && break; sleep 2; done
grep -aq "Ready for takeoff" "$RUN/px4.log" && echo "PX4 ready" || echo "WARNING: PX4 not ready after 120 s"

echo "== Stage 3/4 parameters =="
P param set SYS_FAILURE_EN 1     # allow failure injection (GPS jamming)
P param set EKF2_EV_CTRL 5       # fuse external vision: horizontal position (1) + velocity (4)
P param set EKF2_HGT_REF 0       # height from barometer
P param set EKF2_EV_DELAY 0
P param set NAV_DLL_ACT 0        # no automatic RTL on QGC link loss: the comms loss is simulated by sim_link
P param set EKF2_GPS_CTRL 7

echo "== simulated VIO + radio link =="
( setsid ros2 run lostlink_planner sim_vio --ros-args -p drift_pct:=$(f $DRIFT) -p seed:=$SEED -p world:=$WORLD < /dev/null > "$RUN/sim_vio.log" 2>&1 & )
( setsid ros2 run lostlink_planner sim_link --ros-args -p world_py:="$SPEC" -p jam_link:=true -p world:=$WORLD < /dev/null > "$RUN/sim_link.log" 2>&1 & )
sleep 15
P listener estimator_status_flags 1 | grep -E "cs_gnss_pos|cs_ev_pos|cs_ev_vel"

echo "== mission node (policy $POLICY) =="
timeout 2400 ros2 run lostlink_planner avoid_mission --ros-args \
  -p plan_file:="$REPO/sim/missions/building_${BSEED}.plan" -p avoid:=true -p log_dir:="$RUN" \
  -p gps_deny_east:=$(f $DENY) -p max_mission_s:=1800.0 -p policy:=$POLICY -p r_hat:=$(f $RHAT) \
  -p world:=$WORLD -p world_spec:="$SPEC"
sleep 3
echo "== estimator after flight =="; P listener estimator_status_flags 1 | grep -E "cs_gnss_pos|cs_ev_pos|cs_inertial_dead"
pkill -f 'lostlink_planner/sim_vio'; pkill -f 'lostlink_planner/sim_link'
LOG=$(find "$PX4/build/px4_sitl_default" -name "*.ulg" -newer "$RUN/.start" | sort | tail -1)
mkdir -p "$REPO/_archive"; cp "$LOG" "$REPO/_archive/stage5_$NAME.ulg"; echo "log: $LOG"
echo "RUN FINISHED: $NAME"
