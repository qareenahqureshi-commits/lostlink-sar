#!/usr/bin/env bash
# Stage 3.0 - environment check for GPS denial + simulated VIO.
# 1) Gazebo ground-truth pose topic  2) PX4 vs Gazebo clock  3) does GPS failure injection work in this SITL?
# Report: results/stage3/env_check.txt
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4="${1:-$HOME/PX4-Autopilot}"
OUT="$REPO/results/stage3/env_check.txt"; mkdir -p "$REPO/results/stage3"
exec > >(tee "$OUT") 2>&1
source /opt/ros/humble/setup.bash; source "$HOME/lostlink_ws/install/setup.bash" 2>/dev/null
B="$PX4/build/px4_sitl_default/bin"
P(){ ( cd "$PX4/build/px4_sitl_default/rootfs" && "$B/px4-$1" "${@:2}" ); }
sec(){ echo; echo "===== $1 ====="; }

sec "start headless sim + agent"
pkill -f 'bin/px4'; pkill -f 'gz sim'; pkill -f MicroXRCEAgent; sleep 2
python3 "$REPO/sim/worlds/make_stage1.py" --px4 "$PX4" > /dev/null
( cd "$PX4" && HEADLESS=1 PX4_SYS_AUTOSTART=4002 PX4_SIM_MODEL=gz_x500_depth PX4_GZ_WORLD=sar_building \
  setsid ./build/px4_sitl_default/bin/px4 -d < /dev/null > /tmp/px4_s3.log 2>&1 & )
( setsid MicroXRCEAgent udp4 -p 8888 < /dev/null > /tmp/agent_s3.log 2>&1 & )
echo "waiting 75 s..."; sleep 75
grep -a "Ready for takeoff" /tmp/px4_s3.log | head -1

sec "1. Gazebo pose topics"
gz topic -l | grep -iE "pose|odom|clock"
echo "--- one dynamic_pose message (names + header):"
timeout 8 gz topic -e -n 1 -t /world/sar_building/dynamic_pose/info | grep -nE "sec|nsec|name|x500" | head -20
echo "--- pose/info x500 entry:"
timeout 8 gz topic -e -n 1 -t /world/sar_building/pose/info | grep -n -A12 'name: "x500_depth_0"' | head -16

sec "2. clocks (sample both within ~1 s)"
timeout 6 gz topic -e -n 1 -t /world/sar_building/clock | head -8
timeout 10 ros2 topic echo --once /fmu/out/vehicle_odometry 2>/dev/null | grep -E "^timestamp"
timeout 6 gz topic -e -n 1 -t /world/sar_building/clock | head -8
echo "--- depth image header:"
timeout 8 gz topic -e -n 1 -t /depth_camera | grep -A4 "header" | head -6

sec "3. params"
for p in SYS_FAILURE_EN EKF2_GPS_CTRL EKF2_EV_CTRL EKF2_EV_DELAY EKF2_HGT_REF EKF2_EVP_NOISE COM_POS_FS_DELAY NAV_DLL_ACT; do P param show $p | grep -E "^\s*[x+* ]*\s*$p"; done

sec "4. GPS failure injection test (on the ground)"
echo "--- before:"; P listener estimator_status_flags 1 | grep -E "cs_gnss_pos|cs_gnss_vel|cs_ev_pos|cs_inertial_dead"
P listener sensor_gps 1 | grep -E "fix_type|satellites_used"
P param set SYS_FAILURE_EN 1
P failure gps off; sleep 8
echo "--- after 'failure gps off' (8 s):"; P listener estimator_status_flags 1 | grep -E "cs_gnss_pos|cs_gnss_vel|cs_ev_pos|cs_inertial_dead"
P listener sensor_gps 1 | grep -E "fix_type|satellites_used|timestamp:"
grep -aiE "gps|gnss|fail" /tmp/px4_s3.log | tail -8
echo "--- alternative: EKF2_GPS_CTRL=0 (stop fusing GPS)"
P failure gps ok; sleep 3; P param set EKF2_GPS_CTRL 0; sleep 6
P listener estimator_status_flags 1 | grep -E "cs_gnss_pos|cs_gnss_vel|cs_inertial_dead"
P param set EKF2_GPS_CTRL 7; P param set SYS_FAILURE_EN 0

sec "shutdown"
pkill -f 'bin/px4'; pkill -f 'gz sim'; pkill -f MicroXRCEAgent
echo DONE
