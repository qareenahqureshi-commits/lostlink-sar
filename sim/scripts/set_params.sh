#!/usr/bin/env bash
# Set project PX4 parameters on a RUNNING PX4 SITL instance (run in a second terminal).
# NAV_ACC_RAD: default 10 m makes multicopters switch waypoints ~8 m early (corner cutting).
PX4="${1:-$HOME/PX4-Autopilot}"
B="$PX4/build/px4_sitl_default/bin"
cd "$PX4/build/px4_sitl_default/rootfs"
"$B/px4-param" set NAV_ACC_RAD 0.5
"$B/px4-param" show NAV_ACC_RAD
