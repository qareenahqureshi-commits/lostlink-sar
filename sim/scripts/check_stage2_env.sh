#!/usr/bin/env bash
# Stage 2.1 - environment check for ROS2 offboard control + depth camera.
# Starts PX4 SITL headless in sar_building, inspects topics, then shuts everything down.
# Report: results/stage2/env_check.txt
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4="${1:-$HOME/PX4-Autopilot}"
OUT="$REPO/results/stage2/env_check.txt"
mkdir -p "$REPO/results/stage2"
exec > >(tee "$OUT") 2>&1
source /opt/ros/humble/setup.bash 2>/dev/null

sec(){ echo; echo "===== $1 ====="; }
sec "versions"
lsb_release -d; echo "ROS_DISTRO=$ROS_DISTRO"; (cd "$PX4" && git describe --tags); gz sim --versions | head -1
python3 --version

sec "ROS2 packages of interest"
ros2 pkg list 2>/dev/null | grep -E "^(ros_gz|ros_gz_bridge|ros_gz_image|px4_msgs|px4_ros_com|cv_bridge|vision_opencv)" || echo "(none of ros_gz / px4_msgs / cv_bridge found in sourced ROS)"
dpkg -l | grep -E "ros-humble-ros-gz|ros-humble-cv-bridge|ros-humble-ros-gzharmonic" | awk '{print $2, $3}'

sec "px4_msgs workspaces on disk"
find "$HOME" -maxdepth 4 -type d -name px4_msgs 2>/dev/null
ls -d "$HOME"/*_ws "$HOME"/ros2_ws 2>/dev/null

sec "python modules"
for m in numpy scipy cv2 gz.transport13 gz.msgs10 pyulog; do
  python3 -c "import $m" 2>/dev/null && echo "OK   $m" || echo "MISS $m"
done

sec "Micro XRCE-DDS agent"
which MicroXRCEAgent || echo "MicroXRCEAgent not on PATH"

sec "start headless sim + agent"
pkill -f 'bin/px4' ; pkill -f 'gz sim'; pkill -f MicroXRCEAgent; sleep 2
python3 "$REPO/sim/worlds/make_stage1.py" --px4 "$PX4" > /dev/null
# run the PX4 binary directly in daemon mode (-d: no interactive pxh shell), fully detached from this terminal
( cd "$PX4" && HEADLESS=1 PX4_SYS_AUTOSTART=4002 PX4_SIM_MODEL=gz_x500_depth PX4_GZ_WORLD=sar_building \
  setsid ./build/px4_sitl_default/bin/px4 -d < /dev/null > /tmp/px4_check.log 2>&1 & )
( setsid MicroXRCEAgent udp4 -p 8888 < /dev/null > /tmp/agent_check.log 2>&1 & )
echo "waiting 75 s for PX4 + Gazebo..."; sleep 75
grep -aE "Ready for takeoff|ERROR|gz_bridge" /tmp/px4_check.log | head -8

sec "gz topics (camera / depth)"
gz topic -l | grep -iE "camera|depth|image|points"
for t in $(gz topic -l | grep -iE "depth|points" ); do echo "--- $t"; timeout 8 gz topic -i -t "$t" | head -6; done
D=$(gz topic -l | grep -iE "depth_camera$|depth_image$" | head -1)
[ -n "$D" ] && { echo "--- one depth message header from $D:"; timeout 8 gz topic -e -n 1 -t "$D" | grep -E "width|height|pixel_format|step" | head -6; }
sec "camera sensor definition (FOV, range)"
grep -rhoE "<horizontal_fov>[^<]+|<near>[^<]+|<far>[^<]+|<width>[^<]+|<height>[^<]+|<update_rate>[^<]+" "$PX4"/Tools/simulation/gz/models/OakD-Lite/ 2>/dev/null | sort | uniq -c
grep -rhoE "<pose[^>]*>[^<]+</pose>" "$PX4"/Tools/simulation/gz/models/x500_depth/model.sdf 2>/dev/null

sec "ROS2 topics from PX4 (via agent)"
timeout 15 ros2 topic list 2>/dev/null | grep fmu | head -60

sec "shutdown"
pkill -f 'bin/px4'; pkill -f 'gz sim'; pkill -f MicroXRCEAgent
echo "DONE"
