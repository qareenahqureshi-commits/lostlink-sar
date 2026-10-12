# Setup guide

How to get from a fresh Windows laptop to running this project.

## Verified environment

| Component | Version used |
|---|---|
| Host | Windows 11, WSL2 (GUI apps through WSLg) |
| Linux | Ubuntu 22.04.5 LTS |
| ROS2 | Humble |
| PX4-Autopilot | v1.18.0-beta1 (`~/PX4-Autopilot`) |
| Gazebo | Harmonic (gz-sim 8.15) |
| Ground station | QGroundControl, running inside WSL |
| Drone model | `gz_x500_depth` (X500 quadcopter with a depth camera) |

> This is the standard install route. If you set up your machine differently, the versions in the table are what matter.

## 1. WSL2 + Ubuntu 22.04

In Windows PowerShell (as administrator):

```powershell
wsl --install -d Ubuntu-22.04
```

Restart, open "Ubuntu 22.04" and create your user.

## 2. ROS2 Humble

Follow the official guide: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>. Then add this line to `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
```

## 3. PX4 + Gazebo

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh      # installs build tools and Gazebo
# restart the terminal, then a first build to check everything works:
cd ~/PX4-Autopilot && make px4_sitl gz_x500
```

## 4. Micro XRCE-DDS Agent (PX4 ↔ ROS2 bridge, needed from Stage 2)

```bash
cd ~
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make && sudo make install && sudo ldconfig /usr/local/lib/
# run with:  MicroXRCEAgent udp4 -p 8888
```

## 5. QGroundControl

Download the Linux AppImage from <https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html>, then:

```bash
chmod +x QGroundControl*.AppImage && ./QGroundControl*.AppImage
```

It connects to the PX4 simulator automatically over UDP.

## 6. Python tools for analysis

```bash
pip3 install --user pyulog pandas matplotlib
```

## 7. This repository

The repo lives on the Windows side, and WSL sees it under `/mnt/c/...`:

```bash
cd "/mnt/c/Users/karee/Downloads/projects 2026/custom quadcopter hardware/px4 simulation"
bash sim/scripts/run_stage1.sh
```

## Project-specific PX4 setting

| Parameter | Default | Ours | Why |
|---|---|---|---|
| `NAV_ACC_RAD` | 10 m | **0.5 m** | With 10 m, the drone counts a waypoint as reached 8–10 m early and cuts every corner. Set it with `sim/scripts/set_params.sh`. It stays saved in the SITL parameter file. |

## Known quirks (WSL)

- **The simulation runs slower than real time** (about 15–65% real-time factor here, with no GPU acceleration). Everything still works; missions just take longer on the clock.
- **Holding spacebar in QGroundControl can show "Vehicle rejected arming".** The first press already armed the drone and started the mission, so check the mode at the top bar before retrying.
- **Stopping a run:** press Ctrl+C in the PX4 terminal or type `shutdown` at `pxh>`. To stop everything at once: `pkill -f bin/px4; pkill -f "gz sim"; pkill -f QGroundControl`.
