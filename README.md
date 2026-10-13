# Lost-Link SAR

**A search-and-rescue drone that knows when it's about to lose its way home.**

A quadcopter flies a search mission inside a damaged building. At the entrance it loses GPS (jammed). Inside, it loses its radio link to the ground station. From then on it has to keep searching, avoid debris, and decide *by itself* when to turn back, before it drifts too far to make it home.

Built on PX4 + Gazebo + QGroundControl + ROS2, fully in simulation, with a path to a real lightweight drone.

![Before vs after: plain mission vs depth-camera avoidance](results/stage2/run_1012_1347_avoid/before_after.png)

*Same mission, same building. **Top (Stage 1):** a standard QGroundControl mission — 3 collisions and 31 s of tumbling (real path from Gazebo ground truth). **Bottom (Stage 2):** our ROS2 node with depth-camera mapping (green) and an EGO-Planner-style planner — 0 collisions, never closer than 0.9 m to anything, returns home and lands.*

![Stage 2 flight in Gazebo](media/stage2/stage2_gallery.jpg)

![Stage 4: GPS and radio link jammed at the door](results/stage4/run_1012_1618_stage4_d5_s0/stage4_paths.png)

*Stage 4, real PX4 + Gazebo flight: GPS **and** the radio link are jammed at the door. PX4's own position estimate (orange dotted) drifts 2.9 m, but the drone recognises the parts of the building it mapped with GPS, corrects itself (purple), decides on its own when to turn back, and lands **0.35 m** from home.*

---

## The idea in one table

| Piece | Built on | What it does |
|---|---|---|
| Local obstacle avoidance | EGO-Planner (Zhou et al., ZJU FAST Lab), re-implemented in ROS2 | Bends the flight path around obstacles seen by the depth camera |
| Path home | GBPlanner (NTNU ARL, DARPA SubT winner) | Keeps a graph of known-safe routes back to base |
| **Return decision (new)** | This project | Predicts how lost the drone will be by the time it is back at the exit. It turns back before that passes what it can still recognise (or the landing/battery budget), with the drift rate learned in flight. |
| Relocalisation against an anchor map | This project | Recognises the parts of the building it mapped while GPS still worked, and corrects its own drift |

Full background and references: [docs/01_concept.md](docs/01_concept.md)

## Progress

| Stage | Goal | Status | Write-up |
|---|---|---|---|
| 0 | Plain QGroundControl waypoint mission, empty world | ✅ Done | [stage0_baseline.md](docs/stage0_baseline.md) |
| 1 | Same mission through a damaged building: show it crashes | ✅ Done | [stage1_obstacle_world.md](docs/stage1_obstacle_world.md) |
| 2 | ROS2 offboard node: depth camera → obstacle avoidance | ✅ Done — 0 collisions | [stage2_avoidance.md](docs/stage2_avoidance.md) |
| 3 | GPS jammed at the doorway, drifting VIO | ✅ Done — 2 % drift: home but ~1.5 m off; 5 % drift: **trapped inside** (doorway vanished from its map). Drift breaks the way home before avoidance | [stage3_gps_denied.md](docs/stage3_gps_denied.md) |
| 4 | GPS **and** radio link jammed: drift-aware map, relocalisation, return decision | ✅ Done — real flights land **0.35–0.89 m** from home at 5–10 % drift (Stage 3: trapped, 20.8 m); offline 100 % safe return up to 10 % drift (Stage 3: 0 %) | [stage4_return_decision.md](docs/stage4_return_decision.md) |
| 5 | Random deep multi-room buildings (procedurally generated), better drift-rate learning, recognition alarm | ✅ Done (offline, 400 runs) — in deep buildings at 5 % drift the return decision lifts safe returns from 30 % to **100 %** with **0 collisions** (without it: 10 % of runs collide). Real flights: 0.64 m from home at 5 %; without the decision lost (25 m); at 10 % stuck at a dead end (fix next) | [stage5_random_buildings.md](docs/stage5_random_buildings.md) |

**How results are produced.** Two kinds of simulation:
- the **real simulator** (PX4 + Gazebo + ROS2) for full-stack flights;
- a fast **offline bench** that runs the *same* planning code against a lightweight model of the building and depth camera, for statistics over hundreds of runs (`sim/offline/`).

Each stage page says which numbers come from which.

Day-by-day problems and fixes: [docs/lab_notebook.md](docs/lab_notebook.md)

## Quick start

Tested on Windows 11 + WSL2 Ubuntu 22.04, ROS2 Humble, PX4 v1.18.0-beta1, Gazebo Harmonic (gz-sim 8). Full install guide: [docs/00_setup.md](docs/00_setup.md)

```bash
# Terminal 1: build the damaged-building world + mission and start the simulator
bash sim/scripts/run_stage1.sh

# Terminal 2: once PX4 says "Ready for takeoff", stop the corner cutting
bash sim/scripts/set_params.sh

# QGroundControl: Plan > Open > sim/missions/stage1_obstacle_mission.plan > Upload > Fly > start
# After landing: truth-vs-estimate report from the newest log
python3 sim/scripts/flight_report.py ~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/<date>/<time>.ulg --stage stage1

# Stage 2: obstacle avoidance (one-time build, then fly)
bash sim/scripts/setup_stage2.sh
bash sim/scripts/run_stage2.sh avoid

# Stage 3 / 4: GPS (and link) jammed at the door, drifting VIO
bash sim/scripts/run_stage3.sh 5 0
bash sim/scripts/run_stage4.sh stage4 5 0
python3 sim/offline/stage4_bench.py --policy stage4 --drift 10 --seed 0 --plot out.png   # no simulator needed

# Stage 5: random buildings (offline Monte Carlo, or a real flight in building 0)
python3 sim/worlds/make_building.py --seed 0 --plot
bash sim/scripts/run_stage5.sh stage4 5 0
```

## Repository layout

```
docs/          concept, setup guide, one page per stage, lab notebook
sim/worlds/    Gazebo world generator (obstacles + matching mission in one place)
sim/missions/  QGroundControl .plan files
sim/scripts/   run, parameter, log-analysis and plotting helpers
ros2_ws/       ROS2 package lostlink_planner (perception, mapping, planner, PX4 offboard node)
sim/offline/   offline test bench for the planner (no PX4/Gazebo needed)
results/       figures and data per stage
media/         screenshots and videos
```

Flight logs (`.ulg`) are too large for the repo and are attached to GitHub Releases.

## Author

Kareena Mueenuddin, B.Tech Mechatronics Engineering (Amity University Dubai). Robotics, autonomous systems, UAV path planning.
