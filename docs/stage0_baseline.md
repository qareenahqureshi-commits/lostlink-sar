# Stage 0: Baseline mission

**Date:** 5 Oct 2026  **Status:** ✅ Done

## Goal

Confirm the toolchain works end to end: PX4 SITL ↔ Gazebo ↔ QGroundControl, and fly a standard waypoint mission. This is the reference everything else is compared against.

## Setup

- Simulator: `make px4_sitl gz_x500_depth` (default empty world)
- Bridge to ROS2 started for later stages: `MicroXRCEAgent udp4 -p 8888`
- Home: PX4 default location (Zurich, 47.3980° N, 8.5462° E, 488 m AMSL)

## Mission (planned in QGroundControl)

| Item | Detail |
|---|---|
| Takeoff | to 50 m above home (QGroundControl default altitude) |
| Waypoints | 4, in a loop around home |
| Return | Return To Launch (RTL) |
| Length / time | ~235 m, ~3 min estimated |

File: `sim/missions/stage0_baseline_mission.plan`

## How to run it

1. `cd ~/PX4-Autopilot && make px4_sitl gz_x500_depth`
2. QGroundControl → **Plan** → File → Open → the `.plan` file → **Upload**
3. **Fly** → slide "Start Mission"

## What happened

The drone took off, flew all four waypoints, returned and landed with no issues.

## What we learned

- The full chain works: the mission is uploaded from QGroundControl, flown by PX4, and simulated by Gazebo.
- **The satellite map in QGroundControl is only a background.** The buildings shown on it don't exist in Gazebo, whose default world is an empty plane, so this mission can't hit anything.
- At 50 m, the drone is above everything. Obstacle work has to happen at low altitude (3–5 m).

## Next

Stage 1: put real obstacles in the world and fly at 3 m.
