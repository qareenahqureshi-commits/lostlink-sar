# Concept and background

## Problem

Drones used for search and rescue, or for reconnaissance in contested areas, often have to work exactly where their two lifelines fail:

- **GPS** is blocked indoors, underground, or jammed.
- **The radio link** to the operator drops behind walls or under interference.

Without GPS, the position estimate drifts a little more with every metre flown. Without a link, nobody can help. A drone that keeps exploring can reach a point where it no longer knows its way back accurately enough, and it doesn't realise this until it's too late.

## What existing work does

| # | Work | What it does | What we take |
|---|---|---|---|
| 1 | **EGO-Swarm**, Zhou et al., *Science Robotics* 2022 ([arXiv](https://ar5iv.labs.arxiv.org/html/2011.04183), [code](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)) | Palm-sized drones flying through a forest with no GPS, each planning its own collision-free path | Local avoidance: bend the trajectory away from obstacles |
| 2 | **FUEL**, Zhou et al., *RA-L* 2021 ([arXiv](https://arxiv.org/pdf/2010.11561), [code](https://github.com/HKUST-Aerial-Robotics/FUEL)) | Fast exploration of unknown spaces using frontiers (edges of known space) | Frontier-based search of the building |
| 3 | **CERBERUS / GBPlanner**, NTNU-ETH, DARPA SubT winners 2021 ([paper](https://arxiv.org/pdf/2207.04914), [code](https://github.com/ntnu-arl/gbplanner_ros)) | Exploration of mines and caves with a guaranteed path home and battery-based return | Path-home graph; the baseline return rule we improve on |
| 4 | **SUPER**, HKU MaRS, *Science Robotics* 2025 ([code](https://github.com/hku-mars/SUPER)) | High-speed flight in unknown forests, always keeping a backup safe trajectory | The safety-first idea: always keep a way out |
| 5 | **Back to Newton's Laws**, SJTU 2024–25 ([arXiv](https://arxiv.org/pdf/2407.10648)) | A tiny network trained through differentiable physics, flying from low-resolution depth only | Path to running avoidance on very small hardware |
| 6 | **Depth-based avoidance on nano-UAVs**, Müller et al., ETH, 2023 ([arXiv](https://arxiv.org/pdf/2208.12624)) | Crazyflie avoiding obstacles with an 8×8 time-of-flight sensor, fully on board | Hardware bridge to our own lightweight drone |
| + | **Swarm Gradient Bug Algorithm**, TU Delft, *Science Robotics* 2019 ([news](https://tudelft.nl/en/2019/tu-delft/swarm-of-tiny-drones-explores-unknown-environments)) | Tiny drones explore a building and return to a beacon with minimal comms | Return-to-beacon in GPS- and comms-poor settings |

## The gap

Exploration planners like FUEL and EGO-Planner assume the drone always knows where it is. Systems like GBPlanner return home on a **battery or time budget**. None of them combine:

1. **localisation drift:** how wrong the position may now be, and
2. **link status:** whether the operator can still help or receive data

into the decision of **when to stop exploring and come back**.

## Our contribution

A return-decision layer that runs alongside the planner. At every step it estimates the risk of being unable to get home, using:

- position uncertainty from PX4's estimator (EKF2) since GPS was lost
- time and distance since the last good radio link
- remaining battery versus the cost of the known path home

When that risk crosses a threshold, the drone heads either to base or to the last point where it had a signal, so it can upload what it found.

## Mission scenario

1. Take off outside a damaged building and fly the QGroundControl mission to the entrance.
2. GPS is jammed at the doorway (PX4 failure injection), and the drone switches to visual odometry.
3. Comms drop inside the building. The drone keeps searching and avoiding debris on its own.
4. It decides when to turn back, and returns to the last-link point or to base.

## How it will be evaluated

Many randomised building layouts, comparing our return rule against (a) no return logic and (b) battery-only return:

- area searched
- return success rate
- final position error at home
- number of collisions

## Link to hardware

The full planner needs a companion computer. In parallel, the author is building a ~250 g custom quadcopter (Teensy 4.0 flight controller, custom PCB frame). The plan is to run a lightweight version on it: an 8×8 ToF sensor (VL53L5CX), a small learned avoidance policy, and a reverse-path homing routine when the link drops.
