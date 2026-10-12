# Lab notebook

A dated log of decisions, problems and fixes. Newest at the bottom.

---

### 2026-10-05: Project direction

- Reviewed 6 papers and repos on drone path planning and obstacle avoidance in GPS-denied and unknown environments (see [01_concept.md](01_concept.md)).
- **Decision:** combine EGO-Planner-style local avoidance with GBPlanner-style path-home. The new part is a return decision based on drift, link status and battery.
- **Decision:** EGO-Planner is ROS1, and we're on ROS2 Humble. We'll re-implement the core idea in ROS2 ourselves rather than run an unofficial port, so every line can be explained.
- **Decision:** work in stages (0–4), each with a figure, so there's always something to show.

### 2026-10-05: Stage 0

- QGroundControl mission (takeoff, 4 waypoints, RTL, ~235 m) flew fine in the empty default world.
- Note: the QGroundControl satellite map is only a background, and Gazebo's world is empty.

### 2026-10-05: Stage 1 world

- Wrote `make_stage1.py`: reads PX4's `default.sdf`, adds the obstacles, and writes the matching `.plan` using the same GPS origin. A single source for geometry avoids misalignment between the map and the world.
- Tested the generator against PX4's current `default.sdf` before running.

### 2026-10-05: Stage 1 runs

- **Problem:** run 1 didn't collide. The log showed waypoint switches about 8 m before each waypoint.
  - Tried: acceptance radius 0.5 m + 1 s hold time per waypoint in the `.plan` → no change (run 2).
  - Found: PX4 docs say multicopters switch waypoint on entering `NAV_ACC_RAD` (default 10 m).
  - **Fix:** `param set NAV_ACC_RAD 0.5` on the running SITL → run 3 flew the legs as drawn.
- **Result:** collisions with pillar 1 and the partition wall. PX4 recovered both times.
- **Problem:** the debris block (2.5 m) was below the flight altitude (3 m). **Fix:** raised to 3.5 m in the generator. Not yet re-flown.
- **Observation:** after the wall impact, the EKF position was unreliable for several seconds (speed spike, altitude below ground). Worth noting for the localisation part of the paper.
- **Environment note:** WSL real-time factor is about 15–65%, so missions take longer on the wall clock.

### 2026-10-12: Documentation

- Set up the repository structure and wrote the setup, concept, Stage 0 and Stage 1 pages.
- Made the helper scripts path-independent, so they run from the repo wherever it lives.
- Repo will be **private** until Stage 2 works, then public.

### 2026-10-12: Log analysis, run 4, screenshots

- Uploaded the run 3 log to PX4 Flight Review: <https://review.px4.io/plot_app?log=d08abd44-283e-447d-aed5-68fbf323ed1e>
- **Found:** PX4 SITL logs Gazebo ground truth (`vehicle_local_position_groundtruth`, `vehicle_attitude_groundtruth`). Wrote `flight_report.py` to compare truth with the EKF2 estimate.
- **Correction:** the loops in the Stage 1 track are **real motion**, not estimator glitches. After the wall hit, the drone tumbled across the room for about 9 s (tilt up to 156°, real speed up to 17 m/s). The estimate stayed within 2.6 m thanks to GPS. The only clear estimator error was altitude (estimate went 2 m below the floor).
- Verified: motors saturated (0 % / 100 %) at both impacts. EKF test ratios stayed below 0.06, so GPS was never rejected.
- **Run 4** (debris block now 3.5 m): 3 collisions as designed (pillar, debris, wall). The wall impact led to 31 s of tumbling, against 9 s in run 3, so collision outcomes are chaotic.
- Took simulation screenshots (Gazebo chase camera at offset −7, −7, 6 m via `PX4_GZ_FOLLOW_OFFSET_*`, plus the QGroundControl plan and flown track) → `media/stage1/`.
- Copied the Stage 0 mission file into the repo (`sim/missions/stage0_baseline_mission.plan`).
- Decided to publish through the GitHub website (upload in the browser) instead of GitHub Desktop.
