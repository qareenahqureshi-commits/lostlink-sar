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

### 2026-10-12: Stage 2 — obstacle avoidance

- **2.1 Environment check:** depth camera `/depth_camera` (640×480 float, 73° FOV, 0.2–19.1 m, 30 Hz); PX4 v1.18 uses versioned ROS2 topics (`vehicle_status_v4`); `gz.transport13` Python available → read depth directly, no bridge. First check script hung: PX4 started via `make` in the background grabbed the terminal (pxh shell). **Fix:** run the PX4 binary in daemon mode (`bin/px4 -d`) under `setsid` with stdin from `/dev/null`.
- **Decision:** build `px4_msgs` from the local PX4 source (`setup_stage2.sh`) so message definitions always match the firmware.
- **Decision:** core algorithm in a ROS-free module (`core.py`) + offline bench (`sim/offline/offline_sim.py`) with ray-cast depth of the same building. Offline: no-avoid → 4 hits (pillar at 18.7 s vs 19.1 s real); avoid → 0 hits, robust to 5 % depth noise and to the depth-convention question.
- **Problem:** NumPy 2.2 (user pip) vs system SciPy/matplotlib built for NumPy 1.x → node crashed on first map update. **Fix:** pure-NumPy distance field (matches SciPy to 5e-8 m, 15 ms). Flight reports run outside WSL for now.
- **Result (run_1012_1347_avoid):** full mission + return + landing in OFFBOARD, **0 impacts**, min clearance 0.90 m (doorway), 1.2–1.4 m to pillar/debris/partition, max tilt 13.9°, max estimate error 0.27 m, 47 replans, 1,783 depth frames. Real path ≈ offline prediction.
- **Noted for Stage 3:** map edges ~0.25–0.75 m thicker than truth (likely depth/pose timing) — fix with timestamp matching before GPS denial.

### 2026-10-12: Stage 3 — GPS denial on drifting VIO

- **Design:** GPS jammed by PX4 failure injection at East 14 m; drone continues on simulated VIO (Gazebo truth + seeded drift, % of distance flown) fed to EKF2 as external vision. Height switched to baro, datalink-loss RTL disabled.
- **Env check:** truth pose on `/world/sar_building/dynamic_pose/info`; failure injection works; PX4 = wall-clock µs, Gazebo images = sim time (RTF ~0.3) → explains Stage 2 smear.
- **Offline sweep (12 runs):** 2 % → 3/3 home (0.8–1.8 m off); 5 % → 2/3 trapped inside; 10 % → 0/3 within 2 m; 20 % → lost. **0 collisions at every drift level.** Drift breaks getting home, not avoidance.
- **Bugs:** float param type (sim_vio crash); stale Gazebo blocking restart (force-kill + verify); time-sync v1 used poses up to 15 s old → fake walls in doorway → drone flew around the building outside (aborted). v2: least-squares RTF + 1 s plausibility gate; only ~10 % frames matched, rest fallback.
- **Real run 1012_1449 (2 %, seed 0):** completed, 0 impacts, min clearance 0.80 m, VIO drift 1.57 m, estimate error 1.48 m, **true landing 1.52 m from home** while the drone believed it was home. Matches offline.
- **Real run 1012_1455 (2 %, repeat):** completed, 0 impacts, true landing **1.37 m** from home (vs 1.52 m) → repeatable.
- **Real run 1012_1503 (5 %, seed 0):** all 5 waypoints, then `no_path` on the way home: west wall doubled in the drifting map, doorway filled → **trapped**, hovered 15 min until the timeout, landed 20.8 m from home. Matches offline. New: estimate error 2.7 m meant the drone really hovered **0.39 m** from a pillar while believing it had ≥ 1.2 m → drift shrinks real safety margin silently.
- **Stage 3 closed (first pass).** Next: Stage 4 return decision (trust in own position + doorway still in map → turn back early).

### 2026-10-12: Stage 4 — GPS + link jammed, return decision

- **Design:** drift-aware map (cells forgotten after W = e_tol/r̂ metres of GPS-free flight), anchor map + 2D correlative relocalisation, breadcrumb graph home, return decision predicting σ at the exit vs capture radius (3 m), landing budget (2 m), battery. Link model: wall count + path loss; jammer also takes the link at the door.
- **Offline debugging:** breadcrumbs alone pointed ~4 m off the real door at 5 % drift → added relocalisation. Route retraced the whole loop and stalled → plan straight to exit, breadcrumbs only when stuck. Phantom wall in the doorway from shifted stale cells → drop stale cells on shift. Decision never fired at 20 % → learn drift rate from relocalisation shifts. Landing error from the last 15 m outside → fly backwards facing the facade.
- **Monte Carlo (240 runs, 10 seeds):** safe return within 2 m — Stage 3: 40/0/0/0 % at 2/5/10/20 % drift; Stage 4: 100/100/100/40 %. Map + relocalisation without the decision: 100/100/100/10 % with full coverage → the decision only pays off at 20 % and costs ~0.4–1 waypoint below that. Optimistic 2 % prior → door-frame collisions at ≥10 %.
- **Real flights:** Stage 4 at 5 % (2 seeds) and 10 % landed 0.35 / 0.52 / 0.89 m from home, 0 collisions, while PX4's estimate was 2.9 / 2.9 / 6.0 m off. Link-loss rule: 2/5 waypoints, 0.74 m. All Stage 4 real flights turned back after waypoint 3.
- **Limitation found:** drift-rate estimate biased low with frequent small corrections (real 10 % run learned 5–6 %). Next: long-baseline estimator, deeper building.
- The Monte Carlo sweep is resumable: finished runs are read back from `runs.jsonl` and skipped.

### 2026-10-13: Stage 5 — random deep buildings

- **Generator:** `make_building.py` (seed → corridor + 8–12 rooms, doors, debris, survivor, room-search mission; same building for bench, Gazebo, QGC and radio model).
- **Algorithm changes:** long-baseline drift-rate estimator (reads ~10.5 % at 10 %, ~2.5 % at 2 %); recognition alarm (5 bad matches with the anchor in view → assume 15 %; ambiguous corridor fits don't count); skip stuck waypoints after 15 s; adaptive look-ahead near walls (fixed door-frame scrapes, which had been 40 % of runs at 5 % in an early version).
- **Real simulator problems:** waypoint-skip timer started at node launch (fixed); corridor walls mapped up to 0.8 m too far in (timing smear while turning) → no way in → planner routed outside and **clipped an unseen corner, drone flipped** (run discarded). Fixes: motion-gated mapping (skip frames while yaw rate > 0.25 rad/s), corridors 4.0 m / doors 3.0 m. PX4 land detector hung on the ground with baro height → forced disarm after 60 s (sim only).
- **Bench speed-up:** skip out-of-range obstacles in ray casting + SciPy distance transform (verified identical) → 5.5× faster.
- **Monte Carlo (400 runs):** deep buildings, safe return at 2/5/10/20 % — Stage 5: 100/100/50/40 % (0 collisions); without the decision: 100/30/0/0 % (up to 40 % collisions); Stage 3: 0 %; link-loss rule: 100/90/0/0 % but ~0.5 rooms searched. Stage 1 building with final code: 100/100/100/20 %, more waypoints than Stage 4 v1.
- **Real flights (building 0):**
  - Stage 5 at 5 %: 1 room, landed 0.64 m from home, 0 collisions.
  - No return decision at 5 %: 7 rooms, 276 m without GPS, 13 m drift, 1 collision, landed 25 m from home (timeout).
  - Stage 5 (final code) at 10 %: alarm → turned back after 2 rooms; scraped a wall at the start of the return (5 m error); relocalised 4.9 m (error < 0.5 m); then went into a room and hovered 26 min at a dead end next to its west wall (path ended short of a breadcrumb inside the wall margin, no re-plan, no no-progress rule in the return). Landed 26.6 m from home, own estimate only 1.6 m off.
  - Lesson: the 10 % failure is a planning dead end, not lost localisation.
- **Remaining failures (10–20 %):** trapped deep inside where nothing can be recognised and the drift rate cannot be learned; or turned back in time but landed ~9–10 m off. Next: no-progress watchdog in the return + clearing the old map when stuck.

### 2026-10-13 (evening): Stage 5b — recovery when stuck + correction check

- **Recovery when stuck** (return phase): < 1 m movement in 12 s → re-plan differently (switch direct ↔ breadcrumbs, aim 4 m further along the route); still stuck → clear the drifted map and re-map from 4 headings. Also re-plan when the path ends at the drone but the goal is elsewhere.
- **Real flight 10 % with recovery:** got unstuck, but a wrong-wall match in a doorway (north corridor wall onto the south wall) shifted the belief 2.75 m south; searched the wrong room for 13 min; landed 9.1 m from home, 2 collisions.
- **Correction check:** reject relocalisations larger than 2.5 σ + 0.5 m (counts towards the alarm); accept if the same shift repeats 3 times (within 0.5 m). Without the repeat rule, Stage 1 building at 20 % dropped from 2/10 to 0/10.
- **Real flight 10 %, final code:** 2 rooms, alarm return, landed **0.78 m** from home, 0 collisions, drift rate learned 9.2–10 %, 1 look-alike rejected.
- **Monte Carlo, final code (400 runs, `results/stage5/offline_v2`):** deep buildings Stage 5 safe return 100/100/90/80 % (was 100/100/50/40 %); collisions 0/0/10/0 %. No-decision baseline (also with recovery): 100/20/30/30 %, collisions 20–70 %. Stage 1 building unchanged: 100/100/100/20 %.
- **Remaining:** 3 of 40 Stage 5 deep-building runs at 10–20 % fail (1 trapped, 2 landed 9–13 m off). Next: more real flights with the final code for error bars; then the paper.

### 2026-10-14: more real flights with the final code

- Buildings 1 and 2 at 5 / 10 / 20 % drift (plus building 0 at 10 % from yesterday): **7 of 7 back within 2 m, 0 collisions**.
- Landed 0.35 / 0.92 / 1.97 m (building 1) and 0.25 / 0.87 / 2.00 m (building 2); building 0 at 10 %: 0.78 m.
- Drift rate learned in flight: 5.2 % (true 5), 9.2–10 % (true 10), 17–19 % (true 20).
- 20 % flights land right at the 2 m limit: works, but no margin (matches offline 80 %).
- One 20 % flight (building 1) invalid: the laptop slept mid-flight and the simulation clock jumped; discarded to `_archive/stage5_failed_runs/` and repeated.

