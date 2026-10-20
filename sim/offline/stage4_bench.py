#!/usr/bin/env python3
"""
Stage 4 offline bench: GPS jammed at the door + drifting VIO + radio link model, for several policies.

Same world, camera model and planner code as the ROS2 node (core.py + stage4.py). Kinematic drone.
Ground truth (pt) drives the depth rendering and collisions; the drone only sees its drifted
estimate (pe = pt + drift), exactly like in PX4 + Gazebo with sim_vio.

Usage:
    python3 sim/offline/stage4_bench.py --policy stage4 --drift 5 --seed 0 --plot out.png
    python3 sim/offline/stage4_bench.py --sweep --out results/stage4/offline   # Monte Carlo
"""
import argparse, json, math, os, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "ros2_ws", "src", "lostlink_planner"))
import offline_sim as S                                   # noqa: E402
sys.path.insert(0, os.path.join(REPO, 'sim', 'worlds'))
import make_building as MB                                # noqa: E402
from lostlink_planner import core, stage4, sim_env        # noqa: E402

# Speed-up for the offline bench only: SciPy's exact Euclidean distance transform gives the same
# capped distance field as core.distance_field (checked to 5e-8 m in Stage 2), ~20x faster.
try:
    from scipy.ndimage import distance_transform_edt as _edt

    def _fast_distance_field(occ, res, max_dist=2.0):
        if not occ.any():
            return np.full(occ.shape, max_dist, dtype=np.float32)
        return np.minimum(_edt(~occ) * res, max_dist).astype(np.float32)
    core.distance_field = stage4.distance_field = _fast_distance_field
except ImportError:
    pass

POLICIES = list(stage4.LostLinkMission.POLICIES)


def clearance(world, p, alt=3.0, radius=0.3):
    """True horizontal distance (m) from p (n, e) to the nearest obstacle surface that reaches the
    drone's altitude (low debris the drone flies over does not count)."""
    best = 1e9
    for c, ax1, ax2, h1, h2, H, _ in world.boxes:
        if H < alt - radius:
            continue
        rel = np.array(p[:2]) - c
        dx, dy = max(abs(rel @ ax1) - h1, 0.0), max(abs(rel @ ax2) - h2, 0.0)
        best = min(best, math.hypot(dx, dy))
    for c, r, H, _ in world.cyls:
        if H < alt - radius:
            continue
        best = min(best, float(np.linalg.norm(np.array(p[:2]) - c)) - r)
    return best


def run(policy, drift_pct, seed, r_hat=0.05, budget=2.0, gps_off_east=14.0, tmax=None, keep_traj=False, jam_link=True,
        world=None, rate_estimator="baseline", wall_limit_s=None, capture=3.0):
    """jam_link: the jammer that takes GPS at the door also takes the radio link (Lost-Link scenario)."""
    mk = None
    if world is None:                                    # the Stage 1-4 building
        w = S.load_world()
        link = sim_env.LinkModel(sim_env.load_boxes(os.path.join(REPO, "sim", "worlds", "make_stage1.py")))
    else:                                                # Stage 5: generated building (seed or spec path)
        w = MB.load(world); mk = MB.map_kwargs(w.spec)
        link = sim_env.LinkModel(sim_env.boxes_from_module(w))
        tmax = tmax or (150.0 + 4.0 * float(np.sum(np.linalg.norm(np.diff(np.array([(0, 0)] + list(w.WAYPOINTS) + [(0, 0)]), axis=0), axis=1))))
    world_ = world
    tmax = tmax or 400.0
    world = S.World(w)
    wps = [(n, e) for (e, n) in w.WAYPOINTS]
    rng = np.random.default_rng(seed)
    proj = core.DepthProjector(stride=8)
    m = stage4.LostLinkMission(wps, policy=policy, r_hat=r_hat, budget_m=budget, capture_m=capture, map_kwargs=mk,
                               rate_estimator=rate_estimator)
    pt = np.array([0.0, 0.0, -3.0]); yaw = 0.0; d = np.zeros(2); ang = rng.uniform(0, 2 * np.pi)
    dt, t, gps = 0.1, 0.0, True
    hits, last, min_cl, traj, link_lost_at = [], None, 1e9, [], None
    room_pts = visited = None
    if hasattr(w, "spec"):
        room_pts = np.array([wps[i] for i in w.spec["room_waypoints"]], float)
        visited = np.zeros(len(room_pts), bool)
        sv_idx = w.spec["room_waypoints"].index(w.spec["survivor_wp"])
    import time as _time
    _w0, capped = _time.time(), False
    while t < tmax and not m.done:
        if wall_limit_s and _time.time() - _w0 > wall_limit_s:      # compute-time cap (sweeps only)
            capped = True
            break
        if gps and pt[1] > gps_off_east:
            gps = False
        pe = pt.copy(); pe[:2] += d
        lk = link.ok(pt) and not (jam_link and not gps)
        if not lk and link_lost_at is None:
            link_lost_at = round(t, 1)
        m.update_state(t, pe, not gps, lk, dt)
        img, q = S.render_depth(world, pt, yaw, proj)
        m.insert_points(proj.to_world(img, pe, q))
        n, e, ys, info = m.step(t, yaw, t)
        v = 0.95 * (np.array([n, e]) - pe[:2]); sp = np.linalg.norm(v)
        if sp > 2.0:
            v *= 2.0 / sp
        pt[:2] += v * dt
        yaw += np.clip(core.wrap(ys - yaw), -math.radians(60) * dt, math.radians(60) * dt)
        if not gps:
            step = float(np.linalg.norm(v * dt))
            ang += rng.normal(0, 0.05 * max(step, 1e-3) ** 0.5)
            d += drift_pct / 100.0 * step * np.array([math.cos(ang), math.sin(ang)])
        h = world.collides(pt)
        if h and h != last:
            hits.append(h)
        last = h
        if 1.0 < -pt[2]:
            min_cl = min(min_cl, clearance(world, pt, alt=-pt[2]))
        if room_pts is not None:                         # honest coverage: rooms really entered (truth)
            dd = np.linalg.norm(room_pts - pt[:2], axis=1)
            visited |= dd < 1.5
        if keep_traj:
            traj.append((t, pt[0], pt[1], m.p[0], m.p[1], info.get("mode", ""), m.state, lk, m.sigma_pred))
        t += dt
    res = {
        "policy": policy, "drift_pct": drift_pct, "seed": seed, "r_hat": r_hat, "budget_m": budget, "capture_m": capture,
        "home_by_own_estimate": bool(m.done), "timed_out": not m.done,
        "waypoints_reached": m.k, "n_waypoints": len(wps), "world": world_, "rate_estimator": rate_estimator,
        "r_hat_final": round(m.dec.r_hat, 4), "skipped_wps": len(m.skipped), "stuck_events": getattr(m, "n_stuck", 0), "map_clears": getattr(m, "n_map_clears", 0), "relocs_gated": getattr(m, "n_gated", 0), "compute_capped": capped,
        "rooms_total": int(len(room_pts)) if room_pts is not None else None,
        "rooms_searched": int(visited.sum()) if room_pts is not None else None,
        "survivor_found": bool(visited[sv_idx]) if room_pts is not None else None,
        "return_reason": m.return_reason,
        "true_landing_error_m": round(float(np.linalg.norm(pt[:2])), 2),
        "returned_ok": bool(m.done and np.linalg.norm(pt[:2]) <= budget),
        "collisions": len(hits), "collided_with": sorted(set(hits)),
        "min_clearance_m": round(float(min_cl), 2),
        "final_drift_m": round(float(np.linalg.norm(d)), 2),
        "s_denied_m": round(m.s, 1), "flight_time_s": round(t, 1),
        "relocalisations": m.n_reloc, "est_error_end_m": round(float(np.linalg.norm(m.p - pt[:2])), 2),
        "link_lost_at_s": link_lost_at, "events": m.events, "reloc_log": m.reloc_log,
    }
    if keep_traj:
        res["_traj"] = traj; res["_mission"] = m
    return res


def _job(args):
    policy, dp, seed, r_hat = args
    t0 = time.time()
    r = run(policy, dp, seed, r_hat=r_hat)
    r["wall_s"] = round(time.time() - t0, 1)
    return r


def sweep(out, drifts=(2, 5, 10, 20), seeds=10, policies=POLICIES, r_hats=(0.05,), procs=None):
    from multiprocessing import Pool
    os.makedirs(out, exist_ok=True)
    jobs = [(p, dp, s, rh) for rh in r_hats for p in policies for dp in drifts for s in range(seeds)
            if not (p not in ("stage4", "map", "map_reloc") and rh != r_hats[0])
            and not (p != "stage4" and rh != 0.05)]       # the r_hat variant is only run for stage4
    path = os.path.join(out, "runs.jsonl")
    done = set()
    if os.path.exists(path):
        for line in open(path):
            r = json.loads(line); done.add((r["policy"], r["drift_pct"], r["seed"], r["r_hat"]))
    jobs = [j for j in jobs if j not in done]
    print(f"{len(jobs)} runs to do ({len(done)} already done)", flush=True)
    with Pool(procs or os.cpu_count()) as pool, open(path, "a") as f:
        for r in pool.imap_unordered(_job, jobs):
            f.write(json.dumps(r) + "\n"); f.flush()
            print(r["policy"], r["drift_pct"], r["seed"], r["r_hat"], "wp", r["waypoints_reached"],
                  "ok" if r["returned_ok"] else "FAIL", r["true_landing_error_m"], "hits", r["collisions"],
                  f"{r['wall_s']}s", flush=True)


def plot_run(r, path):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    sys.path.insert(0, os.path.join(REPO, "sim", "scripts"))
    from stage2_report import world as w2, draw_world
    T = r["_traj"]; m = r["_mission"]
    wm = MB.load(r["world"]) if r.get("world") is not None else w2()
    (ex0, ex1), (ny0, ny1) = wm.spec["bounds_en"] if hasattr(wm, "spec") else ((-4, 48), (-12, 12.5))
    fig, ax = plt.subplots(figsize=(11, max(4.0, 11 * (ny1 - ny0) / (ex1 - ex0) + 1.2)), dpi=140)
    draw_world(ax, wm)
    occ = m.grid.occupied; ii, jj = np.nonzero(occ)
    ax.scatter(m.grid.e0 + (jj + .5) * m.grid.res, m.grid.n0 + (ii + .5) * m.grid.res, s=1.2, c="#1baf7a",
               alpha=.6, label="map in use at the end")
    A = np.array([(x[1], x[2], x[3], x[4]) for x in T])
    ax.plot(A[:, 1], A[:, 0], color="#2a78d6", lw=1.7, label="real path")
    ax.plot(A[:, 3], A[:, 2], color="#eb6834", lw=1, ls="--", label="what the drone believed (after its corrections)")
    lost = [i for i, x in enumerate(T) if not x[7]]
    if lost:
        ax.plot(A[lost, 1], A[lost, 0], ".", color="#b00", ms=1.5, label="radio link lost")
    ax.plot(A[-1, 1], A[-1, 0], "X", color="#2a78d6", ms=10)
    ax.set_aspect("equal"); ax.set_xlim(ex0, ex1); ax.set_ylim(ny0, ny1); ax.grid(alpha=.2)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title(f"{'building ' + str(r['world']) + ' | ' if r.get('world') is not None else ''}{r['policy']} | drift {r['drift_pct']}% | wp {r['waypoints_reached']}/{r['n_waypoints']} | "
                 f"landed {r['true_landing_error_m']} m from home | {r['collisions']} collisions", fontsize=9.5)
    ax.legend(fontsize=7.5, loc="lower right", frameon=False)
    fig.tight_layout(); fig.savefig(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="stage4", choices=POLICIES)
    ap.add_argument("--drift", type=float, default=5.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--r-hat", type=float, default=0.05)
    ap.add_argument("--plot", default=None)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--r-hats", default="0.05")
    ap.add_argument("--policies", default=",".join(POLICIES))
    ap.add_argument("--out", default=os.path.join(REPO, "results", "stage4", "offline"))
    a = ap.parse_args()
    if a.sweep:
        sweep(a.out, seeds=a.seeds, r_hats=tuple(float(x) for x in a.r_hats.split(",")),
              policies=a.policies.split(","))
        return
    r = run(a.policy, a.drift, a.seed, r_hat=a.r_hat, keep_traj=bool(a.plot))
    print(json.dumps({k: v for k, v in r.items() if not k.startswith("_")}, indent=1))
    if a.plot:
        plot_run(r, a.plot); print("saved", a.plot)


if __name__ == "__main__":
    main()
