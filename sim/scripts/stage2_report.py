#!/usr/bin/env python3
"""
Stage 2 report: before (Stage 1, plain mission) vs after (Stage 2, avoidance) on the real geometry.

Uses Gazebo ground truth from both PX4 logs, the planner's own log/map, and the exact obstacle
geometry from sim/worlds/make_stage1.py to measure true clearance to every obstacle.

Usage:
    python3 sim/scripts/stage2_report.py --before _archive/stage1_run4_flight.ulg \
        --after _archive/stage2_<run>.ulg --run-dir results/stage2/<run>
Writes into --run-dir: before_after.png, clearance.png, stage2_metrics.json
"""
import argparse, importlib.util, json, math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.transforms import Affine2D
from pyulog import ULog

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BEFORE, AFTER, INK, MUTED, OBST, MAP = "#eb6834", "#2a78d6", "#2b2b2b", "#8a8a85", "#cfcdc6", "#1baf7a"


def world():
    s = importlib.util.spec_from_file_location("w", os.path.join(REPO, "sim", "worlds", "make_stage1.py"))
    w = importlib.util.module_from_spec(s); s.loader.exec_module(w); return w


def truth_track(path):
    u = ULog(path, ["vehicle_local_position_groundtruth"])
    g = u.data_list[0].data
    t = g["timestamp"] / 1e6
    msgs = [(m.timestamp / 1e6, m.message) for m in ULog(path, []).logged_messages]
    return t, g["y"], g["x"], -g["z"], msgs           # east, north, alt


def clearance(e, n, alt, w, radius=0.0):
    """Horizontal distance from point to each colliding obstacle taller than alt. Returns (min_d, name)."""
    best, who = 1e9, None
    for name, c, (sx, sy, sz), yaw, col, coll in w.BOXES:
        if not coll or alt > sz:
            continue
        a = math.radians(yaw)
        dx, dy = e - c[0], n - c[1]
        lx = dx * math.cos(a) + dy * math.sin(a); ly = -dx * math.sin(a) + dy * math.cos(a)
        ox, oy = max(abs(lx) - sx / 2, 0), max(abs(ly) - sy / 2, 0)
        d = math.hypot(ox, oy)
        if d < best:
            best, who = d, name
    for name, c, r, h, col in w.CYLINDERS:
        if alt > h:
            continue
        d = max(math.hypot(e - c[0], n - c[1]) - r, 0)
        if d < best:
            best, who = d, name
    return best, who


def draw_world(ax, w):
    for n_, c, (sx, sy, sz), yaw, col, coll in w.BOXES:
        r = Rectangle((c[0] - sx / 2, c[1] - sy / 2), sx, sy, color=OBST if coll else "#e8a0a0",
                      alpha=1 if coll else 0.8, lw=0)
        r.set_transform(Affine2D().rotate_deg_around(c[0], c[1], yaw) + ax.transData); ax.add_patch(r)
    for n_, c, r, h, col in w.CYLINDERS:
        ax.add_patch(Circle(c, r, color=OBST, ec=MUTED, lw=0.5))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--before", required=True)
    ap.add_argument("--after", required=True)
    ap.add_argument("--run-dir", required=True)
    a = ap.parse_args()
    w = world()
    out = {}

    fig, axs = plt.subplots(2, 1, figsize=(10, 11), dpi=150)
    for ax, path, colr, title in ((axs[0], a.before, BEFORE, "Before (Stage 1): plain QGroundControl mission"),
                                  (axs[1], a.after, AFTER, "After (Stage 2): depth camera + EGO-style planner")):
        t, e, n, alt, msgs = truth_track(path)
        fly = alt > 0.5
        draw_world(ax, w)
        if ax is axs[1]:
            hits_f = os.path.join(a.run_dir, "occupancy_hits.npy")
            meta_f = os.path.join(a.run_dir, "planner_meta.json")
            if os.path.exists(hits_f):
                hits = np.load(hits_f); meta = json.load(open(meta_f))
                ii, jj = np.nonzero(hits >= 2)
                ax.scatter(meta["e0"] + (jj + .5) * meta["res"], meta["n0"] + (ii + .5) * meta["res"],
                           s=1.5, c=MAP, alpha=0.6, label="obstacles mapped by the drone")
        ax.plot(e[fly], n[fly], color=colr, lw=1.8, label="real path (Gazebo truth)")
        xs, ys = zip(*([(0, 0)] + w.WAYPOINTS))
        ax.plot(xs, ys, "--o", color=MUTED, lw=1, ms=3.5, label="mission waypoints")
        # clearance along the low-altitude part
        low = fly & (alt > 1.0) & (alt < 4.0)
        cl = [clearance(ei, ni, ai, w) for ei, ni, ai in zip(e[low], n[low], alt[low])]
        dmin = min(cl, key=lambda x: x[0]) if cl else (None, None)
        key = "before" if ax is axs[0] else "after"
        out[key] = {"log": os.path.basename(path), "min_clearance_m": round(dmin[0], 2) if cl else None,
                    "closest_obstacle": dmin[1]}
        if ax is axs[1]:
            per = {}
            for (d, nm) in cl:
                per[nm] = min(per.get(nm, 9e9), d)
            out["after"]["min_clearance_per_obstacle_m"] = {k: round(v, 2) for k, v in sorted(per.items())}
            out["after"]["clearance_series"] = None
        ax.set_aspect("equal"); ax.set_xlim(-3, 48); ax.set_ylim(-11.5, 11.5); ax.grid(alpha=.25)
        ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)"); ax.set_title(title, fontsize=11)
        ax.legend(fontsize=7.5, loc="upper left", frameon=False)
    fig.tight_layout(); fig.savefig(os.path.join(a.run_dir, "before_after.png"))

    # clearance over time for the avoidance run
    t, e, n, alt, msgs = truth_track(a.after)
    t0 = next((tt for tt, m in msgs if "Takeoff detected" in m), t[0])
    low = (alt > 1.0) & (alt < 4.0)
    d = np.array([clearance(ei, ni, ai, w)[0] for ei, ni, ai in zip(e, n, alt)])
    d[~low] = np.nan
    fig2, ax2 = plt.subplots(figsize=(10, 3.2), dpi=150)
    ax2.plot(t - t0, d, color=AFTER, lw=1.4, label="distance to nearest obstacle (truth)")
    ax2.axhline(0.8, color=MUTED, ls="--", lw=0.9); ax2.text(1, 0.85, "planner inflation radius 0.8 m", fontsize=7.5, color=MUTED)
    ax2.axhline(0.35, color=INK, ls=":", lw=0.9); ax2.text(1, 0.4, "drone half-width ≈ 0.35 m (contact)", fontsize=7.5, color=INK)
    ax2.set_ylim(0, 4); ax2.set_xlabel("Time since takeoff (s, simulation clock)"); ax2.set_ylabel("Clearance (m)")
    ax2.set_title("Stage 2: how close the drone came to any obstacle (centre-to-surface, at 1–4 m altitude)", fontsize=10)
    ax2.grid(alpha=.25)
    for s in ("top", "right"): ax2.spines[s].set_visible(False)
    fig2.tight_layout(); fig2.savefig(os.path.join(a.run_dir, "clearance.png"))
    out["after"]["min_clearance_m"] = round(float(np.nanmin(d)), 2)
    out["after"].pop("clearance_series", None)

    json.dump(out, open(os.path.join(a.run_dir, "stage2_metrics.json"), "w"), indent=2)
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
