#!/usr/bin/env python3
"""
Stage 3 report: GPS-denied flight on drifting VIO. Ground truth vs what PX4 believed.

Usage:
    python3 sim/scripts/stage3_report.py --log _archive/stage3_<run>.ulg --run-dir results/stage3/<run>
Writes into --run-dir: stage3_paths.png, stage3_error.png, stage3_metrics.json
"""
import argparse, json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pyulog import ULog

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stage2_report import world, draw_world, clearance          # same geometry + clearance helpers

TRUE, EST, INK, MUTED, MAP = "#2a78d6", "#eb6834", "#2b2b2b", "#8a8a85", "#1baf7a"


def topic(u, n):
    return [d for d in u.data_list if d.name == n][0].data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--run-dir", required=True)
    a = ap.parse_args()
    w = world()
    u = ULog(a.log, ["vehicle_local_position", "vehicle_local_position_groundtruth", "sensor_combined",
                     "estimator_status_flags"])
    lp, gt = topic(u, "vehicle_local_position"), topic(u, "vehicle_local_position_groundtruth")
    sc = topic(u, "sensor_combined")
    msgs = [(m.timestamp / 1e6, m.message) for m in ULog(a.log, []).logged_messages]
    t0 = next((t for t, m in msgs if "Takeoff detected" in m), lp["timestamp"][0] / 1e6)
    tl, tg = lp["timestamp"] / 1e6 - t0, gt["timestamp"] / 1e6 - t0
    try:
        fl = topic(u, "estimator_status_flags"); tf = fl["timestamp"] / 1e6 - t0
        gps_on = fl["cs_gnss_pos"].astype(bool)
        t_jam = float(tf[np.argmax(~gps_on & (tf > 0))]) if (~gps_on & (tf > 0)).any() else None
    except Exception:
        t_jam = None
    meta = json.load(open(os.path.join(a.run_dir, "planner_meta.json")))

    # error between estimate and truth
    gn, ge = np.interp(tl, tg, gt["x"]), np.interp(tl, tg, gt["y"])
    err = np.hypot(lp["x"] - gn, lp["y"] - ge)
    fly = -lp["z"] > 0.5
    t_land = tl[fly][-1] if fly.any() else tl[-1]
    # impacts
    ts = sc["timestamp"] / 1e6 - t0
    amag = np.linalg.norm(np.vstack([sc[f"accelerometer_m_s2[{i}]"] for i in range(3)]), axis=0) / 9.81
    hits = ts[(amag > 20) & (ts > 1) & (ts < t_land)]
    n_imp = int(len(hits) > 0) + int(np.sum(np.diff(hits) > 5)) if len(hits) else 0
    # clearance on truth
    galt = -gt["z"]; low = (galt > 1.0) & (galt < 4.0) & (tg > 0)
    cl = [clearance(e, n, al, w)[0] for e, n, al in zip(gt["y"][low], gt["x"][low], galt[low])]
    # true landing point vs home
    land_true = (float(gt["x"][-1]), float(gt["y"][-1]))
    m = {
        "outcome": meta.get("outcome"), "waypoints_reached": meta.get("waypoints_reached"),
        "gps_jam_time_s": round(t_jam, 1) if t_jam else None,
        "vio_drift_final_m": meta.get("final_vio_drift_m"),
        "estimate_error_final_m": round(float(err[fly][-1]), 2) if fly.any() else None,
        "estimate_error_max_m": round(float(err[fly].max()), 2) if fly.any() else None,
        "true_landing_ne_m": [round(v, 2) for v in land_true],
        "true_distance_from_home_m": round(float(np.hypot(*land_true)), 2),
        "believed_landing_ne_m": meta.get("final_est_pos_ne"),
        "impacts": n_imp, "min_clearance_m": round(float(min(cl)), 2) if cl else None,
    }
    json.dump(m, open(os.path.join(a.run_dir, "stage3_metrics.json"), "w"), indent=2)
    print(json.dumps(m, indent=2))

    # ---- figure 1: paths
    fig, ax = plt.subplots(figsize=(10, 5.6), dpi=150)
    draw_world(ax, w)
    hp = os.path.join(a.run_dir, "occupancy_hits.npy")
    if os.path.exists(hp):
        H = np.load(hp); ii, jj = np.nonzero(H >= 2)
        ax.scatter(meta["e0"] + (jj + .5) * meta["res"], meta["n0"] + (ii + .5) * meta["res"], s=1.3, c=MAP,
                   alpha=0.55, label="map built by the drone (in its own, drifting frame)")
    gf = (galt > 0.5) & (tg > 0)
    ax.plot(gt["y"][gf], gt["x"][gf], color=TRUE, lw=1.8, label="real path (Gazebo truth)")
    ax.plot(lp["y"][fly], lp["x"][fly], color=EST, lw=1.1, ls="--", label="what PX4 believed (GPS + VIO)")
    ax.axvline(14, color=MUTED, ls=":", lw=0.9); ax.text(14.2, 10.7, "GPS jammed", fontsize=7.5, color=MUTED)
    ax.plot(0, 0, "^", color=INK, ms=9); ax.annotate("home", (0, 0), xytext=(4, -12), textcoords="offset points", fontsize=8)
    ax.plot(land_true[1], land_true[0], "X", color=TRUE, ms=10)
    ax.annotate(f"really landed here\n{m['true_distance_from_home_m']} m from home", (land_true[1], land_true[0]),
                xytext=(10, 18), textcoords="offset points", fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.6))
    ax.set_aspect("equal"); ax.set_xlim(-4, 48); ax.set_ylim(-12, 12.5); ax.grid(alpha=.2)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title(f"Stage 3: GPS jammed at the door, {meta.get('final_vio_drift_m')} m VIO drift by landing — "
                 f"{m['impacts']} collisions", fontsize=10)
    ax.legend(fontsize=7.5, loc="lower right", frameon=False)
    fig.tight_layout(); fig.savefig(os.path.join(a.run_dir, "stage3_paths.png"))

    # ---- figure 2: estimate error over time
    fig2, ax2 = plt.subplots(figsize=(10, 3.2), dpi=150)
    ax2.plot(tl[fly], err[fly], color=EST, lw=1.4, label="|PX4 estimate − truth| (horizontal)")
    if t_jam:
        ax2.axvline(t_jam, color=INK, ls="--", lw=0.9); ax2.text(t_jam + 0.5, ax2.get_ylim()[1] * 0.9 if ax2.get_ylim()[1] > 0 else 1, "GPS jammed", fontsize=8)
    ax2.set_xlabel("Time since takeoff (s, PX4 clock)"); ax2.set_ylabel("Position error (m)")
    ax2.set_title("How far the drone's belief drifted from reality", fontsize=10); ax2.grid(alpha=.25)
    for s in ("top", "right"): ax2.spines[s].set_visible(False)
    fig2.tight_layout(); fig2.savefig(os.path.join(a.run_dir, "stage3_error.png"))


if __name__ == "__main__":
    main()
