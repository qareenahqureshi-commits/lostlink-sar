#!/usr/bin/env python3
"""
Flight report from a PX4 SITL log: what the drone *thought* vs what *really* happened.

PX4 SITL logs Gazebo's ground truth (vehicle_local_position_groundtruth,
vehicle_attitude_groundtruth) next to the EKF2 estimate, so we can measure how
wrong the estimate gets after an impact - the core question for Stages 3-4.

Usage:
    python3 sim/scripts/flight_report.py path/to/log.ulg --stage stage1

Writes:
    results/<stage>/flight_report.png      5 stacked time plots with collision markers
    results/<stage>/truth_vs_estimate.png  top view: real path vs estimated path
    results/<stage>/flight_metrics.json    headline numbers
"""
import argparse, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pyulog import ULog

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TRUTH, EST, SETP, INK, MUTED = "#2a78d6", "#eb6834", "#1baf7a", "#2b2b2b", "#8a8a85"


def topic(u, name):
    return [d for d in u.data_list if d.name == name][0].data


def tilt_deg(q0, q1, q2, q3):
    # angle between body z-axis and world vertical
    c = 1 - 2 * (q1 ** 2 + q2 ** 2)
    return np.degrees(np.arccos(np.clip(c, -1, 1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--stage", default="stage1")
    ap.add_argument("--labels", default="", help='comma-separated names for the impacts, e.g. "hit pillar,hit wall"')
    a = ap.parse_args()

    u = ULog(a.log)
    lp, gt = topic(u, "vehicle_local_position"), topic(u, "vehicle_local_position_groundtruth")
    att = topic(u, "vehicle_attitude_groundtruth")
    sp = topic(u, "vehicle_local_position_setpoint")
    sc = topic(u, "sensor_combined")
    msgs = [(m.timestamp / 1e6, m.message.strip()) for m in u.logged_messages]

    t_to = next(t for t, m in msgs if "Takeoff detected" in m)
    T = lambda d: d["timestamp"] / 1e6 - t_to

    # estimate error vs truth (truth resampled onto estimate timestamps)
    te, tg = T(lp), T(gt)
    gx, gy, gz = (np.interp(te, tg, gt[k]) for k in ("x", "y", "z"))
    err_xy = np.hypot(lp["x"] - gx, lp["y"] - gy)

    # impacts: clusters of accelerometer spikes > 20 g (separated by > 5 s).
    # Each impact = first spike of a cluster; the cluster length = time spent bouncing/tumbling.
    ts = T(sc)
    amag = np.linalg.norm(np.vstack([sc[f"accelerometer_m_s2[{i}]"] for i in range(3)]), axis=0) / 9.81
    spikes = ts[(amag > 20) & (ts > 1)]
    impacts, clusters = [], []
    for t in spikes:
        if clusters and t - clusters[-1][1] < 5:
            clusters[-1][1] = t
        else:
            clusters.append([t, t])
    for t0, t1 in clusters:
        sel = (ts >= t0) & (ts <= t1)
        impacts.append((t0, float(amag[sel].max())))

    events = [(t - t_to, lab) for t, m in msgs for key, lab in
              (("Mission finished", "RTL"), ("Landing detected", "landed")) if key in m]

    # ---------- figure 1: stacked time series ----------
    fig, ax = plt.subplots(5, 1, figsize=(10, 11), dpi=150, sharex=True)
    tq = T(att)
    panels = [
        ("Altitude (m)", [(tg, -gt["z"], TRUTH, "truth (Gazebo)"), (te, -lp["z"], EST, "PX4 estimate"),
                          (T(sp), -sp["z"], SETP, "setpoint")]),
        ("Horizontal speed (m/s)", [(tg, np.hypot(gt["vx"], gt["vy"]), TRUTH, "truth"),
                                    (te, np.hypot(lp["vx"], lp["vy"]), EST, "estimate")]),
        ("Tilt (deg)", [(tq, tilt_deg(att["q[0]"], att["q[1]"], att["q[2]"], att["q[3]"]), TRUTH, "truth")]),
        ("Acceleration (g)", [(ts, amag, INK, "IMU |a|")]),
        ("Position error (m)", [(te, err_xy, EST, "|estimate − truth| horizontal")]),
    ]
    for axi, (ylab, series) in zip(ax, panels):
        for x, y, c, lab in series:
            axi.plot(x, y, color=c, lw=1.2, label=lab)
        axi.set_ylabel(ylab, color=INK, fontsize=9)
        axi.grid(alpha=0.25)
        for s in ("top", "right"):
            axi.spines[s].set_visible(False)
        if len(series) > 1:
            axi.legend(fontsize=7, loc="center right", frameon=False)
        for t0, t1 in clusters:
            axi.axvspan(t0, max(t1, t0 + 0.3), color=INK, alpha=0.07, lw=0)
            axi.axvline(t0, color=INK, lw=0.8, ls="--")
        for t, lab in events:
            axi.axvline(t, color=MUTED, lw=0.8, ls=":")
    names = [n.strip() for n in a.labels.split(",") if n.strip()]
    names += [f"impact {i}" for i in range(len(names) + 1, 20)]
    for k, (t, g) in enumerate(impacts):
        ax[0].annotate(names[k], (t, ax[0].get_ylim()[1]), xytext=(3, -10 - 11 * k), textcoords="offset points",
                       fontsize=8, color=INK)
    for t, lab in events:
        ax[0].annotate(lab, (t, ax[0].get_ylim()[1]), xytext=(3, -10), textcoords="offset points",
                       fontsize=8, color=MUTED)
    ax[-1].set_xlim(-5, T(lp)[-1])
    ax[-1].set_xlabel("Time since takeoff (s, simulation clock)")
    fig.suptitle(f"{a.stage}: flight report (ground truth vs PX4 estimate)", fontsize=11)
    fig.tight_layout()
    out_dir = os.path.join(REPO, "results", a.stage)
    os.makedirs(out_dir, exist_ok=True)
    fig.savefig(os.path.join(out_dir, "flight_report.png"))

    # ---------- figure 2: top view truth vs estimate ----------
    t_rtl = next((t for t, l in events if l == "RTL"), te[-1])
    mg, me = (tg > 0) & (tg < t_rtl), (te > 0) & (te < t_rtl)
    fig2, a2 = plt.subplots(figsize=(9, 5.6), dpi=150)
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location("w", os.path.join(REPO, "sim", "worlds", "make_stage1.py"))
        w = importlib.util.module_from_spec(spec); spec.loader.exec_module(w)
        from matplotlib.patches import Rectangle, Circle
        from matplotlib.transforms import Affine2D
        for n, c, (sx, sy, sz), yaw, col, coll in w.BOXES:
            r = Rectangle((c[0] - sx / 2, c[1] - sy / 2), sx, sy, color="#cfcdc6", alpha=1 if coll else 0.4)
            r.set_transform(Affine2D().rotate_deg_around(c[0], c[1], yaw) + a2.transData); a2.add_patch(r)
        for n, c, rad, h, col in w.CYLINDERS:
            a2.add_patch(Circle(c, rad, color="#cfcdc6", ec=MUTED, lw=0.5))
    except Exception as e:
        print("world overlay skipped:", e)
    a2.plot(gt["y"][mg], gt["x"][mg], color=TRUTH, lw=1.8, label="real path (Gazebo truth)")
    a2.plot(lp["y"][me], lp["x"][me], color=EST, lw=1.2, ls="--", label="what PX4 believed (EKF2)")
    for k, (t, g) in enumerate(impacts):
        a2.plot(np.interp(t, tg, gt["y"]), np.interp(t, tg, gt["x"]), "X", color=INK, ms=10)
        a2.annotate(names[k], (np.interp(t, tg, gt["y"]), np.interp(t, tg, gt["x"])), xytext=(6, 6),
                    textcoords="offset points", fontsize=8)
    a2.set_aspect("equal"); a2.grid(alpha=0.25); a2.set_xlabel("East (m)"); a2.set_ylabel("North (m)")
    a2.set_title(f"{a.stage}: real vs estimated path (takeoff to RTL)", fontsize=11)
    a2.legend(fontsize=8, loc="upper left", frameon=False)
    fig2.tight_layout(); fig2.savefig(os.path.join(out_dir, "truth_vs_estimate.png"))

    # ---------- metrics ----------
    tilt = tilt_deg(att["q[0]"], att["q[1]"], att["q[2]"], att["q[3]"])
    m = {
        "takeoff_to_rtl_s": round(float(t_rtl), 1),
        "impacts": [{"t_s": round(float(c[0]), 2), "contact_duration_s": round(float(c[1] - c[0]), 2),
                     "peak_accel_g": round(float(g), 1),
                     "truth_pos_east_north_m": [round(float(np.interp(c[0], tg, gt["y"])), 2),
                                                round(float(np.interp(c[0], tg, gt["x"])), 2)],
                     "min_truth_alt_during_m": round(float((-gt["z"])[(tg >= c[0]) & (tg <= c[1] + 1)].min()), 2)}
                    for c, (t, g) in zip(clusters, impacts)],
        "max_tilt_deg": round(float(tilt[(tq > 0) & (tq < t_rtl)].max()), 1),
        "min_truth_alt_in_mission_m": round(float((-gt["z"])[mg].min()), 2),
        "max_position_error_m": round(float(err_xy[me].max()), 2),
        "max_estimated_hspeed_mps": round(float(np.hypot(lp["vx"], lp["vy"])[me].max()), 2),
        "max_true_hspeed_mps": round(float(np.hypot(gt["vx"], gt["vy"])[mg].max()), 2),
    }
    with open(os.path.join(out_dir, "flight_metrics.json"), "w") as f:
        json.dump(m, f, indent=2)
    print(json.dumps(m, indent=2))


if __name__ == "__main__":
    main()
