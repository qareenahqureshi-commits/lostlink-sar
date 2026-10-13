#!/usr/bin/env python3
"""
Stage 4 report: GPS + link jammed at the door, drifting VIO, drift-aware map + relocalisation + return decision.

Usage:
    python3 sim/scripts/stage4_report.py --log _archive/stage4_<run>.ulg --run-dir results/stage4/<run>
Writes into --run-dir: stage4_paths.png, stage4_error.png, stage4_metrics.json
"""
import argparse, csv, json, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pyulog import ULog

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stage2_report import world, draw_world, clearance

TRUE, RAW, CORR, INK, MUTED, MAP, ANCH, LOST = "#2a78d6", "#eb6834", "#7a3fb0", "#2b2b2b", "#8a8a85", "#1baf7a", "#c9c6bd", "#b00000"


def topic(u, n):
    return [d for d in u.data_list if d.name == n][0].data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True)
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--budget", type=float, default=2.0)
    ap.add_argument("--world-spec", default=None, help="Stage 5: generated building spec (.json)")
    a = ap.parse_args()
    if a.world_spec:
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "worlds"))
        import make_building
        w = make_building.load(a.world_spec)
        (X0, X1), (Y0, Y1) = w.spec["bounds_en"]
    else:
        w = world()
        (X0, X1), (Y0, Y1) = (-4, 48), (-12, 12.5)
    u = ULog(a.log, ["vehicle_local_position", "vehicle_local_position_groundtruth", "sensor_combined"])
    lp, gt, sc = topic(u, "vehicle_local_position"), topic(u, "vehicle_local_position_groundtruth"), topic(u, "sensor_combined")
    msgs = [(m.timestamp / 1e6, m.message) for m in ULog(a.log, []).logged_messages]
    t0 = next((t for t, m in msgs if "Takeoff detected" in m), lp["timestamp"][0] / 1e6)
    tl, tg = lp["timestamp"] / 1e6 - t0, gt["timestamp"] / 1e6 - t0
    meta = json.load(open(os.path.join(a.run_dir, "planner_meta.json")))
    rows = list(csv.DictReader(open(os.path.join(a.run_dir, "planner_log.csv"))))
    rows = [r for r in rows if r["state"] in ("MISSION", "LAND") and r.get("corr_n", "") != ""]
    cn = np.array([float(r["n"]) + float(r["corr_n"]) for r in rows]); ce = np.array([float(r["e"]) + float(r["corr_e"]) for r in rows])
    link = np.array([r["link_ok"] == "1" for r in rows]); ret = np.array([r["s4_state"] in ("return", "done") for r in rows])

    gn, ge = np.interp(tl, tg, gt["x"]), np.interp(tl, tg, gt["y"])
    err_raw = np.hypot(lp["x"] - gn, lp["y"] - ge)
    fly = -lp["z"] > 0.5
    t_land = tl[fly][-1] if fly.any() else tl[-1]
    ts = sc["timestamp"] / 1e6 - t0
    amag = np.linalg.norm(np.vstack([sc[f"accelerometer_m_s2[{i}]"] for i in range(3)]), axis=0) / 9.81
    hits = ts[(amag > 20) & (ts > 1) & (ts < t_land)]
    n_imp = int(len(hits) > 0) + int(np.sum(np.diff(hits) > 5)) if len(hits) else 0
    galt = -gt["z"]; low = (galt > 1.0) & (galt < 4.0) & (tg > 0)
    cl = [clearance(e, n, al, w)[0] for e, n, al in zip(gt["y"][low], gt["x"][low], galt[low])]
    land_true = (float(gt["x"][-1]), float(gt["y"][-1]))
    corr_final = np.array(meta.get("final_corrected_pos_ne", [np.nan, np.nan]))
    m = {
        "policy": meta.get("policy"), "outcome": meta.get("outcome"), "waypoints_reached": meta.get("waypoints_reached"),
        "return_reason": meta.get("return_reason"), "relocalisations": meta.get("relocalisations"),
        "r_hat_prior": meta.get("r_hat_prior"), "r_hat_final": meta.get("r_hat_final"),
        "vio_drift_final_m": meta.get("final_vio_drift_m"),
        "px4_estimate_error_final_m": round(float(err_raw[fly][-1]), 2) if fly.any() else None,
        "corrected_estimate_error_final_m": round(float(np.hypot(*(corr_final - np.array(land_true)))), 2),
        "true_landing_ne_m": [round(v, 2) for v in land_true],
        "true_distance_from_home_m": round(float(np.hypot(*land_true)), 2),
        "returned_within_budget": bool(meta.get("outcome") == "completed" and np.hypot(*land_true) <= a.budget),
        "impacts": n_imp, "min_clearance_m": round(float(min(cl)), 2) if cl else None,
        "s_denied_m": meta.get("s_denied_m"), "skipped_wps": meta.get("skipped_wps"), "events": meta.get("events"),
    }
    if a.world_spec:                       # honest coverage from the TRUE path: rooms really entered
        sp = w.spec; WP = np.array(sp["WAYPOINTS"])
        rooms = WP[sp["room_waypoints"]]                       # (east, north)
        gpath = np.stack([gt["y"], gt["x"]], 1)[(galt > 1.0)]
        seen = [bool(np.min(np.hypot(*(gpath - c).T)) < 1.5) for c in rooms] if len(gpath) else [False] * len(rooms)
        m["rooms_searched"], m["rooms_total"] = int(sum(seen)), len(rooms)
        m["survivor_found"] = bool(seen[sp["room_waypoints"].index(sp["survivor_wp"])])
    json.dump(m, open(os.path.join(a.run_dir, "stage4_metrics.json"), "w"), indent=2)
    print(json.dumps({k: v for k, v in m.items() if k != "events"}, indent=2))

    # ---- figure 1: paths
    fig, ax = plt.subplots(figsize=(11, max(4.5, 11 * (Y1 - Y0) / (X1 - X0) + 1.4)), dpi=150)
    draw_world(ax, w)
    res, e0, n0 = meta["res"], meta["e0"], meta["n0"]
    for fn, col, lab, sz in (("anchor_hits.npy", ANCH, "anchor map (seen while position was trusted)", 1.0),
                             ("occupied_final.npy", MAP, "map in use at landing (after corrections)", 1.3)):
        pth = os.path.join(a.run_dir, fn)
        if os.path.exists(pth):
            G = np.load(pth); ii, jj = np.nonzero(G >= 2 if G.dtype != bool else G)
            ax.scatter(e0 + (jj + .5) * res, n0 + (ii + .5) * res, s=sz, c=col, alpha=.6, label=lab)
    gf = (galt > 0.5) & (tg > 0)
    ax.plot(gt["y"][gf], gt["x"][gf], color=TRUE, lw=1.8, label="real path (Gazebo truth)")
    ax.plot(lp["y"][fly], lp["x"][fly], color=RAW, lw=0.9, ls=":", label="PX4 estimate (GPS, then drifting VIO)")
    ax.plot(ce, cn, color=CORR, lw=1.1, ls="--", label="drone's belief after its own corrections")
    if (~link).any():
        ax.plot(ce[~link], cn[~link], ".", color=LOST, ms=1.2, label="radio link lost")
    if ret.any():
        k = int(np.argmax(ret)); ax.plot(ce[k], cn[k], "o", mfc="none", mec=INK, ms=11)
        ax.annotate(f"turns back\n({meta.get('return_reason')})", (ce[k], cn[k]), xytext=(-14, -30), textcoords="offset points", fontsize=8)
    ax.axvline(14, color=MUTED, ls=":", lw=0.9); ax.text(14.2, 10.7, "GPS + link jammed", fontsize=7.5, color=MUTED)
    ax.plot(0, 0, "^", color=INK, ms=9)
    ax.plot(land_true[1], land_true[0], "X", color=TRUE, ms=10)
    ax.annotate(f"landed {m['true_distance_from_home_m']} m from home", (land_true[1], land_true[0]),
                xytext=(10, -22), textcoords="offset points", fontsize=8, arrowprops=dict(arrowstyle="->", lw=0.6))
    ax.set_aspect("equal"); ax.set_xlim(X0, X1); ax.set_ylim(Y0, Y1); ax.grid(alpha=.2)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    n_wp = len(w.spec["WAYPOINTS"]) if a.world_spec else 5
    ax.set_title(f"{'Stage 5' if a.world_spec else 'Stage 4'} [{meta.get('policy')}]: {meta.get('final_vio_drift_m')} m VIO drift, "
                 f"{meta.get('relocalisations')} relocalisations, waypoints {meta.get('waypoints_reached')}/{n_wp}, {n_imp} collisions", fontsize=10)
    ax.legend(fontsize=7, loc="lower right", frameon=False)
    fig.tight_layout(); fig.savefig(os.path.join(a.run_dir, "stage4_paths.png"))

    # ---- figure 2: errors over time
    fig2, ax2 = plt.subplots(figsize=(10, 3.2), dpi=150)
    ax2.plot(tl[fly], err_raw[fly], color=RAW, lw=1.2, label="PX4 estimate vs truth (follows the drifting VIO)")
    ax2.set_title("Position error: what PX4 believes vs what the drone believes after relocalising", fontsize=10)
    # The planner log is stamped in wall-clock time, the PX4 log in simulation time (lockstep, RTF < 1).
    # Align them by position: each planner row recorded PX4's estimate (n, e) -> find that moment in the
    # PX4 log (monotone nearest-neighbour search), then compare the corrected belief with the truth then.
    rn = np.array([float(r["n"]) for r in rows]); re_ = np.array([float(r["e"]) for r in rows])
    j, tm = int(np.argmax(tl > 0)), []
    for n_, e_ in zip(rn, re_):
        w = slice(j, min(j + 3000, len(tl)))
        dd = np.hypot(lp["x"][w] - n_, lp["y"][w] - e_)
        k = j + int(np.argmin(dd))
        tm.append((tl[k], float(dd.min()))); j = k
    tm, resid = np.array(tm).T
    good = resid < 0.15                         # drop rows that could not be matched cleanly (e.g. hovering + turning)
    tm, cn, ce, ret = tm[good], cn[good], ce[good], ret[good]
    gn2, ge2 = np.interp(tm, tg, gt["x"]), np.interp(tm, tg, gt["y"])
    err_c = np.hypot(cn - gn2, ce - ge2)
    ax2.plot(tm, err_c, color=CORR, lw=1.2, label="drone's corrected belief vs truth (Stage 4)")
    if ret.any():
        ax2.axvline(tm[int(np.argmax(ret))], color=INK, ls="--", lw=0.8); ax2.text(tm[int(np.argmax(ret))] + 0.5, ax2.get_ylim()[1] * 0.85, "turns back", fontsize=8)
    m["corrected_error_max_m"] = round(float(err_c.max()), 2)
    json.dump(m, open(os.path.join(a.run_dir, "stage4_metrics.json"), "w"), indent=2)
    ax2.set_xlabel("Time since takeoff (s, simulation clock)"); ax2.set_ylabel("Position error (m)"); ax2.grid(alpha=.25)
    ax2.legend(fontsize=7.5, frameon=False)
    for s in ("top", "right"): ax2.spines[s].set_visible(False)
    fig2.tight_layout(); fig2.savefig(os.path.join(a.run_dir, "stage4_error.png"))


if __name__ == "__main__":
    main()
