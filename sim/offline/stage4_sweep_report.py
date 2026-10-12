#!/usr/bin/env python3
"""
Summarise the Stage 4 offline Monte Carlo (results/stage4/offline/runs.jsonl).

Writes: summary.md (table), summary.json, policies_vs_drift.png
Usage:  python3 sim/offline/stage4_sweep_report.py [--dir results/stage4/offline]
"""
import argparse, json, os
from collections import defaultdict
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))

# fixed order = fixed colour (validated categorical palette, slots 1-5) + a marker shape per policy
SERIES = [  # key, label, colour, marker, linestyle
    ("stage4|0.05", "Stage 4 (full, r̂ prior 5 %)", "#2a78d6", "o", "-"),
    ("stage3|0.05", "Stage 3 (no drift awareness)", "#eb6834", "s", "-"),
    ("rtl_link|0.05", "Return on link loss (PX4-style)", "#1baf7a", "^", "-"),
    ("map|0.05", "Ablation: drift-aware map only", "#eda100", "D", "-"),
    ("map_reloc|0.05", "Ablation: map + relocalisation", "#e87ba4", "v", "-"),
    ("stage4|0.02", "Stage 4 (r̂ prior 2 %)", "#2a78d6", "o", "--"),
]


def load(d):
    runs = [json.loads(l) for l in open(os.path.join(d, "runs.jsonl"))]
    g = defaultdict(list)
    for r in runs:
        g[(f"{r['policy']}|{r['r_hat']}", r["drift_pct"])].append(r)
    return runs, g


def stats(rs):
    n = len(rs)
    land = np.array([r["true_landing_error_m"] for r in rs])
    return {
        "n": n,
        "returned_ok_pct": round(100 * np.mean([r["returned_ok"] for r in rs]), 0),
        "home_by_estimate_pct": round(100 * np.mean([r["home_by_own_estimate"] for r in rs]), 0),
        "timeouts_pct": round(100 * np.mean([r["timed_out"] for r in rs]), 0),
        "waypoints_mean": round(float(np.mean([r["waypoints_reached"] for r in rs])), 2),
        "landing_err_median_m": round(float(np.median(land)), 2),
        "runs_with_collision_pct": round(100 * np.mean([r["collisions"] > 0 for r in rs]), 0),
        "min_clearance_median_m": round(float(np.median([r["min_clearance_m"] for r in rs])), 2),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(REPO, "results", "stage4", "offline"))
    a = ap.parse_args()
    runs, g = load(a.dir)
    drifts = sorted({r["drift_pct"] for r in runs})
    S = {k: {dp: stats(g[(k, dp)]) for dp in drifts if g.get((k, dp))} for k, *_ in SERIES}
    json.dump({k: {str(dp): v for dp, v in d.items()} for k, d in S.items()},
              open(os.path.join(a.dir, "summary.json"), "w"), indent=1)

    lines = ["| Policy | Drift | Runs | Back within 2 m | Waypoints (of 5) | Median landing error | Runs with a collision | Timed out (trapped) |",
             "|---|---|---|---|---|---|---|---|"]
    for k, lab, *_ in SERIES:
        for dp, s in S[k].items():
            lines.append(f"| {lab} | {dp:g} % | {s['n']} | {s['returned_ok_pct']:.0f} % | {s['waypoints_mean']:.1f} | "
                         f"{s['landing_err_median_m']} m | {s['runs_with_collision_pct']:.0f} % | {s['timeouts_pct']:.0f} % |")
    open(os.path.join(a.dir, "summary.md"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    INK, MUTED, GRID = "#2b2b2b", "#6b6a66", "#e6e5e1"
    panels = [("returned_ok_pct", "Came back within 2 m of home (%)", (0, 105)),
              ("waypoints_mean", "Waypoints reached (of 5)", (0, 5.3)),
              ("runs_with_collision_pct", "Runs with a collision (%)", (0, 105))]
    fig, axs = plt.subplots(1, 3, figsize=(13, 4.2), dpi=150)
    for ax, (key, title, yl) in zip(axs, panels):
        for k, lab, col, mk, ls in SERIES:
            if not S[k]:
                continue
            x = list(S[k].keys()); y = [S[k][d][key] for d in x]
            ax.plot([drifts.index(d) for d in x], y, color=col, lw=2, ls=ls, marker=mk, ms=7, mec="#fcfcfb", mew=1.5, label=lab)
        ax.set_xticks(range(len(drifts))); ax.set_xticklabels([f"{d:g} %" for d in drifts]); ax.set_xlim(-0.25, len(drifts) - 0.75)
        ax.set_ylim(*yl); ax.set_title(title, fontsize=10, color=INK, loc="left")
        ax.set_xlabel("True VIO drift (% of distance flown)", fontsize=9, color=MUTED)
        ax.grid(color=GRID, lw=0.8); ax.tick_params(colors=MUTED, labelsize=8.5)
        for s in ("top", "right"): ax.spines[s].set_visible(False)
        for s in ("left", "bottom"): ax.spines[s].set_color(GRID)
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h, l, loc="lower center", ncol=3, fontsize=8.5, frameon=False, labelcolor=INK, handlelength=3.5)
    n = max(s["n"] for d in S.values() for s in d.values())
    fig.suptitle(f"Stage 4 offline Monte Carlo: GPS + link jammed at the door, {n} seeds per point", fontsize=11, color=INK, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0.12, 1, 0.95))
    out = os.path.join(a.dir, "policies_vs_drift.png"); fig.savefig(out, facecolor="#fcfcfb"); print("saved", out)


if __name__ == "__main__":
    main()
