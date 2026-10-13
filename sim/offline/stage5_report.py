#!/usr/bin/env python3
"""
Summarise the Stage 5 Monte Carlo (results/stage5/offline/runs.jsonl).

Writes into the same folder:
  summary_stage1.md      final code on the original building (6 policies x 4 drifts x 10 seeds)
  summary_buildings.md   generated buildings (4 policies x 4 drifts x N buildings)
  summary.json
  buildings_vs_drift.png safe return / rooms searched / collisions vs drift, generated buildings
  tradeoff.png           rooms searched vs safe return, every policy and drift level
"""
import argparse, json, os
from collections import defaultdict
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
INK, MUTED, GRID, SURF = "#2b2b2b", "#6b6a66", "#e6e5e1", "#fcfcfb"
# fixed order = fixed colour (validated categorical palette) + marker per policy
SERIES = [("stage4|0.05", "Stage 5 (full)", "#2a78d6", "o", "-"),
          ("stage3|0.05", "Stage 3 (no drift awareness)", "#eb6834", "s", "-"),
          ("rtl_link|0.05", "Return on link loss (PX4-style)", "#1baf7a", "^", "-"),
          ("map|0.05", "Drift-aware map only", "#eda100", "D", "-"),
          ("map_reloc|0.05", "Map + relocalisation, no decision", "#e87ba4", "v", "-"),
          ("stage4|0.02", "Stage 5, optimistic prior (2 %)", "#2a78d6", "o", "--")]


def stats(rs):
    a = lambda k: np.array([r[k] for r in rs], float)
    out = {"n": len(rs),
           "returned_ok_pct": round(100 * a("returned_ok").mean()),
           "timeouts_pct": round(100 * a("timed_out").mean()),
           "collision_runs_pct": round(100 * (a("collisions") > 0).mean()),
           "landing_err_median_m": round(float(np.median(a("true_landing_error_m"))), 2),
           "waypoints_mean": round(float(a("waypoints_reached").mean()), 1)}
    if rs[0].get("rooms_searched") is not None:
        out["rooms_mean"] = round(float(a("rooms_searched").mean()), 1)
        out["rooms_frac_pct"] = round(100 * float((a("rooms_searched") / a("rooms_total")).mean()))
        out["survivor_pct"] = round(100 * a("survivor_found").mean())
        # "useful and safe": searched at least one room AND came back within 2 m
        out["safe_and_useful_pct"] = round(100 * float(((a("rooms_searched") > 0) & (a("returned_ok") > 0)).mean()))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=os.path.join(REPO, "results", "stage5", "offline"))
    a = ap.parse_args()
    runs = [json.loads(l) for l in open(os.path.join(a.dir, "runs.jsonl"))]
    G = defaultdict(list)
    for r in runs:
        grp = "stage1" if r["world"] in (None, "stage1") else "buildings"
        G[(grp, f"{r['policy']}|{r['r_hat']}", r["drift_pct"])].append(r)
    drifts = sorted({r["drift_pct"] for r in runs})
    S = {f"{g}/{k}/{d:g}": stats(v) for (g, k, d), v in G.items()}
    json.dump(S, open(os.path.join(a.dir, "summary.json"), "w"), indent=1)

    L = ["| Policy | Drift | Runs | Back within 2 m | Waypoints (of 5) | Median landing error | Runs with a collision | Timed out |",
         "|---|---|---|---|---|---|---|---|"]
    for k, lab, *_ in SERIES:
        for d in drifts:
            v = G.get(("stage1", k, d))
            if v:
                s = stats(v)
                L.append(f"| {lab} | {d:g} % | {s['n']} | {s['returned_ok_pct']} % | {s['waypoints_mean']} | "
                         f"{s['landing_err_median_m']} m | {s['collision_runs_pct']} % | {s['timeouts_pct']} % |")
    open(os.path.join(a.dir, "summary_stage1.md"), "w").write("\n".join(L) + "\n")
    B = ["| Policy | Drift | Runs | Back within 2 m | Rooms searched (mean) | Rooms searched (%) | Survivor found | Searched ≥1 room AND back safe | Runs with a collision | Timed out | Median landing error |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for k, lab, *_ in SERIES:
        for d in drifts:
            v = G.get(("buildings", k, d))
            if v:
                s = stats(v)
                B.append(f"| {lab} | {d:g} % | {s['n']} | {s['returned_ok_pct']} % | {s['rooms_mean']} | {s['rooms_frac_pct']} % | "
                         f"{s['survivor_pct']} % | {s['safe_and_useful_pct']} % | {s['collision_runs_pct']} % | {s['timeouts_pct']} % | "
                         f"{s['landing_err_median_m']} m |")
    open(os.path.join(a.dir, "summary_buildings.md"), "w").write("\n".join(B) + "\n")
    print("\n".join(L)); print(); print("\n".join(B))

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def style(ax, title):
        ax.set_title(title, fontsize=10, color=INK, loc="left"); ax.grid(color=GRID, lw=0.8)
        ax.tick_params(colors=MUTED, labelsize=8.5)
        for s_ in ("top", "right"): ax.spines[s_].set_visible(False)
        for s_ in ("left", "bottom"): ax.spines[s_].set_color(GRID)

    # figure 1: generated buildings vs drift
    panels = [("returned_ok_pct", "Came back within 2 m of home (%)", (0, 105)),
              ("rooms_mean", "Rooms really searched (mean)", None),
              ("collision_runs_pct", "Runs with a collision (%)", (0, 105))]
    fig, axs = plt.subplots(1, 3, figsize=(13, 4.3), dpi=150)
    for ax, (key, title, yl) in zip(axs, panels):
        for k, lab, col, mk, ls in SERIES:
            pts = [(i, stats(G[("buildings", k, d)])[key]) for i, d in enumerate(drifts) if G.get(("buildings", k, d))]
            if pts:
                x, y = zip(*pts)
                ax.plot(x, y, color=col, lw=2, ls=ls, marker=mk, ms=7, mec=SURF, mew=1.5, label=lab)
        ax.set_xticks(range(len(drifts))); ax.set_xticklabels([f"{d:g} %" for d in drifts])
        ax.set_xlabel("True VIO drift (% of distance flown)", fontsize=9, color=MUTED)
        if yl: ax.set_ylim(*yl)
        else: ax.set_ylim(bottom=0)
        style(ax, title)
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h, l, loc="lower center", ncol=4, fontsize=8.5, frameon=False, labelcolor=INK, handlelength=3.5)
    nb = len({r["world"] for r in runs if r["world"] not in (None, "stage1")})
    fig.suptitle(f"Stage 5: {nb} random multi-room buildings, GPS + link jammed at the entrance", fontsize=11, color=INK, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0.1, 1, 0.94)); fig.savefig(os.path.join(a.dir, "buildings_vs_drift.png"), facecolor=SURF)

    # figure 2: trade-off - rooms searched vs safe return (one point per policy and drift level)
    fig2, ax = plt.subplots(figsize=(7.5, 5.2), dpi=150)
    for k, lab, col, mk, ls in SERIES:
        P = [(stats(G[("buildings", k, d)])["rooms_mean"], stats(G[("buildings", k, d)])["returned_ok_pct"], d)
             for d in drifts if G.get(("buildings", k, d))]
        if not P:
            continue
        x, y, d = zip(*P)
        ax.plot(x, y, color=col, lw=1, ls=ls, alpha=.6)
        ax.scatter(x, y, color=col, marker=mk, s=60, edgecolor=SURF, linewidth=1.5, label=lab, zorder=3)
        for xi, yi, di in P:
            ax.annotate(f"{di:g}%", (xi, yi), xytext=(5, 4), textcoords="offset points", fontsize=7, color=MUTED)
    ax.set_xlabel("Rooms really searched (mean)", fontsize=9, color=MUTED); ax.set_ylabel("Came back within 2 m (%)", fontsize=9, color=MUTED)
    ax.set_ylim(-5, 105); ax.set_xlim(left=0)
    style(ax, "Search more, or come back? (labels: true drift)")
    ax.legend(fontsize=7.5, frameon=False, loc="center right")
    fig2.tight_layout(); fig2.savefig(os.path.join(a.dir, "tradeoff.png"), facecolor=SURF)
    print("figures saved")


if __name__ == "__main__":
    main()
