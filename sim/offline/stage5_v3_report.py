#!/usr/bin/env python3
"""
Stage 5 v3 report: generated buildings with 3 drift seeds each (30 runs per cell) + sensitivity.

Usage:
    python3 sim/offline/stage5_v3_report.py results/stage5/offline_v3
Writes summary_v3.md, sensitivity.md, fig_safe_return_v3.png, fig_sensitivity.png into that folder.
"""
import json, math, os, sys
import numpy as np
from scipy.stats import fisher_exact

D = (2, 5, 10, 20)
POL = ("stage4", "map_reloc", "rtl_link", "stage3")
LAB = {"stage4": "Proposed", "map_reloc": "No return decision", "rtl_link": "Return on link loss", "stage3": "No drift awareness"}


def wilson(k, n, z=1.96):
    if n == 0:
        return 0, 0
    p = k / n; d = 1 + z * z / n; c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0, c - h), min(1, c + h)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "results/stage5/offline_v3"
    R = [json.loads(l) for l in open(os.path.join(out, "runs.jsonl"))]
    B = [r for r in R if r["world"] != "stage1"]
    cap = lambda r: float(r.get("capture_m", 3.0))                                  # noqa: E731
    main_ = [r for r in B if abs(r["r_hat"] - 0.05) < 1e-9 and cap(r) == 3.0]
    sel = lambda pol, d, rows=main_: [r for r in rows if r["policy"] == pol and r["drift_pct"] == d]   # noqa: E731

    L = [f"# Stage 5 v3: generated buildings, 3 drift seeds per building ({len(main_)} main runs)\n",
         "| policy | drift | safe return (95 % CI) | runs with collision | rooms searched (mean) | n |", "|---|---|---|---|---|---|"]
    for p in POL:
        for d in D:
            S = sel(p, d); n = len(S)
            if not n:
                continue
            k = sum(r["returned_ok"] for r in S); a, b = wilson(k, n)
            c = sum(r["collisions"] > 0 for r in S)
            L.append(f"| {LAB[p]} | {d} % | {k}/{n} = {100 * k / n:.0f} % ({100 * a:.0f}-{100 * b:.0f}) | "
                     f"{c}/{n} = {100 * c / n:.0f} % | {np.mean([r['rooms_searched'] for r in S]):.1f} | {n} |")

    def count(p, ds, f):
        S = [r for d in ds for r in sel(p, d)]
        return sum(f(r) for r in S), len(S)
    L.append("\n## Fisher exact tests (two-sided)\n")
    ok = lambda r: r["returned_ok"]                                                 # noqa: E731
    hit = lambda r: r["collisions"] > 0                                             # noqa: E731
    for other in ("map_reloc", "rtl_link", "stage3"):
        k1, n1 = count("stage4", (5, 10, 20), ok); k2, n2 = count(other, (5, 10, 20), ok)
        if n1 and n2:
            p = fisher_exact([[k1, n1 - k1], [k2, n2 - k2]])[1]
            L.append(f"- safe return at 5-20 % drift, proposed {k1}/{n1} vs {LAB[other]} {k2}/{n2}: p = {p:.2g}")
    for other in ("map_reloc", "stage3"):
        k1, n1 = count("stage4", D, hit); k2, n2 = count(other, D, hit)
        if n1 and n2:
            p = fisher_exact([[k1, n1 - k1], [k2, n2 - k2]])[1]
            L.append(f"- runs with a collision (all drifts), proposed {k1}/{n1} vs {LAB[other]} {k2}/{n2}: p = {p:.2g}")
    open(os.path.join(out, "summary_v3.md"), "w").write("\n".join(L) + "\n")
    print("\n".join(L))

    # ---------------- sensitivity (proposed method, one setting at a time)
    V = [("gamma 2 m", 0.05, 2.0), ("gamma 3 m, r0 5 % (default)", 0.05, 3.0), ("gamma 4 m", 0.05, 4.0),
         ("r0 2 %", 0.02, 3.0), ("r0 10 %", 0.10, 3.0)]
    S2 = ["# Sensitivity of the proposed method (generated buildings, 30 runs per cell)\n",
          "| setting | " + " | ".join(f"{d} %: safe / coll. / rooms" for d in D) + " |", "|---|" + "---|" * len(D)]
    sens = {}
    for name, rh, g in V:
        rows = [r for r in B if r["policy"] == "stage4" and abs(r["r_hat"] - rh) < 1e-9 and cap(r) == g]
        cells = []
        for d in D:
            S = [r for r in rows if r["drift_pct"] == d]; n = len(S)
            if not n:
                cells.append("-"); continue
            k = sum(r["returned_ok"] for r in S); c = sum(r["collisions"] > 0 for r in S)
            rm = np.mean([r["rooms_searched"] for r in S])
            sens[(name, d)] = (k, n, c, rm)
            cells.append(f"{100 * k / n:.0f} % / {100 * c / n:.0f} % / {rm:.1f} (n={n})")
        S2.append(f"| {name} | " + " | ".join(cells) + " |")
    open(os.path.join(out, "sensitivity.md"), "w").write("\n".join(S2) + "\n")
    print("\n" + "\n".join(S2))

    # ---------------- figures
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    C = {"stage4": "#2a78d6", "map_reloc": "#eb6834", "rtl_link": "#1baf7a", "stage3": "#eda100"}
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.4))
    for i, p in enumerate(POL):
        ys, lo, hi, cs = [], [], [], []
        for d in D:
            S = sel(p, d); n = max(len(S), 1); k = sum(r["returned_ok"] for r in S); a, b = wilson(k, n)
            ys.append(100 * k / n); lo.append(100 * a); hi.append(100 * b); cs.append(100 * sum(r["collisions"] > 0 for r in S) / n)
        x = np.arange(4) + (i - 1.5) * 0.06
        axs[0].errorbar(x, ys, yerr=[np.clip(np.array(ys) - lo, 0, None), np.clip(np.array(hi) - ys, 0, None)], color=C[p], marker="o", capsize=2, label=LAB[p])
        axs[1].plot(np.arange(4), cs, color=C[p], marker="o", label=LAB[p])
    for ax, yl in zip(axs, ("safe return within 2 m (%)", "runs with a collision (%)")):
        ax.set_xticks(range(4)); ax.set_xticklabels([f"{d} %" for d in D]); ax.set_xlabel("VIO drift"); ax.set_ylabel(yl); ax.set_ylim(-5, 105)
    axs[0].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(out, "fig_safe_return_v3.png"), dpi=130); plt.close(fig)

    fig, axs = plt.subplots(1, 2, figsize=(10, 3.4))
    for name, _, _ in V:
        if not any((name, d) in sens for d in D):
            continue
        ys = [100 * sens[(name, d)][0] / sens[(name, d)][1] if (name, d) in sens else np.nan for d in D]
        rm = [sens[(name, d)][3] if (name, d) in sens else np.nan for d in D]
        st = "-" if "default" in name else "--"
        axs[0].plot(range(4), ys, st, marker="o", label=name); axs[1].plot(range(4), rm, st, marker="o", label=name)
    for ax, yl in zip(axs, ("safe return within 2 m (%)", "rooms searched (mean)")):
        ax.set_xticks(range(4)); ax.set_xticklabels([f"{d} %" for d in D]); ax.set_xlabel("VIO drift"); ax.set_ylabel(yl)
    axs[0].set_ylim(-5, 105); axs[0].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(out, "fig_sensitivity.png"), dpi=130); plt.close(fig)


if __name__ == "__main__":
    main()
