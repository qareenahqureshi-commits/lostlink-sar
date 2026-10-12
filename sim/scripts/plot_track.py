#!/usr/bin/env python3
"""
Draw the flown track over the sar_building world map.

Usage:
    python3 sim/scripts/plot_track.py --stage stage1 [--tmax 629] [--title "..."]

Reads  results/<stage>/track.csv   (from analyze_log.py)
Writes results/<stage>/track.png
Obstacles and waypoints come from sim/worlds/make_stage1.py, so the map always matches the world.
"""
import argparse, importlib.util, os
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.transforms import Affine2D

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def load_world():
    p = os.path.join(REPO, "sim", "worlds", "make_stage1.py")
    spec = importlib.util.spec_from_file_location("world", p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def rgb(s):
    return [float(v) for v in s.split()[:3]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", default="stage1")
    ap.add_argument("--tmax", type=float, default=None, help="cut the track at this time (s), e.g. before RTL climb")
    ap.add_argument("--title", default=None)
    a = ap.parse_args()

    w = load_world()
    d = pd.read_csv(os.path.join(REPO, "results", a.stage, "track.csv"))
    f = d[(d.alt_m > 0.8) & (d.alt_m < 6)]          # low-altitude part = inside the building
    if a.tmax:
        f = f[f.t_s < a.tmax]

    fig, ax = plt.subplots(figsize=(10, 6.2), dpi=150)
    for name, c, (sx, sy, sz), yaw, col, collides in w.BOXES:
        r = Rectangle((c[0] - sx / 2, c[1] - sy / 2), sx, sy, color=rgb(col), alpha=0.9 if collides else 0.6)
        r.set_transform(Affine2D().rotate_deg_around(c[0], c[1], yaw) + ax.transData)
        ax.add_patch(r)
    for name, c, rad, h, col in w.CYLINDERS:
        ax.add_patch(Circle(c, rad, color=rgb(col), ec="k", lw=0.5))

    xs, ys = zip(*([(0, 0)] + w.WAYPOINTS))
    ax.plot(xs, ys, "--o", color="#e08a00", lw=1.2, ms=4, alpha=0.8, label=f"Planned mission ({w.FLIGHT_ALT:.0f} m)")
    ax.plot(f.east_m, f.north_m, color="#1f5fbf", lw=1.6, label="Flown track (PX4 log)")
    for i, (x, y) in enumerate(w.WAYPOINTS, 1):
        ax.annotate(f"WP{i}", (x, y), xytext=(4, 5), textcoords="offset points", fontsize=8)

    ax.set_aspect("equal"); ax.set_xlim(-3, 50); ax.set_ylim(-14, 13); ax.grid(alpha=0.3)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title(a.title or f"{a.stage}: flown track in sar_building")
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    out = os.path.join(REPO, "results", a.stage, "track.png")
    fig.savefig(out)
    print("saved:", out)


if __name__ == "__main__":
    main()
