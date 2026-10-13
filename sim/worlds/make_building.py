#!/usr/bin/env python3
"""
Lost-Link SAR - Stage 5 random building generator
-------------------------------------------------
One seed -> one damaged multi-room building: a long corridor from the entrance with rooms
on both sides, a door into every room, debris and pillars inside, and a search mission that
visits every room (corridor point in front of the door, then the room centre) in order of depth.

Why: the Stage 1 building is small - from the doorway the camera sees most of it, so
relocalisation almost always works. In these deeper buildings the rooms cannot be seen from
the entrance, which is where the return decision has to earn its keep. Random seeds show the
method is not tuned to one map (the usual practice: procedurally generated environments).

Outputs (same geometry everywhere):
  sim/worlds/generated/building_<seed>.json   spec used by the offline bench, reports, sim_link
  <PX4>/.../gz/worlds/lostlink_b<seed>.sdf     Gazebo world            (with --px4)
  sim/missions/building_<seed>.plan            QGroundControl mission  (with --px4)

Usage:
    python3 sim/worlds/make_building.py --seed 3 --plot            # spec + top-down picture
    python3 sim/worlds/make_building.py --seed 3 --px4 ~/PX4-Autopilot

Frames: like make_stage1.py, x = East, y = North (ENU), metres.
"""
import argparse
import importlib.util
import json
import math
import os
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GEN_DIR = os.path.join(HERE, "generated")
WALL_H, WALL_T = 4.0, 0.3
GREY, DARK, CONCRETE, RED = "0.55 0.55 0.55 1", "0.35 0.32 0.30 1", "0.7 0.68 0.62 1", "0.9 0.1 0.1 1"
DOOR_W = 3.0          # room doors: the real depth map in Gazebo is ~0.5 m noisier than the offline bench, so leave margin
CORR_HALF = 2.0       # corridor half width (4.0 m)
X0 = 15.0             # west wall (entrance) - same as the Stage 1 building


def _split(rng, x0, x1, wmin=8.0, wmax=12.0):
    """Split [x0, x1] into room widths in [wmin, wmax]."""
    cuts, x = [x0], x0
    while x1 - x > wmax:
        w = rng.uniform(wmin, wmax)
        if x1 - (x + w) < wmin:
            w = (x1 - x) / 2
        x += w; cuts.append(x)
    cuts.append(x1)
    return cuts


def _hwall(name, xa, xb, y, gaps):
    """Horizontal wall from xa to xb at y, with door gaps [(xc, w)]. Returns box list."""
    out, x, k = [], xa, 0
    for xc, w in sorted(gaps):
        if xc - w / 2 - x > 0.05:
            out.append((f"{name}_{k}", ((x + xc - w / 2) / 2, y), (xc - w / 2 - x, WALL_T, WALL_H), 0, GREY, True)); k += 1
        x = xc + w / 2
    if xb - x > 0.05:
        out.append((f"{name}_{k}", ((x + xb) / 2, y), (xb - x, WALL_T, WALL_H), 0, GREY, True))
    return out


def generate(seed, length=None):
    rng = np.random.default_rng(1000 + seed)
    L = float(length or rng.uniform(42, 60))
    X1 = X0 + L
    cy = float(rng.uniform(-1.5, 1.5))                         # corridor centre line (North)
    dn, ds = float(rng.uniform(7, 9)), float(rng.uniform(7, 9))  # room depths north / south
    yc_n, yc_s = cy + CORR_HALF, cy - CORR_HALF                  # corridor walls
    y_top, y_bot = yc_n + dn, yc_s - ds
    boxes, cyls, rooms = [], [], []

    # outer walls (entrance = open west end of the corridor)
    boxes += [("wall_west_n", (X0, (yc_n + y_top) / 2), (WALL_T, y_top - yc_n, WALL_H), 0, GREY, True),
              ("wall_west_s", (X0, (y_bot + yc_s) / 2), (WALL_T, yc_s - y_bot, WALL_H), 0, GREY, True),
              ("wall_east", (X1, (y_top + y_bot) / 2), (WALL_T, y_top - y_bot, WALL_H), 0, GREY, True),
              ("wall_north", ((X0 + X1) / 2, y_top), (L, WALL_T, WALL_H), 0, GREY, True),
              ("wall_south", ((X0 + X1) / 2, y_bot), (L, WALL_T, WALL_H), 0, GREY, True)]

    for side, ywall, yfar in (("n", yc_n, y_top), ("s", yc_s, y_bot)):
        cuts = _split(rng, X0, X1)
        gaps = []
        for i, (a, b) in enumerate(zip(cuts[:-1], cuts[1:])):
            xd = float(rng.uniform(a + 0.6 + DOOR_W / 2, b - 0.6 - DOOR_W / 2))
            gaps.append((xd, DOOR_W))
            centre = ((a + b) / 2, (ywall + yfar) / 2)
            rooms.append({"name": f"room_{side}{i}", "x": [a, b], "y": sorted([ywall, yfar]),
                          "door": [xd, ywall], "centre": list(centre)})
            if i > 0:                                            # dividing wall between rooms
                boxes.append((f"div_{side}{i}", (a, (ywall + yfar) / 2), (WALL_T, abs(yfar - ywall), WALL_H), 0, GREY, True))
        boxes += _hwall(f"corr_{side}", X0, X1, ywall, gaps)

    # debris inside rooms: keep clear of the door->centre lane and of the walls
    def free(p, r, room):
        (a, b), (lo, hi) = room["x"], room["y"]
        if not (a + r + 0.5 < p[0] < b - r - 0.5 and lo + r + 0.5 < p[1] < hi - r - 0.5):
            return False
        d0, c = np.array(room["door"]), np.array(room["centre"])
        d0 = d0 + np.array([0, np.sign(c[1] - d0[1]) * 0.5])
        t = np.clip(np.dot(p - d0, c - d0) / np.dot(c - d0, c - d0), 0, 1)
        if np.linalg.norm(p - (d0 + t * (c - d0))) < r + 1.8:
            return False
        return np.linalg.norm(p - c) > r + 1.8

    k = 0
    for room in rooms:
        for _ in range(int(rng.integers(1, 3))):
            for _try in range(30):
                sx, sy = rng.uniform(0.8, 2.4), rng.uniform(0.6, 1.4)
                p = np.array([rng.uniform(*room["x"]), rng.uniform(*room["y"])])
                if free(p, math.hypot(sx, sy) / 2, room):
                    boxes.append((f"debris_{k}", (float(p[0]), float(p[1])), (float(sx), float(sy), float(rng.uniform(1.2, 3.5))),
                                  float(rng.uniform(0, 90)), DARK, True)); k += 1
                    break
        if rng.random() < 0.6:
            for _try in range(30):
                p = np.array([rng.uniform(*room["x"]), rng.uniform(*room["y"])])
                if free(p, 0.4, room):
                    cyls.append((f"pillar_{k}", (float(p[0]), float(p[1])), 0.35, WALL_H, CONCRETE)); k += 1
                    break

    # a little clutter in the corridor, against a wall and not in front of a door (>= 2.2 m stays free)
    doors_x = [r["door"][0] for r in rooms]
    for j in range(int(rng.integers(1, 3))):
        for _try in range(30):
            x = float(rng.uniform(X0 + 6, X1 - 3)); side = 1 if rng.random() < 0.5 else -1
            if min(abs(x - d) for d in doors_x) > DOOR_W:
                y = cy + side * (CORR_HALF - 0.45)
                boxes.append((f"corr_debris_{j}", (x, y), (float(rng.uniform(1.0, 2.0)), 0.6, 1.5), 0, DARK, True))
                break

    # survivor in one random room (visual only)
    sv = rooms[int(rng.integers(len(rooms)))]
    svp = (sv["centre"][0] + 1.2, sv["centre"][1])
    boxes.append(("survivor", svp, (0.5, 1.8, 0.3), 0, RED, False))

    # search mission: outside, just inside, then every room by depth: corridor point at its door, room centre
    wps, room_wp = [(8.0, cy), (X0 + 4.0, cy)], []
    for room in sorted(rooms, key=lambda r: (r["door"][0], r["name"])):
        wps.append((room["door"][0], cy))
        wps.append(tuple(room["centre"])); room_wp.append(len(wps) - 1)
    spec = {
        "seed": seed, "world_name": f"lostlink_b{seed}", "length_m": round(L, 1),
        "BOXES": [[n, list(c), list(s), y, col, coll] for n, c, s, y, col, coll in boxes],
        "CYLINDERS": [[n, list(c), r, h, col] for n, c, r, h, col in cyls],
        "WAYPOINTS": [list(map(float, w)) for w in wps], "room_waypoints": room_wp,
        "rooms": rooms, "survivor_room": sv["name"], "survivor_wp": room_wp[[r["name"] for r in sorted(rooms, key=lambda r: (r["door"][0], r["name"]))].index(sv["name"])],
        "exit_en": [X0, cy], "bounds_en": [[-10.0, X1 + 8.0], [y_bot - 6.0, y_top + 6.0]],
    }
    return spec


def as_module(spec):
    """Spec -> object with the same attributes as make_stage1 (BOXES, CYLINDERS, WAYPOINTS, ...)."""
    m = types.SimpleNamespace()
    m.BOXES = [(n, tuple(c), tuple(s), y, col, coll) for n, c, s, y, col, coll in spec["BOXES"]]
    m.CYLINDERS = [(n, tuple(c), r, h, col) for n, c, r, h, col in spec["CYLINDERS"]]
    m.WAYPOINTS = [tuple(w) for w in spec["WAYPOINTS"]]
    m.WORLD_NAME, m.spec = spec["world_name"], spec
    return m


def load(seed_or_path):
    """Seed (int / digit string) or path to a spec JSON -> module-like world (generated on the fly)."""
    s = str(seed_or_path)
    if s.isdigit():
        return as_module(generate(int(s)))
    return as_module(json.load(open(s)))


def map_kwargs(spec, res=0.25):
    """Occupancy-grid extent for the planner (north, east ranges) covering the whole building."""
    (e0, e1), (n0, n1) = spec["bounds_en"]
    return {"n_range": (n0, n1), "e_range": (e0, e1), "res": res}


def _stage1():
    s = importlib.util.spec_from_file_location("ms1", os.path.join(HERE, "make_stage1.py"))
    m = importlib.util.module_from_spec(s); s.loader.exec_module(m); return m


def plot(spec, path):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
    from matplotlib.transforms import Affine2D
    w = as_module(spec)
    (e0, e1), (n0, n1) = spec["bounds_en"]
    fig, ax = plt.subplots(figsize=(11, 11 * (n1 - n0) / (e1 - e0) + 1.2), dpi=140)
    for name, c, (sx, sy, _), yaw, rgba, coll in w.BOXES:
        r = Rectangle((c[0] - sx / 2, c[1] - sy / 2), sx, sy, color="#b9b6ad" if coll else "#e05050", lw=0)
        r.set_transform(Affine2D().rotate_deg_around(c[0], c[1], yaw) + ax.transData); ax.add_patch(r)
    for name, c, rad, h, rgba in w.CYLINDERS:
        ax.add_patch(Circle(c, rad, color="#b9b6ad", ec="#8a8a85", lw=0.5))
    P = np.array([(0, 0)] + list(w.WAYPOINTS))
    ax.plot(P[:, 0], P[:, 1], "-", color="#eb6834", lw=1, alpha=.8, label="search mission")
    rw = spec["room_waypoints"]
    ax.plot(P[1:][rw, 0], P[1:][rw, 1], "o", color="#eb6834", ms=5, label="room centres to search")
    ax.plot(0, 0, "^", color="#2b2b2b", ms=9); ax.annotate("home", (0, 0), xytext=(4, -12), textcoords="offset points", fontsize=8)
    ax.set_aspect("equal"); ax.set_xlim(e0, e1); ax.set_ylim(n0, n1); ax.grid(alpha=.2)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title(f"Building seed {spec['seed']}: {len(spec['rooms'])} rooms, {spec['length_m']} m deep, "
                 f"{len(spec['WAYPOINTS'])} waypoints (red = survivor)", fontsize=10)
    ax.legend(fontsize=8, loc="lower right", frameon=False)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--px4", default=None, help="also write the Gazebo world + QGC plan")
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()
    spec = generate(a.seed)
    os.makedirs(GEN_DIR, exist_ok=True)
    jp = os.path.join(GEN_DIR, f"building_{a.seed}.json")
    json.dump(spec, open(jp, "w"), indent=1)
    print(f"[ok] spec  -> {jp}  ({len(spec['rooms'])} rooms, {spec['length_m']} m, {len(spec['WAYPOINTS'])} waypoints)")
    if a.plot:
        pp = os.path.join(GEN_DIR, f"building_{a.seed}.png"); plot(spec, pp); print(f"[ok] map   -> {pp}")
    if a.px4:
        ms1, w = _stage1(), as_module(spec)
        ms1.BOXES, ms1.CYLINDERS, ms1.WAYPOINTS, ms1.WORLD_NAME = w.BOXES, w.CYLINDERS, w.WAYPOINTS, w.WORLD_NAME
        sdf = ms1.find_default_sdf(os.path.expanduser(a.px4))
        text = open(sdf).read()
        home = ms1.read_home(text) or ms1.FALLBACK_HOME
        wp = os.path.join(os.path.dirname(sdf), f"{w.WORLD_NAME}.sdf")
        open(wp, "w").write(ms1.build_world(text)); print(f"[ok] world -> {wp}")
        os.makedirs(os.path.join(os.path.dirname(HERE), "missions"), exist_ok=True)
        pl = os.path.join(os.path.dirname(HERE), "missions", f"building_{a.seed}.plan")
        json.dump(ms1.build_plan(home), open(pl, "w"), indent=2); print(f"[ok] plan  -> {pl}")


if __name__ == "__main__":
    main()
