#!/usr/bin/env python3
"""
Lost-Link SAR - Stage 1 generator
---------------------------------
Builds two files that line up exactly:
  1. A Gazebo world  "sar_building.sdf"  (damaged building with walls, pillars, debris)
  2. A QGroundControl mission  "stage1_obstacle_mission.plan"  flying straight through it

Both use the SAME home coordinates, read from your PX4 default.sdf, so the
QGC waypoints land exactly on the obstacles in Gazebo.

Usage:
    python3 sim/worlds/make_stage1.py           # run from repo root; assumes ~/PX4-Autopilot
    python3 make_stage1.py --px4 /path/to/PX4-Autopilot
    python3 make_stage1.py --plot               # also save a top-down map (needs matplotlib)

Then:
    cd ~/PX4-Autopilot
    PX4_GZ_WORLD=sar_building make px4_sitl gz_x500_depth

Frames: Gazebo world is ENU -> x = East (m), y = North (m), z = Up (m).
"""

import argparse
import glob
import json
import math
import os
import re
import sys

# ---------------------------------------------------------------------------
# 1. Scenario definition (edit here - world, plan and plot all follow)
# ---------------------------------------------------------------------------
WORLD_NAME = "sar_building"
FLIGHT_ALT = 3.0      # m above home - inside the building, below the 4 m walls
MISSION_SPEED = 2.0   # m/s - slow so the crash is easy to see
ACCEPT_RADIUS = 0.5   # m - PX4 default NAV_ACC_RAD (10 m) makes it cut every corner
HOLD_TIME = 1.0       # s - stop briefly at each waypoint -> straight legs
WALL_H = 4.0
WALL_T = 0.3

GREY = "0.55 0.55 0.55 1"
DARK = "0.35 0.32 0.30 1"
CONCRETE = "0.7 0.68 0.62 1"
RED = "0.9 0.1 0.1 1"

# Boxes: name, centre (x East, y North), size (sx, sy, sz), yaw_deg, colour, collides
BOXES = [
    # Outer walls - building footprint x 15..45, y -10..10, door gap on west wall y -1.5..1.5
    ("wall_west_s", (15.0, -5.75), (WALL_T, 8.5, WALL_H), 0, GREY, True),
    ("wall_west_n", (15.0, 5.75), (WALL_T, 8.5, WALL_H), 0, GREY, True),
    ("wall_east",   (45.0, 0.0),  (WALL_T, 20.0, WALL_H), 0, GREY, True),
    ("wall_north",  (30.0, 10.0), (30.0, WALL_T, WALL_H), 0, GREY, True),
    ("wall_south",  (30.0, -10.0), (30.0, WALL_T, WALL_H), 0, GREY, True),
    # Interior partition (south half only)
    ("partition",   (30.0, -6.5), (WALL_T, 7.0, WALL_H), 0, GREY, True),
    # Debris
    ("debris_block", (38.0, -1.0), (2.5, 1.2, 3.5), 20, DARK, True),
    ("fallen_slab",  (27.0, -7.5), (3.0, 1.5, 0.25), 15, CONCRETE, True),
    ("rubble_1",     (42.0, 6.5),  (1.5, 1.5, 1.0), 35, DARK, True),
    # Survivor marker (visual only - the SAR "target")
    ("survivor",     (41.0, -8.0), (0.5, 1.8, 0.3), 0, RED, False),
]

# Cylinders: name, centre (x, y), radius, height, colour
CYLINDERS = [
    ("pillar_1", (25.0, 1.4), 0.4, WALL_H, CONCRETE),
    ("pillar_2", (33.0, 7.0), 0.4, WALL_H, CONCRETE),
    ("pillar_3", (21.0, -6.0 + 3.0), 0.4, WALL_H, CONCRETE),
]

# Mission waypoints (x East, y North) - deliberately NOT obstacle-aware
WAYPOINTS = [
    (8.0, 0.0),    # WP1 outside the door
    (20.0, 0.0),   # WP2 just inside the door
    (38.0, 5.0),   # WP3 -> path clips pillar_1
    (38.0, -6.0),  # WP4 -> path hits debris_block
    (22.0, -6.0),  # WP5 -> path hits partition wall
]

# Fallback home if default.sdf can't be parsed (PX4 default: Zurich)
FALLBACK_HOME = (47.397971057728974, 8.546163739800146, 0.0)


# ---------------------------------------------------------------------------
# 2. Helpers
# ---------------------------------------------------------------------------
def find_default_sdf(px4_dir):
    pats = [
        os.path.join(px4_dir, "Tools", "simulation", "gz", "worlds", "default.sdf"),
        os.path.join(px4_dir, "**", "gz", "worlds", "default.sdf"),
    ]
    for p in pats:
        hits = glob.glob(p, recursive=True)
        if hits:
            return hits[0]
    return None


def read_home(sdf_text):
    def grab(tag):
        m = re.search(rf"<{tag}>\s*([-0-9.eE]+)\s*</{tag}>", sdf_text)
        return float(m.group(1)) if m else None
    lat, lon, elev = grab("latitude_deg"), grab("longitude_deg"), grab("elevation")
    if lat is None or lon is None:
        return None
    return lat, lon, elev or 0.0


def enu_to_latlon(x_east, y_north, home):
    lat0, lon0, _ = home
    R = 6378137.0
    dlat = (y_north / R) * 180.0 / math.pi
    dlon = (x_east / (R * math.cos(math.radians(lat0)))) * 180.0 / math.pi
    return lat0 + dlat, lon0 + dlon


def material(rgba):
    return f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>"


def box_model(name, c, size, yaw_deg, rgba, collides):
    sx, sy, sz = size
    geo = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    col = f'<collision name="collision">{geo}</collision>' if collides else ""
    return (
        f'<model name="{name}"><static>true</static>'
        f"<pose>{c[0]} {c[1]} {sz / 2} 0 0 {math.radians(yaw_deg):.4f}</pose>"
        f'<link name="link">{col}<visual name="visual">{geo}{material(rgba)}</visual></link></model>'
    )


def cyl_model(name, c, r, h, rgba):
    geo = f"<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>"
    return (
        f'<model name="{name}"><static>true</static>'
        f"<pose>{c[0]} {c[1]} {h / 2} 0 0 0</pose>"
        f'<link name="link"><collision name="collision">{geo}</collision>'
        f'<visual name="visual">{geo}{material(rgba)}</visual></link></model>'
    )


def build_world(default_text):
    text = re.sub(r'<world name="[^"]*">', f'<world name="{WORLD_NAME}">', default_text, count=1)
    models = [box_model(*b) for b in BOXES] + [cyl_model(*c) for c in CYLINDERS]
    block = "\n    <!-- ===== Lost-Link SAR stage 1 obstacles ===== -->\n    " + "\n    ".join(models) + "\n  "
    idx = text.rfind("</world>")
    if idx < 0:
        sys.exit("ERROR: </world> not found in default.sdf")
    return text[:idx] + block + text[idx:]


def simple_item(seq, command, frame, params):
    return {
        "AMSLAltAboveTerrain": None,
        "Altitude": params[6],
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": command,
        "doJumpId": seq,
        "frame": frame,
        "params": params,
        "type": "SimpleItem",
    }


def build_plan(home):
    lat0, lon0, alt0 = home
    items, seq = [], 1
    # Takeoff (MAV_CMD_NAV_TAKEOFF = 22), frame 3 = GLOBAL_RELATIVE_ALT
    items.append(simple_item(seq, 22, 3, [0, 0, 0, None, lat0, lon0, FLIGHT_ALT])); seq += 1
    # Change speed (MAV_CMD_DO_CHANGE_SPEED = 178), frame 2 = MISSION
    it = simple_item(seq, 178, 2, [1, MISSION_SPEED, -1, 0, 0, 0, 0]); it["Altitude"] = 0; items.append(it); seq += 1
    # Waypoints (MAV_CMD_NAV_WAYPOINT = 16)
    for x, y in WAYPOINTS:
        lat, lon = enu_to_latlon(x, y, home)
        items.append(simple_item(seq, 16, 3, [HOLD_TIME, ACCEPT_RADIUS, 0, None, lat, lon, FLIGHT_ALT])); seq += 1
    # Return to launch (MAV_CMD_NAV_RETURN_TO_LAUNCH = 20)
    it = simple_item(seq, 20, 2, [0, 0, 0, 0, 0, 0, 0]); it["Altitude"] = 0; items.append(it)
    return {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": 15,
            "firmwareType": 12,          # PX4
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": MISSION_SPEED,
            "items": items,
            "plannedHomePosition": [lat0, lon0, alt0],
            "vehicleType": 2,            # multirotor
            "version": 2,
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1,
    }


def plot_map(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
    from matplotlib.transforms import Affine2D

    fig, ax = plt.subplots(figsize=(9, 6), dpi=150)
    for name, c, (sx, sy, _), yaw, rgba, collides in BOXES:
        rgb = [float(v) for v in rgba.split()[:3]]
        r = Rectangle((c[0] - sx / 2, c[1] - sy / 2), sx, sy, color=rgb, alpha=0.9 if collides else 0.6)
        r.set_transform(Affine2D().rotate_deg_around(c[0], c[1], yaw) + ax.transData)
        ax.add_patch(r)
    for name, c, rad, h, rgba in CYLINDERS:
        ax.add_patch(Circle(c, rad, color=[float(v) for v in rgba.split()[:3]], ec="k", lw=0.5))
    pts = [(0.0, 0.0)] + WAYPOINTS + [(0.0, 0.0)]
    xs, ys = zip(*pts)
    ax.plot(xs[:-1], ys[:-1], "-o", color="#e08a00", lw=2, ms=5, label="QGC mission (no avoidance)")
    ax.plot(xs[-2:], ys[-2:], "--", color="#e08a00", lw=1, alpha=0.6, label="RTL")
    ax.plot(0, 0, "^", color="#d62728", ms=11, label="Home / takeoff")
    for i, (x, y) in enumerate(WAYPOINTS, 1):
        ax.annotate(f"WP{i}", (x, y), textcoords="offset points", xytext=(5, 6), fontsize=8)
    ax.annotate("survivor", (41, -8), textcoords="offset points", xytext=(-10, -14), fontsize=8, color="#b00")
    ax.set_aspect("equal"); ax.set_xlim(-3, 48); ax.set_ylim(-13, 13)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title(f"Stage 1 - sar_building world, mission at {FLIGHT_ALT:.0f} m")
    ax.grid(alpha=0.3); ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout(); fig.savefig(path)
    print(f"[ok] map    -> {path}")


# ---------------------------------------------------------------------------
# 3. Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--px4", default=os.path.expanduser("~/PX4-Autopilot"))
    SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # <repo>/sim
    ap.add_argument("--out", default=SIM_DIR)
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    default_sdf = find_default_sdf(args.px4)
    if default_sdf is None:
        sys.exit(f"ERROR: could not find gz/worlds/default.sdf under {args.px4}. Use --px4 <path>.")
    print(f"[..] using  {default_sdf}")
    default_text = open(default_sdf).read()

    home = read_home(default_text)
    if home is None:
        print("[!!] no spherical_coordinates in default.sdf - using PX4 default Zurich home")
        home = FALLBACK_HOME
    print(f"[..] home   lat={home[0]:.7f} lon={home[1]:.7f}")

    world_path = os.path.join(os.path.dirname(default_sdf), f"{WORLD_NAME}.sdf")
    with open(world_path, "w") as f:
        f.write(build_world(default_text))
    print(f"[ok] world  -> {world_path}")

    os.makedirs(os.path.join(args.out, "missions"), exist_ok=True)
    plan_path = os.path.join(args.out, "missions", "stage1_obstacle_mission.plan")
    with open(plan_path, "w") as f:
        json.dump(build_plan(home), f, indent=4)
    print(f"[ok] plan   -> {plan_path}")

    if args.plot:
        res = os.path.join(os.path.dirname(args.out), "results", "stage1"); os.makedirs(res, exist_ok=True)
        plot_map(os.path.join(res, "stage1_world_map.png"))

    print("\nNext:\n  cd " + args.px4 + f"\n  PX4_GZ_WORLD={WORLD_NAME} make px4_sitl gz_x500_depth")


if __name__ == "__main__":
    main()
