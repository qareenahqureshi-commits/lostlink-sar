"""
Simulated environment effects that the drone itself cannot see (used by the offline bench and
the sim_link ROS2 node): the radio link between the ground station and the drone.

Link model (simple, documented, deliberately conservative):
    received power  Pr = Ptx - PL(d) - n_walls * L_wall
    PL(d)           = 40 dB + 20 log10(d)           (free-space-like path loss at 2.4 GHz, d in m)
    link OK         if Pr >= sensitivity
Defaults: Ptx 20 dBm, sensitivity -85 dBm, 20 dB per wall (reinforced concrete / rubble).
=> one wall between drone and ground station is fine, two walls cut the link.
"""
import importlib.util
import math
import os
import numpy as np


def load_boxes(make_world_py):
    """Obstacle boxes from sim/worlds/make_stage1.py as (centre_ne, ax1, ax2, half1, half2, name)."""
    spec = importlib.util.spec_from_file_location("w", make_world_py)
    w = importlib.util.module_from_spec(spec); spec.loader.exec_module(w)
    out = []
    for name, (e, n), (sx, sy, sz), yaw_deg, col, collides in w.BOXES:
        if not collides:
            continue
        a = math.radians(yaw_deg)
        ax1 = np.array([math.sin(a), math.cos(a)]); ax2 = np.array([math.cos(a), -math.sin(a)])
        out.append((np.array([n, e]), ax1, ax2, sx / 2, sy / 2, name))
    return out


def boxes_from_module(w):
    """Same as load_boxes, from an already-loaded world object (e.g. a Stage 5 generated building)."""
    out = []
    for name, (e, n), (sx, sy, sz), yaw_deg, col, collides in w.BOXES:
        if not collides:
            continue
        a = math.radians(yaw_deg)
        ax1 = np.array([math.sin(a), math.cos(a)]); ax2 = np.array([math.cos(a), -math.sin(a)])
        out.append((np.array([n, e]), ax1, ax2, sx / 2, sy / 2, name))
    return out


def load_world_any(path):
    """make_stage1.py-style module, or a Stage 5 spec .json -> list of boxes."""
    if path.endswith(".json"):
        import json, types
        sp = json.load(open(path))
        w = types.SimpleNamespace(BOXES=[(n, tuple(c), tuple(s), y, col, coll) for n, c, s, y, col, coll in sp["BOXES"]])
        return boxes_from_module(w)
    return load_boxes(path)


def segment_hits_box(a, b, box):
    c, ax1, ax2, h1, h2, _ = box
    A = np.array([(a - c) @ ax1, (a - c) @ ax2]); B = np.array([(b - c) @ ax1, (b - c) @ ax2])
    D = B - A; t0, t1 = 0.0, 1.0
    for k, h in ((0, h1), (1, h2)):
        if abs(D[k]) < 1e-12:
            if abs(A[k]) > h:
                return False
            continue
        ta, tb = (-h - A[k]) / D[k], (h - A[k]) / D[k]
        t0, t1 = max(t0, min(ta, tb)), min(t1, max(ta, tb))
        if t0 > t1:
            return False
    return True


class LinkModel:
    def __init__(self, boxes, gs_ne=(0.0, 0.0), ptx_dbm=20.0, sens_dbm=-85.0, wall_db=20.0):
        self.boxes, self.gs = boxes, np.asarray(gs_ne, float)
        self.ptx, self.sens, self.wall_db = ptx_dbm, sens_dbm, wall_db

    def rssi(self, p_ne):
        p = np.asarray(p_ne[:2], float)
        d = max(1.0, float(np.linalg.norm(p - self.gs)))
        walls = sum(segment_hits_box(self.gs, p, b) for b in self.boxes)
        return self.ptx - (40.0 + 20.0 * math.log10(d)) - self.wall_db * walls, walls

    def ok(self, p_ne):
        r, _ = self.rssi(p_ne)
        return r >= self.sens
