#!/usr/bin/env python3
"""
Offline test bench for the Stage 2 planner (no PX4, no Gazebo).

Renders a synthetic depth image of the sar_building world (same geometry as
sim/worlds/make_stage1.py) with the same camera model as the x500_depth, feeds it
through the SAME core.py pipeline used by the ROS2 node, and flies a simple
kinematic drone. Lets us debug perception/planning logic in seconds.

Usage:
    python3 sim/offline/offline_sim.py [--no-avoid] [--plot out.png]
"""
import argparse, importlib.util, math, os, sys
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "ros2_ws", "src", "lostlink_planner"))
from lostlink_planner import core  # noqa: E402


def load_world():
    spec = importlib.util.spec_from_file_location("w", os.path.join(REPO, "sim", "worlds", "make_stage1.py"))
    w = importlib.util.module_from_spec(spec); spec.loader.exec_module(w)
    return w


class World:
    """Obstacles in NED. Boxes: oriented in the horizontal plane. Cylinders: vertical."""

    def __init__(self, w):
        self.boxes, self.cyls = [], []
        for name, (e, n), (sx, sy, sz), yaw_deg, col, collides in w.BOXES:
            if not collides:
                continue
            # ENU yaw (from East, CCW) -> box axes in (n, e): ax1 along local x, ax2 along local y
            a = math.radians(yaw_deg)
            ax1 = np.array([math.sin(a), math.cos(a)])        # local x in (n,e)
            ax2 = np.array([math.cos(a), -math.sin(a)])       # local y in (n,e)
            self.boxes.append((np.array([n, e]), ax1, ax2, sx / 2, sy / 2, sz, name))
        for name, (e, n), r, h, col in w.CYLINDERS:
            self.cyls.append((np.array([n, e]), r, h, name))

    def raycast(self, origin, dirs, far=19.1):
        """origin (3,) NED, dirs (N,3) unit NED. Returns range t (N,)."""
        N = len(dirs)
        best = np.full(N, np.inf)
        o2, d2 = origin[:2], dirs[:, :2]
        oz, dz = origin[2], dirs[:, 2]
        # ground (z = 0, NED): t = -oz/dz for dz > 0 (pointing down)
        with np.errstate(divide="ignore", invalid="ignore"):
            tg = np.where(dz > 1e-6, -oz / dz, np.inf)
        best = np.minimum(best, np.where(tg > 0, tg, np.inf))
        for c, ax1, ax2, h1, h2, H, _ in self.boxes:
            lo = np.array([-h1, -h2, -H]); hi = np.array([h1, h2, 0.0])   # local (x, y, z_ned)
            rel = o2 - c
            O = np.array([rel @ ax1, rel @ ax2, oz])
            D = np.stack([d2 @ ax1, d2 @ ax2, dz], axis=1)
            with np.errstate(divide="ignore", invalid="ignore"):
                t1 = (lo - O) / D; t2 = (hi - O) / D
            tmin = np.nanmax(np.minimum(t1, t2), axis=1); tmax = np.nanmin(np.maximum(t1, t2), axis=1)
            hit = (tmax >= np.maximum(tmin, 0))
            t = np.where(tmin > 0, tmin, tmax)
            best = np.where(hit & (t > 0) & (t < best), t, best)
        for c, r, H, _ in self.cyls:
            rel = o2 - c
            a = (d2 ** 2).sum(1); b = 2 * (d2 @ rel); cc = rel @ rel - r * r
            disc = b * b - 4 * a * cc
            with np.errstate(invalid="ignore", divide="ignore"):
                t = (-b - np.sqrt(disc)) / (2 * a)
            z = oz + t * dz
            ok = (disc >= 0) & (t > 0) & (z <= 0) & (z >= -H)
            best = np.where(ok & (t < best), t, best)
        best[best > far] = np.inf
        return best

    def collides(self, p, radius=0.3):
        n, e, alt = p[0], p[1], -p[2]
        for c, ax1, ax2, h1, h2, H, name in self.boxes:
            rel = np.array([n, e]) - c
            if abs(rel @ ax1) < h1 + radius and abs(rel @ ax2) < h2 + radius and alt < H:
                return name
        for c, r, H, name in self.cyls:
            if np.hypot(n - c[0], e - c[1]) < r + radius and alt < H:
                return name
        return None


def render_depth(world, pos, yaw, proj_full, planar=True):
    """Full 640x480 depth image (planar depth, like Gazebo) for a level drone."""
    q = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    R = core.quat_to_rot(q)
    cam = np.asarray(pos) + R @ (core.CAM_OFFSET_FLU * np.array([1, -1, -1]))
    dirs_flu = proj_full.unit
    dirs_ned = (dirs_flu * np.array([1, -1, -1])) @ R.T
    t = world.raycast(cam, dirs_ned)
    depth_vals = t * proj_full.unit[:, 0] if planar else t           # planar = range * cos(angle)
    img = np.full((core.DEPTH_H, core.DEPTH_W), np.inf, dtype=np.float32)
    img[proj_full.idx] = depth_vals
    return img, q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-avoid", action="store_true")
    ap.add_argument("--plot", default=None)
    ap.add_argument("--tmax", type=float, default=240)
    a = ap.parse_args()

    w = load_world()
    world = World(w)
    wps_ne = [(n, e) for (e, n) in w.WAYPOINTS]            # make_stage1 is (east, north)
    proj = core.DepthProjector(stride=8)
    mf = core.MissionFollower(wps_ne, avoid=not a.no_avoid)

    dt, t = 0.1, 0.0
    pos = np.array([0.0, 0.0, -3.0]); yaw = 0.0
    traj, hits, last_hit = [], [], None
    while t < a.tmax and not mf.done:
        img, q = render_depth(world, pos, yaw, proj)
        mf.grid.insert(proj.to_world(img, pos, q))
        n_sp, e_sp, yaw_sp, info = mf.step(pos, yaw)
        # kinematic drone: P-controller on position (like PX4 MPC_XY_P ~0.95), speed cap 2 m/s
        v = 0.95 * (np.array([n_sp, e_sp]) - pos[:2])
        sp = np.linalg.norm(v)
        if sp > 2.0:
            v *= 2.0 / sp
        pos[:2] += v * dt
        yaw += np.clip(core.wrap(yaw_sp - yaw), -math.radians(60) * dt, math.radians(60) * dt)
        hit = world.collides(pos)
        if hit and hit != last_hit:
            hits.append((round(t, 1), hit, pos[:2].round(2).tolist()))
        last_hit = hit
        traj.append((t, pos[0], pos[1], yaw, info["mode"]))
        t += dt

    print(f"finished={mf.done} t={t:.1f}s replans={mf.replans} collisions={len(hits)}")
    for h in hits:
        print("  HIT", h)

    if a.plot:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        T = np.array([(x[1], x[2]) for x in traj])
        fig, ax = plt.subplots(figsize=(10, 6), dpi=130)
        occ = mf.grid.occupied
        ii, jj = np.nonzero(occ)
        ax.scatter(mf.grid.e0 + (jj + .5) * mf.grid.res, mf.grid.n0 + (ii + .5) * mf.grid.res,
                   s=2, c="#8a8a85", label="mapped obstacles (from depth)")
        ax.plot(T[:, 1], T[:, 0], color="#2a78d6", lw=1.6, label="flown path")
        ax.plot([0] + [e for n, e in wps_ne], [0] + [n for n, e in wps_ne], "--o", color="#eb6834",
                ms=4, lw=1, label="mission waypoints")
        for h in hits:
            ax.plot(h[2][1], h[2][0], "X", color="k", ms=10)
        ax.set_aspect("equal"); ax.grid(alpha=.3); ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
        ax.set_title(f"Offline test: {'NO avoidance' if a.no_avoid else 'EGO-style avoidance'} "
                     f"- {len(hits)} collisions")
        ax.legend(fontsize=8, loc="upper left"); fig.tight_layout(); fig.savefig(a.plot)
        print("saved", a.plot)
    return len(hits)


if __name__ == "__main__":
    main()
