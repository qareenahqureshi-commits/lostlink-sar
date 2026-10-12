"""
Lost-Link SAR - core perception + planning (no ROS dependency, so it can be unit-tested offline).

Frames
------
* World / map: PX4 local NED, horizontal only -> (n, e) in metres. Altitude = -z.
* Camera (Gazebo convention): x forward, y left, z up.
* Body (PX4): FRD (x forward, y right, z down). Camera is mounted forward at CAM_OFFSET_FLU.

Pipeline
--------
depth image -> 3D points (camera) -> body -> world NED -> keep points in an altitude band
-> 2D occupancy grid (hit counts) -> inflate by drone radius -> A* guide path
-> line-of-sight pruning -> gradient smoothing on a distance field (EGO-Planner-style:
"guide path + push away from obstacles + stay smooth") -> carrot point for PX4 offboard.
"""
import heapq
import math
import numpy as np



def distance_field(occ, res, max_dist=2.0):
    """Euclidean distance (m) from every cell to the nearest occupied cell, capped at max_dist.

    Pure NumPy (no SciPy): for every integer offset inside the cap radius, shift the occupancy
    grid and keep the smallest offset length that hits an obstacle. ~200 shifted array ops.
    """
    R = int(math.ceil(max_dist / res))
    dist = np.full(occ.shape, max_dist, dtype=np.float32)
    if not occ.any():
        return dist
    H, W = occ.shape
    offs = sorted(((di, dj) for di in range(-R, R + 1) for dj in range(-R, R + 1)
                   if di * di + dj * dj <= R * R), key=lambda o: o[0] ** 2 + o[1] ** 2)
    for di, dj in offs:
        d = math.hypot(di, dj) * res
        # cell (i, j) is within d of an obstacle if occ[i+di, j+dj]
        src = occ[max(di, 0):H + min(di, 0), max(dj, 0):W + min(dj, 0)]
        dst = dist[max(-di, 0):H + min(-di, 0), max(-dj, 0):W + min(-dj, 0)]
        np.minimum(dst, np.where(src, d, max_dist), out=dst)
    return dist

# x500_depth / OakD-Lite depth camera (from PX4 model files, checked in stage 2.1)
DEPTH_W, DEPTH_H = 640, 480
DEPTH_HFOV = 1.274          # rad (73 deg)
DEPTH_NEAR, DEPTH_FAR = 0.2, 19.1
CAM_OFFSET_FLU = np.array([0.12, 0.03, 0.242])


# ----------------------------------------------------------------------------- geometry
def quat_to_rot(q):
    """PX4 quaternion [w, x, y, z] (FRD body -> NED) to 3x3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def yaw_from_quat(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class DepthProjector:
    """Turns a depth image into world-frame points (subsampled)."""

    def __init__(self, w=DEPTH_W, h=DEPTH_H, hfov=DEPTH_HFOV, stride=8, planar=True,
                 near=DEPTH_NEAR, far=DEPTH_FAR):
        self.w, self.h, self.stride, self.planar = w, h, stride, planar
        self.near, self.far = near, far
        f = (w / 2.0) / math.tan(hfov / 2.0)
        us = np.arange(0, w, stride)
        vs = np.arange(0, h, stride)
        uu, vv = np.meshgrid(us, vs)
        self.idx = (vv.ravel(), uu.ravel())
        # unit-depth direction for each sampled pixel in camera frame (x fwd, y left, z up)
        dy = -(uu.ravel() - (w - 1) / 2.0) / f
        dz = -(vv.ravel() - (h - 1) / 2.0) / f
        self.dirs = np.stack([np.ones_like(dy), dy, dz], axis=1)          # planar-depth scaling
        self.unit = self.dirs / np.linalg.norm(self.dirs, axis=1, keepdims=True)  # range scaling

    def to_world(self, depth, pos_ned, q):
        """depth: HxW float array (metres). pos_ned: (3,), q: [w,x,y,z]. Returns Nx3 NED points."""
        d = depth[self.idx]
        ok = np.isfinite(d) & (d > self.near) & (d < self.far - 0.1)
        d = d[ok]
        if d.size == 0:
            return np.zeros((0, 3))
        rays = (self.dirs if self.planar else self.unit)[ok]
        p_cam = rays * d[:, None]                                  # FLU camera frame
        p_flu = p_cam + CAM_OFFSET_FLU
        p_frd = p_flu * np.array([1.0, -1.0, -1.0])
        return p_frd @ quat_to_rot(q).T + np.asarray(pos_ned)


# ----------------------------------------------------------------------------- map
class OccupancyMap2D:
    """Accumulates obstacle hits in an altitude band into a 2D grid (static world)."""

    def __init__(self, n_range=(-15.0, 20.0), e_range=(-10.0, 55.0), res=0.25,
                 alt_band=(1.0, 5.0), min_hits=2, inflate=0.8):
        self.n0, self.e0, self.res = n_range[0], e_range[0], res
        self.nn = int(round((n_range[1] - n_range[0]) / res))
        self.ne = int(round((e_range[1] - e_range[0]) / res))
        self.hits = np.zeros((self.nn, self.ne), dtype=np.uint16)
        self.alt_band, self.min_hits, self.inflate_r = alt_band, min_hits, inflate
        self._dirty = True
        self._occ = self._infl = self._dist = None

    # grid helpers
    def to_cell(self, n, e):
        return int((n - self.n0) / self.res), int((e - self.e0) / self.res)

    def to_xy(self, i, j):
        return self.n0 + (i + 0.5) * self.res, self.e0 + (j + 0.5) * self.res

    def inside(self, i, j):
        return 0 <= i < self.nn and 0 <= j < self.ne

    def insert(self, pts_ned):
        if len(pts_ned) == 0:
            return
        alt = -pts_ned[:, 2]
        m = (alt > self.alt_band[0]) & (alt < self.alt_band[1])
        p = pts_ned[m]
        i = ((p[:, 0] - self.n0) / self.res).astype(int)
        j = ((p[:, 1] - self.e0) / self.res).astype(int)
        k = (i >= 0) & (i < self.nn) & (j >= 0) & (j < self.ne)
        np.add.at(self.hits, (i[k], j[k]), 1)
        self._dirty = True

    def _update(self):
        if not self._dirty:
            return
        self._occ = self.hits >= self.min_hits
        free = ~self._occ
        self._dist = distance_field(self._occ, self.res)               # metres to nearest obstacle (capped 2 m)
        self._infl = self._dist < self.inflate_r
        self._dirty = False

    @property
    def occupied(self):
        self._update(); return self._occ

    @property
    def inflated(self):
        self._update(); return self._infl

    @property
    def dist(self):
        self._update(); return self._dist

    def dist_at(self, n, e):
        i, j = self.to_cell(n, e)
        if not self.inside(i, j):
            return 10.0
        return float(self.dist[i, j])

    def line_free(self, a, b, use_inflated=True, step=None):
        grid = self.inflated if use_inflated else self.occupied
        step = step or self.res * 0.5
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(2, int(L / step) + 1)
        for t in np.linspace(0, 1, n):
            i, j = self.to_cell(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            if self.inside(i, j) and grid[i, j]:
                return False
        return True


# ----------------------------------------------------------------------------- planner
class EgoStylePlanner:
    """A* guide path on the inflated grid + pruning + distance-field gradient smoothing."""

    def __init__(self, grid: OccupancyMap2D, d_safe=1.2, smooth_iters=60):
        self.g, self.d_safe, self.iters = grid, d_safe, smooth_iters

    def _nearest_free(self, cell):
        infl = self.g.inflated
        if self.g.inside(*cell) and not infl[cell]:
            return cell
        for r in range(1, 20):
            for di in range(-r, r + 1):
                for dj in (-r, r) if abs(di) != r else range(-r, r + 1):
                    c = (cell[0] + di, cell[1] + dj)
                    if self.g.inside(*c) and not infl[c]:
                        return c
        return cell

    def astar(self, start, goal):
        infl = self.g.inflated
        s = self._nearest_free(self.g.to_cell(*start))
        t = self._nearest_free(self.g.to_cell(*goal))
        if s == t:
            return [start, goal]
        nbrs = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
                (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        h = lambda c: math.hypot(c[0] - t[0], c[1] - t[1])
        openq = [(h(s), 0.0, s)]
        came, cost = {s: None}, {s: 0.0}
        while openq:
            _, gc, c = heapq.heappop(openq)
            if c == t:
                break
            if gc > cost[c]:
                continue
            for di, dj, w in nbrs:
                nb = (c[0] + di, c[1] + dj)
                if not self.g.inside(*nb) or infl[nb]:
                    continue
                # small extra cost near obstacles keeps the guide path centred in gaps
                ng = gc + w * (1.0 + 0.5 * max(0.0, self.d_safe - self.g.dist[nb]))
                if ng < cost.get(nb, 1e18):
                    cost[nb], came[nb] = ng, c
                    heapq.heappush(openq, (ng + h(nb), ng, nb))
        if t not in came:
            return None
        cells, c = [], t
        while c is not None:
            cells.append(c); c = came[c]
        cells.reverse()
        pts = [self.g.to_xy(*c) for c in cells]
        pts[0], pts[-1] = tuple(start), tuple(goal) if not infl[self.g.to_cell(*goal)] else pts[-1]
        return pts

    def prune(self, pts):
        out, i = [pts[0]], 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self.g.line_free(pts[i], pts[j]):
                j -= 1
            out.append(pts[j]); i = j
        return out

    def smooth(self, pts, spacing=0.5):
        """Resample, then gradient descent on smoothness + obstacle (distance-field) cost."""
        P = [np.array(pts[0], float)]
        for a, b in zip(pts[:-1], pts[1:]):
            a, b = np.array(a, float), np.array(b, float)
            n = max(1, int(np.linalg.norm(b - a) / spacing))
            for k in range(1, n + 1):
                P.append(a + (b - a) * k / n)
        P = np.array(P)
        if len(P) < 3:
            return P
        dist = self.g.dist
        gi, gj = np.gradient(dist)                     # gradient of distance field (per cell)
        for _ in range(self.iters):
            lap = 2 * P[1:-1] - P[:-2] - P[2:]          # smoothness gradient
            grad = 0.5 * lap
            for k, p in enumerate(P[1:-1]):
                i, j = self.g.to_cell(*p)
                if self.g.inside(i, j) and dist[i, j] < self.d_safe:
                    push = np.array([gi[i, j], gj[i, j]])
                    nrm = np.linalg.norm(push)
                    if nrm > 1e-6:
                        grad[k] -= 0.6 * (self.d_safe - dist[i, j]) * push / nrm
            P[1:-1] -= 0.5 * grad
        # keep result only if it is still collision-free
        for a, b in zip(P[:-1], P[1:]):
            if not self.g.line_free(a, b):
                return np.array(pts, float)
        return P

    def plan(self, start, goal):
        start, goal = tuple(start), tuple(goal)
        if self.g.line_free(start, goal):
            return np.array([start, goal], float), "direct"
        raw = self.astar(start, goal)
        if raw is None:
            return None, "no_path"
        return self.smooth(self.prune(raw)), "astar"


def carrot(path, pos, lookahead=1.5):
    """Point on the path `lookahead` metres beyond the closest point to pos."""
    P = np.asarray(path, float)
    if len(P) == 1:
        return P[0]
    best, best_d, best_s = 0, 1e9, 0.0
    seg_len = np.linalg.norm(np.diff(P, axis=0), axis=1)
    s_acc = np.concatenate([[0], np.cumsum(seg_len)])
    for k in range(len(P) - 1):
        a, b = P[k], P[k + 1]
        ab = b - a
        t = 0.0 if seg_len[k] < 1e-9 else np.clip(np.dot(pos - a, ab) / seg_len[k] ** 2, 0, 1)
        q = a + t * ab
        d = np.linalg.norm(pos - q)
        if d < best_d:
            best, best_d, best_s = k, d, s_acc[k] + t * seg_len[k]
    target_s = min(best_s + lookahead, s_acc[-1])
    k = int(np.searchsorted(s_acc, target_s, side="right") - 1)
    k = min(k, len(P) - 2)
    t = 0.0 if seg_len[k] < 1e-9 else (target_s - s_acc[k]) / seg_len[k]
    return P[k] + t * (P[k + 1] - P[k])


# ----------------------------------------------------------------------------- mission logic
class MissionFollower:
    """Waypoint sequencing + replanning + yaw-first turning. Pure logic, no I/O."""

    def __init__(self, waypoints_ne, home_ne=(0.0, 0.0), alt=3.0, avoid=True,
                 reach_r=0.6, lookahead=1.5, yaw_first_deg=40.0, map_kwargs=None):
        self.grid = OccupancyMap2D(**(map_kwargs or {}))
        self.planner = EgoStylePlanner(self.grid)
        self.goals = [tuple(w) for w in waypoints_ne] + [tuple(home_ne)]
        self.k, self.alt, self.avoid = 0, alt, avoid
        self.reach_r, self.lookahead = reach_r, lookahead
        self.yaw_first = math.radians(yaw_first_deg)
        self.path, self.mode, self.done = None, "init", False
        self.replans = 0

    @property
    def goal(self):
        return self.goals[min(self.k, len(self.goals) - 1)]

    def step(self, pos_ned, yaw):
        """Returns (setpoint_n, setpoint_e, setpoint_yaw, info). Call at ~5-10 Hz."""
        p = np.array(pos_ned[:2], float)
        if self.done:
            return p[0], p[1], yaw, {"mode": "done"}
        if np.linalg.norm(p - self.goal) < self.reach_r:
            self.k += 1; self.path = None
            if self.k >= len(self.goals):
                self.done = True
                return p[0], p[1], yaw, {"mode": "done"}
        if not self.avoid:
            self.path, self.mode = np.array([p, self.goal]), "direct(no-avoid)"
        else:
            need = self.path is None
            if not need:  # replan if remaining path is now blocked by newly mapped obstacles
                for a, b in zip(self.path[:-1], self.path[1:]):
                    if not self.grid.line_free(a, b):
                        need = True; break
            if not need and np.linalg.norm(self.path[0] - p) > 2.0:
                need = True
            if need:
                path, mode = self.planner.plan(p, self.goal)
                self.replans += 1
                if path is None:      # no path known: hold and look around
                    return p[0], p[1], wrap(yaw + 0.5), {"mode": "no_path"}
                self.path, self.mode = path, mode
        c = carrot(self.path, p, self.lookahead)
        d = c - p
        if np.linalg.norm(d) < 0.2:
            return c[0], c[1], yaw, {"mode": self.mode}
        yaw_sp = math.atan2(d[1], d[0])
        if abs(wrap(yaw_sp - yaw)) > self.yaw_first:   # turn on the spot first: camera must see where we go
            return p[0], p[1], yaw_sp, {"mode": self.mode + "+yaw"}
        return c[0], c[1], yaw_sp, {"mode": self.mode}
