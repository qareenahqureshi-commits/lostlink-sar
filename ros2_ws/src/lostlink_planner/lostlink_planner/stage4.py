"""
Lost-Link SAR - Stage 4: drift-aware mapping, anchor relocalisation, breadcrumb homing and the
return decision. (No ROS dependency: the same code runs in the ROS2 node and the offline bench.)

What Stage 3 showed (GPS jammed at the door, drifting VIO):
  1. the map draws walls twice (same wall seen from two drifted positions); the doorway closes
     in the map -> the drone is TRAPPED;
  2. obstacles mapped long ago sit in the wrong place -> REAL clearance shrinks silently;
  3. nothing decides WHEN the drone should stop the mission and come back.

Stage 4 pieces
--------------
DriftAwareMap    every cell remembers when (in metres flown without GPS, s) it was last seen;
                 cells not re-seen within W = e_tol / r_hat metres are ignored for planning.
                 Everything the planner uses is consistent with the current pose to ~e_tol.
                 It also keeps an ANCHOR map: everything seen while the position was still
                 trustworthy (GPS, or the first metres after losing it).
Relocalisation   the drone matches its recent depth observations against the anchor map
                 (2D correlative scan matching, translation only) to measure its drift and
                 correct it - e.g. when it comes back to the doorway it saw with GPS.
Breadcrumbs      the route actually flown, as a graph (GBPlanner-style global graph);
                 Dijkstra gives a known way back to the exit/home and its length.
ReturnDecision   (the novelty) before and during every leg, predict the position uncertainty
                 the drone will have when it is back where it can recognise the exit:
                     sigma_exit = sigma_fix + r_hat * (s - s_fix + d(p, wp) + L_ret(wp))
                 If sigma_exit exceeds the relocalisation capture radius (it would no longer
                 recognise the way out), or the battery would not last -> turn back now.

Frames: (north, east) metres in PX4 local NED, as in core.py. The VIO/PX4 position is the
"raw" frame; this module keeps a correction `corr` so that  p = p_raw + corr.
Setpoints are returned in the raw frame (what PX4 understands).
"""
import heapq
import math
import numpy as np

from lostlink_planner.core import OccupancyMap2D, EgoStylePlanner, carrot, wrap, distance_field


# ----------------------------------------------------------------------------- map
class DriftAwareMap(OccupancyMap2D):
    def __init__(self, window_m=8.0, keep_near_m=1.5, **kw):
        super().__init__(**kw)
        self.window, self.keep_near = float(window_m), float(keep_near_m)
        self.last_s = np.zeros(self.hits.shape, dtype=np.float32)
        self.s, self.pos = 0.0, None
        self._s_at_update = 0.0
        self.anchor_hits = np.zeros(self.hits.shape, dtype=np.uint16)
        self.anchor_open = True
        self.anchor_dist = None

    def set_state(self, s_denied, pos_ne):
        self.s, self.pos = float(s_denied), np.asarray(pos_ne[:2], float)
        if self.s - self._s_at_update > 0.25:
            self._dirty = True

    def _cells(self, pts_ned):
        alt = -pts_ned[:, 2]
        p = pts_ned[(alt > self.alt_band[0]) & (alt < self.alt_band[1])]
        i = ((p[:, 0] - self.n0) / self.res).astype(int)
        j = ((p[:, 1] - self.e0) / self.res).astype(int)
        k = (i >= 0) & (i < self.nn) & (j >= 0) & (j < self.ne)
        return i[k], j[k]

    def insert(self, pts_ned):
        if len(pts_ned) == 0:
            return
        i, j = self._cells(pts_ned)
        stale = (self.s - self.last_s[i, j]) > self.window
        self.hits[i[stale], j[stale]] = 0          # old evidence from a drifted frame: start again
        np.add.at(self.hits, (i, j), 1)
        self.last_s[i, j] = self.s
        if self.anchor_open:
            np.add.at(self.anchor_hits, (i, j), 1)
        self._dirty = True

    def close_anchor(self):
        if self.anchor_open:
            self.anchor_open = False
            self.anchor_dist = distance_field(self.anchor_hits >= self.min_hits, self.res, max_dist=1.0)
            # how far every cell is from anything in the anchor map (up to 5 m): used to tell whether
            # what the camera sees now is a part of the building that the anchor map covers at all
            self.anchor_reach = distance_field(self.anchor_hits >= self.min_hits, self.res, max_dist=5.0)

    def shift(self, dn, de):
        """Move the (non-anchor) map by (dn, de) metres - after a relocalisation."""
        di, dj = int(round(dn / self.res)), int(round(de / self.res))
        self.hits[(self.s - self.last_s) > self.window] = 0     # forgotten cells are not carried along
        for name, fill in (("hits", 0), ("last_s", -1e9)):
            a = getattr(self, name); b = np.full_like(a, fill)
            H, W = a.shape
            b[max(di, 0):H + min(di, 0), max(dj, 0):W + min(dj, 0)] = a[max(-di, 0):H + min(-di, 0), max(-dj, 0):W + min(-dj, 0)]
            setattr(self, name, b)
        self._dirty = True

    def recent_cells(self, span_m):
        m = (self.hits >= self.min_hits) & ((self.s - self.last_s) <= span_m)
        return np.nonzero(m)

    def _update(self):
        if not self._dirty:
            return
        fresh = (self.s - self.last_s) <= self.window
        if self.pos is not None and self.keep_near > 0:          # never forget what is right next to us
            ci, cj = self.to_cell(*self.pos); r = int(math.ceil(self.keep_near / self.res))
            i0, i1, j0, j1 = max(ci - r, 0), min(ci + r + 1, self.nn), max(cj - r, 0), min(cj + r + 1, self.ne)
            if i0 < i1 and j0 < j1:                              # (but only moderately old cells)
                fresh[i0:i1, j0:j1] |= (self.s - self.last_s[i0:i1, j0:j1]) <= 2 * self.window
        self._occ = (self.hits >= self.min_hits) & fresh
        self._dist = distance_field(self._occ, self.res)
        self._infl = self._dist < self.inflate_r
        self._s_at_update = self.s
        self._dirty = False


# ----------------------------------------------------------------------------- relocalisation
def match_to_anchor(anchor_dist, ci, cj, radius_m, res, sig_d=0.3, min_cells=40):
    """Translation that best aligns recent cells (ci, cj) with the anchor map.

    Returns (dn, de, score, score_at_zero, unique) or None. Correlative search on a grid:
    score = mean(exp(-(d/sig_d)^2)), d = distance (m) from a shifted cell to the nearest anchor wall.
    """
    if anchor_dist is None or len(ci) < min_cells:
        return None
    H, W = anchor_dist.shape
    if len(ci) > 800:
        sel = np.linspace(0, len(ci) - 1, 800).astype(int); ci, cj = ci[sel], cj[sel]
    R = int(math.ceil(radius_m / res))

    def score(di, dj):
        i, j = ci + di, cj + dj
        ok = (i >= 0) & (i < H) & (j >= 0) & (j < W)
        d = np.full(len(i), 1.0, dtype=np.float32)
        d[ok] = anchor_dist[i[ok], j[ok]]
        return float(np.mean(np.exp(-(d / sig_d) ** 2)))

    coarse = {}
    for di in range(-R, R + 1, 2):
        for dj in range(-R, R + 1, 2):
            if di * di + dj * dj <= R * R:
                coarse[(di, dj)] = score(di, dj)
    (bi, bj) = max(coarse, key=coarse.get)
    best, bs = (bi, bj), coarse[(bi, bj)]
    for di in range(bi - 1, bi + 2):
        for dj in range(bj - 1, bj + 2):
            s = score(di, dj)
            if s > bs:
                best, bs = (di, dj), s
    far = [v for (di, dj), v in coarse.items() if math.hypot(di - best[0], dj - best[1]) * res > 1.0]
    unique = (not far) or (max(far) < bs - 0.08)
    return best[0] * res, best[1] * res, bs, score(0, 0), unique


# ----------------------------------------------------------------------------- homing graph
class Breadcrumbs:
    """Graph of the flown route. Node 0 = takeoff point = home. Anchored crumbs are never shifted."""

    def __init__(self, spacing=1.0, shortcut_r=2.5):
        self.spacing, self.shortcut_r = spacing, shortcut_r
        self.pts, self.adj, self.anchored = [], [], []

    def add(self, p, anchored, grid=None):
        p = np.asarray(p[:2], float)
        if self.pts and np.linalg.norm(p - self.pts[-1]) < self.spacing:
            return
        k = len(self.pts)
        self.pts.append(p.copy()); self.adj.append({}); self.anchored.append(bool(anchored))
        if k == 0:
            return
        self._link(k, k - 1)
        if k > 2:
            P = np.array(self.pts[:k - 1]); d = np.linalg.norm(P - p, axis=1)
            for m in np.nonzero(d < self.shortcut_r)[0]:
                if grid is None or grid.line_free(self.pts[m], p, use_inflated=False):
                    self._link(k, int(m))

    def _link(self, a, b):
        L = float(np.linalg.norm(self.pts[a] - self.pts[b]))
        self.adj[a][b] = L; self.adj[b][a] = L

    def shift_unanchored(self, dn, de):
        for k, a in enumerate(self.anchored):
            if not a:
                self.pts[k] = self.pts[k] + np.array([dn, de])

    @property
    def exit_node(self):
        """Last crumb laid while the position was still trusted (normally just inside the door)."""
        idx = [k for k, a in enumerate(self.anchored) if a]
        return idx[-1] if idx else 0

    def route(self, p, target=0, grid=None, entry_r=4.0):
        """(points from p to crumb `target`, length m). Multi-source Dijkstra: the route may start
        at any crumb within entry_r that is in plain view (line free in the map), not only the nearest."""
        p = np.asarray(p[:2], float)
        if not self.pts:
            return [p, np.zeros(2)], float(np.linalg.norm(p))
        P = np.array(self.pts)
        dp = np.linalg.norm(P - p, axis=1)
        starts = [int(k) for k in np.nonzero(dp < entry_r)[0]
                  if grid is None or grid.line_free(p, self.pts[k], use_inflated=False)]
        if not starts:
            starts = [int(np.argmin(dp))]
        dist, prev, pq = {}, {}, []
        for k in starts:
            dist[k], prev[k] = float(dp[k]), None
            heapq.heappush(pq, (float(dp[k]), k))
        while pq:
            d, u = heapq.heappop(pq)
            if u == target:
                break
            if d > dist[u]:
                continue
            for v, w in self.adj[u].items():
                nd = d + w
                if nd < dist.get(v, 1e18):
                    dist[v], prev[v] = nd, u
                    heapq.heappush(pq, (nd, v))
        if target not in dist:
            return [p, self.pts[target]], float(np.linalg.norm(p - self.pts[target]))
        path, u = [], target
        while u is not None:
            path.append(self.pts[u]); u = prev[u]
        path.reverse()
        return [p] + path, dist[target]


def point_along(route, s):
    R = np.asarray(route, float)
    for a, b in zip(R[:-1], R[1:]):
        L = float(np.linalg.norm(b - a))
        if s <= L:
            return a + (b - a) * (s / max(L, 1e-9))
        s -= L
    return R[-1]


# ----------------------------------------------------------------------------- decision
class ReturnDecision:
    """r_hat: assumed VIO drift rate (fraction of distance). capture_m: largest position error at
    which the drone can still recognise the exit (relocalisation search radius, conservative).
    budget_m: acceptable landing error. kappa: detour factor for the unknown way back from a
    waypoint not yet visited. Battery as usable flight time."""

    def __init__(self, r_hat=0.05, capture_m=3.0, budget_m=2.0, sigma_fix=0.2, kappa=1.25,
                 endurance_s=1200.0, reserve_s=60.0, v_nom=1.0):
        self.r_hat, self.capture, self.budget, self.kappa = r_hat, capture_m, budget_m, kappa
        self.endurance, self.reserve, self.v_nom = endurance_s, reserve_s, v_nom
        self.sigma_fix, self.s_fix = sigma_fix, 0.0
        self.r_prior, self._num, self._den = r_hat, 0.0, 0.0
        self.estimator, self.r_floor, self.base_min, self._hist = "increments", 0.02, 15.0, []
        self.lookback_leg = None

    def fixed(self, s, sigma, shift_m=None, corr=None):
        """A relocalisation at odometer s. Learn the drift rate online.

        estimator "increments" (Stage 4 v1): sum of each shift / distance since the previous fix,
            minus 0.25 m quantisation per fix; never below the prior. Biased LOW when the drone
            relocalises often (every small correction is rounded away).
        estimator "baseline" (Stage 5): compare the total correction now with the total correction
            at an earlier fix at least `base_min` metres back (longest such baseline, up to 60 m):
                r = (|corr_now - corr_then| + 0.25) / distance flown in between
            The +0.25 m (one map cell) makes it a conservative (upper) estimate. It can go down as
            well as up, but never below r_floor."""
        if self.estimator == "baseline" and corr is not None:
            c = np.asarray(corr, float)
            best = None
            for s_i, c_i in self._hist:
                ds = s - s_i
                if self.base_min <= ds <= 60.0 and (best is None or ds > best[0]):
                    best = (ds, c_i)
            if best is not None:
                self.r_hat = max(self.r_floor, (float(np.linalg.norm(c - best[1])) + 0.25) / best[0])
            self._hist.append((s, c.copy()))
        elif shift_m is not None and s - self.s_fix > 0.5:
            self._num += max(0.0, shift_m - 0.25); self._den += s - self.s_fix
            if self._den >= 5.0:
                self.r_hat = max(self.r_prior, self._num / self._den)
        self.sigma_fix, self.s_fix = sigma, s

    def landing_sigma(self, L_exit_home, sigma_at_exit=0.3):
        # Stage 5: with the look-back final leg the drone keeps relocalising against the facade,
        # so only the distance between two fixes (~4 m) counts, not the whole way home
        L = min(L_exit_home, self.lookback_leg) if self.lookback_leg else L_exit_home
        return sigma_at_exit + self.r_hat * L

    def sigma(self, s):
        return self.sigma_fix + self.r_hat * max(0.0, s - self.s_fix)

    def evaluate(self, s, p, wp, exit_pt, L_exit, L_exit_home, t_flight):
        """Go to `wp`? -> (go, reason, predicted sigma back at the exit)."""
        p, wp, exit_pt = np.asarray(p[:2], float), np.asarray(wp, float), np.asarray(exit_pt, float)
        leg = float(np.linalg.norm(wp - p))
        L_ret = min(leg + L_exit, self.kappa * float(np.linalg.norm(wp - exit_pt)))
        sig_exit = self.sigma(s + leg + L_ret)
        t_need = (leg + L_ret + L_exit_home) / self.v_nom
        if t_flight + t_need > self.endurance - self.reserve:
            return False, "battery", sig_exit
        if sig_exit > self.capture:
            return False, "drift", sig_exit
        if self.landing_sigma(L_exit_home) > self.budget:           # cannot land within budget even from the exit
            return False, "drift(landing)", sig_exit
        return True, "ok", sig_exit


# ----------------------------------------------------------------------------- mission
class LostLinkMission:
    """Waypoint mission with selectable Stage-4 features (ablations).

    policy      map            relocalise   return decision   way home
    rtl_link    plain          no           link lost > dll_s straight
    stage3      plain          no           no (full mission) straight
    map         drift-aware    no           no                breadcrumbs
    map_reloc   drift-aware    yes          no                breadcrumbs
    stage4      drift-aware    yes          yes               breadcrumbs
    """
    POLICIES = ("rtl_link", "stage3", "map", "map_reloc", "stage4")

    def __init__(self, waypoints_ne, policy="stage4", r_hat=0.05, e_tol=0.4, capture_m=3.0,
                 budget_m=2.0, sigma_anchor=0.4, endurance_s=1200.0, dll_s=10.0, reach_r=0.6,
                 lookahead=1.5, yaw_first_deg=40.0, look_back=True, map_kwargs=None, rate_estimator="baseline"):
        assert policy in self.POLICIES, policy
        self.look_back = look_back
        self.policy = policy
        self.forget = policy in ("map", "map_reloc", "stage4")
        self.reloc = policy in ("map_reloc", "stage4")
        self.decide = policy == "stage4"
        mk = dict(map_kwargs or {})
        self.grid = DriftAwareMap(window_m=max(3.0, e_tol / max(r_hat, 1e-3)), **mk) if self.forget else OccupancyMap2D(**mk)
        self.planner = EgoStylePlanner(self.grid)
        self.crumbs = Breadcrumbs()
        self.dec = ReturnDecision(r_hat=r_hat, capture_m=capture_m, budget_m=budget_m, endurance_s=endurance_s)
        self.dec.estimator = rate_estimator
        self.dec.lookback_leg = 4.0 if (look_back and self.reloc and rate_estimator == "baseline") else None
        self.r_alarm, self.alarm_after = 3 * r_hat, 5           # Stage 5: recognition alarm (None/0 to disable)
        self.skipped = []
        self.adaptive_lookahead = True                         # Stage 5
        # Stage 5b: recovery when stuck on the way home (map policies only). If the drone has moved
        # less than stuck_move_m in stuck_s seconds: first re-plan (drop the path, switch between
        # direct planning and breadcrumbs, aim further along the route); if still stuck, clear the
        # drifted map and rebuild it from what the camera sees now (four headings, 90 deg apart).
        self.recover = self.forget
        self.stuck_s, self.stuck_move_m, self.rescan_dwell_s = 12.0, 1.0, 2.5
        self._mv_p, self._mv_t, self._stuck_n, self._rescan_t0, self._rescan_yaw0, self._push_far = None, 0.0, 0, None, 0.0, False
        self.n_map_clears, self.n_stuck = 0, 0
        self.reloc_gate, self.n_gated = 2.5, 0                 # Stage 5b: innovation gate (x sigma)
        self.r_hat, self.sigma_anchor, self.e_tol = r_hat, sigma_anchor, e_tol
        self.wps = [np.asarray(w, float) for w in waypoints_ne]
        self.k = 0
        self.reach_r, self.lookahead, self.yaw_first, self.dll_s = reach_r, lookahead, math.radians(yaw_first_deg), dll_s
        self.state, self.return_reason, self.done = "mission", None, False
        self.path, self.mode, self._goal_used = None, "init", None
        self.replans = 0
        self.corr = np.zeros(2)
        self.s, self.denied, self.link_lost_t = 0.0, False, 0.0
        self._prev_raw = None
        self.p = np.zeros(2)
        self._route, self._route_L, self._route_t = None, 0.0, -1e9
        self._exit_L, self._exit_home_L = 0.0, 0.0
        self._match_t, self.n_reloc, self.reloc_log = -1e9, 0, []
        self._scan_t0 = None
        self.phase, self.use_crumbs, self._best_d, self._best_t = "to_exit", False, 1e9, 0.0
        self.sigma_pred = 0.0
        self.events = []

    # ---- inputs -------------------------------------------------------------
    def update_state(self, t, pos_raw, denied, link_ok, dt):
        raw = np.asarray(pos_raw[:2], float)
        if self._prev_raw is not None and denied:
            self.s += float(np.linalg.norm(raw - self._prev_raw))
        if denied and not self.denied:
            self.events.append((round(t, 1), "GPS lost"))
        self.denied, self._prev_raw = denied, raw.copy()
        self.p = raw + self.corr
        if self.forget:
            self.grid.set_state(self.s, self.p)
            if self.grid.anchor_open and denied and self.dec.sigma(self.s) > self.sigma_anchor:
                self.grid.close_anchor()
                self.events.append((round(t, 1), f"anchor map closed after {self.s:.1f} m without GPS"))
            self.crumbs.add(self.p, anchored=self.grid.anchor_open, grid=self.grid)
        self.link_lost_t = 0.0 if link_ok else self.link_lost_t + dt

    def insert_points(self, pts_ned_raw):
        pts = np.array(pts_ned_raw, float, copy=True)
        if len(pts):
            pts[:, 0] += self.corr[0]; pts[:, 1] += self.corr[1]
        self.grid.insert(pts)

    # ---- relocalisation -------------------------------------------------------
    def _anchor_in_view(self, rng_m=10.0, half_fov=0.55, min_cells=60):
        """Are there enough anchor-map walls inside the camera's field of view (within 10 m)?
        Insensitive to a few metres of drift, unlike comparing cell positions directly."""
        if self._anchor_ne is None:
            ai, aj = np.nonzero(self.grid.anchor_hits >= self.grid.min_hits)
            self._anchor_ne = np.stack([self.grid.n0 + (ai + .5) * self.grid.res, self.grid.e0 + (aj + .5) * self.grid.res], 1)
        rel = self._anchor_ne - self.p
        d = np.hypot(rel[:, 0], rel[:, 1])
        b = np.arctan2(rel[:, 1], rel[:, 0])
        vis = (d < rng_m) & (np.abs((b - self._yaw + np.pi) % (2 * np.pi) - np.pi) < half_fov)
        return int(vis.sum()) >= min_cells

    _anchor_ne = None

    def _try_reloc(self, t):
        if not (self.reloc and self.denied and not self.grid.anchor_open) or t - self._match_t < 1.0:
            return False
        self._match_t = t
        sig = self.dec.sigma(self.s)
        ci, cj = self.grid.recent_cells(span_m=4.0)
        # only where the anchor map has something to match against
        ai, aj = np.nonzero(self.grid.anchor_hits >= self.grid.min_hits)
        pi, pj = self.grid.to_cell(*self.p)
        if np.sum(np.hypot(ai - pi, aj - pj) * self.grid.res < 12.0) < 40:
            return False
        # search radius from the expected error; after 3 failed matches in a row (in view of the anchor)
        # the error is probably bigger than assumed -> search the full 5 m (Stage 5)
        radius = 5.0 if self._fails >= 3 else min(3 * sig + 0.5, 5.0)
        r = match_to_anchor(self.grid.anchor_dist, ci, cj, radius_m=radius, res=self.grid.res)
        if r is None:
            return False
        dn, de, sc, s0, unique = r
        good = unique and sc >= 0.55 and (sc - s0 >= 0.08 or math.hypot(dn, de) < 0.3)
        # Stage 5b: innovation gate. A correction much larger than the error we expect is more likely
        # a look-alike (a corridor wall matched onto the opposite wall, a mirror-image room) than drift.
        # The recognition alarm raises the expected error, which then lets large corrections through.
        gated = bool(good and self.reloc_gate and math.hypot(dn, de) > self.reloc_gate * sig + 0.5)
        if gated:                                        # a gated match also counts towards the alarm
            good = False
            self.n_gated += 1
        self.reloc_log.append((round(t, 1), round(dn, 2), round(de, 2), round(sc, 3), round(s0, 3), bool(unique), bool(good)))
        if not good:
            # count it as a failure only if most of what we see now lies within the search radius
            # of anchor-map walls (i.e. the anchor really covers this view)
            # a corridor gives a good but ambiguous fit (not unique) - that is geometry, not drift.
            # Only a poor fit everywhere in the search area counts as evidence of large drift.
            if (sc < 0.35 or gated) and self._anchor_in_view():
                self._fails += 1
            # Stage 5 "recognition alarm": the anchor map is in view but repeated matches fail ->
            # the drift is probably much larger than assumed. Be pessimistic from now on.
            if self._fails == self.alarm_after and self.dec.r_hat < self.r_alarm:
                self.dec.r_hat = self.r_alarm
                self.grid.window = max(3.0, self.e_tol / self.dec.r_hat)
                self.events.append((round(t, 1), f"recognition alarm: {self._fails} failed matches -> assume {100 * self.r_alarm:.0f} % drift"))
            return False
        self._fails = 0
        if math.hypot(dn, de) >= 0.25:
            self.corr += np.array([dn, de]); self.p = self.p + np.array([dn, de])
            self.grid.shift(dn, de); self.crumbs.shift_unanchored(dn, de)
            self.path, self._goal_used, self._route_t = None, None, -1e9
        r_before = self.dec.r_hat
        self.dec.fixed(self.s, 0.3, shift_m=math.hypot(dn, de), corr=self.corr)
        if abs(self.dec.r_hat - r_before) > 0.002:
            self.grid.window = max(3.0, self.e_tol / self.dec.r_hat)
            self.events.append((round(t, 1), f"drift rate re-estimated: {100 * self.dec.r_hat:.1f} %"))
        self.n_reloc += 1
        self.events.append((round(t, 1), f"relocalised: shift {math.hypot(dn, de):.2f} m (score {sc:.2f})"))
        return True

    # ---- driving -------------------------------------------------------------
    def _drive(self, p, yaw, goal, turn_first=True):
        goal = np.asarray(goal, float)
        need = self.path is None or self._goal_used is None or np.linalg.norm(goal - self._goal_used) > 0.75
        if not need:
            for a, b in zip(self.path[:-1], self.path[1:]):
                if not self.grid.line_free(a, b):
                    need = True; break
        if not need and np.linalg.norm(self.path[0] - p) > 2.0:
            need = True
        if not need and np.linalg.norm(self.path[-1] - p) < 0.3 and np.linalg.norm(goal - p) > 0.75:
            need = True                                  # Stage 5b: path ends here, goal is elsewhere
        if need:
            if self._t < self._nopath_until:                 # no path a moment ago: keep looking around
                return p[0], p[1], wrap(yaw + 0.5), "no_path"
            path, mode = self.planner.plan(p, goal)
            self.replans += 1
            if path is None:
                self.path, self._goal_used, self._nopath_until = None, None, self._t + 1.0
                return p[0], p[1], wrap(yaw + 0.5), "no_path"
            self.path, self.mode, self._goal_used = path, mode, goal
        # Stage 5: adaptive look-ahead - close to walls (door frames) chase a nearer point on the path,
        # otherwise the drone cuts the inside of tight corners
        la = self.lookahead
        if self.adaptive_lookahead:
            la = float(np.clip(self.grid.dist_at(*p) - 0.3, 0.5, self.lookahead))
        c = carrot(self.path, p, la)
        d = c - p
        if np.linalg.norm(d) < 0.2:
            return c[0], c[1], yaw, self.mode
        yaw_sp = math.atan2(d[1], d[0])
        if turn_first and abs(wrap(yaw_sp - yaw)) > self.yaw_first:
            return p[0], p[1], yaw_sp, self.mode + "+yaw"
        return c[0], c[1], yaw_sp, self.mode

    def _out(self, n, e, ys, info):
        """Convert a setpoint from the corrected frame back to PX4's (raw VIO) frame."""
        return n - self.corr[0], e - self.corr[1], ys, info

    def _start_return(self, t, reason):
        if self.state == "mission":
            self.state, self.return_reason, self.path = "return", reason, None
            self._best_d, self._best_t, self._route_t = 1e9, t, -1e9
            self.events.append((round(t, 1), f"return: {reason} (after wp {self.k}/{len(self.wps)})"))

    _t, _nopath_until, _fails, _yaw = 0.0, -1.0, 0, 0.0
    wp_best, wp_best_t, skip_after = 1e9, None, 15.0

    def step(self, t, yaw, t_flight):
        self._t, self._yaw = t, yaw
        if self.wp_best_t is None:          # waypoint-progress timer starts with the mission, not the node
            self.wp_best_t = t
        p = self.p
        info = {"state": self.state, "s": round(self.s, 2), "sigma": round(self.dec.sigma(self.s), 2), "corr": self.corr.round(2).tolist()}
        if self.done:
            return self._out(p[0], p[1], yaw, dict(info, mode="done"))
        self._try_reloc(t)
        p = self.p

        if self.forget and t - self._route_t > 1.0:
            ex = self.crumbs.exit_node
            self._route, self._route_L = self.crumbs.route(p, ex if self.phase == "to_exit" else 0, self.grid)
            ex_pt = self.crumbs.pts[ex] if self.crumbs.pts else np.zeros(2)
            L_crumb = self.crumbs.route(p, ex, self.grid)[1] if self.crumbs.pts else 0.0
            self._exit_L = min(L_crumb, self.dec.kappa * float(np.linalg.norm(p - ex_pt)))
            self._exit_home_L = self.crumbs.route(ex_pt, 0)[1] if self.crumbs.pts else 0.0
            self._route_t = t

        if self.state == "mission":
            if self.k < len(self.wps):
                d = float(np.linalg.norm(p - self.wps[self.k]))
                if d < self.wp_best - 0.5:
                    self.wp_best, self.wp_best_t = d, t
                if d < self.reach_r:
                    self.events.append((round(t, 1), f"reached wp {self.k + 1}"))
                    self.k += 1; self.path = None; self.wp_best, self.wp_best_t = 1e9, t
                elif self.skip_after and t - self.wp_best_t > self.skip_after:
                    # Stage 5: no progress towards this waypoint (e.g. it sits inside an obstacle in
                    # the drone's own map) -> give it up and continue the search
                    self.events.append((round(t, 1), f"skipped wp {self.k + 1} (no progress for {self.skip_after:.0f} s)"))
                    self.skipped.append(self.k)
                    self.k += 1; self.path = None; self.wp_best, self.wp_best_t = 1e9, t
            if self.k >= len(self.wps):
                self._start_return(t, "mission complete")
            elif self.policy == "rtl_link" and self.link_lost_t > self.dll_s:
                self._start_return(t, "link lost")
            elif self.decide and self.denied and not self.grid.anchor_open:
                ex = self.crumbs.pts[self.crumbs.exit_node]
                go, why, sig = self.dec.evaluate(self.s, p, self.wps[self.k], ex, self._exit_L, self._exit_home_L, t_flight)
                self.sigma_pred = sig
                if not go:
                    self._start_return(t, why)
            if self.state == "mission":
                n, e, ys, mode = self._drive(p, yaw, self.wps[self.k])
                return self._out(n, e, ys, dict(info, mode=mode, goal=self.k))

        # ---------------- return
        if np.linalg.norm(p) < self.reach_r:
            self.done, self.state = True, "done"
            self.events.append((round(t, 1), "home (own estimate)"))
            return self._out(p[0], p[1], yaw, dict(info, mode="done"))
        if self.recover:
            out = self._recover(t, p, yaw, info)
            if out is not None:
                return out
        if not self.forget:
            n, e, ys, mode = self._drive(p, yaw, np.zeros(2))
            return self._out(n, e, ys, dict(info, mode=mode, goal=len(self.wps)))
        ex = self.crumbs.pts[self.crumbs.exit_node]
        d_ex = float(np.linalg.norm(p - ex))
        # near the exit and not yet relocalised recently: stop and look around once
        if self.reloc and self._scan_t0 is None and d_ex < 3.0 and self.dec.s_fix < self.s - 6.0:
            self._scan_t0 = t
            self.events.append((round(t, 1), "near exit: scanning to relocalise"))
        if self._scan_t0 is not None and self._scan_t0 > 0:
            if self.dec.s_fix >= self.s - 0.5 or t - self._scan_t0 > 15.0:      # matched, or give up
                self._scan_t0 = -1.0
                self._route_t = -1e9
            else:
                return self._out(p[0], p[1], wrap(yaw + 0.6), dict(info, mode="scan", goal=len(self.wps)))
        if self.phase == "to_exit" and (d_ex < 1.5 or self.crumbs.exit_node == 0):
            self.phase, self._route_t = "home", -1e9
            self.events.append((round(t, 1), "at exit"))
        # progress monitor: plan straight to the target through the map; if that makes no
        # progress for 20 s, fall back to following the breadcrumbs
        target = ex if self.phase == "to_exit" else np.zeros(2)
        dt_ = float(np.linalg.norm(p - target))
        if dt_ < self._best_d - 1.0:
            self._best_d, self._best_t = dt_, t
        if not self.use_crumbs and t - self._best_t > 20.0:
            self.use_crumbs = True
            self.events.append((round(t, 1), "no progress: following breadcrumbs"))
        if self.use_crumbs and self._route is not None and len(self._route) > 1:
            goal = self._subgoal(p)
        else:
            goal = target
        if self.reloc and self.look_back and self.phase == "home" and self._known_free(p, target):
            # last leg over ground flown with trusted position: fly backwards, camera on the building
            # we mapped with GPS, so relocalisation keeps working until touchdown
            n, e, _, mode = self._drive(p, yaw, goal, turn_first=False)
            ys = math.atan2(ex[1] - p[1], ex[0] - p[0])
            return self._out(n, e, ys, dict(info, mode=mode + "+lookback", goal=len(self.wps)))
        n, e, ys, mode = self._drive(p, yaw, goal)
        return self._out(n, e, ys, dict(info, mode=mode, goal=len(self.wps)))

    def _recover(self, t, p, yaw, info):
        """Stage 5b stuck watchdog for the return. Returns a setpoint while re-mapping, else None."""
        if self._rescan_t0 is not None:                       # re-mapping: hold, look in four directions
            k = int((t - self._rescan_t0) / self.rescan_dwell_s)
            if k < 4:
                return self._out(p[0], p[1], wrap(self._rescan_yaw0 + k * math.pi / 2),
                                 dict(info, mode="rescan", goal=len(self.wps)))
            self._rescan_t0, self._mv_p, self._mv_t = None, p.copy(), t
            self.path, self._route_t = None, -1e9
            return None
        if self._scan_t0 is not None and self._scan_t0 > 0:  # the planned scan near the exit is not "stuck"
            self._mv_p, self._mv_t = p.copy(), t
            return None
        if self._mv_p is None or np.linalg.norm(p - self._mv_p) > self.stuck_move_m:
            if self._mv_p is not None and np.linalg.norm(p - self._mv_p) > 2 * self.stuck_move_m:
                self._stuck_n, self._push_far = 0, False         # really moving again
            self._mv_p, self._mv_t = p.copy(), t
            return None
        if t - self._mv_t < self.stuck_s:
            return None
        self._stuck_n += 1; self.n_stuck += 1
        self._mv_p, self._mv_t = p.copy(), t
        if self._stuck_n % 2 == 1:                            # level 1: re-plan differently
            self.use_crumbs = not self.use_crumbs
            self._push_far = True
            self.path, self._goal_used, self._nopath_until, self._route_t = None, None, -1.0, -1e9
            self._best_d, self._best_t = 1e9, t
            self.events.append((round(t, 1), f"stuck {self.stuck_s:.0f} s: re-plan ({'breadcrumbs' if self.use_crumbs else 'direct'})"))
            return None
        # level 2: the map itself is the problem -> clear it and rebuild from what is visible now
        if hasattr(self.grid, "hits"):
            self.grid.hits[:] = 0; self.grid._dirty = True
        self.n_map_clears += 1
        self.path, self._goal_used, self._nopath_until = None, None, -1.0
        self._rescan_t0, self._rescan_yaw0 = t, yaw
        self.events.append((round(t, 1), "stuck again: map cleared, re-mapping"))
        return self._out(p[0], p[1], yaw, dict(info, mode="rescan", goal=len(self.wps)))

    def _known_free(self, p, target):
        """Straight line p->target runs along anchored crumbs (flown while the position was trusted)."""
        A = np.array([c for c, a in zip(self.crumbs.pts, self.crumbs.anchored) if a])
        if len(A) < 2:
            return False
        for q in np.linspace(p, target, max(2, int(np.linalg.norm(target - p) / 1.0) + 1)):
            if np.min(np.linalg.norm(A - q, axis=1)) > 2.0:
                return False
        return True

    def _subgoal(self, p, ahead=4.0):
        """Farthest point along the breadcrumb route, within `ahead` m, that is in plain view."""
        R = np.asarray(self._route, float)
        if self._push_far:
            return point_along(R, ahead)
        best = R[min(1, len(R) - 1)]
        for s in np.arange(0.5, ahead + 0.01, 0.5):
            q = point_along(R, s)
            if self.grid.line_free(p, q):
                best = q
            else:
                break
        return best
