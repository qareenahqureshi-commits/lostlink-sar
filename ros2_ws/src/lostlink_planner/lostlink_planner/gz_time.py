"""
Time + pose helpers shared by the Stage 3 nodes.

Why: on this setup PX4 stamps messages with WALL-clock microseconds, while Gazebo stamps
sensor data with SIMULATION time, and the sim runs slower than real time (RTF ~0.3).
Gazebo's /world/<w>/clock message carries both (sim + system), so we keep a short history
of (sim, wall) pairs and convert between the two by interpolation.
"""
import bisect, threading
from collections import deque
import numpy as np


class SimWallClock:
    def __init__(self, maxlen=600):
        self.sim, self.wall = deque(maxlen=maxlen), deque(maxlen=maxlen)
        self.lock = threading.Lock()

    def on_clock(self, msg):
        s = msg.sim.sec + msg.sim.nsec * 1e-9
        w = msg.system.sec + msg.system.nsec * 1e-9
        with self.lock:
            if not self.sim or s > self.sim[-1]:
                self.sim.append(s); self.wall.append(w)

    def sim_to_wall(self, s, window_s=10.0):
        """Wall time (s) at which the simulation was at sim time s. None if unknown yet.

        Robust version: real-time factor from a least-squares line over the last `window_s` of
        wall time (the clock topic arrives at a few Hz with jittery wall stamps, so a 2-point
        slope is far too noisy), anchored at the latest sample.
        """
        with self.lock:
            if len(self.sim) < 5:
                return None
            sim = np.fromiter(self.sim, float); wall = np.fromiter(self.wall, float)
        keep = wall > wall[-1] - window_s
        if keep.sum() < 5:
            keep[-5:] = True
        S, W = sim[keep], wall[keep]
        if S[-1] - S[0] < 0.2:
            return None
        k = np.polyfit(S, W, 1)[0]                  # wall seconds per sim second (= 1 / RTF)
        k = float(np.clip(k, 0.5, 20.0))
        return float(W[-1] + (s - S[-1]) * k)


class PoseBuffer:
    """Recent PX4 poses keyed by wall time (s); returns the pose interpolated at any time."""

    def __init__(self, maxlen=400):
        self.t, self.p, self.q = deque(maxlen=maxlen), deque(maxlen=maxlen), deque(maxlen=maxlen)
        self.lock = threading.Lock()

    def add(self, t, pos, q):
        with self.lock:
            if self.t and t <= self.t[-1]:
                return
            self.t.append(t); self.p.append(np.asarray(pos, float)); self.q.append(np.asarray(q, float))

    def at(self, t):
        with self.lock:
            if not self.t:
                return None, None, None
            ts = list(self.t)
            i = bisect.bisect_left(ts, t)
            if i <= 0:
                return self.p[0].copy(), self.q[0].copy(), ts[0] - t
            if i >= len(ts):
                return self.p[-1].copy(), self.q[-1].copy(), t - ts[-1]
            a = (t - ts[i - 1]) / max(ts[i] - ts[i - 1], 1e-9)
            p = (1 - a) * self.p[i - 1] + a * self.p[i]
            q0, q1 = self.q[i - 1], self.q[i]
            if np.dot(q0, q1) < 0:
                q1 = -q1
            q = (1 - a) * q0 + a * q1
            return p, q / np.linalg.norm(q), 0.0


# ---------------------------------------------------------------- frames
R_NED_ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
R_FLU_FRD = np.diag([1.0, -1.0, -1.0])


def quat_to_R(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        s = 2 * np.sqrt(tr + 1); w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2 * np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]); w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2 * np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]); w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s; y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2 * np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]); w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    q = np.array([w, x, y, z]); return q / np.linalg.norm(q)


def enu_flu_to_ned_frd(pos_enu, q_enu_flu_wxyz):
    """Gazebo world pose (ENU, body FLU) -> PX4 convention (NED, body FRD)."""
    p = R_NED_ENU @ np.asarray(pos_enu, float)
    R = R_NED_ENU @ quat_to_R(*q_enu_flu_wxyz) @ R_FLU_FRD
    return p, R_to_quat(R)
