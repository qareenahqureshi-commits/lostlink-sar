#!/usr/bin/env python3
"""
Stage 3 simulated visual-inertial odometry (VIO) for PX4.

Takes the drone's TRUE pose from Gazebo, adds a controllable drift, and sends it to PX4 as
external vision (/fmu/in/vehicle_visual_odometry) - the same interface a real VIO system
(OpenVINS, VINS-Fusion, T265...) uses. PX4's EKF2 fuses it (EKF2_EV_CTRL), so when GPS is
jammed the drone keeps flying on VIO, and its position estimate drifts exactly as much as we set.

Drift model (starts when /lostlink/gps_denied goes True):
    |drift| grows by `drift_pct` % of the distance flown, in a direction that wanders slowly
    (random walk on heading, sigma `drift_turn` rad per metre) - typical of real VIO error growth.

Params: drift_pct (default 2.0), seed, drift_turn, rate_hz (30), model (x500_depth_0), world (sar_building)
"""
import math, threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float32MultiArray
from px4_msgs.msg import VehicleOdometry

from lostlink_planner.gz_time import SimWallClock, enu_flu_to_ned_frd


class SimVIO(Node):
    def __init__(self):
        super().__init__("lostlink_sim_vio")
        for k, v in (("drift_pct", 2.0), ("seed", 0), ("drift_turn", 0.05), ("rate_hz", 30.0),
                     ("model", "x500_depth_0"), ("world", "sar_building"), ("pos_std", 0.05)):
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        self.drift_pct, self.turn, self.model = float(g("drift_pct")), float(g("drift_turn")), g("model")
        self.pos_var = float(g("pos_std")) ** 2
        self.rng = np.random.default_rng(int(g("seed")))
        self.ang = self.rng.uniform(0, 2 * math.pi)
        self.drift = np.zeros(2); self.denied = False
        self.prev = None; self.lock = threading.Lock(); self.latest = None
        self.min_dt = 1.0 / float(g("rate_hz")); self.last_pub_sim = -1.0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub = self.create_publisher(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", qos)
        self.pub_drift = self.create_publisher(Float32MultiArray, "/lostlink/vio_drift", 10)
        self.create_subscription(Bool, "/lostlink/gps_denied", self.on_denied, 10)

        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.clock_pb2 import Clock
        self.clock = SimWallClock()
        self.gz = GzNode()
        w = g("world")
        self.gz.subscribe(Clock, f"/world/{w}/clock", self.clock.on_clock)
        self.gz.subscribe(Pose_V, f"/world/{w}/dynamic_pose/info", self.on_pose)
        self.get_logger().info(f"sim VIO: drift {self.drift_pct}% of distance after GPS denial, seed {g('seed')}")

    def on_denied(self, m):
        if m.data and not self.denied:
            self.get_logger().warn("GPS denied -> VIO drift starts accumulating")
        self.denied = bool(m.data)

    def on_pose(self, msg):
        s = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        if s - self.last_pub_sim < self.min_dt:
            return
        pose = next((p for p in msg.pose if p.name == self.model), None)
        if pose is None:
            return
        wall = self.clock.sim_to_wall(s)
        if wall is None:
            return
        self.last_pub_sim = s
        o = pose.orientation
        p_ned, q_ned = enu_flu_to_ned_frd([pose.position.x, pose.position.y, pose.position.z], [o.w, o.x, o.y, o.z])
        # true velocity by finite difference (sim time)
        vel = np.zeros(3)
        if self.prev is not None and s > self.prev[0]:
            vel = (p_ned - self.prev[1]) / (s - self.prev[0])
            if self.denied:                                   # grow drift with distance flown
                step = float(np.linalg.norm(p_ned[:2] - self.prev[1][:2]))
                self.ang += self.rng.normal(0, self.turn * max(step, 1e-3) ** 0.5)
                self.drift += self.drift_pct / 100.0 * step * np.array([math.cos(self.ang), math.sin(self.ang)])
        self.prev = (s, p_ned.copy())

        m = VehicleOdometry()
        m.timestamp = m.timestamp_sample = int(wall * 1e6)
        m.pose_frame = VehicleOdometry.POSE_FRAME_NED
        m.position = [float(p_ned[0] + self.drift[0]), float(p_ned[1] + self.drift[1]), float(p_ned[2])]
        m.q = [float(v) for v in q_ned]
        m.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        m.velocity = [float(v) for v in vel]
        m.angular_velocity = [float("nan")] * 3
        m.position_variance = [self.pos_var] * 3
        m.orientation_variance = [0.01] * 3
        m.velocity_variance = [0.01] * 3
        m.quality = 100
        self.pub.publish(m)
        d = Float32MultiArray(); d.data = [float(self.drift[0]), float(self.drift[1]), float(np.linalg.norm(self.drift))]
        self.pub_drift.publish(d)


def main():
    rclpy.init()
    n = SimVIO()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
