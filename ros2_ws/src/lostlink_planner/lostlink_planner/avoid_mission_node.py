#!/usr/bin/env python3
"""
Stage 2 ROS2 node: fly the Stage 1 mission in PX4 OFFBOARD mode with depth-camera obstacle avoidance.

Inputs
  /fmu/out/vehicle_odometry           px4_msgs/VehicleOdometry   (pose, NED)
  /fmu/out/vehicle_status_v4          px4_msgs/VehicleStatus     (armed? offboard?)
  Gazebo /depth_camera                gz.msgs.Image R_FLOAT32    (read directly via gz-transport)
Outputs
  /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint, /fmu/in/vehicle_command

Sequence: stream setpoints -> OFFBOARD + ARM -> take off to `alt` -> waypoints (+avoidance) -> home -> LAND.

Run:
  ros2 run lostlink_planner avoid_mission --ros-args -p plan_file:=<.plan> -p avoid:=true -p log_dir:=<dir>
"""
import csv, json, math, os, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus

from lostlink_planner import core

R_EARTH = 6378137.0


def plan_waypoints_ned(plan_file):
    """NAV_WAYPOINT items of a QGC .plan -> list of (north, east) relative to plannedHomePosition."""
    p = json.load(open(plan_file))
    lat0, lon0 = p["mission"]["plannedHomePosition"][:2]
    out = []
    for it in p["mission"]["items"]:
        if it.get("command") == 16:
            lat, lon = it["params"][4], it["params"][5]
            n = math.radians(lat - lat0) * R_EARTH
            e = math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))
            out.append((n, e))
    return out


class AvoidMission(Node):
    def __init__(self):
        super().__init__("lostlink_avoid_mission")
        self.declare_parameter("plan_file", "")
        self.declare_parameter("alt", 3.0)
        self.declare_parameter("avoid", True)
        self.declare_parameter("lookahead", 1.5)
        self.declare_parameter("depth_topic", "/depth_camera")
        self.declare_parameter("planar_depth", True)
        self.declare_parameter("status_topic", "/fmu/out/vehicle_status_v4")
        self.declare_parameter("log_dir", "/tmp/lostlink")
        gp = lambda k: self.get_parameter(k).value
        self.alt, self.avoid = float(gp("alt")), bool(gp("avoid"))
        self.log_dir = gp("log_dir"); os.makedirs(self.log_dir, exist_ok=True)

        wps = plan_waypoints_ned(gp("plan_file"))
        self.get_logger().info(f"{len(wps)} waypoints (N,E): {[tuple(round(v, 1) for v in w) for w in wps]}")
        self.mf = core.MissionFollower(wps, alt=self.alt, avoid=self.avoid, lookahead=float(gp("lookahead")))
        self.proj = core.DepthProjector(stride=8, planar=bool(gp("planar_depth")))
        self.lock = threading.Lock()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.on_odom, qos)
        self.create_subscription(VehicleStatus, gp("status_topic"), self.on_status, qos)

        self.pos = None; self.q = None; self.yaw = 0.0
        self.armed = False; self.offboard = False
        self.state, self.t_state, self.tick = "WAIT_ODOM", time.time(), 0
        self.sp = (0.0, 0.0, -self.alt, 0.0); self.info = {"mode": "-"}
        self.depth_frames, self.last_depth_t = 0, 0.0

        self._start_depth(gp("depth_topic"))
        f = open(os.path.join(self.log_dir, "planner_log.csv"), "w", newline="")
        self.csv_f, self.csv = f, csv.writer(f)
        self.csv.writerow(["t", "state", "n", "e", "alt", "yaw", "sp_n", "sp_e", "sp_yaw", "mode", "goal_idx",
                           "occupied_cells", "depth_frames"])
        self.t0 = time.time()
        self.create_timer(0.05, self.loop)       # 20 Hz control
        self.get_logger().info(f"avoidance={'ON' if self.avoid else 'OFF'}, logging to {self.log_dir}")

    # ------------------------------------------------------------------ inputs
    def _start_depth(self, topic):
        try:
            from gz.transport13 import Node as GzNode
            from gz.msgs10.image_pb2 import Image
        except ImportError as e:
            self.get_logger().error(f"gz-transport python bindings missing: {e}"); return
        self.gz = GzNode()
        ok = self.gz.subscribe(Image, topic, self.on_depth)
        self.get_logger().info(f"depth subscription {topic}: {'ok' if ok else 'FAILED'}")

    def on_depth(self, msg):
        now = time.time()
        if now - self.last_depth_t < 0.1 or self.pos is None:     # 10 Hz is plenty
            return
        self.last_depth_t = now
        img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        with self.lock:
            pos, q = self.pos.copy(), list(self.q)
        pts = self.proj.to_world(img, pos, q)
        with self.lock:
            self.mf.grid.insert(pts)
        self.depth_frames += 1

    def on_odom(self, m):
        with self.lock:
            self.pos = np.array([m.position[0], m.position[1], m.position[2]], float)
            self.q = [m.q[0], m.q[1], m.q[2], m.q[3]]
        self.yaw = core.yaw_from_quat(self.q)

    def on_status(self, m):
        self.armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.offboard = m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    # ------------------------------------------------------------------ outputs
    def _us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def command(self, cmd, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = self._us(); m.command = cmd; m.param1 = float(p1); m.param2 = float(p2)
        m.target_system = 1; m.target_component = 1; m.source_system = 1; m.source_component = 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def publish_setpoint(self):
        mo = OffboardControlMode(); mo.timestamp = self._us(); mo.position = True
        self.pub_mode.publish(mo)
        n, e, z, yaw = self.sp
        sp = TrajectorySetpoint(); sp.timestamp = self._us()
        sp.position = [float(n), float(e), float(z)]
        sp.velocity = [float("nan")] * 3; sp.acceleration = [float("nan")] * 3
        sp.yaw = float(yaw)
        self.pub_sp.publish(sp)

    def set_state(self, s):
        self.get_logger().info(f"state {self.state} -> {s}")
        self.state, self.t_state = s, time.time()

    # ------------------------------------------------------------------ main loop
    def loop(self):
        self.tick += 1
        el = time.time() - self.t_state
        if self.state == "WAIT_ODOM":
            if self.pos is not None:
                self.sp = (self.pos[0], self.pos[1], -self.alt, self.yaw)
                self.set_state("STREAM")
            return
        self.publish_setpoint()                                  # must stream continuously in offboard
        if self.state == "STREAM" and el > 1.5:
            self.set_state("ARM")
        elif self.state == "ARM":
            if self.tick % 20 == 0:
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)            # PX4 custom mode OFFBOARD
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
            if self.armed and self.offboard:
                self.set_state("TAKEOFF")
        elif self.state == "TAKEOFF":
            if abs(-self.pos[2] - self.alt) < 0.3 and el > 3:
                self.set_state("MISSION")
        elif self.state == "MISSION":
            if self.tick % 4 == 0:                               # plan at 5 Hz
                with self.lock:
                    pos = self.pos.copy()
                    n, e, yaw, self.info = self.mf.step(pos, self.yaw)
                self.sp = (n, e, -self.alt, yaw)
                if self.mf.done:
                    self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self.set_state("LAND")
        elif self.state == "LAND":
            if self.tick % 20 == 0:
                self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if not self.armed and el > 3:
                self.set_state("DONE")
                self.finish()
                return
        if self.tick % 4 == 0:
            occ = int(self.mf.grid.occupied.sum())
            self.csv.writerow([round(time.time() - self.t0, 2), self.state, *np.round(self.pos[:2], 2),
                               round(-self.pos[2], 2), round(self.yaw, 3), *np.round(self.sp[:2], 2),
                               round(self.sp[3], 3), self.info.get("mode"), self.mf.k, occ, self.depth_frames])
        if self.tick % 100 == 0:
            self.get_logger().info(f"{self.state} pos=({self.pos[0]:.1f},{self.pos[1]:.1f},{-self.pos[2]:.1f}) "
                                   f"goal#{self.mf.k} mode={self.info.get('mode')} "
                                   f"depth_frames={self.depth_frames} occ={int(self.mf.grid.occupied.sum())}")

    def finish(self):
        self.csv_f.close()
        np.save(os.path.join(self.log_dir, "occupancy_hits.npy"), self.mf.grid.hits)
        json.dump({"n0": self.mf.grid.n0, "e0": self.mf.grid.e0, "res": self.mf.grid.res,
                   "replans": self.mf.replans, "depth_frames": self.depth_frames, "avoid": self.avoid},
                  open(os.path.join(self.log_dir, "planner_meta.json"), "w"), indent=2)
        self.get_logger().info("mission complete, logs saved")
        raise SystemExit


def main():
    rclpy.init()
    node = AvoidMission()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
