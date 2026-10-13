#!/usr/bin/env python3
"""
ROS2 mission node (Stages 2-4): fly the Stage 1 mission in PX4 OFFBOARD mode with depth-camera obstacle avoidance.

  policy:=stage2 (default)      Stage 2/3 behaviour (core.MissionFollower)
  policy:=stage3|map|map_reloc|stage4|rtl_link   Stage 4 logic (stage4.LostLinkMission), see stage4.py

Inputs
  /fmu/out/vehicle_odometry           px4_msgs/VehicleOdometry   (pose, NED)
  /fmu/out/vehicle_status_v4          px4_msgs/VehicleStatus     (armed? offboard?)
  Gazebo /depth_camera                gz.msgs.Image R_FLOAT32    (read directly via gz-transport)
Outputs
  /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint, /fmu/in/vehicle_command

Sequence: stream setpoints -> OFFBOARD + ARM -> take off to `alt` -> waypoints (+avoidance) -> home -> LAND.

Run:
  ros2 run lostlink_planner avoid_mission --ros-args -p plan_file:=<.plan> -p avoid:=true -p log_dir:=<dir>
  ros2 run lostlink_planner avoid_mission --ros-args ... -p policy:=stage4 -p r_hat:=0.05 -p gps_deny_east:=14.0
"""
import csv, json, math, os, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float32MultiArray
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus

from lostlink_planner import core, stage4
from lostlink_planner.gz_time import SimWallClock, PoseBuffer

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


class Stage4Adapter:
    """Gives stage4.LostLinkMission the interface the node expects (grid, k, done, replans, step)."""

    def __init__(self, m):
        self.m = m

    grid = property(lambda self: self.m.grid)
    k = property(lambda self: self.m.k)
    done = property(lambda self: self.m.done)
    replans = property(lambda self: self.m.replans)

    def insert_points(self, pts):
        self.m.insert_points(pts)

    def step(self, pos, yaw, t, denied, link_ok, dt, t_flight):
        self.m.update_state(t, pos, denied, link_ok, dt)
        return self.m.step(t, yaw, t_flight)


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
        self.declare_parameter("world", "sar_building")
        self.declare_parameter("gps_deny_east", -1.0)     # Stage 3: jam GPS when estimated East > this (m); <0 = never
        self.declare_parameter("max_mission_s", 900.0)    # wall-clock limit for the MISSION phase, then land where it is
        self.declare_parameter("policy", "stage2")         # Stage 4: rtl_link | stage3 | map | map_reloc | stage4
        self.declare_parameter("r_hat", 0.05)              # Stage 4: assumed VIO drift rate (prior)
        self.declare_parameter("budget_m", 2.0)            # Stage 4: acceptable landing error
        self.declare_parameter("capture_m", 3.0)           # Stage 4: max error at which the exit can still be recognised
        self.declare_parameter("world_spec", "")           # Stage 5: generated building spec (.json) -> map extent
        self.declare_parameter("max_yaw_rate", 0.25)       # Stage 5: skip depth frames while turning faster (rad/s); 0 = off
        self.declare_parameter("land_timeout_s", 60.0)     # sim only: force-disarm if PX4 never detects the landing
        gp = lambda k: self.get_parameter(k).value
        self.alt, self.avoid = float(gp("alt")), bool(gp("avoid"))
        self.log_dir = gp("log_dir"); os.makedirs(self.log_dir, exist_ok=True)

        wps = plan_waypoints_ned(gp("plan_file"))
        self.get_logger().info(f"{len(wps)} waypoints (N,E): {[tuple(round(v, 1) for v in w) for w in wps]}")
        self.policy = gp("policy")
        if self.policy == "stage2":
            self.mf = core.MissionFollower(wps, alt=self.alt, avoid=self.avoid, lookahead=float(gp("lookahead")))
        else:
            mk = None
            if gp("world_spec"):
                sp = json.load(open(gp("world_spec")))
                (e0, e1), (n0, n1) = sp["bounds_en"]
                mk = {"n_range": (n0, n1), "e_range": (e0, e1), "res": 0.25}
            self.mf = Stage4Adapter(stage4.LostLinkMission(
                wps, policy=self.policy, r_hat=float(gp("r_hat")), budget_m=float(gp("budget_m")),
                capture_m=float(gp("capture_m")), lookahead=float(gp("lookahead")), map_kwargs=mk))
        self.link_ok, self.t_takeoff = True, None
        self.max_yaw_rate, self.yaw_rate, self.n_gated = float(gp("max_yaw_rate")), 0.0, 0
        self.land_timeout = float(gp("land_timeout_s"))
        self.create_subscription(Bool, "/lostlink/link_ok", lambda m: setattr(self, "link_ok", bool(m.data)), 10)
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
        self.poses, self.sync_lag = PoseBuffer(), []
        self.n_sync = self.n_fallback = 0; self.last_gap = None; self.depth_err = None
        self.gps_deny_east = float(gp("gps_deny_east")); self.max_mission_s = float(gp("max_mission_s"))
        self.gps_denied, self.t_denied, self.vio_drift, self.outcome = False, None, 0.0, "running"
        self.pub_denied = self.create_publisher(Bool, "/lostlink/gps_denied", 10)
        self.create_subscription(Float32MultiArray, "/lostlink/vio_drift", self.on_drift, 10)
        self.world = gp("world")

        self._start_depth(gp("depth_topic"))
        f = open(os.path.join(self.log_dir, "planner_log.csv"), "w", newline="")
        self.csv_f, self.csv = f, csv.writer(f)
        self.csv.writerow(["t", "state", "n", "e", "alt", "yaw", "sp_n", "sp_e", "sp_yaw", "mode", "goal_idx",
                           "occupied_cells", "depth_frames", "gps_denied", "vio_drift_m",
                           "s4_state", "corr_n", "corr_e", "sigma", "r_hat", "link_ok"])
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
        from gz.msgs10.clock_pb2 import Clock
        self.clock = SimWallClock()
        self.gz = GzNode()
        self.gz.subscribe(Clock, f"/world/{self.world}/clock", self.clock.on_clock)
        ok = self.gz.subscribe(Image, topic, self.on_depth)
        self.get_logger().info(f"depth subscription {topic}: {'ok' if ok else 'FAILED'}")

    def on_depth(self, msg):
        try:
            self._on_depth(msg)
        except Exception as e:                       # gz callbacks swallow exceptions -> log them
            self.depth_err = f"{type(e).__name__}: {e}"

    def _on_depth(self, msg):
        now = time.time()
        if now - self.last_depth_t < 0.1 or self.pos is None:     # 10 Hz is plenty
            return
        self.last_depth_t = now
        img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        # Stage 3 fix: use the pose at the moment the image was rendered (sim stamp -> wall -> PX4 pose)
        s = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        wall = self.clock.sim_to_wall(s)
        pos = q = None
        if wall is not None and abs(now - wall) > 1.0:   # implausible capture time -> don't trust it
            self.last_gap = round(now - wall, 2); wall = None
        if wall is not None:
            pos, q, gap = self.poses.at(wall)
            if pos is not None and (gap is None or gap <= 0.3):
                self.sync_lag.append(now - wall); self.n_sync += 1
            else:
                pos = q = None
                self.last_gap = gap
        if pos is None:                               # fall back to the latest pose (Stage 2 behaviour)
            self.n_fallback += 1
            with self.lock:
                pos, q = self.pos.copy(), list(self.q)
        # Stage 5 motion gating: while the drone turns fast, a depth image placed with a slightly wrong
        # pose paints walls up to ~1 m off (the time-sync limitation). Skip those frames; the camera
        # sees the same scene again as soon as the turn ends.
        if self.max_yaw_rate > 0 and abs(self.yaw_rate) > self.max_yaw_rate:
            self.n_gated += 1
            return
        pts = self.proj.to_world(img, pos, q)
        with self.lock:
            if hasattr(self.mf, "insert_points"):
                self.mf.insert_points(pts)            # Stage 4: applies the relocalisation correction
            else:
                self.mf.grid.insert(pts)
        self.depth_frames += 1

    def on_odom(self, m):
        with self.lock:
            self.pos = np.array([m.position[0], m.position[1], m.position[2]], float)
            self.q = [m.q[0], m.q[1], m.q[2], m.q[3]]
        self.poses.add(m.timestamp_sample * 1e-6, self.pos, self.q)
        self.yaw = core.yaw_from_quat(self.q)
        w = float(m.angular_velocity[2])
        self.yaw_rate = w if np.isfinite(w) else 0.0

    def on_drift(self, m):
        self.vio_drift = float(m.data[2])

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
                self.t_takeoff = time.time()
                self.set_state("MISSION")
        elif self.state == "MISSION":
            if not self.gps_denied and self.gps_deny_east >= 0 and self.pos[1] > self.gps_deny_east:
                self.gps_denied, self.t_denied = True, time.time()
                self.command(VehicleCommand.VEHICLE_CMD_INJECT_FAILURE, 4, 1)   # unit GPS(4), type OFF(1)
                self.get_logger().warn(f"GPS JAMMED at east={self.pos[1]:.1f} m (failure injection)")
            if self.gps_denied and self.tick % 20 == 0:
                self.pub_denied.publish(Bool(data=True))
            if el > self.max_mission_s:
                self.outcome = "timeout_landed_away"
                self.get_logger().error("mission time limit reached -> landing where it is")
                self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND); self.set_state("LAND")
                return
            if self.tick % 4 == 0:                               # plan at 5 Hz
                with self.lock:
                    pos = self.pos.copy()
                    if isinstance(self.mf, Stage4Adapter):
                        n, e, yaw, self.info = self.mf.step(pos, self.yaw, time.time() - self.t0, self.gps_denied,
                                                            self.link_ok, 0.2, time.time() - self.t_takeoff)
                    else:
                        n, e, yaw, self.info = self.mf.step(pos, self.yaw)
                self.sp = (n, e, -self.alt, yaw)
                if self.mf.done:
                    self.outcome = "completed"
                    self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self.set_state("LAND")
        elif self.state == "LAND":
            if self.tick % 20 == 0:
                self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            if self.land_timeout > 0 and el > self.land_timeout and self.tick % 40 == 0:
                # sim only: with baro height after GPS loss PX4's land detector can hang on the ground
                if self.tick % 400 == 0 or el < self.land_timeout + 2.5:
                    self.get_logger().warn("landing not detected -> forced disarm (simulation only)")
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
            if not self.armed and el > 3:
                self.set_state("DONE")
                self.finish()
                return
        if self.tick % 4 == 0:
            occ = int(self.mf.grid.occupied.sum())
            self.csv.writerow([round(time.time() - self.t0, 2), self.state, *np.round(self.pos[:2], 2),
                               round(-self.pos[2], 2), round(self.yaw, 3), *np.round(self.sp[:2], 2),
                               round(self.sp[3], 3), self.info.get("mode"), self.mf.k, occ, self.depth_frames,
                               int(self.gps_denied), round(self.vio_drift, 3), *self.mf_extra()])
        if self.tick % 100 == 0:
            self.get_logger().info(f"{self.state} pos=({self.pos[0]:.1f},{self.pos[1]:.1f},{-self.pos[2]:.1f}) "
                                   f"goal#{self.mf.k} mode={self.info.get('mode')} "
                                   f"depth_frames={self.depth_frames} occ={int(self.mf.grid.occupied.sum())} "
                                   f"gps={'DENIED' if self.gps_denied else 'ok'} drift={self.vio_drift:.2f}m "
                                   f"sync={self.n_sync}/fb={self.n_fallback} gap={self.last_gap} "
                                   f"lag={(self.sync_lag[-1] if self.sync_lag else float('nan')):.2f}s err={self.depth_err} "
                                   f"s4={self.mf_extra()}")

    def mf_extra(self):
        if not isinstance(self.mf, Stage4Adapter):
            return ["", "", "", "", "", ""]
        m = self.mf.m
        return [m.state, round(float(m.corr[0]), 2), round(float(m.corr[1]), 2), round(m.dec.sigma(m.s), 2),
                round(m.dec.r_hat, 4), int(self.link_ok)]

    def finish(self):
        self.csv_f.close()
        np.save(os.path.join(self.log_dir, "occupancy_hits.npy"), self.mf.grid.hits)
        extra = {}
        if isinstance(self.mf, Stage4Adapter):
            m = self.mf.m
            np.save(os.path.join(self.log_dir, "occupied_final.npy"), m.grid.occupied)
            if hasattr(m.grid, "anchor_hits"):
                np.save(os.path.join(self.log_dir, "anchor_hits.npy"), m.grid.anchor_hits)
            extra = {"policy": self.policy, "return_reason": m.return_reason, "events": m.events,
                     "reloc_log": m.reloc_log, "relocalisations": m.n_reloc, "corr_final": m.corr.round(3).tolist(),
                     "r_hat_prior": m.dec.r_prior, "r_hat_final": round(m.dec.r_hat, 4),
                     "s_denied_m": round(m.s, 1), "final_corrected_pos_ne": (self.pos[:2] + m.corr).round(2).tolist(),
                     "skipped_wps": list(map(int, m.skipped)), "stuck_events": int(getattr(m, "n_stuck", 0)), "map_clears": int(getattr(m, "n_map_clears", 0)), "relocs_gated": int(getattr(m, "n_gated", 0)), "world_spec": self.get_parameter("world_spec").value,
                     "crumbs": np.round(np.array(m.crumbs.pts), 2).tolist() if m.crumbs.pts else []}
        lag = np.array(self.sync_lag) if self.sync_lag else np.array([np.nan])
        json.dump({"n0": self.mf.grid.n0, "e0": self.mf.grid.e0, "res": self.mf.grid.res,
                   "replans": self.mf.replans, "depth_frames": self.depth_frames, "avoid": self.avoid,
                   "outcome": self.outcome, "waypoints_reached": int(self.mf.k),
                   "gps_deny_east": self.gps_deny_east, "final_vio_drift_m": round(self.vio_drift, 3),
                   "final_est_pos_ne": [round(float(self.pos[0]), 2), round(float(self.pos[1]), 2)],
                   "frames_time_synced": self.n_sync, "frames_fallback": self.n_fallback,
                   "depth_sync_lag_s_median": round(float(np.nanmedian(lag)), 3),
                   "depth_sync_lag_s_p95": round(float(np.nanpercentile(lag, 95)), 3),
                   "frames_gated_turning": self.n_gated, **extra},
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
