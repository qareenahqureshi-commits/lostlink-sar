#!/usr/bin/env python3
"""
Stage 4 simulated radio link between the ground station (at home) and the drone.

Uses the drone's TRUE pose from Gazebo and the building geometry (sim/worlds/make_stage1.py):
link OK if  Ptx - PL(distance) - 20 dB x (walls in between)  >= -85 dBm   (see sim_env.py).
If jam_link is true, the jammer that takes GPS also takes the link: link lost while
/lostlink/gps_denied is true (the "Lost-Link" scenario).

Publishes /lostlink/link_ok (std_msgs/Bool) and /lostlink/link_rssi (Float32MultiArray [rssi_dbm, walls]) at 2 Hz.
Params: world_py (make_stage1.py or a Stage 5 building_<seed>.json), jam_link (true), world (sar_building), model (x500_depth_0)
"""
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from lostlink_planner.sim_env import LinkModel, load_world_any


class SimLink(Node):
    def __init__(self):
        super().__init__("lostlink_sim_link")
        for k, v in (("world_py", ""), ("jam_link", True), ("world", "sar_building"), ("model", "x500_depth_0")):
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        self.link = LinkModel(load_world_any(g("world_py")))
        self.jam, self.model, self.denied = bool(g("jam_link")), g("model"), False
        self.ne, self.lock = None, threading.Lock()
        self.pub = self.create_publisher(Bool, "/lostlink/link_ok", 10)
        self.pub_rssi = self.create_publisher(Float32MultiArray, "/lostlink/link_rssi", 10)
        self.create_subscription(Bool, "/lostlink/gps_denied", lambda m: setattr(self, "denied", bool(m.data)), 10)
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, f"/world/{g('world')}/dynamic_pose/info", self.on_pose)
        self.create_timer(0.5, self.tick)
        self.last_ok = None
        self.get_logger().info(f"sim link: wall model, jam_link={self.jam}")

    def on_pose(self, msg):
        p = next((p for p in msg.pose if p.name == self.model), None)
        if p is not None:
            with self.lock:
                self.ne = (p.position.y, p.position.x)          # ENU -> (north, east)

    def tick(self):
        with self.lock:
            ne = self.ne
        if ne is None:
            return
        rssi, walls = self.link.rssi(ne)
        ok = rssi >= self.link.sens and not (self.jam and self.denied)
        self.pub.publish(Bool(data=bool(ok)))
        m = Float32MultiArray(); m.data = [float(rssi), float(walls)]; self.pub_rssi.publish(m)
        if ok != self.last_ok:
            self.get_logger().warn(f"LINK {'OK' if ok else 'LOST'} (rssi {rssi:.0f} dBm, walls {walls}, jammed={self.jam and self.denied})")
            self.last_ok = ok


def main():
    rclpy.init()
    n = SimLink()
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
