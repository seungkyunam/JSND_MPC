#!/usr/bin/env python3
"""MPC pitwall relay: vehicle domain 173 (loopback) <-> laptop domain 174 (WiFi).

Single-direction whitelist, no generic bridging:
  173 -> 174 : map, tf, tf_static, scan, PF pose/particles, car_state/odom,
               /global_waypoints (converted to nav_msgs/Path), gate/controller status
  174 -> 173 : /initialpose ONLY (validated, rate-limited, dropped while the gate is armed)

Never publishes on 173 anything other than /initialpose, so the gate's graph
ownership checks (/scan, /car_state/odom, /vesc/odom, /global_waypoints, /joy,
actuator topics) are unaffected. Parameter services are disabled on both sides.
"""
import json
import math
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from f110_msgs.msg import WpntArray

VEHICLE_DOMAIN = 173
PITWALL_DOMAIN = 174

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
RELIABLE1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
TF_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)

# (type, vehicle topic, pitwall topic, sub qos, pub qos, max Hz or None)
FORWARD = [
    (OccupancyGrid, '/map', '/map', LATCHED, LATCHED, None),
    (TFMessage, '/tf', '/tf', TF_QOS, TF_QOS, None),
    (LaserScan, '/scan', '/scan', qos_profile_sensor_data, RELIABLE1, 10.),
    (PoseStamped, '/pf/viz/inferred_pose', '/pf/viz/inferred_pose', RELIABLE1, RELIABLE1, 20.),
    (PoseArray, '/pf/viz/particles', '/pf/viz/particles', RELIABLE1, RELIABLE1, 5.),
    (Odometry, '/car_state/odom', '/car_state/odom', qos_profile_sensor_data, RELIABLE1, 10.),
    (String, '/mpc_vehicle/gate_status', '/mpc_vehicle/gate_status', RELIABLE1, RELIABLE1, 2.),
    (String, '/mpc_vehicle/controller_status', '/mpc_vehicle/controller_status', RELIABLE1, RELIABLE1, 2.),
]

NO_PARAMS = dict(start_parameter_services=False, enable_rosout=True)


def make_node(name, domain):
    ctx = Context()
    rclpy.init(context=ctx, domain_id=domain)
    node = Node(name, context=ctx, **NO_PARAMS)
    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)
    return ctx, node, executor


class Relay:
    def __init__(self):
        self.vctx, self.vehicle, self.vexec = make_node('mpc_pitwall_relay', VEHICLE_DOMAIN)
        self.pctx, self.pitwall, self.pexec = make_node('mpc_pitwall_relay_wifi', PITWALL_DOMAIN)
        self.log = self.vehicle.get_logger()
        self.gate = None; self.gate_at = -math.inf
        self.last_pose_at = -math.inf
        self.static = {}

        for cls, src, dst, sqos, pqos, hz in FORWARD:
            pub = self.pitwall.create_publisher(cls, dst, pqos)
            self.vehicle.create_subscription(cls, src, self.forwarder(pub, src, hz), sqos)

        # tf_static: merge every static publisher into one latched message.
        self.static_pub = self.pitwall.create_publisher(
            TFMessage, '/tf_static', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.vehicle.create_subscription(
            TFMessage, '/tf_static', self.on_static,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # WpntArray -> nav_msgs/Path so the laptop needs no f110_msgs.
        self.path_pub = self.pitwall.create_publisher(Path, '/mpc_pitwall/global_path', LATCHED)
        self.path_key = None
        self.vehicle.create_subscription(WpntArray, '/global_waypoints', self.on_waypoints, QoSProfile(depth=1))

        self.vehicle.create_subscription(String, '/mpc_vehicle/gate_status', self.on_gate, RELIABLE1)

        # The only laptop -> vehicle path.
        self.pose_pub = self.vehicle.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.pitwall.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 1)
        self.log.info(f'relay up: vehicle domain {VEHICLE_DOMAIN} (loopback) <-> pitwall domain {PITWALL_DOMAIN}')

    def forwarder(self, pub, topic, hz):
        period = 0. if hz is None else 1. / hz
        state = {'t': -math.inf}

        def cb(msg):
            now = time.monotonic()
            if now - state['t'] < period:
                return
            state['t'] = now
            pub.publish(msg)
        return cb

    def on_static(self, msg):
        for t in msg.transforms:
            self.static[t.child_frame_id] = t
        self.static_pub.publish(TFMessage(transforms=list(self.static.values())))

    def on_waypoints(self, msg):
        key = (len(msg.wpnts), msg.wpnts[0].x_m if msg.wpnts else 0., msg.wpnts[-1].y_m if msg.wpnts else 0.)
        if key == self.path_key:
            return
        self.path_key = key
        path = Path()
        path.header.frame_id = msg.header.frame_id or 'map'
        path.header.stamp = self.vehicle.get_clock().now().to_msg()
        for w in msg.wpnts:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = float(w.x_m); p.pose.position.y = float(w.y_m)
            p.pose.orientation.z = math.sin(w.psi_rad / 2.); p.pose.orientation.w = math.cos(w.psi_rad / 2.)
            path.poses.append(p)
        self.path_pub.publish(path)
        self.log.info(f'global path forwarded ({len(path.poses)} points)')

    def on_gate(self, msg):
        try:
            self.gate = json.loads(msg.data); self.gate_at = time.monotonic()
        except ValueError:
            self.gate = None

    def on_initialpose(self, msg):
        now = time.monotonic()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        reason = None
        if msg.header.frame_id.lstrip('/') != 'map':
            reason = f'frame_id={msg.header.frame_id!r} (map required)'
        elif not all(math.isfinite(v) for v in values) or abs(p.x) > 1e3 or abs(p.y) > 1e3:
            reason = 'non-finite or out-of-range pose'
        elif abs(math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2) - 1.) > 1e-2:
            reason = 'non-unit quaternion'
        elif now - self.last_pose_at < 1.:
            reason = 'rate limit (1/s)'
        elif self.gate is not None and now - self.gate_at < 1. and self.gate.get('armed'):
            reason = 'gate is ARMED: PF re-initialisation refused'
        if reason:
            self.log.warn(f'/initialpose dropped: {reason}')
            return
        self.last_pose_at = now
        msg.header.frame_id = 'map'
        msg.header.stamp = self.vehicle.get_clock().now().to_msg()  # laptop clock may be skewed
        self.pose_pub.publish(msg)
        yaw = math.degrees(2. * math.atan2(q.z, q.w))
        self.log.info(f'/initialpose forwarded to PF: x={p.x:.2f} y={p.y:.2f} yaw={yaw:.1f}deg')

    def spin(self):
        t = threading.Thread(target=self.pexec.spin, daemon=True)
        t.start()
        try:
            self.vexec.spin()
        finally:
            for ex, node, ctx in ((self.pexec, self.pitwall, self.pctx), (self.vexec, self.vehicle, self.vctx)):
                ex.shutdown(); node.destroy_node()
                if ctx.ok(): rclpy.shutdown(context=ctx)


def main():
    relay = Relay()
    try:
        relay.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
