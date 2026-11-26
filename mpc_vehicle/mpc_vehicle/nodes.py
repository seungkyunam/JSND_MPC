"""Separate MPC vehicle adapter and post-mux gate; original PP/mux untouched."""
import json
import math
import os
import time
import fcntl
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterEvent
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from f110_msgs.msg import WpntArray
from controller.mpc.integrated_mpc import IntegratedMPC, IntegratedConfig
from .core import GatePolicy, load_profile, validate_raceline, combine_speeds, solver_speed

PREFIX = '/mpc_vehicle'
RAW = PREFIX + '/raw_command'
MUX = PREFIX + '/mux_command'
FINAL = '/vesc/ackermann_cmd'
HEALTH = PREFIX + '/solver_ok'
APPLIED = PREFIX + '/applied_command'
ACTUATORS = ('motor/speed', 'motor/brake', 'servo/position')
DISABLED = ('motor/current', 'motor/duty_cycle', 'motor/position')
# Raceline of whatever map the running stack selected (global_republisher, 2 s period)
# and its sector-scaled copy (speed_sector_tuner, 0.5 s period).
WAYPOINTS = '/global_waypoints'
SCALED_WAYPOINTS = '/global_waypoints_scaled'
WAYPOINT_TIMEOUT_S = 5.
MAX_PATH_OFFSET_M = 1.


def fqn(endpoint):
    return endpoint.node_namespace.rstrip('/') + '/' + endpoint.node_name


def profile_for(node):
    read_only = ParameterDescriptor(read_only=True)
    node.declare_parameter('profile', '', read_only)
    node.declare_parameter('mode', 'shadow', read_only)
    node.declare_parameter('map_dir', '', read_only)
    mode = node.get_parameter('mode').value
    p = load_profile(node.get_parameter('profile').value, mode,
                     node.get_parameter('map_dir').value or None)
    if node.get_parameter('use_sim_time').value:
        raise ValueError('vehicle nodes require system ROS time')
    node.add_on_set_parameters_callback(lambda params: SetParametersResult(
        successful=False, reason='vehicle configuration is restart-only'))
    return p, mode


def stamp_ok(node, msg, limit=.2):
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.e-9
    now = node.get_clock().now().nanoseconds * 1.e-9
    return stamp > 0 and -.025 <= now-stamp <= limit


def command(node, speed=0., steer=0.):
    msg = AckermannDriveStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'base_link'
    msg.drive.speed = float(speed); msg.drive.steering_angle = float(steer)
    return msg


class VehicleController(Node):
    """Low-speed global-path tracking only. No FTG/overtaking/PP fallback."""
    def __init__(self):
        super().__init__('mpc_vehicle_controller')
        self.p, self.mode = profile_for(self)
        p = self.p
        # No map is bound at start-up: the raceline arrives on WAYPOINTS from
        # whichever map the running stack selected, and is replaced live when
        # that map changes. Until a valid one is fresh, the command is a stop.
        self.path = None; self.speeds = None; self.path_at = -math.inf
        self.path_reason = 'waiting_for_global_waypoints'
        self.scaled = None; self.scaled_at = -math.inf
        self.mpc = IntegratedMPC(IntegratedConfig(
            wheelbase=p['wheelbase'], max_steer=p['max_steer_rad'],
            max_steer_rate=p['max_steer_rate_radps'], horizon=20,
            max_speed=p['speed_limit_mps'], max_accel=p['acceleration_mps2'],
            max_decel=p['deceleration_mps2'], max_lateral_accel=p['lateral_acceleration_mps2']))
        self.odom = None; self.odom_at = -math.inf
        self.applied = None; self.applied_at = -math.inf
        self.create_subscription(Odometry, '/car_state/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(AckermannDriveStamped, APPLIED, self.applied_cb, 1)
        self.create_subscription(WpntArray, WAYPOINTS, self.waypoints_cb, QoSProfile(depth=1))
        self.create_subscription(WpntArray, SCALED_WAYPOINTS, self.scaled_cb, QoSProfile(depth=1))
        self.pub = self.create_publisher(AckermannDriveStamped, RAW, 1)
        self.health = self.create_publisher(Bool, HEALTH, 1)
        self.status = self.create_publisher(String, PREFIX+'/controller_status', 1)
        self.last_tick = time.monotonic()
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def odom_cb(self, msg):
        if stamp_ok(self, msg) and msg.header.frame_id == 'map' and msg.child_frame_id == 'base_link':
            self.odom = msg; self.odom_at = time.monotonic()
        else: self.odom_at = -math.inf

    def applied_cb(self, msg):
        if stamp_ok(self, msg):
            self.applied = msg; self.applied_at = time.monotonic()

    @staticmethod
    def raceline(msg):
        return validate_raceline([[w.x_m, w.y_m] for w in msg.wpnts], [w.vx_mps for w in msg.wpnts])

    def waypoints_cb(self, msg):
        try:
            xy, speeds = self.raceline(msg)
        except ValueError as exc:
            # An invalid raceline withdraws the previous one; never keep following a stale map.
            self.path = None; self.speeds = None; self.path_reason = 'invalid_global_waypoints:'+str(exc)
            return
        if self.path is None or self.path.shape != xy.shape or not np.allclose(self.path, xy, atol=1.e-6):
            # The OSQP setup/factorization happens on the first solve of the process.
            # Do that here, outside the 20 ms control tick, so the first real solve
            # is not a cold one. The result and its warm start are discarded.
            try:
                heading = math.atan2(*(xy[1]-xy[0])[::-1])
                window = slice(0, min(len(xy), 200))
                self.mpc.solve([*xy[0], heading, 0.], np.column_stack(
                    (xy[window], np.minimum(speeds[window], self.p['speed_limit_mps']))), 0.)
            except Exception:
                pass
            self.mpc.reset()  # new map/raceline: no warm start from the old line or the priming solve
        self.path = xy; self.speeds = speeds; self.path_at = time.monotonic()

    def scaled_cb(self, msg):
        try:
            self.scaled = self.raceline(msg); self.scaled_at = time.monotonic()
        except ValueError:
            self.scaled = None

    def current_raceline(self, now):
        if self.path is None: raise ValueError(self.path_reason)
        if now-self.path_at > WAYPOINT_TIMEOUT_S: raise ValueError('stale_global_waypoints')
        scaled = self.scaled if now-self.scaled_at <= WAYPOINT_TIMEOUT_S else None
        return self.path, combine_speeds(self.path, self.speeds, scaled)

    def tick(self):
        now = time.monotonic(); gap = now-self.last_tick; self.last_tick = now
        speed = 0.; steer = 0.; ok = False; reason = 'waiting_for_fresh_odometry_and_gate'
        try:
            line, line_speeds = self.current_raceline(now)
            if (now-self.odom_at <= .2 and now-self.applied_at <= .2 and gap <= .1
                    and stamp_ok(self, self.odom) and stamp_ok(self, self.applied)):
                pose = self.odom.pose.pose
                q = pose.orientation
                quat = np.array([q.x, q.y, q.z, q.w])
                if not np.isfinite(quat).all() or abs(np.linalg.norm(quat)-1) > .05:
                    raise ValueError('invalid quaternion')
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                xy = np.array([pose.position.x, pose.position.y])
                xy += self.p['pose_to_rear_axle_m'] * np.array([math.cos(yaw), math.sin(yaw)])
                if not np.isfinite(xy).all(): raise ValueError('nonfinite pose')
                distances = np.linalg.norm(line-xy, axis=1)
                nearest = int(np.argmin(distances))
                if distances[nearest] > MAX_PATH_OFFSET_M:
                    raise ValueError('pose_far_from_global_waypoints')  # wrong map or lost localization
                # Keep a short segment behind the vehicle and forward points over the lap seam.
                indices = (np.arange(-3, min(len(line)-3, 200)) + nearest) % len(line)
                path = line[indices]
                # Existing global profile is only reduced, never increased; no new race speed profile.
                previous = self.applied.drive.steering_angle
                measured = solver_speed(self.odom.twist.twist.linear.x)
                profile_path = np.column_stack((path, np.minimum(line_speeds[indices], self.p['speed_limit_mps'])))
                result = self.mpc.solve([*xy, yaw, measured], profile_path, previous)
                speed = result.speed
                ok = result.success
                steer = result.steering
                reason = result.status
                if not ok: speed = 0.
        except Exception as exc:
            self.mpc.reset(); speed = 0.; ok = False; reason = type(exc).__name__+':'+str(exc)
        self.pub.publish(command(self, speed if ok else 0., steer))
        self.health.publish(Bool(data=ok))
        self.status.publish(String(data=json.dumps({'ok': ok, 'reason': reason, 'mode': self.mode})))


class VehicleGate(Node):
    def __init__(self):
        super().__init__('mpc_vehicle_gate')
        self.p, self.mode = profile_for(self)
        # All launch entry paths share a same-host advisory lock, including shadow.
        lockpath = f'/tmp/jsnd_mpc_vehicle_{os.getuid()}.lock'
        self.lock = os.open(lockpath, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.policy = GatePolicy(self.p)
        self.inputs = {}; self.times = {}
        self.graph_ok = False; self.graph_at = -math.inf; self.graph_reason = 'starting'
        self.engaged_at = None
        self.config_ok = True
        self.create_subscription(ParameterEvent, '/parameter_events', self.parameter_event, 100)
        topics = [(Joy, '/joy', 'joy'), (LaserScan, '/scan', 'scan'),
                  (Odometry, '/car_state/odom', 'pose'), (Odometry, '/vesc/odom', 'odom'),
                  (Bool, PREFIX+'/estop_clear', 'estop'), (Bool, HEALTH, 'health'),
                  (AckermannDriveStamped, RAW, 'raw'), (AckermannDriveStamped, MUX, 'mux')]
        for cls, topic, key in topics:
            self.create_subscription(cls, topic, lambda msg, k=key: self.receive(k, msg),
                                     qos_profile_sensor_data if key in ('joy', 'scan', 'pose', 'odom') else QoSProfile(depth=1))
        # No publisher on FINAL (or any hardware topic) exists in shadow.
        self.output = self.create_publisher(AckermannDriveStamped,
                                            PREFIX+'/shadow_output' if self.mode == 'shadow' else FINAL, 1)
        self.applied_pub = self.create_publisher(AckermannDriveStamped, APPLIED, 1)
        self.status = self.create_publisher(String, PREFIX+'/gate_status', 1)
        self.create_service(Trigger, PREFIX+'/arm', self.arm)
        self.create_service(Trigger, PREFIX+'/stop', self.stop)
        self.create_timer(.1, self.inspect_graph, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def parameter_event(self, msg):
        protected = ('/vesc/vesc_driver_node', '/vesc/ackermann_to_vesc_node',
                     '/vesc/vesc_to_odom_node', '/mpc_vehicle_mux', '/particle_filter',
                     '/ekf_localization', '/static_tf_publisher', '/imu_bias_corrector')
        if msg.node in protected and (msg.changed_parameters or msg.deleted_parameters):
            self.config_ok = False
            self.policy.stop('runtime_configuration_changed_restart_required')

    def receive(self, key, msg):
        if hasattr(msg, 'header') and not stamp_ok(self, msg):
            self.times[key] = -math.inf
            if self.policy.armed and key in ('joy', 'scan', 'pose', 'odom', 'mux'):
                self.policy.stop('invalid_header_stamp:'+key)
            return
        if key in ('raw', 'mux') and not all(math.isfinite(v) for v in (msg.drive.speed, msg.drive.steering_angle)):
            self.times[key] = -math.inf
            self.policy.stop('nonfinite_command:'+key)
            return
        self.inputs[key] = msg; self.times[key] = time.monotonic()
        # A short press / clearance withdrawal must latch even between timer ticks.
        if (key == 'estop' and not msg.data) or (key == 'joy' and msg.buttons and msg.buttons[0]):
            self.policy.stop('emergency_stop')
        if key == 'health' and not msg.data and self.policy.armed:
            joy = self.inputs.get('joy')
            if joy is not None and len(joy.buttons) >= 6 and joy.buttons[5] and not joy.buttons[4]:
                self.policy.stop('mpc_solver_failure')

    def fresh(self, key, now):
        if now-self.times.get(key, -math.inf) > .2: return False
        msg = self.inputs[key]
        return not hasattr(msg, 'header') or stamp_ok(self, msg)

    def owned(self, topic, name, subscriber=False):
        endpoints = (self.get_subscriptions_info_by_topic(topic) if subscriber
                     else self.get_publishers_info_by_topic(topic))
        return len(endpoints) == 1 and fqn(endpoints[0]) == name

    def inspect_graph(self):
        try:
            expected = {RAW: '/mpc_vehicle_controller', HEALTH: '/mpc_vehicle_controller',
                        MUX: '/mpc_vehicle_mux', APPLIED: '/mpc_vehicle_gate',
                        '/joy': self.p['joy_source_node'],
                        PREFIX+'/estop_clear': self.p['estop_source_node']}
            if self.mode != 'shadow':
                expected[FINAL] = '/mpc_vehicle_gate'
                expected[WAYPOINTS] = '/mpc_vehicle_waypoints'
                for suffix in ACTUATORS:
                    expected[PREFIX+'/actuator/'+suffix] = '/vesc/ackermann_to_vesc_node'
            reasons = [topic for topic, name in expected.items() if not self.owned(topic, name)]
            if self.mode != 'shadow':
                # No ordinary PP high-level path or alternate motor control is permitted.
                for topic in ['/vesc/high_level/ackermann_cmd'] + ['/vesc/commands/'+s for s in ACTUATORS]:
                    if self.get_publishers_info_by_topic(topic): reasons.append(topic)
                if not self.owned(FINAL, '/vesc/ackermann_to_vesc_node', True): reasons.append('final_subscriber')
                for suffix in ACTUATORS:
                    subscribers = self.get_subscriptions_info_by_topic(PREFIX+'/actuator/'+suffix)
                    expected_subs = ['/vesc/vesc_driver_node']
                    if sorted(fqn(e) for e in subscribers) != sorted(expected_subs):
                        reasons.append('actuator_subscriber:'+suffix)
                for suffix in DISABLED:
                    for topic in ('/vesc/commands/'+suffix, PREFIX+'/actuator/'+suffix):
                        if self.get_publishers_info_by_topic(topic): reasons.append(topic)
            for topic in (PREFIX+'/disabled_keyboard', PREFIX+'/disabled_keyboard_select'):
                if self.get_publishers_info_by_topic(topic): reasons.append(topic)
            names = [ns.rstrip('/')+'/'+name for name, ns in self.get_node_names_and_namespaces()]
            for name in expected.values():
                if names.count(name) != 1: reasons.append('duplicate_or_missing_node:'+name)
            for topic in ('/scan', '/car_state/odom', '/vesc/odom'):
                if len(self.get_publishers_info_by_topic(topic)) != 1: reasons.append(topic)
            if self.mode != 'shadow' and '/controller_manager' in names:
                reasons.append('PP_controller_present')
            self.graph_ok = not reasons
            self.graph_reason = ','.join(reasons)
        except Exception as exc:
            self.graph_ok = False; self.graph_reason = str(exc)
        self.graph_at = time.monotonic()
        if not self.graph_ok: self.policy.stop('graph_ownership')

    def readiness(self, now):
        required = ('joy', 'scan', 'pose', 'odom', 'estop')
        if not self.config_ok or not self.graph_ok or now-self.graph_at > .25 or not all(self.fresh(k, now) for k in required):
            return False, False, False, True
        joy = self.inputs['joy']
        if len(joy.buttons) < 6 or len(joy.axes) < 4 or not all(math.isfinite(v) and abs(v)<=1.01 for v in joy.axes):
            return False, False, False, True
        manual, auto = bool(joy.buttons[4]), bool(joy.buttons[5])
        emergency = bool(joy.buttons[0]) or not self.inputs['estop'].data
        scan = self.inputs['scan']; odom = self.inputs['odom']; pose = self.inputs['pose']
        v = odom.twist.twist.linear.x
        q = pose.pose.pose.orientation
        xyzq = [pose.pose.pose.position.x, pose.pose.pose.position.y, q.x, q.y, q.z, q.w]
        if (pose.header.frame_id != 'map' or pose.child_frame_id != 'base_link'
                or not all(math.isfinite(x) for x in xyzq)
                or abs(math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)-1.) > .05
                or not math.isfinite(v) or abs(v) > self.p['speed_limit_mps'] + .2
                or not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0):
            return False, manual, auto, emergency
        front = [r for i, r in enumerate(scan.ranges)
                 if math.isfinite(r) and max(.05, scan.range_min) <= r <= scan.range_max
                 and abs(scan.angle_min+i*scan.angle_increment) <= math.radians(35)]
        if len(front) < 10 or min(front) < max(.35, abs(v)*.5+.2):
            return False, manual, auto, emergency
        return True, manual, auto, emergency

    def arm(self, request, response):
        now = time.monotonic()
        ready, manual, auto, emergency = self.readiness(now)
        response.success = self.policy.arm(now, ready, manual, auto, emergency)
        if response.success: self.engaged_at = None
        response.message = ('armed: HOLD LB(manual) or RB(MPC); release latches stop' if response.success
                            else 'blocked: require fresh inputs, sole owners, estop clear, neutral buttons for 0.5s')
        return response

    def stop(self, request, response):
        self.policy.stop('stop_service')
        self.output.publish(command(self, 0., self.policy.steer))
        response.success = True; response.message = 'latched stop; explicit neutral re-arm required'
        return response

    def tick(self):
        now = time.monotonic()
        ready, manual, auto, emergency = self.readiness(now)
        cmd = None
        auto_ok = (self.fresh('health', now) and self.inputs['health'].data and self.fresh('raw', now))
        if self.policy.armed and (manual or auto):
            if self.engaged_at is None: self.engaged_at = now
            if self.mode == 'bench' and now-self.engaged_at > 5.:
                self.policy.stop('bench_5_second_limit')
        if self.fresh('mux', now):
            mux = self.inputs['mux'].drive
            if manual:
                joy = self.inputs['joy']
                expected = (joy.axes[1]*self.p['speed_limit_mps'], joy.axes[3]*self.p['max_steer_rad'])
            elif auto_ok:
                raw = self.inputs['raw'].drive
                expected = (raw.speed, raw.steering_angle)
            else: expected = None
            # Original mux may still hold previous authority for one tick. Never pass that sample.
            if expected is not None and all(math.isfinite(v) for v in (mux.speed, mux.steering_angle)):
                if abs(mux.speed-expected[0]) < .01 and abs(mux.steering_angle-expected[1]) < .02:
                    cmd = (mux.speed, mux.steering_angle)
        elif self.policy.armed and (manual or auto):
            ready = False
        speed, steer = self.policy.tick(now, ready=ready, manual=manual, auto=auto,
                                       emergency=emergency, command=cmd, auto_ok=auto_ok)
        msg = command(self, speed, steer)
        self.output.publish(msg)
        # This is limited applied command, NOT measured road-wheel feedback.
        self.applied_pub.publish(msg)
        self.status.publish(String(data=json.dumps({'mode': self.mode, 'armed': self.policy.armed,
                          'reason': self.policy.reason, 'graph_ok': self.graph_ok,
                          'graph_reason': self.graph_reason, 'ready': ready,
                          'speed': speed, 'steer': steer})))


class EstopBridge(Node):
    """ELRS kill switch (/joy buttons[0], 1 = kill) -> PREFIX/estop_clear.

    Fail-safe: clear=True only after 1.0 s of continuous, fresh (<0.2 s, same as gate), well-formed
    /joy from the sole configured joy source with the kill switch off. Any kill,
    staleness, malformed message or foreign publisher latches False immediately and
    restarts the 1.0 s hold. The gate additionally latches its own stop.
    """
    HOLD_S = 1.
    FRESH_S = .2

    def __init__(self):
        super().__init__('mpc_vehicle_estop')
        self.p, self.mode = profile_for(self)
        self.last_ok = -math.inf
        self.clear_since = None
        self.graph_ok = False; self.graph_at = -math.inf
        self.pub = self.create_publisher(Bool, PREFIX+'/estop_clear', 1)
        self.create_subscription(Joy, '/joy', self.joy, qos_profile_sensor_data)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self.inspect_graph, clock=steady)
        self.create_timer(.02, self.tick, clock=steady)

    def deny(self):
        self.clear_since = None
        self.pub.publish(Bool(data=False))

    def inspect_graph(self):
        try:
            pubs = self.get_publishers_info_by_topic('/joy')
            self.graph_ok = len(pubs) == 1 and fqn(pubs[0]) == self.p['joy_source_node']
        except Exception:
            self.graph_ok = False
        self.graph_at = time.monotonic()
        if not self.graph_ok: self.deny()

    def joy(self, msg):
        valid = (stamp_ok(self, msg, self.FRESH_S) and len(msg.buttons) >= 6 and len(msg.axes) >= 4
                 and all(math.isfinite(v) and abs(v) <= 1.01 for v in msg.axes))
        if not valid or msg.buttons[0] != 0:
            self.last_ok = -math.inf
            self.deny()
            return
        now = time.monotonic()
        self.last_ok = now
        if self.clear_since is None: self.clear_since = now

    def tick(self):
        now = time.monotonic()
        ok = (self.graph_ok and now-self.graph_at <= .25 and now-self.last_ok <= self.FRESH_S
              and self.clear_since is not None)
        if not ok:
            self.deny()
            return
        self.pub.publish(Bool(data=now-self.clear_since >= self.HOLD_S))


class WaypointSource(Node):
    """bench/drive only: the race stack (and its global_republisher) must be stopped there,
    so republish the launch-selected map's raceline on WAYPOINTS. Shadow never starts this."""
    def __init__(self):
        super().__init__('mpc_vehicle_waypoints')
        self.p, self.mode = profile_for(self)
        if self.mode == 'shadow': raise ValueError('shadow uses the running stack /global_waypoints')
        from rosidl_runtime_py.set_message import set_message_fields
        data = json.loads((Path(self.p['map_dir'])/'global_waypoints.json').read_text())
        self.msg = WpntArray()
        set_message_fields(self.msg, data['global_traj_wpnts_iqp'])
        VehicleController.raceline(self.msg)  # refuse to start on an unusable raceline
        self.msg.header.frame_id = 'map'
        self.pub = self.create_publisher(WpntArray, WAYPOINTS, 1)
        self.create_timer(1., self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish()

    def publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def spin(cls):
    rclpy.init()
    node = None
    try:
        node = cls(); rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if isinstance(node, VehicleGate): node.output.publish(command(node, 0., node.policy.steer))
            if isinstance(node, EstopBridge): node.pub.publish(Bool(data=False))
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def controller_main(): spin(VehicleController)
def gate_main(): spin(VehicleGate)
def waypoints_main(): spin(WaypointSource)
def estop_main(): spin(EstopBridge)
