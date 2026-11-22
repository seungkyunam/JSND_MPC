#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from ackermann_msgs.msg import AckermannDriveStamped
from copy import deepcopy

from controller.estop import EStop
from controller.safety_gate import SafetyGate


class SimpleMuxNode(Node):

    def __init__(self):
        super().__init__('simple_mux')

        self.declare_parameter('out_topic',                      'low_level/ackermann_cmd_mux/output')
        self.declare_parameter('in_topic',                       'high_level/ackermann_cmd')
        self.declare_parameter('joy_topic',                      '/joy')
        self.declare_parameter('keyboard_joy_topic',             '/joy_keyboard')
        self.declare_parameter('ego_control_topic',              '/ego/use_keyboard')
        self.declare_parameter('scan_topic',                     '/scan')
        self.declare_parameter('odom_topic',                     '/vesc/odom')
        self.declare_parameter('rate_hz',                        50.0)
        self.declare_parameter('joy_max_speed',                  4.0)
        self.declare_parameter('joy_max_steer',                  0.4)
        self.declare_parameter('joy_freshness_threshold',        1.0)
        self.declare_parameter('servo_min',                      0.15)
        self.declare_parameter('servo_max',                      0.85)
        self.declare_parameter('steering_angle_to_servo_offset', 0.5)
        self.declare_parameter('steering_angle_to_servo_gain',  -1.2135)
        self.declare_parameter('use_estop',  False)
        self.declare_parameter('sim',  False)
        # initial human-drive source; the pitwall panel can flip it at runtime
        # via ego_control_topic. False = joy, True = keyboard (/joy_keyboard).
        self.declare_parameter('use_keyboard', False)
        # safety_mode (default False = legacy behaviour, unchanged): latched E-stop (A),
        # LB = legacy manual drive (hold), RB = autodrive (hold), explicit restart
        # approval (/simple_mux/reset_estop), zero command whenever not engaged.
        # A latch blocks both autodrive and manual drive.
        self.declare_parameter('safety_mode', False)
        self.declare_parameter('safety_estop_buttons', [0])
        self.declare_parameter('safety_engage_button', 5)
        self.declare_parameter('safety_manual_button', 4)      # -1 disables manual drive
        self.declare_parameter('safety_require_engage_hold', True)
        self.declare_parameter('safety_joy_timeout_s', 0.5)
        self.declare_parameter('safety_min_latch_s', 2.0)
        self.declare_parameter('safety_standstill_mps', 0.05)
        self.declare_parameter('safety_standstill_hold_s', 1.0)
        self.declare_parameter('safety_start_latched', True)
        p = lambda name: self.get_parameter(name).value

        out_topic          = p('out_topic')
        in_topic           = p('in_topic')
        joy_topic          = p('joy_topic')
        keyboard_joy_topic = p('keyboard_joy_topic')
        ego_control_topic  = p('ego_control_topic')
        scan_topic         = p('scan_topic')
        odom_topic         = p('odom_topic')

        self.use_estop = p('use_estop')
        self.max_speed               = p('joy_max_speed')
        self.max_steer               = p('joy_max_steer')
        self.joy_freshness_threshold = p('joy_freshness_threshold')

        servo_offset = p('steering_angle_to_servo_offset')
        servo_gain   = p('steering_angle_to_servo_gain')
        self.servo_max_abs = min(
            abs((p('servo_max') - servo_offset) / servo_gain),
            abs((p('servo_min') - servo_offset) / servo_gain),
        )
        

        # start idle in every mode (sim included): nothing is published until a
        # human engages via the joy/keyboard A button (-> autodrive) or H (-> humandrive).
        self.current_host = None
        self.human_drive  = None
        self.autodrive    = None
        self.scan         = None
        self.odom         = None
        # Ego human-drive input source: initial value from the use_keyboard param,
        # then overridable at runtime by the pitwall panel via ego_control_topic.
        # False = joy (physical pad on joy_topic), True = keyboard (keyboard_joy_topic).
        self.use_keyboard = p('use_keyboard')

        self.create_subscription(AckermannDriveStamped, in_topic,           self._drive_cb,         10)
        self.create_subscription(Joy,                   joy_topic,          self._joy_cb,           10)
        self.create_subscription(Joy,                   keyboard_joy_topic, self._joy_keyboard_cb,  10)
        self.create_subscription(Bool,                  ego_control_topic,  self._ego_control_cb,   10)
        if self.use_estop:
            self.estop = EStop(self)

            self.create_subscription(LaserScan, scan_topic, self._scan_cb, qos_profile_sensor_data)
            self.create_subscription(Odometry,  odom_topic, self._odom_cb, 10)

        self.safety_mode = bool(p('safety_mode'))
        self.gate = None
        if self.safety_mode:
            self.gate = SafetyGate(
                estop_buttons=p('safety_estop_buttons'), engage_button=p('safety_engage_button'),
                manual_button=p('safety_manual_button'),
                joy_timeout_s=p('safety_joy_timeout_s'), min_latch_s=p('safety_min_latch_s'),
                standstill_mps=p('safety_standstill_mps'), standstill_hold_s=p('safety_standstill_hold_s'),
                require_hold=p('safety_require_engage_hold'), start_latched=p('safety_start_latched'))
            if not self.use_estop:   # wheel odometry is needed for the standstill check
                self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
            self.create_subscription(Bool, '/simple_mux/estop', self._soft_estop_cb, 10)
            self.estop_state_pub = self.create_publisher(String, '/simple_mux/estop_state', 10)
            self.create_service(Trigger, '/simple_mux/reset_estop', self._reset_estop_cb)
            self.get_logger().warn(
                f'safety_mode ON: estop buttons {list(p("safety_estop_buttons"))}, engage button '
                f'{p("safety_engage_button")} (hold={p("safety_require_engage_hold")}), manual button '
                f'{p("safety_manual_button")} (hold, joy_max_speed={self.max_speed} m/s). '
                'Software stop only - keep the physical power cut within reach.')
            self._flush_gate_events()

        self.drive_pub = self.create_publisher(AckermannDriveStamped, out_topic, 10)
        # Selected source (none/autodrive/humandrive) for the MPC controller: it holds
        # its speed command at 0 unless autodrive is selected. Pure status, no control.
        self.host_pub = self.create_publisher(String, '/simple_mux/active_host', 10)
        self.create_timer(1.0 / p('rate_hz'), self._loop)

    def _scan_cb(self, msg): self.scan = msg

    def _odom_cb(self, msg):
        self.odom = msg
        if self.gate is not None:
            self.gate.on_odom(msg.twist.twist.linear.x, self._now_s())

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _flush_gate_events(self):
        for _, text in self.gate.events:
            self.get_logger().warn(f'[safety] {text}')
        self.gate.events.clear()

    def _soft_estop_cb(self, msg):
        if msg.data:
            self.gate.trigger('software /simple_mux/estop', self._now_s())
            self._flush_gate_events()

    def _reset_estop_cb(self, request, response):
        response.success, response.message = self.gate.request_reset(self._now_s())
        self.get_logger().warn(f'[safety] reset request: {response.message}')
        self._flush_gate_events()
        return response

    def _safety_loop(self):
        now = self._now_s()
        host = self.gate.update(now)
        self._flush_gate_events()
        self.host_pub.publish(String(data=host))
        self.estop_state_pub.publish(String(
            data=f'latched: {self.gate.reason}' if self.gate.latched else f'clear: {host}'))
        out = AckermannDriveStamped()          # speed 0, steering 0
        out.header.stamp = self.get_clock().now().to_msg()
        if host == 'autodrive' and self._is_fresh(self.autodrive):
            out = deepcopy(self.autodrive)
        elif host == 'humandrive' and self._is_fresh(self.human_drive):
            out = deepcopy(self.human_drive)
        if self.use_estop:
            out = self.estop.should_stop(self.scan, self.odom, out)
        self.drive_pub.publish(out)
    def _drive_cb(self, msg): self.autodrive = msg

    def _is_fresh(self, msg):
        if msg is None:
            return False
        dt = (self.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        return abs(dt) < self.joy_freshness_threshold

    def _clip(self, msg):
        out = deepcopy(msg)
        out.drive.steering_angle = max(-self.servo_max_abs, min(self.servo_max_abs, out.drive.steering_angle))
        return out

    def _loop(self):
        if self.safety_mode:
            self._safety_loop()
            return
        zero = AckermannDriveStamped()
        zero.header.stamp = self.get_clock().now().to_msg()
        self.host_pub.publish(String(data=self.current_host or 'none'))

        if self.current_host is None:
            return
        elif self.current_host == 'autodrive' and self._is_fresh(self.autodrive):            # out = self._clip(self.autodrive)
            out = deepcopy(self.autodrive)
        elif self.current_host == 'humandrive' and self._is_fresh(self.human_drive):
            # out = self._clip(self.human_drive)
            out = deepcopy(self.human_drive)
        else:
            out = zero

        if self.use_estop:
            out = self.estop.should_stop(self.scan, self.odom, out)

        self.drive_pub.publish(out)

    def _ego_control_cb(self, msg):
        self.use_keyboard = bool(msg.data)

    def _joy_cb(self, msg):
        # physical pad: honoured only while joy is the selected source
        if not self.use_keyboard:
            self._handle_joy(msg)

    def _joy_keyboard_cb(self, msg):
        # keyboard_joy_node: honoured only while keyboard is the selected source
        if self.use_keyboard:
            self._handle_joy(msg)

    def _joy_to_drive(self, msg):
        # legacy manual mapping, unchanged: left stick Y x joy_max_speed, right stick X x joy_max_steer
        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.drive.steering_angle = msg.axes[3] * self.max_steer if len(msg.axes) > 3 else 0.0
        drive.drive.speed          = msg.axes[1] * self.max_speed  if len(msg.axes) > 1 else 0.0
        return drive

    def _handle_joy(self, msg):
        if self.safety_mode:     # A latches the E-stop; LB manual / RB autodrive via the gate
            self.gate.on_joy(msg.buttons, self._now_s())
            self._flush_gate_events()
            if self.gate.host == 'humandrive':
                self.human_drive = self._joy_to_drive(msg)
            return
        use_human = msg.buttons[4] if len(msg.buttons) > 4 else False
        use_auto  = msg.buttons[5] if len(msg.buttons) > 5 else False

        if use_human:
            self.human_drive   = self._joy_to_drive(msg)
            self.current_host  = 'humandrive'
        elif use_auto:
            self.current_host = 'autodrive'


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
