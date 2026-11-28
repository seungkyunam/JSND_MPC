#!/usr/bin/env python3

import os
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
import yaml

from ackermann_msgs.msg import AckermannDriveStamped
from f110_msgs.msg import WpntArray, BehaviorStrategy
from sensor_msgs.msg import LaserScan
from frenet_conversion.frenet_converter import FrenetConverter
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool, String
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

from controller.combined.src.Controller import Controller
from controller.ftg.ftg import FTG
from controller.mpc.configuration import MPC_DEFAULTS, make_config, make_steering_config


# tunable L1 params (ROS1 controller.cfg / dyn_controller). All consumed by the
# Controller; live-tunable + saved to controller.yaml.
L1_PARAMS = [
    't_clip_min', 't_clip_max', 'm_l1', 'q_l1', 'speed_lookahead', 'lat_err_coeff',
    'acc_scaler_for_steer', 'dec_scaler_for_steer', 'start_scale_speed',
    'end_scale_speed', 'downscale_factor', 'speed_lookahead_for_steer',
    'trailing_gap', 'trailing_vel_gain', 'trailing_p_gain', 'trailing_i_gain',
    'trailing_d_gain', 'blind_trailing_speed', 'static_target_hold_sec',
    'static_min_standoff_m', 'static_resume_hysteresis_m',
    'static_crawl_speed_mps', 'curvature_factor',
    'speed_factor_for_lat_err', 'speed_factor_for_curvature', 'KP', 'KI', 'KD',
    'heading_error_thres', 'steer_gain_for_speed', 'future_constant', 'AEB_thres',
    'speed_diff_thres', 'start_speed', 'start_curvature_factor',
]

# FTG params live-tunable via rqt (controller.yaml). Mapped onto the FTG instance
# in dyn_param_cb. Disparity-extender tunables are in metres/degrees.
FTG_PARAMS = [
    'ftg_debug', 'ftg_max_lidar_dist', 'ftg_max_speed', 'ftg_track_width',
    'ftg_front_fov_deg', 'ftg_smooth_deg', 'ftg_disp_thresh', 'ftg_bubble_m',
    'ftg_steer_ema', 'ftg_max_steer',
]


class ControllerManager(Node):
    """ROS2 port of the ROS1 unicorn controller_manager (Pure-Pursuit only; the
    MAP / steering-lookup branch was intentionally removed).

    Subscribes /behavior_strategy, /car_state/odom, /imu/data,
    /car_state/odom_frenet, /scan, /vesc/odom, /save_start_traj.
    Publishes the ackermann command (default high_level/ackermann_cmd -> simple_mux),
    plus lookahead/future/trailing/l1 visualization.
    """

    def __init__(self):
        super().__init__('controller_manager',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.name = "controller_manager"
        self.loop_rate = int(self._get_param('control_loop_rate_hz', 50))  # rate in hertz
        if self.loop_rate < 20 or self.loop_rate > 100:
            raise ValueError('control_loop_rate_hz must be between 20 and 100 Hz')
        self.scan = None
        self._save_requested = False

        self.mapping = self._get_param('mapping', False)

        # state shared with callbacks
        self.position_in_map = []
        self.position_in_map_frenet = []
        self.waypoint_list_in_map = []
        self.waypoint_array_in_map = None
        self.speed_now = 0
        self.acc_now = np.zeros(10)
        self.speed_now_y = 0
        self.yaw_rate = 0
        self.waypoint_safety_counter = 0
        self.opponent = [0, 0, 0, False, True]  # s, d, vs, is_static, is_visible
        self.last_static_opponent = None
        self.last_static_opponent_time_sec = None
        self.state = ""
        self.trailing_command = 2
        self.i_gap = 0
        self.curvature_waypoints = 0
        self.converter = None
        self.controller = None
        self.waypoints = None
        self.track_length = None
        self.timer = None

        self.use_sim = self._get_param('sim', False)
        wheelbase_explicit = self.has_parameter('wheelbase')
        self.wheelbase = self._get_param('wheelbase', 0.321)
        # Startup-only selector: PP (legacy, default) or MPC (integrated speed+steering).
        self.lateral_controller = self._get_param('lateral_controller', 'PP')
        if self.lateral_controller not in ('PP', 'MPC'):
            raise ValueError('lateral_controller must be PP or MPC (startup parameter)')
        mpc_values = {name: self._get_param(name, value) for name, value in MPC_DEFAULTS.items()}
        self.mpc_values = mpc_values  # also read later in global_wpnts_cb (Controller construction)
        self.pose_to_rear_axle_m = mpc_values['mpc_pose_to_rear_axle_m']
        self.mpc_cmd_max_lead_mps = float(self._get_param('mpc_cmd_max_lead_mps', 0.5))
        self.mpc_cmd_accel_lead_mps = float(self._get_param('mpc_cmd_accel_lead_mps', 0.0))  # step 52, 0 = off
        self.mpc_static_approach_decel_mps2 = float(self._get_param('mpc_static_approach_decel_mps2', 1.5))
        self.mpc_envelope_recovery_speed_mps = float(self._get_param('mpc_envelope_recovery_speed_mps', 0.5))
        self.mpc_accel_jerk_mps3 = float(self._get_param('mpc_accel_jerk_mps3', 0.0))  # step 47, 0 = off
        self.mpc_odom_timeout_s = float(self._get_param('mpc_odom_timeout_s', 0.2))
        self.last_odom_time = None
        # MPC holds 0 speed unless simple_mux has autodrive selected (fresh status).
        self.mux_host = None
        self.mux_host_time = None
        self.mpc_mux_host_timeout_s = float(self._get_param('mpc_mux_host_timeout_s', 0.25))
        self.lateral_mpc = None
        self.mpc_speed_source = str(mpc_values['mpc_speed_source'])
        if self.mpc_speed_source not in ('jsnd', 'integrated'):
            raise ValueError('mpc_speed_source must be jsnd or integrated')
        if self.lateral_controller == 'MPC':
            heading = mpc_values['mpc_heading_robust']
            history = mpc_values['mpc_history_prediction']
            if heading and not history:
                raise ValueError('Step 45 heading-only variant failed validation; enable history too')
            if (heading or history) and self.mpc_speed_source != 'integrated':
                raise ValueError('Step 45 input improvements require mpc_speed_source=integrated')
            self.get_logger().info(f'[Step 45] heading_robust={heading} history_prediction={history}')
        if self.lateral_controller == 'MPC' and self.mpc_speed_source == 'jsnd':
            # Step 23: JSND speed pipeline (unchanged) + steering-only MPC.
            from controller.mpc.steering_mpc import SteeringMPC  # imports OSQP
            config = make_steering_config(mpc_values, self.wheelbase, wheelbase_explicit, self.loop_rate)
            self.lateral_mpc = SteeringMPC(config)
            self.get_logger().info(f"[{self.name}] JSND speed + steering MPC selected: {config}")
        elif self.lateral_controller == 'MPC':
            from controller.mpc.integrated_mpc import IntegratedMPC  # imports OSQP
            config = make_config(mpc_values, self.wheelbase, wheelbase_explicit, self.loop_rate)
            self.lateral_mpc = IntegratedMPC(config)
            self.get_logger().info(f"[{self.name}] Integrated MPC selected: {config}")
        self.measuring = self._get_param('measure', False)
        self.state_machine_rate = self._get_param('state_machine_rate', 40)

        # save-back path (controller.yaml in stack_master/config)
        try:
            from ament_index_python.packages import get_package_share_directory
            default_yaml = os.path.join(
                get_package_share_directory('stack_master'), 'config', 'controller.yaml')
        except Exception:
            default_yaml = ''
        self.save_yaml_path = self._get_param('save_yaml_path', default_yaml)

        # load all L1 params into members (startup-apply; mirror ROS1 init)
        for p in L1_PARAMS:
            setattr(self, p, self._get_param(p))

        # Publishers (ROS1 topic names kept)
        # [Hz] Drawing rate for this node's markers, independent of loop_rate.
        # These four draws sit INSIDE the 50 Hz control loop, so with pitwall
        # connected they were adding to steering latency 50 times a second.
        # The subscriber gates make them free with pitwall closed; this caps
        # what a connected pitwall can cost. 0 disables drawing.
        self.viz_rate_hz = float(self._get_param('viz_rate_hz', 10.0))
        self._last_viz_sec = 0.0

        self.lookahead_pub = self.create_publisher(Marker, 'lookahead_point', 10)
        # steering-direction arrow on its own topic (was on lookahead_point with the
        # point marker; RViz2's singular-Marker display drops one of two ids at 100Hz)
        self.steering_pub = self.create_publisher(Marker, 'lookahead_steering', 10)
        self.future_position_pub = self.create_publisher(Marker, 'future_position', 10)
        self.trailing_pub = self.create_publisher(Marker, 'trailing_opponent_marker', 10)
        self.l1_pub = self.create_publisher(Point, 'l1_distance', 10)
        self.predict_pub = self.create_publisher(MarkerArray, '/controller_prediction/markers', 10)
        self.publish_topic = self._get_param('drive_topic', '/vesc/high_level/ackermann_cmd')
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.publish_topic, 10)
        if self.measuring:
            self.measure_pub = self.create_publisher(Float32, '/controller/latency', 10)
        if self.lateral_controller == 'MPC':
            self.create_subscription(String, self._get_param('mpc_mux_host_topic', '/simple_mux/active_host'),
                                     self.mux_host_cb, 10)
            self.mpc_time_pub = self.create_publisher(Float32, '/controller/mpc_elapsed_ms', 10)
            self.mpc_status_pub = self.create_publisher(String, '/controller/mpc_status', 10)
            self.mpc_diag_pub = self.create_publisher(String, '/controller/mpc_diagnostics', 10)
            # Output period vs computation, reported separately: wall time between
            # control-loop starts, and the whole loop's compute time (MPC solve
            # alone is /controller/mpc_elapsed_ms).
            self.mpc_period_pub = self.create_publisher(Float32, '/controller/mpc_loop_period_ms', 10)
            self.mpc_cycle_pub = self.create_publisher(Float32, '/controller/mpc_cycle_ms', 10)
            # Visualisation only: the XY trajectory the MPC actually predicted
            # (result.predicted, map frame). Throttled; never feeds control.
            self.mpc_pred_pub = self.create_publisher(Marker, '/controller/mpc_predicted_path', 1)
            self._mpc_pred_last_ns = 0
            self._mpc_pred_shown = False
            self._mpc_last_loop_start = None

        # FTG controller (params injected from /state_machine/* equivalents)
        self.ftg_controller = FTG(
            node=self,
            mapping=False,
            debug=self._get_param('ftg_debug', False),
            max_lidar_dist=self._get_param('ftg_max_lidar_dist', 10.0),
            max_speed=self._get_param('ftg_max_speed', 1.5),
            track_width=self._get_param('ftg_track_width', 2.0),
            front_fov_deg=self._get_param('ftg_front_fov_deg', 90.0),
            smooth_deg=self._get_param('ftg_smooth_deg', 1.0),
            disp_thresh=self._get_param('ftg_disp_thresh', 0.5),
            bubble_m=self._get_param('ftg_bubble_m', 0.30),
            steer_ema=self._get_param('ftg_steer_ema', 0.0),
            max_steer=self._get_param('ftg_max_steer', 0.4),
        )

        # Subscribers
        self.create_subscription(BehaviorStrategy, '/behavior_strategy', self.behavior_cb, 10)
        self.create_subscription(Odometry, '/car_state/odom', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(Odometry, '/car_state/odom_frenet', self.car_state_frenet_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/vesc/odom', self.vesc_odom_cb, 10)
        self.create_subscription(Bool, '/save_start_traj', self.save_start_traj_cb, 10)
        # global waypoints to build the FrenetConverter + Controller lazily
        self.create_subscription(WpntArray, '/global_waypoints', self.global_wpnts_cb, 10)

        # live param tuning (ROS1 dynamic_reconfigure)
        self.add_on_set_parameters_callback(self.dyn_param_cb)

        # 50 Hz control loop (gated until lazy-init done)
        self.timer = self.create_timer(1.0 / self.loop_rate, self.control_loop)
        self.get_logger().info(f"[{self.name}] up; waiting for /global_waypoints + state...")

    # ---- param helpers ----
    def _get_param(self, name, default=None):
        if not self.has_parameter(name):
            if default is None:
                self.get_logger().error(f'[{self.name}] missing required parameter: {name}')
                return None
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _num(self, name):
        # int-or-double tolerant read for the L1 params
        p = self.get_parameter(name)
        v = p.value
        return v

    ############################################ LAZY INIT ############################################
    def global_wpnts_cb(self, data: WpntArray):
        if self.controller is not None:
            return
        if len(data.wpnts) < 2:
            return
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m] for wpnt in data.wpnts])
        if self.lateral_mpc is not None and hasattr(self.lateral_mpc, 'set_global_path'):
            self.lateral_mpc.set_global_path(np.array([
                [wpnt.x_m, wpnt.y_m, wpnt.vx_mps, 0., wpnt.s_m,
                 wpnt.kappa_radpm, wpnt.psi_rad, wpnt.ax_mps2, wpnt.d_m]
                for wpnt in data.wpnts], dtype=float))
        # ROS1 read /global_republisher/track_length; derive from the waypoints' s_m
        self.track_length = data.wpnts[-1].s_m
        self.converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1])
        self.controller = Controller(
            self.t_clip_min, self.t_clip_max, self.m_l1, self.q_l1,
            self.curvature_factor,
            self.KP, self.KI, self.KD, self.heading_error_thres, self.steer_gain_for_speed,
            self.future_constant,
            self.speed_lookahead, self.lat_err_coeff, self.acc_scaler_for_steer,
            self.dec_scaler_for_steer, self.start_scale_speed, self.end_scale_speed,
            self.downscale_factor, self.speed_lookahead_for_steer,
            self.trailing_gap, self.trailing_vel_gain, self.trailing_p_gain,
            self.trailing_i_gain, self.trailing_d_gain, self.blind_trailing_speed,
            self.static_min_standoff_m, self.static_resume_hysteresis_m,
            self.static_crawl_speed_mps,
            self.loop_rate, self.wheelbase,
            self.speed_factor_for_lat_err, self.speed_factor_for_curvature,
            self.speed_diff_thres, self.start_speed, self.start_curvature_factor,
            self.AEB_thres,
            self.converter,
            predict_pub=self.predict_pub,
            logger_info=self.get_logger().info,
            logger_warn=self.get_logger().warning,
            lateral_controller=self.lateral_controller,
            lateral_mpc=self.lateral_mpc,
            pose_to_rear_axle_m=self.pose_to_rear_axle_m,
            mpc_cmd_max_lead_mps=self.mpc_cmd_max_lead_mps,
            mpc_cmd_accel_lead_mps=self.mpc_cmd_accel_lead_mps,
            mpc_speed_source=self.mpc_speed_source,
            mpc_plan_lateral_accel_limit=float(self.mpc_values['mpc_plan_lateral_accel_limit_mps2']),
            mpc_fallback_max_ticks=max(1, int(round(float(self.mpc_values['mpc_fallback_max_s']) * self.loop_rate))),
            mpc_latency_comp_s=float(self.mpc_values['mpc_latency_comp_s']),
            mpc_heading_robust=bool(self.mpc_values['mpc_heading_robust']),
            mpc_history_prediction=bool(self.mpc_values['mpc_history_prediction']),
            mpc_static_approach_decel_mps2=self.mpc_static_approach_decel_mps2,
            mpc_envelope_recovery_speed_mps=self.mpc_envelope_recovery_speed_mps,
            mpc_accel_jerk_mps3=self.mpc_accel_jerk_mps3,
        )
        self.controller.pose_prediction.clock = lambda: self.get_clock().now().nanoseconds * 1e-9
        self.controller.speed_now = self.speed_now
        self.controller.yaw_rate = self.yaw_rate
        self.get_logger().info(f"[{self.name}] initialized FrenetConverter + Controller. Ready!")

    ############################################ CALLBACKS ############################################
    def save_start_traj_cb(self, msg):
        if self.controller is not None:
            self.controller.boost_mode = True
            self.controller.cur_state_speed = self.controller.start_speed

    def scan_cb(self, data: LaserScan):
        self.scan = data

    def dyn_param_cb(self, params):
        # ROS2 replacement for /dyn_controller/parameter_updates: live-update the
        # L1 params on the manager AND the Controller instance (mirror ROS1 l1_params_cb).
        # This is an on-set-parameters callback (fires BEFORE commit), so it must NOT
        # call set_parameters() itself -> save-back is deferred to the timer.
        # Controller selection / MPC model parameters are startup-only.
        for param in params:
            if (param.name in ('lateral_controller', 'wheelbase', 'control_loop_rate_hz')
                    or param.name.startswith('mpc_')):
                return SetParametersResult(successful=False, reason='Lateral/MPC parameters require node restart')
        for param in params:
            name = param.name
            if name in L1_PARAMS:
                setattr(self, name, param.value)
                if self.controller is not None:
                    setattr(self.controller, name, param.value)
            elif name in FTG_PARAMS:
                self._apply_ftg_param(name, param.value)
            elif name == 'save_params' and param.value:
                self._save_requested = True
        return SetParametersResult(successful=True)

    def _apply_ftg_param(self, name, value):
        """Live-update an ftg_* param on the FTG instance (rqt tuning)."""
        if self.ftg_controller is None:
            return
        f = self.ftg_controller
        if name == 'ftg_debug':
            f.DEBUG = bool(value)
        elif name == 'ftg_max_lidar_dist':
            f.MAX_LIDAR_DIST = float(value)
        elif name == 'ftg_track_width':
            f.track_width = float(value)
        elif name == 'ftg_front_fov_deg':
            f.FRONT_FOV = np.radians(float(value) / 2.0)   # param = TOTAL front FOV
        elif name == 'ftg_smooth_deg':
            f.SMOOTH_RAD = np.radians(float(value))
        elif name == 'ftg_disp_thresh':
            f.DISP_THRESH = float(value)
        elif name == 'ftg_bubble_m':
            f.BUBBLE_M = float(value)
        elif name == 'ftg_steer_ema':
            f.STEER_EMA = float(value)
        elif name == 'ftg_max_steer':
            f.MAX_STEER = float(value)
        elif name == 'ftg_max_speed':
            f.MAX_SPEED = float(value)
            f.recompute_speeds()

    def save_yaml(self):
        if not self.save_yaml_path:
            self.get_logger().warn("No save_yaml_path; skipping save.")
            return
        try:
            data = {}
            if os.path.exists(self.save_yaml_path):
                with open(self.save_yaml_path, "r") as f:
                    data = yaml.safe_load(f) or {}
            params = {p: float(getattr(self, p)) for p in L1_PARAMS}
            for p in FTG_PARAMS:                      # keep FTG tuning + current values
                if self.has_parameter(p):
                    params[p] = self.get_parameter(p).value
            params['save_params'] = False
            block = data.setdefault('controller_manager', {}).setdefault('ros__parameters', {})
            block.update(params)                      # merge: don't drop other keys
            with open(self.save_yaml_path, "w") as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            self.get_logger().info(f"controller params saved to: {self.save_yaml_path}")
        except Exception as e:
            self.get_logger().error(f"failed to save controller params: {e}")

    def mux_host_cb(self, msg):
        self.mux_host = msg.data
        self.mux_host_time = self.get_clock().now()

    def odom_cb(self, data: Odometry):
        self.last_odom_time = self.get_clock().now()
        self.speed_now = data.twist.twist.linear.x
        self.speed_now_y = data.twist.twist.linear.y
        # car pose: formerly a separate /car_state/pose (PoseStamped); read it
        # straight from /car_state/odom so it works on the real car too (nothing
        # publishes /car_state/pose there — only the simulator used to).
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        theta = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                       data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
        self.position_in_map = np.array([x, y, theta])[np.newaxis]
        if self.controller is not None:
            self.controller.speed_now = self.speed_now
            # EKF already fuses bias-corrected gyro in base_link. Do not fuse the
            # legacy /imu/data sign-flipped rate or AHRS orientation a second time.
            stamp = data.header.stamp.sec + data.header.stamp.nanosec * 1e-9
            rate = data.twist.twist.angular.z
            if data.twist.covariance[35] < 0:
                rate = float('nan')
            self.controller.pose_prediction.observe(stamp, rate)
        self.ftg_controller.set_vel(data.twist.twist.linear.x)

    def vesc_odom_cb(self, data: Odometry):
        self.wheelspeed_now = data.twist.twist.linear.x
        self.ftg_controller.set_vel(data.twist.twist.linear.x)

    def car_state_frenet_cb(self, data: Odometry):
        s = data.pose.pose.position.x
        d = data.pose.pose.position.y
        vs = data.twist.twist.linear.x
        vd = data.twist.twist.linear.y
        self.position_in_map_frenet = np.array([s, d, vs, vd])

    def behavior_cb(self, data: BehaviorStrategy):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if len(data.trailing_targets) != 0:
            opponent = data.trailing_targets[0]
            opponent_s = opponent.s_center
            opponent_d = opponent.d_center
            opponent_vs = opponent.vs
            opponent_visible = opponent.is_visible
            opponent_static = opponent.is_static
            self.opponent = [opponent_s, opponent_d, opponent_vs, opponent_static, opponent_visible]
            if opponent_static:
                self.last_static_opponent = self.opponent.copy()
                self.last_static_opponent_time_sec = now_sec
            else:
                self.last_static_opponent = None
                self.last_static_opponent_time_sec = None
        else:
            static_target_age = (
                None if self.last_static_opponent_time_sec is None
                else now_sec - self.last_static_opponent_time_sec
            )
            if (
                data.state == "TRAILING"
                and self.last_static_opponent is not None
                and static_target_age is not None
                and static_target_age <= self.static_target_hold_sec
            ):
                # A short tracker/state-machine dropout must not turn TRAILING
                # into a global-speed command for one frame.
                self.opponent = self.last_static_opponent.copy()
            else:
                self.opponent = None
                if data.state != "TRAILING":
                    self.last_static_opponent = None
                    self.last_static_opponent_time_sec = None

        self.waypoint_list_in_map = []

        for waypoint in data.local_wpnts:
            waypoint_in_map = [waypoint.x_m, waypoint.y_m]
            speed = waypoint.vx_mps
            if waypoint.d_right + waypoint.d_left != 0:
                self.waypoint_list_in_map.append([waypoint_in_map[0],
                                                  waypoint_in_map[1],
                                                  speed,
                                                  min(waypoint.d_left, waypoint.d_right)/(waypoint.d_right + waypoint.d_left),
                                                  waypoint.s_m, waypoint.kappa_radpm, waypoint.psi_rad, waypoint.ax_mps2, waypoint.d_m]
                                                 )
            else:
                self.waypoint_list_in_map.append([waypoint_in_map[0], waypoint_in_map[1], speed, 0, waypoint.s_m, waypoint.kappa_radpm, waypoint.psi_rad, waypoint.ax_mps2, waypoint.d_m])
        self.waypoint_array_in_map = np.array(self.waypoint_list_in_map)
        self.waypoint_safety_counter = 0
        self.state = data.state

    def imu_cb(self, data):
        self.acc_now[1:] = self.acc_now[:-1]
        self.acc_now[0] = -data.linear_acceleration.x  # vesc is rotated 90 deg, so (-acc_y) == (long_acc)

        self.yaw_rate = -data.angular_velocity.z  # vesc is rotated 90 deg, so (-acc_y) == (long_acc)
        if self.controller is not None:
            self.controller.yaw_rate = self.yaw_rate

    ############################################ MAIN LOOP ############################################
    def control_loop(self):
        # save-back requested via the param callback (done here, outside the
        # on-set-parameters callback, so set_parameters() is safe).
        if self._save_requested:
            self._save_requested = False
            self.save_yaml()
            self.set_parameters([rclpy.parameter.Parameter(
                'save_params', rclpy.Parameter.Type.BOOL, False)])

        if self.mapping:
            self.mapping_loop()
            return
        # gate until lazy-init + first inputs
        if self.controller is None or self.waypoint_array_in_map is None or len(self.position_in_map) == 0 or len(self.position_in_map_frenet) == 0:
            return

        if self.measuring:
            start = time.perf_counter()
        if self.lateral_controller == 'MPC':
            loop_start = time.perf_counter()
            if self._mpc_last_loop_start is not None:
                self.mpc_period_pub.publish(Float32(data=float(1000 * (loop_start - self._mpc_last_loop_start))))
            self._mpc_last_loop_start = loop_start
        speed, acceleration, jerk, steering_angle = 0, 0, 0, 0

        # Logic to select controller
        if self.state != "FTGONLY":
            speed, acceleration, jerk, steering_angle = self.controller_cycle()
        else:
            speed, steering_angle = self.ftg_cycle()

        if self.lateral_controller == 'MPC':
            speed, acceleration = self.mpc_output_guard(speed, acceleration)

        ack_msg = self.create_ack_msg(speed, acceleration, jerk, steering_angle)
        self.drive_pub.publish(ack_msg)
        if self.lateral_controller == 'MPC':
            self.controller.record_applied_steering(steering_angle)
            self.mpc_cycle_pub.publish(Float32(data=float(1000 * (time.perf_counter() - loop_start))))
        if self.measuring:
            end = time.perf_counter()
            msg = Float32()
            msg.data = float(1/(end-start))
            self.measure_pub.publish(msg)

    def mpc_output_guard(self, speed, acceleration):
        """MPC mode only: stop on stale odom, publish solver status, and restart the
        integrated speed setpoint whenever anything other than a solved MPC
        command (FTG, waypoint-loss stop, STOP request) drives the car."""
        odom_age = (float('inf') if self.last_odom_time is None
                    else (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9)
        if odom_age > self.mpc_odom_timeout_s:
            self.get_logger().error(f"[{self.name}] [MPC] odom stale ({odom_age:.2f}s). STOPPING!!",
                                    throttle_duration_sec=0.5)
            speed, acceleration = 0.0, 0.0
        host_age = (float('inf') if self.mux_host_time is None
                    else (self.get_clock().now() - self.mux_host_time).nanoseconds / 1e9)
        if self.mux_host != 'autodrive' or host_age > self.mpc_mux_host_timeout_s:
            # Not engaged (RB not selected / LB human / mux status lost): keep the
            # setpoint at 0 so engaging starts from standstill with limited accel.
            speed, acceleration = 0.0, 0.0
        result = self.controller.mpc_result
        if self.state == "FTGONLY" or speed <= 0.0:
            self.controller.reset_mpc_speed_command()
        if result is not None and self.state != "FTGONLY":
            self.mpc_time_pub.publish(Float32(data=float(1000 * result.elapsed_s)))
            self.mpc_status_pub.publish(String(data=str(result.status)))
            if self.controller.mpc_speed_source == 'integrated':
                diagnostic = dict(self.controller.mpc_speed_diagnostics)
                diagnostic.update(stamp_ns=self.get_clock().now().nanoseconds,
                                  state=str(self.state), guarded_speed_mps=float(speed),
                                  steering_rad=float(self.controller.curr_applied_steering), iterations=int(result.iterations),
                                  solver_ms=float(result.solver_s * 1000),
                                  status=str(result.status))
                diagnostic = {k: (None if isinstance(v, float) and not np.isfinite(v) else v)
                              for k, v in diagnostic.items()}
                self.mpc_diag_pub.publish(String(data=json.dumps(diagnostic, allow_nan=False)))
        self._publish_mpc_prediction(result)
        return speed, acceleration

    def _publish_mpc_prediction(self, result):
        """RViz marker of the MPC's predicted XY path (20 Hz max). Cleared when
        there is no valid prediction so a stale line is never shown."""
        try:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._mpc_pred_last_ns < 50_000_000:
                return
            self._mpc_pred_last_ns = now_ns
            pred = None if result is None or self.state == "FTGONLY" or not result.success \
                else getattr(result, 'predicted', None)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id = "mpc_predicted_path", 0
            if pred is None or len(pred) < 2 or not np.isfinite(np.asarray(pred)[:, :2]).all():
                if self._mpc_pred_shown:
                    marker.action = Marker.DELETE
                    self.mpc_pred_pub.publish(marker)
                    self._mpc_pred_shown = False
                return
            marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.04
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 1.0, 1.0
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=0.05) for p in np.asarray(pred)]
            self.mpc_pred_pub.publish(marker)
            self._mpc_pred_shown = True
        except Exception as exc:  # visualisation must never affect control
            self.get_logger().warn(f"[MPC] prediction marker skipped: {exc}", throttle_duration_sec=5.0)

    def mapping_loop(self):
        if self.scan is None:
            return
        speed, acceleration, jerk, steering_angle = 0, 0, 0, 0
        speed, steering_angle = self.ftg_controller.process_lidar(
            self.scan.ranges, self.scan.angle_min, self.scan.angle_increment)
        ack_msg = self.create_ack_msg(speed, acceleration, jerk, steering_angle)
        self.drive_pub.publish(ack_msg)

    def controller_cycle(self):
        speed, acceleration, jerk, steering_angle, L1_point, L1_distance, idx_nearest_waypoint, curvature_waypoints, future_position = self.controller.main_loop(
            self.state,
            self.position_in_map,
            self.waypoint_array_in_map,
            self.speed_now,
            self.opponent,
            self.position_in_map_frenet,
            self.acc_now,
            self.track_length)

        # Viz is rate-limited as a group: one clock check instead of four, and
        # the whole block is skipped on most control ticks. Each function still
        # has its own subscriber gate for the pitwall-closed case.
        if self.viz_rate_hz > 0.0:
            _now = time.monotonic()
            if _now - self._last_viz_sec >= 1.0 / self.viz_rate_hz:
                self._last_viz_sec = _now
                self.set_lookahead_marker(L1_point, 100)
                self.visualize_steering(steering_angle)
                self.visualize_trailing_opponent()
                self.viz_future_position(future_position, 200)

        self.curvature_waypoints = curvature_waypoints
        self.l1_pub.publish(Point(x=float(idx_nearest_waypoint), y=float(L1_distance), z=float(self.curvature_waypoints)))

        self.waypoint_safety_counter += 1
        if self.waypoint_safety_counter >= self.loop_rate/self.state_machine_rate * 10:
            self.get_logger().error(f"[{self.name}] Received no local wpnts. STOPPING!!", throttle_duration_sec=0.5)
            speed = 0
            steering_angle = 0

        return speed, acceleration, jerk, steering_angle

    def ftg_cycle(self):
        speed, steer = self.ftg_controller.process_lidar(
            self.scan.ranges, self.scan.angle_min, self.scan.angle_increment)
        self.get_logger().warning(f"[{self.name}] FTGONLY!!!")
        return speed, steer

    def create_ack_msg(self, speed, acceleration, jerk, steering_angle):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.get_clock().now().to_msg()
        ack_msg.header.frame_id = 'base_link'
        ack_msg.drive.steering_angle = float(steering_angle)
       
        ack_msg.drive.speed = float(speed)  # 초기값 ack_msg.drive.speed = float(speed) -> float(min(speed,3.5s))
        ack_msg.drive.jerk = float(jerk)
        ack_msg.drive.acceleration = float(acceleration)
        return ack_msg

    ############################################ VIZ ############################################
    def visualize_steering(self, theta):
        # Viz only, and this runs INSIDE the 50 Hz control loop -- every
        # microsecond here is added to the steering latency. Skipped when
        # nothing is subscribed; unchanged when pitwall is open.
        if self.steering_pub.get_subscription_count() == 0:
            return
        quaternions = quaternion_from_euler(0, 0, theta)

        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "base_link"
        lookahead_marker.header.stamp = self.get_clock().now().to_msg()
        lookahead_marker.type = Marker.ARROW
        lookahead_marker.id = 50
        lookahead_marker.scale.x = 0.6
        lookahead_marker.scale.y = 0.05
        lookahead_marker.scale.z = 0.05  # nonzero so RViz doesn't warn "scale of 0"
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.pose.position.x = 0.0
        lookahead_marker.pose.position.y = 0.0
        lookahead_marker.pose.position.z = 0.0
        lookahead_marker.pose.orientation.x = quaternions[0]
        lookahead_marker.pose.orientation.y = quaternions[1]
        lookahead_marker.pose.orientation.z = quaternions[2]
        lookahead_marker.pose.orientation.w = quaternions[3]
        self.steering_pub.publish(lookahead_marker)

    def set_lookahead_marker(self, lookahead_point, id):
        # Viz only, and this runs INSIDE the 50 Hz control loop -- every
        # microsecond here is added to the steering latency. Skipped when
        # nothing is subscribed; unchanged when pitwall is open.
        if self.lookahead_pub.get_subscription_count() == 0:
            return
        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "map"
        lookahead_marker.header.stamp = self.get_clock().now().to_msg()
        lookahead_marker.type = 2
        lookahead_marker.id = id
        lookahead_marker.scale.x = 0.35
        lookahead_marker.scale.y = 0.35
        lookahead_marker.scale.z = 0.35
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.pose.position.x = float(lookahead_point[0])
        lookahead_marker.pose.position.y = float(lookahead_point[1])
        lookahead_marker.pose.position.z = 0.0
        lookahead_marker.pose.orientation.x = 0.0
        lookahead_marker.pose.orientation.y = 0.0
        lookahead_marker.pose.orientation.z = 0.0
        lookahead_marker.pose.orientation.w = 1.0
        self.lookahead_pub.publish(lookahead_marker)

    def viz_future_position(self, future_position, id):
        # Viz only, and this runs INSIDE the 50 Hz control loop -- every
        # microsecond here is added to the steering latency. Skipped when
        # nothing is subscribed; unchanged when pitwall is open.
        if self.future_position_pub.get_subscription_count() == 0:
            return
        quaternions = quaternion_from_euler(0, 0, future_position[0, 2])

        future_position_marker = Marker()
        future_position_marker.header.frame_id = "map"
        future_position_marker.header.stamp = self.get_clock().now().to_msg()
        future_position_marker.type = Marker.ARROW
        future_position_marker.id = id
        future_position_marker.scale.x = 1.2
        future_position_marker.scale.y = 0.06
        future_position_marker.scale.z = 0.06  # nonzero so RViz doesn't warn "scale of 0"
        future_position_marker.color.r = 0.5
        future_position_marker.color.g = 0.0
        future_position_marker.color.b = 0.5
        future_position_marker.color.a = 1.0
        future_position_marker.pose.position.x = float(future_position[0, 0])
        future_position_marker.pose.position.y = float(future_position[0, 1])
        future_position_marker.pose.position.z = 0.0
        future_position_marker.pose.orientation.x = quaternions[0]
        future_position_marker.pose.orientation.y = quaternions[1]
        future_position_marker.pose.orientation.z = quaternions[2]
        future_position_marker.pose.orientation.w = quaternions[3]
        self.future_position_pub.publish(future_position_marker)

    def visualize_trailing_opponent(self):
        # Viz only, and this runs INSIDE the 50 Hz control loop -- every
        # microsecond here is added to the steering latency. Skipped when
        # nothing is subscribed; unchanged when pitwall is open.
        if self.trailing_pub.get_subscription_count() == 0:
            return
        if (self.state == "TRAILING" and (self.opponent is not None)):
            on = True
        else:
            on = False
        opponent_marker = Marker()
        opponent_marker.header.frame_id = "map"
        opponent_marker.header.stamp = self.get_clock().now().to_msg()
        opponent_marker.type = 2
        opponent_marker.scale.x = 0.3
        opponent_marker.scale.y = 0.3
        opponent_marker.scale.z = 0.3
        opponent_marker.color.r = 1.0
        opponent_marker.color.g = 0.0
        opponent_marker.color.b = 0.0
        opponent_marker.color.a = 1.0
        if self.opponent is not None:
            pos = self.converter.get_cartesian([self.opponent[0]], [self.opponent[1]])
            # ROS2 FrenetConverter returns a (2, N) array; pull the scalar out.
            opponent_marker.pose.position.x = float(pos[0][0])
            opponent_marker.pose.position.y = float(pos[1][0])
            opponent_marker.pose.position.z = 0.0

        opponent_marker.pose.orientation.x = 0.0
        opponent_marker.pose.orientation.y = 0.0
        opponent_marker.pose.orientation.z = 0.0
        opponent_marker.pose.orientation.w = 1.0
        if on == False:
            opponent_marker.action = Marker.DELETE
        self.trailing_pub.publish(opponent_marker)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
