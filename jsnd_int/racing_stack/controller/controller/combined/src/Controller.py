#!/usr/bin/env python3

import logging

import numpy as np

from visualization_msgs.msg import Marker, MarkerArray

from controller.mpc.lateral_mpc import MPCResult, SafetyRequest
from controller.mpc.steering_mpc import ENVELOPE_REASONS


class Controller:
    """This class implements the L1 / Pure-Pursuit controller for autonomous driving.
    Input and output topics are managed by the controller manager.

    ROS2 port note: the ROS1 MAP (steering-lookup) branch was intentionally removed;
    only Pure-Pursuit lateral control remains. The marker publisher and loggers are
    injected by the manager so this class stays ROS-node-free.
    """

    def __init__(self,
                t_clip_min,
                t_clip_max,
                m_l1,
                q_l1,

                curvature_factor,

                KP,
                KI,
                KD,
                heading_error_thres,
                steer_gain_for_speed,

                future_constant,

                speed_lookahead,
                lat_err_coeff,
                acc_scaler_for_steer,
                dec_scaler_for_steer,
                start_scale_speed,
                end_scale_speed,
                downscale_factor,
                speed_lookahead_for_steer,

                trailing_gap,
                trailing_vel_gain,
                trailing_p_gain,
                trailing_i_gain,
                trailing_d_gain,
                blind_trailing_speed,
                static_min_standoff_m,
                static_resume_hysteresis_m,
                static_crawl_speed_mps,

                loop_rate,
                wheelbase,

                speed_factor_for_lat_err,
                speed_factor_for_curvature,

                speed_diff_thres,
                start_speed,
                start_curvature_factor,

                AEB_thres,

                converter,

                predict_pub=None,
                logger_info=logging.info,
                logger_warn=logging.warning,
                lateral_controller="PP",
                lateral_mpc=None,
                pose_to_rear_axle_m=0.0,
                mpc_cmd_max_lead_mps=0.5,
                mpc_speed_source="integrated",
                mpc_plan_lateral_accel_limit=4.5,
                mpc_fallback_max_ticks=10,
                mpc_latency_comp_s=0.0,
            ):

        # Parameters from manager
        self.t_clip_min = t_clip_min
        self.t_clip_max = t_clip_max
        self.m_l1 = m_l1
        self.q_l1 = q_l1
        self.speed_lookahead = speed_lookahead
        self.lat_err_coeff = lat_err_coeff
        self.acc_scaler_for_steer = acc_scaler_for_steer
        self.dec_scaler_for_steer = dec_scaler_for_steer
        self.start_scale_speed = start_scale_speed
        self.end_scale_speed = end_scale_speed
        self.downscale_factor = downscale_factor
        self.speed_lookahead_for_steer = speed_lookahead_for_steer

        # marker publisher injected by the manager (ROS2: a Node-created publisher)
        self.predict_pub = predict_pub

        # L1 dist calc param
        self.curvature_factor = curvature_factor

        self.speed_factor_for_lat_err = speed_factor_for_lat_err
        self.speed_factor_for_curvature = speed_factor_for_curvature

        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.heading_error_thres = heading_error_thres
        self.steer_gain_for_speed = steer_gain_for_speed

        self.future_constant = future_constant

        self.trailing_gap = trailing_gap
        self.trailing_vel_gain = trailing_vel_gain
        self.trailing_p_gain = trailing_p_gain
        self.trailing_i_gain = trailing_i_gain
        self.trailing_d_gain = trailing_d_gain
        self.blind_trailing_speed = blind_trailing_speed
        self.static_min_standoff_m = static_min_standoff_m
        self.static_resume_hysteresis_m = static_resume_hysteresis_m
        self.static_crawl_speed_mps = static_crawl_speed_mps

        self.loop_rate = loop_rate
        self.AEB_thres = AEB_thres
        self.converter = converter

        # Parameters in the controller
        self.curr_steering_angle = 0
        self.idx_nearest_waypoint = None  # index of nearest waypoint to car
        self.track_length = None
        self.gap = None
        self.gap_should = None
        self.gap_error = None
        self.gap_actual = None
        self.v_diff = None
        self.i_gap = 0
        self.trailing_command = 2
        self.static_stop_latched = False
        self.speed_command = None
        self.last_valid_speed = 0
        self.curvature_waypoints = 0
        self.current_steer_command = 0
        self.yaw_rate = 0

        self.logger_info = logger_info
        self.logger_warn = logger_warn

        self.speed_diff_thres = speed_diff_thres
        self.start_speed = start_speed
        self.start_curvature_factor = start_curvature_factor

        self.wheelbase = wheelbase

        self.start_mode = False
        self.future_lat_err = 0.0
        self.future_lat_e_norm = 0.0
        self.boost_mode = False

        # ---- Integrated (speed + steering) MPC, selected at startup only ----
        if lateral_controller not in ("PP", "MPC"):
            raise ValueError("lateral_controller must be PP or MPC")
        if lateral_controller == "MPC" and lateral_mpc is None:
            raise ValueError("MPC selected without a configured solver")
        self.lateral_controller = lateral_controller
        self.lateral_mpc = lateral_mpc
        self.pose_to_rear_axle_m = pose_to_rear_axle_m
        self.mpc_cmd_max_lead_mps = float(mpc_cmd_max_lead_mps)
        self.mpc_result = None
        self.mpc_trailing_cap = None
        self.curr_applied_steering = 0.0
        self._mpc_last_status = None
        self.mpc_speed_cmd = None  # VESC speed setpoint integrated from MPC acceleration
        # Step 23: 'jsnd' = legacy JSND/PP speed pipeline + steering-only MPC (SteeringMPC);
        # 'integrated' = previous joint speed+steering IntegratedMPC.
        if mpc_speed_source not in ("jsnd", "integrated"):
            raise ValueError("mpc_speed_source must be jsnd or integrated")
        self.mpc_speed_source = mpc_speed_source
        self.mpc_plan_lateral_accel_limit = float(mpc_plan_lateral_accel_limit)
        self.mpc_fallback_max_ticks = int(mpc_fallback_max_ticks)
        # Pose/actuation dead time the MPC state is predicted over (PP: future_constant).
        self.mpc_latency_comp_s = max(0.0, float(mpc_latency_comp_s))
        self.mpc_fallback_ticks = 0
        self.mpc_stop_ticks = 0           # integrated: consecutive STOP ticks (soft-stop window)
        self.mpc_lat_cap_active = False   # physical a_lat check bound the plan speed this tick
        self.mpc_profile_cap_active = False  # mpc_profile operator ceiling bound this tick
        self.mpc_pp_steering = 0.0        # legacy PP steering of this tick (fallback/diagnosis)

    # ============================ Integrated MPC ============================
    STANDSTILL_NOISE_MPS = 0.02

    def reset_mpc_speed_command(self):
        """Next MPC command restarts from the measured speed (stop, FTG, input loss)."""
        self.mpc_speed_cmd = None

    def record_applied_steering(self, steering):
        """Manager feedback of the steering actually published (FTG / input-loss stop)."""
        if self.lateral_controller == "MPC" and np.isfinite(steering):
            limit = self.lateral_mpc.config.max_steer
            self.curr_steering_angle = float(np.clip(steering, -limit, limit))
            self.current_steer_command = self.curr_steering_angle
            self.curr_applied_steering = self.curr_steering_angle

    def _report_mpc_result(self):
        if self.mpc_result.status != self._mpc_last_status:
            if self.mpc_result.success and self.mpc_result.status != 'solved':
                self.logger_warn(f"[MPC] {self.mpc_result.status}")  # bounded plan continuation
            elif self.mpc_result.success:
                self.logger_info("[MPC] tracking local path")
            else:
                self.logger_warn(f"[MPC] STOP requested: {self.mpc_result.status}")
            self._mpc_last_status = self.mpc_result.status

    def _mpc_speed_command(self, result, measured):
        """Convert the MPC acceleration into the VESC speed setpoint.

        The solver keeps using the measured odom speed as feedback. The setpoint,
        however, is integrated from the previous setpoint: v_cmd += a*dt. Using
        v_meas + a*dt (the solver's own first prediction) never leaves standstill,
        because 0 + 1.5*0.025 = 0.04 m/s (~170 ERPM) keeps the wheels still.
        Bounds: never above the speed the MPC plan itself reaches in its horizon
        (already within the profile/curvature/max_speed caps), never more than
        mpc_cmd_max_lead_mps above the measured speed (anti-windup), never < 0.
        """
        if not result.success:
            # Soft STOP (step 32): a STOP while moving ramps the setpoint down at the MPC's
            # own max_decel for at most mpc_fallback_max_ticks, instead of a 0 setpoint
            # (VESC current braking -> stutter on every one-tick solver miss). A STOP that
            # lasts longer, or starts at standstill, commands 0 as before. simple_mux
            # estop/TTC and the not-engaged guard still force 0 downstream.
            if self._mpc_soft_stop(measured):
                cfg = self.lateral_mpc.config
                self.mpc_speed_cmd = max(0.0, self.mpc_speed_cmd - cfg.max_decel * cfg.dt)
                return self.mpc_speed_cmd
            self.reset_mpc_speed_command()
            return 0.0
        base = measured if self.mpc_speed_cmd is None else self.mpc_speed_cmd
        # Step 34: symmetric anti-windup. A soft-STOP ramp (or a lagging setpoint) must
        # not leave the base far below the measured speed: otherwise every solver miss
        # is a permanent speed loss recovered only at max_accel (cumulative slowdown
        # seen in step 32 vs 31). The MPC's own braking still acts through a*dt.
        base = max(base, measured - self.mpc_cmd_max_lead_mps)
        command = base + result.acceleration * self.lateral_mpc.config.dt
        upper = min(float(np.max(result.predicted[:, 3])), measured + self.mpc_cmd_max_lead_mps,
                    self.lateral_mpc.config.max_speed)  # hard cap: remove solver tolerance
        self.mpc_speed_cmd = float(np.clip(command, 0.0, max(0.0, upper)))
        return self.mpc_speed_cmd

    # Behaviour/input STOPs keep the immediate 0 setpoint (state-machine semantics unchanged).
    HARD_STOP_PREFIXES = ('trailing_without_target', 'unsupported_behavior', 'invalid_local_control_input')

    def _mpc_soft_stop(self, measured):
        status = str(getattr(self.mpc_result, 'status', ''))
        return (not status.startswith(self.HARD_STOP_PREFIXES) and self.mpc_stop_ticks <= self.mpc_fallback_max_ticks and self.mpc_speed_cmd is not None
                and self.mpc_speed_cmd > 0.0 and measured > self.STANDSTILL_NOISE_MPS)

    def _mpc_recenter_steering(self):
        """Steering while the MPC requests STOP: move the last applied steering toward 0
        at the configured rate limit. Holding it forever (previous behaviour) kept the
        servo off-centre and fed the same stale steering back into every new solve."""
        cfg = self.lateral_mpc.config
        step = cfg.max_steer_rate * cfg.dt
        current = float(self.curr_steering_angle) if np.isfinite(self.curr_steering_angle) else 0.0
        current = float(np.clip(current, -cfg.max_steer, cfg.max_steer))
        return current - min(max(current, -step), step)

    def _mpc_trailing_cap(self, opponent, frenet, speed, track_length):
        """TRAILING speed cap for the MPC: the same trailing controller PP uses (gap PID to
        the target, static-obstacle standoff stop / crawl, blind-trailing floor), with the
        MPC max speed as the global speed. No target -> None: the caller requests STOP,
        as PP commands 0 when TRAILING has lost its target."""
        if opponent is None or frenet is None or not track_length:
            self.i_gap = 0
            return None
        self.opponent = opponent
        self.position_in_map_frenet = frenet
        self.speed_now = max(0.0, float(speed))
        self.track_length = track_length
        cap = float(self.trailing_controller(self.lateral_mpc.config.max_speed))
        return cap if np.isfinite(cap) else 0.0

    def integrated_loop(self, state, position, waypoints, speed, opponent=None, frenet=None, track_length=None):
        # Entire PP speed/steering preprocessing is bypassed. Unsupported behavior
        # states stop rather than silently losing opponent-following safeguards.
        # TRAILING: the MPC tracks the local path with every waypoint speed cap clipped
        # to the PP trailing-controller command (target gap / static standoff).
        measured = 0.0
        try:
            position = np.asarray(position, dtype=float)
            waypoints = np.asarray(waypoints, dtype=float)
            if position.shape != (1, 3):
                raise ValueError('invalid pose shape')
            measured = float(speed)
            if not np.isfinite(measured) or measured < -self.STANDSTILL_NOISE_MPS:
                raise ValueError('invalid_or_reverse_speed')
            measured = max(0.0, measured)  # clamp only standstill odom jitter
            rear = position[0].copy()
            rear[:2] += self.pose_to_rear_axle_m * np.array([np.cos(rear[2]), np.sin(rear[2])])
            cfg = self.lateral_mpc.config
            for _ in range(int(round(self.mpc_latency_comp_s / cfg.dt))):
                rear += cfg.dt * measured * np.array([np.cos(rear[2]), np.sin(rear[2]),
                                                      np.tan(self.curr_applied_steering)/cfg.wheelbase])
            if state == 'TRAILING':
                cap = self._mpc_trailing_cap(opponent, frenet, measured, track_length)
                self.mpc_trailing_cap = cap
                if cap is None:
                    result = self.lateral_mpc.reject('trailing_without_target', self.curr_steering_angle)
                else:
                    result = self.lateral_mpc.solve(np.r_[rear, measured], waypoints,
                                                    self.curr_steering_angle, safety_speed_cap=cap)
            else:
                self.mpc_trailing_cap = None
                self.i_gap = 0
                self.static_stop_latched = False
                if state not in ('GB_TRACK', 'START', 'OVERTAKE', 'RECOVERY'):
                    result = self.lateral_mpc.reject('unsupported_behavior:'+str(state), self.curr_steering_angle)
                else:
                    result = self.lateral_mpc.solve(np.r_[rear, measured], waypoints, self.curr_steering_angle)
        except (ValueError, IndexError, TypeError) as exc:
            # Reason kept in the status (e.g. invalid_or_reverse_speed) so the log
            # tells input loss from a reverse/odom-undershoot after hard braking.
            rear = np.zeros(3)
            result = self.lateral_mpc.reject('invalid_local_control_input:'+str(exc), self.curr_steering_angle)
        self.mpc_result = result
        self._report_mpc_result()
        self.mpc_stop_ticks = 0 if result.success else self.mpc_stop_ticks + 1
        if result.success:
            steering = result.steering
        elif self._mpc_soft_stop(measured):
            steering = float(self.curr_steering_angle)  # hold during the soft-stop ramp (no corner jolt)
        else:
            steering = self._mpc_recenter_steering()
        self.record_applied_steering(steering)
        speed_cmd = self._mpc_speed_command(result, measured)
        if self.mpc_trailing_cap is not None:
            speed_cmd = min(speed_cmd, self.mpc_trailing_cap)
            self.mpc_speed_cmd = speed_cmd
        point = result.reference[-1, :2] if result.success else rear[:2]
        nearest = int(np.argmin(np.linalg.norm(waypoints[:, :2]-rear[:2], axis=1))) if result.success else 0
        return (speed_cmd, result.acceleration, 0., steering,
                point, float(np.linalg.norm(point-rear[:2])), nearest, 0., rear.reshape(1, 3))

    def _plan_lateral_accel_check(self, waypoints):
        """Physical lateral-acceleration limit applied to the PLAN, point by point:
        v_i <= sqrt(a_lat_max / |kappa_i|) with the planner's own kappa (column 5). The
        limit is the ggv ay_max the raceline was optimised with, so for a feasible plan
        this is a no-op (bit-identical waypoints); it only acts on a physically
        infeasible plan (e.g. sector scaling > 1, an avoidance spline). Returns the
        (possibly) clipped copy; self.mpc_lat_cap_active says whether it acted."""
        w = np.asarray(waypoints, dtype=float)
        kappa = np.abs(w[:, 5])
        with np.errstate(divide='ignore'):
            cap = np.where(kappa > 1e-6, np.sqrt(self.mpc_plan_lateral_accel_limit / np.maximum(kappa, 1e-12)), np.inf)
        over = w[:, 2] > cap + 0.01
        self.mpc_lat_cap_active = bool(np.any(over))
        if not self.mpc_lat_cap_active:
            return waypoints
        w = w.copy()
        w[over, 2] = cap[over]
        return w

    def jsnd_mpc_loop(self, state, position_in_map, waypoint_array_in_map, speed_now, opponent,
                      position_in_map_frenet, acc_now, track_length):
        """Step 23 architecture: the legacy JSND controller pipeline runs unchanged and
        decides the SPEED (raceline x sector scaling via /local_waypoints, lookahead speed,
        lateral/heading error scaling, trailing controller, AEB, all states). The MPC
        replaces only the PP steering. STOP only on: path/heading envelope violation,
        invalid input, or solver misses longer than mpc_fallback_max_ticks (in between,
        the legacy PP steering of this tick is used and the status says so)."""
        waypoint_array_in_map = self._plan_lateral_accel_check(waypoint_array_in_map)
        out = list(self._pp_main_loop(state, position_in_map, waypoint_array_in_map, speed_now,
                                      opponent, position_in_map_frenet, acc_now, track_length))
        speed, pp_steer = float(out[0]), float(out[3])
        self.mpc_pp_steering = pp_steer
        # Operator speed ceiling of the selected mpc_profile (lowspeed 1.0 ... 5p0 5.0), kept
        # as the explicit staged-ramp limit it was; above the plan's maximum it never binds.
        self.mpc_profile_cap_active = speed > self.lateral_mpc.config.max_speed + 1e-6
        speed = min(speed, self.lateral_mpc.config.max_speed)
        cfg = self.lateral_mpc.config
        prev = float(np.clip(self.curr_applied_steering, -cfg.max_steer, cfg.max_steer))
        try:
            pose = np.asarray(position_in_map, dtype=float)[0].copy()
            pose[:2] += self.pose_to_rear_axle_m * np.array([np.cos(pose[2]), np.sin(pose[2])])
            measured = float(speed_now)
            if not np.isfinite(measured) or measured < -self.STANDSTILL_NOISE_MPS:
                raise ValueError('invalid_or_reverse_speed')
            # Dead-time compensation: the steering-only MPC has no PP-style lookahead, so a
            # stale pose (PF/odom latency) makes it chase the past and diverge. Predict the
            # rear axle over mpc_latency_comp_s with the applied steering (kinematic bicycle).
            v_now = max(0.0, measured)
            for _ in range(int(round(self.mpc_latency_comp_s / cfg.dt))):
                pose += cfg.dt * v_now * np.array([np.cos(pose[2]), np.sin(pose[2]), np.tan(prev) / cfg.wheelbase])
            result = self.lateral_mpc.solve(np.r_[pose, max(0.0, measured)],
                                            np.asarray(waypoint_array_in_map, dtype=float), prev)
        except (ValueError, IndexError, TypeError) as exc:
            result = self.lateral_mpc.reject('invalid_local_control_input:' + str(exc), prev)
        if result.success:
            self.mpc_fallback_ticks = 0
            steering = result.steering
        elif (result.status not in ENVELOPE_REASONS
              and not result.status.startswith('invalid_local_control_input')
              and self.mpc_fallback_ticks < self.mpc_fallback_max_ticks):
            # Transient compute/solver miss: legacy PP steering (rate/limit as the MPC).
            self.mpc_fallback_ticks += 1
            step = cfg.max_steer_rate * cfg.dt
            steering = float(np.clip(pp_steer, max(-cfg.max_steer, prev - step), min(cfg.max_steer, prev + step)))
            result = MPCResult(steering, SafetyRequest.NONE, 'pp_fallback:' + result.status, result.elapsed_s)
        else:
            # STOP. The fallback budget is NOT refilled here: it refills only on a solved
            # MPC tick, so an exhausted budget cannot alternate STOP / PP-fallback.
            steering = self._mpc_recenter_steering_from(prev)
            speed = 0.0
            if result.status != self._mpc_last_status:   # diagnosis: log has no steering otherwise
                p0 = np.asarray(position_in_map, dtype=float)[0]
                self.logger_warn(f"[MPC] STOP detail: pose=({p0[0]:.2f},{p0[1]:.2f},{p0[2]:.3f}) "
                                 f"v={float(speed_now):.2f} prev_steer={prev:+.3f} pp_steer={pp_steer:+.3f} "
                                 f"latency_comp={self.mpc_latency_comp_s:.2f}s")
        self.mpc_result = result
        self._report_mpc_result()
        self.record_applied_steering(steering)
        out[0], out[3] = speed, steering
        return tuple(out)

    def _mpc_recenter_steering_from(self, current):
        cfg = self.lateral_mpc.config
        step = cfg.max_steer_rate * cfg.dt
        return current - min(max(current, -step), step)

    def main_loop(self, state, position_in_map, waypoint_array_in_map, speed_now, opponent, position_in_map_frenet, acc_now, track_length):
        if self.lateral_controller == "MPC":
            if self.mpc_speed_source == "jsnd":
                return self.jsnd_mpc_loop(state, position_in_map, waypoint_array_in_map, speed_now,
                                          opponent, position_in_map_frenet, acc_now, track_length)
            return self.integrated_loop(state, position_in_map, waypoint_array_in_map, speed_now,
                                        opponent, position_in_map_frenet, track_length)
        return self._pp_main_loop(state, position_in_map, waypoint_array_in_map, speed_now, opponent,
                                  position_in_map_frenet, acc_now, track_length)

    def _pp_main_loop(self, state, position_in_map, waypoint_array_in_map, speed_now, opponent, position_in_map_frenet, acc_now, track_length):
        # Legacy JSND PP loop, unchanged (also the speed/fallback source of jsnd_mpc_loop).
        # Updating parameters from manager
        self.state = state
        self.position_in_map = position_in_map

        #-------------------------------Future Position-----------------------------
        self.future_position = np.zeros((1, 3))
        #-------------------------------Future Position-----------------------------

        self.waypoint_array_in_map = waypoint_array_in_map
        self.speed_now = speed_now
        self.opponent = opponent
        self.position_in_map_frenet = position_in_map_frenet
        self.acc_now = acc_now
        self.track_length = track_length

        ## PREPROCESS ##
        # speed vector
        yaw = self.position_in_map[0, 2]

        v = [np.cos(yaw)*self.speed_now, np.sin(yaw)*self.speed_now]

        #-------------------------------Future Position-----------------------------

        self.future_position = self.calc_future_position(self.future_constant)

        #-------------------------------Future Position-----------------------------

        self.idx_nearest_waypoint = self.nearest_waypoint(self.position_in_map[0, :2], self.waypoint_array_in_map[:, :2])

        # if all waypoints are equal set self.idx_nearest_waypoint to 0
        if np.isnan(self.idx_nearest_waypoint):
            self.idx_nearest_waypoint = 0

        if len(self.waypoint_array_in_map[self.idx_nearest_waypoint:]) > 2:
            # calculate curvature of global optimizer waypoints
            self.curvature_waypoints = np.mean(abs(self.waypoint_array_in_map[self.idx_nearest_waypoint+10:self.idx_nearest_waypoint+20, 5]))

        # calculate future lateral error and future lateral error norm

        self.future_lat_e_norm, self.future_lat_err = self.calc_future_lateral_error_norm()

        ### LONGITUDINAL CONTROL ###

        #-----------------------------------------Future-------------------------------------------
        self.speed_command = self.calc_speed_command(v, self.future_lat_e_norm)
        #-----------------------------------------Future-------------------------------------------

        self.speed_command = self.speed_adjust_heading(self.speed_command)

        # POSTPROCESS for acceleration/speed decision

        # Only advance the last-valid anchor on a finite command; np.isfinite
        # keeps a NaN/inf speed (e.g. from an empty curvature slice when the car
        # runs off a stale frozen local path) from reaching VESC/sim, which has
        # no guard and would poison odom. Mirrors the steering safety net below.
        if self.speed_command is not None and np.isfinite(self.speed_command):
            speed = max(self.speed_command, 0)
            self.last_valid_speed = speed
            acceleration = 0
            jerk = 0

        else:
            speed = self.last_valid_speed
            jerk = 0
            acceleration = 0
            self.logger_warn("[Controller] non-finite/none speed; holding last valid speed")

        ### LATERAL CONTROL ###

        steering_angle = None
        self.future_idx_nearest_waypoint = self.nearest_waypoint(self.future_position[0, :2], self.waypoint_array_in_map[:, :2])

        #-----------------------------------------Future-------------------------------------------
        L1_point, L1_distance = self.calc_future_L1_point(self.future_lat_err)
        #-----------------------------------------Future-------------------------------------------

        if L1_point.any() is not None:

            #-----------------------------------------Future-------------------------------------------
            steering_angle = self.calc_steering_angle_for_future(L1_point, L1_distance, yaw, self.future_lat_e_norm, v)
            #-----------------------------------------Future-------------------------------------------

            self.current_steer_command = steering_angle

        else:
            raise Exception("L1_point is None")

        # Final safety net: never emit a non-finite steering command. A NaN/inf
        # here (e.g. from a degenerate local trajectory during overtaking) would
        # otherwise propagate into calc_future_position next cycle and poison the
        # controller permanently, and downstream (VESC / sim) has no guard. Hold
        # the last valid steer instead so the node stays alive and recovers once
        # clean waypoints return; current_steer_command is kept finite for the
        # future-position feedback.
        if not np.isfinite(steering_angle):
            self.logger_warn("[Controller] non-finite steering; holding last valid steer")
            steering_angle = self.curr_steering_angle
            self.current_steer_command = steering_angle

        speed = self.AEB_for_weird_local_wpnt(speed)

        return speed, acceleration, jerk, steering_angle, L1_point, L1_distance, self.idx_nearest_waypoint, self.curvature_waypoints, self.future_position

    def AEB_for_weird_local_wpnt(self, speed):
        nearest_local_wpnt = self.waypoint_array_in_map[self.idx_nearest_waypoint, :2]

        local_wpnt_dist = np.sqrt((self.position_in_map[0, 0] - nearest_local_wpnt[0])**2 + (self.position_in_map[0, 1] - nearest_local_wpnt[1])**2)

        if local_wpnt_dist >= self.AEB_thres:
            return 2.0
        else:
            return speed

    def calc_steering_angle_for_future(self, future_L1_point, L1_distance, yaw, furture_lat_e_norm, v):
        """
        The purpose of this function is to calculate the steering angle based on the L1 point, desired lateral acceleration and velocity

        Inputs:
            future_L1_point: future_L1_point in frenet coordinates at L1 distance in front of the car
            L1_distance: distance of the L1 point to the car
            yaw: yaw angle of the car
            furture_lat_e_norm: future normed lateral error
            v : future speed vector

        Returns:
            steering_angle: calculated steering angle


        """
        marks = MarkerArray()
        for i in range(1):
            mrk = Marker()
            mrk.header.frame_id = "map"
            mrk.type = mrk.SPHERE
            mrk.scale.x = 0.3
            mrk.scale.y = 0.3
            mrk.scale.z = 0.3
            mrk.color.a = 1.0
            mrk.color.b = 1.0

            mrk.id = i
            mrk.pose.position.x = self.future_position[0, 0]
            mrk.pose.position.y = self.future_position[0, 1]
            mrk.pose.orientation.w = 1.0
            marks.markers.append(mrk)

        if self.predict_pub is not None:
            self.predict_pub.publish(marks)

        if (self.state == "TRAILING") and (self.opponent is not None):
            speed_la_for_lu = self.speed_now
        else:
            adv_ts_st = self.speed_lookahead_for_steer
            la_position_steer = [self.future_position[0, 0] + v[0]*adv_ts_st, self.future_position[0, 1] + v[1]*adv_ts_st]
            idx_future_la_steer = self.nearest_waypoint(la_position_steer, self.waypoint_array_in_map[:, :2])
            speed_la_for_lu = self.waypoint_array_in_map[idx_future_la_steer, 2]

        speed_for_lu = self.speed_adjust_lat_err(speed_la_for_lu, furture_lat_e_norm)

        Future_L1_vector = np.array([future_L1_point[0] - self.future_position[0, 0], future_L1_point[1] - self.future_position[0, 1]])

        L1_norm = np.linalg.norm(Future_L1_vector)
        if L1_norm < 1e-6:
            self.logger_warn("[Controller] norm of L1 vector was ~0, eta is set to 0")
            eta = 0
        else:
            # clip to [-1, 1]: the ratio is analytically bounded but can exceed
            # 1 by fp rounding (esp. on sharp evasion where L1 is ~normal to the
            # heading), and arcsin(>1)=NaN would poison steering. Mirrors the
            # guarded arcsin in calc_future_position.
            sin_eta = np.dot([-np.sin(yaw), np.cos(yaw)], Future_L1_vector) / L1_norm
            eta = np.arcsin(np.clip(sin_eta, -1.0, 1.0))

        # Pure-Pursuit steering (ROS1 MAP/steering-lookup branch removed)
        steering_angle = np.arctan(2*self.wheelbase*np.sin(eta)/L1_distance)

        dt = 1.0 / self.loop_rate

        #-------------------------Steering Scaling-----------------------------

        # modifying steer based on heading

        steering_angle += self.compute_future_heading_correction(Future_L1_vector, yaw, dt, self.speed_now)

        # modifying steer based on acceleration
        #########################################
        steering_angle = self.acc_scaling(steering_angle)
        #########################################

        # modifying steer based on speed

        steering_angle = self.speed_steer_scaling(steering_angle, speed_for_lu)

        # modifying steer based on velocity

        steering_angle *= np.clip(1 + (self.speed_now/10), 1, self.steer_gain_for_speed)

        # modifying steer based on lateral error

        steering_angle = self.steer_scaling_for_lat_err(steering_angle, self.future_lat_err)

        #-------------------------Steering Scaling-----------------------------

        # limit change of steering angle
        threshold = 0.4
        if abs(steering_angle - self.curr_steering_angle) > threshold:
            self.logger_info("steering angle clipped")
        steering_angle = np.clip(steering_angle, self.curr_steering_angle - threshold, self.curr_steering_angle + threshold)
        steering_angle = np.clip(steering_angle, -0.53, 0.53)

        # np.clip passes NaN through unchanged, so guard curr_steering_angle
        # (the rate-limit/feedback anchor) against poisoning: only advance it on
        # a finite result. main_loop's final net then falls back to this value.
        if np.isfinite(steering_angle):
            self.curr_steering_angle = steering_angle

        return steering_angle

    def calc_future_L1_point(self, future_lateral_error):

        # calculate future L1 guidance

        if self.speed_now < 2.0:

            speed = np.clip(self.speed_command, self.speed_now - 1, self.speed_now + 1)
            speed_scaler = self.m_l1 * speed

        else:

            speed_scaler = self.m_l1 * self.speed_now

        if self.state == "START":
            curvature_scaler = self.start_curvature_factor*self.curvature_waypoints
        else:
            curvature_scaler = self.curvature_factor*self.curvature_waypoints*self.speed_now*self.speed_now

        L1_distance = (speed_scaler - curvature_scaler) + self.q_l1

        # clip lower bound to avoid ultraswerve when far away from mincurv
        lower_bound = max(self.t_clip_min, np.sqrt(2)*future_lateral_error)

        L1_distance = np.clip(L1_distance, lower_bound, self.t_clip_max)

        future_L1_point = self.waypoint_at_distance_before_car(L1_distance, self.waypoint_array_in_map[:, :2], self.future_idx_nearest_waypoint)

        return future_L1_point, L1_distance

    def calc_speed_command(self, v, lat_e_norm):
        """
        The purpose of this function is to isolate the speed calculation from the main control_loop

        Inputs:
            v: speed vector
            lat_e_norm: normed lateral error
            curvature_waypoints: -
        Returns:
            speed_command: calculated and adjusted speed, which can be sent to mux
        """

        # lookahead for speed (speed delay incorporation by propagating position)
        adv_ts_sp = self.speed_lookahead
        offset = 2
        la_position = [self.position_in_map[0, 0] + v[0]*adv_ts_sp, self.position_in_map[0, 1] + v[1]*adv_ts_sp]
        idx_la_position = self.nearest_waypoint(la_position, self.waypoint_array_in_map[:, :2])
        idx_la_position = np.clip(idx_la_position + offset, 0, len(self.waypoint_array_in_map) - 1)
        global_speed = self.waypoint_array_in_map[idx_la_position, 2]
        cur_speed = self.speed_now

        if cur_speed < 0:
            cur_speed = 0

        if (self.state == "START"
            and self.boost_mode
            and self.waypoint_array_in_map[0, 7] > 0):
            if (global_speed-cur_speed) > 0:
                global_speed = self.start_speed
            elif self.cur_state_speed - cur_speed > 0:
                self.cur_state_speed -= self.speed_diff_thres * (self.cur_state_speed - cur_speed)
                global_speed = self.cur_state_speed
            else:
                self.boost_mode = False
        else:
            self.boost_mode = False

        if self.state == "TRAILING":
            if self.opponent is not None:
                speed_command = self.trailing_controller(global_speed)
            else:
                # TRAILING is a safety state. If its target disappears beyond
                # the manager's short hold window, stop instead of falling
                # through to the global-path speed for a frame.
                self.i_gap = 0
                speed_command = 0.0
        else:
            self.trailing_speed = global_speed
            self.i_gap = 0
            self.static_stop_latched = False
            speed_command = global_speed

        speed_command = self.speed_adjust_lat_err(speed_command, lat_e_norm)

        return speed_command

    def trailing_controller(self, global_speed):
        """
        Adjust the speed of the ego car to trail the opponent at a fixed distance
        Inputs:
            speed_command: velocity of global raceline
            self.opponent: frenet s position and vs velocity of opponent
            self.position_in_map_frenet: frenet s position and vs veloctz of ego car
        Returns:
            trailing_command: reference velocity for trailing
        """

        self.gap = (self.opponent[0] - self.position_in_map_frenet[0]) % self.track_length  # gap to opponent
        self.gap_actual = self.gap
        self.gap_should = self.trailing_vel_gain * self.speed_now + self.trailing_gap

        self.gap_error = self.gap_should - self.gap_actual
        self.v_diff = self.position_in_map_frenet[2] - self.opponent[2]
        self.i_gap = np.clip(self.i_gap + self.gap_error/self.loop_rate, -10, 10)

        p_value = self.gap_error * self.trailing_p_gain
        d_value = self.v_diff * self.trailing_d_gain
        i_value = self.i_gap * self.trailing_i_gain

        self.trailing_command = np.clip(self.opponent[2] - p_value - i_value - d_value, 0, global_speed)

        if self.opponent[3]:
            # Static obstacle. The gap to it does not change while we sit still, so the
            # previous behaviour -- stop at trailing_gap and release only once the gap grows
            # again -- could never release: whenever the avoidance planner failed to produce a
            # path, the car waited in front of the obstacle forever.
            #
            # Instead keep crawling towards it. Every centimetre changes the approach pose, so
            # the planner gets a fresh geometry to solve on each tick and usually finds a path
            # it could not find from further back. The only thing that stops the car is the
            # hard standoff distance, which is a collision limit rather than a trailing gap.
            stop_gap = self.static_min_standoff_m
            resume_gap = stop_gap + self.static_resume_hysteresis_m

            if self.gap_actual <= stop_gap:
                self.static_stop_latched = True
            elif self.static_stop_latched and self.gap_actual > resume_gap:
                # Hysteresis band, so centimetre-level Frenet noise cannot toggle the motor
                # between zero and a tiny positive command right at the standoff.
                self.static_stop_latched = False

            if self.static_stop_latched:
                # At the standoff there is nothing left to try that does not involve reversing.
                return 0.0

            if not self.opponent[4]:
                # Position is held from a stale detection: keep moving, but only at crawl speed.
                return self.static_crawl_speed_mps

            # Never settle at zero: the trailing PID wants to hold trailing_gap, which for a
            # static obstacle means standing still.
            return max(self.trailing_command, self.static_crawl_speed_mps)
        else:
            self.static_stop_latched = False

        if not self.opponent[3] and not self.opponent[4] and self.gap_actual > self.gap_should:
            self.trailing_command = max(self.blind_trailing_speed, self.trailing_command)

        return self.trailing_command

    def distance(self, point1, point2):
        return np.linalg.norm(point2 - point1)

    def acc_scaling(self, steer):
        """
        Steer scaling based on acceleration
        increase steer when accelerating
        decrease steer when decelerating

        Returns:
            steer: scaled steering angle based on acceleration
        """

        if self.start_mode:
            return steer

        if np.mean(self.acc_now) >= 1:
            steer *= self.acc_scaler_for_steer
        elif np.mean(self.acc_now) <= -3.0:
            if self.state == "START":
                steer *= 0.7
            else:
                steer *= self.dec_scaler_for_steer

        return steer

    def speed_steer_scaling(self, steer, speed):
        """
        Steer scaling based on speed
        decrease steer when driving fast

        Returns:
            steer: scaled steering angle based on speed
        """
        speed_diff = max(0.1, self.end_scale_speed-self.start_scale_speed)  # to prevent division by zero
        factor = 1 - np.clip((speed - self.start_scale_speed)/(speed_diff), 0.0, 1.0) * self.downscale_factor
        steer *= factor
        return steer

    def steer_scaling_for_lat_err(self, steer, lateral_error):

        if self.start_mode:
            return steer

        factor = np.exp(np.log(2)*lateral_error)

        steer *= factor
        return steer

    def calc_future_lateral_error_norm(self):
        """
        Calculates future lateral error

        Returns:
           future lat_e_norm: normalization of the future lateral error
           future lateral_error: future distance from car's position to nearest waypoint
        """
        future_position = self.future_position[0, :2]
        idx_future_local_wpnts = self.nearest_waypoint(future_position, self.waypoint_array_in_map[:, :2])
        future_local_wpnts_d = abs(self.waypoint_array_in_map[idx_future_local_wpnts, 8])
        future_potision_s, future_position_d = self.converter.get_frenet([self.future_position[0, 0]], [self.future_position[0, 1]])
        future_position_d = abs(future_position_d[0])
        future_lat_err = future_position_d - future_local_wpnts_d

        max_lat_e = 1
        min_lat_e = 0.
        lat_e_clip = np.clip(future_lat_err, a_min=min_lat_e, a_max=max_lat_e)
        lat_e_norm = ((lat_e_clip - min_lat_e) / (max_lat_e - min_lat_e))
        return lat_e_norm, future_lat_err

    def speed_adjust_lat_err(self, global_speed, lat_e_norm):
        """
        Reduce speed from the global_speed based on the lateral error
        and curvature of the track. lat_e_coeff scales the speed reduction:
        lat_e_coeff = 0: no account for lateral error
        lat_e_coaff = 1: maximum accounting

        Returns:
            global_speed: the speed we want to follow
        """
        # scaling down global speed with lateral error and curvature
        lat_e_coeff = self.lat_err_coeff  # must be in [0, 1]
        lat_e_norm *= self.speed_factor_for_lat_err
        curv = np.clip(2*(np.mean(self.curvature_waypoints)/0.8) - 2, a_min=0.0, a_max=1.0)  # 0.8 ca. max curvature mean
        curv *= self.speed_factor_for_curvature
        global_speed *= (1.0 - lat_e_coeff + lat_e_coeff*np.exp(-lat_e_norm*curv))
        return global_speed

    def speed_adjust_heading(self, speed_command):
        """
        Reduce speed from the global_speed based on the heading error.
        If the difference between the map heading and the actual heading
        is larger than 10 degrees, the speed gets scaled down linearly up to 0.5x

        Returns:
            global_speed: the speed we want to follow
        """

        heading = self.position_in_map[0, 2]
        map_heading = self.waypoint_array_in_map[self.idx_nearest_waypoint, 6]
        if abs(heading - map_heading) > np.pi:
            heading_error = 2*np.pi - abs(heading - map_heading)
        else:
            heading_error = abs(heading - map_heading)

        if heading_error < self.heading_error_thres*np.pi/180:  # 10 degrees error is okay
            return speed_command
        elif heading_error < np.pi/2:
            scaler = 1 - 0.5 * heading_error/(np.pi/2)
        else:
            scaler = 0.5
        return speed_command * scaler

    def compute_future_heading_correction(self, L1_vector, yaw, dt, speed,
                               alpha=0.1, v_threshold=15.0,
                               use_pid=True, use_filter=True):

        target_heading = np.arctan2(L1_vector[1], L1_vector[0])
        heading_error = target_heading - yaw
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

        if use_filter:
            if not hasattr(self, 'filtered_heading_error'):
                self.filtered_heading_error = heading_error
            self.filtered_heading_error = alpha * heading_error + (1 - alpha) * self.filtered_heading_error
            heading_error = self.filtered_heading_error

        if speed < v_threshold:
            dynamic_gain = self.KP * (speed / v_threshold)
        else:
            dynamic_gain = self.KP

        if self.state == "OVERTAKE":
            dynamic_gain *= 0.65

        if not hasattr(self, 'heading_error_integral'):
            self.heading_error_integral = 0.0
        if not hasattr(self, 'prev_heading_error'):
            self.prev_heading_error = heading_error

        if use_pid:
            self.heading_error_integral += heading_error * dt
            derivative = (heading_error - self.prev_heading_error) / dt if dt > 0 else 0.0
            self.prev_heading_error = heading_error

            correction = dynamic_gain * heading_error + self.KI * self.heading_error_integral + self.KD * derivative
        else:
            correction = dynamic_gain * heading_error

        return correction

    def calc_future_position(self, T):
        """
        Predicts the future vehicle state (position and heading) T seconds ahead
        based on the current vehicle state and updates self.position_in_map[0].

        Inputs:
            T: Prediction time (seconds), e.g., 0.25

        Assumes the following variables exist in self:
            self.position_in_map : 2D array with the first row containing [x, y, psi]
            self.speed_now       : Current vehicle speed (v)
            self.current_steer_command : Current steering input (delta)
            self.yaw_rate        : Current yaw rate from the IMU (rad/s)
            self.wheelbase       : Vehicle wheelbase (distance between front and rear axles)
        """

        x_current = self.position_in_map[0, 0]

        # Extract current state
        x_current = self.position_in_map[0, 0]
        y_current = self.position_in_map[0, 1]
        psi_current = self.position_in_map[0, 2]
        v = self.speed_now
        delta = self.current_steer_command  # Steering input

        # Vehicle geometry parameters.
        # Here, L_f and L_r are assumed to be 52% and 48% of the total wheelbase respectively.
        L_total = self.wheelbase
        L_f = 0.52 * L_total
        L_r = 0.48 * L_total

        # 1. Compute geometric slip angle (basic model)
        beta_model = np.arctan((L_r / (L_f + L_r)) * np.tan(delta))

        # 2. Estimate slip angle indirectly using IMU yaw rate data
        if abs(v) > 2.0:
            # If speed is sufficient, estimate slip angle from IMU yaw rate
            beta_imu = np.arcsin(np.clip(((L_f + L_r) * self.yaw_rate / v), -1.0, 1.0))
        else:
            beta_imu = beta_model  # Maintain basic model when speed is very low

        # 3. Fuse the geometric and IMU-based slip angles using weighted average
        lambda_weight = 1.0
        beta_fused = lambda_weight * beta_model + (1 - lambda_weight) * beta_imu

        # 4. Predict future position using the fused slip angle
        future_x = x_current + v * np.cos(psi_current + beta_fused) * T
        future_y = y_current + v * np.sin(psi_current + beta_fused) * T

        # 5. Predict future heading:
        # Option A: Model-based prediction
        future_psi_model = psi_current + (v / (L_f + L_r)) * np.sin(beta_fused) * T
        # Option B: IMU-based prediction
        future_psi_imu = psi_current + self.yaw_rate * T
        # Fuse the two heading predictions using a weighted average
        gamma_weight = 1.0
        future_psi = gamma_weight * future_psi_model + (1 - gamma_weight) * future_psi_imu
        # Normalize heading to the range [-pi, pi]
        future_psi = np.arctan2(np.sin(future_psi), np.cos(future_psi))

        # Update the global state: overwrite self.position_in_map[0] with the future state.

        future_position = np.zeros((1, 3))

        future_position[0, 0] = future_x
        future_position[0, 1] = future_y
        future_position[0, 2] = future_psi

        return future_position

    def nearest_waypoint(self, position, waypoints):
        """
        Calculates index of nearest waypoint to the car

        Returns:
            index of nearest waypoint to the car
        """
        position_array = np.array([position]*len(waypoints))
        distances_to_position = np.linalg.norm(abs(position_array - waypoints), axis=1)
        return np.argmin(distances_to_position)

    def waypoint_at_distance_before_car(self, distance, waypoints, idx_waypoint_behind_car):
        """
        Calculates the waypoint at a certain frenet distance in front of the car

        Returns:
            waypoint as numpy array at a ceratin distance in front of the car
        """

        if distance is None:
            distance = self.t_clip_min
        d_distance = distance

        # Extract only waypoints ahead of current index
        waypoints_ahead = waypoints[idx_waypoint_behind_car:]

        # Compute segment-wise distances between waypoints
        deltas = np.diff(waypoints_ahead, axis=0)
        seg_lengths = np.linalg.norm(deltas, axis=1)

        # Compute cumulative distances
        cum_lengths = np.cumsum(seg_lengths)

        # Find the first index where cumulative distance exceeds lookahead
        idx_offset = min(np.searchsorted(cum_lengths, d_distance), len(waypoints_ahead) - 1)

        return waypoints_ahead[idx_offset]
