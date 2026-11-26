"""Startup configuration shared by ROS adapter and offline tests."""

import numpy as np

from .integrated_mpc import IntegratedConfig
from .steering_mpc import SteeringConfig


MPC_DEFAULTS = {
    # These are NOT certified vehicle values. MPC cannot start until the
    # operator explicitly supplies geometry and positive actuator limits.
    'mpc_geometry_confirmed': False,
    'mpc_pose_to_rear_axle_m': 0.0,
    'mpc_max_steer_rad': 0.0,
    'mpc_max_steer_rate_radps': 0.0,
    'mpc_horizon': 20,
    'mpc_preview_dt': .1,
    'mpc_braking_preview_ratio': .85,
    'mpc_max_speed_mps': 3.0,
    'mpc_max_accel_mps2': 1.5,
    'mpc_max_decel_mps2': 2.5,
    'mpc_max_lateral_accel_mps2': 3.0,
    'mpc_q_position': 12.0,
    'mpc_q_speed': 8.0,
    'mpc_r_accel': .15,
    'mpc_q_heading': 4.0,
    'mpc_r_steer': 0.2,
    'mpc_r_rate': 1.0,
    'mpc_terminal_scale': 3.0,
    'mpc_solver_time_limit_s': 0.008,
    'mpc_deadline_s': 0.015,
    'mpc_max_iter': 1000,
    'mpc_max_tracking_error_m': 1.0,
    'mpc_max_heading_error_rad': 0.7,
    # ---- step 23: speed source. 'jsnd' (default) = legacy JSND/PP speed pipeline decides
    # the speed, SteeringMPC decides only the steering. 'integrated' = previous joint
    # speed+steering IntegratedMPC (kept selectable for comparison). ----
    'mpc_speed_source': 'jsnd',
    'mpc_steer_horizon': 15,
    'mpc_steer_preview_dt': .05,
    'mpc_forecast_accel_mps2': 5.0,      # ggv ax_max (CAR/veh_dyn_info/ggv.csv), forecast only
    'mpc_forecast_decel_mps2': 5.0,
    # physical lateral-acceleration limit checked against the PLAN (planner kappa x commanded
    # speed). = ggv ay_max the raceline was optimised with, so it binds only if a plan is
    # physically infeasible (never on the iqp raceline x sector scaling <= 1).
    'mpc_plan_lateral_accel_limit_mps2': 4.5,
    # transient solver/compute miss -> legacy PP steering for at most this long, then STOP.
    # Envelope violations (path/heading) STOP immediately.
    'mpc_fallback_max_s': .2,
    'mpc_latency_comp_s': 0.,
    'mpc_heading_robust': False,
    'mpc_history_prediction': False,
    # Step 47 (integrated only; 0 = previous behaviour)
    'mpc_brake_lead_s': 0.,
    'mpc_steer_preview_s': 0.,
    'mpc_time_weighted_accel_cost': False,
}


def _check(values, wheelbase_explicit):
    for key in ('mpc_heading_robust', 'mpc_history_prediction'):
        if type(values[key]) is not bool:
            raise ValueError(key + ' must be boolean')
    if not np.isfinite(values['mpc_latency_comp_s']) or not 0 <= values['mpc_latency_comp_s'] <= 1.0:
        raise ValueError('mpc_latency_comp_s must be finite and between 0 and 1 s')
    if values['mpc_geometry_confirmed'] is not True or not wheelbase_explicit:
        raise ValueError('MPC requires explicit wheelbase and mpc_geometry_confirmed=true; '
                         'confirm pose reference, steering units/sign and measured limits first')
    if not np.isfinite(values['mpc_pose_to_rear_axle_m']):
        raise ValueError('mpc_pose_to_rear_axle_m must be finite')


def make_steering_config(values, wheelbase, wheelbase_explicit, loop_rate):
    _check(values, wheelbase_explicit)
    return SteeringConfig(
        wheelbase=wheelbase, dt=1.0 / loop_rate,
        max_steer=values['mpc_max_steer_rad'], max_steer_rate=values['mpc_max_steer_rate_radps'],
        horizon=int(values['mpc_steer_horizon']), preview_dt=values['mpc_steer_preview_dt'],
        forecast_accel=values['mpc_forecast_accel_mps2'], forecast_decel=values['mpc_forecast_decel_mps2'],
        max_speed=values['mpc_max_speed_mps'],
        q_position=values['mpc_q_position'], q_heading=values['mpc_q_heading'],
        r_steer=values['mpc_r_steer'], r_rate=values['mpc_r_rate'],
        terminal_scale=values['mpc_terminal_scale'],
        solver_time_limit=values['mpc_solver_time_limit_s'], deadline=values['mpc_deadline_s'],
        max_iter=int(values['mpc_max_iter']), max_tracking_error=values['mpc_max_tracking_error_m'],
        max_heading_error=values['mpc_max_heading_error_rad'],
    )


def make_config(values, wheelbase, wheelbase_explicit, loop_rate):
    _check(values, wheelbase_explicit)
    return IntegratedConfig(
        wheelbase=wheelbase, dt=1.0 / loop_rate,
        braking_preview_ratio=values['mpc_braking_preview_ratio'],
        preview_dt=values['mpc_preview_dt'], max_speed=values['mpc_max_speed_mps'],
        max_accel=values['mpc_max_accel_mps2'], max_decel=values['mpc_max_decel_mps2'],
        max_lateral_accel=values['mpc_max_lateral_accel_mps2'],
        q_speed=values['mpc_q_speed'], r_accel=values['mpc_r_accel'],
        max_steer=values['mpc_max_steer_rad'],
        max_steer_rate=values['mpc_max_steer_rate_radps'],
        horizon=values['mpc_horizon'], q_position=values['mpc_q_position'],
        q_heading=values['mpc_q_heading'], r_steer=values['mpc_r_steer'],
        r_rate=values['mpc_r_rate'], terminal_scale=values['mpc_terminal_scale'],
        solver_time_limit=values['mpc_solver_time_limit_s'],
        deadline=values['mpc_deadline_s'], max_iter=values['mpc_max_iter'],
        max_tracking_error=values['mpc_max_tracking_error_m'],
        max_heading_error=values['mpc_max_heading_error_rad'],
        brake_lead_s=float(values['mpc_brake_lead_s']),
        steer_preview_s=float(values['mpc_steer_preview_s']),
        time_weighted_accel_cost=values['mpc_time_weighted_accel_cost'],
    )
