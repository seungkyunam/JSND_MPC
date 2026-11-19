"""Startup configuration shared by ROS adapter and offline tests."""

import numpy as np

from .integrated_mpc import IntegratedConfig


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
}


def make_config(values, wheelbase, wheelbase_explicit, loop_rate):
    if values['mpc_geometry_confirmed'] is not True or not wheelbase_explicit:
        raise ValueError('MPC requires explicit wheelbase and mpc_geometry_confirmed=true; '
                         'confirm pose reference, steering units/sign and measured limits first')
    if not np.isfinite(values['mpc_pose_to_rear_axle_m']):
        raise ValueError('mpc_pose_to_rear_axle_m must be finite')
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
    )
