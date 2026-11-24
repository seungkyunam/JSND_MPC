#!/usr/bin/env python3
"""Step 41: vary MPC longitudinal limits only in the existing test0 lap simulator.

OPENBLAS_NUM_THREADS=1 python3 jsnd_int/tests_offline/compare_step41.py
Requires the same dependencies as compare_step39.py. No ROS or hardware.
The simulated plant remains at +4/-6 m/s^2 with a 0.12 s speed-loop time
constant; MPC limit overrides do not increase simulated actuator capability.
"""
import json

import compare_step39 as env


def main():
    lap = env.lap_sim
    lap.REAL_DEADLINES = True
    for limit in (5.0, 6.0, 8.0):
        result, _ = lap.simulate('mpc', '8p88', {
            'mpc_speed_source': 'integrated',
            'mpc_max_speed_mps': 8.88,
            'mpc_max_lateral_accel_mps2': 8.0,
            'mpc_max_accel_mps2': limit,
            'mpc_max_decel_mps2': limit,
        }, rate=50, laps=2)
        print(json.dumps({
            'step': 41, 'map': 'test0', 'rate_hz': 50,
            'speed_ceiling_mps': 8.88, 'lateral_limit_mps2': 8.0,
            'max_accel_mps2': limit, 'max_decel_mps2': limit,
            'plant_accel_mps2': lap.VESC_ACC, 'plant_decel_mps2': lap.VESC_DEC,
            'plant_tau_s': lap.VESC_TAU,
            'laps_done': result['laps_done'], 'lap_times_s': result['lap_times'],
            'mean_lateral_error_m': result['lat_err_mean'],
            'max_lateral_error_m': result['lat_err_max'],
            'actual_max_speed_mps': result['max_speed'],
            'stops': result['stops'], 'stop_ticks': result['stop_ticks'],
            'status': result['status'],
        }), flush=True)


if __name__ == '__main__':
    main()
