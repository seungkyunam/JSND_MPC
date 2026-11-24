#!/usr/bin/env python3
"""Step 43: four offline runs, unchanged PP vs Step 42 MPC, seed 0.

OPENBLAS_NUM_THREADS=1 python3 jsnd_int/tests_offline/compare_step43.py
Step 38 summaries do not identify error distributions or separate actuator and
localization latency. Interpret position p95 as radial Gaussian error, heading
p95 as absolute Gaussian error, and 110 ms as a lumped steering queue delay.
This is a sensitivity experiment, not a replay or identified real-car model.
"""
import json
import numpy as np
import compare_step39 as env
from controller.mpc.integrated_mpc import IntegratedMPC


def main():
    lap = env.lap_sim
    lap.REAL_DEADLINES = True
    # Process-local Step 42 setting; production constant remains 0.85.
    IntegratedMPC.LATERAL_PLAN_FRACTION = 1.0
    cases = [('normal', 1., 0., 0.),
             ('step38_errors', .110*50, .09/np.sqrt(-2*np.log(.05)), .17/1.959963984540054)]
    for condition, delay, pos, yaw in cases:
        for ctrl in ('pp', 'mpc'):
            overrides = {} if ctrl == 'pp' else {
                'mpc_speed_source': 'integrated', 'mpc_max_speed_mps': 8.88,
                'mpc_max_lateral_accel_mps2': 8.,
                'mpc_max_accel_mps2': 6., 'mpc_max_decel_mps2': 6.,
            }
            r, _ = lap.simulate(ctrl, '8p88', overrides, rate=50, laps=2,
                servo_delay_ticks=delay, noise_pos=pos, noise_yaw=yaw, noise_v=0., seed=0)
            print(json.dumps(dict(step=43, condition=condition, controller=ctrl,
                map='test0', rate_hz=50, seed=0, t_max_s=60.,
                servo_delay_ticks=delay, servo_delay_s=delay/50,
                noise_pos_axis_sigma_m=pos, noise_yaw_sigma_rad=yaw, noise_v_sigma_mps=0.,
                mpc_overrides=overrides, mpc_plan_fraction=1.0 if ctrl=='mpc' else None,
                laps_done=r['laps_done'], lap_times_s=r['lap_times'], sim_time_s=r['sim_time'],
                mean_lateral_error_m=r['lat_err_mean'], max_lateral_error_m=r['lat_err_max'],
                stops=r['stops'], stop_ticks=r['stop_ticks'], status=r.get('status',{})),
                ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
