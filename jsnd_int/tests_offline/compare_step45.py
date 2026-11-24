#!/usr/bin/env python3
"""Step 45: fixed 2x2 feature ablation, normal/combined Step 44 conditions.

OPENBLAS_NUM_THREADS=1 /tmp/jsnd-step39-venv/bin/python jsnd_int/tests_offline/compare_step45.py
No parameter sweep. Baselines rerun solely to measure comparable main-loop p99.
"""
import argparse
import json
from collections import Counter
from time import perf_counter
import numpy as np
import compare_step39 as env
from controller.combined.src.Controller import Controller
from controller.mpc.integrated_mpc import IntegratedMPC


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--final', action='store_true', help='Validate only the shipped baseline/combined modes')
    args = parser.parse_args()
    env.lap_sim.REAL_DEADLINES = True
    IntegratedMPC.LATERAL_PLAN_FRACTION = 1.0
    elapsed = []
    original = Controller.main_loop

    def timed(self, *args, **kwargs):
        start = perf_counter()
        try:
            return original(self, *args, **kwargs)
        finally:
            elapsed.append((perf_counter() - start) * 1000)

    Controller.main_loop = timed
    try:
        for condition, delay, pos, yaw in [
            ('normal', 1., 0., 0.),
            ('step38_errors', 5.5, .09 / np.sqrt(-2*np.log(.05)), .17 / 1.959963984540054),
        ]:
            for variant, heading, history in [('baseline', False, False), ('heading', True, False),
                                               ('history', False, True), ('combined', True, True)]:
                if args.final and variant not in ('baseline', 'combined'):
                    continue
                elapsed.clear()
                overrides = dict(mpc_speed_source='integrated', mpc_max_speed_mps=8.88,
                                 mpc_max_lateral_accel_mps2=8., mpc_max_accel_mps2=6.,
                                 mpc_max_decel_mps2=6., mpc_heading_robust=heading,
                                 mpc_history_prediction=history)
                result, trace = env.lap_sim.simulate('mpc', '8p88', overrides, rate=50, laps=2,
                    servo_delay_ticks=delay, noise_pos=pos, noise_yaw=yaw, noise_v=0., seed=0)
                failures = Counter(s for s, stop in zip(result.pop('_status_seq'), trace['stop']) if stop)
                print(json.dumps(dict(step=45, condition=condition, variant=variant,
                    map='test0', rate_hz=50, seed=0, t_max_s=60., mpc_plan_fraction=1.0,
                    servo_delay_ticks=delay, noise_pos_axis_sigma_m=pos, noise_yaw_sigma_rad=yaw,
                    overrides=overrides, laps_done=result['laps_done'], lap_times_s=result['lap_times'],
                    sim_time_s=result['sim_time'], mean_lateral_error_m=result['lat_err_mean'],
                    max_lateral_error_m=result['lat_err_max'], stops=result['stops'], stop_ticks=result['stop_ticks'],
                    stop_causes_full=dict(failures), status=result['status'],
                    controller_p99_ms=round(float(np.percentile(elapsed, 99)), 3),
                    solver_elapsed_mean_p99_max_ms=result['mpc_ms_mean_p99_max']), ensure_ascii=False), flush=True)
    finally:
        Controller.main_loop = original


if __name__ == '__main__':
    main()
