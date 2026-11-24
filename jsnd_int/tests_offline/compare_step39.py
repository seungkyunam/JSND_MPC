#!/usr/bin/env python3
"""Run existing lap_sim on repository test0, PP unchanged vs MPC 5p0/8p88.

OPENBLAS_NUM_THREADS=1 python3 jsnd_int/tests_offline/compare_step39.py
Requires numpy, scipy, osqp, pyyaml. No ROS, hardware, or real-car map needed.
P99 measures Controller.main_loop for both controllers, excluding simulated plant.
"""
import json
import os
from pathlib import Path
import sys
from time import perf_counter

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'jsnd_int/racing_stack/controller'), str(ROOT/'controller/test'),
                str(ROOT/'race_utils/f110_utils/libs/frenet_conversion')]
import controller.mpc
controller.mpc.__path__.append(str(ROOT/'controller/controller/mpc'))
from offline_support import install_marker_shims
install_marker_shims()
os.environ['JSND_LAP_STACK_ROOT'] = str(ROOT/'jsnd_int/racing_stack/stack_master')
os.environ['JSND_LAP_MAP_ROOT'] = str(ROOT/'stack_master/maps/test0')
os.environ['JSND_LAP_BASE_CONFIG'] = str(ROOT/'stack_master/config')
import lap_sim
from controller.combined.src.Controller import Controller


def main():
    lap_sim.REAL_DEADLINES = True
    original = Controller.main_loop
    times = []

    def timed(self, *args, **kwargs):
        started = perf_counter()
        try:
            return original(self, *args, **kwargs)
        finally:
            times.append((perf_counter()-started)*1000.)

    Controller.main_loop = timed
    try:
        for label, ctrl, profile in [('PP', 'pp', '5p0'), ('MPC5', 'mpc', '5p0'),
                                     ('MPC8.88', 'mpc', '8p88')]:
            times.clear()
            result, trace = lap_sim.simulate(ctrl, profile,
                {'mpc_speed_source': 'integrated'} if ctrl == 'mpc' else {}, rate=50, laps=2)
            print(json.dumps(dict(controller=label, map='test0', rate_hz=50,
                laps_done=result['laps_done'], lap_times_s=result['lap_times'],
                mean_lateral_error_m=result['lat_err_mean'], max_lateral_error_m=result['lat_err_max'],
                actual_max_speed_mps=result['max_speed'], mean_speed_mps=result['avg_speed'],
                stops=result['stops'], controller_p99_ms=round(float(np.percentile(times,99)),3),
                # Physical bottleneck diagnostic; lap_sim's old binding_layer_pct is
                # deliberately omitted because mpc_layers predates the current planner.
                max_lateral_accel_mps2=result['ay_max']), ensure_ascii=False), flush=True)
    finally:
        Controller.main_loop = original


if __name__ == '__main__':
    main()
