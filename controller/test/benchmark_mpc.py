#!/usr/bin/env python3
"""Offline benchmark only. No ROS imports/nodes/publishers or real map files.

Output creation is exclusive. Run from the repository root with PYTHONPATH=controller.
"""

import argparse
from collections import Counter
from dataclasses import asdict
import json
import platform
from time import perf_counter

import numpy as np
import osqp
import scipy

from controller.mpc.bicycle import step
from controller.mpc.lateral_mpc import LateralMPC, MPCConfig
from offline_support import cycle, make_controller, virtual_path


def stats(seconds):
    ms = 1000 * np.asarray(seconds)
    return {'count': len(ms), 'p50_ms': float(np.percentile(ms, 50)),
            'p95_ms': float(np.percentile(ms, 95)), 'p99_ms': float(np.percentile(ms, 99)),
            'max_ms': float(np.max(ms)), 'over_20ms': int(np.sum(ms > 20.0))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    config = MPCConfig(wheelbase=0.32, max_steer=0.53, max_steer_rate=2.0)
    report = {'environment': {'python': platform.python_version(), 'platform': platform.platform(),
                              'numpy': np.__version__, 'scipy': scipy.__version__, 'osqp': osqp.__version__},
              'synthetic_config_not_vehicle_calibration': asdict(config),
              'scope': 'No ROS/DDS/visualization/actuator timing; module cold/warm timing and mocked-message Controller.main_loop',
              'paths': {}}
    for name, count, speed in [('straight', 200, 2.0), ('sharp_curve', 100, 1.5),
                               ('s_curve', 250, 2.0), ('static_avoidance', 350, 2.0)]:
        path = virtual_path(name)
        start = int(np.argmin(abs(path[:, 0]))) if name != 'sharp_curve' else int(np.argmin(abs(path[:, 4] - 0.6)))
        state = np.array([path[start, 0], path[start, 1] + 0.08, path[start, 6]])
        lateral = LateralMPC(config)
        previous = 0.0
        times, solver_times, errors, status, states = [], [], [], Counter(), []
        for _ in range(count):
            states.append(state.copy())
            started = perf_counter()
            result = lateral.solve(state, path, speed, previous)
            times.append(perf_counter() - started)
            solver_times.append(result.solver_s)
            status[result.status] += 1
            # Apply explicit stop request in synthetic plant on failure.
            state = step(state, result.steering, speed if result.success else 0.0, config.dt, config.wheelbase)
            previous = result.steering
            errors.append(float(np.min(np.linalg.norm(path[:, :2] - state[:2], axis=1))))
        # Full legacy speed + MPC adapter, at recorded synthetic states. The
        # mock global Frenet converter is deliberately not a real map converter.
        integrated = make_controller(lateral_controller='MPC', lateral_mpc=LateralMPC(config))
        integration_times, integration_status = [], Counter()
        for state in states:
            started = perf_counter()
            cycle(integrated, path, position=state, speed=speed)
            integration_times.append(perf_counter() - started)
            integration_status[integrated.mpc_result.status] += 1
        report['paths'][name] = {
            'module_all': stats(times), 'module_warm': stats(times[1:]), 'module_cold_ms': times[0] * 1000,
            'osqp_all': stats(solver_times), 'statuses': dict(status),
            'rms_cross_track_m': float(np.sqrt(np.mean(np.square(errors)))), 'max_cross_track_m': max(errors),
            'controller_main_loop_all': stats(integration_times),
            'controller_statuses': dict(integration_status),
        }
    with open(args.output, 'x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
