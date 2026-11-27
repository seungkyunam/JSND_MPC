#!/usr/bin/env python3
"""Deterministic 11_18_6 closed-loop trial using production PP/MPC code.

This is an offline bicycle/actuator simulation.  It never starts ROS or touches
vehicle hardware.  A trial writes raw CSV, solver diagnostics and a JSON summary.
"""
import argparse
import collections
import csv
import dataclasses
import json
import os
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'jsnd_int/racing_stack/controller'), str(ROOT/'controller/test'),
                str(ROOT/'race_utils/f110_utils/libs/frenet_conversion'),
                str(ROOT/'jsnd_int/tests_offline')]
import controller.mpc
controller.mpc.__path__.insert(0, str(ROOT/'jsnd_int/racing_stack/controller/controller/mpc'))
from offline_support import install_marker_shims
install_marker_shims()
os.environ['JSND_LAP_STACK_ROOT'] = str(ROOT/'jsnd_int/racing_stack/stack_master')
os.environ['JSND_LAP_MAP_ROOT'] = str(ROOT/'jsnd_int/racing_stack/stack_master/maps/11_18_6')
os.environ['JSND_LAP_BASE_CONFIG'] = str(ROOT/'stack_master/config')
import lap_sim as sim

RAW_WAYPOINTS = json.loads((ROOT/'jsnd_int/racing_stack/stack_master/maps/11_18_6/global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts'][:-1]
RAW_SPEED = np.asarray([point['vx_mps'] for point in RAW_WAYPOINTS], float)


def local_waypoints(s, fair_pp=False, n=80):
    index = int(s/sim.WP_DIST+.5)
    indices = (np.arange(n)+index) % len(sim.GB)
    points = sim.GB[indices].copy()
    if fair_pp:
        # Keep PP's own waypoint/lookahead/error speed logic, but remove the two
        # legacy 0.6 sector multipliers which apply only to PP in this map.
        points[:, 2] = RAW_SPEED[indices]
    return points


def profile(name):
    base = {'mpc_speed_source': 'integrated', 'mpc_history_prediction': True}
    if name == 'legacy057':
        return {**base, 'mpc_horizon': 12, 'mpc_preview_dt': .05,
                'mpc_time_scaled_objective': False}
    if name.startswith('n'):
        n = int(name[1:])
        if n not in (10, 15, 20, 25):
            raise ValueError('N must be 10, 15, 20 or 25')
        return {**base, 'mpc_horizon': n, 'mpc_preview_dt': .98/(n-1),
                'mpc_time_scaled_objective': True}
    if name == 'final':
        # Populated from the completed horizon/weight sweep; physical limits are
        # inherited unchanged from mpc_car.yaml.
        return {**base, 'mpc_horizon': 10, 'mpc_preview_dt': .98/9,
                'mpc_time_scaled_objective': True, 'mpc_q_speed': 10.0,
                'mpc_r_accel': .05, 'mpc_cmd_accel_lead_mps': 1.2,
                'mpc_brake_lead_s': .08, 'mpc_steer_preview_s': .03}
    if name == 'pp':
        return {}
    raise ValueError(name)


def scenario(name):
    if name == 'ideal':
        return dict(pose_delay=0, servo_delay=0, noise_xy=0., noise_yaw=0., noise_v=0., seed=0)
    if name == 'realistic':
        # Existing Step51 assumptions: 120 ms pose + 60 ms servo.  Noise is a
        # documented stress assumption, not a claim about measured sensors.
        return dict(pose_delay=6, servo_delay=3, noise_xy=.02, noise_yaw=.01,
                    noise_v=.05, seed=20261127)
    raise ValueError(name)


def run(controller_name, profile_name, scenario_name, output, laps, extra_overrides=None):
    ctrl_kind = 'pp' if controller_name == 'pp' else 'mpc'
    overrides = profile(profile_name)
    overrides.update(extra_overrides or {})
    c, params = sim.build(ctrl_kind, '5p0', overrides, 50)
    if ctrl_kind == 'mpc':
        c.mpc_cmd_accel_lead_mps = params['mpc_cmd_accel_lead_mps']
    cfg = scenario(scenario_name)
    rng = np.random.default_rng(cfg['seed'])
    dt, wheelbase = .02, .324
    motor_max = 23250/4614
    conv = c.converter
    x, y, yaw = map(float, sim.FULL[0, [0, 1, 6]])
    v = delta = t = total = 0.
    previous_s = float(conv.get_frenet([x], [y])[0][0])
    poses = collections.deque([(x, y, yaw)]*cfg['pose_delay'])
    servo = collections.deque([0.]*cfg['servo_delay'])
    acceleration_history = np.zeros(10)
    lap_marks, rows, diagnostics = [], [], []
    c.pose_prediction.clock = lambda: t
    mapdir = Path(os.environ['JSND_LAP_MAP_ROOT'])
    meta = yaml.safe_load((mapdir/'11_18_6.yaml').read_text())
    grid = np.array(Image.open(mapdir/meta['image']).convert('L'))

    def occupied(px, py):
        ix = int(np.floor((px-meta['origin'][0])/meta['resolution']))
        iy = grid.shape[0]-1-int(np.floor((py-meta['origin'][1])/meta['resolution']))
        return (float(1-grid[iy, ix]/255) > meta['occupied_thresh']
                if 0 <= ix < grid.shape[1] and 0 <= iy < grid.shape[0] else True)

    for tick in range(20000):
        s, d = map(float, conv.get_frenet([x], [y])[:, 0])
        total += (s-previous_s+sim.TRACK_LEN/2) % sim.TRACK_LEN-sim.TRACK_LEN/2
        previous_s = s
        if total >= sim.TRACK_LEN*(len(lap_marks)+1):
            lap_marks.append(t)
            if len(lap_marks) >= laps:
                break
        wp = local_waypoints(s, fair_pp=(ctrl_kind == 'pp'))
        if cfg['pose_delay']:
            poses.append((x, y, yaw)); px, py, pyaw = poses.popleft()
        else:
            px, py, pyaw = x, y, yaw
        measured_v = max(0., v+cfg['noise_v']*rng.standard_normal())
        pose = np.array([[px+cfg['noise_xy']*rng.standard_normal(),
                          py+cfg['noise_xy']*rng.standard_normal(),
                          pyaw+cfg['noise_yaw']*rng.standard_normal()]])
        c.yaw_rate = v*np.tan(delta)/wheelbase
        c.pose_prediction.observe(t, c.yaw_rate)
        out = c.main_loop('GB_TRACK', pose, wp, measured_v, None,
                          np.array([s, d, measured_v, 0.]), acceleration_history, sim.TRACK_LEN)
        command, steer = float(out[0]), float(out[3])
        status, solver_ms, elapsed_ms, iterations = 'PP', 0., 0., 0
        failure = False
        if ctrl_kind == 'mpc':
            result = c.mpc_result
            status, solver_ms = result.status, result.solver_s*1000
            elapsed_ms, iterations = result.elapsed_s*1000, result.iterations
            failure = not result.success
            if command <= 0:
                c.reset_mpc_speed_command()
            c.record_applied_steering(steer)
        motor = float(np.clip(command, 0., motor_max))
        target_delta = float(np.clip(steer, -sim.SERVO_LIMIT, sim.SERVO_LIMIT))
        if cfg['servo_delay']:
            servo.append(target_delta); target_delta = servo.popleft()
        delta += float(np.clip(target_delta-delta, -sim.SERVO_RATE*dt, sim.SERVO_RATE*dt))
        ay = v*v*abs(np.tan(delta))/wheelbase
        desired_accel = (motor-v)/sim.VESC_TAU
        longitudinal_limit = sim.VESC_ACC if desired_accel >= 0 else sim.VESC_DEC
        # Shared friction ellipse: lateral demand reduces available drive/brake.
        # Lateral excess is still recorded as a violation because this kinematic
        # model has no tire-slip state with which to resolve it.
        available = longitudinal_limit*np.sqrt(max(0., 1.-(ay/4.5)**2))
        accel = float(np.clip(desired_accel, -available, available))
        combined = (accel/longitudinal_limit)**2+(ay/4.5)**2
        rows.append(dict(t=t, lap=len(lap_marks)+1, s=s, d=d, x=x, y=y, yaw=yaw,
                         speed=v, measured_speed=measured_v, command_speed=command,
                         motor_command=motor, command_steer=steer, actual_steer=delta,
                         accel=accel, lateral_accel=ay, combined_limit=combined,
                         occupied=int(occupied(x, y)), status=status,
                         solver_ms=solver_ms, elapsed_ms=elapsed_ms,
                         iterations=iterations, solver_failure=int(failure)))
        diagnostics.append(dict(t=t, status=status, solver_ms=solver_ms,
                                elapsed_ms=elapsed_ms, iterations=iterations))
        acceleration_history[1:] = acceleration_history[:-1]
        acceleration_history[0] = accel
        x += dt*v*np.cos(yaw); y += dt*v*np.sin(yaw)
        yaw += dt*v*np.tan(delta)/wheelbase
        v = max(0., v+accel*dt); t = (tick+1)*dt

    output.mkdir(parents=True, exist_ok=False)
    with (output/'trace.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    with (output/'diagnostics.jsonl').open('w') as handle:
        for row in diagnostics:
            handle.write(json.dumps(row)+'\n')
    stat_laps = np.diff(lap_marks)[0:] if len(lap_marks) > 1 else np.array([])
    active = np.array([row['lap'] > 1 for row in rows])
    speeds = np.array([row['speed'] for row in rows])[active]
    errors = np.array([row['d'] for row in rows])[active]
    solve = np.array([row['solver_ms'] for row in rows])[active]
    elapsed = np.array([row['elapsed_ms'] for row in rows])[active]
    failures = np.array([row['solver_failure'] for row in rows])[active]
    stopped = speeds < .1
    stop_events = int(np.sum(stopped & ~np.r_[False, stopped[:-1]]))
    summary = dict(controller=controller_name, profile=profile_name, scenario=scenario_name,
                   map='11_18_6', control_hz=50, completed_laps=len(lap_marks),
                   excluded_laps=1, lap_times_s=stat_laps.tolist(),
                   best_lap_s=float(np.min(stat_laps)) if len(stat_laps) else None,
                   mean_lap_s=float(np.mean(stat_laps)) if len(stat_laps) else None,
                   lap_std_s=float(np.std(stat_laps, ddof=1)) if len(stat_laps)>1 else None,
                   max_speed_mps=float(np.max(speeds)), mean_speed_mps=float(np.mean(speeds)),
                   tracking_rmse_m=float(np.sqrt(np.mean(errors**2))),
                   solver_ms=dict(p50=float(np.percentile(solve, 50)), p95=float(np.percentile(solve, 95)),
                                  p99=float(np.percentile(solve, 99)), max=float(np.max(solve))),
                   cycle_overrun_pct=float(100*np.mean(elapsed > 20)),
                   solver_failure_pct=float(100*np.mean(failures)), stop_events=stop_events,
                   occupied_samples=int(sum(row['occupied'] for row in rows if row['lap'] > 1)),
                   lateral_limit_violations=int(sum(row['lateral_accel'] > 4.5+1e-6 for row in rows if row['lap'] > 1)),
                   combined_limit_violation_pct=float(100*np.mean(np.array([row['combined_limit'] for row in rows])[active] > 1+1e-6)),
                   config=dataclasses.asdict(c.lateral_mpc.config) if ctrl_kind == 'mpc' else None,
                   assumptions=dict(wheelbase_m=wheelbase, motor_max_mps=motor_max,
                                    accel_mps2=sim.VESC_ACC, decel_mps2=sim.VESC_DEC,
                                    lateral_accel_mps2=4.5, motor_tau_s=sim.VESC_TAU, **cfg))
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary))
    return 0 if len(lap_marks) == laps else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--controller', choices=['pp', 'mpc'], required=True)
    parser.add_argument('--profile', choices=['pp', 'legacy057', 'n10', 'n15', 'n20', 'n25', 'final'], required=True)
    parser.add_argument('--scenario', choices=['ideal', 'realistic'], default='ideal')
    parser.add_argument('--laps', type=int, default=11)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--set', nargs='*', default=[], metavar='KEY=VALUE')
    args = parser.parse_args()
    extra = {}
    for item in args.set:
        key, value = item.split('=', 1)
        extra[key] = yaml.safe_load(value)
    return run(args.controller, args.profile, args.scenario, args.output, args.laps, extra)


if __name__ == '__main__':
    raise SystemExit(main())
