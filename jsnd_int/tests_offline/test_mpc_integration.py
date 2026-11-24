"""Offline checks for the JSND_INT integrated MPC wiring (no ROS graph, no hardware).

Run with the overlay sourced and osqp on PYTHONPATH:
  PYTHONPATH=~/JSND_INT/pydeps:$PYTHONPATH python3 -m pytest -q ~/JSND_INT/tests_offline
"""
import inspect
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from controller.combined.src.Controller import Controller
from controller.mpc.configuration import MPC_DEFAULTS, make_config
from controller.mpc.integrated_mpc import IntegratedMPC

ROOT = Path.home() / 'JSND_INT/src/racing_stack/stack_master'
CAR = yaml.safe_load((ROOT / 'config/mpc_car.yaml').read_text())['controller_manager']['ros__parameters']
ERPM_GAIN = 4614.0          # CAR/vehicle_config.yaml speed_to_erpm_gain
VESC_MIN_ERPM = 900.0        # assumed VESC speed-PID minimum ERPM (to be read from VESC Tool)
DT = 1.0 / CAR['control_loop_rate_hz']


def load_raceline():
    data = json.loads((ROOT / 'maps/11_18_6/global_waypoints.json').read_text())
    w = data['global_traj_wpnts_iqp']['wpnts']   # what /global_waypoints carries
    rows = [[p['x_m'], p['y_m'], p['vx_mps'], 0, p['s_m'], p['kappa_radpm'], p['psi_rad'], p['ax_mps2'], p['d_m']]
            for p in w]
    return np.asarray(rows, float)


RACELINE = load_raceline()


def local_window(i, n=60):
    idx = (np.arange(n) + i) % len(RACELINE)
    return RACELINE[idx]


# Unverified track geometry (sim value) - only to exercise the algorithm around real corners.
TRACK_STEER = {'mpc_max_steer_rad': 0.4, 'mpc_max_steer_rate_radps': 3.2}


def make_controller(overrides=None):
    values = {k: CAR.get(k, v) for k, v in MPC_DEFAULTS.items()}
    values.update(overrides or {})
    mpc = IntegratedMPC(make_config(values, CAR['wheelbase'], True, CAR['control_loop_rate_hz']))
    sig = inspect.signature(Controller.__init__)
    kwargs = {name: 0.0 for name, p in sig.parameters.items()
              if name != 'self' and p.default is inspect.Parameter.empty}
    kwargs.update(converter=None, lateral_controller='MPC', lateral_mpc=mpc,
                  pose_to_rear_axle_m=CAR['mpc_pose_to_rear_axle_m'],
                  mpc_cmd_max_lead_mps=CAR['mpc_cmd_max_lead_mps'])
    return Controller(**kwargs)


def pose_on_line(i):
    p = RACELINE[i]
    return np.array([[p[0], p[1], p[6]]])


def step(ctrl, state, pose, wpts, v):
    out = ctrl.main_loop(state, pose, wpts, v, None, np.zeros(4), 0.0, 0.0)
    return out[0], out[3]


def nearest(pose):
    return int(np.argmin(np.linalg.norm(RACELINE[:, :2] - pose[0, :2], axis=1)))


def closed_loop(ctrl, ticks, legacy_setpoint=False, wheels_blocked=False, lifted=False):
    """Kinematic car + VESC model: wheel speed follows setpoint (tau 0.15 s) only above min ERPM."""
    pose = pose_on_line(0).astype(float)
    v, steer = 0.0, 0.0
    log = []
    for _ in range(ticks):
        i = nearest(pose)
        cmd, steer = step(ctrl, 'GB_TRACK', pose, local_window(i), v)
        if legacy_setpoint:   # previous behaviour: published speed = v_meas + a*dt
            cmd = max(0.0, v + ctrl.mpc_result.acceleration * DT) if ctrl.mpc_result.success else 0.0
        driven = (not wheels_blocked) and abs(cmd * ERPM_GAIN) >= VESC_MIN_ERPM
        target = cmd if driven else 0.0
        v += (target - v) * DT / 0.15
        if not lifted:   # wheels lifted: wheels spin, chassis (localized pose) stays put
            yaw = pose[0, 2]
            pose[0, 0] += v * np.cos(yaw) * DT
            pose[0, 1] += v * np.sin(yaw) * DT
            pose[0, 2] += v * np.tan(steer) / CAR['wheelbase'] * DT
        log.append((cmd, v, steer, ctrl.mpc_result.status, ctrl.mpc_result.elapsed_s))
    return np.array([r[:3] for r in log]), [r[3] for r in log], np.array([r[4] for r in log])


def test_legacy_setpoint_never_leaves_standstill():
    log, _, _ = closed_loop(make_controller(), 80, legacy_setpoint=True)
    assert np.max(log[:, 0]) * ERPM_GAIN < VESC_MIN_ERPM
    assert np.max(log[:, 1]) < 1e-6


def test_bench_lifted_conservative_steer():
    """Next real test: wheels lifted, mpc_car.yaml as is (step 12: steer 0.4 rad / 0.3 rad/s)."""
    log, status, _ = closed_loop(make_controller(), 400, lifted=True)
    cmd, v, steer = log[:, 0], log[:, 1], log[:, 2]
    assert all(s == 'solved' for s in status), set(status)
    assert cmd[0] <= CAR['mpc_max_accel_mps2'] * DT + 1e-9          # starts from 0
    assert np.all(np.diff(cmd) <= CAR['mpc_max_accel_mps2'] * DT + 1e-9)  # limited accel
    assert v.max() > 0.5 and cmd.max() <= CAR['mpc_max_speed_mps'] + 1e-9
    assert np.abs(steer).max() <= CAR['mpc_max_steer_rad'] + 1e-9
    assert np.all(np.abs(np.diff(steer)) <= CAR['mpc_max_steer_rate_radps'] * DT + 1e-9)


VEHICLE = yaml.safe_load((ROOT / 'config/CAR/vehicle_config.yaml').read_text())['vesc/**']['ros__parameters']
LAUNCH = (ROOT / 'launch/race.launch.xml').read_text()


def test_step12_steer_limit_and_servo_range():
    """MPC steer = legacy FTG/manual 0.4 rad; rate 3.2 rad/s since step 23 (was 0.3 bench value);
    servo command stays inside VESC 0..1."""
    assert CAR['mpc_max_steer_rad'] == 0.4 and CAR['mpc_max_steer_rate_radps'] == 3.2
    assert 'ftg_max_steer: 0.4' in (ROOT / 'config/controller.yaml').read_text()
    gain, off = VEHICLE['steering_angle_to_servo_gain'], VEHICLE['steering_angle_to_servo_offset']
    servo = [off + gain * d for d in (-CAR['mpc_max_steer_rad'], CAR['mpc_max_steer_rad'])]
    assert VEHICLE['servo_min'] < min(servo) and max(servo) < VEHICLE['servo_max'], servo
    for prof in ROOT.glob('config/mpc_profile_*.yaml'):   # profiles must not override steering
        assert 'steer' not in prof.read_text().split('ros__parameters')[-1]
    assert CAR['control_loop_rate_hz'] == 100 and CAR['mpc_horizon'] == 10


def test_step12_lb_manual_speed_mpc_only():
    """race.launch: joy_max_speed 1.0 only for controller:=mpc, legacy 4.0 otherwise."""
    assert ('<arg name="joy_max_speed" default="$(eval &quot;\'1.0\' if \'$(var controller)\' == '
            '\'mpc\' else \'4.0\'&quot;)"') in LAUNCH
    low = (ROOT / 'launch/low_level.launch.xml').read_text()
    assert 'name="joy_max_speed"  default="4.0"' in low


def test_step12_lap_with_car_limits():
    """One 11_18_6 lap at 1.0 m/s with mpc_car.yaml limits as shipped (0.4 rad / 0.3 rad/s)."""
    ctrl = make_controller()
    length = np.sum(np.linalg.norm(np.diff(RACELINE[:, :2], axis=0), axis=1))
    pose = pose_on_line(0).astype(float); v = dist = max_lat = 0.0; steers = []
    while dist < length and len(steers) < 3000:
        cmd, steer = step(ctrl, 'GB_TRACK', pose, local_window(nearest(pose)), v)
        assert ctrl.mpc_result.status == 'solved', ctrl.mpc_result.status
        v += ((cmd if abs(cmd * ERPM_GAIN) >= VESC_MIN_ERPM else 0.0) - v) * DT / 0.15
        yaw = pose[0, 2]
        pose[0, 0] += v * np.cos(yaw) * DT; pose[0, 1] += v * np.sin(yaw) * DT
        pose[0, 2] += v * np.tan(steer) / CAR['wheelbase'] * DT
        dist += v * DT; steers.append(steer)
        max_lat = max(max_lat, np.min(np.linalg.norm(RACELINE[:, :2] - pose[0, :2], axis=1)))
    steers = np.array(steers)
    assert dist >= length and max_lat < 0.10
    assert np.abs(steers).max() <= 0.4 + 1e-9 and np.abs(steers).max() > 0.15   # 0.15 would not suffice
    print(f'\nlap {length:.1f} m in {len(steers)*DT:.1f} s, max|steer| {np.abs(steers).max():.3f}, '
          f'max rate {np.abs(np.diff(steers)).max()/DT:.3f} rad/s, max lateral {max_lat:.3f} m')


def test_integrated_setpoint_starts_and_respects_limits():
    """Driving around 11_18_6 corners needs > 0.15 rad: uses the UNVERIFIED 0.4 rad / 3.2 rad/s."""
    log, status, elapsed = closed_loop(make_controller(TRACK_STEER), 400)   # 10 s
    cmd, v = log[:, 0], log[:, 1]
    assert all(s == 'solved' for s in status), set(status)
    assert np.max(v) > 0.5                                       # car actually moves
    assert np.max(cmd) <= CAR['mpc_max_speed_mps'] + 1e-9        # speed limit
    assert np.all(cmd >= 0)
    assert np.all(cmd - v <= CAR['mpc_cmd_max_lead_mps'] + 1e-9)  # anti-windup lead
    assert np.max(np.abs(log[:, 2])) <= TRACK_STEER['mpc_max_steer_rad'] + 1e-9
    # speed and steering are both produced by the MPC
    assert np.ptp(cmd) > 0.3 and np.ptp(log[:, 2]) > 0.05
    print(f'\nsolver ms p50={1e3*np.median(elapsed):.2f} p99={1e3*np.percentile(elapsed, 99):.2f} '
          f'max={1e3*elapsed.max():.2f}; first tick with cmd >= min ERPM: '
          f'{np.argmax(cmd*ERPM_GAIN >= VESC_MIN_ERPM)*DT:.3f}s')


def test_blocked_wheels_setpoint_is_bounded():
    log, _, _ = closed_loop(make_controller(), 200, wheels_blocked=True)
    assert np.max(log[:, 0]) <= CAR['mpc_cmd_max_lead_mps'] + 1e-9


def test_stop_resets_setpoint_and_unsupported_state_stops():
    ctrl = make_controller()
    pose, w = pose_on_line(0), local_window(0)
    for _ in range(40):
        cmd, _ = step(ctrl, 'GB_TRACK', pose, w, 0.3)
    assert cmd > 0
    cmd, _ = step(ctrl, 'TRAILING', pose, w, 0.3)
    assert cmd == 0.0 and ctrl.mpc_speed_cmd is None
    cmd, _ = step(ctrl, 'GB_TRACK', pose, w, 0.3)                 # restarts from measured speed
    assert cmd <= 0.3 + DT * CAR['mpc_max_accel_mps2'] + 1e-9


@pytest.mark.parametrize('speed,ok', [(-0.01, True), (-0.1, False), (float('nan'), False)])
def test_speed_input_validation(speed, ok):
    ctrl = make_controller()
    cmd, _ = step(ctrl, 'GB_TRACK', pose_on_line(0), local_window(0), speed)
    assert ctrl.mpc_result.success is ok
    if not ok:
        assert cmd == 0.0


def test_off_path_requests_stop():
    ctrl = make_controller()
    pose = pose_on_line(0) + np.array([[0.0, 2.0, 0.0]])
    cmd, _ = step(ctrl, 'GB_TRACK', pose, local_window(0), 0.5)
    assert cmd == 0.0 and not ctrl.mpc_result.success


def test_solver_stop_while_moving_ramps_down_then_zero():
    """Step 32: a solver STOP while moving ramps the setpoint at max_decel with steering
    held for mpc_fallback_max_ticks, then commands 0 (no one-tick 0 -> VESC braking jolt)."""
    ctrl = make_controller()
    pose, w = pose_on_line(0), local_window(0)
    for _ in range(40):
        cmd, steer = step(ctrl, 'GB_TRACK', pose, w, 0.3)
    assert cmd > 0 and ctrl.mpc_result.success
    off = pose + np.array([[0.0, 2.0, 0.0]])
    dv = ctrl.lateral_mpc.config.max_decel * DT
    prev = cmd
    for _ in range(ctrl.mpc_fallback_max_ticks):
        cmd, s = step(ctrl, 'GB_TRACK', off, w, 0.3)
        assert not ctrl.mpc_result.success
        assert cmd == pytest.approx(max(0.0, prev - dv)) and s == steer
        prev = cmd
        if cmd == 0.0:
            break
    cmd, _ = step(ctrl, 'GB_TRACK', off, w, 0.3)
    assert cmd == 0.0
