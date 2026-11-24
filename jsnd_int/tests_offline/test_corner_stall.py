"""2026-11-22 corner stall fix (2p25 profile, 50 Hz, N=10): profile refinement,
steering-rate-consistent speed caps, bounded plan continuation on compute-budget misses.

  PYTHONPATH=~/JSND_INT/src/racing_stack/controller:~/JSND_INT/pydeps:$PYTHONPATH \
      python3 -m pytest -q ~/JSND_INT/tests_offline/test_corner_stall.py
"""
import collections
import dataclasses

import numpy as np
import yaml

from controller.mpc import integrated_mpc as im
from controller.mpc.configuration import MPC_DEFAULTS, make_config
from test_stabilization import CAR, CFG, DATA, LOCK, legacy_module

P2 = yaml.safe_load((CFG / 'mpc_profile_2p25.yaml').read_text())['controller_manager']['ros__parameters']
RATE = 50
DRIVE = np.flatnonzero(~LOCK)


def config(**over):
    v = {k: CAR.get(k, d) for k, d in MPC_DEFAULTS.items()}
    v.update(P2)
    v.update({'mpc_deadline_s': .8/RATE, 'mpc_solver_time_limit_s': .4/RATE})  # race.launch values
    v.update({'mpc_max_steer_rate_radps': 0.3})  # step-17 record (integrated MPC at 0.3 rad/s; step 23 -> 3.2)
    # step-17 record: integrated limits then (step 26 -> ggv 5.0/5.0/4.5, beta 1.0)
    v.update({'mpc_max_accel_mps2': 1.0, 'mpc_max_decel_mps2': 2.0,
              'mpc_max_lateral_accel_mps2': 2.0, 'mpc_braking_preview_ratio': 0.3})
    v.update(over)
    return make_config(v, CAR['wheelbase'], True, RATE)


def replay(mpc, idx=DRIVE):
    out = []
    for i in idx:
        prev = float(np.clip(DATA['prev_steering'][i], -mpc.config.max_steer, mpc.config.max_steer))
        out.append(mpc.solve(DATA['state'][i], DATA['waypoints'][i], prev))
    return out


def straight(n=70, step=.05, speed=2.25):
    x = np.arange(n)*step
    return np.column_stack((x, np.zeros(n), np.full(n, speed)))


def test_config_is_2p25_50hz_n12():
    c = config()
    assert (c.max_speed, c.horizon, c.dt) == (2.25, 12, 1/RATE)


def test_profile_refinement_removes_profile_stops_without_relaxing_the_check():
    old = legacy_module().IntegratedMPC(config())
    new = im.IntegratedMPC(config())
    a, b = replay(old), replay(new)
    ca = collections.Counter(r.status for r in a)
    cb = collections.Counter(r.status for r in b)
    print('legacy', ca.most_common(), '\nnew', cb.most_common())
    assert ca['nonlinear_speed_profile_violation'] > 0
    assert cb['nonlinear_speed_profile_violation'] <= 1
    # What is left is real overspeed: the car is already above the profile where it is
    # (cannot be met even at max_decel) -> the strict check still STOPs (safety kept).
    for i, r in zip(DRIVE, b):
        if r.status == 'nonlinear_speed_profile_violation':
            st, path = DATA['state'][i], DATA['waypoints'][i]
            assert st[3] > path[np.argmin(np.hypot(*(path[:, :2]-st[:2]).T)), 2]
    # Every accepted plan meets the profile at its own predicted positions (strict check).
    for i, r in zip(DRIVE, b):
        if r.status != 'solved':
            continue
        path = DATA['waypoints'][i]
        seg = np.diff(path[:, :2], axis=0)
        for p in r.predicted[1:]:
            off = p[:2]-path[:-1, :2]
            f = np.clip(np.sum(off*seg, 1)/np.sum(seg**2, 1), 0, 1)
            j = np.argmin(np.sum((off-f[:, None]*seg)**2, 1))
            cap = min((1-f[j])*path[j, 2]+f[j]*path[j+1, 2], 2.25)
            assert p[3] <= cap+1e-4


def corner_exit(delta0=.3, v=1.3, n=70, step=.05, L=.324):
    """Curvature unwinding from tan(delta0)/L to 0 at the 0.3 rad/s steering rate, then straight."""
    s = np.arange(n)*step
    k = np.maximum(0., np.tan(delta0)/L - .3/(L*v)*s)
    yaw = np.r_[0., np.cumsum(k[:-1]*step)]
    x, y = np.r_[0., np.cumsum(np.cos(yaw[:-1])*step)], np.r_[0., np.cumsum(np.sin(yaw[:-1])*step)]
    return np.column_stack((x, y, np.full(n, 2.25)))


def test_unwinding_steering_limits_speed_instead_of_infeasible():
    """Corner exit onto a fast straight at 0.3 rad: the rate limit keeps |delta| high for
    a while. Legacy: empty coupled steering box (primal infeasible -> STOP). New: the speed
    cap follows the reachable steering, so the plan accelerates only as the wheels unwind."""
    c = config()
    state = np.array([0., 0., 0., 1.3])
    old = legacy_module().IntegratedMPC(c).solve(state, corner_exit(), .3)
    new = im.IntegratedMPC(c).solve(state, corner_exit(), .3)
    assert old.status == 'osqp:primal infeasible', old.status
    assert new.status == 'solved', new.status
    v, d = new.predicted[:, 3], new.controls[:, 1]
    lat = np.maximum(v[:-1]**2, v[1:]**2)*np.abs(np.tan(d))/c.wheelbase
    assert lat.max() <= c.max_lateral_accel+1e-3
    assert np.all(np.abs(np.diff(np.r_[.3, d])) <= c.max_steer_rate*np.r_[c.dt, np.full(9, c.preview_dt)]+1e-6)


def _solved_then_deadline(c):
    mpc = im.IntegratedMPC(c)
    state = np.array([0., 0., 0., 1.0])
    first = mpc.solve(state, straight(), 0.)
    assert first.status == 'solved'
    late = dataclasses.replace(c)
    object.__setattr__(late, 'deadline', 1e-9)  # every later solve misses the deadline (test only)
    mpc.config = late
    return mpc, first


def _expected_state(first, ticks, c):
    t = ticks*c.dt
    nodes = np.r_[0., np.cumsum(np.r_[c.dt, np.full(c.horizon-1, c.preview_dt)])]
    return np.array([np.interp(t, nodes, first.predicted[:, k]) for k in range(4)])


def test_deadline_miss_continues_verified_plan_at_most_two_ticks():
    c = config()
    mpc, first = _solved_then_deadline(c)
    steer = first.steering
    for tick in (1, 2):
        r = mpc.solve(_expected_state(first, tick, c), straight(), steer)
        assert r.success and r.status == 'plan_continuation:deadline_exceeded', r.status
        assert abs(r.steering-steer) <= c.max_steer_rate*c.dt+1e-9
        assert r.acceleration >= -c.max_decel-1e-9 and r.speed <= 2.25
        steer = r.steering
    third = mpc.solve(_expected_state(first, 3, c), straight(), steer)
    assert not third.success and third.status == 'deadline_exceeded'


def test_no_continuation_when_state_left_the_plan_or_for_safety_failures():
    c = config()
    mpc, first = _solved_then_deadline(c)
    off = _expected_state(first, 1, c) + np.array([0., .3, 0., 0.])   # 0.3 m lateral jump
    assert mpc.solve(off, straight(), first.steering).status == 'deadline_exceeded'
    # path / envelope failure is never continued
    mpc, first = _solved_then_deadline(c)
    far = _expected_state(first, 1, c) + np.array([0., 5., 0., 0.])
    r = mpc.solve(far, straight(), first.steering)
    assert not r.success and 'outside_tracking_envelope' in r.status
    # an external STOP (input loss / unsupported behaviour) drops the plan
    mpc, first = _solved_then_deadline(c)
    mpc.reject('invalid_local_control_input:test', first.steering)
    assert mpc.solve(_expected_state(first, 1, c), straight(), first.steering).status == 'deadline_exceeded'


def test_compute_time_on_drive_windows():
    mpc = im.IntegratedMPC(config(mpc_deadline_s=1/RATE, mpc_solver_time_limit_s=1/RATE))
    el = np.array([r.elapsed_s for r in replay(mpc)])*1e3
    print('elapsed ms p50 %.2f p99 %.2f max %.2f' % (*np.percentile(el, [50, 99]), el.max()))
    assert np.percentile(el, 50) < 8.
