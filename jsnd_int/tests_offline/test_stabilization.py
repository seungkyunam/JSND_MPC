"""2026-11-22 stabilisation: OSQP lock-up, STOP steering, 100 Hz budget, profiles, safety gate.

Uses real bench inputs recorded on 2026-11-22 (bag_mpc_1230, max_speed 2.0, 40 Hz):
bench_20261122_mpc_inputs.npz holds /car_state/odom state, the 70 local waypoints of
/behavior_strategy and the previously published steering for ticks of the three
"osqp: maximum iterations reached" lock windows and three driving windows.

  PYTHONPATH=~/JSND_INT/src/racing_stack/controller:~/JSND_INT/pydeps:$PYTHONPATH \
      python3 -m pytest -q ~/JSND_INT/tests_offline
"""
import collections
import importlib.util
import inspect
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

from controller.combined.src.Controller import Controller
from controller.mpc import integrated_mpc as im
from controller.mpc.configuration import MPC_DEFAULTS, make_config
from controller.safety_gate import SafetyGate

HERE = Path(__file__).parent
REPO = Path.home() / 'JSND_INT/src/racing_stack'
CFG = REPO / 'stack_master/config'
CAR = yaml.safe_load((CFG / 'mpc_car.yaml').read_text())['controller_manager']['ros__parameters']
PROFILES = {name: yaml.safe_load((CFG / f'mpc_profile_{name}.yaml').read_text())['controller_manager']['ros__parameters']
            for name in ('lowspeed', 'exp2mps')}
DATA = np.load(HERE / 'bench_20261122_mpc_inputs.npz')
LOCK = np.char.startswith(DATA['window'], 'lock')
STANDSTILL = DATA['state'][:, 3] < 0.05


def values(overrides=None, profile='exp2mps'):
    v = {k: CAR.get(k, d) for k, d in MPC_DEFAULTS.items()}
    v.update(PROFILES[profile])
    v.update(overrides or {})
    return v


def make_mpc(overrides=None, rate=None, profile='exp2mps'):
    return im.IntegratedMPC(make_config(values(overrides, profile), CAR['wheelbase'], True,
                                        rate or CAR['control_loop_rate_hz']))


def legacy_module():
    """integrated_mpc.py exactly as run on the car on 2026-11-22 (JSND_INT 82a1fd5)."""
    src = subprocess.run(['git', '-C', str(REPO), 'show', '82a1fd5:controller/controller/mpc/integrated_mpc.py'],
                         check=True, capture_output=True, text=True).stdout
    spec = importlib.util.spec_from_loader('controller.mpc._legacy_integrated_mpc', loader=None)
    mod = importlib.util.module_from_spec(spec)
    mod.__package__ = 'controller.mpc'
    sys.modules[spec.name] = mod
    exec(compile(src, 'legacy_integrated_mpc.py', 'exec'), mod.__dict__)
    return mod


# Legacy bench configuration (what ran at 12:37-12:48): 40 Hz, 400 iterations, 12 ms.
LEGACY = {'mpc_max_iter': 400, 'mpc_solver_time_limit_s': 0.012, 'mpc_deadline_s': 0.020,
          'mpc_max_steer_rad': 0.15}   # steer limit in force at the 2026-11-22 bench (step 12 raised it to 0.4)


def replay(mpc, idx, sequential=True):
    out = []
    for i in idx:
        if not sequential:
            mpc.reset()
        out.append(mpc.solve(DATA['state'][i], DATA['waypoints'][i], DATA['prev_steering'][i]))
    return out


# ------------------------------------------------------------------ P8 lock-up
def test_lockup_reproduced_with_deployed_solver():
    """The deployed code + config reproduces the bench lock on the recorded inputs."""
    legacy = legacy_module()
    mpc = legacy.IntegratedMPC(make_config(values(LEGACY), CAR['wheelbase'], True, 40))
    res = replay(mpc, np.flatnonzero(LOCK))
    fails = sum(r.status == 'osqp:maximum iterations reached' for r in res)
    assert fails / len(res) > 0.9, collections.Counter(r.status for r in res)
    assert np.all(STANDSTILL[LOCK]), 'every lock tick was at standstill (v < 0.05 m/s)'


def test_lockup_is_iteration_budget_not_infeasibility():
    """Same legacy code, only max_iter raised: every lock tick solves (budget, not geometry)."""
    legacy = legacy_module()
    mpc = legacy.IntegratedMPC(make_config(values(dict(LEGACY, mpc_max_iter=20000)), CAR['wheelbase'], True, 40))
    res = replay(mpc, np.flatnonzero(LOCK), sequential=False)
    assert all(r.status == 'solved' for r in res), collections.Counter(r.status for r in res)
    assert max(r.iterations for r in res) > 400


def test_held_steering_worsens_but_does_not_alone_cause_lock():
    """Diagnosis record (legacy code + budget): the held off-centre steering makes the
    lock much worse, but zero steering alone still fails most standstill ticks."""
    legacy = legacy_module()
    idx = np.flatnonzero(LOCK)
    solved = {}
    for label, prev in (('held', DATA['prev_steering']), ('zero', np.zeros(len(DATA['state'])))):
        mpc = legacy.IntegratedMPC(make_config(values(LEGACY), CAR['wheelbase'], True, 40))
        solved[label] = sum(mpc.solve(DATA['state'][i], DATA['waypoints'][i], prev[i]).success for i in idx) / len(idx)
    print('legacy solved fraction', solved)
    assert solved['held'] < 0.1
    assert solved['held'] < solved['zero'] < 0.6


@pytest.mark.parametrize('rate', [40, 100])
def test_lock_inputs_solve_with_new_solver(rate):
    mpc = make_mpc(rate=rate)
    seq = replay(mpc, np.flatnonzero(LOCK))
    cold = replay(make_mpc(rate=rate), np.flatnonzero(LOCK), sequential=False)
    for res in (seq, cold):
        statuses = collections.Counter(r.status for r in res)
        assert statuses['osqp:maximum iterations reached'] == 0, statuses
        assert statuses['osqp:run time limit reached'] == 0, statuses
        assert statuses['solved'] >= 0.99 * len(res), statuses     # rare 'solved inaccurate' -> 1-tick STOP


def test_drive_windows_no_osqp_failure_and_equivalent_to_legacy():
    """Optimised module == legacy module within OSQP tolerance on the driving windows."""
    legacy = legacy_module()
    idx = np.flatnonzero(~LOCK)
    unlimited = {'mpc_max_iter': 20000, 'mpc_solver_time_limit_s': 0.02, 'mpc_deadline_s': 0.025}
    old = legacy.IntegratedMPC(make_config(values(unlimited), CAR['wheelbase'], True, 40))
    im.IntegratedMPC.WARM_START_DUAL = False
    try:
        new = make_mpc(unlimited, rate=40)
        a, b = replay(old, idx, False), replay(new, idx, False)
    finally:
        im.IntegratedMPC.WARM_START_DUAL = True
    same = [(x, y) for x, y in zip(a, b) if x.status == y.status == 'solved']
    # 2026-11-22 profile refinement: legacy STOPs on a profile miss at the reached
    # positions; the new module re-solves with tightened stage caps (same strict check).
    changed = collections.Counter((x.status, y.status) for x, y in zip(a, b) if x.status != y.status)
    assert set(changed) <= {('nonlinear_speed_profile_violation', 'solved')}, changed
    assert len(same) > 0.8 * len(idx)
    diff = np.array([[x.steering - y.steering, x.speed - y.speed] for x, y in same])
    assert np.abs(diff).max() < 5e-4
    # inputs were recorded with the 0.15 rad limit -> replay under that limit
    new_seq = replay(make_mpc({'mpc_max_steer_rad': 0.15}, rate=100), idx)
    assert not any(r.status.startswith('osqp:') for r in new_seq)
    # step 12 (0.4 rad): real-car lowspeed profile has no OSQP failure on the same inputs.
    # (exp2mps 2.0 m/s + 0.4 rad: 1/368 ticks hits the 4 ms time limit -> that tick stops; bench-only profile)
    low = replay(make_mpc(rate=100, profile='lowspeed'), idx)
    assert CAR['mpc_max_steer_rad'] == 0.4 and not any(r.status.startswith('osqp:') for r in low)


def test_budget_miss_recovers_over_ticks():
    """If the budget is missed (here the legacy 400 iterations, and 200), the next ticks
    continue from OSQP's last iterate instead of a cold start: on a constant standstill
    input (the bench lock situation) the solver recovers within a few ticks."""
    for budget in (400, 200):
        for i in np.flatnonzero(LOCK)[::20]:
            mpc = make_mpc({'mpc_max_iter': budget}, rate=100)
            res = [mpc.solve(DATA['state'][i], DATA['waypoints'][i], DATA['prev_steering'][i]) for _ in range(10)]
            first_ok = next((k for k, r in enumerate(res) if r.success), None)
            assert first_ok is not None and first_ok <= 5, (budget, i, [r.status for r in res])
            assert all(r.success for r in res[first_ok:])


def test_gradient_matches_numpy():
    rng = np.random.default_rng(0)
    for _ in range(50):
        x = np.cumsum(rng.uniform(0.01, 0.3, 40))
        f = rng.normal(size=(40, 2))
        assert np.allclose(im._gradient(f, x), np.gradient(f, x, axis=0), rtol=0, atol=1e-10)
        assert np.allclose(im._gradient(f[:, 0], x), np.gradient(f[:, 0], x), rtol=0, atol=1e-10)


def test_constant_constraint_matrix_matches_linearisation():
    """The speed rows (and hence A) are independent of the linearisation point."""
    mpc = make_mpc(rate=100)
    n, p = mpc.config.horizon, mpc.periods
    rng = np.random.default_rng(1)
    for _ in range(5):
        v, yaw, delta = rng.uniform(0, 3, n), rng.uniform(-3, 3, n), rng.uniform(-.15, .15, n)
        aa = np.tile(np.eye(4), (n, 1, 1))
        aa[:, 0, 2], aa[:, 0, 3] = -v*np.sin(yaw)*p, np.cos(yaw)*p
        aa[:, 1, 2], aa[:, 1, 3] = v*np.cos(yaw)*p, np.sin(yaw)*p
        aa[:, 2, 3] = np.tan(delta)*p/mpc.config.wheelbase
        bb = np.zeros((n, 4, 2))
        bb[:, 3, 0] = p
        bb[:, 2, 1] = v*p/(mpc.config.wheelbase*np.cos(delta)**2)
        m, rows = np.zeros((4, 2*n)), []
        for k in range(n):
            m = aa[k] @ m
            m[:, 2*k:2*k+2] += bb[k]
            rows.append(m[3].copy())
        dense = np.vstack((np.eye(2*n), mpc.d, np.array(rows)))
        assert np.array_equal(mpc.matrix.toarray(), dense)


# ------------------------------------------------------------------ STOP steering
def make_controller(mpc):
    sig = inspect.signature(Controller.__init__)
    kwargs = {name: 0.0 for name, p in sig.parameters.items()
              if name != 'self' and p.default is inspect.Parameter.empty}
    kwargs.update(converter=None, lateral_controller='MPC', lateral_mpc=mpc,
                  pose_to_rear_axle_m=0.0, mpc_cmd_max_lead_mps=CAR['mpc_cmd_max_lead_mps'])
    return Controller(**kwargs)


def test_stop_recenters_steering_at_rate_limit():
    mpc = make_mpc(rate=100)
    ctrl = make_controller(mpc)
    ctrl.curr_steering_angle = -0.102           # value held for minutes on 2026-11-22
    step = mpc.config.max_steer_rate * mpc.config.dt
    i = np.flatnonzero(LOCK)[0]
    pose = DATA['state'][i][None, :3]
    steer = []
    for _ in range(60):                         # TRAILING -> MPC STOP every tick
        out = ctrl.main_loop('TRAILING', pose, DATA['waypoints'][i], 0.0, None, np.zeros(4), 0., 0.)
        assert out[0] == 0.0
        steer.append(out[3])
    steer = np.array(steer)
    assert np.all(np.abs(np.diff(np.r_[-0.102, steer])) <= step + 1e-12)
    assert steer[-1] == 0.0 and ctrl.curr_steering_angle == 0.0


def test_recovery_after_stop_from_standstill():
    """STOP (unsupported state) -> GB_TRACK at standstill on lock inputs: solved again,
    speed command restarts from 0 and ramps with max_accel."""
    mpc = make_mpc(rate=100)
    ctrl = make_controller(mpc)
    i = np.flatnonzero(LOCK)[10]
    pose, wp = DATA['state'][i][None, :3], DATA['waypoints'][i]
    ctrl.curr_steering_angle = DATA['prev_steering'][i]
    for _ in range(20):
        ctrl.main_loop('TRAILING', pose, wp, 0.0, None, np.zeros(4), 0., 0.)
    speeds = []
    for _ in range(30):
        out = ctrl.main_loop('GB_TRACK', pose, wp, 0.0, None, np.zeros(4), 0., 0.)
        speeds.append(out[0])
        assert ctrl.mpc_result.success, ctrl.mpc_result.status
    speeds = np.array(speeds)
    assert np.all(np.diff(speeds) <= mpc.config.max_accel * mpc.config.dt + 1e-9)
    assert speeds[-1] > 0.0 and speeds.max() <= CAR['mpc_cmd_max_lead_mps'] + 1e-9


# ------------------------------------------------------------------ 100 Hz budget
def test_100hz_compute_budget_offline():
    """Whole solve (Python + OSQP) on recorded inputs, persistent solver, 100 Hz config.
    Offline on an idle Jetson: the on-car margin is checked separately (ROS / bench)."""
    mpc = make_mpc(rate=100)
    for i in range(20):
        mpc.solve(DATA['state'][i], DATA['waypoints'][i], DATA['prev_steering'][i])
    elapsed = []
    for _ in range(3):
        for r in replay(mpc, range(len(DATA['state']))):
            elapsed.append(r.elapsed_s)
    elapsed = np.array(elapsed) * 1e3
    print(f'100 Hz solve ms p50 {np.percentile(elapsed, 50):.2f} p99 {np.percentile(elapsed, 99):.2f} max {elapsed.max():.2f}')
    assert np.percentile(elapsed, 99) < 0.6 * 1e3 * mpc.config.deadline


# ------------------------------------------------------------------ profiles
def test_profiles_and_rate_constraints():
    assert CAR['mpc_horizon'] == 10
    assert CAR['mpc_max_speed_mps'] == 1.0 and PROFILES['lowspeed']['mpc_max_speed_mps'] == 1.0
    assert PROFILES['exp2mps']['mpc_max_speed_mps'] == 2.0
    assert set(PROFILES['exp2mps']) == {'mpc_max_speed_mps'}
    for rate in (40, 100):          # mpc_rate_hz:=40 fallback must still be a valid config
        for prof in PROFILES:
            assert make_config(values(profile=prof), CAR['wheelbase'], True, rate).deadline <= 1.0 / rate
    launch = (REPO / 'stack_master/launch/race.launch.xml').read_text()
    assert 'mpc_profile_$(var mpc_profile).yaml' in launch and 'default="lowspeed"' in launch


# ------------------------------------------------------------------ safety gate
A, LB, RB = 0, 4, 5


def pad(*pressed, n=11):
    return [1 if i in pressed else 0 for i in range(n)]


def ready_gate(t=0.0):
    g = SafetyGate()
    for k in range(60):              # 3 s of idle pad and still wheels (> min_latch_s)
        g.on_joy(pad(), t + 0.05*k)
        g.on_odom(0.0, t + 0.05*k)
    return g, t + 0.05*59


def test_gate_starts_latched_and_needs_approval():
    g, t = ready_gate()
    assert g.update(t) == 'estop' and g.latched
    g.on_joy(pad(RB), t + 0.01)
    assert g.update(t + 0.01) == 'estop'             # RB ignored while latched
    g.on_joy(pad(), t + 0.02)
    ok, msg = g.request_reset(t + 0.02)
    assert ok, msg
    assert g.update(t + 0.02) == 'none'


def test_gate_reset_conditions():
    g, t = ready_gate()
    g.on_joy(pad(A), t)                               # latch again now
    g.on_joy(pad(), t + 0.1)
    assert not g.request_reset(t + 0.5)[0]            # < min_latch_s
    g.on_odom(0.4, t + 2.5)                            # wheels turning
    g.on_joy(pad(), t + 2.5)
    ok, msg = g.request_reset(t + 2.5)
    assert not ok and 'wheels not still' in msg
    for k in range(25):
        g.on_odom(0.0, t + 2.55 + 0.05*k)
        g.on_joy(pad(), t + 2.55 + 0.05*k)
    tt = t + 2.55 + 0.05*24
    g.on_joy(pad(RB), tt)                              # holding a button
    assert 'release every pad button' in g.request_reset(tt)[1]
    g.on_joy(pad(), tt + 0.01)
    assert not g.request_reset(tt + 2.0)[0]            # joy and odom stale
    assert g.request_reset(tt + 0.02)[0]


def approved_gate():
    g, t = ready_gate()
    g.on_joy(pad(), t + 0.01)
    assert g.request_reset(t + 0.01)[0]
    return g, t + 0.01


def test_gate_a_latches_from_auto_and_manual():
    for drive in (RB, LB):
        g, t = approved_gate()
        g.on_joy(pad(), t + 0.05)
        g.on_joy(pad(drive), t + 0.1)
        assert g.update(t + 0.1) == ('autodrive' if drive == RB else 'humandrive')
        g.on_joy(pad(drive, A), t + 0.15)
        assert g.update(t + 0.15) == 'estop'
        g.on_joy(pad(drive), t + 0.2)                 # releasing A does not clear it
        g.on_joy(pad(), t + 0.25)
        g.on_joy(pad(drive), t + 0.3)
        assert g.update(t + 0.3) == 'estop'           # both auto and manual blocked


def test_gate_lb_is_manual_hold_and_overrides_rb():
    g, t = approved_gate()
    g.on_joy(pad(LB), t + 0.05)
    assert g.update(t + 0.05) == 'humandrive' and not g.latched
    g.on_joy(pad(), t + 0.1)
    assert g.update(t + 0.1) == 'none'                # dead-man: release -> 0
    g.on_joy(pad(RB), t + 0.15)
    assert g.update(t + 0.15) == 'autodrive'
    g.on_joy(pad(RB, LB), t + 0.2)
    assert g.update(t + 0.2) == 'humandrive'          # LB takes over (legacy priority)
    g.on_joy(pad(RB), t + 0.25)
    assert g.update(t + 0.25) == 'none'               # RB still held: needs a new press
    g.on_joy(pad(), t + 0.3)
    g.on_joy(pad(RB), t + 0.35)
    assert g.update(t + 0.35) == 'autodrive'


def test_gate_lb_blocked_while_latched_and_startup():
    g, t = ready_gate()                               # starts latched
    g.on_joy(pad(LB), t + 0.01)
    assert g.update(t + 0.01) == 'estop'
    g.on_joy(pad(), t + 0.02)
    g.on_joy(pad(LB), t + 0.03)
    assert not g.request_reset(t + 0.03)[0]           # LB held -> refused


def test_gate_joy_loss_latches_in_manual():
    g, t = approved_gate()
    g.on_joy(pad(LB), t + 0.05)
    assert g.update(t + 0.3) == 'humandrive'
    assert g.update(t + 0.6) == 'estop' and 'joy lost' in g.reason


def test_gate_engage_requires_new_press_and_hold():
    g, t = ready_gate()
    g.on_joy(pad(RB), t + 0.01)                       # RB held through the reset
    assert not g.request_reset(t + 0.01)[0]
    g.on_joy(pad(), t + 0.02)
    assert g.request_reset(t + 0.02)[0]
    g.on_joy(pad(), t + 0.05)
    g.on_joy(pad(RB), t + 0.1)
    assert g.update(t + 0.1) == 'autodrive'
    g.on_joy(pad(RB), t + 0.15)
    assert g.update(t + 0.15) == 'autodrive'
    g.on_joy(pad(), t + 0.2)                          # hold-to-run: release -> none
    assert g.update(t + 0.2) == 'none' and not g.latched


def test_gate_joy_loss_latches_only_when_engaged():
    g, t = approved_gate()
    assert g.update(t + 5.0) == 'none'                # idle: no latch from pad silence
    g.on_joy(pad(), t + 5.0)
    g.on_joy(pad(RB), t + 5.05)
    assert g.update(t + 5.3) == 'autodrive'
    assert g.update(t + 5.6) == 'estop' and 'joy lost' in g.reason


def test_gate_rejects_overlapping_buttons():
    with pytest.raises(ValueError):
        SafetyGate(estop_buttons=(0, 5), engage_button=5)
    with pytest.raises(ValueError):
        SafetyGate(estop_buttons=(0, 4), engage_button=5, manual_button=4)
