"""Hardware-free v2 checks; run with PYTHONPATH=controller:controller/test."""
from pathlib import Path
from dataclasses import replace
import numpy as np
import pytest
import controller.mpc
ROOT = Path(__file__).resolve().parents[2]
controller.mpc.__path__ = [str(ROOT/'jsnd_int/racing_stack/controller/controller/mpc'),
                           *controller.mpc.__path__]
from controller.mpc.integrated_mpc import IntegratedMPC, IntegratedConfig
from offline_support import load_controller, make_controller, cycle, virtual_path


def config(**kw):
    c = IntegratedConfig(wheelbase=.324, max_steer=.4, max_steer_rate=3.2,
        horizon=10, max_speed=1., max_accel=1., max_decel=2., max_lateral_accel=2.,
        braking_preview_ratio=.3, max_iter=4000, solver_time_limit=.015, deadline=.02)
    return replace(c, **kw)


def test_speed_columns_independent():
    path = virtual_path('s_curve')
    state = [0., 0., path[np.argmin(abs(path[:,0])),6], .3]
    outputs = []
    for speed in [0., .01, 100., np.nan, -10.]:
        p = path.copy(); p[:,2] = speed
        # cold solve of this config needs ~2200 ADMM iterations (~19 ms on the Jetson
        # Orin CPU): give the timing budget room, the test is about speed columns
        r = IntegratedMPC(config(dt=.05, solver_time_limit=.04, deadline=.05)).solve(state, p, 0.)
        assert r.success, r.status
        outputs.append(r.controls)
    for u in outputs[1:]: np.testing.assert_allclose(u, outputs[0], atol=1e-10)


@pytest.mark.parametrize('name', ['straight','s_curve','static_avoidance'])
def test_physics_and_convexity(name):
    p = virtual_path(name); m = IntegratedMPC(config()); yaw=p[np.argmin(abs(p[:,0])),6]
    r=m.solve([0,0,yaw,.5],p,0.)
    assert r.success, r.status
    a,d=r.controls.T; v=r.predicted[:,3]; c=m.config
    lat=np.maximum(v[:-1]**2,v[1:]**2)*abs(np.tan(d))/c.wheelbase
    assert np.max((a/np.where(a>=0,c.max_accel,c.max_decel))**2+(lat/c.max_lateral_accel)**2)<=1.001
    assert np.max(abs(d))<=c.max_steer+1e-9
    assert np.all(abs(np.diff(np.r_[0.,d]))<=c.max_steer_rate*m.periods+1e-9)
    assert v.min()>=-1e-4 and v.max()<=c.max_speed+1e-4
    # Positive control penalties + PSD state/rate terms => positive definite P.
    assert np.linalg.eigvalsh(m.h_base).min()>0


def test_terminal_braking_and_short_path():
    m=IntegratedMPC(config()); p=np.c_[np.linspace(0,2,100),np.zeros(100)]
    arc,cap=m._geometry_caps(p,np.inf)
    assert cap[-1]==0
    # step 26: full braking on the friction ellipse (straight path: a_y = 0 -> beta*D)
    assert np.all(np.diff(cap**2)>=-2*m.config.braking_preview_ratio*m.config.max_decel*np.diff(arc)-1e-10)
    assert not m.solve([1.95,0,0,1.],p,0).success


def test_obstacle_cap_and_invalid_input():
    p=virtual_path('straight'); m=IntegratedMPC(config())
    r=m.solve([0,0,0,.1],p,0,safety_speed_cap=.2)
    assert r.success and r.predicted[:,3].max()<=.2001
    brake=m.solve([0,0,0,1.],p,0,safety_speed_cap=0)
    assert brake.success and brake.status=='braking_recovery'
    assert brake.acceleration <= -m.config.max_decel + .01
    assert not m.solve([0,0,0,0],None,0).success


def test_current_context_invalidates_continuation():
    p=virtual_path('straight'); m=IntegratedMPC(config()); r=m.solve([0,0,0,.1],p,0)
    assert r.success
    m._solve=lambda *args: m._fail('deadline_exceeded',0)
    p[:,1]+=.01
    assert not m.solve(r.predicted[1],p,r.steering).success


def test_controller_trailing_and_speed_independence():
    cls=load_controller(ROOT/'jsnd_int/racing_stack/controller/controller/combined/src/Controller.py')
    outputs=[]
    for val in [0., 10., np.nan]:
        # This test checks speed-column independence/soft STOP, not host scheduling.
        c=make_controller(cls=cls,lateral_controller='MPC',lateral_mpc=IntegratedMPC(
            config(dt=.05, solver_time_limit=.04, deadline=.05)))
        p=virtual_path('straight');p[:,2]=val
        out=cycle(c,p,position=[0,0,0],speed=.1)
        assert c.mpc_result.success,c.mpc_result.status
        outputs.append(out[:4])
        out=cycle(c,p,position=[0,0,0],speed=.1,state='TRAILING',opponent=None)
        assert out[0]<outputs[-1][0] and not c.mpc_result.success
        for _ in range(c.mpc_fallback_max_ticks+1):
            out=cycle(c,p,position=[0,0,0],speed=.1,state='TRAILING',opponent=None)
        assert out[0]==0
    np.testing.assert_allclose(outputs,np.tile(outputs[0],(3,1)),atol=1e-10)


def test_continuation_ignores_speed_columns_but_rechecks_obstacle_cap():
    p=virtual_path('straight');m=IntegratedMPC(config());r=m.solve([0,0,0,.1],p,0)
    assert r.success
    m._solve=lambda *args: m._fail('deadline_exceeded',0)
    p[:,2]=np.nan
    r2=m.solve(r.predicted[1],p,r.steering)
    assert r2.success and r2.status.startswith('plan_continuation:')
    assert not m.solve(r.predicted[1],p,r.steering,safety_speed_cap=0.).success


def test_no_relaxation_of_unreachable_braking_limit():
    m=IntegratedMPC(config());p=virtual_path('straight')
    r=m.solve([0,0,0,2.],p,0)
    assert not r.success and r.status=='braking_envelope_infeasible'


def test_pp_code_and_safety_files_preserved():
    import subprocess
    relative='jsnd_int/racing_stack/controller/controller/combined/src/Controller.py'
    old=subprocess.check_output(['git','show','b1e130f6:'+relative],cwd=ROOT,text=True)
    new=(ROOT/relative).read_text()
    assert old[old.index('    def _pp_main_loop'):]==new[new.index('    def _pp_main_loop'):]
    for relative in ['controller/controller/mpc/steering_mpc.py',
                     'controller/controller/safety_gate.py',
                     'stack_master/scripts/simple_mux_node.py']:
        f='jsnd_int/racing_stack/'+relative
        assert subprocess.check_output(['git','show','b1e130f6:'+f],cwd=ROOT)==(ROOT/f).read_bytes()


def test_full_actuator_acceleration_available_on_straight():
    m = IntegratedMPC(config(max_speed=2., q_speed=80.))
    r = m.solve([0., 0., 0., .1], virtual_path('straight'), 0.)
    assert r.success, r.status
    assert r.acceleration > m.config.max_accel/np.sqrt(2.)+.1
    assert r.acceleration <= m.config.max_accel


def test_friction_polygon_axis_limits_and_inner_ellipse():
    m = IntegratedMPC(config()); c = m.config
    vmax = np.full(c.horizon, 2.)
    bounds = np.minimum(c.max_steer, np.arctan(c.max_lateral_accel*c.wheelbase/vmax**2/np.sqrt(2.)))
    matrix = m._friction_rows(vmax, bounds)
    rhs = np.cos(np.pi/16)
    # Both full longitudinal actuator endpoints remain feasible at zero steering.
    for acceleration in [c.max_accel, -c.max_decel]:
        u = np.tile([acceleration, 0.], c.horizon)
        assert np.max(matrix@u) <= rhs+1e-12
        u[0] *= 1.001
        assert np.max(matrix@u) > rhs
    # Exhaustive normalized grid: every accepted polygon point is inside the ellipse.
    scale = vmax[0]**2*np.tan(bounds[0])/bounds[0]/(c.wheelbase*c.max_lateral_accel)
    for x in np.linspace(-1.05, 1.05, 85):
        for y in np.linspace(-1., 1., 81):
            a = x*(c.max_accel if x >= 0 else c.max_decel)
            u = np.tile([a, y/scale], c.horizon)
            if np.max(matrix@u) <= rhs+1e-12:
                assert x*x+y*y <= 1.+1e-12
    # The old rectangle's corners and thus its guaranteed braking reserve survive.
    for a in [-c.max_decel/np.sqrt(2.), c.max_accel/np.sqrt(2.)]:
        for sign in [-1, 1]:
            assert np.max(matrix@np.tile([a, sign/np.sqrt(2.)/scale], c.horizon)) <= rhs+1e-12


@pytest.mark.parametrize('reason', IntegratedMPC.CONTINUATION_REASONS)
def test_transient_failure_bounded_ticks_then_stop(reason, monkeypatch):
    m = IntegratedMPC(config()); p = virtual_path('straight')
    r = m.solve([0, 0, 0, .1], p, 0.); assert r.success
    plan = m.plan
    # Deterministic wall clock; the logical control tick still advances.
    monkeypatch.setattr('controller.mpc.integrated_mpc.perf_counter', lambda: plan['started']+.02)
    m._solve = lambda *args: m._fail(reason, 0.)
    for tick in range(1, IntegratedMPC.MAX_CONTINUATION_TICKS+1):
        t = tick*m.config.dt
        state = np.array([np.interp(t, m.node_times, plan['predicted'][:, j]) for j in range(4)])
        out = m.solve(state, p, r.steering)
        assert out.success and out.status == 'plan_continuation:'+reason
        assert -m.config.max_decel <= out.acceleration <= m.config.max_accel
    assert not m.solve(state, p, out.steering).success
    assert m.plan is None


@pytest.mark.parametrize('reason', ['osqp:primal infeasible', 'osqp:dual infeasible',
    'constraint_violation', 'nonlinear_prediction_outside_envelope', 'braking_envelope_infeasible'])
def test_unsafe_failure_never_reuses_plan(reason):
    m = IntegratedMPC(config()); p = virtual_path('straight')
    r = m.solve([0, 0, 0, .1], p, 0.); assert r.success
    m._solve = lambda *args: m._fail(reason, 0.)
    assert not m.solve(r.predicted[1], p, r.steering).success
    assert m.plan is None


@pytest.mark.parametrize('invalidate', ['age', 'reset', 'state', 'steering', 'cap', 'no_plan'])
def test_continuation_requires_fresh_valid_plan(invalidate, monkeypatch):
    m = IntegratedMPC(config()); p = virtual_path('straight')
    r = m.solve([0, 0, 0, .1], p, 0.); assert r.success
    state, steer, cap = r.predicted[1].copy(), r.steering, np.inf
    if invalidate == 'age':
        then = m.plan['started']+1.
        monkeypatch.setattr('controller.mpc.integrated_mpc.perf_counter', lambda: then)
    elif invalidate == 'reset': m.reset()
    elif invalidate == 'state': state[0] += .5
    elif invalidate == 'steering': steer = m.config.max_steer+.01
    elif invalidate == 'cap': cap = 0.
    elif invalidate == 'no_plan': m.plan = None
    m._solve = lambda *args: m._fail('deadline_exceeded', 0.)
    assert not m.solve(state, p, steer, safety_speed_cap=cap).success
    assert m.plan is None


def test_success_after_miss_refreshes_plan(monkeypatch):
    m = IntegratedMPC(config()); p = virtual_path('straight')
    r = m.solve([0, 0, 0, .1], p, 0.); assert r.success
    with monkeypatch.context() as patch:
        patch.setattr(m, '_solve', lambda *args: m._fail('deadline_exceeded', 0.))
        r = m.solve(r.predicted[1], p, r.steering)
        assert r.success and m.continuation == 1
    r = m.solve([0.004, 0, 0, .1], p, r.steering)
    assert r.success and r.status == 'solved'
    assert m.continuation == 0 and m.plan is not None


@pytest.mark.parametrize('status', ['maximum iterations reached', 'run time limit reached', 'solved inaccurate'])
def test_osqp_failure_reuses_only_previously_verified_controls(status, monkeypatch):
    from types import SimpleNamespace
    m = IntegratedMPC(config()); p = virtual_path('straight')
    r = m.solve([0, 0, 0, .1], p, 0.); assert r.success
    plan = m.plan
    # Deliberately unsafe last iterate may seed the next solve, never the command.
    failed = SimpleNamespace(info=SimpleNamespace(status_val=7, status=status),
                             x=np.full_like(m.previous, 1000.), y=np.zeros_like(m.previous_dual))
    monkeypatch.setattr(m.solver, 'solve', lambda **kwargs: failed)
    monkeypatch.setattr('controller.mpc.integrated_mpc.perf_counter', lambda: plan['started']+.02)
    continued = m.solve(r.predicted[1], p, r.steering)
    assert continued.success and continued.status == 'plan_continuation:osqp:'+status
    assert continued.acceleration == pytest.approx(plan['controls'][1, 0])
    assert abs(continued.steering-r.steering) <= m.config.max_steer_rate*m.config.dt+1e-9


def test_continuation_revalidates_short_stages_and_expires(monkeypatch):
    m = IntegratedMPC(config(preview_dt=.01)); p = virtual_path('straight')
    r = m.solve([0, 0, 0, .1], p, 0.); assert r.success
    then = m.plan['started']+.02
    monkeypatch.setattr('controller.mpc.integrated_mpc.perf_counter', lambda: then)
    monkeypatch.setattr(m, '_solve', lambda *args: m._fail('deadline_exceeded', 0.))
    out=m.solve(r.predicted[1], p, r.steering)
    assert out.success and abs(out.acceleration)<=m.config.max_accel
    monkeypatch.setattr('controller.mpc.integrated_mpc.perf_counter', lambda: then+1.)
    assert not m.solve(r.predicted[1], p, r.steering).success


# ---- step 26 (2026-11-24): constraint revision for lap time ----
def car_config(**kw):
    """Real-car integrated limits (mpc_car.yaml step 26: ggv ax 5.0 / ay 4.5, beta 1.0)."""
    return config(max_speed=5., max_accel=5., max_decel=5., max_lateral_accel=4.5,
                  braking_preview_ratio=1., **kw)


def test_step26_lateral_cap_uses_plan_fraction_of_full_y():
    m = IntegratedMPC(car_config()); r = 2.
    t = np.linspace(0, 2.5, 120); p = np.c_[r*np.sin(t), r*(1-np.cos(t))]
    _, cap = m._geometry_caps(p, np.inf)
    v_lat = np.sqrt(m.LATERAL_PLAN_FRACTION*4.5*r)
    assert np.allclose(cap[5:40], v_lat, rtol=2e-2)      # no Y/sqrt(2), single margin
    assert cap[-1] == 0.


def test_step26_standstill_far_behind_reference_is_not_stop():
    # Previously the 'tracking' check measured distance to the time-indexed reference
    # sample and STOPped a car that merely accelerated slower than the reference.
    p = virtual_path('straight'); m = IntegratedMPC(car_config())
    for _ in range(3):
        r = m.solve([0., 0., 0., 0.], p, 0.)
        assert r.success, r.status


def test_step35_safety_cap_drop_brakes_without_losing_tracking():
    # Speed slightly above an (obstacle) cap the full deceleration cannot meet in
    # 20 ms: brake as hard as feasible while tracking instead of STOP.
    p = virtual_path('straight')
    r = IntegratedMPC(car_config()).solve([0., 0., 0., 1.2], p, 0., safety_speed_cap=1.)
    assert r.success and r.status == 'braking_recovery', r.status
    assert r.acceleration <= -5.+1e-2           # caps carry +1e-4 m/s numerical slack
    # Step 35: sudden obstacle cap drops also request maximum feasible braking.
    r = IntegratedMPC(car_config()).solve([0., 0., 0., 2.], p, 0., safety_speed_cap=1.)
    assert r.success and r.status == 'braking_recovery'
    assert r.acceleration <= -4.99


def test_step26_stacked_matrix_equals_vstack():
    m = IntegratedMPC(car_config()); rng = np.random.default_rng(3)
    for _ in range(10):
        v = rng.uniform(0, 5, 10); b = rng.uniform(.05, .4, 10)
        np.testing.assert_array_equal(m._stacked_matrix(v, b).toarray(),
                                      m.sparse.vstack((m.matrix, m._friction_rows(v, b))).toarray())
