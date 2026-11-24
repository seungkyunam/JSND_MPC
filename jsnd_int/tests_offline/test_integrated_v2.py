"""Hardware-free v2 checks; run with PYTHONPATH=controller:controller/test."""
from pathlib import Path
from dataclasses import replace
import numpy as np
import pytest
import controller.mpc
ROOT = Path(__file__).resolve().parents[2]
controller.mpc.__path__.insert(0, str(ROOT/'jsnd_int/racing_stack/controller/controller/mpc'))
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
        r = IntegratedMPC(config()).solve(state, p, 0.)
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
    assert np.all(np.diff(cap**2)>=-2*m.config.braking_preview_ratio*m.config.max_decel/np.sqrt(2)*np.diff(arc)-1e-10)
    assert not m.solve([1.95,0,0,1.],p,0).success


def test_obstacle_cap_and_invalid_input():
    p=virtual_path('straight'); m=IntegratedMPC(config())
    r=m.solve([0,0,0,.1],p,0,safety_speed_cap=.2)
    assert r.success and r.predicted[:,3].max()<=.2001
    assert not m.solve([0,0,0,1.],p,0,safety_speed_cap=0).success
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
        c=make_controller(cls=cls,lateral_controller='MPC',lateral_mpc=IntegratedMPC(config()))
        p=virtual_path('straight');p[:,2]=val
        out=cycle(c,p,position=[0,0,0],speed=.1)
        assert c.mpc_result.success,c.mpc_result.status
        outputs.append(out[:4])
        out=cycle(c,p,position=[0,0,0],speed=.1,state='TRAILING',opponent=None)
        assert out[0]==0 and not c.mpc_result.success
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
