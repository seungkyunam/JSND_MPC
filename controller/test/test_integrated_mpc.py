from dataclasses import replace
from unittest.mock import Mock
import numpy as np
import pytest
from controller.mpc.integrated_mpc import IntegratedConfig, IntegratedMPC


def config(**kwargs):
    return replace(IntegratedConfig(wheelbase=.3302, max_steer=.4189, max_steer_rate=3.2,
                                    max_iter=2000, solver_time_limit=.015, deadline=.02), **kwargs)


def straight(speed=2.):
    x=np.linspace(-1,30,400)
    return np.column_stack((x,np.zeros_like(x),np.full_like(x,speed)))


def test_joint_acceleration_and_steering():
    c=config();m=IntegratedMPC(c)
    r=m.solve([0,.1,.03,1.],straight(),0.)
    assert r.success,r.status
    assert 1 < r.speed <= 1+c.max_accel*c.dt+1e-5
    assert r.steering < 0
    assert r.controls.shape==(c.horizon,2)
    assert np.all(r.controls[:,0] >= -c.max_decel-1e-5)
    assert np.all(r.controls[:,0] <= c.max_accel+1e-5)
    assert np.max(abs(r.controls[:,1])) <= c.max_steer+1e-5
    rates=np.diff(np.r_[0.,r.controls[:,1]])/np.r_[c.dt,np.full(c.horizon-1,c.preview_dt)]
    assert np.max(abs(rates)) <= c.max_steer_rate+1e-3
    assert np.max(r.predicted[:,3]) <= 2.+1e-4


def test_brakes_before_lower_profile():
    c=config();m=IntegratedMPC(c);path=straight(3.)
    path[path[:,0]>.9,2]=.5
    r=m.solve([0,0,0,2.],path,0.)
    assert r.success,r.status
    assert 2-c.max_decel*c.dt-1e-4 <= r.speed < 2.


def test_curve_lateral_acceleration_bound():
    c=config();m=IntegratedMPC(c)
    theta=np.linspace(-.4,2.5,400)
    p=np.column_stack((2*np.sin(theta),2*(1-np.cos(theta)),np.full_like(theta,3.)))
    r=m.solve([0,0,0,1.],p,0.)
    assert r.success,r.status
    lat=r.predicted[:-1,3]**2*abs(np.tan(r.controls[:,1]))/c.wheelbase
    assert np.max(lat) <= c.max_lateral_accel+1e-3


@pytest.mark.parametrize('state,path,previous', [
    ([0,0,0,float('nan')],straight(),0.), ([0,0,0,-1.],straight(),0.),
    ([0,0,0,1.],straight(-1.),0.), ([0,0,0,1.],np.zeros((4,3)),0.),
    ([0,0,0,1.],straight(),float('nan')), ([0,5,0,1.],straight(),0.)])
def test_invalid_input_stops(state,path,previous):
    r=IntegratedMPC(config()).solve(state,path,previous)
    assert not r.success and r.speed==0
    assert np.isfinite(r.steering)


def test_impossible_profile_stops_instead_of_violating_deceleration():
    r=IntegratedMPC(config()).solve([0,0,0,3.],straight(.1),0.)
    assert not r.success and r.speed==0


def test_deadline_failure_stops():
    r=IntegratedMPC(config(solver_time_limit=1e-6,deadline=1e-6)).solve([0,0,0,1.],straight(),0.)
    assert not r.success and r.speed==0


def test_adapter_never_calls_pp_or_legacy_speed():
    from offline_support import make_controller, cycle, virtual_path
    c=make_controller(lateral_controller='MPC',lateral_mpc=IntegratedMPC(config()))
    for name in ('calc_speed_command','speed_adjust_heading','calc_steering_angle_for_future',
                 'calc_future_L1_point','AEB_for_weird_local_wpnt'):
        setattr(c,name,Mock(side_effect=AssertionError('legacy control invoked')))
    out=cycle(c,virtual_path('straight'))
    assert c.mpc_result.success,c.mpc_result.status
    assert out[0]==c.mpc_result.speed
    assert out[3]==c.mpc_result.steering
    assert cycle(c,virtual_path('straight'),state='TRAILING')[0]==0.


@pytest.mark.parametrize('field', ['max_accel', 'max_decel', 'max_speed', 'max_lateral_accel', 'preview_dt', 'q_speed', 'r_accel'])
@pytest.mark.parametrize('value', [0., -1., float('nan')])
def test_joint_configuration_rejects_invalid_limits(field, value):
    with pytest.raises(ValueError):
        config(**{field: value})


def test_zero_speed_profile_cannot_creep():
    result = IntegratedMPC(config()).solve([0., 0., 0., 0.], straight(0.), 0.)
    assert result.success, result.status
    assert result.speed <= 1e-4


def test_non_numeric_previous_steering_fails_closed():
    result = IntegratedMPC(config()).solve([0., 0., 0., 0.], straight(), 'bad')
    assert not result.success and result.speed == 0.
    assert np.isfinite(result.steering)


def test_exit_of_slow_sector_accelerates_again():
    # Same controller and actual speed input on each tick; profile is an upper bound.
    mpc = IntegratedMPC(config())
    result = mpc.solve([0., 0., 0., .5], straight(2.), 0.)
    assert result.success, result.status
    assert result.acceleration > 0. and result.speed > .5


def test_cached_solver_updates_speed_and_curved_path_constraints():
    c=config()
    mpc=IntegratedMPC(c)
    for speed_limit in [2., .5, 1.5, 0., 2.]:
        result=mpc.solve([0.,0.,0.,0.],straight(speed_limit),0.)
        assert result.success,result.status
        assert result.speed <= speed_limit+1e-4
        assert np.max(result.predicted[:,3]) <= speed_limit+1e-4
    theta=np.linspace(-.4,2.5,400)
    curve=np.column_stack((2*np.sin(theta),2*(1-np.cos(theta)),np.full_like(theta,2.)))
    for path in [curve,straight(),curve]:
        cached=mpc.solve([0.,0.,0.,1.],path,0.)
        cold=IntegratedMPC(c).solve([0.,0.,0.,1.],path,0.)
        assert cached.success and cold.success,(cached.status,cold.status)
        assert cached.speed==pytest.approx(cold.speed,abs=1e-4)
        assert cached.steering==pytest.approx(cold.steering,abs=1e-4)



def test_braking_preview_cannot_exceed_physical_limit():
    with pytest.raises(ValueError): config(braking_preview_ratio=1.1)


def test_conservative_preview_brakes_earlier():
    path=straight(3.);path[path[:,0]>2.5,2]=.5
    nominal=IntegratedMPC(config()).solve([0.,0.,0.,1.9],path,0.)
    conservative=IntegratedMPC(config(braking_preview_ratio=.3)).solve([0.,0.,0.,1.9],path,0.)
    assert nominal.success and conservative.success,(nominal.status,conservative.status)
    assert conservative.controls[:,0].sum()<nominal.controls[:,0].sum()



def test_preview_target_lag_does_not_relax_original_profile_cap():
    path=straight(3.);path[path[:,0]>2.5,2]=.5
    c=config(braking_preview_ratio=.3)
    result=IntegratedMPC(c).solve([0.,0.,0.,2.1],path,0.)
    assert result.success,result.status
    assert result.acceleration<0.
    assert result.speed<=3.
    # An actual profile overrun remains infeasible and still requests STOP.
    result=IntegratedMPC(c).solve([0.,0.,0.,2.1],straight(.5),0.)
    assert not result.success and result.speed==0.
