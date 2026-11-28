"""ROS-free tests of production code, including extracted StateMachine methods.
AST loading avoids importing/starting ROS; method bodies are used unmodified.
"""
import ast
import sys
import types
from pathlib import Path
from dataclasses import replace
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import compare_step39 as env
from controller.mpc.integrated_mpc import IntegratedMPC
ROOT=Path(__file__).resolve().parents[3]


def state_machine():
    p=ROOT/'jsnd_int/racing_stack/state_machine/state_machine/state_machine_node.py'
    tree=ast.parse(p.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='StateMachine')
    names={'_forward_horizon','_check_enemy_in_front','obstacle_perception_cb','_fresh_obstacles_in_interest'}
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names]
    module=ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(name='SM',bases=[],keywords=[],body=methods,decorator_list=[])],type_ignores=[]))
    ns={};exec(compile(module,str(p),'exec'),ns)
    sm=ns['SM'](); sm.track_length=13.9;sm.obstacle_rear_exclusion_m=2.;sm.gb_horizon_m=15.;sm.interest_horizon_m=20.
    sm.cur_s=7.;sm.timetrials_only=False;sm.now_sec=lambda:1.;sm.obstacle_stale_timeout_s=.5;sm._obstacles_stale_warned=False
    return sm


def test_passed_obstacle_no_longer_becomes_next_lap_target():
    sm=state_machine(); obs=types.SimpleNamespace(s_start=4.1)
    # Step 50 hypothesis is arithmetically reproducible, not proof of the real log's target.
    assert (obs.s_start-sm.cur_s)%sm.track_length < sm.track_length-2.
    sm.obstacle_perception_cb(types.SimpleNamespace(obstacles=[obs]))
    assert sm.obstacles_in_interest==[] and not sm._check_enemy_in_front()


@pytest.mark.parametrize('ego,obstacle',[(13.5,.2),(0.,2.),(7.,7.3)])
def test_genuine_forward_and_seam_obstacles_remain(ego,obstacle):
    sm=state_machine();sm.cur_s=ego;obs=types.SimpleNamespace(s_start=obstacle)
    sm.obstacle_perception_cb(types.SimpleNamespace(obstacles=[obs]))
    assert sm.obstacles_in_interest==[obs] and sm._check_enemy_in_front()


def test_transition_does_not_invent_trailing_for_passed_obstacle():
    sys.path.insert(0,str(ROOT/'state_machine'))
    from state_machine.state_transitions import GlobalTrackingTransition, OvertakingTransition
    from state_machine.states_types import StateType
    sm=state_machine();sm._check_close_to_raceline=lambda *a:True;sm._check_close_to_raceline_heading=lambda *a:True
    sm.cur_gb_wpnts=None;sm._check_free_frenet=lambda *a:False
    sm._check_overtaking_mode=lambda:False;sm._check_static_overtaking_mode=lambda:False
    sm._check_overtaking_mode_sustainability=lambda:False
    sm.overtaking_ttl_count=0;sm.overtaking_ttl_count_threshold=10
    obs=types.SimpleNamespace(s_start=4.1)
    sm.obstacle_perception_cb(types.SimpleNamespace(obstacles=[obs]))
    sm.cur_obstacles_in_interest=[obs]  # old modulo horizon
    assert GlobalTrackingTransition(sm)[0]==StateType.TRAILING
    sm.cur_obstacles_in_interest=sm.obstacles_in_interest
    assert OvertakingTransition(sm)==(StateType.GB_TRACK,StateType.GB_TRACK)


def build(weighted=False):
    return env.lap_sim.build('mpc','5p0',{'mpc_speed_source':'integrated','mpc_time_weighted_accel_cost':weighted},50)[0]


def straight():
    return np.column_stack((np.linspace(0,20,201),np.zeros(201),np.full(201,5.)))


def test_time_cost_improves_executed_accel_with_same_limits():
    a,b=build(),build(True)
    ra=a.lateral_mpc.solve(np.array([0.,0.,0.,1.]),straight(),0.)
    rb=b.lateral_mpc.solve(np.array([0.,0.,0.,1.]),straight(),0.)
    assert ra.success and rb.success
    assert rb.acceleration > ra.acceleration
    assert 0 < rb.acceleration <= b.lateral_mpc.config.max_accel
    for r in (ra,rb):
        assert np.max(r.predicted[:,3]) <= 5.001
        assert np.max(abs(r.controls[:,0]))<=5.001
    with pytest.raises(ValueError): replace(b.lateral_mpc.config,time_weighted_accel_cost=1)


def test_safety_heading_and_missing_target_still_stop():
    c=build(True);mpc=c.lateral_mpc
    assert not mpc.solve(np.array([0.,0.,1.,2.]),straight(),0.).success
    out=c.main_loop('TRAILING',np.array([[0.,0.,0.]]),straight(),0.,None,np.zeros(4),np.zeros(10),13.9)
    assert c.mpc_result.status=='trailing_without_target' and out[0]==0.


def test_telemetry_separates_jerk_plan_and_lead():
    c=build();r=c.lateral_mpc.solve(np.array([0.,0.,0.,1.]),straight(),0.)
    assert r.success
    c._mpc_speed_command(r,1.)
    d=c.mpc_speed_diagnostics
    assert d['optimal_accel_mps2']==r.acceleration
    assert d['applied_accel_mps2']<=.4+1e-9
    assert d['jerk_limited'] and d['command_clipped_mps']<=1.5
    c.mpc_accel_jerk_mps3=0.;c.mpc_speed_cmd=2.
    c._mpc_speed_command(r,1.)
    assert c.mpc_speed_diagnostics['lead_limited']


def test_fresh_acceleration_rebases_stale_braking_setpoint():
    c = build(); r = c.lateral_mpc.solve(np.array([0., 0., 0., 1.]), straight(), 0.)
    assert r.success and r.acceleration > 0.
    c.mpc_accel_jerk_mps3 = 0.
    c.mpc_speed_cmd = .5
    command = c._mpc_speed_command(r, 1.)
    assert command >= 1. and command == pytest.approx(min(
        1.+r.acceleration*c.lateral_mpc.config.dt, np.max(r.predicted[:, 3])))

@pytest.mark.parametrize('weighted',[False,True])
@pytest.mark.parametrize('curvature,speed',[(0.,3.),(.2,2.),(.6,1.)])
def test_candidate_retains_nonlinear_friction_and_steering_bounds(weighted,curvature,speed):
    c=build(weighted);m=c.lateral_mpc;cfg=m.config;s=np.linspace(0,8,161)
    path=np.column_stack((s,np.zeros_like(s))) if not curvature else np.column_stack((np.sin(curvature*s)/curvature,(1-np.cos(curvature*s))/curvature))
    previous=np.arctan(.324*curvature)
    r=m.solve([0.,0.,0.,speed],path,previous)
    assert r.success,r.status
    a,steer=r.controls.T;v=r.predicted[:,3]
    lat=np.maximum(v[:-1]**2,v[1:]**2)*abs(np.tan(steer))/.324
    assert np.max((a/5.)**2+(lat/4.5)**2)<=1.001
    assert np.max(abs(steer))<=.4+1e-8
    assert np.all(abs(np.diff(np.r_[previous,steer]))<=3.2*m.periods+1e-8)
