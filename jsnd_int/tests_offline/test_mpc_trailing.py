"""Step 21: MPC in TRAILING (PP trailing-controller speed cap) and stale-obstacle drop.
No ROS graph, no hardware. Run as test_mpc_integration.py."""
import numpy as np

from test_mpc_integration import CAR, DT, RACELINE, local_window, make_controller, pose_on_line

TRAIL = dict(trailing_gap=1.5, trailing_vel_gain=0.25, trailing_p_gain=0.5, trailing_i_gain=0.0,
             trailing_d_gain=0.25, blind_trailing_speed=1.5, static_min_standoff_m=0.5,
             static_resume_hysteresis_m=0.15, static_crawl_speed_mps=0.2, loop_rate=CAR['control_loop_rate_hz'])
TRACK = float(RACELINE[-1, 4] + (RACELINE[1, 4] - RACELINE[0, 4]))


def ctrl_trailing():
    c = make_controller()
    for k, v in TRAIL.items():
        setattr(c, k, v)
    return c


def run(c, state, opp, ticks, v=0.0, i=0):
    pose, w = pose_on_line(i), local_window(i)
    s_ego = RACELINE[i, 4]
    out = []
    for _ in range(ticks):
        r = c.main_loop(state, pose, w, v, opp, np.array([s_ego, 0.0, v, 0.0]), 0.0, TRACK)
        out.append((r[0], c.mpc_result.success, c.mpc_result.status, c.mpc_trailing_cap))
        v = r[0]                       # ideal wheel speed follow
    return out


def test_trailing_far_static_target_drives_like_gb_track():
    c = ctrl_trailing()
    opp = [RACELINE[0, 4] + 8.0, 0.0, 0.0, True, True]          # static 8 m ahead
    out = run(c, 'TRAILING', opp, 60)
    assert all(o[1] for o in out), out[-1]
    assert out[-1][0] > 0.3                                       # leaves standstill
    assert all(o[0] <= c.lateral_mpc.config.max_speed + 1e-9 for o in out)


def test_trailing_static_target_inside_standoff_stops():
    c = ctrl_trailing()
    opp = [RACELINE[0, 4] + 0.4, 0.0, 0.0, True, True]          # closer than 0.5 m standoff
    out = run(c, 'TRAILING', opp, 30)
    assert all(o[3] == 0.0 for o in out)
    assert all(o[0] == 0.0 for o in out)


def test_trailing_slow_dynamic_target_caps_speed():
    c = ctrl_trailing()
    opp = [RACELINE[0, 4] + 1.2, 0.0, 0.5, False, True]         # 0.5 m/s car 1.2 m ahead (< gap_should)
    out = run(c, 'TRAILING', opp, 60, v=1.0)
    assert all(o[0] <= max(o[3], 0.0) + 1e-6 for o in out if o[1])
    assert out[-1][0] < 1.0


def test_trailing_without_target_still_stops():
    c = ctrl_trailing()
    out = run(c, 'TRAILING', None, 5, v=0.3)
    assert all(o[0] == 0.0 and not o[1] and o[2] == 'trailing_without_target' for o in out)


def test_leaving_trailing_clears_static_latch():
    c = ctrl_trailing()
    run(c, 'TRAILING', [RACELINE[0, 4] + 0.4, 0.0, 0.0, True, True], 3)
    assert c.static_stop_latched
    run(c, 'GB_TRACK', None, 1)
    assert not c.static_stop_latched and c.mpc_trailing_cap is None


def test_stale_obstacles_dropped():
    from state_machine.state_machine_node import StateMachine
    sm = StateMachine.__new__(StateMachine)
    sm.obstacle_stale_timeout_s = 1.0
    sm.obstacles_in_interest = ['obs']
    sm._obstacles_stale_warned = False
    t = [100.0]
    sm.now_sec = lambda: t[0]
    sm.get_logger = lambda: type('L', (), {'warn': staticmethod(lambda *_: None)})()
    sm._obstacles_rx_sec = None
    assert sm._fresh_obstacles_in_interest() == ['obs']      # never received: unchanged
    sm._obstacles_rx_sec = 99.5
    assert sm._fresh_obstacles_in_interest() == ['obs']      # fresh
    t[0] = 101.0
    assert sm._fresh_obstacles_in_interest() == []           # 1.5 s old -> dropped
    sm.obstacle_stale_timeout_s = 0.0
    assert sm._fresh_obstacles_in_interest() == ['obs']      # disabled


def test_trailing_cap_drop_while_moving_brakes_instead_of_stop():
    """Step 35 (real car lowspeed 20:32 bag): GB_TRACK->TRAILING dropped the safety cap
    from 1.0 to the static-obstacle crawl in one tick at 0.97 m/s. The deficit > 0.30 m/s
    returned braking_envelope_infeasible (STOP, setpoint ramp to 0, VESC current brake)
    every lap. Now: tracked at the strongest feasible braking."""
    import yaml
    from controller.mpc import integrated_mpc as im
    from controller.mpc.configuration import MPC_DEFAULTS, make_config
    from test_stabilization import CFG
    v = dict(MPC_DEFAULTS)
    for f in ('mpc_car.yaml', 'mpc_profile_lowspeed.yaml'):
        p = yaml.safe_load((CFG / f).read_text())['controller_manager']['ros__parameters']
        v.update({k: x for k, x in p.items() if k in MPC_DEFAULTS})
    cfg = make_config(v, 0.324, True, 50)
    path = np.column_stack((np.arange(80)*.1, np.zeros(80), np.full(80, 1.0)))
    state = np.array([0.3, 0.0, 0.0, 0.97])
    free = im.IntegratedMPC(cfg).solve(state, path, 0.0)
    assert free.success, free.status
    r = im.IntegratedMPC(cfg).solve(state, path, 0.0, safety_speed_cap=0.2)
    assert r.success and r.status.startswith('braking_recovery'), r.status
    assert r.acceleration < -0.5 * cfg.max_decel
