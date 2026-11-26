"""Step 23: JSND speed pipeline + steering-only MPC (mpc_speed_source: jsnd) and the
short-track obstacle-horizon guard. No ROS graph, no hardware.

  source /opt/ros/jazzy/setup.bash; source ~/JSND_INT/install/setup.bash
  PYTHONPATH=~/JSND_INT/src/racing_stack/controller:~/JSND_INT/pydeps:$PYTHONPATH \
      python3 -m pytest -q ~/JSND_INT/tests_offline/test_jsnd_mpc.py
"""
import dataclasses
import importlib.util
import types
from pathlib import Path

import numpy as np
import pytest

import lap_sim as L


def ctrl_pair(profile='5p0', **over):
    pp, _ = L.build('pp', profile, {}, 50)
    mpc, _ = L.build('mpc', profile, over, 50)
    return pp, mpc


def test_config_selects_jsnd_speed_source_and_steering_mpc():
    _, mpc = ctrl_pair()
    from controller.mpc.steering_mpc import SteeringMPC
    assert mpc.mpc_speed_source == 'jsnd' and isinstance(mpc.lateral_mpc, SteeringMPC)
    c = mpc.lateral_mpc.config
    assert (c.max_steer, c.max_steer_rate, c.horizon) == (0.4, 3.2, 15)


def test_speed_command_is_the_legacy_jsnd_speed():
    """Same inputs -> MPC mode commands exactly the PP speed (no second speed optimiser)."""
    pp, mpc = ctrl_pair()
    acc = np.zeros(10)
    for i in range(0, len(L.GB), 7):
        for v, dd in ((0.0, 0.0), (1.5, 0.05), (3.0, -0.08)):
            p = L.FULL[i]
            pose = np.array([[p[0] - dd*np.sin(p[6]), p[1] + dd*np.cos(p[6]), p[6]]])
            wp = L.local_wpnts(p[4])
            fr = np.array([p[4], dd, v, 0.])
            # PP's future-position speed term uses the APPLIED steering of the last tick
            # (in MPC mode that is the MPC steering): align that state before comparing.
            pp.current_steer_command = pp.curr_steering_angle = mpc.current_steer_command
            a = pp.main_loop('GB_TRACK', pose, wp, v, None, fr, acc, L.TRACK_LEN)
            b = mpc.main_loop('GB_TRACK', pose, wp, v, None, fr, acc, L.TRACK_LEN)
            assert mpc.mpc_result.success, mpc.mpc_result.status
            assert b[0] == pytest.approx(a[0], abs=1e-9), (i, v, a[0], b[0])
            assert not mpc.mpc_lat_cap_active and not mpc.mpc_profile_cap_active


def test_plan_lateral_accel_check_is_noop_on_raceline_and_acts_on_infeasible_plan():
    _, mpc = ctrl_pair()
    assert mpc._plan_lateral_accel_check(L.GB) is L.GB and not mpc.mpc_lat_cap_active
    fast = L.GB.copy()
    fast[:, 2] *= 1.5                      # e.g. a sector scaling of 1.5 at the ay limit
    out = mpc._plan_lateral_accel_check(fast)
    assert mpc.mpc_lat_cap_active
    assert np.max(out[:, 2]**2*np.abs(out[:, 5])) <= 4.5 + 1e-6


def test_profile_ceiling_still_applies():
    _, mpc = ctrl_pair(profile='lowspeed')
    p = L.FULL[60]
    out = mpc.main_loop('GB_TRACK', np.array([[p[0], p[1], p[6]]]), L.local_wpnts(p[4]), 2.0, None,
                        np.array([p[4], 0, 2.0, 0]), np.zeros(10), L.TRACK_LEN)
    assert out[0] == pytest.approx(1.0) and mpc.mpc_profile_cap_active


def _drive(mpc, ticks, v=2.0, i=60, dd=0.0):
    p = L.FULL[i]
    pose = np.array([[p[0] - dd*np.sin(p[6]), p[1] + dd*np.cos(p[6]), p[6]]])
    out = []
    for _ in range(ticks):
        r = mpc.main_loop('GB_TRACK', pose, L.local_wpnts(p[4]), v, None, np.array([p[4], dd, v, 0]),
                          np.zeros(10), L.TRACK_LEN)
        out.append((r[0], mpc.mpc_result.status))
    return out


def test_solver_miss_uses_pp_steering_then_stops_without_alternating():
    _, mpc = ctrl_pair()
    c = mpc.lateral_mpc.config
    late = dataclasses.replace(c)
    object.__setattr__(late, 'deadline', 1e-9)   # every solve misses the deadline (test only)
    mpc.lateral_mpc.config = late
    out = _drive(mpc, 25)
    n = mpc.mpc_fallback_max_ticks
    assert all(s.startswith('pp_fallback:deadline_exceeded') and v > 0 for v, s in out[:n])
    assert all(s == 'deadline_exceeded' and v == 0.0 for v, s in out[n:]), out[n:]
    mpc.lateral_mpc.config = c                   # solver healthy again -> drives, budget refilled
    v, s = _drive(mpc, 1)[0]
    assert s == 'solved' and v > 0 and mpc.mpc_fallback_ticks == 0


def test_envelope_violation_stops_immediately():
    _, mpc = ctrl_pair()
    v, s = _drive(mpc, 1, dd=1.3)[0]
    assert s == 'outside_tracking_envelope' and v == 0.0


def test_lap_11_18_6_same_speed_plan_as_pp():
    pp, _ = L.simulate('pp', '5p0', laps=2)
    mpc, _ = L.simulate('mpc', '5p0', laps=2)
    print(pp, '\n', mpc)
    assert mpc['laps_done'] == 2 and mpc['stops'] == 0
    assert mpc['status'] == {'solved': mpc['status']['solved']}
    assert mpc['flying_lap_lat_err_max'] < 0.05 < pp['flying_lap_lat_err_max']
    assert mpc['flying_lap_avg_speed'] >= 0.97*pp['flying_lap_avg_speed']
    assert mpc['max_speed'] >= 0.97*pp['max_speed']
    assert mpc['binding_layer_pct'] == {'jsnd_plan': 100.0}


def test_lap_robust_to_servo_delay_noise_and_start_offset():
    res, _ = L.simulate('mpc', '5p0', laps=2, servo_delay_ticks=3, noise_pos=0.02, noise_v=0.05,
                        offset_d=0.3, offset_yaw=0.2)
    assert res['laps_done'] == 2 and res['stops'] == 0 and res['flying_lap_lat_err_max'] < 0.06, res


# ---------------- state machine: forward obstacle horizon on a short lap ----------------
def _sm_class():
    path = Path.home()/'JSND_INT/src/racing_stack/state_machine/state_machine/state_machine_node.py'
    spec = importlib.util.spec_from_file_location('sm_node_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.StateMachine


def test_forward_horizon_does_not_wrap_behind_the_car():
    SM = _sm_class()
    sm = types.SimpleNamespace(track_length=13.91, obstacle_rear_exclusion_m=2.0, gb_horizon_m=15.0,
                               cur_s=1.0)
    assert SM._forward_horizon(sm, 20.0) == pytest.approx(6.955)
    assert SM._forward_horizon(sm, 8.0) == pytest.approx(6.955)       # forward half-lap guard
    sm.track_length = 80.0
    assert SM._forward_horizon(sm, 20.0) == 20.0
    # enemy-in-front: obstacle 1.4 m behind (gap 12.51 on the 13.91 m lap) is not in front
    sm.track_length = 13.91
    sm._forward_horizon = types.MethodType(SM._forward_horizon, sm)
    sm.obstacles = [types.SimpleNamespace(s_start=(1.0 - 1.4) % 13.91)]
    assert SM._check_enemy_in_front(sm) is False
    sm.obstacles = [types.SimpleNamespace(s_start=4.0)]
    assert SM._check_enemy_in_front(sm) is True
