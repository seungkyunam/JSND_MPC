"""Step 49 (2026-11-25): fewer needless STOPs in normal driving, safety checks unchanged.
No ROS graph, no hardware. Same environment as test_corner_stall.py."""
import numpy as np

from test_corner_stall import _expected_state, _solved_then_deadline, config, straight
from test_mpc_integration import local_window, make_controller, pose_on_line


def test_continuation_tolerates_speed_loop_lag_but_not_cross_track_or_overspeed():
    c = config()
    mpc, first = _solved_then_deadline(c)
    lag = _expected_state(first, 1, c) + np.array([-.2, 0., 0., -.6])   # behind + slower than plan
    r = mpc.solve(lag, straight(), first.steering)
    assert r.success and r.status == 'plan_continuation:deadline_exceeded', r.status
    mpc, first = _solved_then_deadline(c)
    side = _expected_state(first, 1, c) + np.array([0., .15, 0., 0.])  # 0.15 m cross-track
    assert mpc.solve(side, straight(), first.steering).status == 'deadline_exceeded'
    mpc, first = _solved_then_deadline(c)
    fast = _expected_state(first, 1, c) + np.array([0., 0., 0., .6])   # faster than plan
    assert mpc.solve(fast, straight(), first.steering).status == 'deadline_exceeded'


def test_post_braking_odom_undershoot_is_standstill_reverse_still_stops():
    c = make_controller()
    pose, w = pose_on_line(0), local_window(0)
    frenet = np.array([0., 0., 0., 0.])
    c.main_loop('GB_TRACK', pose, w, -0.1, None, frenet, 0.0, 14.0)
    assert 'invalid_or_reverse_speed' not in c.mpc_result.status, c.mpc_result.status
    c.main_loop('GB_TRACK', pose, w, -0.5, None, frenet, 0.0, 14.0)
    assert c.mpc_result.status == 'invalid_local_control_input:invalid_or_reverse_speed'


def test_trailing_without_target_ramps_down_then_stops():
    c = make_controller()
    pose, w = pose_on_line(0), local_window(0)
    frenet = np.array([0., 0., 1.0, 0.])
    for _ in range(5):
        out = c.main_loop('GB_TRACK', pose, w, 1.0, None, frenet, 0.0, 14.0)
    moving = float(out[0])
    assert moving > 0.5
    speeds = [float(c.main_loop('TRAILING', pose, w, 1.0, None, frenet, 0.0, 14.0)[0])
              for _ in range(c.mpc_fallback_max_ticks + 2)]
    assert c.mpc_result.status == 'trailing_without_target'
    assert 0.0 < speeds[0] < moving                         # soft ramp, no 0 step
    assert all(b <= a + 1e-9 for a, b in zip(speeds, speeds[1:]))
    assert speeds[-1] == 0.0                                # still a STOP after the window
