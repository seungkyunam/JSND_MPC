"""Focused Step 45 checks, independent of ROS/real-car maps."""
import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import compare_step39 as env
from controller.mpc.pose_prediction import PosePrediction, angle


def helper():
    clock = [0.0]
    return PosePrediction(lambda: clock[0]), clock


def test_gyro_rotation_no_phase_delay_and_wrap():
    p, clock = helper()
    for i in range(15):
        clock[0] = i * .02
        yaw = angle(3.1 + 2. * clock[0])
        p.observe(clock[0], 2.)
        assert abs(angle(p.condition_heading(np.array([0., 0., yaw])) - yaw)) < 1e-12
    assert p.limited_samples == 0


def test_heading_spike_is_bounded_and_duplicate_sample_not_integrated_twice():
    p, clock = helper()
    p.observe(0., 1.)
    assert p.condition_heading(np.zeros(3)) == 0
    clock[0] = .02
    p.observe(.02, 1.)
    pose = np.array([0., 0., .5])
    yaw = p.condition_heading(pose)
    assert yaw == pytest.approx(.02 + .034)
    assert p.condition_heading(pose) == yaw
    assert p.limited_samples == 1


@pytest.mark.parametrize('rate,stamp,xy', [(float('nan'), .02, 0.), (1., -.1, 0.), (1., .02, 1.)])
def test_invalid_stale_or_relocalized_input_reanchors(rate, stamp, xy):
    p, clock = helper()
    p.observe(0., 0.)
    p.condition_heading(np.zeros(3))
    clock[0] = .02
    p.observe(stamp, rate)
    assert p.condition_heading(np.array([xy, 0., .4])) == .4


def test_history_uses_pending_order_and_one_horizon_not_last_command_twice():
    p, clock = helper()
    for t, delta in [(0., .2), (.04, -.1), (.08, .3)]:
        clock[0] = t
        p.record(delta)
    clock[0] = .1
    result = p.predict(np.zeros(3), 2., .324, .1)
    expected_yaw = 2 / .324 * (.04*np.tan(.2) - .04*np.tan(.1) + .02*np.tan(.3))
    assert result[2] == pytest.approx(expected_yaw)
    assert not np.isclose(result[2], 2/.324*.1*np.tan(.3))
    assert np.linalg.norm(result[:2]) <= .2 + 1e-12
    assert np.array_equal(p.predict(np.zeros(3), 2., .324, 0.), np.zeros(3))


def test_startup_and_long_gap_do_not_reuse_stale_steering():
    p, clock = helper()
    assert p.predict(np.zeros(3), 2., .324, .1)[2] == 0
    p.record(.3)
    clock[0] = 1.
    assert p.predict(np.zeros(3), 2., .324, .1)[2] == 0
    clock[0] = -.1
    p.record(-.1)
    assert len(p.commands) == 1


def test_only_published_commands_enter_history():
    c, _ = env.lap_sim.build('mpc', '8p88', {'mpc_speed_source': 'integrated',
        'mpc_history_prediction': True}, 50)
    c.record_applied_steering(.1, published=False)
    assert not c.pose_prediction.commands
    c.record_applied_steering(.2)
    assert c.pose_prediction.commands[-1][1] == .2


def test_legacy_prediction_preserved_and_history_replaces_it():
    # Capture solver input without running OSQP; verifies the wiring, not just helper math.
    for enabled in (False, True):
        c, _ = env.lap_sim.build('mpc', '8p88', {'mpc_speed_source': 'integrated',
            'mpc_history_prediction': enabled, 'mpc_latency_comp_s': .1}, 50)
        c.pose_prediction.clock = lambda: .1
        c.curr_applied_steering = .2
        captured = []
        def solve(state, waypoints, previous):
            captured.append(state.copy())
            return c.lateral_mpc.reject('test_stop', previous)
        c.lateral_mpc.solve = solve
        c.integrated_loop('GB_TRACK', np.zeros((1, 3)), env.lap_sim.local_wpnts(0), 2.)
        expected = np.zeros(3)
        for _ in range(5):
            expected += .02*2*np.array([np.cos(expected[2]), np.sin(expected[2]),
                                        0. if enabled else np.tan(.2)/c.lateral_mpc.config.wheelbase])
        assert np.allclose(captured[0][:3], expected)
        assert c.mpc_result.success is False


def test_launch_defaults_and_manager_ekf_wiring():
    root = Path(__file__).resolve().parents[2] / 'jsnd_int/racing_stack'
    launch = ET.parse(root / 'stack_master/launch/race.launch.xml').getroot()
    args = {a.attrib['name']: a.attrib.get('default') for a in launch.findall('arg')}
    assert args['mpc_heading_robust'] == args['mpc_history_prediction'] == 'false'
    assert args['mpc_rate_hz'] == '50'
    tree = ast.parse((root / 'controller/controller/controller_manager.py').read_text())
    callback = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'odom_cb')
    text = ast.unparse(callback)
    assert 'data.twist.twist.angular.z' in text
    assert 'pose_prediction.observe(stamp, rate)' in text
    assert 'data.header.stamp' in text


@pytest.mark.parametrize('state,pose', [('FTGONLY', np.zeros((1, 3))),
                                      ('GB_TRACK', np.array([[0., 0., float('nan')]]))])
def test_enhanced_input_does_not_bypass_invalid_or_unsupported_stop(state, pose):
    c, _ = env.lap_sim.build('mpc', 'lowspeed', {'mpc_speed_source': 'integrated',
        'mpc_heading_robust': True, 'mpc_history_prediction': True}, 50)
    c.pose_prediction.clock = lambda: 0.
    c.pose_prediction.observe(0., 0.)
    result = c.integrated_loop(state, pose, env.lap_sim.local_wpnts(0), 0.)
    assert not c.mpc_result.success
    assert result[0] == 0.


@pytest.mark.parametrize('stale,host', [(True, 'autodrive'), (False, 'manual')])
def test_existing_manager_guard_still_overrides_enhanced_commands(stale, host):
    from types import SimpleNamespace as NS
    root = Path(__file__).resolve().parents[1] / 'racing_stack'
    tree = ast.parse((root / 'controller/controller/controller_manager.py').read_text())
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'mpc_output_guard')
    namespace = {'Float32': lambda **kw: NS(**kw), 'String': lambda **kw: NS(**kw)}
    exec(compile(ast.Module(body=[method], type_ignores=[]), '<manager guard>', 'exec'), namespace)
    class Time:
        def __init__(self, value): self.nanoseconds = int(value*1e9)
        def __sub__(self, other): return Time((self.nanoseconds-other.nanoseconds)/1e9)
    c, _ = env.lap_sim.build('mpc', 'lowspeed', {'mpc_speed_source': 'integrated',
        'mpc_heading_robust': True, 'mpc_history_prediction': True}, 50)
    c.mpc_speed_cmd = .5
    node = NS(name='test_controller', last_odom_time=Time(0 if stale else .99), mpc_odom_timeout_s=.2,
              mux_host=host, mux_host_time=Time(.99), mpc_mux_host_timeout_s=.25,
              controller=c, state='GB_TRACK',
              get_clock=lambda: NS(now=lambda: Time(1.)),
              get_logger=lambda: NS(error=lambda *a, **kw: None))
    assert namespace['mpc_output_guard'](node, .5, 1.) == (0., 0.)
    assert c.mpc_speed_cmd is None
