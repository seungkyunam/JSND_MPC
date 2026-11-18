"""Offline integration of real Controller and extracted manager method bodies.

ROS messages/publishers are value-object mocks. These tests do not claim ROS
launch, DDS, actuator or real-time validation.
"""

import ast
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from controller.mpc.lateral_mpc import MPCResult, SafetyRequest
from offline_support import ROOT, cycle, make_controller, regression_cases, regression_outputs, virtual_path


@pytest.mark.parametrize('name,state', list(regression_cases()))
def test_pp_golden_regression(name, state):
    golden = json.loads((Path(__file__).parent / 'legacy_pp_golden.json').read_text())
    actual = regression_outputs(make_controller(lateral_controller='PP'), name, state)
    expected = golden['cases'][name + ':' + state]
    for a_tick, e_tick in zip(actual, expected):
        for actual_item, expected_item in zip(a_tick, e_tick):
            np.testing.assert_allclose(actual_item, expected_item, atol=1e-12, rtol=1e-12)


class FakeMPC:
    def __init__(self, success=True):
        self.success = success
        self.calls = []

    def solve(self, state, waypoints, speeds, previous):
        self.calls.append((state.copy(), waypoints.copy(), speeds, previous))
        return MPCResult(0.02, SafetyRequest.NONE if self.success else SafetyRequest.STOP,
                         'solved' if self.success else 'injected_failure', 0.001,
                         reference=np.array([[0, 0, 0], [1, 0, 0]]))

    def reject(self, reason, previous):
        return MPCResult(previous, SafetyRequest.STOP, reason, 0.0)


@pytest.mark.parametrize('state', ['GB_TRACK', 'TRAILING', 'OVERTAKE', 'RECOVERY', 'START'])
def test_mpc_preserves_speed_and_bypasses_all_pp_shaping(state):
    path = virtual_path('static_avoidance')
    fake = FakeMPC()
    mpc = make_controller(lateral_controller='MPC', lateral_mpc=fake)
    pp = make_controller(lateral_controller='PP')
    for method in ('calc_steering_angle_for_future', 'calc_future_L1_point',
                   'compute_future_heading_correction', 'acc_scaling',
                   'speed_steer_scaling', 'steer_scaling_for_lat_err'):
        setattr(mpc, method, Mock(side_effect=AssertionError('PP shaping called by MPC')))
    args = dict(state=state, opponent=[4.0, 0.0, 0.0, True, True] if state == 'TRAILING' else None)
    a, b = cycle(mpc, path, **args), cycle(pp, path, **args)
    assert a[:3] == b[:3]
    assert a[3] == 0.02
    np.testing.assert_array_equal(fake.calls[0][1], path)
    assert fake.calls[0][2] == 2.0
    assert mpc.current_steer_command == mpc.curr_steering_angle == 0.02


def test_failure_stop_cannot_be_undone_by_legacy_aeb():
    controller = make_controller(lateral_controller='MPC', lateral_mpc=FakeMPC(False))
    controller.AEB_for_weird_local_wpnt = Mock(return_value=2.0)
    result = cycle(controller, virtual_path('straight'))
    assert result[0] == 0.0
    controller.AEB_for_weird_local_wpnt.assert_called_once()


@pytest.mark.parametrize('path', [np.empty((0, 9)), np.zeros((5, 9)), np.full((30, 9), np.nan)])
def test_invalid_local_input_stops_before_legacy_preprocessing(path):
    controller = make_controller(lateral_controller='MPC', lateral_mpc=FakeMPC())
    controller.calc_speed_command = Mock(side_effect=AssertionError('bad input reached speed logic'))
    assert cycle(controller, path, position=[0, 0, 0])[0] == 0.0
    assert not controller.mpc_result.success


def test_rear_axle_transform_and_applied_steer_feedback():
    fake = FakeMPC()
    controller = make_controller(lateral_controller='MPC', lateral_mpc=fake, pose_to_rear_axle_m=-0.1)
    controller.record_applied_steering(0.15)
    cycle(controller, virtual_path('straight'), position=[0, 0.05, 0])
    np.testing.assert_allclose(fake.calls[0][0], [-0.1, 0.05, 0])
    assert fake.calls[0][3] == 0.15


def test_pp_record_applied_does_not_change_legacy_state():
    controller = make_controller()
    controller.record_applied_steering(0.3)
    assert controller.current_steer_command == controller.curr_steering_angle == 0


def test_pp_runs_with_solver_imports_blocked():
    script = '''
import builtins
original = builtins.__import__
def guarded(name, *args, **kwargs):
    if name.split('.')[0] in ('osqp', 'scipy'):
        raise ImportError('optional solver deliberately unavailable')
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
from offline_support import make_controller, cycle, virtual_path
from controller.mpc.configuration import MPC_DEFAULTS
assert cycle(make_controller(), virtual_path('straight'))[0] > 0
'''
    env = {**os.environ, 'PYTHONPATH': str(ROOT / 'controller') + os.pathsep + str(Path(__file__).parent)}
    subprocess.run([sys.executable, '-c', script], env=env, check=True, capture_output=True, text=True)


def manager_method(name):
    # Execute the real method AST without importing rclpy/starting a ROS context.
    source = (ROOT / 'controller/controller/controller_manager.py').read_text()
    cls = next(node for node in ast.parse(source).body if isinstance(node, ast.ClassDef) and node.name == 'ControllerManager')
    method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = {'Point': SimpleNamespace, 'SetParametersResult': SimpleNamespace}
    exec(compile(ast.Module(body=[method], type_ignores=[]), '<manager-offline>', 'exec'), namespace)
    return namespace[name]


def test_ftg_priority_and_final_steering_feedback():
    manager = SimpleNamespace(
        _save_requested=False, mapping=False, controller=Mock(),
        waypoint_array_in_map=np.ones((30, 9)), position_in_map=np.ones((1, 3)),
        position_in_map_frenet=np.ones(4), measuring=False, state='FTGONLY',
        controller_cycle=Mock(side_effect=AssertionError('MPC invoked during FTG')),
        ftg_cycle=Mock(return_value=(1.0, 0.3)), create_ack_msg=Mock(), drive_pub=Mock())
    manager_method('control_loop')(manager)
    manager.ftg_cycle.assert_called_once()
    manager.create_ack_msg.assert_called_once_with(1.0, 0, 0, 0.3)
    manager.controller.record_applied_steering.assert_called_once_with(0.3)


def test_waypoint_watchdog_still_overrides_mpc():
    result = (2.0, 0, 0, 0.1, np.zeros(2), 1.0, 0, 0.0, np.zeros((1, 3)))
    manager = SimpleNamespace(name='offline', controller=SimpleNamespace(main_loop=Mock(return_value=result)),
        state='GB_TRACK', position_in_map=None, waypoint_array_in_map=None, speed_now=2,
        opponent=None, position_in_map_frenet=None, acc_now=None, track_length=10,
        viz_rate_hz=0, l1_pub=Mock(), waypoint_safety_counter=9, loop_rate=50,
        state_machine_rate=50, get_logger=Mock(return_value=Mock()))
    output = manager_method('controller_cycle')(manager)
    assert output[0] == output[3] == 0.0


def test_parameter_batch_rejected_before_any_mutation():
    manager = SimpleNamespace()
    params = [SimpleNamespace(name='KP', value=100), SimpleNamespace(name='lateral_controller', value='MPC')]
    result = manager_method('dyn_param_cb')(manager, params)
    assert not result.successful and not hasattr(manager, 'KP')
