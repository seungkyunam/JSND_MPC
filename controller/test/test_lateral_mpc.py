"""Run with PYTHONPATH=controller pytest controller/test; no ROS execution."""

from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from controller.mpc.bicycle import linearize, step
from controller.mpc.configuration import MPC_DEFAULTS, make_config
from controller.mpc.lateral_mpc import LateralMPC, MPCConfig, SafetyRequest
import controller.mpc.lateral_mpc as mpc_module
from offline_support import virtual_path


@pytest.fixture
def config():
    # Synthetic plant ONLY. These numbers never enter ROS configuration.
    return MPCConfig(wheelbase=0.32, max_steer=0.53, max_steer_rate=2.0)


@pytest.fixture
def deterministic_time(monkeypatch):
    # Model unit tests should not fail because the host scheduler preempts Python.
    # Real end-to-end timing is measured separately by benchmark_mpc.py.
    monkeypatch.setattr(mpc_module, 'perf_counter', lambda: 0.0)


def test_bicycle_jacobians():
    state = np.array([1.0, -0.4, 0.7])
    delta, v, dt, length = 0.2, 3.0, 0.02, 0.32
    a, b, c = linearize(state, delta, v, dt, length)
    eps = 1e-6
    numerical_a = np.column_stack([
        (step(state + eps * e, delta, v, dt, length) - step(state - eps * e, delta, v, dt, length)) / (2 * eps)
        for e in np.eye(3)])
    numerical_b = (step(state, delta + eps, v, dt, length) - step(state, delta - eps, v, dt, length)) / (2 * eps)
    np.testing.assert_allclose(a, numerical_a, atol=1e-8)
    np.testing.assert_allclose(b, numerical_b, atol=1e-8)
    np.testing.assert_allclose(a @ state + b * delta + c, step(state, delta, v, dt, length))


@pytest.mark.parametrize('name,steps,speed', [
    ('straight', 200, 2.0), ('sharp_curve', 100, 1.5),
    ('s_curve', 250, 2.0), ('static_avoidance', 350, 2.0),
])
def test_closed_loop_tracking(config, deterministic_time, name, steps, speed):
    path = virtual_path(name)
    start = int(np.argmin(abs(path[:, 0]))) if name != 'sharp_curve' else int(np.argmin(abs(path[:, 4] - 0.6)))
    state = np.array([path[start, 0], path[start, 1] + 0.08, path[start, 6]])
    controller = LateralMPC(config)
    previous = 0.0
    errors, trajectory = [], []
    for _ in range(steps):
        result = controller.solve(state, path, speed, previous)
        assert result.success, result.status
        assert abs(result.steering) <= config.max_steer
        assert abs(result.steering - previous) <= config.max_steer_rate * config.dt + 1e-12
        assert np.max(np.abs(np.diff(np.r_[previous, result.controls]))) <= config.max_steer_rate * config.dt + 2e-4
        state = step(state, result.steering, speed, config.dt, config.wheelbase)
        previous = result.steering
        errors.append(np.min(np.linalg.norm(path[:, :2] - state[:2], axis=1)))
        trajectory.append(state.copy())
    assert np.sqrt(np.mean(np.square(errors))) < 0.10
    assert max(errors) < 0.20
    if name == 'straight':
        assert abs(state[1]) < 0.015
    if name == 'static_avoidance':
        trajectory = np.asarray(trajectory)
        # Point-model check against a SYNTHETIC obstacle, not vehicle clearance certification.
        assert np.min(np.linalg.norm(trajectory[:, :2] - [5.0, 0.0], axis=1)) > 0.7
        assert state[0] > 10.0 and abs(state[1]) < 0.05


def test_local_path_not_global_yaw(config, deterministic_time):
    path = virtual_path('s_curve')
    controller = LateralMPC(config)
    a = controller.solve([0.0, 0.0, 0.35], path, 2.0, 0.0)
    path[:, 2:] = 999.0  # including global yaw/speed/curvature, unused by lateral model
    controller.reset()
    b = controller.solve([0.0, 0.0, 0.35], path, 2.0, 0.0)
    assert a.success and b.success
    assert a.steering == pytest.approx(b.steering, abs=1e-6)


@pytest.mark.parametrize('bad_path', [np.empty((0, 2)), np.zeros((20, 2)), [[0, 0], [1, 1]], [[0, 0], [1, float('nan')], [2, 0]]])
def test_invalid_path_requests_stop(config, bad_path):
    result = LateralMPC(config).solve([0, 0, 0], bad_path, 2.0, 0.1)
    assert result.safety_request == SafetyRequest.STOP
    assert result.steering == pytest.approx(0.1)


@pytest.mark.parametrize('state,speed,previous', [([0, 0, float('nan')], 2, 0), ([0, 0, 0], -1, 0),
    ([0, 0, 0], float('inf'), 0), ([0, 0, 0], 2, float('nan')), ([0, 0, 0], 2, 0.6),
    ([0, 0, 0], 2, None)])
def test_invalid_state_requests_stop(config, state, speed, previous):
    result = LateralMPC(config).solve(state, virtual_path('straight'), speed, previous)
    assert not result.success and np.isfinite(result.steering)


def test_no_path_endpoint_extrapolation(config):
    result = LateralMPC(config).solve([29.8, 0, 0], virtual_path('straight'), 2.0, 0.0)
    assert not result.success and 'insufficient_path_horizon' in result.status


def test_yaw_wrap_duplicates_zero_speed(config, deterministic_time):
    path = virtual_path('straight')
    path[:, :2] *= -1
    path = np.repeat(path, 2, axis=0)
    result = LateralMPC(config).solve([0, 0, -np.pi], path, 0.0, 0.1)
    assert result.success
    assert np.isfinite(result.predicted).all()


def test_geometry_and_limits_required(config):
    with pytest.raises(ValueError):
        make_config(MPC_DEFAULTS.copy(), 0.321, False, 50)
    values = {**MPC_DEFAULTS, 'mpc_geometry_confirmed': True}
    with pytest.raises(ValueError):
        make_config(values, 0.32, True, 50)  # zero actuator limits intentionally invalid
    with pytest.raises(ValueError):
        replace(config, max_steer_rate=float('nan'))


class FakeSolver:
    def __init__(self, status_val=7, solution=None):
        self.status_val = status_val
        self.solution = solution

    def update(self, **kwargs):
        pass

    def warm_start(self, **kwargs):
        pass

    def solve(self, **kwargs):
        return SimpleNamespace(x=self.solution, info=SimpleNamespace(
            run_time=0.001, iter=10, status_val=self.status_val, status='injected_failure'))


@pytest.mark.parametrize('status,solution', [(7, None), (3, None), (2, np.zeros(25)),
    (1, np.full(25, np.nan)), (1, np.ones(25))])
def test_solver_failure_and_bad_solution(config, status, solution):
    controller = LateralMPC(config)
    controller._solver = FakeSolver(status, solution)
    result = controller.solve([0, 0, 0], virtual_path('straight'), 2.0, 0.0)
    assert result.safety_request == SafetyRequest.STOP


def test_deadline_overrun_requests_stop(config, monkeypatch):
    controller = LateralMPC(config)
    controller._solver = FakeSolver(1, np.zeros(config.horizon))
    clock = iter([0.0, 0.03, 0.03])
    monkeypatch.setattr(mpc_module, 'perf_counter', lambda: next(clock))
    result = controller.solve([0, 0, 0], virtual_path('straight'), 2.0, 0.0)
    assert result.status == 'deadline_exceeded'


def test_recovery_after_failure(config, deterministic_time):
    controller = LateralMPC(config)
    assert not controller.solve([0, 0, 0], [], 2.0, 0.0).success
    assert controller.solve([0, 0, 0], virtual_path('straight'), 2.0, 0.0).success


def test_external_speed_sequence_is_not_modified(config, deterministic_time):
    forecast = np.linspace(1.0, 2.0, config.horizon)
    before = forecast.copy()
    result = LateralMPC(config).solve([0, 0, 0], virtual_path('straight'), forecast, 0.0)
    assert result.success
    np.testing.assert_array_equal(forecast, before)
    assert result.reference[-1, 0] == pytest.approx(np.sum(forecast) * config.dt)


def test_unreachable_curvature_and_large_errors_request_stop(config):
    tight = virtual_path('sharp_curve')
    tight[:, :2] *= 0.2
    result = LateralMPC(config).solve([0, 0, 0], tight, 0.5, 0.0)
    assert not result.success and 'reference_exceeds_steering_limit' in result.status
    for state in ([0, 2, 0], [0, 0, np.pi]):
        assert not LateralMPC(config).solve(state, virtual_path('straight'), 2.0, 0.0).success


def test_mirror_symmetry_and_large_map_origin(config, deterministic_time):
    path = virtual_path('s_curve')
    a = LateralMPC(config).solve([0, 0, 0.35], path, 2.0, 0.0)
    mirrored = path.copy()
    mirrored[:, 1] *= -1
    b = LateralMPC(config).solve([0, 0, -0.35], mirrored, 2.0, 0.0)
    shifted = path.copy()
    shifted[:, :2] += [1e6, -1e6]
    d = LateralMPC(config).solve([1e6, -1e6, 0.35], shifted, 2.0, 0.0)
    assert a.success and b.success and d.success
    assert a.steering == pytest.approx(-b.steering, abs=1e-6)
    assert a.steering == pytest.approx(d.steering, abs=1e-6)
