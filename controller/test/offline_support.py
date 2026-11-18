"""Synthetic paths and lightweight ROS message shims; never initialize ROS."""

import importlib.util
import inspect
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]


def virtual_path(name):
    if name == 'sharp_curve':
        theta = np.linspace(-0.5, 3.8, 1200)
        xy = np.column_stack((1.2 * np.sin(theta), 1.2 * (1 - np.cos(theta))))
    else:
        x = np.linspace(-2.0, 30.0, 1601)
        if name == 'straight':
            y = np.zeros_like(x)
        elif name == 's_curve':
            y = 0.6 * np.sin(0.65 * x)
        elif name == 'static_avoidance':
            y = 0.5 * (np.tanh((x - 3.0) / 0.9) - np.tanh((x - 7.0) / 0.9))
        else:
            raise ValueError(name)
        xy = np.column_stack((x, y))
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    tangent = np.gradient(xy, arc, axis=0)
    yaw = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
    curvature = np.gradient(yaw, arc)
    # Same nine-column layout as controller_manager.behavior_cb.
    return np.column_stack((xy, np.full(len(xy), 2.0), np.full(len(xy), 0.5),
                            arc, curvature, yaw, np.zeros(len(xy)), xy[:, 1]))


def install_marker_shims():
    # Only visualization value objects are stubbed. No ROS node, DDS or publisher.
    class Marker:
        SPHERE = 2

        def __init__(self):
            self.header = SimpleNamespace()
            self.scale = SimpleNamespace()
            self.color = SimpleNamespace()
            self.pose = SimpleNamespace(position=SimpleNamespace(), orientation=SimpleNamespace())

    class MarkerArray:
        def __init__(self):
            self.markers = []

    package = ModuleType('visualization_msgs')
    module = ModuleType('visualization_msgs.msg')
    module.Marker, module.MarkerArray = Marker, MarkerArray
    package.msg = module
    sys.modules['visualization_msgs'] = package
    sys.modules['visualization_msgs.msg'] = module


def load_controller(path=None):
    install_marker_shims()
    if path is None:
        path = ROOT / 'controller/controller/combined/src/Controller.py'
    spec = importlib.util.spec_from_file_location('offline_controller', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Controller


class StraightFrenet:
    """Fixed synthetic x-axis global line, only for legacy-speed isolation tests."""
    def get_frenet(self, x, y):
        return np.asarray(x), np.asarray(y)


def make_controller(cls=None, **overrides):
    cls = cls or load_controller()
    params = yaml.safe_load((ROOT / 'stack_master/config/controller.yaml').read_text())['controller_manager']['ros__parameters']
    # Explicit SYNTHETIC test geometry, not a physical vehicle calibration.
    params.update(loop_rate=50, wheelbase=0.32, converter=StraightFrenet(),
                  logger_info=lambda *_: None, logger_warn=lambda *_: None)
    params.update(overrides)
    names = inspect.signature(cls).parameters
    return cls(**{k: v for k, v in params.items() if k in names})


def cycle(controller, path, position=None, speed=2.0, state='GB_TRACK', opponent=None):
    if position is None:
        position = [0.0, 0.05, float(path[np.argmin(abs(path[:, 0])), 6])]
    return controller.main_loop(state, np.asarray([position], dtype=float), path,
                                speed, opponent, np.array([2.0, 0.05, speed, 0.0]),
                                np.zeros(10), 100.0)


def regression_cases():
    for name in ('straight', 'sharp_curve', 's_curve', 'static_avoidance'):
        for state in ('GB_TRACK', 'OVERTAKE', 'RECOVERY', 'START', 'TRAILING'):
            yield name, state


def regression_outputs(controller, name, state):
    path = virtual_path(name)
    outputs = []
    for tick in range(8):
        idx = int(np.argmin(abs(path[:, 0] - 0.03 * tick)))
        position = [path[idx, 0], path[idx, 1] + 0.08, path[idx, 6] + 0.02]
        output = cycle(controller, path, position=position, speed=0.4 + 0.2 * tick,
                       state=state, opponent=[4.0, 0.0, 0.0, True, True] if state == 'TRAILING' else None)
        outputs.append([np.asarray(item).tolist() for item in output])
    return outputs
