"""Related checks for the opt-in 8.88 m/s profile; no ROS or hardware."""
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
import yaml

from test_integrated_v2 import IntegratedMPC, car_config

CFG = Path(__file__).resolve().parents[1] / 'racing_stack/stack_master/config'


def test_profile_changes_only_operator_ceiling():
    def profile(name):
        return yaml.safe_load((CFG / f'mpc_profile_{name}.yaml').read_text())['controller_manager']['ros__parameters']
    assert profile('5p0') == {'mpc_max_speed_mps': 5.0}
    assert profile('8p88') == {'mpc_max_speed_mps': 8.88}


@pytest.mark.parametrize('curvature,speed', [(0., 8.), (.08, 5.)])
def test_high_speed_plan_retains_physical_limits(curvature, speed):
    c = replace(car_config(), max_speed=8.88, horizon=12, preview_dt=.05,
                dt=.02, solver_time_limit=.015, deadline=.02)
    s = np.linspace(0., 30., 301)
    if curvature:
        path = np.c_[np.sin(curvature*s)/curvature, (1.-np.cos(curvature*s))/curvature]
    else:
        path = np.c_[s, np.zeros_like(s)]
    prev = np.arctan(c.wheelbase*curvature)
    r = IntegratedMPC(c).solve([0., 0., 0., speed], path, prev)
    assert r.success, r.status
    acceleration, steering = r.controls.T
    v = r.predicted[:, 3]
    lat = np.maximum(v[:-1]**2, v[1:]**2)*abs(np.tan(steering))/c.wheelbase
    assert np.max((acceleration/np.where(acceleration>=0, c.max_accel, c.max_decel))**2
                  + (lat/c.max_lateral_accel)**2) <= 1.001
    assert np.max(abs(steering)) <= c.max_steer+1e-9
    assert np.all(abs(np.diff(np.r_[prev, steering])) <= c.max_steer_rate*np.r_[c.dt, np.full(11,c.preview_dt)]+1e-6)
    assert v.min() >= -1e-4 and v.max() <= 8.88+1e-4
    if not curvature:
        assert v.max() > 5.0  # the former operator ceiling is actually lifted
