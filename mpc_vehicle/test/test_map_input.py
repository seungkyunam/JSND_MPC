"""Map-independent waypoint input: the profile holds no map, bad racelines stop."""
import json
import math
from pathlib import Path
import numpy as np
import pytest
from mpc_vehicle.core import combine_speeds, load_profile, validate_raceline


def circle(n=60, radius=2., speed=1.):
    t = np.linspace(0., 2*math.pi, n, endpoint=False)
    return np.column_stack((radius*np.cos(t), radius*np.sin(t))), np.full(n, speed)


@pytest.fixture
def profile(tmp_path):
    p = json.loads((Path(__file__).parents[1]/'config/vehicle.example.json').read_text())
    for key in ('vehicle_config', 'static_transforms'):
        f = tmp_path/(key+'.yaml'); f.write_text('{}'); p[key] = str(f)
    path = tmp_path/'vehicle.json'; path.write_text(json.dumps(p))
    return path, p


def test_example_profile_is_vehicle_only():
    p = json.loads((Path(__file__).parents[1]/'config/vehicle.example.json').read_text())
    assert not any('map' in key for key in p)


def test_profile_with_map_is_refused(profile):
    path, p = profile
    path.write_text(json.dumps(dict(p, map_dir='/some/track')))
    with pytest.raises(ValueError, match='must not contain map_dir'): load_profile(path, 'shadow')


def test_shadow_needs_no_map_and_refuses_one(profile, tmp_path):
    path, _ = profile
    assert 'map_dir' not in load_profile(path, 'shadow')
    with pytest.raises(ValueError, match='do not pass map_dir'): load_profile(path, 'shadow', tmp_path)


def test_hardware_modes_require_runtime_map(profile):
    path, _ = profile
    for mode in ('bench', 'drive'):
        with pytest.raises(ValueError, match='requires an explicit map_dir'): load_profile(path, mode)


def test_valid_closed_raceline_accepts_duplicate_endpoint():
    xy, v = circle()
    xy2, v2 = validate_raceline(np.vstack((xy, xy[:1])), np.r_[v, v[:1]])
    assert xy2.shape == xy.shape and np.allclose(xy2, xy) and np.allclose(v2, v)


@pytest.mark.parametrize('mutate,reason', [
    (lambda xy, v: (xy[:10], v[:10]), 'too_few'),
    (lambda xy, v: (xy, v[:-1]), 'malformed'),
    (lambda xy, v: ([], []), 'malformed'),
    (lambda xy, v: (np.where(np.arange(len(xy))[:, None] == 3, np.nan, xy), v), 'nonfinite'),
    (lambda xy, v: (xy, np.r_[-1., v[1:]]), 'nonfinite_or_negative'),
    (lambda xy, v: (xy[:40], v[:40]), 'not_closed'),
])
def test_invalid_raceline_is_rejected(mutate, reason):
    with pytest.raises(ValueError, match=reason): validate_raceline(*mutate(*circle()))


def test_scaled_profile_only_lowers_speed_on_the_same_line():
    xy, v = circle(speed=2.)
    lower = np.r_[np.full(30, 1.), np.full(30, 3.)]
    assert np.allclose(combine_speeds(xy, v, (xy, lower)), np.minimum(v, lower))
    assert combine_speeds(xy, v, None) is v
    other = circle(radius=3.)[0]
    assert np.allclose(combine_speeds(xy, v, (other, lower)), v)  # other map/line: ignored
