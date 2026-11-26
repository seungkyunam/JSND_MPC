import copy
import json
from pathlib import Path
import pytest
from mpc_vehicle.core import GatePolicy, load_profile

P = {'speed_limit_mps': .2, 'max_steer_rad': .15,
     'max_steer_rate_radps': .3, 'acceleration_mps2': .2}


def tick(g, now, **changes):
    args = dict(ready=True, manual=False, auto=False, emergency=False,
                command=(.2, .15), auto_ok=True)
    args.update(changes)
    return g.tick(now, **args)


def arm(g):
    for i in range(31): tick(g, i*.02)
    assert g.arm(.6, True, False, False, False)


def test_default_disarmed_and_neutral_rearm():
    g = GatePolicy(P)
    assert tick(g, 0., manual=True)[0] == 0
    assert not g.arm(.1, True, True, False, False)
    for i in range(1, 32): tick(g, i*.02)
    assert g.arm(.62, True, False, False, False)
    assert tick(g, .64)[0] == 0  # waiting for hold is allowed after explicit arm
    assert g.armed
    assert tick(g, .66, auto=True)[0] > 0
    assert tick(g, .68)[0] == 0
    assert not g.armed
    assert tick(g, .70, auto=True)[0] == 0


@pytest.mark.parametrize('failure', [{'ready': False}, {'emergency': True}, {'auto_ok': False}])
def test_fault_latches_and_cannot_automatically_recover(failure):
    g = GatePolicy(P); arm(g)
    assert tick(g, .62, auto=True)[0] > 0
    assert tick(g, .64, auto=True, **failure)[0] == 0
    assert not g.armed
    assert tick(g, .66, auto=True)[0] == 0


def test_loop_stall_stops_without_ramping():
    g = GatePolicy(P); arm(g)
    tick(g, .62, auto=True)
    assert tick(g, .9, auto=True)[0] == 0
    assert not g.armed


def test_manual_overrides_mpc_failure_but_not_estop():
    g = GatePolicy(P); arm(g)
    assert tick(g, .62, manual=True, auto=True, auto_ok=False)[0] > 0
    assert tick(g, .64, manual=True, emergency=True)[0] == 0
    assert not g.armed


def test_limits_and_no_reverse_and_instant_deceleration():
    g = GatePolicy(P); arm(g)
    speed, angle = tick(g, .62, manual=True, command=(10., 2.))
    assert speed == pytest.approx(.004)
    assert angle == pytest.approx(.006)
    for i in range(32, 100):
        speed, angle = tick(g, i*.02, manual=True, command=(10., 2.))
        assert 0 <= speed <= .2 and abs(angle) <= .15
    assert tick(g, 2., manual=True, command=(-1., 0.))[0] == 0


@pytest.mark.parametrize('bad', [(float('nan'), 0), (0, float('inf'))])
def test_nan_never_reaches_output(bad):
    g = GatePolicy(P); arm(g)
    assert tick(g, .62, manual=True, command=bad)[0] == 0
    assert not g.armed


def test_mux_transition_sample_is_not_forwarded():
    g = GatePolicy(P); arm(g)
    assert tick(g, .62, auto=True)[0] > 0
    assert tick(g, .64, manual=True, command=None)[0] == 0


def test_profile_default_blocks_hardware_before_any_ros_access(tmp_path):
    source = Path(__file__).parents[1]/'config/vehicle.example.json'
    p = json.loads(source.read_text())
    folder = tmp_path/'any_track'; folder.mkdir()
    for name in ('global_waypoints.json', 'speed_scaling.yaml', 'ot_sectors.yaml', 'map.pgm'):
        (folder/name).write_text('{}')
    (folder/'any_track.yaml').write_text('image: map.pgm\n')
    for key in ('vehicle_config', 'static_transforms'):
        path = tmp_path/(key+'.yaml'); path.write_text('{}'); p[key] = str(path)
    profile = tmp_path/'profile.json'; profile.write_text(json.dumps(p))
    assert load_profile(profile, 'shadow')['geometry_confirmed'] is False
    with pytest.raises(ValueError, match='geometry_confirmed'): load_profile(profile, 'bench', folder)
    p['speed_limit_mps'] = 1.; profile.write_text(json.dumps(p))
    with pytest.raises(ValueError, match='commissioning limit'): load_profile(profile, 'shadow')


def test_solver_speed_clamps_only_standstill_noise():
    from mpc_vehicle.core import solver_speed, STANDSTILL_NOISE_MPS
    # Observed EKF jitter on the stopped car: +-1.8e-4 m/s.
    for v in (-1.8e-4, -1.e-6, -0.0, -STANDSTILL_NOISE_MPS):
        assert solver_speed(v) == 0.
    for v in (0., 1.8e-4, .2):
        assert solver_speed(v) == v
    # Real reverse motion must still be refused, not turned into 0.
    for v in (-STANDSTILL_NOISE_MPS-1e-6, -.05, -.2):
        with pytest.raises(ValueError, match='reverse_motion'):
            solver_speed(v)
    for v in (float('nan'), float('inf'), float('-inf')):
        with pytest.raises(ValueError, match='nonfinite'):
            solver_speed(v)
