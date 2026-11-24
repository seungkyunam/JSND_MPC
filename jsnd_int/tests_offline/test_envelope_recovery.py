"""Step 37: stopped outside the heading envelope must not be a permanent STOP (5p0 bag
2026-11-24 20:56, t=81.2 s). No ROS graph, no hardware. Run as test_mpc_integration.py."""
import numpy as np

from test_mpc_integration import DT, RACELINE, local_window, make_controller, nearest, pose_on_line

YAW_ERR = 0.9   # > mpc_max_heading_error_rad 0.7


def _stopped_misaligned(i=40):
    pose = pose_on_line(i).copy()
    pose[0, 2] += YAW_ERR
    return pose


def _creep(c, pose, ticks, L=0.324):
    statuses, v = [], 0.0
    for _ in range(ticks):
        out = c.main_loop('GB_TRACK', pose, local_window(nearest(pose)), v, None, np.zeros(4), 0.0, 0.0)
        statuses.append(c.mpc_result.status)
        v, delta = out[0], out[3]
        pose = pose + DT * v * np.array([[np.cos(pose[0, 2]), np.sin(pose[0, 2]), np.tan(delta) / L]])
    return statuses, v


def test_disabled_reproduces_permanent_stop():
    c = make_controller()
    c.mpc_envelope_recovery_speed_mps = 0.0
    statuses, v = _creep(c, _stopped_misaligned(), 100)
    assert v == 0.0 and all(s.endswith('outside_heading_envelope') for s in statuses)


def test_stopped_outside_heading_envelope_creeps_back_and_mpc_resumes():
    c = make_controller()
    statuses, _ = _creep(c, _stopped_misaligned(), 150)
    assert statuses[0] == 'envelope_recovery:ValueError:outside_heading_envelope'
    first = next(k for k, s in enumerate(statuses) if not s.startswith('envelope_recovery'))
    assert statuses[first] in ('solved', 'solved_late', 'solved_inexact_verified', 'braking_recovery')
    assert first * DT < 2.0
