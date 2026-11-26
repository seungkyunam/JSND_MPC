"""Step 54: OVERTAKE is held briefly (<= 0.2 s) when the avoidance path is blocked only
by far static obstacles (bag 19:31, id 115 gap 3.6 m), and left at once for a close one."""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from state_machine import state_transitions as st  # noqa: E402
from state_machine.states_types import StateType  # noqa: E402


def _sm(gap):
    w = SimpleNamespace(is_init=True, free_dbg={"obs": [
        {"id": 115, "gap": gap, "blocked": True}, {"id": 107, "gap": 0.4, "blocked": False}]})
    return SimpleNamespace(
        static_overtaking_mode=True, cur_static_avoidance_wpnts=w, cur_avoidance_wpnts=w,
        rate_hz=50.0, overtaking_ttl_count=0, overtaking_ttl_count_threshold=10,
        _check_overtaking_mode_sustainability=lambda: False,
        _check_enemy_in_front=lambda: True,
        _check_close_to_raceline=lambda *_: False,
        _check_close_to_raceline_heading=lambda *_: False,
    )


def test_far_blocker_holds_then_releases(monkeypatch):
    monkeypatch.setattr(st, "GlobalTrackingTransition", lambda *_: (StateType.TRAILING, StateType.GB_TRACK))
    sm = _sm(3.6)
    out = [st.OvertakingTransition(sm)[0] for _ in range(12)]
    assert out[:10] == [StateType.OVERTAKE] * 10
    assert out[10] == StateType.TRAILING


def test_close_blocker_leaves_immediately(monkeypatch):
    monkeypatch.setattr(st, "GlobalTrackingTransition", lambda *_: (StateType.TRAILING, StateType.GB_TRACK))
    sm = _sm(1.0)
    assert st.OvertakingTransition(sm)[0] == StateType.TRAILING


def test_no_path_leaves_immediately(monkeypatch):
    monkeypatch.setattr(st, "GlobalTrackingTransition", lambda *_: (StateType.TRAILING, StateType.GB_TRACK))
    sm = _sm(3.6)
    sm.cur_static_avoidance_wpnts.is_init = False
    assert st.OvertakingTransition(sm)[0] == StateType.TRAILING
