from __future__ import annotations

from typing import TYPE_CHECKING, Tuple

from state_machine.states_types import StateType

if TYPE_CHECKING:
    from state_machine.state_machine_node import StateMachine

"""
Transitions should loosely follow the following template (basically a match-case)

if (logic sum of bools obtained by methods of state_machine):
    return StateType.<DESIRED STATE>
elif (e.g. state_machine.obstacles are near):
    return StateType.<ANOTHER DESIRED STATE>
...

NOTE: ideally put the most common cases on top of the match-case

NOTE 2: notice that, when implementing new states, if an attribute/condition in the
    StateMachine is not available, your IDE will tell you, but only if you have a smart
    enough IDE.

NOTE 3: transitions must not have side effects on the state machine!
    i.e. any attribute of the state machine should not be modified in the transitions.
    (The UNICORN implementation does mutate overtaking_ttl_count / cur_start_wpnts here;
     behaviour preserved as-is from the ROS1 stack.)
"""


def GlobalTrackingTransition(state_machine: "StateMachine", close_to_raceline=None) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.GB_TRACK`"""
    if close_to_raceline is None:
        close_to_raceline = state_machine._check_close_to_raceline()

    if len(state_machine.cur_obstacles_in_interest) == 0:
        return NonObstacleTransition(state_machine, close_to_raceline)
    else:
        return ObstacleTransition(state_machine, close_to_raceline)


def RecoveryTransition(state_machine: "StateMachine") -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.RECOVERY`"""
    recovery_sustainability = state_machine._check_sustainability(
        state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts
    )
    close_to_raceline = state_machine._check_close_to_raceline(0.05) * state_machine._check_close_to_raceline_heading(20)

    if recovery_sustainability and not close_to_raceline:
        return StateType.RECOVERY, StateType.RECOVERY

    return GlobalTrackingTransition(state_machine, close_to_raceline)


def TrailingTransition(state_machine: "StateMachine") -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.TRAILING`"""
    close_to_raceline = state_machine._check_close_to_raceline(0.05) * state_machine._check_close_to_raceline_heading(20)
    if len(state_machine.cur_obstacles_in_interest) == 0:
        return NonObstacleTransition(state_machine, close_to_raceline)
    else:
        if state_machine._check_ftg():
            return StateType.FTGONLY, StateType.FTGONLY
        return ObstacleTransition(state_machine, close_to_raceline)


def OvertakingTransition(state_machine: "StateMachine") -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.OVERTAKE`"""
    ot_sustainability = state_machine._check_overtaking_mode_sustainability()
    enemy_in_front = state_machine._check_enemy_in_front()
    if ot_sustainability and enemy_in_front:
        state_machine.overtaking_ttl_count = 0
        return StateType.OVERTAKE, StateType.OVERTAKE
    if ot_sustainability and state_machine.overtaking_ttl_count < state_machine.overtaking_ttl_count_threshold:
        state_machine.overtaking_ttl_count += 1
        state_machine.ot_flicker_hold_count = 0
        return StateType.OVERTAKE, StateType.OVERTAKE
    if ot_sustainability:
        state_machine.ot_flicker_hold_count = 0
    elif _ot_flicker_hold(state_machine):
        return StateType.OVERTAKE, StateType.OVERTAKE
    state_machine.overtaking_ttl_count = 0
    close_to_raceline = state_machine._check_close_to_raceline(0.05) * state_machine._check_close_to_raceline_heading(20)
    return GlobalTrackingTransition(state_machine, close_to_raceline)


def StartTransition(state_machine: "StateMachine") -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.START`"""
    start_free = state_machine._check_free_cartesian(state_machine.cur_start_wpnts)
    on_spline = state_machine._check_on_spline(state_machine.cur_start_wpnts)

    if start_free and on_spline:
        return StateType.START, StateType.START
    else:
        close_to_raceline = (
            state_machine._check_close_to_raceline(0.05) * state_machine._check_close_to_raceline_heading(20)
        )
        state_machine.cur_start_wpnts.is_init = False
        return GlobalTrackingTransition(state_machine, close_to_raceline)


def FTGOnlyTransition(state_machine: "StateMachine") -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.FTGONLY`"""
    close_to_raceline = state_machine._check_close_to_raceline(0.05) * state_machine._check_close_to_raceline_heading(20)
    if len(state_machine.cur_obstacles_in_interest) == 0:
        return NonObstacleTransition(state_machine, close_to_raceline)
    else:
        if close_to_raceline and state_machine._check_free_frenet(state_machine.cur_gb_wpnts):
            return StateType.GB_TRACK, StateType.GB_TRACK

        recovery_availability = state_machine._check_latest_wpnts(
            state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts
        )
        if recovery_availability and state_machine._check_free_frenet(state_machine.cur_recovery_wpnts):
            return StateType.RECOVERY, StateType.RECOVERY

        if state_machine._check_overtaking_mode() or state_machine._check_static_overtaking_mode():
            return StateType.OVERTAKE, StateType.OVERTAKE
        else:
            return StateType.FTGONLY, StateType.FTGONLY


##################################################################################################################
##################################################################################################################


def NonObstacleTransition(state_machine: "StateMachine", close_to_raceline) -> Tuple[StateType, StateType]:
    if close_to_raceline:
        return StateType.GB_TRACK, StateType.GB_TRACK

    if state_machine._check_latest_wpnts(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts):
        if state_machine._check_on_spline(state_machine.cur_recovery_wpnts):
            return StateType.RECOVERY, StateType.RECOVERY

    return StateType.LOSTLINE, StateType.GB_TRACK


def ObstacleTransition(state_machine: "StateMachine", close_to_raceline) -> Tuple[StateType, StateType]:
    recovery_availability = False
    if close_to_raceline and state_machine._check_free_frenet(state_machine.cur_gb_wpnts):
        return StateType.GB_TRACK, StateType.GB_TRACK

    # Overtake takeover always wins: compute it once (it mutates static_overtaking_mode)
    # and reuse below, so the recovery-hold branch can defer to it.
    can_overtake = state_machine._check_overtaking_mode() or state_machine._check_static_overtaking_mode()

    if not close_to_raceline:
        recovery_fresh = state_machine._check_latest_wpnts(
            state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts
        )
        # Hold the RECOVERY src through brief recovery/blended dropouts: once we are
        # already trailing on the recovery source, off the raceline and not about to
        # overtake, keep it as long as the sliced cache is still valid -- avoids the
        # 0.1 s latest_threshold flicker bouncing src to GB every time the planner
        # skips a frame. Entry (first pick) still requires a fresh path.
        recovery_hold = (
            state_machine.local_wpnts_src == StateType.RECOVERY
            and state_machine.cur_recovery_wpnts.is_init
            and not can_overtake
        )
        recovery_availability = recovery_fresh or recovery_hold
        if recovery_availability and state_machine._check_free_frenet(state_machine.cur_recovery_wpnts):
            return StateType.RECOVERY, StateType.RECOVERY

    if can_overtake:
        return StateType.OVERTAKE, StateType.OVERTAKE
    else:
        if close_to_raceline:
            return StateType.TRAILING, StateType.GB_TRACK
        elif recovery_availability:
            return StateType.TRAILING, StateType.RECOVERY
        elif state_machine._check_free_frenet(state_machine.cur_gb_wpnts):
            # Step 52 (vehicle bag 18:20): in the hairpin cur_d jumps to ~-0.9 every lap,
            # so close_to_raceline is False; with any non-blocking obstacle in interest this
            # branch returned TRAILING with no target -> MPC trailing_without_target STOP,
            # the car stood off the line (1.7 s, 454 ticks, all at s 3-5). If the raceline
            # is free, the offset is a returnable one and no obstacle lies in the lateral
            # corridor between the car and the raceline just ahead, track back to the line
            # exactly as NonObstacleTransition does (LOSTLINE, GB source).
            if _gb_return_clear(state_machine):
                return StateType.LOSTLINE, StateType.GB_TRACK
            return StateType.TRAILING, StateType.GB_TRACK
        else:
            return StateType.TRAILING, StateType.GB_TRACK


OT_FLICKER_HOLD_S = 0.2       # step 54: max OVERTAKE hold while the avoidance path is momentarily not free
OT_FLICKER_MIN_GAP_M = 1.5    # step 54: hold only if every blocking obstacle is at least this far ahead


def _ot_flicker_hold(state_machine: "StateMachine") -> bool:
    """Step 54 (bag 19:31): OVERTAKE<->TRAILING chattered every 0.02-0.3 s because the
    avoidance path was judged blocked for a few ticks by a static obstacle 2.5-3.6 m
    ahead (or the path dropped for a frame), each TRAILING tick capping speed. Keep
    OVERTAKE for at most OT_FLICKER_HOLD_S when a path still exists and every blocking
    obstacle is >= OT_FLICKER_MIN_GAP_M ahead. A close blocker (or no path) leaves
    OVERTAKE immediately, as before; a persistent far blocker leaves after the hold."""
    wpnts = (state_machine.cur_static_avoidance_wpnts if state_machine.static_overtaking_mode
             else state_machine.cur_avoidance_wpnts)
    count = getattr(state_machine, "ot_flicker_hold_count", 0)
    limit = max(1, int(OT_FLICKER_HOLD_S * state_machine.rate_hz))
    dbg = getattr(wpnts, "free_dbg", None) or {}
    blockers = [o for o in dbg.get("obs", []) if o.get("blocked")]
    ok = (
        bool(getattr(wpnts, "is_init", False))
        and count < limit
        and all(o.get("gap") is not None and o["gap"] >= OT_FLICKER_MIN_GAP_M for o in blockers)
    )
    state_machine.ot_flicker_hold_count = count + 1 if ok else 0
    return ok


GB_RETURN_MAX_D_M = 1.2      # step 52: largest |d| treated as a direct return to the raceline
GB_RETURN_GAP_M = 2.5         # step 52: obstacles this far ahead (or 0.5 m behind) must clear the corridor


def _gb_return_clear(state_machine: "StateMachine") -> bool:
    cur_d = float(state_machine.cur_d)
    if not abs(cur_d) <= GB_RETURN_MAX_D_M:
        return False
    margin = float(state_machine.gb_ego_width_m)
    lo, hi = min(cur_d, 0.0) - margin, max(cur_d, 0.0) + margin
    max_s = float(state_machine.max_s)
    if max_s <= 0.0:
        return False
    for obs in state_machine.cur_obstacles_in_interest:
        gap = (float(obs.s_center) - float(state_machine.cur_s)) % max_s
        if gap > GB_RETURN_GAP_M and gap < max_s - 0.5:
            continue
        half = 0.5 * float(getattr(obs, "size", 0.0) or 0.0)
        obs_d = float(obs.d_center)
        # Step 53b (bag 18:47, 12/14 laps): after the s~4 overtake the car returns from
        # d +0.3..0.5 while an off-line static object sits further out (s~5.7, d~0.85).
        # Returning moves away from it, yet the margin on the car's outer side flagged it
        # -> TRAILING / trailing_without_target slow-down every lap. An obstacle wholly
        # outside the car's current body, on the side away from the raceline, cannot be
        # hit by the return.
        if cur_d * obs_d > 0.0 and abs(obs_d) - half >= abs(cur_d) + 0.5 * margin:
            continue
        if obs_d + half >= lo and obs_d - half <= hi:
            return False
    return True
