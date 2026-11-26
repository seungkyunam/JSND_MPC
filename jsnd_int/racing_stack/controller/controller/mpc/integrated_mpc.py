"""ROS-independent joint acceleration/steering LTV MPC with bounded verified-plan continuation.

Inputs are rear-axle [x,y,yaw,v] and ordered local XY points. Extra columns
are ignored. Geometry, operator ceiling and an explicit obstacle safety cap
define a backward braking envelope; no racing speed profile is consumed.
The first stage is the control period; later stages use the preview period.
"""
from dataclasses import dataclass
from time import perf_counter
import numpy as np
from .lateral_mpc import MPCConfig, SafetyRequest


@dataclass(frozen=True)
class IntegratedConfig(MPCConfig):
    horizon: int = 20
    max_iter: int = 1000
    preview_dt: float = .1
    max_speed: float = 3.0
    # Configured conservative actuator limits, NOT optional tuning caps.
    # No measured replacement is available; never infer these from waypoint speeds.
    max_accel: float = 1.5
    max_decel: float = 2.5
    max_lateral_accel: float = 3.0
    q_speed: float = 8.0
    r_accel: float = .15
    braking_preview_ratio: float = .85
    # Step 47 (0 = previous behaviour). brake_lead_s: each curvature speed cap also
    # applies brake_lead_s*v earlier along the path (VESC speed-loop lag -> braking
    # starts before the corner, tightening only). steer_preview_s: steering
    # linearisation/feed-forward reference sampled steer_preview_s*v ahead.
    ZERO_ALLOWED = ('brake_lead_s', 'steer_preview_s', 'time_weighted_accel_cost')
    brake_lead_s: float = 0.
    steer_preview_s: float = 0.
    # Opt-in until command/pose delay is measured: equal stage penalties charge
    # the 20 ms executed action as much as a 50 ms future action (postponement).
    time_weighted_accel_cost: bool = False

    def __post_init__(self):
        super().__post_init__()
        if type(self.time_weighted_accel_cost) is not bool:
            raise ValueError('time_weighted_accel_cost must be boolean')
        if self.braking_preview_ratio > 1.:
            raise ValueError('braking preview cannot exceed maximum deceleration')
        for name in ('brake_lead_s', 'steer_preview_s'):
            value = getattr(self, name)
            if not np.isfinite(value) or not 0. <= value <= .5:
                raise ValueError(name + ' must be within [0, 0.5] s')


@dataclass(frozen=True)
class IntegratedResult:
    steering: float
    speed: float
    acceleration: float
    safety_request: SafetyRequest
    status: str
    elapsed_s: float
    solver_s: float = 0.
    iterations: int = 0
    reference: object = None
    predicted: object = None
    controls: object = None

    @property
    def success(self):
        return self.safety_request == SafetyRequest.NONE


def _gradient(f, x):
    """np.gradient(f, x, axis=0) for 1-D increasing x (2nd-order interior, 1st-order
    edges), without numpy's generic-argument overhead. Same formula as numpy."""
    out = np.empty_like(f)
    dx = np.diff(x)
    dx1, dx2 = dx[:-1], dx[1:]
    a = -dx2/(dx1*(dx1+dx2))
    b = (dx2-dx1)/(dx1*dx2)
    c = dx1/(dx2*(dx1+dx2))
    if f.ndim == 2:
        a, b, c = a[:, None], b[:, None], c[:, None]
    out[1:-1] = a*f[:-2] + b*f[1:-1] + c*f[2:]
    out[0] = (f[1]-f[0])/dx[0]
    out[-1] = (f[-1]-f[-2])/dx[-1]
    return out


class IntegratedMPC:
    WARM_START_DUAL = True   # warm start OSQP duals from the last accepted solve
    # Compute-budget misses only (CPU preemption / iteration budget). Path, envelope,
    # profile, infeasibility and input failures are never continued: they STOP.
    CONTINUATION_REASONS = ('deadline_exceeded', 'osqp:run time limit reached',
                            'osqp:maximum iterations reached', 'osqp:solved inaccurate')
    ACCEPT_VERIFIED_INEXACT = True
    ACCEPT_VERIFIED_LATE = True   # step 34: verified solve late vs deadline but within dt is served
    INEXACT_STATUSES = ('solved inaccurate', 'maximum iterations reached', 'run time limit reached')
    # Step 49: 5 -> 10 consecutive ticks (200 ms at 50 Hz; the plan spans 0.57 s). Real car 16:14:
    # 14 deadline_exceeded + 7 osqp STOPs in 42 s, clustered in hairpin braking where the CPU
    # misses come in bursts longer than 5 ticks. Every served tick is still re-verified.
    MAX_CONTINUATION_TICKS = 10
    # Step 48: speed 0.30 -> 0.50 m/s. The plan assumes v follows a instantly; the real VESC
    # speed loop lags the setpoint by up to mpc_cmd_max_lead (0.5, cmd-meas p90 0.48 on the car),
    # so every compute miss during accel/braking was refused -> STOP -> soft-stop decel ramp.
    # Cap, lateral, friction and tracking are still re-verified from the MEASURED state.
    CONTINUATION_TOL = (.10, .15, .50)  # measured vs planned: cross-track m, heading rad, speed (faster) m/s
    CONTINUATION_ALONG_TOL = .30        # step 49: along-track (behind/ahead of the plan) m
    BRAKING_RECOVERY_TOL = .30          # braking-envelope overshoot answered by max braking (unchanged)
    # Step 48: OSQP eps 1e-5 -> 1e-4 (+ adaptive rho every 10 iterations). The QP residual is
    # then up to ~1e-3, so the row check uses this tolerance; every safety property is still
    # enforced after it on the clipped controls and the NONLINEAR prediction (coupled/lateral
    # accel, tracking, heading, braking caps), whose tolerances are unchanged.
    QP_EPS = 1e-4
    QP_ROW_TOL = 2e-3
    PROFILE_REFINEMENTS = 2             # bounded geometry-envelope refinement
    # Speed caps use this fraction of the lateral limit Y; steering may still use all of Y.
    # The difference is the steering authority left to correct tracking errors in a corner.
    LATERAL_PLAN_FRACTION = .85

    def __init__(self, config):
        import osqp
        from scipy import sparse
        self.config = config
        self.osqp, self.sparse = osqp, sparse
        self.solver = None
        n = config.horizon
        size = 2*n
        self.p_rows = np.concatenate([np.arange(j+1) for j in range(size)])
        self.p_indptr = np.concatenate(([0], np.cumsum(np.arange(1, size+1))))
        self.p_cols = np.repeat(np.arange(size), np.arange(1, size+1))
        # Everything below depends only on the configuration and is built once.
        self.periods = np.concatenate(([config.dt], np.full(n-1, config.preview_dt)))
        self.preview_times = np.concatenate(([0.], np.cumsum(self.periods[:-1])))
        self.end_times = np.cumsum(self.periods)          # time at which v_{k+1} is reached
        self.node_times = np.concatenate(([0.], self.end_times))
        d = np.zeros((n, size))
        for k in range(n):
            d[k, 2*k+1] = 1.
            if k: d[k, 2*k-1] = -1.
        self.d = d
        penalties = np.tile([config.r_accel, config.r_steer], n)
        if config.time_weighted_accel_cost:
            penalties[0::2] *= self.periods / config.preview_dt
        self.h_base = np.diag(penalties) + config.r_rate*d.T@d
        # Speed rows of the condensed prediction: v_{k+1} = v_0 + sum_j<=k periods_j*a_j.
        # Row 3 of every linearised A_k is [0,0,0,1] and B_k[3] = [period_k, 0], so these
        # rows never change; appended friction rows have fixed sparsity but variable values.
        speed_rows = np.zeros((n, size))
        for k in range(n):
            speed_rows[k, 0:2*k+2:2] = self.periods[:k+1]
        self.matrix = sparse.csc_matrix(np.vstack((np.eye(size), d, speed_rows)))
        # Stacked [base; friction] matrix with fixed sparsity (step 26, speed): built once;
        # per tick only the friction entries of .data are rewritten (no scipy vstack).
        theta = (np.arange(16)+.5)*2*np.pi/16
        self._poly_cx, self._poly_sy = np.cos(theta), np.sin(theta)
        self._poly_ax = self._poly_cx/np.where(self._poly_cx >= 0, config.max_accel, config.max_decel)
        rows = np.repeat(np.arange(16*n), 2)
        cols = np.repeat(2*np.arange(n), 32)+np.tile([0, 1], 16*n)
        marker = sparse.csc_matrix((1e9+np.arange(32*n, dtype=float), (rows, cols)), shape=(16*n, 2*n))
        stacked = sparse.vstack((self.matrix, marker), format='csc')
        stacked.sort_indices()
        mask = stacked.data >= 1e9
        self._stack_indices, self._stack_indptr = stacked.indices.copy(), stacked.indptr.copy()
        self._stack_data = stacked.data.copy()
        self._stack_mask = mask
        self._stack_order = (stacked.data[mask]-1e9).astype(int)
        self.state_weights = np.tile([config.q_position, config.q_position, config.q_heading, 0.], (n, 1))
        self.state_weights[-1] *= config.terminal_scale
        self.box_accel_lo = np.full(n, -config.max_decel)
        self.box_accel_hi = np.full(n, config.max_accel)
        self.plan = None          # last accepted, fully verified plan (absolute coordinates)
        self.continuation = 0     # consecutive ticks served from that plan
        self.reset()

    def reset(self):
        """External reset invalidates commands as well as solver warm starts."""
        self.plan = None
        self.continuation = 0
        self._reset_warm_start()

    def _reset_warm_start(self):
        self.previous = None
        self.previous_dual = None

    def reject(self, reason, previous_steering=0., started=None):
        """STOP and drop the last plan (external callers: input/behaviour failures)."""
        self.plan = None
        self.continuation = 0
        return self._fail(reason, previous_steering, started)

    def _fail(self, reason, previous_steering=0., started=None):
        self._reset_warm_start()
        try:
            steer = float(previous_steering)
            if not np.isfinite(steer): steer = 0.
        except (TypeError, ValueError):
            steer = 0.
        return IntegratedResult(float(np.clip(steer, -self.config.max_steer, self.config.max_steer)),
                                0., 0., SafetyRequest.STOP, reason,
                                0. if started is None else perf_counter()-started)

    def _stacked_matrix(self, vmax, steer_bounds):
        """[base rows; 16-gon friction rows] with the same values as
        vstack((self.matrix, self._friction_rows(...))), assembled from the
        precomputed sparsity pattern."""
        c, n = self.config, self.config.horizon
        lateral = np.maximum(vmax**2, 1e-6)*np.tan(steer_bounds)/steer_bounds/(c.wheelbase*c.max_lateral_accel)
        values = np.empty((n, 16, 2))
        values[:, :, 0] = self._poly_ax
        values[:, :, 1] = lateral[:, None]*self._poly_sy
        data = self._stack_data.copy()
        data[self._stack_mask] = values.ravel()[self._stack_order]
        return self.sparse.csc_matrix((data, self._stack_indices, self._stack_indptr),
                                      shape=(self.matrix.shape[0]+16*n, 2*n))

    def _friction_rows(self, vmax, steer_bounds):
        """16-sided inner polygon of the asymmetric acceleration ellipse.

        At |delta| <= b, |tan(delta)| <= tan(b)/b * |delta|. Using
        the stage speed upper bound makes y = V^2*tan(b)/b*delta/(L*Y)
        a conservative lateral proxy. Axis vertices allow full A/D at zero
        steering; the 45-degree vertices retain the old braking reserve.
        """
        c, n = self.config, self.config.horizon
        theta = (np.arange(16)+.5)*2*np.pi/16
        cx, sy = np.cos(theta), np.sin(theta)
        ax = cx/np.where(cx >= 0, c.max_accel, c.max_decel)
        lateral = np.maximum(vmax**2, 1e-6)*np.tan(steer_bounds)/steer_bounds/(c.wheelbase*c.max_lateral_accel)
        values = np.stack((np.broadcast_to(ax, (n, 16)), lateral[:, None]*sy), axis=-1)
        rows = np.repeat(np.arange(16*n), 2)
        cols = np.repeat(2*np.arange(n), 32)+np.tile([0, 1], 16*n)
        return self.sparse.csc_matrix((values.ravel(), (rows, cols)), shape=(16*n, 2*n))

    def _geometry_caps(self, path, safety_speed_cap):
        # Same path and cap within one tick (reference, profile check): compute once.
        key = (float(safety_speed_cap), path.shape, path[:, :2].tobytes())
        cached = getattr(self, '_caps_cache', None)
        if cached is not None and cached[0] == key:
            return cached[1]
        result = self._geometry_caps_uncached(path, safety_speed_cap)
        self._caps_cache = (key, result)
        return result

    def _geometry_caps_uncached(self, path, safety_speed_cap):
        c = self.config
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1))]
        tangent = _gradient(path[:, :2], arc)
        yaw = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
        curvature = _gradient(yaw, arc)
        # Full friction-ellipse limits (step 26): no Y/sqrt(2) allocation, no 0.85
        # curvature factor. Lateral cap = sqrt(Y/|kappa|) (a_long = 0 at the limit).
        kappa = np.maximum(abs(curvature), 1e-6)
        lateral_cap = np.sqrt(self.LATERAL_PLAN_FRACTION*c.max_lateral_accel/kappa)
        if c.brake_lead_s > 0.:
            # Step 47: a corner cap also holds brake_lead_s*v before it (only lowers caps).
            reach = np.searchsorted(arc, arc+c.brake_lead_s*np.minimum(lateral_cap, c.max_speed), 'right')
            lateral_cap = np.array([lateral_cap[i:j].min() for i, j in enumerate(reach.tolist())])
        v2 = np.minimum(min(c.max_speed, safety_speed_cap), lateral_cap)**2
        # Unknown path beyond the supplied endpoint: reserve enough distance to stop.
        v2[-1] = 0.
        # Backward braking pass on the same g-g ellipse the raceline generator uses:
        # decel available at point i = D*sqrt(1-(a_y/Y)^2) with a_y = v^2*kappa_i.
        ds = np.diff(arc).tolist()
        k = kappa.tolist()
        vv = v2.tolist()
        dec, ylim = c.braking_preview_ratio*c.max_decel, c.max_lateral_accel
        for i in range(len(vv)-2, -1, -1):
            nxt = vv[i+1]
            if vv[i] > nxt:
                r = nxt*k[i]/ylim
                vv[i] = min(vv[i], nxt + 2.*dec*np.sqrt(max(0., 1.-r*r))*ds[i])
        return arc, np.sqrt(np.asarray(vv))

    def _reference(self, state, path, periods, safety_speed_cap=np.inf):
        c = self.config
        keep = np.concatenate(([True], np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1) > 1e-6))
        path = path[keep]
        if len(path) < 3:
            raise ValueError('degenerate_path')
        xy = path[:, :2] - state[:2]
        seg = np.diff(xy, axis=0)
        ds = np.linalg.norm(seg, axis=1)
        arc = np.concatenate(([0.], np.cumsum(ds)))
        fraction = np.clip(-np.sum(xy[:-1]*seg, axis=1)/ds**2, 0., 1.)
        projected = xy[:-1] + fraction[:, None]*seg
        idx = np.argmin(np.sum(projected**2, axis=1))
        if np.linalg.norm(projected[idx]) > c.max_tracking_error:
            raise ValueError('outside_tracking_envelope')
        start = arc[idx]+fraction[idx]*ds[idx]
        tangent = _gradient(xy, arc)
        yaw = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
        yaw += 2*np.pi*np.round((state[2]-np.interp(start, arc, yaw))/(2*np.pi))
        if abs(np.interp(start, arc, yaw)-state[2]) > c.max_heading_error:
            raise ValueError('outside_heading_envelope')
        curvature = _gradient(yaw, arc)
        _, caps = self._geometry_caps(path, safety_speed_cap)
        hard_caps = caps
        sample, velocity = float(start), float(state[3])
        samples, speeds = [sample], [velocity]
        for dt in periods.tolist():
            target = float(np.interp(sample, arc, caps))
            v = min(max(target, max(0., velocity-c.max_decel*dt)), velocity+c.max_accel*dt)
            sample += max(.05, .5*(velocity+v))*dt
            velocity = v
            samples.append(sample)
            speeds.append(v)
        samples = np.asarray(samples)
        if samples[-1] > arc[-1]:
            raise ValueError('insufficient_path_horizon')
        targets = np.interp(samples[1:], arc, caps)
        # The backward braking envelope is hard, including its terminal reserve.
        limits = np.interp(samples[1:], arc, hard_caps)
        # Include geometric minima between preview samples.
        # arc is strictly increasing, so {arc in [s_k, s_k+1]} is one index slice.
        first = np.searchsorted(arc, samples[:-1], 'left')
        last = np.searchsorted(arc, samples[1:], 'right')
        for k in np.flatnonzero(last > first).tolist():
            limits[k] = min(limits[k], hard_caps[first[k]:last[k]].min())
        reference = np.empty((len(samples), 4))
        reference[:, 0] = np.interp(samples, arc, xy[:, 0])
        reference[:, 1] = np.interp(samples, arc, xy[:, 1])
        reference[:, 2] = np.interp(samples, arc, yaw)
        # Linearise about the dynamically reachable speed sequence (step 26), not
        # about the caps: with the caps far above the current speed the LTV model
        # predicted too much yaw per step and the closed loop oscillated.
        reference[:, 3] = speeds
        steer_at = samples[:-1]+c.steer_preview_s*np.asarray(speeds[:-1])
        steering = np.arctan(c.wheelbase*np.interp(steer_at, arc, curvature))
        return reference, steering, limits

    @staticmethod
    def _same_path(old, new):
        """True if `new` is `old` itself or the same path with the local window slid
        forward/back (identical overlapping points). A slid window is not a new path;
        any changed point (avoidance, replanning) still counts as a new path."""
        if old.shape == new.shape and np.array_equal(old, new):
            return True
        for a, b in ((old, new), (new, old)):
            j = np.flatnonzero((a == b[0]).all(axis=1))
            if len(j):
                m = min(len(a)-j[0], len(b))
                if m >= 3 and np.array_equal(a[j[0]:j[0]+m], b[:m]):
                    return True
        return False

    def solve(self, state, waypoints, previous_steering, safety_speed_cap=np.inf):
        started = perf_counter()
        # A changed avoidance path or obstacle cap invalidates continuation.
        try:
            context = (np.asarray(waypoints, float)[:, :2].copy(), float(safety_speed_cap))
        except (ValueError, TypeError, IndexError):
            return self.reject('invalid_state_path_or_steering', previous_steering, started)
        if getattr(self, '_context', None) is not None:
            old_path, old_cap = self._context
            if old_cap != safety_speed_cap or not self._same_path(old_path, context[0]):
                self.plan = None
        self._context = context
        result = self._solve(state, waypoints, previous_steering, started, safety_speed_cap)
        if result.success:
            self.continuation = 0
            return result
        if result.status in self.CONTINUATION_REASONS:
            continued = self._continue_plan(state, previous_steering, result.status, started)
            if continued is not None:
                return continued
        self.plan = None
        self.continuation = 0
        return result

    def _continue_plan(self, state, previous_steering, reason, started):
        """Serve the next stage of the last verified plan for a compute-budget miss.

        Bounded (MAX_CONTINUATION_TICKS), and only while the measured state still
        matches what that plan predicted for this instant. The command is re-limited
        against the current steering (rate), the plan's steering bound and verified
        stage speed cap, and the lateral-acceleration limit; any miss -> None (STOP).
        Wall-clock expiry also prevents reusing a plan after a scheduling gap.
        The status stays visible as 'plan_continuation:<reason>'.
        """
        c, plan = self.config, self.plan
        if plan is None or self.continuation >= self.MAX_CONTINUATION_TICKS:
            return None
        try:
            state = np.asarray(state, float)
            prev = float(previous_steering)
            if state.shape != (4,) or not np.isfinite(state).all() or state[3] < 0 or not np.isfinite(prev) or abs(prev) > c.max_steer:
                return None
            age = perf_counter()-plan['started']
            if age > (self.MAX_CONTINUATION_TICKS+1)*c.dt:
                return None
            t = max((self.continuation+1)*c.dt, age)
            stage = int(np.searchsorted(self.node_times, t, 'right'))-1
            # The served tick may cross into the next stage: its control is still re-limited
            # and rechecked below (state match, cap, lateral accel, tracking).
            if stage < 1 or stage >= c.horizon:
                return None
            w = (t-self.node_times[stage])/self.periods[stage]
            p = plan['predicted']
            expect = (1-w)*p[stage]+w*p[stage+1]
            pos_tol, yaw_tol, v_tol = self.CONTINUATION_TOL
            dyaw = (state[2]-expect[2]+np.pi) % (2*np.pi)-np.pi
            # Step 49: split the position check. Along the planned heading the car is mostly
            # BEHIND the plan (VESC speed loop lags the planned acceleration), which is not a
            # tracking fault: along-track tolerance CONTINUATION_ALONG_TOL, cross-track stays
            # pos_tol. Speed: only FASTER than planned is rejected (slower is covered by the
            # cap/friction re-checks below). Offline: 9 of 14 refused continuations failed this state match.
            ex, ey = state[0]-expect[0], state[1]-expect[1]
            along = np.cos(expect[2])*ex + np.sin(expect[2])*ey
            cross = -np.sin(expect[2])*ex + np.cos(expect[2])*ey
            if (abs(cross) > pos_tol or abs(along) > self.CONTINUATION_ALONG_TOL or abs(dyaw) > yaw_tol
                    or state[3]-expect[3] > v_tol):
                return None
            accel, steer = (float(x) for x in plan['controls'][stage])
            bound, step = float(plan['steer_bounds'][stage]), c.max_steer_rate*c.dt
            lo_s, hi_s = max(-bound, prev-step), min(bound, prev+step)
            if lo_s > hi_s+1e-9:
                return None
            steer = min(max(steer, lo_s), hi_s)
            v0 = float(state[3])
            accel = min(accel, c.max_accel, (float(plan['caps'][stage])-v0)/c.dt)
            if accel < -c.max_decel-1e-4:
                return None
            accel = max(accel, -c.max_decel, -v0/c.dt)
            v1 = max(0., v0+accel*c.dt)
            lat = max(v0, v1)**2*abs(np.tan(steer))/c.wheelbase
            # Recheck actual projected location, including state mismatch tolerated above.
            path, safety_cap = self._context
            path = path[np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1) > 1e-6]]
            arc, geometric_caps = self._geometry_caps(path, safety_cap)
            end_xy = state[:2] + c.dt*v0*np.array([np.cos(state[2]), np.sin(state[2])])
            segments = np.diff(path, axis=0)
            fraction = np.clip(np.sum((end_xy-path[:-1])*segments, axis=1)
                               /np.sum(segments**2, axis=1), 0., 1.)
            projection = path[:-1]+fraction[:, None]*segments
            idx = np.argmin(np.linalg.norm(projection-end_xy, axis=1))
            actual_cap = np.interp(arc[idx]+fraction[idx]*(arc[idx+1]-arc[idx]), arc, geometric_caps)
            if np.linalg.norm(projection[idx]-end_xy) > c.max_tracking_error:
                return None
            if v1 > actual_cap+1e-4:
                # Step 49: same rule as braking_recovery in _solve. The measured speed lags the
                # planned braking (VESC speed loop); within BRAKING_RECOVERY_TOL brake at the
                # strongest deceleration the friction ellipse leaves with the planned steering
                # (re-checked below) instead of STOP.
                if v1-actual_cap > self.BRAKING_RECOVERY_TOL:
                    return None
                ratio = v0*v0*abs(np.tan(steer))/c.wheelbase/c.max_lateral_accel
                if ratio > 1.:
                    return None
                accel = max(-c.max_decel*np.sqrt(1.-ratio*ratio)*(1-1e-6), -v0/c.dt)
                v1 = max(0., v0+accel*c.dt)
            longitudinal = c.max_accel if accel >= 0 else c.max_decel
            if (lat > c.max_lateral_accel+1e-3
                    or (accel/longitudinal)**2 + (lat/c.max_lateral_accel)**2 > 1.+1e-3):
                return None
        except (ValueError, TypeError, IndexError, KeyError):
            return None
        self.continuation += 1
        return IntegratedResult(steer, v1, accel, SafetyRequest.NONE, 'plan_continuation:'+reason,
                                perf_counter()-started, 0., 0, plan['reference'],
                                plan['predicted'][stage:], plan['controls'][stage:])

    def _solve(self, state, waypoints, previous_steering, started, safety_speed_cap=np.inf):
        c, n = self.config, self.config.horizon
        try:
            state, path = np.asarray(state, float), np.asarray(waypoints, float)
            if (state.shape != (4,) or not np.isfinite(state).all() or state[3] < 0
                    or path.ndim != 2 or path.shape[1] < 2 or len(path) < 3
                    or not np.isfinite(path[:, :2]).all() or np.isnan(safety_speed_cap) or safety_speed_cap < 0
                    or not np.isfinite(previous_steering) or abs(previous_steering) > c.max_steer+1e-6):
                raise ValueError('invalid_state_path_or_steering')
            path = path[np.r_[True, np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1) > 1e-6]]
            periods = self.periods
            ref, delta_ref, caps = self._reference(state, path, periods, safety_speed_cap)
            # Steering-rate-consistent speed caps (tightening only). |delta_k| cannot
            # fall below |delta_prev| - rate*t_{k+1}, and the lateral check couples
            # delta_k with v_k and v_{k+1}. A cap above sqrt(a_lat*L/tan|delta|_min)
            # made the coupled steering box empty while the wheels were still
            # unwinding out of a corner (OSQP primal infeasible -> STOP).
            min_abs = abs(float(previous_steering))-c.max_steer_rate*self.end_times
            steer_speed = np.full(n+1, np.inf)
            turning = min_abs > 1e-6
            steer_speed[:n][turning] = (np.sqrt(c.max_lateral_accel*c.wheelbase
                                                / np.tan(np.minimum(min_abs[turning], c.max_steer)))*(1-1e-6))
            caps = np.minimum(caps, np.minimum(steer_speed[:n], steer_speed[1:]))
            # Stage-0 deceleration the friction polygon allows with the steering the
            # rate limit cannot yet remove (full D only at zero steering); later
            # stages can reach zero steering. Step 26: the pre-check used pure D, so a
            # braking need between the coupled and the pure limit reached OSQP as
            # 'primal infeasible' (STOP) in corners.
            v0 = float(state[3])
            bound0 = min(c.max_steer, np.arctan(c.max_lateral_accel*c.wheelbase/max(v0*v0, 1e-6)))
            steer0 = min(bound0, max(0., abs(float(previous_steering))-c.max_steer_rate*c.dt))
            eta0 = max(v0*v0, 1e-6)*np.tan(bound0)/bound0*steer0/(c.wheelbase*c.max_lateral_accel)
            theta = (np.arange(16)+.5)*2*np.pi/16
            back = np.cos(theta) < 0
            dec0 = float(np.min(-c.max_decel*(np.cos(np.pi/16)-np.sin(theta[back])*eta0)/np.cos(theta[back])))
            dec0 = min(c.max_decel, max(0., dec0))
            reachable = np.maximum(0., v0-dec0*c.dt-c.max_decel*(self.end_times-c.dt))
            # A small overshoot of the braking envelope (VESC speed-loop lag behind the
            # planned deceleration) is answered with the strongest feasible braking
            # while still tracking the path, instead of STOP (= recentred steering,
            # which leaves the corner). Bounded by the existing speed tolerance of
            # plan continuation; a larger overshoot still STOPs. A cap is never raised
            # above what that braking from the measured speed reaches.
            deficit = float(np.max(reachable-caps))
            braking_recovery = deficit > 1e-4
            if braking_recovery:
                # Step 35 (real car lowspeed 20:32 bag): a GB_TRACK->TRAILING switch drops
                # the safety cap in one tick (static obstacle at s~4.2 seen from s~1.3, every
                # lap). That deficit is not a tracking fault: brake at the strongest feasible
                # rate and keep tracking. Before, it STOPped (5 ticks) and the soft-STOP ramp
                # drove the setpoint to 0 -> VESC current braking 0.97->0.78 m/s (stutter).
                # A deficit the geometry/endpoint alone causes still STOPs as before.
                if deficit > self.BRAKING_RECOVERY_TOL and (
                        not np.isfinite(safety_speed_cap)
                        or float(np.max(reachable-self._reference(state, path, periods)[2]))
                        > self.BRAKING_RECOVERY_TOL):
                    return self._fail('braking_envelope_infeasible', previous_steering, started)
                caps = np.maximum(caps, reachable+1e-4)
            m = np.zeros((4, 2*n))
            b = np.array([0., 0., state[2], state[3]])
            h = self.h_base.copy()
            d = self.d
            prev = np.zeros(n)
            prev[0] = previous_steering
            g = -c.r_rate*(d.T@prev)
            # Batch linearization and quadratic cost assembly. The small state
            # recursion is sequential; forming N separate Hessians is unnecessary.
            v, yaw = ref[:-1, 3], ref[:-1, 2]
            delta = np.clip(delta_ref, -c.max_steer, c.max_steer)
            aa = np.tile(np.eye(4), (n, 1, 1))
            aa[:, 0, 2] = -v*np.sin(yaw)*periods
            aa[:, 0, 3] = np.cos(yaw)*periods
            aa[:, 1, 2] = v*np.cos(yaw)*periods
            aa[:, 1, 3] = np.sin(yaw)*periods
            aa[:, 2, 3] = np.tan(delta)*periods/c.wheelbase
            bb = np.zeros((n, 4, 2))
            bb[:, 3, 0] = periods
            bb[:, 2, 1] = v*periods/(c.wheelbase*np.cos(delta)**2)
            ff = ref[:-1]+periods[:, None]*np.column_stack((v*np.cos(yaw), v*np.sin(yaw),
                        v*np.tan(delta)/c.wheelbase, np.zeros(n)))
            affine = ff-np.einsum('nij,nj->ni', aa, ref[:-1])-bb[:, :, 1]*delta[:, None]
            all_m = np.empty((n, 4, 2*n))
            all_b = np.empty((n, 4))
            for k in range(n):
                m = aa[k]@m
                m[:, 2*k:2*k+2] += bb[k]
                b = aa[k]@b+affine[k]
                all_m[k], all_b[k] = m, b
            weights = self.state_weights
            stacked = all_m.reshape(4*n, 2*n)
            h += stacked.T@(weights.reshape(-1, 1)*stacked)
            g += stacked.T@(weights*(all_b-ref[1:])).ravel()
            # -q_progress * sum tangent_k . (p_{k+1}-p_k), affine in U.
            dm = all_m[:, :2] - np.concatenate((np.zeros((1, 2, 2*n)), all_m[:-1, :2]))
            tangents = np.column_stack((np.cos(yaw), np.sin(yaw)))
            g -= .5*c.q_speed*np.einsum('ki,kij->j', tangents, dm)
            g[1::2] -= c.r_steer*delta
            lower = -all_b[:, 3]
            # Base speed/rate rows are constant; friction rows depend on caps.
            rate = c.max_steer_rate*periods
            p_values = 2*h[self.p_rows, self.p_cols]
            # Profile check geometry (path only; independent of the solution).
            segments = np.diff(path[:, :2], axis=0)
            lengths2 = np.sum(segments**2, axis=1)
            valid = lengths2 > 1e-12
            segments, lengths2 = segments[valid], lengths2[valid]
            origins = path[:-1, :2][valid]
            segment_yaw = np.arctan2(segments[:, 1], segments[:, 0])
            geometry_arc, geometry_caps = self._geometry_caps(path, safety_speed_cap)
            seg_v0, seg_v1 = geometry_caps[:-1][valid], geometry_caps[1:][valid]
            for attempt in range(self.PROFILE_REFINEMENTS+1):
                upper = caps-all_b[:, 3]
                # Conservative coupled envelope: speed is bounded above by caps;
                # steering bound guarantees v^2*tan(delta)/L <= lateral limit.
                # Exact upper bound of the QP speed at node k+1: below the cap AND below
                # v0 + A*t_{k+1} (input bound). Step 26: the cap alone (up to the
                # operator ceiling) over-tightened steering while accelerating.
                vup = np.minimum(caps, state[3]+c.max_accel*self.end_times)
                vmax = np.maximum(np.concatenate(([state[3]], vup[:-1])), vup)
                steer_bounds = np.minimum(c.max_steer, np.arctan(c.max_lateral_accel*c.wheelbase/np.maximum(vmax**2, 1e-6)))
                box_lo = np.column_stack((self.box_accel_lo, -steer_bounds)).ravel()
                box_hi = np.column_stack((self.box_accel_hi, steer_bounds)).ravel()
                matrix = self._stacked_matrix(vmax, steer_bounds)
                lo = np.concatenate((box_lo, prev-rate, lower, np.full(16*n, -np.inf)))
                hi = np.concatenate((box_hi, prev+rate, upper, np.full(16*n, np.cos(np.pi/16))))
                # Fixed sparsity: update friction coefficients with the current speed caps.
                if self.solver is None:
                    self.solver = self.osqp.OSQP()
                    p = self.sparse.csc_matrix((p_values, self.p_rows, self.p_indptr), shape=(2*n, 2*n))
                    self.solver.setup(P=p, q=2*g, A=matrix, l=lo, u=hi,
                        verbose=False, polishing=False, eps_abs=self.QP_EPS, eps_rel=self.QP_EPS,
                        max_iter=c.max_iter, time_limit=c.solver_time_limit,
                        adaptive_rho_interval=10, check_termination=10)
                elif attempt == 0:
                    self.solver.update(Px=p_values, Ax=matrix.data, q=2*g, l=lo, u=hi)
                else:
                    self.solver.update(Ax=matrix.data, l=lo, u=hi)
                if self.previous is None:
                    self.solver.warm_start(x=np.zeros(2*n), y=np.zeros(len(lo)))
                elif self.previous_dual is None or not self.WARM_START_DUAL:
                    self.solver.warm_start(x=self.previous, y=np.zeros(len(lo)))
                else:
                    self.solver.warm_start(x=self.previous, y=self.previous_dual)
                sol = self.solver.solve(raise_error=False)
                # Budget-limited ADMM iterate (hairpin QPs): accepted only if it passes the
                # SAME constraint and nonlinear-envelope checks as a converged solution below;
                # a failing iterate keeps the original osqp:<status> reason (continuation/STOP).
                inexact = (sol.info.status_val != 1 and self.ACCEPT_VERIFIED_INEXACT
                           and sol.info.status in self.INEXACT_STATUSES and sol.x is not None
                           and np.isfinite(sol.x).all() and sol.y is not None and np.isfinite(sol.y).all())
                if sol.info.status_val != 1 and not inexact:
                    result = self._fail('osqp:'+sol.info.status, previous_steering, started)
                    # Still not accepted this tick, but continue from OSQP's last iterate next
                    # tick instead of from zero: an iteration/time budget miss then converges
                    # over a few ticks rather than repeating the same cold solve forever.
                    if (sol.info.status in ('maximum iterations reached', 'run time limit reached', 'solved inaccurate')
                            and sol.x is not None and sol.y is not None
                            and np.isfinite(sol.x).all() and np.isfinite(sol.y).all()):
                        self.previous, self.previous_dual = np.array(sol.x), np.array(sol.y)
                    return result
                u = sol.x
                au = matrix@u
                if not np.isfinite(u).all() or np.any(au < lo-self.QP_ROW_TOL) or np.any(au > hi+self.QP_ROW_TOL):
                    if inexact:
                        result = self._fail('osqp:'+sol.info.status, previous_steering, started)
                        self.previous, self.previous_dual = np.array(sol.x), np.array(sol.y)
                        return result
                    return self._fail('constraint_violation', previous_steering, started)
                controls = u.reshape(n, 2).copy()
                controls[:, 0] = np.clip(controls[:, 0], self.box_accel_lo, self.box_accel_hi)
                previous_delta = float(previous_steering)
                steer_plan = controls[:, 1].tolist()
                for k, (dt, bound) in enumerate(zip(periods.tolist(), steer_bounds.tolist())):
                    previous_delta = min(max(steer_plan[k], max(-bound, previous_delta-c.max_steer_rate*dt)),
                                         min(bound, previous_delta+c.max_steer_rate*dt))
                    steer_plan[k] = previous_delta
                controls[:, 1] = steer_plan
                # Remove numerical tolerance from the commanded speed upper bound.
                controls[0, 0] = min(controls[0, 0], (caps[0]-state[3])/c.dt)
                if controls[0, 0] < -c.max_decel-1e-4:
                    return self._fail('first_speed_bound_infeasible', previous_steering, started)
                controls[0, 0] = max(controls[0, 0], -c.max_decel)
                predicted = np.zeros((n+1, 4))
                predicted[0, 3] = state[3]
                predicted[1:, 3] = state[3]+np.cumsum(periods*controls[:, 0])
                predicted[0, 2] = state[2]
                predicted[1:, 2] = state[2]+np.cumsum(periods*predicted[:-1, 3]*np.tan(controls[:, 1])/c.wheelbase)
                predicted[1:, 0] = np.cumsum(periods*predicted[:-1, 3]*np.cos(predicted[:-1, 2]))
                predicted[1:, 1] = np.cumsum(periods*predicted[:-1, 3]*np.sin(predicted[:-1, 2]))
                lat = np.maximum(predicted[:-1, 3]**2, predicted[1:, 3]**2)*abs(np.tan(controls[:, 1]))/c.wheelbase
                longitudinal = np.where(controls[:, 0] >= 0, c.max_accel, c.max_decel)
                coupled = (controls[:, 0]/longitudinal)**2 + (lat/c.max_lateral_accel)**2
                # Project the nonlinear trajectory onto the path once: used for the
                # control-loss envelope (cross-track / heading to the PATH, step 26:
                # previously distance to the time-indexed reference sample, which also
                # counted along-track lag and STOPped a car that merely accelerated
                # slower than the reference, e.g. permanently at standstill) and for
                # the braking-profile check at the actual projected locations.
                offsets = predicted[1:, None, :2] + state[:2] - origins
                fractions = np.clip(np.sum(offsets*segments, axis=2)/lengths2, 0., 1.)
                distances = np.sum((offsets-fractions[:, :, None]*segments)**2, axis=2)
                indices = np.argmin(distances, axis=1)
                alpha = fractions[np.arange(n), indices]
                cross_track = np.sqrt(distances[np.arange(n), indices])
                heading_err = (predicted[1:, 2]-segment_yaw[indices]+np.pi) % (2*np.pi)-np.pi
                checks = (('coupled_accel', np.max(coupled) > 1.+1e-3),
                          ('negative_speed', np.min(predicted[:, 3]) < -1e-4),
                          ('lateral_accel', np.max(lat) > c.max_lateral_accel+1e-3),
                          ('tracking', np.max(cross_track) > c.max_tracking_error),
                          ('heading', np.max(abs(heading_err)) > c.max_heading_error))
                bad = [name for name, failed in checks if failed]
                if bad:
                    if inexact:
                        result = self._fail('osqp:'+sol.info.status, previous_steering, started)
                        self.previous, self.previous_dual = np.array(sol.x), np.array(sol.y)
                        return result
                    return self._fail('nonlinear_prediction_outside_envelope:'+','.join(bad), previous_steering, started)
                actual_caps = np.minimum((1-alpha)*seg_v0[indices] + alpha*seg_v1[indices], c.max_speed)
                # Step 48: the plan rides the braking envelope; a small overshoot found only
                # at the REACHED positions (VESC lag, pose noise) is the same case as the
                # braking_recovery above: brake at the strongest feasible rate and keep
                # tracking (same 0.30 m/s bound) instead of STOP + soft-stop ramp.
                short = float(np.max(reachable-actual_caps))
                if not braking_recovery and 0. < short <= self.BRAKING_RECOVERY_TOL:
                    braking_recovery = True
                if braking_recovery:
                    floor = reachable+2*self.QP_ROW_TOL+1e-4   # QP residual above the brake floor
                    actual_caps = np.maximum(actual_caps, floor)
                    caps = np.maximum(caps, np.minimum(floor, caps+self.BRAKING_RECOVERY_TOL))
                violation = predicted[1:, 3] > actual_caps+self.QP_ROW_TOL   # step 48: QP residual (2 mm/s)
                if not np.any(violation):
                    break
                # The QP caps are sampled at the reference positions; the solution can
                # run slightly ahead of them into a lower part of the profile. Tighten
                # only the violating stages to the profile at the reached positions
                # (never below the full-braking envelope) and re-solve; the check above
                # is unchanged, so an unresolved violation still STOPs.
                tightened = np.where(violation, np.maximum(np.minimum(caps, actual_caps-2*self.QP_ROW_TOL), reachable), caps)
                if attempt == self.PROFILE_REFINEMENTS or np.all(tightened >= caps):
                    return self._fail('nonlinear_braking_envelope_violation', previous_steering, started)
                caps = tightened
                self.previous, self.previous_dual = np.array(u), np.array(sol.y)
            elapsed = perf_counter()-started
            # Step 34: a fully verified solution that is late only against the soft
            # deadline but still inside the control period is fresher than any plan
            # continuation: serve it (status suffix _late) instead of discarding it.
            late = self.ACCEPT_VERIFIED_LATE and c.deadline < elapsed <= c.dt
            if elapsed > c.deadline and not late:
                result = self._fail('deadline_exceeded', previous_steering, started)
                # The late solution is valid, only late: keep it as the next warm start.
                self.previous, self.previous_dual = u.copy(), np.array(sol.y, copy=True)
                return result
            self.previous = u.copy()
            self.previous_dual = np.array(sol.y, copy=True)
            predicted[:, :2] += state[:2]; ref[:, :2] += state[:2]
            self.plan = {'started': started, 'controls': controls, 'predicted': predicted, 'reference': ref,
                         'caps': caps.copy(), 'steer_bounds': steer_bounds.copy()}
            return IntegratedResult(float(controls[0, 1]), max(0., float(predicted[1, 3])),
                                    float(controls[0, 0]), SafetyRequest.NONE,
                                    ('braking_recovery' if braking_recovery else 'solved')
                                    + ('_inexact_verified' if inexact else '') + ('_late' if late else ''), elapsed,
                                    sol.info.run_time, sol.info.iter, ref, predicted, controls)
        except Exception as exc:
            self.solver = None
            return self._fail(type(exc).__name__+':'+str(exc), previous_steering, started)
