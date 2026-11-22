"""ROS-independent joint acceleration/steering LTV MPC; failures request STOP.

Inputs are rear-axle [x,y,yaw,v] and ordered local [x,y,target_speed] points.
The speed profile is both a target and a hard upper bound. A backward braking
pass and curvature limit provide a feasible preview, not a separate controller.
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
    max_accel: float = 1.5
    max_decel: float = 2.5
    max_lateral_accel: float = 3.0
    q_speed: float = 8.0
    r_accel: float = .15
    braking_preview_ratio: float = .85

    def __post_init__(self):
        super().__post_init__()
        if self.braking_preview_ratio > 1.:
            raise ValueError('braking preview cannot exceed maximum deceleration')


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
    MAX_CONTINUATION_TICKS = 2          # at most 2 consecutive ticks (40 ms at 50 Hz)
    CONTINUATION_TOL = (.10, .15, .30)  # measured vs planned: position m, heading rad, speed m/s
    PROFILE_REFINEMENTS = 2             # re-solves with tightened stage caps on a profile miss

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
        self.h_base = np.diag(np.tile([config.r_accel, config.r_steer], n)) + config.r_rate*d.T@d
        # Speed rows of the condensed prediction: v_{k+1} = v_0 + sum_j<=k periods_j*a_j.
        # Row 3 of every linearised A_k is [0,0,0,1] and B_k[3] = [period_k, 0], so these
        # rows (hence the whole constraint matrix) never change between solves.
        speed_rows = np.zeros((n, size))
        for k in range(n):
            speed_rows[k, 0:2*k+2:2] = self.periods[:k+1]
        self.matrix = sparse.csc_matrix(np.vstack((np.eye(size), d, speed_rows)))
        self.state_weights = np.tile([config.q_position, config.q_position, config.q_heading, config.q_speed], (n, 1))
        self.state_weights[-1] *= config.terminal_scale
        self.box_accel_lo = np.full(n, -config.max_decel)
        self.box_accel_hi = np.full(n, config.max_accel)
        self.plan = None          # last accepted, fully verified plan (absolute coordinates)
        self.continuation = 0     # consecutive ticks served from that plan
        self.reset()

    def reset(self):
        self.previous = None
        self.previous_dual = None

    def reject(self, reason, previous_steering=0., started=None):
        """STOP and drop the last plan (external callers: input/behaviour failures)."""
        self.plan = None
        self.continuation = 0
        return self._fail(reason, previous_steering, started)

    def _fail(self, reason, previous_steering=0., started=None):
        self.reset()
        try:
            steer = float(previous_steering)
            if not np.isfinite(steer): steer = 0.
        except (TypeError, ValueError):
            steer = 0.
        return IntegratedResult(float(np.clip(steer, -self.config.max_steer, self.config.max_steer)),
                                0., 0., SafetyRequest.STOP, reason,
                                0. if started is None else perf_counter()-started)

    def _reference(self, state, path, periods):
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
        hard_caps = np.minimum(path[:, 2], c.max_speed)
        hard_caps = np.minimum(hard_caps, np.sqrt(c.max_lateral_accel/np.maximum(abs(curvature), 1e-6)))
        caps = np.minimum(hard_caps, np.sqrt(.85*c.max_lateral_accel/np.maximum(abs(curvature), 1e-6)))
        # Preview braking across the full supplied local path, not just one tick.
        braking = 2*c.braking_preview_ratio*c.max_decel*arc
        caps = np.sqrt(np.maximum(0., np.minimum.accumulate((caps**2+braking)[::-1])[::-1]-braking))
        sample, velocity = float(start), float(state[3])
        samples = [sample]
        for dt in periods.tolist():
            target = float(np.interp(sample, arc, caps))
            v = min(max(target, max(0., velocity-c.max_decel*dt)), velocity+c.max_accel*dt)
            sample += max(.05, .5*(velocity+v))*dt
            velocity = v
            samples.append(sample)
        samples = np.asarray(samples)
        if samples[-1] > arc[-1]:
            raise ValueError('insufficient_path_horizon')
        targets = np.interp(samples[1:], arc, caps)
        # The braking preview is a target, not an instantaneous measured-speed
        # limit. Keep the original profile/physical limits hard; otherwise small
        # velocity-servo lag makes a safe preview target spuriously infeasible.
        limits = np.interp(samples[1:], arc, hard_caps)
        # Include minima between sample points so a narrow low-speed sector is not skipped.
        # arc is strictly increasing, so {arc in [s_k, s_k+1]} is one index slice.
        first = np.searchsorted(arc, samples[:-1], 'left')
        last = np.searchsorted(arc, samples[1:], 'right')
        for k in np.flatnonzero(last > first).tolist():
            limits[k] = min(limits[k], hard_caps[first[k]:last[k]].min())
        reference = np.empty((len(samples), 4))
        reference[:, 0] = np.interp(samples, arc, xy[:, 0])
        reference[:, 1] = np.interp(samples, arc, xy[:, 1])
        reference[:, 2] = np.interp(samples, arc, yaw)
        reference[0, 3] = state[3]
        reference[1:, 3] = targets
        steering = np.arctan(c.wheelbase*np.interp(samples[:-1], arc, curvature))
        return reference, steering, limits

    def solve(self, state, waypoints, previous_steering):
        started = perf_counter()
        result = self._solve(state, waypoints, previous_steering, started)
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
        The status stays visible as 'plan_continuation:<reason>'.
        """
        c, plan = self.config, self.plan
        if plan is None or self.continuation >= self.MAX_CONTINUATION_TICKS:
            return None
        try:
            state = np.asarray(state, float)
            prev = float(previous_steering)
            if state.shape != (4,) or not np.isfinite(state).all() or not np.isfinite(prev):
                return None
            t = (self.continuation+1)*c.dt
            stage = int(np.searchsorted(self.node_times, t, 'right'))-1
            if stage < 1 or stage >= c.horizon:
                return None
            w = (t-self.node_times[stage])/self.periods[stage]
            p = plan['predicted']
            expect = (1-w)*p[stage]+w*p[stage+1]
            pos_tol, yaw_tol, v_tol = self.CONTINUATION_TOL
            dyaw = (state[2]-expect[2]+np.pi) % (2*np.pi)-np.pi
            if (np.hypot(state[0]-expect[0], state[1]-expect[1]) > pos_tol or abs(dyaw) > yaw_tol
                    or abs(state[3]-expect[3]) > v_tol):
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
            accel = max(accel, -c.max_decel)
            v1 = max(0., v0+accel*c.dt)
            if max(v0, v1)**2*abs(np.tan(steer))/c.wheelbase > c.max_lateral_accel+1e-3:
                return None
        except (ValueError, TypeError, IndexError, KeyError):
            return None
        self.continuation += 1
        return IntegratedResult(steer, v1, accel, SafetyRequest.NONE, 'plan_continuation:'+reason,
                                perf_counter()-started, 0., 0, plan['reference'],
                                plan['predicted'][stage:], plan['controls'][stage:])

    def _solve(self, state, waypoints, previous_steering, started):
        c, n = self.config, self.config.horizon
        try:
            state, path = np.asarray(state, float), np.asarray(waypoints, float)
            if (state.shape != (4,) or not np.isfinite(state).all() or state[3] < 0
                    or path.ndim != 2 or path.shape[1] < 3 or len(path) < 3
                    or not np.isfinite(path[:, :3]).all() or np.any(path[:, 2] < 0)
                    or not np.isfinite(previous_steering) or abs(previous_steering) > c.max_steer+1e-6):
                raise ValueError('invalid_state_path_or_steering')
            periods = self.periods
            ref, delta_ref, caps = self._reference(state, path, periods)
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
            # A measured speed can be above a sharp profile cap when entering a
            # corner; the cap cannot be met before the vehicle can brake into it.
            # Allow only the full-braking envelope at the END of each stage (the
            # time v_{k+1} is reached), so such a plan brakes at max_decel from the
            # first stage. The nonlinear profile check below stays strict.
            reachable = np.maximum(0., state[3]-c.max_decel*self.end_times)
            caps = np.maximum(caps, reachable)
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
            g[1::2] -= c.r_steer*delta
            lower = -all_b[:, 3]
            matrix = self.matrix      # constant (see __init__)
            rate = c.max_steer_rate*periods
            p_values = 2*h[self.p_rows, self.p_cols]
            # Profile check geometry (path only; independent of the solution).
            segments = np.diff(path[:, :2], axis=0)
            lengths2 = np.sum(segments**2, axis=1)
            valid = lengths2 > 1e-12
            segments, lengths2 = segments[valid], lengths2[valid]
            origins = path[:-1, :2][valid]
            seg_v0, seg_v1 = path[:-1, 2][valid], path[1:, 2][valid]
            for attempt in range(self.PROFILE_REFINEMENTS+1):
                upper = caps-all_b[:, 3]
                # Conservative coupled envelope: speed is bounded above by caps;
                # steering bound guarantees v^2*tan(delta)/L <= lateral limit.
                vmax = np.maximum(np.concatenate(([state[3]], caps[:-1])), caps)
                steer_bounds = np.minimum(c.max_steer, np.arctan(c.max_lateral_accel*c.wheelbase/np.maximum(vmax**2, 1e-6)))
                # Preserve a feasible first slew step if the measured steering is
                # just outside the new lateral-acceleration envelope.  The rate
                # limit then brings it back inside over subsequent samples.
                steer_bounds[0] = max(steer_bounds[0],
                                      min(c.max_steer, abs(previous_steering)-c.max_steer_rate*periods[0]))
                box_lo = np.column_stack((self.box_accel_lo, -steer_bounds)).ravel()
                box_hi = np.column_stack((self.box_accel_hi, steer_bounds)).ravel()
                lo = np.concatenate((box_lo, prev-rate, lower))
                hi = np.concatenate((box_hi, prev+rate, upper))
                # Reuse the OSQP factorization; A is constant, so only P, q and the
                # bounds change between solves.
                if self.solver is None:
                    self.solver = self.osqp.OSQP()
                    p = self.sparse.csc_matrix((p_values, self.p_rows, self.p_indptr), shape=(2*n, 2*n))
                    self.solver.setup(P=p, q=2*g, A=matrix, l=lo, u=hi,
                        verbose=False, polishing=False, eps_abs=1e-5, eps_rel=1e-5,
                        max_iter=c.max_iter, time_limit=c.solver_time_limit,
                        adaptive_rho_interval=25, check_termination=10)
                elif attempt == 0:
                    self.solver.update(Px=p_values, q=2*g, l=lo, u=hi)
                else:
                    self.solver.update(l=lo, u=hi)
                if self.previous is None:
                    self.solver.warm_start(x=np.zeros(2*n), y=np.zeros(len(lo)))
                elif self.previous_dual is None or not self.WARM_START_DUAL:
                    self.solver.warm_start(x=self.previous, y=np.zeros(len(lo)))
                else:
                    self.solver.warm_start(x=self.previous, y=self.previous_dual)
                sol = self.solver.solve(raise_error=False)
                if sol.info.status_val != 1:
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
                if not np.isfinite(u).all() or np.any(au < lo-1e-4) or np.any(au > hi+1e-4):
                    return self._fail('constraint_violation', previous_steering, started)
                controls = u.reshape(n, 2).copy()
                controls[:, 0] = np.clip(controls[:, 0], -c.max_decel, c.max_accel)
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
                if (np.max(lat) > c.max_lateral_accel+1e-3
                        or np.max(np.linalg.norm(predicted[:, :2]-ref[:, :2], axis=1)) > c.max_tracking_error
                        or np.max(abs(predicted[:, 2]-ref[:, 2])) > c.max_heading_error):
                    return self._fail('nonlinear_prediction_outside_envelope', previous_steering, started)
                # Verify the nonlinear trajectory against the profile at its actual
                # projected locations, not only the nominal reference samples.
                offsets = predicted[1:, None, :2] + state[:2] - origins
                fractions = np.clip(np.sum(offsets*segments, axis=2)/lengths2, 0., 1.)
                distances = np.sum((offsets-fractions[:, :, None]*segments)**2, axis=2)
                indices = np.argmin(distances, axis=1)
                alpha = fractions[np.arange(n), indices]
                actual_caps = np.minimum((1-alpha)*seg_v0[indices] + alpha*seg_v1[indices], c.max_speed)
                violation = predicted[1:, 3] > actual_caps+1e-4
                if not np.any(violation):
                    break
                # The QP caps are sampled at the reference positions; the solution can
                # run slightly ahead of them into a lower part of the profile. Tighten
                # only the violating stages to the profile at the reached positions
                # (never below the full-braking envelope) and re-solve; the check above
                # is unchanged, so an unresolved violation still STOPs.
                tightened = np.where(violation, np.maximum(np.minimum(caps, actual_caps-1e-3), reachable), caps)
                if attempt == self.PROFILE_REFINEMENTS or np.all(tightened >= caps):
                    return self._fail('nonlinear_speed_profile_violation', previous_steering, started)
                caps = tightened
                self.previous, self.previous_dual = np.array(u), np.array(sol.y)
            elapsed = perf_counter()-started
            if elapsed > c.deadline:
                result = self._fail('deadline_exceeded', previous_steering, started)
                # The late solution is valid, only late: keep it as the next warm start.
                self.previous, self.previous_dual = u.copy(), np.array(sol.y, copy=True)
                return result
            self.previous = u.copy()
            self.previous_dual = np.array(sol.y, copy=True)
            predicted[:, :2] += state[:2]; ref[:, :2] += state[:2]
            self.plan = {'controls': controls, 'predicted': predicted, 'reference': ref,
                         'caps': caps.copy(), 'steer_bounds': steer_bounds.copy()}
            return IntegratedResult(float(controls[0, 1]), max(0., float(predicted[1, 3])),
                                    float(controls[0, 0]), SafetyRequest.NONE, 'solved', elapsed,
                                    sol.info.run_time, sol.info.iter, ref, predicted, controls)
        except Exception as exc:
            self.solver = None
            return self._fail(type(exc).__name__+':'+str(exc), previous_steering, started)
