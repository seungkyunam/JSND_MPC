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
        self.reset()

    def reset(self):
        self.previous = None
        self.previous_dual = None

    def reject(self, reason, previous_steering=0., started=None):
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
            # A measured speed can be above a sharp profile cap when entering
            # a corner.  Enforcing that cap immediately would make the QP
            # mathematically infeasible even though the vehicle can brake
            # into it.  Use the physically reachable envelope for this solve;
            # the original cap remains the target and the next stages continue
            # braking toward it.
            reachable = state[3] - c.max_decel * self.preview_times
            caps = np.maximum(caps, np.maximum(0., reachable))
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
            lower, upper = -all_b[:, 3], caps-all_b[:, 3]
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
            matrix = self.matrix      # constant (see __init__)
            rate = c.max_steer_rate*periods
            lo = np.concatenate((box_lo, prev-rate, lower))
            hi = np.concatenate((box_hi, prev+rate, upper))
            # Reuse the OSQP factorization; A is constant, so only P, q and the
            # bounds change between solves.
            p_values = 2*h[self.p_rows, self.p_cols]
            if self.solver is None:
                self.solver = self.osqp.OSQP()
                p = self.sparse.csc_matrix((p_values, self.p_rows, self.p_indptr), shape=(2*n, 2*n))
                self.solver.setup(P=p, q=2*g, A=matrix, l=lo, u=hi,
                    verbose=False, polishing=False, eps_abs=1e-5, eps_rel=1e-5,
                    max_iter=c.max_iter, time_limit=c.solver_time_limit,
                    adaptive_rho_interval=25, check_termination=10)
            else:
                self.solver.update(Px=p_values, q=2*g, l=lo, u=hi)
            if self.previous is None:
                self.solver.warm_start(x=np.zeros(2*n), y=np.zeros(len(lo)))
            elif self.previous_dual is None or not self.WARM_START_DUAL:
                self.solver.warm_start(x=self.previous, y=np.zeros(len(lo)))
            else:
                self.solver.warm_start(x=self.previous, y=self.previous_dual)
            sol = self.solver.solve(raise_error=False)
            if sol.info.status_val != 1:
                result = self.reject('osqp:'+sol.info.status, previous_steering, started)
                # Still STOP this tick, but continue from OSQP's last iterate next tick
                # instead of from zero: an iteration/time budget miss then converges
                # over a few ticks rather than repeating the same cold solve forever.
                if (sol.info.status in ('maximum iterations reached', 'run time limit reached', 'solved inaccurate')
                        and sol.x is not None and sol.y is not None
                        and np.isfinite(sol.x).all() and np.isfinite(sol.y).all()):
                    self.previous, self.previous_dual = np.array(sol.x), np.array(sol.y)
                return result
            u = sol.x
            au = matrix@u
            if not np.isfinite(u).all() or np.any(au < lo-1e-4) or np.any(au > hi+1e-4):
                return self.reject('constraint_violation', previous_steering, started)
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
                return self.reject('first_speed_bound_infeasible', previous_steering, started)
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
                return self.reject('nonlinear_prediction_outside_envelope', previous_steering, started)
            # Verify the nonlinear trajectory against the profile at its actual
            # projected locations, not only the nominal reference samples.
            segments = np.diff(path[:, :2], axis=0)
            lengths2 = np.sum(segments**2, axis=1)
            valid = lengths2 > 1e-12
            segments, lengths2 = segments[valid], lengths2[valid]
            origins = path[:-1, :2][valid]
            offsets = predicted[1:, None, :2] + state[:2] - origins
            fractions = np.clip(np.sum(offsets*segments, axis=2)/lengths2, 0., 1.)
            distances = np.sum((offsets-fractions[:, :, None]*segments)**2, axis=2)
            indices = np.argmin(distances, axis=1)
            alpha = fractions[np.arange(n), indices]
            actual_caps = ((1-alpha)*path[:-1, 2][valid][indices]
                           + alpha*path[1:, 2][valid][indices])
            if np.any(predicted[1:, 3] > np.minimum(actual_caps, c.max_speed)+1e-4):
                return self.reject('nonlinear_speed_profile_violation', previous_steering, started)
            elapsed = perf_counter()-started
            if elapsed > c.deadline: return self.reject('deadline_exceeded', previous_steering, started)
            self.previous = u.copy()
            self.previous_dual = np.array(sol.y, copy=True)
            predicted[:, :2] += state[:2]; ref[:, :2] += state[:2]
            return IntegratedResult(float(controls[0, 1]), max(0., float(predicted[1, 3])),
                                    float(controls[0, 0]), SafetyRequest.NONE, 'solved', elapsed,
                                    sol.info.run_time, sol.info.iter, ref, predicted, controls)
        except Exception as exc:
            self.solver = None
            return self.reject(type(exc).__name__+':'+str(exc), previous_steering, started)
