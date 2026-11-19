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


class IntegratedMPC:
    def __init__(self, config):
        import osqp
        from scipy import sparse
        self.config = config
        self.osqp, self.sparse = osqp, sparse
        self.solver = None
        size = 2*config.horizon
        self.p_rows = np.concatenate([np.arange(j+1) for j in range(size)])
        self.p_indptr = np.r_[0, np.cumsum(np.arange(1, size+1))]
        self.p_cols = np.repeat(np.arange(size), np.arange(1, size+1))
        self.reset()

    def reset(self):
        self.previous = None

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
        keep = np.r_[True, np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1) > 1e-6]
        path = path[keep]
        if len(path) < 3:
            raise ValueError('degenerate_path')
        xy = path[:, :2] - state[:2]
        seg = np.diff(xy, axis=0)
        ds = np.linalg.norm(seg, axis=1)
        arc = np.r_[0., np.cumsum(ds)]
        fraction = np.clip(-np.sum(xy[:-1]*seg, axis=1)/ds**2, 0., 1.)
        projected = xy[:-1] + fraction[:, None]*seg
        idx = np.argmin(np.sum(projected**2, axis=1))
        if np.linalg.norm(projected[idx]) > c.max_tracking_error:
            raise ValueError('outside_tracking_envelope')
        start = arc[idx]+fraction[idx]*ds[idx]
        tangent = np.gradient(xy, arc, axis=0)
        yaw = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
        yaw += 2*np.pi*np.round((state[2]-np.interp(start, arc, yaw))/(2*np.pi))
        if abs(np.interp(start, arc, yaw)-state[2]) > c.max_heading_error:
            raise ValueError('outside_heading_envelope')
        curvature = np.gradient(yaw, arc)
        hard_caps = np.minimum(path[:, 2], c.max_speed)
        hard_caps = np.minimum(hard_caps, np.sqrt(c.max_lateral_accel/np.maximum(abs(curvature), 1e-6)))
        caps = np.minimum(hard_caps, np.sqrt(.85*c.max_lateral_accel/np.maximum(abs(curvature), 1e-6)))
        # Preview braking across the full supplied local path, not just one tick.
        braking = 2*c.braking_preview_ratio*c.max_decel*arc
        caps = np.sqrt(np.maximum(0., np.minimum.accumulate((caps**2+braking)[::-1])[::-1]-braking))
        samples = [start]
        velocities = [state[3]]
        for dt in periods:
            target = np.interp(samples[-1], arc, caps)
            v = np.clip(target, max(0., velocities[-1]-c.max_decel*dt), velocities[-1]+c.max_accel*dt)
            samples.append(samples[-1]+max(.05, .5*(velocities[-1]+v))*dt)
            velocities.append(v)
        samples = np.asarray(samples)
        if samples[-1] > arc[-1]:
            raise ValueError('insufficient_path_horizon')
        targets = np.interp(samples[1:], arc, caps)
        # The braking preview is a target, not an instantaneous measured-speed
        # limit. Keep the original profile/physical limits hard; otherwise small
        # velocity-servo lag makes a safe preview target spuriously infeasible.
        limits = np.interp(samples[1:], arc, hard_caps)
        # Include minima between sample points so a narrow low-speed sector is not skipped.
        for k in range(len(periods)):
            inside = (arc >= samples[k]) & (arc <= samples[k+1])
            if np.any(inside): limits[k] = min(limits[k], np.min(hard_caps[inside]))
        reference = np.column_stack((np.interp(samples, arc, xy[:, 0]),
                                     np.interp(samples, arc, xy[:, 1]),
                                     np.interp(samples, arc, yaw),
                                     np.r_[state[3], targets]))
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
            periods = np.r_[c.dt, np.full(n-1, c.preview_dt)]
            ref, delta_ref, caps = self._reference(state, path, periods)
            # A measured speed can be above a sharp profile cap when entering
            # a corner.  Enforcing that cap immediately would make the QP
            # mathematically infeasible even though the vehicle can brake
            # into it.  Use the physically reachable envelope for this solve;
            # the original cap remains the target and the next stages continue
            # braking toward it.
            reachable = state[3] - c.max_decel * np.r_[0., np.cumsum(periods[:-1])]
            caps = np.maximum(caps, np.maximum(0., reachable))
            m = np.zeros((4, 2*n))
            b = np.array([0., 0., state[2], state[3]])
            h = np.diag(np.tile([c.r_accel, c.r_steer], n))
            g = np.zeros(2*n)
            rows, lower, upper = [], [], []
            d = np.zeros((n, 2*n))
            for k in range(n):
                d[k, 2*k+1] = 1.
                if k: d[k, 2*k-1] = -1.
            prev = np.r_[previous_steering, np.zeros(n-1)]
            h += c.r_rate*d.T@d
            g -= c.r_rate*d.T@prev
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
            weights = np.tile([c.q_position, c.q_position, c.q_heading, c.q_speed], (n, 1))
            weights[-1] *= c.terminal_scale
            stacked = all_m.reshape(4*n, 2*n)
            h += stacked.T@(weights.reshape(-1, 1)*stacked)
            g += stacked.T@(weights*(all_b-ref[1:])).ravel()
            g[1::2] -= c.r_steer*delta
            rows = all_m[:, 3, :]
            lower, upper = -all_b[:, 3], caps-all_b[:, 3]
            # Conservative coupled envelope: speed is bounded above by caps;
            # steering bound guarantees v^2*tan(delta)/L <= lateral limit.
            vmax = np.maximum(np.r_[state[3], caps[:-1]], caps)
            steer_bounds = np.minimum(c.max_steer, np.arctan(c.max_lateral_accel*c.wheelbase/np.maximum(vmax**2, 1e-6)))
            # Preserve a feasible first slew step if the measured steering is
            # just outside the new lateral-acceleration envelope.  The rate
            # limit then brings it back inside over subsequent samples.
            steer_bounds[0] = max(steer_bounds[0],
                                  min(c.max_steer, abs(previous_steering)-c.max_steer_rate*periods[0]))
            box_lo = np.column_stack((np.full(n, -c.max_decel), -steer_bounds)).ravel()
            box_hi = np.column_stack((np.full(n, c.max_accel), steer_bounds)).ravel()
            matrix = self.sparse.csc_matrix(np.vstack((np.eye(2*n), d, rows)))
            lo = np.r_[box_lo, prev-c.max_steer_rate*periods, lower]
            hi = np.r_[box_hi, prev+c.max_steer_rate*periods, upper]
            # Reuse the OSQP factorization.  The speed-row coefficients change
            # with the linearization, so update A's numeric values as well as
            # the bounds and cost on every solve.
            p_values = 2*h[self.p_rows, self.p_cols]
            if self.solver is None:
                self.solver = self.osqp.OSQP()
                p = self.sparse.csc_matrix((p_values, self.p_rows, self.p_indptr), shape=(2*n, 2*n))
                self.solver.setup(P=p, q=2*g, A=matrix, l=lo, u=hi,
                    verbose=False, polishing=False, eps_abs=1e-5, eps_rel=1e-5,
                    max_iter=c.max_iter, time_limit=c.solver_time_limit,
                    adaptive_rho_interval=25, check_termination=10)
            else:
                self.solver.update(Px=p_values, Ax=matrix.data, q=2*g, l=lo, u=hi)
            self.solver.warm_start(x=self.previous if self.previous is not None else np.zeros(2*n),
                                   y=np.zeros(len(lo)))
            sol = self.solver.solve(raise_error=False)
            if sol.info.status_val != 1: return self.reject('osqp:'+sol.info.status, previous_steering, started)
            u = sol.x
            if not np.isfinite(u).all() or np.any(matrix@u < lo-1e-4) or np.any(matrix@u > hi+1e-4):
                return self.reject('constraint_violation', previous_steering, started)
            controls = u.reshape(n, 2).copy()
            controls[:, 0] = np.clip(controls[:, 0], -c.max_decel, c.max_accel)
            previous_delta = previous_steering
            for k, dt in enumerate(periods):
                controls[k, 1] = np.clip(controls[k, 1],
                    max(-steer_bounds[k], previous_delta-c.max_steer_rate*dt),
                    min(steer_bounds[k], previous_delta+c.max_steer_rate*dt))
                previous_delta = controls[k, 1]
            # Remove numerical tolerance from the commanded speed upper bound.
            controls[0, 0] = min(controls[0, 0], (caps[0]-state[3])/c.dt)
            if controls[0, 0] < -c.max_decel-1e-4:
                return self.reject('first_speed_bound_infeasible', previous_steering, started)
            predicted = np.zeros((n+1, 4))
            predicted[:, 3] = np.r_[state[3], state[3]+np.cumsum(periods*controls[:, 0])]
            predicted[:, 2] = np.r_[state[2], state[2]+np.cumsum(
                periods*predicted[:-1, 3]*np.tan(controls[:, 1])/c.wheelbase)]
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
            predicted[:, :2] += state[:2]; ref[:, :2] += state[:2]
            return IntegratedResult(float(controls[0, 1]), max(0., float(predicted[1, 3])),
                                    float(controls[0, 0]), SafetyRequest.NONE, 'solved', elapsed,
                                    sol.info.run_time, sol.info.iter, ref, predicted, controls)
        except Exception as exc:
            self.solver = None
            return self.reject(type(exc).__name__+':'+str(exc), previous_steering, started)
