"""Steering-only LTV MPC on top of the unchanged JSND speed plan (step 23).

The longitudinal command is NOT decided here: the caller (Controller) runs the legacy
JSND/PP speed pipeline (raceline x sector scaling -> local waypoints -> lookahead speed,
lateral/heading error scaling, trailing controller, AEB). This solver only chooses the
road-wheel steering that tracks the local path, given a speed FORECAST taken from that
same plan (measured speed at stage 0, then the waypoint speeds, ramped by the vehicle's
ggv acceleration so the forecast is physically reachable).

Decision variables: steering only. Constraints: |delta| <= max_steer and
|delta_k - delta_{k-1}| <= max_steer_rate * period_k (actuator limits). The first stage
lasts one control period, later stages preview_dt (longer preview at the same size).

Failures are classified so the caller can tell geometry from compute:
  ENVELOPE_REASONS  vehicle > max_tracking_error from the path / heading error too large
                    -> the caller requests STOP (safety, unchanged from the old MPC).
  anything else     solver/compute miss -> caller may use its fallback steering for a
                    bounded number of ticks, then STOP.
"""
from dataclasses import dataclass
from time import perf_counter

import numpy as np

from .lateral_mpc import MPCConfig, MPCResult, SafetyRequest


@dataclass(frozen=True)
class SteeringConfig(MPCConfig):
    preview_dt: float = .05
    forecast_accel: float = 5.0     # ggv ax_max (CAR/veh_dyn_info/ggv.csv), forecast only
    forecast_decel: float = 5.0
    max_speed: float = 1.0          # operator ceiling (mpc_profile), applied by the Controller


ENVELOPE_REASONS = ('outside_tracking_envelope', 'outside_heading_envelope',
                    'prediction_outside_tracking_envelope')


def _gradient(f, x):
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


class SteeringMPC:
    def __init__(self, config):
        import osqp
        from scipy import sparse
        self.config = c = config
        self.osqp, self.sparse = osqp, sparse
        n = c.horizon
        self.periods = np.concatenate(([c.dt], np.full(n-1, c.preview_dt)))
        self.diff = np.eye(n) - np.eye(n, k=-1)
        self.matrix = sparse.csc_matrix(np.vstack((np.eye(n), self.diff)))
        self.p_rows = np.concatenate([np.arange(j+1) for j in range(n)])
        self.p_cols = np.concatenate([np.full(j+1, j) for j in range(n)])
        self.p_indptr = np.r_[0, np.cumsum(np.arange(1, n+1))]
        self.h_base = c.r_steer*np.eye(n) + c.r_rate*self.diff.T@self.diff
        w = np.tile([c.q_position, c.q_position, c.q_heading], (n, 1))
        w[-1] *= c.terminal_scale
        self.weights = w
        self.solver = None
        self.previous = None

    def reset(self):
        self.previous = None

    def reject(self, reason, previous_steering, started=None):
        self.previous = None
        try:
            steer = float(previous_steering)
            if not np.isfinite(steer):
                steer = 0.
        except (TypeError, ValueError):
            steer = 0.
        c = self.config
        return MPCResult(float(np.clip(steer, -c.max_steer, c.max_steer)), SafetyRequest.STOP, reason,
                         0. if started is None else perf_counter()-started)

    def _reference(self, state, path):
        """Project the rear axle on the local path, forecast speed from the plan, sample
        the path by forecast travel. path columns: x, y, planned speed."""
        c = self.config
        keep = np.concatenate(([True], np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1) > 1e-6))
        path = path[keep]
        if len(path) < 3:
            raise ValueError('degenerate_path')
        xy = path[:, :2] - state[:2]
        seg = np.diff(xy, axis=0)
        ds = np.linalg.norm(seg, axis=1)
        arc = np.concatenate(([0.], np.cumsum(ds)))
        frac = np.clip(-np.sum(xy[:-1]*seg, axis=1)/ds**2, 0., 1.)
        proj = xy[:-1] + frac[:, None]*seg
        i = int(np.argmin(np.sum(proj**2, axis=1)))
        if np.linalg.norm(proj[i]) > c.max_tracking_error:
            raise ValueError('outside_tracking_envelope')
        start = arc[i] + frac[i]*ds[i]
        tangent = _gradient(xy, arc)
        heading = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
        heading += 2*np.pi*np.round((state[2]-np.interp(start, arc, heading))/(2*np.pi))
        if abs(np.interp(start, arc, heading)-state[2]) > c.max_heading_error:
            raise ValueError('outside_heading_envelope')
        curvature = _gradient(heading, arc)
        s, v = float(start), max(0., float(state[3]))
        samples, speeds = [s], []
        for dt in self.periods.tolist():
            target = float(np.interp(s, arc, path[:, 2]))
            v_next = min(max(target, v-c.forecast_decel*dt), v+c.forecast_accel*dt)
            speeds.append(v)                 # stage k is integrated with v_k (explicit Euler)
            s += v*dt
            v = max(0., v_next)
            samples.append(s)
        samples = np.asarray(samples)
        if samples[-1] > arc[-1]:
            raise ValueError('insufficient_path_horizon')
        ref = np.column_stack((np.interp(samples, arc, xy[:, 0]), np.interp(samples, arc, xy[:, 1]),
                               np.interp(samples, arc, heading)))
        steer_ref = np.clip(np.arctan(c.wheelbase*np.interp(samples[:-1], arc, curvature)),
                            -c.max_steer, c.max_steer)
        return ref, steer_ref, np.asarray(speeds)

    def solve(self, state, path, previous_steering):
        """state: rear axle [x, y, yaw, v_measured]; path: local waypoints [x, y, v_plan, ...]."""
        started = perf_counter()
        c, n = self.config, self.config.horizon
        try:
            state = np.asarray(state, float)
            path = np.asarray(path, float)[:, :3]
            prev = float(previous_steering)
            if (state.shape != (4,) or not np.isfinite(state).all() or path.ndim != 2 or len(path) < 3
                    or not np.isfinite(path).all() or not np.isfinite(prev)):
                raise ValueError('invalid_state_or_path')
            prev = float(np.clip(prev, -c.max_steer, c.max_steer))
            ref, delta, v = self._reference(state, path)
            p = self.periods
            yaw = ref[:-1, 2]
            # Linearised rear-axle bicycle around the reference: z=[x,y,yaw], u=delta.
            aa = np.tile(np.eye(3), (n, 1, 1))
            aa[:, 0, 2] = -v*np.sin(yaw)*p
            aa[:, 1, 2] = v*np.cos(yaw)*p
            bb = v*p/(c.wheelbase*np.cos(delta)**2)
            ff = ref[:-1] + p[:, None]*np.column_stack((v*np.cos(yaw), v*np.sin(yaw), v*np.tan(delta)/c.wheelbase))
            affine = ff - np.einsum('nij,nj->ni', aa, ref[:-1])
            affine[:, 2] -= bb*delta
            m = np.zeros((3, n))
            b = np.array([0., 0., state[2]])
            all_m = np.empty((n, 3, n))
            all_b = np.empty((n, 3))
            for k in range(n):
                m = aa[k]@m
                m[2, k] += bb[k]
                b = aa[k]@b + affine[k]
                all_m[k], all_b[k] = m, b
            stacked = all_m.reshape(3*n, n)
            h = self.h_base + stacked.T@(self.weights.reshape(-1, 1)*stacked)
            prev_vec = np.zeros(n)
            prev_vec[0] = prev
            g = stacked.T@(self.weights*(all_b-ref[1:])).ravel() - c.r_steer*delta - c.r_rate*self.diff.T@prev_vec
            rate = c.max_steer_rate*p
            lo = np.concatenate((np.full(n, -c.max_steer), prev_vec-rate))
            hi = np.concatenate((np.full(n, c.max_steer), prev_vec+rate))
            px = 2*h[self.p_rows, self.p_cols]
            if self.solver is None:
                self.solver = self.osqp.OSQP()
                self.solver.setup(P=self.sparse.csc_matrix((px, self.p_rows, self.p_indptr), shape=(n, n)),
                                  q=2*g, A=self.matrix, l=lo, u=hi, verbose=False, polishing=False,
                                  eps_abs=1e-5, eps_rel=1e-5, max_iter=c.max_iter,
                                  time_limit=c.solver_time_limit, check_termination=10)
            else:
                self.solver.update(Px=px, q=2*g, l=lo, u=hi)
            guess = (np.r_[self.previous[1:], self.previous[-1]] if self.previous is not None
                     else np.clip(delta, prev-np.cumsum(rate), prev+np.cumsum(rate)))
            self.solver.warm_start(x=guess)
            sol = self.solver.solve(raise_error=False)
            if sol.info.status_val != 1 or sol.x is None or not np.isfinite(sol.x).all():
                return self.reject('osqp:'+str(sol.info.status), prev, started)
            u = np.array(sol.x)
            # Remove solver tolerance: replay the actuator limits exactly.
            last = prev
            for k in range(n):
                last = float(np.clip(u[k], max(-c.max_steer, last-rate[k]), min(c.max_steer, last+rate[k])))
                u[k] = last
            # Nonlinear rollout check against the geometric envelope (safety, not speed).
            z = np.array([0., 0., state[2]])
            pred = [z]
            for k in range(n):
                z = np.array([z[0]+p[k]*v[k]*np.cos(z[2]), z[1]+p[k]*v[k]*np.sin(z[2]),
                              z[2]+p[k]*v[k]*np.tan(u[k])/c.wheelbase])
                pred.append(z)
            pred = np.asarray(pred)
            if (np.max(np.linalg.norm(pred[:, :2]-ref[:, :2], axis=1)) > c.max_tracking_error
                    or np.max(np.abs(pred[:, 2]-ref[:, 2])) > c.max_heading_error):
                return self.reject('prediction_outside_tracking_envelope', prev, started)
            elapsed = perf_counter()-started
            if elapsed > c.deadline:
                self.previous = u.copy()       # valid but late: keep as warm start
                return MPCResult(float(u[0]), SafetyRequest.STOP, 'deadline_exceeded', elapsed,
                                 float(sol.info.run_time), int(sol.info.iter))
            self.previous = u.copy()
            ref[:, :2] += state[:2]
            pred[:, :2] += state[:2]
            return MPCResult(float(u[0]), SafetyRequest.NONE, 'solved', elapsed, float(sol.info.run_time),
                             int(sol.info.iter), ref, pred, u)
        except ValueError as exc:
            return self.reject(str(exc), previous_steering, started)
        except Exception as exc:
            self.solver = None
            return self.reject(type(exc).__name__+':'+str(exc), previous_steering, started)
