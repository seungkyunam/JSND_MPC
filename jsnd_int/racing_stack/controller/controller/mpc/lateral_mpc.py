"""Condensed steering-only LTV MPC. No ROS, maps, publishers or speed commands.

The caller supplies an ordered local path and fixed speed forecast. Geometry is
recomputed from that path (including avoidance offsets), not a global map's yaw.
Every unsuccessful result requests STOP; a held steer is not a safe-speed claim.
"""

from dataclasses import dataclass
from enum import Enum
from time import perf_counter

import numpy as np

from .bicycle import linearize, step


class SafetyRequest(str, Enum):
    NONE = "none"
    STOP = "stop"


@dataclass(frozen=True)
class MPCConfig:
    wheelbase: float
    max_steer: float
    max_steer_rate: float
    dt: float = 0.02
    horizon: int = 25
    q_position: float = 12.0
    q_heading: float = 4.0
    r_steer: float = 0.2
    r_rate: float = 1.0
    terminal_scale: float = 3.0
    solver_time_limit: float = 0.008
    deadline: float = 0.015
    max_iter: int = 400
    max_tracking_error: float = 1.0
    max_heading_error: float = 0.7

    def __post_init__(self):
        for name, value in vars(self).items():
            if name in getattr(self, 'ZERO_ALLOWED', ()):  # step 47: optional features, 0 = off
                continue
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"MPC {name} must be finite and positive")
        if not isinstance(self.horizon, int) or not 2 <= self.horizon <= 100:
            raise ValueError("MPC horizon must be an integer in [2, 100]")
        if not isinstance(self.max_iter, int):
            raise ValueError("MPC max_iter must be an integer")
        if self.max_steer >= np.pi / 2:
            raise ValueError("MPC steering must be below pi/2")
        if not self.solver_time_limit <= self.deadline <= self.dt:
            raise ValueError("Require solver_time_limit <= deadline <= control dt")


@dataclass(frozen=True)
class MPCResult:
    steering: float
    safety_request: SafetyRequest
    status: str
    elapsed_s: float
    solver_s: float = 0.0
    iterations: int = 0
    reference: object = None
    predicted: object = None
    controls: object = None

    @property
    def success(self):
        return self.safety_request == SafetyRequest.NONE


class LateralMPC:
    def __init__(self, config):
        # Optional dependency: PP must remain usable without OSQP/SciPy.
        import osqp
        from scipy import sparse

        self.config = config
        self._osqp = osqp
        self._sparse = sparse
        self._solver = None
        self._previous_solution = None
        n = config.horizon
        self._difference = np.eye(n) - np.eye(n, k=-1)
        self._constraints = sparse.csc_matrix(np.vstack((np.eye(n), self._difference)))
        # Explicit upper-triangular CSC pattern, including zeros: never change
        # sparsity when updating the LTV Hessian between straight/curved paths.
        self._p_rows = np.concatenate([np.arange(j + 1) for j in range(n)])
        self._p_cols = np.concatenate([np.full(j + 1, j) for j in range(n)])
        self._p_indptr = np.r_[0, np.cumsum(np.arange(1, n + 1))]

    def reset(self):
        self._previous_solution = None
        if self._solver is not None:
            self._solver.warm_start(
                x=np.zeros(self.config.horizon), y=np.zeros(2 * self.config.horizon))

    def reject(self, reason, previous_steering, started=None, solver_s=0.0, iterations=0):
        self._previous_solution = None
        try:
            steer = float(previous_steering)
            if not np.isfinite(steer):
                steer = 0.0
        except (TypeError, ValueError):
            steer = 0.0
        return MPCResult(
            float(np.clip(steer, -self.config.max_steer, self.config.max_steer)),
            SafetyRequest.STOP, reason,
            0.0 if started is None else perf_counter() - started,
            solver_s, iterations,
        )

    def _reference(self, state, waypoints, speeds):
        """Project onto a local polyline, then sample by travelled arc length.

        No cyclic wrapping or extrapolation past a local path endpoint. Headings
        are unwrapped before interpolation, including across +/-pi.
        """
        c = self.config
        xy = np.asarray(waypoints, dtype=float)
        if xy.ndim != 2 or xy.shape[1] < 2 or len(xy) < 3:
            raise ValueError("invalid_path_shape")
        xy = xy[:, :2]
        if not np.isfinite(xy).all():
            raise ValueError("nonfinite_path")
        # Work near the vehicle origin to avoid large map coordinates in the QP.
        xy = xy - state[:2]
        keep = np.r_[True, np.linalg.norm(np.diff(xy, axis=0), axis=1) > 1e-6]
        xy = xy[keep]
        if len(xy) < 3:
            raise ValueError("degenerate_path")
        segments = np.diff(xy, axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        arc = np.r_[0.0, np.cumsum(lengths)]
        fraction = np.clip(-np.sum(xy[:-1] * segments, axis=1) / lengths**2, 0.0, 1.0)
        projections = xy[:-1] + fraction[:, None] * segments
        nearest = int(np.argmin(np.sum(projections**2, axis=1)))
        if np.linalg.norm(projections[nearest]) > c.max_tracking_error:
            raise ValueError("outside_tracking_envelope")
        start = arc[nearest] + fraction[nearest] * lengths[nearest]
        sample_s = start + np.r_[0.0, np.cumsum(speeds) * c.dt]
        if sample_s[-1] > arc[-1] + 1e-6:
            raise ValueError("insufficient_path_horizon")
        tangent = np.gradient(xy, arc, axis=0)
        headings = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
        yaw = np.interp(sample_s, arc, headings)
        yaw += 2 * np.pi * np.round((state[2] - yaw[0]) / (2 * np.pi))
        if abs(yaw[0] - state[2]) > c.max_heading_error:
            raise ValueError("outside_heading_envelope")
        curvature = np.gradient(headings, arc)
        steer_ref = np.arctan(c.wheelbase * np.interp(sample_s[:-1], arc, curvature))
        if np.max(np.abs(steer_ref)) > c.max_steer + 1e-6:
            raise ValueError("reference_exceeds_steering_limit")
        reference = np.column_stack((np.interp(sample_s, arc, xy[:, 0]),
                                     np.interp(sample_s, arc, xy[:, 1]), yaw))
        return reference, steer_ref

    def solve(self, state, waypoints, speed_prediction, previous_steering):
        """state: rear-axle [x,y,yaw]; speed: scalar or N external samples [m/s]."""
        started = perf_counter()
        c = self.config
        n = c.horizon
        try:
            state = np.asarray(state, dtype=float)
            speeds = np.asarray(speed_prediction, dtype=float)
            if speeds.ndim == 0:
                speeds = np.full(n, float(speeds))
            if (state.shape != (3,) or not np.isfinite(state).all()
                    or speeds.shape != (n,) or not np.isfinite(speeds).all()
                    or np.any(speeds < 0) or not np.isfinite(previous_steering)):
                raise ValueError("invalid_state_or_speed")
            if abs(previous_steering) > c.max_steer + 1e-6:
                raise ValueError("previous_steering_outside_limit")
            reference, steer_ref = self._reference(state, waypoints, speeds)
            # Condense all states into z_k = M_k delta + b_k. Decision variables
            # contain ONLY steering; speed is never part of the decision vector.
            m = np.zeros((3, n))
            b = np.array([0.0, 0.0, state[2]])
            hessian = c.r_steer * np.eye(n) + c.r_rate * self._difference.T @ self._difference
            previous = np.zeros(n)
            previous[0] = previous_steering
            gradient = -c.r_steer * steer_ref - c.r_rate * self._difference.T @ previous
            q_diag = np.array([c.q_position, c.q_position, c.q_heading])
            for k in range(n):
                a, b_input, affine = linearize(reference[k], steer_ref[k], speeds[k], c.dt, c.wheelbase)
                m = a @ m
                m[:, k] += b_input
                b = a @ b + affine
                weights = q_diag * (c.terminal_scale if k == n - 1 else 1.0)
                hessian += m.T @ (weights[:, None] * m)
                gradient += m.T @ (weights * (b - reference[k + 1]))
            p_values = 2 * hessian[self._p_rows, self._p_cols]
            rate_step = c.max_steer_rate * c.dt
            lower = np.r_[np.full(n, -c.max_steer), previous - rate_step]
            upper = np.r_[np.full(n, c.max_steer), previous + rate_step]
            if self._solver is None:
                self._solver = self._osqp.OSQP()
                p = self._sparse.csc_matrix((p_values, self._p_rows, self._p_indptr), shape=(n, n))
                self._solver.setup(P=p, q=2 * gradient, A=self._constraints, l=lower, u=upper,
                                   verbose=False, warm_starting=True, polishing=False,
                                   max_iter=c.max_iter, eps_abs=1e-5, eps_rel=1e-5,
                                   check_termination=10, time_limit=c.solver_time_limit)
            else:
                self._solver.update(Px=p_values, q=2 * gradient, l=lower, u=upper)
            if self._previous_solution is not None:
                guess = np.r_[self._previous_solution[1:], self._previous_solution[-1]]
            else:
                guess = np.full(n, previous_steering)
            self._solver.warm_start(x=guess, y=np.zeros(2 * n))
            solution = self._solver.solve(raise_error=False)
            solver_s = float(solution.info.run_time)
            iterations = int(solution.info.iter)
            if solution.info.status_val != 1:  # deliberately reject solved-inaccurate
                return self.reject(f"osqp:{solution.info.status}", previous_steering, started, solver_s, iterations)
            controls = solution.x
            if controls is None or not np.isfinite(controls).all():
                return self.reject("nonfinite_solution", previous_steering, started, solver_s, iterations)
            constrained = self._constraints @ controls
            if np.any(constrained < lower - 1e-4) or np.any(constrained > upper + 1e-4):
                return self.reject("constraint_violation", previous_steering, started, solver_s, iterations)
            # Remove solver tolerance at the applied first input, never PP scaling.
            steering = float(np.clip(controls[0], max(-c.max_steer, previous_steering - rate_step),
                                    min(c.max_steer, previous_steering + rate_step)))
            controls = controls.copy()
            controls[0] = steering
            predicted = [np.array([0.0, 0.0, state[2]])]
            for k in range(n):
                predicted.append(step(predicted[-1], controls[k], speeds[k], c.dt, c.wheelbase))
            predicted = np.asarray(predicted)
            if (not np.isfinite(predicted).all()
                    or np.max(np.linalg.norm(predicted[:, :2] - reference[:, :2], axis=1)) > c.max_tracking_error
                    or np.max(np.abs(predicted[:, 2] - reference[:, 2])) > c.max_heading_error):
                return self.reject("prediction_outside_tracking_envelope", previous_steering, started, solver_s, iterations)
            predicted[:, :2] += state[:2]
            reference[:, :2] += state[:2]
            elapsed = perf_counter() - started
            if elapsed > c.deadline:
                return self.reject("deadline_exceeded", previous_steering, started, solver_s, iterations)
            self._previous_solution = controls.copy()
            return MPCResult(steering, SafetyRequest.NONE, "solved", elapsed, solver_s,
                             iterations, reference, predicted, controls)
        except Exception as exc:
            # Numerical/solver/input failures never reuse an unchecked solution.
            # Include exception type for diagnosis; KeyboardInterrupt is not swallowed.
            self._solver = None
            return self.reject(f"{type(exc).__name__}:{exc}", previous_steering, started)
