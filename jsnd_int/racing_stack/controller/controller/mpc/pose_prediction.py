"""Opt-in input conditioning; no solver, physical-limit or safety-gate changes.

The heading innovation limiter uses the EKF's body-frame yaw rate, NOT raw IMU
orientation and NOT the legacy sign-flipped /imu/data callback. It does not
low-pass normal rotation: gyro-predicted rotation passes at the current sample.
Command history models a fixed transport delay, not measured servo dynamics.
"""
from collections import deque
from time import monotonic

import numpy as np


def angle(value):
    return (value + np.pi) % (2 * np.pi) - np.pi


class PosePrediction:
    # Fixed, predeclared Step 45 design: limit absolute correction to the Step 38
    # p95 heading difference (0.17 rad) over 0.1 s, rather than sweeping gains.
    CORRECTION_RATE = 1.7  # rad/s, absolute-pose innovation only, not vehicle yaw rate
    MAX_SAMPLE_GAP = 0.1

    def __init__(self, clock=monotonic):
        self.clock = clock
        self.commands = deque(maxlen=256)
        self.sample = None
        self.heading = None
        self.previous_stamp = None
        self.previous_xy = None
        self.limited_samples = 0

    def observe(self, stamp, yaw_rate):
        """stamp and command clock share a time base; EKF angular.z is base_link."""
        self.sample = (float(stamp), float(yaw_rate))

    def record(self, steering):
        now = self.clock()
        if not np.isfinite(steering):
            return
        if self.commands and now < self.commands[-1][0]:
            self.commands.clear()
        if self.commands and now == self.commands[-1][0]:
            self.commands[-1] = (now, float(steering))
        else:
            self.commands.append((now, float(steering)))
        while len(self.commands) > 1 and self.commands[1][0] < now - 1.0:
            self.commands.popleft()

    def condition_heading(self, pose):
        """Limit new-sample innovation; duplicates never integrate gyro twice.

        Invalid/stale rate, time gaps and spatial relocalisation re-anchor to the
        input pose. The existing manager freshness/finite-input guards still own
        stopping; this helper never manufactures fresh odometry.
        """
        raw = float(pose[2])
        if self.sample is None:
            self.heading = None
            return raw
        stamp, rate = self.sample
        age = self.clock() - stamp
        if not np.isfinite([raw, stamp, rate, age, *pose[:2]]).all() or not 0 <= age <= self.MAX_SAMPLE_GAP:
            self.heading = None
            self.previous_stamp = None
            return raw
        if stamp == self.previous_stamp and self.heading is not None:
            return self.heading
        dt = 0.0 if self.previous_stamp is None else stamp - self.previous_stamp
        relocated = self.previous_xy is not None and np.linalg.norm(pose[:2] - self.previous_xy) > 0.5
        if self.heading is None or not 0 < dt <= self.MAX_SAMPLE_GAP or relocated:
            self.heading = raw
        else:
            predicted = self.heading + rate * dt
            innovation = angle(raw - predicted)
            bound = self.CORRECTION_RATE * dt
            self.limited_samples += int(abs(innovation) > bound)
            self.heading = angle(predicted + np.clip(innovation, -bound, bound))
        self.previous_stamp = stamp
        self.previous_xy = np.array(pose[:2], copy=True)
        return self.heading

    def predict(self, pose, speed, wheelbase, horizon):
        """Forecast through the pending command queue over ONE delay horizon.

        A command issued at t-horizon becomes active at t. Missing startup/gap
        history is neutral (0 rad); never extend a stale command across a gap.
        Integration splits at command timestamps and at <=20 ms steps. Constant
        speed matches the legacy predictor; no extra pose-age extrapolation.
        """
        result = np.array(pose, dtype=float, copy=True)
        if horizon <= 0:
            return result
        now = self.clock()
        start = now - horizon
        history = [(t, d) for t, d in self.commands if t <= now]
        boundaries = sorted({start, now, *(t for t, _ in history if start < t < now),
                             *(t + self.MAX_SAMPLE_GAP for t, _ in history
                               if start < t + self.MAX_SAMPLE_GAP < now)})
        for begin, end in zip(boundaries, boundaries[1:]):
            available = [(t, d) for t, d in history if t <= begin + 1e-12]
            steering = available[-1][1] if available and begin - available[-1][0] < self.MAX_SAMPLE_GAP else 0.0
            count = max(1, int(np.ceil((end - begin) / 0.02)))
            dt = (end - begin) / count
            for _ in range(count):
                result += dt * speed * np.array([np.cos(result[2]), np.sin(result[2]),
                                                 np.tan(steering) / wheelbase])
        return result
