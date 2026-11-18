"""Rear-axle kinematic bicycle, forward Euler, exogenous longitudinal speed.

State is [x, y, yaw], input is road-wheel steering [rad]. No speed dynamics,
wheelbase defaults, slip assumptions, or ROS dependencies belong in this model.
"""

import numpy as np


def step(state, steering, speed, dt, wheelbase):
    x, y, yaw = state
    return np.array([
        x + dt * speed * np.cos(yaw),
        y + dt * speed * np.sin(yaw),
        yaw + dt * speed * np.tan(steering) / wheelbase,
    ])


def linearize(state, steering, speed, dt, wheelbase):
    """Return A, B, c for z_next = A z + B delta + c about (state, steering)."""
    yaw = state[2]
    a = np.eye(3)
    a[0, 2] = -dt * speed * np.sin(yaw)
    a[1, 2] = dt * speed * np.cos(yaw)
    b = np.array([0.0, 0.0, dt * speed / (wheelbase * np.cos(steering)**2)])
    c = step(state, steering, speed, dt, wheelbase) - a @ state - b * steering
    return a, b, c
