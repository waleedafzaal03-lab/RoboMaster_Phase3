"""Lecture 11A five-state Euler motion prediction and Jacobians."""

import math
import numpy as np


def predict(state, acceleration, angular_acceleration, dt):
    """Return (next_state, F, G).

    state = [x metres, y metres, heading radians,
             forward speed m/s, turn rate rad/s]
    inputs = [forward acceleration m/s^2, angular acceleration rad/s^2]
    """
    x = np.asarray(state, dtype=float)
    if x.shape != (5,) or not np.all(np.isfinite(x)):
        raise ValueError("state must contain five finite numbers")
    if not all(math.isfinite(q) for q in
               (acceleration, angular_acceleration, dt)) or dt <= 0:
        raise ValueError("inputs must be finite and dt must be positive")

    px, py, heading, speed, turn_rate = x
    c, s = math.cos(heading), math.sin(heading)

    next_state = np.array([
        px + speed * c * dt,
        py + speed * s * dt,
        heading + turn_rate * dt,
        speed + acceleration * dt,
        turn_rate + angular_acceleration * dt,
    ])

    F = np.eye(5)
    F[0, 2] = -speed * s * dt
    F[0, 3] = c * dt
    F[1, 2] = speed * c * dt
    F[1, 3] = s * dt
    F[2, 4] = dt

    G = np.zeros((5, 2))
    G[3, 0] = dt
    G[4, 1] = dt

    return next_state, F, G
