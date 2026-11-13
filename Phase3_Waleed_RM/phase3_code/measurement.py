"""Lecture 11B wheel-speed measurement and scalar correction."""

import math
import numpy as np

# State order: [x, y, heading, forward speed, turn rate]
# Wheel/odometry measurement: [forward speed, turn rate]
H = np.array([
    [0., 0., 0., 1., 0.],
    [0., 0., 0., 0., 1.],
])


def expected_measurement(state):
    """Return the speed and turn rate predicted by the state."""
    x = np.asarray(state, dtype=float)
    if x.shape != (5,):
        raise ValueError("state must have five values")
    return H @ x


def correct_with_speed(state, measured_speed, p_vv, p_xv, r_vv):
    """Correct forward speed and x using Lecture 11B, pages 12–15.

    Variances and covariance will later be learned from repeated runs.
    Return (corrected_state, innovation, speed_gain, position_gain).
    """
    x = np.asarray(state, dtype=float).copy()
    values = (measured_speed, p_vv, p_xv, r_vv)
    if x.shape != (5,) or not np.all(np.isfinite(x)):
        raise ValueError("state must contain five finite values")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("measurement and statistics must be finite")
    if p_vv < 0 or r_vv < 0 or p_vv + r_vv <= 0:
        raise ValueError("variances must be non-negative with positive sum")

    innovation = measured_speed - x[3]
    speed_gain = p_vv / (p_vv + r_vv)
    position_gain = p_xv / (p_vv + r_vv)

    x[3] += speed_gain * innovation
    x[0] += position_gain * innovation
    return x, innovation, speed_gain, position_gain
