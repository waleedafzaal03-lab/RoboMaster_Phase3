"""Five-state prediction and position correction for Phase 3."""

import math
import numpy as np

from .motion_model import predict


class PositionEKF:
    """State: [forward x, lateral y, heading, speed, turn rate]."""

    def __init__(self, state, covariance):
        self.x = np.asarray(state, dtype=float).copy()
        self.P = np.asarray(covariance, dtype=float).copy()
        if self.x.shape != (5,) or self.P.shape != (5, 5):
            raise ValueError("Expected a five-state vector and 5x5 covariance")
        if not np.all(np.isfinite(self.x)) or not np.all(np.isfinite(self.P)):
            raise ValueError("State and covariance must be finite")

    def predict(self, acceleration, angular_acceleration, dt,
                accel_std=0.30, angular_accel_std=0.30):
        """Lecture 11A: x- = f(x,u), P- = F P F' + G Q G'."""
        if not 0 < dt <= 0.5:
            raise ValueError("dt must be between 0 and 0.5 seconds")
        self.x, F, G = predict(
            self.x, acceleration, angular_acceleration, dt
        )
        Q = np.diag([accel_std**2, angular_accel_std**2])
        self.P = F @ self.P @ F.T + G @ Q @ G.T
        self.P = (self.P + self.P.T) / 2
        return self.x.copy()

    def correct_forward_position(self, measured_x_m, variance_m2):
        """Lecture 11B: innovation, Kalman gain, Joseph covariance update."""
        if not math.isfinite(measured_x_m) or not (
            math.isfinite(variance_m2) and variance_m2 > 0
        ):
            raise ValueError("Measurement must be finite; variance positive")

        H = np.array([[1., 0., 0., 0., 0.]])
        innovation = measured_x_m - float(H @ self.x)
        S = float(H @ self.P @ H.T) + variance_m2
        K = (self.P @ H.T) / S

        self.x = self.x + K[:, 0] * innovation
        I_KH = np.eye(5) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + (
            K @ K.T
        ) * variance_m2
        self.P = (self.P + self.P.T) / 2
        return innovation, K[:, 0].copy()
