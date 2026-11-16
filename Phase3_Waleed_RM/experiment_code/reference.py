"""Cubic position reference and its exact derivatives."""

import math
from dataclasses import dataclass

from .config import START_FRONT_CLEARANCE_M, STOP_FRONT_CLEARANCE_M


@dataclass(frozen=True)
class CubicReference:
    travel_m: float
    duration_s: float

    def sample(self, t_s):
        """Return desired (x metres, speed m/s, acceleration m/s^2)."""
        if t_s >= self.duration_s:
            return self.travel_m, 0.0, 0.0

        tau = max(0.0, t_s / self.duration_s)
        L, T = self.travel_m, self.duration_s
        x = L * (3 * tau**2 - 2 * tau**3)
        speed = (6 * L / T) * tau * (1 - tau)
        acceleration = (6 * L / T**2) * (1 - 2 * tau)
        return x, speed, acceleration


def make_reference():
    travel = START_FRONT_CLEARANCE_M - STOP_FRONT_CLEARANCE_M
    if not math.isfinite(travel) or travel <= 0:
        raise ValueError("Starting clearance must exceed stopping clearance")

    duration = max(4.0, 1.6 * travel / 0.16)
    return CubicReference(travel, duration)
