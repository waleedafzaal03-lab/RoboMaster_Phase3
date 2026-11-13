"""Phase 3 calibration from 19 completed unfiltered trials.

Endpoint statistics describe repeatability at the measured final position.
Intermediate positions have no independent physical ground truth.
"""

REFERENCE_TRAVEL_M = 1.487

# Median leave-one-out scale for ToF-derived travel.
TOF_TRAVEL_SCALE = 0.9192

# Median endpoint (odometry travel - physical travel).
ODOM_ENDPOINT_BIAS_M = -0.014

# Across-trial endpoint error standard deviations.
ODOM_ENDPOINT_STD_M = 0.00803
TOF_ENDPOINT_STD_M = 0.01381

# Use a ToF reading only when it is fresh. Reused readings do not
# constitute new independent measurements.
MAX_TOF_AGE_S = 0.30
MIN_TOF_UPDATE_INTERVAL_S = 1.0


def corrected_odom_progress(odom_x_m):
    """First approximation: endpoint bias grows with travel."""
    return odom_x_m - ODOM_ENDPOINT_BIAS_M * (
        odom_x_m / REFERENCE_TRAVEL_M
    )


def tof_progress(start_tof_m, current_tof_m):
    """Travel estimate from the calibrated change in box range."""
    return TOF_TRAVEL_SCALE * (start_tof_m - current_tof_m)
