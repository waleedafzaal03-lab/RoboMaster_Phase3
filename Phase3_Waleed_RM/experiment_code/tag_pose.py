"""Tag bearing and camera depth using the measured marker size."""

import math

import cv2
import numpy as np

from experiment_config import TAG_EDGE_M

dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_APRILTAG_36h11
)
parameters = cv2.aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

half = TAG_EDGE_M / 2.0
tag_corners_3d = np.array([
    [-half,  half, 0.0],
    [ half,  half, 0.0],
    [ half, -half, 0.0],
    [-half, -half, 0.0],
], dtype=np.float64)


def detect_target(bgr_image, camera_matrix, distortion, target_id):
    """Return (bearing_rad, camera_depth_m), or None."""
    gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return None

    k = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    d = np.asarray(distortion, dtype=np.float64)

    for index, marker_id in enumerate(ids.flatten()):
        if int(marker_id) != target_id:
            continue

        image_corners = corners[index].reshape(4, 2).astype(np.float64)
        success, _, translation = cv2.solvePnP(
            tag_corners_3d, image_corners, k, d,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not success or not np.isfinite(translation).all():
            return None
        if translation[2, 0] <= 0:
            return None

        centre_x = image_corners[:, 0].mean()
        bearing = math.atan2(centre_x - k[0, 2], k[0, 0])
        return bearing, float(translation[2, 0])

    return None
