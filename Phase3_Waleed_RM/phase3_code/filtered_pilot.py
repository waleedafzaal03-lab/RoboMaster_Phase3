"""Phase 3 filtered-progress pilot; motion disabled by default."""

import csv
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy

from .adaptive_filter import PositionEKF
from .feedback_pilot import FeedbackPilot
from .filter_calibration import (
    MAX_TOF_AGE_S,
    MIN_TOF_UPDATE_INTERVAL_S,
    ODOM_ENDPOINT_STD_M,
    REFERENCE_TRAVEL_M,
    TOF_ENDPOINT_STD_M,
    corrected_odom_progress,
    tof_progress,
)
from .odom_progress import forward_progress


class FilteredPilot(FeedbackPilot):
    def __init__(self):
        super().__init__(node_name="phase3_filtered_pilot")
        self.estimate = None
        self.start_tof_m = None
        self.last_estimate_time = None
        self.last_odom_stamp = None
        self.last_tof_stamp = None
        self.last_tof_update = -float("inf")
        self.last_command = (0.0, 0.0)
        self.previous_command = (0.0, 0.0)
        self.filter_log_path = (
            Path.home() / "robomaster_phase3/logs"
            / f"filtered_state_{datetime.now():%Y%m%d_%H%M%S}.csv"
        )
        self.filter_file = self.filter_log_path.open("w", newline="")
        self.filter_writer = csv.writer(self.filter_file)
        self.filter_writer.writerow([
            "time_s", "filtered_x_m", "raw_odom_x_m",
            "filtered_position_variance_m2", "command_v_mps",
            "tof_m", "controller_state"
        ])
        self.get_logger().info(f"Filter state log: {self.filter_log_path}")
        self.get_logger().info(
            "Filtered progress active; raw ToF stop remains enabled"
        )

    def command(self, forward, yaw):
        self.last_command = (float(forward), float(yaw))
        super().command(forward, yaw)

    def progress_from_odom(self, odom):
        """Use the filter when ready; otherwise use normal odometry."""
        if self.estimate is None:
            return super().progress_from_odom(odom)
        return float(self.estimate.x[0])

    def tick(self):
        now = time.monotonic()
        odom = self.odom if self.odom and now - self.odom[0] < 0.5 else None
        tof = self.tof if self.tof and now - self.tof[0] < 0.5 else None

        if (self.estimate is None
                and self.started_at is not None
                and self.start_pose is not None
                and odom is not None and tof is not None):
            self.start_tof_m = float(tof[1])
            self.estimate = PositionEKF(
                np.zeros(5),
                np.diag([0.02**2, 0.03**2, 0.05**2,
                         0.05**2, 0.05**2])
            )
            self.last_estimate_time = now
            self.previous_command = self.last_command

        if self.estimate is not None:
            dt = now - self.last_estimate_time
            if dt > 0:
                a = (self.last_command[0]
                     - self.previous_command[0]) / dt
                alpha = (self.last_command[1]
                         - self.previous_command[1]) / dt
                steps = max(1, math.ceil(dt / 0.1))
                for _ in range(steps):
                    self.estimate.predict(a, alpha, dt / steps)
                self.last_estimate_time = now
                self.previous_command = self.last_command

            if odom is not None and odom[0] != self.last_odom_stamp:
                raw_x = forward_progress(
                    self.start_pose, (odom[1], odom[2])
                )
                measurement = corrected_odom_progress(raw_x)
                odom_std = max(
                    0.004,
                    ODOM_ENDPOINT_STD_M * math.sqrt(
                        max(0.0, min(1.0, raw_x / REFERENCE_TRAVEL_M))
                    )
                )
                self.estimate.correct_forward_position(
                    measurement, odom_std**2
                )
                self.last_odom_stamp = odom[0]

            if (tof is not None
                    and now - tof[0] < MAX_TOF_AGE_S
                    and tof[0] != self.last_tof_stamp
                    and now - self.last_tof_update
                        >= MIN_TOF_UPDATE_INTERVAL_S):
                measurement = tof_progress(
                    self.start_tof_m, float(tof[1])
                )
                far_fraction = max(
                    0.0, min(1.0, (float(tof[1]) - 0.35) / 0.35)
                )
                tof_std = TOF_ENDPOINT_STD_M * (
                    1.0 + 1.5 * far_fraction
                )
                self.estimate.correct_forward_position(
                    measurement, tof_std**2
                )
                self.last_tof_stamp = tof[0]
                self.last_tof_update = now

        # The original state machine, guards, tag steering and command
        # publishing remain in FeedbackPilot.tick().
        super().tick()
        if (self.estimate is not None and self.start_pose is not None
                and self.odom is not None):
            raw_x = forward_progress(
                self.start_pose, (self.odom[1], self.odom[2])
            )
            self.filter_writer.writerow([
                now - self.started_at, float(self.estimate.x[0]),
                raw_x, float(self.estimate.P[0, 0]),
                self.last_command[0],
                self.tof[1] if self.tof is not None else "",
                self.last_state.value
                if hasattr(self.last_state, "value") else self.last_state
            ])
            self.filter_file.flush()


def main():
    rclpy.init()
    node = FilteredPilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if not node.finished:
            raise
    finally:
        try:
            if rclpy.ok():
                node.command(0.0, 0.0)
        finally:
            node.filter_file.close()
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
