"""Camera-position target with an independent ToF safety stop."""

import time

import cv2
import numpy as np
import rclpy

from rm_phase3_estimation.baseline import BaselinePilot
from rm_phase3_estimation.reference import CubicReference
from experiment_config import TARGET_ID, PILOT_STOP_TOF_M
from tag_pose import detect_target

MEASURED_FRONT_M = 0.370
MEASURED_CAMERA_Z_M = 0.5228
TARGET_FRONT_M = 0.200
CAMERA_STOP_Z_M = MEASURED_CAMERA_Z_M - (MEASURED_FRONT_M - TARGET_FRONT_M)


class CameraStopPilot(BaselinePilot):
    def __init__(self):
        super().__init__(
            node_name="lecture11_camera_stop_pilot",
            min_stop_tof_m=PILOT_STOP_TOF_M,
        )
        # The extra duration lets the robot continue slowly near the box.
        self.ref = CubicReference(travel_m=2.0, duration_s=20.0)
        self.tag_pause_started = None
        self.get_logger().info(
            f"Camera stop depth: {CAMERA_STOP_Z_M:.4f} m"
        )

    def on_image(self, msg):
        info = self.info
        if info is None or msg.encoding not in ("bgr8", "rgb8"):
            return
        if (msg.width, msg.height) != (info.width, info.height):
            return

        pixels = np.frombuffer(msg.data, dtype=np.uint8)
        image = pixels.reshape(msg.height, msg.step)
        image = image[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        result = detect_target(image, info.k, info.d, TARGET_ID)
        if result is not None:
            bearing, camera_z = result
            self.tag = (time.monotonic(), bearing, camera_z)

    def tick(self):
        now = time.monotonic()
        tag = self.tag

        if not self.finished and self.started_at is not None:
            tof = self.tof
            # Never pause past a missing ToF reading or the ToF stop distance.
            if (tof is None or now - tof[0] >= 0.5
                    or tof[1] <= self.stop_tof_m):
                super().tick()
                return

            if tag is None or now - tag[0] >= 0.25:
                if self.tag_pause_started is None:
                    self.tag_pause_started = now
                    self.get_logger().warning(
                        "PAUSE_TAG_LOST | commanding zero speed"
                    )
                self.command(0.0, 0.0)
                if now - self.tag_pause_started > 2.0:
                    self.finished = True
                    self.get_logger().error(
                        "ABORT_TAG_LOST_PROLONGED | T3 missing over 2 s"
                    )
                return

            if self.tag_pause_started is not None:
                pause_s = now - self.tag_pause_started
                self.started_at += pause_s
                self.tag_pause_started = None
                self.get_logger().info(
                    f"RESUME_TAG | paused {pause_s:.2f} s"
                )

        if (not self.finished and tag is not None
                and now - tag[0] < 0.25
                and tag[2] <= CAMERA_STOP_Z_M):
            self.finished = True
            self.command(0.0, 0.0)
            self.get_logger().info(
                f"PILOT_STOP_CAMERA | camera_z={tag[2]:.3f} m "
                f"| target={CAMERA_STOP_Z_M:.3f} m"
            )
            return

        super().tick()


def main():
    rclpy.init()
    node = CameraStopPilot()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.command(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
