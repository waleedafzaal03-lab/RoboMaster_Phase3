import argparse
import csv
import math
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, Range

from tag_pose import detect_target
from experiment_config import TARGET_ID, WHEEL_NAMES, WHEEL_RADIUS_M

log_dir = Path.home() / "robomaster_phase3/lecture11_wheel_error/logs"
log_dir.mkdir(parents=True, exist_ok=True)
parser = argparse.ArgumentParser()
parser.add_argument("--label", default="stationary_check")
parser.add_argument("--duration", type=float, default=15.0)
args = parser.parse_args()
if not args.label or not args.label.replace("_", "").isalnum():
    parser.error("label must contain only letters, numbers, or underscores")
if not 1 <= args.duration <= 180:
    parser.error("duration must be between 1 and 180 seconds")

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
path = log_dir / f"{args.label}_{stamp}.csv"

fields = [
    "t_s", "odom_stamp_s", "x_m", "y_m", "yaw_rad",
    "v_odom_mps", "omega_odom_rps",
    "t3_bearing_rad", "t3_camera_z_m", "tag_stamp_s", "tag_age_s",
    "tof_m", "tof_age_s", "cmd_v_mps", "cmd_omega_rps",
    "wheel_stamp_s", "wheel_age_s", "wheel_forward_mps",
    "omega_fr_rad_s", "omega_fl_rad_s",
    "omega_rl_rad_s", "omega_rr_rad_s"
]

rclpy.init()
node = rclpy.create_node("lecture11_passive_recorder")
file = path.open("w", newline="")
writer = csv.DictWriter(file, fieldnames=fields)
writer.writeheader()

data = {"info": None, "tag": None, "tof": None, "cmd": None,
        "wheel": None,
        "start": None, "origin": None, "rows": 0, "tag_frames": 0}

def on_info(msg):
    data["info"] = msg

def on_image(msg):
    info = data["info"]
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
        data["tag"] = (
            time.monotonic(), bearing, camera_z,
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )
        data["tag_frames"] += 1

def on_range(msg):
    data["tof"] = (time.monotonic(), float(msg.range))

def on_wheel(msg):
    if len(msg.name) != len(msg.velocity):
        return
    by_name = dict(zip(msg.name, msg.velocity))
    if any(name not in by_name for name in WHEEL_NAMES):
        return
    omega = tuple(float(by_name[name]) for name in WHEEL_NAMES)
    if not all(math.isfinite(value) for value in omega):
        return
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    data["wheel"] = (time.monotonic(), stamp, omega)


def on_cmd(msg):
    data["cmd"] = (float(msg.linear.x), float(msg.angular.z))

def on_odom(msg):
    now = time.monotonic()
    pose = msg.pose.pose
    q = pose.orientation
    yaw = math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
    )
    x, y = pose.position.x, pose.position.y

    if data["start"] is None:
        data["start"] = now
        data["origin"] = (x, y, yaw)

    x0, y0, yaw0 = data["origin"]
    dx, dy = x - x0, y - y0
    x_rel = math.cos(yaw0) * dx + math.sin(yaw0) * dy
    y_rel = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
    yaw_rel = math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0))

    tag = data["tag"]
    tag_age = now - tag[0] if tag else None
    tag_valid = tag is not None and tag_age < 0.5

    tof = data["tof"]
    tof_age = now - tof[0] if tof else None
    tof_valid = tof is not None and tof_age < 0.5

    wheel = data["wheel"]
    wheel_age = now - wheel[0] if wheel else None
    wheel_valid = wheel is not None and 0 <= wheel_age < 0.3
    omega = wheel[2] if wheel_valid else None
    wheel_forward = (
        WHEEL_RADIUS_M * sum(omega) / 4.0
        if omega is not None else None
    )

    command = data["cmd"]
    writer.writerow({
        "t_s": now - data["start"],
        "odom_stamp_s": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "x_m": x_rel, "y_m": y_rel, "yaw_rad": yaw_rel,
        "v_odom_mps": msg.twist.twist.linear.x,
        "omega_odom_rps": msg.twist.twist.angular.z,
        "t3_bearing_rad": tag[1] if tag_valid else "",
        "t3_camera_z_m": tag[2] if tag_valid else "",
        "tag_stamp_s": tag[3] if tag_valid else "",
        "tag_age_s": tag_age if tag_valid else "",
        "tof_m": tof[1] if tof_valid else "",
        "tof_age_s": tof_age if tof_valid else "",
        "cmd_v_mps": command[0] if command else "",
        "cmd_omega_rps": command[1] if command else "",
        "wheel_stamp_s": wheel[1] if wheel_valid else "",
        "wheel_age_s": wheel_age if wheel_valid else "",
        "wheel_forward_mps": wheel_forward if wheel_valid else "",
        "omega_fr_rad_s": omega[0] if wheel_valid else "",
        "omega_fl_rad_s": omega[1] if wheel_valid else "",
        "omega_rl_rad_s": omega[2] if wheel_valid else "",
        "omega_rr_rad_s": omega[3] if wheel_valid else "",
    })
    data["rows"] += 1

node.create_subscription(CameraInfo, "/camera/camera_info", on_info, 10)
node.create_subscription(Image, "/camera/image_color", on_image,
                         qos_profile_sensor_data)
node.create_subscription(Range, "/range_0", on_range,
                         qos_profile_sensor_data)
node.create_subscription(JointState, "/joint_states_p", on_wheel,
                         qos_profile_sensor_data)
node.create_subscription(Odometry, "/odom", on_odom,
                         qos_profile_sensor_data)
node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)

print(f"Recording {args.label} for {args.duration:.0f} seconds; recorder sends no motion commands.", flush=True)
try:
    end = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
finally:
    file.close()
    node.destroy_node()
    rclpy.try_shutdown()

print("Saved:", path)
print("Odometry rows:", data["rows"])
print("T3 detection frames:", data["tag_frames"])
