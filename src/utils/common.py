import os

import numpy as np
import rospkg
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

PACKAGE_NAME = "navigators"

COMPASS_UNCERTAINTY = np.deg2rad(5)
GPS_UNCERTAINTY = [10, 10]

GNSS_TOPIC = "/gnss"
IMU_DATA_TOPIC = "/imu/data"
IMU_MAG_TOPIC = "/imu/mag"
POSE_TOPIC = "/pose"
ODOM_TOPIC = "/odom"
YAW_EAST_TOPIC = "/yaw_east"
TWIST_TOPIC = "/cmd_vel"
IMG_TOPIC = "/camera/color/image_raw/compressed"


MAP_FRAME = "map"

DEG = "°"


def rotation_z(angle):
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def translation(x=0, y=0, z=0):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


def bring_angle_around(angle: float, value: float = 0):
    """Add/subtract 2*pi as necessary, to bring `angle` within +/-pi from `value`."""

    while angle > value + np.pi:
        angle -= 2 * np.pi

    while angle < value - np.pi:
        angle += 2 * np.pi

    return angle


def get_yaw_from_imu(imu: Imu):
    """Extract yaw from the Imu message."""
    return yaw_from_quaternion(imu.orientation)


def get_yaw_from_vector3(vector: Vector3):
    """Extract yaw from the Vector value."""
    return np.arctan2(vector.y, vector.x)


def yaw_from_quaternion(q: Quaternion):
    values = [q.x, q.y, q.z, q.w]
    return euler_from_quaternion(values)[2]


def limit_velocity(vel, absolute_limit):
    if vel < -absolute_limit:
        return -absolute_limit

    if vel > absolute_limit:
        return absolute_limit

    return vel


def get_package_path():
    return rospkg.RosPack().get_path(PACKAGE_NAME)


def get_debug_folder():
    debug_folder = os.path.join(get_package_path(), "debug")
    os.makedirs(debug_folder, exist_ok=True)
    return debug_folder
