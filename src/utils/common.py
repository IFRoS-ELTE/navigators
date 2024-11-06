import numpy as np
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

IMU_TOPIC = "/replace_me"
COMPASS_UNCERTAINTY = np.deg2rad(5)
GPS_UNCERTAINTY = [10, 10]
GNSS_TOPIC = "/gnss"
POSE_TOPIC = "/pose"

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


def yaw_from_quaternion(q: Quaternion):
    values = [q.x, q.y, q.z, q.w]
    return euler_from_quaternion(values)[2]
