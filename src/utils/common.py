import numpy as np

COMPASS_TOPIC = "/replace_me"
GNSS_TOPIC = "/gnss"

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
