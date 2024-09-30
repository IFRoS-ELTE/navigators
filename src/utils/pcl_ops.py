import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

d2r = np.deg2rad


def get_pointcloud_points(pc: PointCloud2):
    points_raw = list(point_cloud2.read_points(pc))

    return np.array(points_raw)[:, :3]


def assert_point_data(points):
    assert len(points.shape) == 2
    assert points.shape[1] == 3


def filter_points_angle_range(points, min_angle, max_angle):
    assert_point_data(points)

    angles = np.arctan2(points[:, 1], points[:, 0])
    selector = np.logical_and(angles > d2r(min_angle), angles < d2r(max_angle))

    return points[selector]


def filter_points_distance(points, min_distance, max_distance):
    assert_point_data(points)
    distances = np.linalg.norm(points[:, :2], axis=1)
    selector = np.logical_and(distances > min_distance, distances < max_distance)
    return points[selector]


def filter_points_z(points, min_z, max_z):
    z_values = points[:, 2]
    selector = np.logical_and(z_values > min_z, z_values < max_z)
    return points[selector]
