from collections import deque
from typing import List

import matplotlib

matplotlib.use("Agg")

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Vector3
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf import transformations
from utils.common import GNSS_TOPIC, ODOM_TOPIC, YAW_EAST_TOPIC, bring_angle_around
from utils.ekf_measurement import (
    CompassMeasurement,
    LocationMeasurement,
    OdomMeasurement,
    combine_measurements,
)
from utils.gps import GPSDataPoint, GPSHandler, GPSLocation, GPSReceiver
from utils.pose import Pose3D
from utils.rviz_publisher import RVizPublisher
from utils.twist_integration import TwistIntegration
from utils.yaw_provider import YawProvider


def publish_odom_transform(yaw):
    br = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "odom"

    t.transform.translation = Vector3(0, 0, 0)
    t.transform.rotation = Quaternion(*transformations.quaternion_from_euler(0, 0, yaw))

    br.sendTransform(t)


class Robot:
    def __init__(self):
        rospy.loginfo("Initialising robot...")

        self.twist_integration = TwistIntegration()

        self.pose_history = deque(maxlen=100)

        self.yaw_provider = YawProvider(integration_count=15)
        self.inital_yaw = self.yaw_provider.get_initial_yaw()

        publish_odom_transform(self.inital_yaw)

        self.odom_origin_pose = Pose3D.from_values(0, 0, self.inital_yaw)

        self.pose: Pose3D = Pose3D.from_values(0, 0, self.inital_yaw)
        self.cov: np.ndarray = np.diag([1, 1, 0.1])

        self.gps_datapoints: List[GPSDataPoint] = []
        self.last_save_time = rospy.Time.now()

        self.location_measurement: LocationMeasurement = None
        self.gps_receiver = GPSReceiver(GNSS_TOPIC, custom_callback=self.gps_callback)
        self.gps_handler: GPSHandler = self.gps_receiver.create_handler(init_sleep_s=3)

        # self.odom_measurement: OdomMeasurement = None
        # self.odom_sub = rospy.Subscriber(ODOM_TOPIC, Odometry, self.odom_callback)

        self.compass_measurement: CompassMeasurement = None
        rospy.Subscriber(YAW_EAST_TOPIC, PoseStamped, self.compass_callback)

        self.rviz_pub = RVizPublisher()
        rospy.Timer(rospy.Duration(0.1), self.publish_pose)

        # Run EKF at the given frequency
        rospy.Timer(rospy.Duration(0.1), self.ekf_step)

        rospy.loginfo(">" * 10 + " Robot init successful!")

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        position = np.array([p.x, p.y]).reshape((2, 1))

        self.odom_measurement = OdomMeasurement(position, self.odom_origin_pose)

    def gps_callback(self, msg: NavSatFix):
        """Convert a NavSatFix to a XY and store the location measurement."""
        if "gps_handler" not in dir(self) or self.gps_handler is None:
            # Handler not initialised yet
            return

        gps_position = GPSLocation.from_lat_lon(msg.latitude, msg.longitude)

        xy = self.gps_handler.get_xy(gps_position)
        self.location_measurement = LocationMeasurement(xy.reshape((2, 1)))

        # Save datapoints for later plotting
        datapoint = GPSDataPoint(
            msg.header.stamp.to_nsec(),
            msg.latitude,
            msg.longitude,
            msg.altitude,
            xy,
            msg.position_covariance,
        ).to_dict()
        self.gps_datapoints.append(datapoint)

        if rospy.Time.now() - self.last_save_time > rospy.Duration(10):
            self.last_save_time = rospy.Time.now()
            GPSDataPoint.save_points(self.gps_datapoints)

    def compass_callback(self, message: PoseStamped):
        """Store compass value."""
        yaw = message.pose.orientation.z
        # print(
        #     f"raw yaw: {np.rad2deg(yaw):.2f}",
        # )
        adjusted_reading = bring_angle_around(yaw, self.pose.theta)
        self.compass_measurement = CompassMeasurement(
            np.array(adjusted_reading).reshape((1, 1))
        )

    def get_measurements(self):
        return [
            self.compass_measurement,
            # self.location_measurement,
            # self.odom_measurement,
        ]

    def ekf_update(self):
        not_used_ms = [
            m for m in self.get_measurements() if m is not None and not m.used
        ]

        if not not_used_ms:
            return self.pose, self.cov

        z, R, H, V, h = combine_measurements(not_used_ms, self.pose)

        P_bar = self.cov

        K_gain = P_bar @ H.T @ np.linalg.inv(H @ P_bar @ H.T + V @ R @ V.T)
        new_pose = self.pose + K_gain @ (z - h)

        I = np.identity((K_gain @ H).shape[0])
        new_cov = (I - K_gain @ H) @ P_bar

        for m in not_used_ms:
            m.used = True

        return new_pose, new_cov

    def ekf_predict(self):
        """EKF prediction step."""

        displacement = self.twist_integration.get_displacement()

        xk_1 = self.pose
        self.pose = xk_1.oplus(displacement)
        self.pose.normalize_theta()

        Ak = xk_1.J_1oplus(displacement)

        Q = np.diag([0.1, 0.1, np.deg2rad(3)])
        W = np.eye(3)
        return self.pose, Ak @ self.cov @ Ak.T + W @ Q @ W.T

    def ekf_step(self, t):
        """Perform the EKF prediction and update."""

        # Prediction from odometry?
        self.pose, self.cov = self.ekf_predict()
        # print("pred pose:", self.pose)

        # Update using measurements
        self.pose, self.cov = self.ekf_update()

        self.pose.normalize_theta()

    def publish_pose(self, t):
        self.rviz_pub.publish_pose_list(self.pose_history)
        self.rviz_pub.publish_pose(self.pose, rospy.Time.now())

        self.pose_history.append(self.pose)
