import numpy as np
import rospy
from common import COMPASS_UNCERTAINTY, GNSS_TOPIC, IMU_TOPIC, get_yaw_from_imu
from gps import GPSHandler, GPSReceiver
from pose import Pose3D
from sensor_msgs.msg import Imu
from sensors import get_initial_compass_reading


class Robot:
    def __init__(self):
        self.initial_compass = get_initial_compass_reading()
        self.pose: Pose3D = Pose3D.from_values(0, 0, self.initial_compass)
        self.cov: np.ndarray = np.diagonal([0.05, 0.05, 0.1])

        self.gps_receiver = GPSReceiver(GNSS_TOPIC)
        self.gps_handler = self.gps_receiver.create_handler(init_sleep_s=3)

        self.compass_reading = None
        rospy.Subscriber(IMU_TOPIC, Imu, self.compass_cb)

        # Run EKF at the given frequency
        rospy.Timer(0.1, self.ekf_step)

    def compute_compass_update(self):
        """EKF computation corresponding to compass update."""
        assert self.compass_reading is not None

        H = np.array([0, 0, 1]).reshape((1, 3))
        V = np.array([1]).reshape((1, 1))

        P_bar = self.cov
        R = COMPASS_UNCERTAINTY

        K_gain = P_bar @ H.T @ np.linalg.inv(H @ P_bar @ H.T + V * R * V.T)
        new_pose = self.pose + K_gain @ (self.compass_reading - H @ self.pose)

        I = np.identity((K_gain @ H).shape[0])
        new_cov = (I - K_gain @ H) @ P_bar @ (I - K_gain @ H).T
        return new_pose, new_cov

    def update_compass(self):
        """Perform the EKF update step for the compass."""
        if self.compass_reading is None:
            return

        self.pose, self.cov = self.compute_compass_update()
        self.compass_reading = None  # to avoid reusing a reading

    def compass_callback(self, message: Imu):
        """Store compass value."""
        self.compass_reading = get_yaw_from_imu(message)

    def ekf_step(self):
        """Perform the EKF prediction and update."""

        # Prediction from odometry?

        # Update compass
        self.update_compass()
