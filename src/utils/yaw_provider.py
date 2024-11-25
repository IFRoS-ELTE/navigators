from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from utils.common import (
    IMU_DATA_TOPIC,
    IMU_MAG_TOPIC,
    YAW_EAST_TOPIC,
    get_yaw_from_imu,
    get_yaw_from_vector3,
)

MAGNETIC_DECLINATION = np.deg2rad(5.78)


class YawProvider:
    def __init__(self, integration_count):
        self.integration_count = integration_count
        self.values: deque[Vector3Stamped] = deque([], maxlen=20)

        self.pub = rospy.Publisher(YAW_EAST_TOPIC, PoseStamped, queue_size=10)
        self.initial_yaw = None
        self.current_yaw = None

        # rospy.Subscriber(IMU_MAG_TOPIC, Vector3Stamped, self.mag_callback)
        rospy.Subscriber(IMU_DATA_TOPIC, Imu, self.imu_callback)

        self.x_acc_pub = rospy.Publisher("/lin_acc/x", Float32, queue_size=1)
        self.y_acc_pub = rospy.Publisher("/lin_acc/y", Float32, queue_size=1)

    def __compute_yaw_east(self):
        # raw_values = [get_yaw_from_vector3(v.vector) for v in self.values]

        raw_values = [(v.vector.x, v.vector.y) for v in self.values]
        raw_values = np.array(raw_values)

        amplitudes = np.linalg.norm(raw_values, axis=1).reshape((-1, 1))
        raw_values = raw_values / amplitudes

        raw_values = np.mean(raw_values, axis=0)

        # return mean_yaw  # + MAGNETIC_DECLINATION + np.pi / 2
        yaw = np.arctan2(raw_values[1], raw_values[0])
        # print(yaw)

        return yaw

    def imu_callback(self, imu: Imu):
        yaw = get_yaw_from_imu(imu)

        print(f"YAW: {np.rad2deg(yaw):.3f}")

        self.x_acc_pub.publish(imu.linear_acceleration.x)
        self.y_acc_pub.publish(imu.linear_acceleration.y)

        self.current_yaw = yaw
        if self.initial_yaw is None:
            self.initial_yaw = yaw

        pose_msg = PoseStamped()
        pose_msg.header.stamp = imu.header.stamp
        pose_msg.pose.orientation.z = yaw

        self.pub.publish(pose_msg)

    def mag_callback(self, mag: Vector3Stamped):

        yaw = get_yaw_from_vector3(mag.vector)

        # self.values.append(mag)

        # if len(self.values) < self.integration_count:
        #     # Not enough values
        #     return

        pose_msg = PoseStamped()
        # pose_msg.header.stamp = self.values[-1].header.stamp
        pose_msg.header.stamp = mag.header.stamp
        # yaw = self.__compute_yaw_east()
        print(f"YAW: {np.rad2deg(yaw):.3f}")
        pose_msg.pose.orientation.z = yaw

        self.current_yaw = yaw
        if self.initial_yaw is None:
            self.initial_yaw = yaw

        self.pub.publish(pose_msg)

    def get_initial_yaw(self, timeout_s=3):
        print("Getting initial yaw...")
        rospy.sleep(timeout_s)
        if self.initial_yaw is not None:
            return self.initial_yaw

        rospy.logwarn("NO INITIAL YAW!")
        return 0
