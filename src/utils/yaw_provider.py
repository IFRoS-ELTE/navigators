from typing import List

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from utils.common import IMU_MAG_TOPIC, YAW_EAST_TOPIC, get_yaw_from_vector3

MAGNETIC_DECLINATION = np.deg2rad(5.78)


class YawProvider:
    def __init__(self, integration_count):
        self.integration_count = integration_count
        self.values: List[Vector3Stamped] = []

        self.pub = rospy.Publisher(YAW_EAST_TOPIC, PoseStamped)
        self.initial_yaw = None
        self.current_yaw = None

        rospy.Subscriber(IMU_MAG_TOPIC, Vector3Stamped, self.mag_callback)

    def __compute_yaw_east(self):
        raw_values = [get_yaw_from_vector3(v.vector) for v in self.values]

        mean_yaw = np.mean(raw_values)
        return mean_yaw + MAGNETIC_DECLINATION + np.pi / 2

    def mag_callback(self, mag: Vector3Stamped):
        self.values.append(mag)

        if len(self.values) < self.integration_count:
            # Not enough values
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.values[-1].header.stamp
        yaw = self.__compute_yaw_east()
        pose_msg.pose.orientation.z = yaw

        self.current_yaw = yaw
        if self.initial_yaw is None:
            self.initial_yaw = yaw

        self.pub.publish(pose_msg)

        # Reset buffer
        self.values = []

    def get_initial_yaw(self, timeout_s=3):
        print("Getting initial yaw...")
        rospy.sleep(timeout_s)
        if self.initial_yaw is not None:
            return self.initial_yaw

        rospy.logwarn("NO INITIAL YAW!")
        return 0
