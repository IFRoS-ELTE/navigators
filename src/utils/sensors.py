import numpy as np
import rospy
from common import IMU_TOPIC, get_yaw_from_imu
from sensor_msgs.msg import Imu


def get_initial_compass_reading(timeout_s=3, wanted_readings=10):
    compass_readings = []

    def compass_cb(msg: Imu):
        compass_readings.append(get_yaw_from_imu(msg))

    sub = rospy.Subscriber(IMU_TOPIC, Imu, compass_cb)

    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(secs=timeout_s):
        # Accumulate readings until timeout or wanted_readings have been collected
        if len(compass_readings) >= wanted_readings:
            break

    sub.unregister()

    return np.mean(compass_readings)
