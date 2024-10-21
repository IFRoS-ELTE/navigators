import numpy as np
import rospy
from sensor_msgs.msg import Imu
from utils.common import IMU_TOPIC, get_yaw_from_imu


def get_initial_compass_reading(timeout_s=3, wanted_readings=10):
    rospy.loginfo("Getting initial compass reading...")
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

    if not compass_readings:
        rospy.logwarn("No compass readings received.")
        return 0

    return np.mean(compass_readings)
