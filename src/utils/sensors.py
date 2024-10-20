import numpy as np
import rospy
from common import COMPASS_TOPIC


def get_initial_compass_reading(timeout_s=3, wanted_readings=10):
    compass_readings = []

    def compass_cb(compass_msg):
        # TODO Extract compass reading from compass msg
        compass_readings.append(compass_msg.reading)

    sub = rospy.Subscriber(COMPASS_TOPIC, "compass-data-type-goes-here", compass_cb)

    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(secs=timeout_s):
        # Accumulate readings until timeout or wanted_readings have been collected
        if len(compass_readings) >= wanted_readings:
            break

    sub.unregister()

    return np.mean(compass_readings)
