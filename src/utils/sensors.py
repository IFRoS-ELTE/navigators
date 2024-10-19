import numpy as np
import rospy
from common import COMPASS_TOPIC, GNSS_TOPIC
from gps import GPSHandler, GPSLocation
from sensor_msgs.msg import NavSatFix, NavSatStatus


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


def init_gps_handler(sleep_secs=3):
    gps_fix: NavSatFix = None

    def gnss_cb(msg: NavSatFix):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            # Ignore no-fix messages
            return

        global gps_fix
        gps_fix = msg

    sub = rospy.Subscriber(GNSS_TOPIC, NavSatFix, gnss_cb)
    rospy.sleep(sleep_secs)
    sub.unregister()

    if not gps_fix:
        print(f"WARNING: Unable to get GPS fix after {sleep_secs}s")
        return None

    return GPSHandler(GPSLocation.from_navsatfix(gps_fix))
