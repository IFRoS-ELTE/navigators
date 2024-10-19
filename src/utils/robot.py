import numpy as np
import rospy
from common import GNSS_TOPIC
from gps import GPSHandler
from pose import Pose3D
from rospy import Subscriber
from sensor_msgs.msg import NavSatFix, NavSatStatus
from sensors import get_initial_compass_reading, init_gps_handler


class Robot:
    def __init__(self):
        self.initial_compass = get_initial_compass_reading()
        self.pose: Pose3D = Pose3D.from_values(0, 0, self.initial_compass)

        self.gps_handler = init_gps_handler()

        Subscriber(GNSS_TOPIC, NavSatFix, self.gnss_callback)

    def gnss_callback(self, gnss_msg: NavSatFix):
        good_fix = gnss_msg.status.status != NavSatStatus.STATUS_NO_FIX
        if not good_fix:
            print("NO GPS FIX")
            return

        print("Got GPS fix:", gnss_msg)
