import numpy as np
import rospy
from common import GNSS_TOPIC
from gps import GPSHandler, GPSReceiver
from pose import Pose3D
from sensors import get_initial_compass_reading


class Robot:
    def __init__(self):
        self.initial_compass = get_initial_compass_reading()
        self.pose: Pose3D = Pose3D.from_values(0, 0, self.initial_compass)

        self.gps_receiver = GPSReceiver(GNSS_TOPIC)
        self.gps_handler = self.gps_receiver.create_handler(init_sleep_s=3)
