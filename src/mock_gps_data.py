#!/usr/bin/python3
import numpy as np
import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from utils.common import GNSS_TOPIC
from utils.gps import GPSHandler, GPSLocation

r2d = np.rad2deg

LOCATIONS = [
    (0, 0),
    (1, 0),
    (2, 0),
    (3, 0),
    (4, 0),
]


def create_navsatfix(gps_location: GPSLocation):
    msg = NavSatFix()
    msg.header.stamp = rospy.Time.now()
    msg.status.status = NavSatStatus.STATUS_FIX
    msg.latitude = r2d(gps_location.latitude_rad)
    msg.longitude = r2d(gps_location.longitude_rad)
    return msg


class GPSMock:
    def __init__(self, ref):
        self.ref = ref
        self.handler = GPSHandler(ref)
        self.publisher = rospy.Publisher(GNSS_TOPIC, NavSatFix, queue_size=1)

    def publish_navsatfix(self, xy):
        gps_location = self.handler.get_gps(xy)

        msg = create_navsatfix(gps_location)
        self.publisher.publish(msg)
        rospy.logdebug("Published mock NavSatFix.")

    def run(self):
        forth_and_back = LOCATIONS + list(reversed(LOCATIONS))

        should_run = True
        while should_run:
            for location in forth_and_back:
                for _ in range(100):
                    self.publish_navsatfix(location)
                    rospy.sleep(0.1)

                    if rospy.is_shutdown():
                        should_run = False
                        break


def main():
    rospy.init_node("mock_gps")
    ref = GPSLocation.from_lat_lon(47.473820, 19.057358)

    mock = GPSMock(ref)
    mock.run()


if __name__ == "__main__":
    main()
