from collections import deque
from dataclasses import dataclass

import numpy as np
import rospy
from rospy import Subscriber
from sensor_msgs.msg import NavSatFix

EARTH_RADIUS = 6366 * 1e3  # https://rechneronline.de/earth-radius/


d2r = np.deg2rad
r2d = np.rad2deg


@dataclass
class GPSLocation:
    latitude: float
    longitude: float

    @staticmethod
    def from_lat_lon(lat: float, lon: float, degree_input=True) -> "GPSLocation":
        lat_lon = (lat, lon)
        args = map(d2r, lat_lon) if degree_input else lat_lon
        return GPSLocation(*args)

    @staticmethod
    def from_navsatfix(msg: NavSatFix) -> "GPSLocation":
        return GPSLocation.from_lat_lon(msg.latitude, msg.longitude)

    def __repr__(self):
        return f"<lat={r2d(self.latitude):.6f}, lon={r2d(self.longitude):.6f}>"


class GPSHandler:
    def __init__(self, reference: GPSLocation):
        self.ref = reference
        self.cos = np.cos(self.ref.latitude)

        print(f"Initialised LocationHandler at {self.ref}")

    def get_xy(self, gps: GPSLocation):
        x = EARTH_RADIUS * (gps.longitude - self.ref.longitude) * self.cos
        y = EARTH_RADIUS * (gps.latitude - self.ref.latitude)
        return np.array([x, y])

    def get_gps(self, xy: np.ndarray) -> GPSLocation:
        x, y = xy
        lat_rad = y / EARTH_RADIUS + self.ref.latitude
        lon_rad = x / (EARTH_RADIUS * self.cos) + self.ref.longitude
        return GPSLocation.from_lat_lon(lat_rad, lon_rad, degree_input=False)


class GPSReceiver:
    def __init__(self, topic: str, buffer_size: int = 10, custom_callback=None):
        self.buffer = deque(maxlen=buffer_size)
        self.custom_callback = custom_callback
        self.sub = Subscriber(topic, NavSatFix, self.callback)

    def get_latest_fix(self):
        """Return the last NavSatFix received."""
        if not self.buffer:
            return None

        return self.buffer[-1]

    def callback(self, msg: NavSatFix):
        if msg.status.status == msg.STATUS_NO_FIX:
            print("NO GPS FIX (YET)")
            return

        self.buffer.append(msg)
        if self.custom_callback is not None:
            self.custom_callback(msg)

    def create_handler(self, init_sleep_s=0) -> GPSHandler:
        """Create a GPSHandler based on the location of the latest fix, after sleeping for `init_sleep_s`"""
        rospy.loginfo("Creating GPSHandler...")
        rospy.sleep(init_sleep_s)
        latest_fix = self.get_latest_fix()
        if not latest_fix:
            rospy.logwarn("No GPS fix available.")
            return None
        return GPSHandler(GPSLocation.from_navsatfix(latest_fix))


if __name__ == "__main__":
    locations = [
        [47.477, 19.055353],
        [47.476906, 19.062326],
        [47.470604, 19.056640],
        [47.637154, 19.060938],
        [47.473663, 19.059029],
    ]

    ref = GPSLocation.from_lat_lon(47.473820, 19.057358)
    handler = GPSHandler(ref)
    # ref_xy = handler.get_xy(ref)

    # for loc in locations:
    #     gps_location = GPSLocation.from_lat_lon(*loc)
    #     xy = handler.get_xy(gps_location)
    #     print(xy)

    print(handler.get_gps(np.array([0, 0])))
    print(handler.get_gps(np.array([100, 0])))  # Towards East
    print(handler.get_gps(np.array([0, 100])))  # Towards North
