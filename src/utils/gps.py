import os
import pickle
from collections import deque
from dataclasses import dataclass
from typing import List

import numpy as np
import rospy
from rospy import Subscriber
from sensor_msgs.msg import NavSatFix, NavSatStatus
from utils.common import get_debug_folder

EARTH_RADIUS = 6366 * 1e3  # https://rechneronline.de/earth-radius/


d2r = np.deg2rad
r2d = np.rad2deg


@dataclass
class GPSDataPoint:
    ts: float
    lat: float
    lon: float
    alt: float
    xy: np.ndarray
    cov: List[float]

    def to_dict(self):
        return {
            "ts": self.ts,
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "xy": list(self.xy),
            "cov": list(self.cov),
        }

    @staticmethod
    def save_points(points: List["GPSDataPoint"]):
        output_root = os.path.join(get_debug_folder(), "gps_datapoints")
        os.makedirs(output_root, exist_ok=True)
        filename = f"{rospy.Time.now().to_nsec()}.pickle"

        path = os.path.join(output_root, filename)

        with open(path, "wb+") as file:
            pickle.dump(points, file)
            print("SAVED GPS POINTS", filename)


@dataclass
class GPSLocation:
    latitude_rad: float
    longitude_rad: float

    @staticmethod
    def from_lat_lon(lat: float, lon: float, degree_input=True) -> "GPSLocation":
        lat_lon = (lat, lon)
        args = map(d2r, lat_lon) if degree_input else lat_lon
        return GPSLocation(*args)

    @staticmethod
    def from_navsatfix(msg: NavSatFix) -> "GPSLocation":
        return GPSLocation.from_lat_lon(msg.latitude, msg.longitude)

    def __repr__(self):
        return f"<lat={r2d(self.latitude_rad):.6f}, lon={r2d(self.longitude_rad):.6f}>"


class GPSHandler:
    def __init__(self, reference: GPSLocation):
        self.ref = reference
        self.cos = np.cos(self.ref.latitude_rad)

        print(f"Initialised LocationHandler at {self.ref}")

    def get_xy(self, gps: GPSLocation):
        x = EARTH_RADIUS * (gps.longitude_rad - self.ref.longitude_rad) * self.cos
        y = EARTH_RADIUS * (gps.latitude_rad - self.ref.latitude_rad)
        return np.array([x, y])

    def get_gps(self, xy: np.ndarray) -> GPSLocation:
        x, y = xy
        lat_rad = y / EARTH_RADIUS + self.ref.latitude_rad
        lon_rad = x / (EARTH_RADIUS * self.cos) + self.ref.longitude_rad
        return GPSLocation.from_lat_lon(lat_rad, lon_rad, degree_input=False)


class GPSReceiver:
    def __init__(self, topic: str, buffer_size: int = 10, custom_callback=None):
        self.buffer: deque[NavSatFix] = deque(maxlen=buffer_size)
        self.custom_callback = custom_callback
        self.sub = Subscriber(topic, NavSatFix, self.callback)

    def get_latest_fix(self):
        """Return the last NavSatFix received."""
        if not self.buffer:
            return None

        return self.buffer[-1]

    def callback(self, msg: NavSatFix):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            # print("NO GPS FIX (YET)")
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
