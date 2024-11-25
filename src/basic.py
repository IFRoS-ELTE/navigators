#!/usr/bin/python3

import os
from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from rospy import Publisher
from sensor_msgs.msg import NavSatFix
from utils.common import (
    GNSS_TOPIC,
    IMU_MAG_TOPIC,
    TWIST_TOPIC,
    bring_angle_around,
    get_debug_folder,
    limit_velocity,
)
from utils.gps import GPSDataPoint, GPSHandler, GPSLocation, GPSReceiver

# Workflow:
# For a target in GPS coordinates, convert to XY
# Compute desired heading (using current XY)
# Align robot to desired heading
# - Need to know current heading in XY world

MAGNETIC_DECLINATION = np.deg2rad(5.78)

IMU_CORRECTION = np.array([-0.1908, -0.0518, -0.5559])  # Thanks to Zed


def to_0_360(angle_rad):
    return np.rad2deg(bring_angle_around(angle_rad, np.pi))


class VelocityHandler:
    def __init__(self, twist_topic: str, v_max: float, w_max: float):

        self.pub = rospy.Publisher(twist_topic, Twist, queue_size=10)

        self.v_max = v_max
        self.w_max = w_max

    def publish_vel(self, v: float, w: float):
        t = Twist()
        t.linear.x = limit_velocity(v, self.v_max)
        t.angular.z = limit_velocity(w, self.w_max)

        print("Will publish", t.linear.x, t.angular.z)

        self.pub.publish(t)


class Robot:
    def __init__(self, k_linear: float, k_angular: float, goal_threshold_m: float = 5):

        self.k_linear = k_linear
        self.k_angular = k_angular

        self.vel_handler = VelocityHandler(TWIST_TOPIC, 1, 1)

        self.gps_receiver = AvgGPSReceiver(xy_callback_fn=self.xy_callback)

        self.imu_receiver = AvgImuMagReceiver()

        self.goal: GPSLocation = None

        self.xy = None

        self.goal_threshold = goal_threshold_m

        self.control_timer = rospy.Timer(rospy.Duration(0.5), self.control_towards_goal)

    def xy_callback(self, xy: np.ndarray):
        self.xy = xy

    def set_goal(self, new_goal: GPSLocation):
        self.goal = new_goal

    def control_towards_goal(self, t: rospy.timer.TimerEvent):
        if self.goal is None:
            print("No goal available.")
            return

        if self.xy is None:
            print("No location yet.")
            return

        target_xy = self.gps_receiver.handler.get_xy(self.goal)

        position_diff = target_xy - self.xy
        distance = np.linalg.norm(position_diff)
        print(f"D: {distance:.2f}m")

        angle_to_goal = np.arctan2(position_diff[1], position_diff[0])
        current_angle = self.imu_receiver.yaw

        angle_diff = angle_to_goal - current_angle

        if distance < self.goal_threshold:
            # if abs(angle_diff) < np.deg2rad(3):
            print("Goal has been reached.")
            self.goal = None
            return

        angular_vel = self.k_angular * angle_diff
        fwd_vel = 0

        if abs(angle_diff) < np.deg2rad(6):
            fwd_vel = self.k_linear * distance

        self.vel_handler.publish_vel(v=fwd_vel, w=angular_vel)


class AvgGPSReceiver:
    def __init__(self, integration_count=15, xy_callback_fn=None):
        self.wanted_readings = integration_count

        self.xy_callback_fn = xy_callback_fn

        self.gps_datapoints = []
        self.last_save_time = rospy.Time.now()

        self.receiver = GPSReceiver(
            GNSS_TOPIC, custom_callback=self.gps_callback, buffer_size=integration_count
        )
        self.handler: GPSHandler = self.receiver.create_handler(init_sleep_s=3)

    def gps_callback(self, msg: NavSatFix):
        if len(self.receiver.buffer) < self.wanted_readings:
            return

        all_lats = [n.latitude for n in self.receiver.buffer]
        avg_lat = np.mean(all_lats)

        all_longs = [n.longitude for n in self.receiver.buffer]
        avg_long = np.mean(all_longs)

        gps_loc = GPSLocation.from_lat_lon(avg_lat, avg_long, degree_input=True)

        xy = self.handler.get_xy(gps_loc)

        # Save datapoints for later plotting
        datapoint = GPSDataPoint(
            msg.header.stamp.to_nsec(),
            avg_lat,
            avg_long,
            msg.altitude,
            xy,
            msg.position_covariance,
        ).to_dict()
        self.gps_datapoints.append(datapoint)

        if rospy.Time.now() - self.last_save_time > rospy.Duration(10):
            self.last_save_time = rospy.Time.now()
            GPSDataPoint.save_points(self.gps_datapoints)

        if self.xy_callback_fn is not None:
            self.xy_callback_fn(xy)


class AvgImuMagReceiver:
    def __init__(self, integration_count=15):
        self.values: deque[Vector3Stamped] = deque([], maxlen=integration_count)
        self.integration_count = integration_count
        self.yaw = None
        rospy.Subscriber(IMU_MAG_TOPIC, Vector3Stamped, self.mag_callback)

        self.raw_mag = []

        # rospy.Timer(rospy.Duration(10), self.save_raw_mag)

    def save_raw_mag(self, t):
        data = np.array(self.raw_mag)

        output_root = os.path.join(get_debug_folder(), "raw_mag")
        os.makedirs(output_root, exist_ok=True)

        filename = f"{rospy.Time.now().to_nsec()}.txt"

        np.savetxt(os.path.join(output_root, filename), data)
        print("SAVED RAW MAG")

    def mag_callback(self, mag: Vector3Stamped):

        xyz = [mag.vector.x, mag.vector.y, mag.vector.z]

        raw_value = np.array(xyz)
        calibrated_value = raw_value - IMU_CORRECTION

        self.values.append(calibrated_value)
        if len(self.values) < self.integration_count:
            # Not enough values
            return

        self.yaw = self.__compute_yaw_east()

    def __compute_yaw_east(self):

        raw_values = np.array(self.values)[:, :2]

        # Normalize using vector size?
        # amplitudes = np.linalg.norm(raw_values, axis=1).reshape((-1, 1))
        # raw_values = raw_values / amplitudes

        raw_values = np.mean(raw_values, axis=0)

        mean_yaw = np.arctan2(raw_values[1], raw_values[0])

        yaw = -mean_yaw + MAGNETIC_DECLINATION
        yaw = bring_angle_around(yaw)

        # print(f"YAW {to_0_360(yaw):.1f} MEAN YAW: {to_0_360(mean_yaw):.1f}")

        return yaw


if __name__ == "__main__":
    rospy.init_node("basic")

    targets = [
        # (47.4311503, 19.0553650),
        # (47.4772108, 19.1360045),
        # (47.6916700, 19.0783735),
        # (47.4425020, 18.2609145),
        (47.4738730, 19.0580338),
        (47.4740833, 19.0579239),
    ]

    r = Robot(k_linear=1, k_angular=1, goal_threshold_m=4)

    for i, (lat, lon) in enumerate(targets):
        goal = GPSLocation.from_lat_lon(lat, lon, degree_input=True)
        print(f"Setting goal #{i+1}: {(lat, lon)}")
        r.set_goal(goal)

        while r.goal is not None and not rospy.is_shutdown():
            rospy.sleep(1)
            print("Moving...")

    print("All targets visited!")

    rospy.spin()
