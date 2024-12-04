#!/usr/bin/python3

import os
from collections import deque

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3Stamped
from rospy import Publisher
from sensor_msgs.msg import CompressedImage, NavSatFix, PointCloud2
from std_msgs.msg import String
from utils import pcl_ops
from utils.common import (
    DEG,
    GNSS_TOPIC,
    IMU_MAG_TOPIC,
    TWIST_TOPIC,
    bring_angle_around,
    get_debug_folder,
    limit_velocity,
)
from utils.gps import GPSDataPoint, GPSHandler, GPSLocation, GPSReceiver
from utils.pose import Pose3D
from utils.rviz_publisher import RVizPublisher

# Workflow:
# For a target in GPS coordinates, convert to XY
# Compute desired heading (using current XY)
# Align robot to desired heading
# - Need to know current heading in XY world


# roslaunch realsense2_camera rs_camera.launch


MAGNETIC_DECLINATION = np.deg2rad(5.78)
POMONA = "pomona"
SILVANUS = "silvanus"

# MODE = POMONA
MODE = SILVANUS

if MODE == POMONA:
    # For Pomona
    IMU_CORRECTION_T = np.array([-0.1908, -0.0518, -0.5559])  # Thanks to Zed
    IMU_CORRECTION_A = np.eye(3)  # Thanks to Zed
    ANGLE_CORRECTION = 0
    W_MAX = 0.5
else:
    # For Silvanus
    IMU_CORRECTION_T = np.array([0.0243, -0.3730, -1.1995])
    IMU_CORRECTION_A = np.array(
        [
            [0.8965, 0.0139, 0.1370],
            [0.0139, 1.5075, 0.3872],
            [0.1370, 0.3872, 0.8593],
        ]
    )
    ANGLE_CORRECTION = np.pi / 2
    W_MAX = 0.3


def correct_imu(raw):
    return (
        (np.array(raw).reshape(1, 3) - np.array(IMU_CORRECTION_T).reshape(1, 3))
        @ IMU_CORRECTION_A
    ).squeeze()


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
    def __init__(self, k_linear: float, k_angular: float, goal_threshold_m: float):

        self.k_linear = k_linear
        self.k_angular = k_angular

        self.vel_handler = VelocityHandler(
            twist_topic=TWIST_TOPIC, v_max=1, w_max=W_MAX
        )

        self.gps_receiver = AvgGPSReceiver(xy_callback_fn=self.xy_callback)

        self.imu_receiver = AvgImuMagReceiver()

        self.goal: GPSLocation = None

        self.pose_history = deque([], maxlen=100)

        self.xy = None

        self.goal_threshold = goal_threshold_m

        self.rviz_pub = RVizPublisher()

        self.obstacle_pts = 0

        self.last_pcl_check = rospy.Time.now()

        self.img_folder = os.path.join(get_debug_folder(), "imgs")
        os.makedirs(self.img_folder, exist_ok=True)

        if MODE == POMONA:
            rospy.Subscriber("/velodyne_points", PointCloud2, self.pointcloud_callback)

        rospy.Subscriber("/sent_image", CompressedImage, self.get_img_callback)
        self.send_image_pub = Publisher("/send_image", String, queue_size=1)

        self.control_timer = rospy.Timer(rospy.Duration(0.5), self.control_towards_goal)

    def get_img_callback(self, img: CompressedImage):
        np_arr = np.frombuffer(img.data, np.uint8)
        received_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        filename = f"{int(rospy.Time.now().to_sec())}_{goal}.jpg"
        cv2.imwrite(os.path.join(self.img_folder, filename), received_img)
        print("Saved image to", filename)

    def pointcloud_callback(self, pc: PointCloud2):
        if rospy.Time.now() - self.last_pcl_check < rospy.Duration(0.1):
            return
        pts = pcl_ops.get_pointcloud_points(pc)

        pts = pcl_ops.filter_points_z(points=pts, min_z=-0.5, max_z=0.2)
        pts = pcl_ops.filter_points_angle_range(points=pts, min_angle=-30, max_angle=30)
        pts = pcl_ops.filter_points_distance(points=pts, min_distance=0, max_distance=1)

        self.obstacle_pts = len(pts)
        self.last_pcl_check = rospy.Time.now()

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

        if self.obstacle_pts > 0:
            print("OBSTACLE: ", self.obstacle_pts)
            self.vel_handler.publish_vel(v=0, w=0)
            return

        goal_xy = self.gps_receiver.handler.get_xy(self.goal)

        self.rviz_pub.publish_goal_xy(goal_xy)

        position_diff = goal_xy - self.xy
        distance = np.linalg.norm(position_diff)

        angle_to_goal = np.arctan2(position_diff[1], position_diff[0])
        current_angle = self.imu_receiver.yaw

        pose = Pose3D.from_values(*self.xy, current_angle)
        self.rviz_pub.publish_pose(pose, rospy.Time.now())

        self.pose_history.append(pose)
        self.rviz_pub.publish_pose_list(self.pose_history)

        angle_diff = angle_to_goal - current_angle
        angle_diff = bring_angle_around(angle_diff)

        print(f"D: {distance:.1f}m Angle: {np.rad2deg(angle_diff):.1f}{DEG}")

        if distance < self.goal_threshold:
            # if abs(angle_diff) < np.deg2rad(3):
            print("Goal has been reached.")
            self.send_image_pub.publish("I want an image NOW!!")
            self.goal = None
            return

        angular_vel = self.k_angular * angle_diff
        fwd_vel = 0.3

        if abs(angle_diff) < np.deg2rad(10):
            fwd_vel = self.k_linear * distance

        self.vel_handler.publish_vel(v=fwd_vel, w=angular_vel)


class AvgGPSReceiver:
    def __init__(self, integration_count=15, xy_callback_fn=None):
        self.wanted_readings = integration_count

        self.xy_callback_fn = xy_callback_fn

        self.save_gps_points = False
        self.gps_datapoints = []
        self.last_save_time = rospy.Time.now()

        self.receiver = GPSReceiver(
            GNSS_TOPIC, custom_callback=self.gps_callback, buffer_size=integration_count
        )

    def gps_callback(self, msg: NavSatFix):
        if len(self.receiver.buffer) < self.wanted_readings:
            return

        if "handler" not in dir(self) or self.handler is None:
            print("NO GPS handler")
            self.handler: GPSHandler = self.receiver.create_handler(init_sleep_s=3)
            return

        all_lats = [n.latitude for n in self.receiver.buffer]
        avg_lat = np.mean(all_lats)

        all_longs = [n.longitude for n in self.receiver.buffer]
        avg_long = np.mean(all_longs)

        gps_loc = GPSLocation.from_lat_lon(avg_lat, avg_long, degree_input=True)

        xy = self.handler.get_xy(gps_loc)

        if self.save_gps_points:
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
        self.yaw = 0
        rospy.Subscriber(IMU_MAG_TOPIC, Vector3Stamped, self.mag_callback)

        self.save_raw_mag = False
        self.raw_mag = []

        if self.save_raw_mag:
            rospy.Timer(rospy.Duration(10), self.__save_raw_mag)

    def __save_raw_mag(self, t):
        data = np.array(self.raw_mag)

        output_root = os.path.join(get_debug_folder(), "raw_mag")
        os.makedirs(output_root, exist_ok=True)

        filename = f"{rospy.Time.now().to_nsec()}.txt"

        np.savetxt(os.path.join(output_root, filename), data)
        print("SAVED RAW MAG", filename)

    def mag_callback(self, mag: Vector3Stamped):
        # print("Time diff:", (rospy.Time.now() - mag.header.stamp).to_sec())

        xyz = [mag.vector.x, mag.vector.y, mag.vector.z]
        if self.save_raw_mag:
            self.raw_mag.append(xyz)

        raw_value = np.array(xyz)
        calibrated_value = correct_imu(raw_value)

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

        yaw = -mean_yaw + MAGNETIC_DECLINATION + ANGLE_CORRECTION
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
        (47.4738730, 19.0580338),  # Near fence
        (47.4740833, 19.0579239),  # Near entrance
        # (47.473820, 19.057358)  # mock
    ]

    targets = [
        (47.4739076, 19.0579875),  # Near fence...
        (47.4740616, 19.0579337),  # Near entrance
    ] * 3

    # Green area
    # targets = [
    #     (47.473795, 19.061668),
    #     (47.473586, 19.061892),
    #     (47.473659, 19.062236),
    #     (47.473803, 19.062184),
    # ]

    # targets = [
    #     (47.47378759, 19.0619692),
    #     (47.473641, 19.0620519),
    #     (47.4736415, 19.0622771),
    #     (47.47379245, 19.0621895),
    # ] * 10

    # targets = [
    #     (47.4736596, 19.0619257),
    #     (47.4738172, 19.0619063),
    #     (47.4736301, 19.06197079),
    #     (47.4737228, 19.062226),
    # ] * 10

    r = Robot(k_linear=1, k_angular=1, goal_threshold_m=3)

    for i, (lat, lon) in enumerate(targets):
        goal = GPSLocation.from_lat_lon(lat, lon, degree_input=True)
        print(f"Setting goal #{i+1}: {(lat, lon)}")
        r.set_goal(goal)

        while r.goal is not None and not rospy.is_shutdown():
            rospy.sleep(1)

    print("All targets visited!")

    rospy.spin()
