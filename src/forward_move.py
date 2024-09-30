#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from rospy import Publisher, Subscriber
from sensor_msgs.msg import PointCloud2
from utils import pcl_ops

# - rosrun scout_bringup setup_can2usb.bash

# Running on the robot:
# - roscore
# - roslaunch rslidar_pointcloud rs_lidar_16.launch
# - rosrun scout_base scout_base_node


class Controller:
    def __init__(self):
        self.free_space_in_front = False
        self.last_check_time = rospy.Time.now()
        self.pub = Publisher("/cmd_vel", Twist, queue_size=0)
        Subscriber("/velodyne_points", PointCloud2, self.pointcloud_callback)

    def pointcloud_callback(self, pc: PointCloud2):
        if rospy.Time.now() - self.last_check_time < rospy.Duration(0.1):
            print("skip")
            return
        points = pcl_ops.get_pointcloud_points(pc)

        points = pcl_ops.filter_points_z(points, -0.5, 0.2)
        points = pcl_ops.filter_points_angle_range(points, -30, 30)
        points = pcl_ops.filter_points_distance(points, 0, 1)
        print(f"Obstacle points:{len(points)}")
        self.free_space_in_front = len(points) == 0
        self.last_check_time = rospy.Time.now()

    def run(self):
        while not rospy.is_shutdown():
            t = Twist()
            sleep = 5

            check_age = rospy.Time.now() - self.last_check_time

            if self.free_space_in_front and check_age < rospy.Duration(1):
                t.linear.x = 0.1
                sleep = 0.1
                state = "Moving"
            else:
                t.linear.x = 0
                t.angular.z = 0.2
                sleep = 0.2
                state = "Rotating"

            print(state)
            self.pub.publish(t)
            rospy.sleep(sleep)


if __name__ == "__main__":
    rospy.init_node("navigation")

    Controller().run()
