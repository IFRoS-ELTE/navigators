#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from rospy import Publisher
from utils.robot import Robot

if __name__ == "__main__":
    rospy.init_node("gps_navigation")

    r = Robot()
    rospy.spin()
