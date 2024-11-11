#!/usr/bin/python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from rospy import Publisher, Subscriber
from utils import common
from utils.robot import Robot


class Controller:
    def __init__(self, robot: Robot, k_linear=1.0, k_angular=1.0):
        self.robot = robot

        self.k_linear = k_linear  # Proportional gain for linear velocity
        self.k_angular = k_angular  # Proportional gain for angular velocity

        self.v_max = 0.3  # Maximum linear velocity
        self.w_max = 0.3  # Maximum angular velocity

        # Current robot pose [x, y, yaw], None if unknown
        self.current_pose = None

        # Goal where the robot has to go, None if not set
        self.goal = None

        # Publishers
        self.pub = Publisher("/cmd_vel", Twist, queue_size=0)

        # Subscribers
        self.goal_sub = Subscriber("/move_base_simple/goal", PoseStamped, self.get_goal)

        self.current_pose = self.robot.pose

    def get_goal(self, goal):
        self.goal = np.array([goal.pose.position.x, goal.pose.position.y])
        self.goal_orientation = common.yaw_from_quaternion(goal.pose.orientation)
        rospy.loginfo(f"Goal is {self.goal}")
        rospy.loginfo(f"Robot is at {self.current_pose}")

    def compute_velocities(self):
        if self.goal is None:
            return 0, 0

        # Robot's current position and orientation (in radians)
        x_r = self.robot.pose.x
        y_r = self.robot.pose.y
        theta_r = self.robot.pose.theta

        x_g, y_g = self.goal  # Goal position

        # Calculate the distance to the goal
        dx = x_g - x_r
        dy = y_g - y_r
        distance_to_goal = np.sqrt(dx**2 + dy**2)

        # Calculate the angle to the goal
        angle_to_goal = np.arctan2(dy, dx)

        # Calculate the angle difference
        angle_diff = angle_to_goal - theta_r

        # Normalize the angle difference to the range [-pi, pi]
        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

        if abs(angle_diff) > np.deg2rad(5):
            linear_velocity = 0
        else:
            linear_velocity = self.k_linear * distance_to_goal

        angular_velocity = self.k_angular * angle_diff

        return linear_velocity, angular_velocity

    def publish_velocities(self):
        v, w = self.compute_velocities()

        t = Twist()
        t.linear.x = common.limit_velocity(v, self.v_max)
        t.angular.z = common.limit_velocity(w, self.w_max)

        print("Will publish", t.linear.x, t.angular.z)

        self.pub.publish(t)


if __name__ == "__main__":
    rospy.init_node("controller_node")
    robot = Robot()

    controller = Controller(robot)

    while not rospy.is_shutdown():
        controller.publish_velocities()
        rospy.sleep(0.5)

    # Run forever
    rospy.spin()
