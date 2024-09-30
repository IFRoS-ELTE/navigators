#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from rospy import Publisher

if __name__ == "__main__":
    rospy.init_node("navigation")

    p = Publisher("/cmd_vel", Twist, queue_size=0)

    i = 0
    while not rospy.is_shutdown():
        t = Twist()
        sleep = 5

        if i % 2:
            t.linear.x = 0.1
            sleep = 1
            state = "Moving"
        else:
            t.linear.x = 0
            sleep = 3
            state = "Waiting"

        p.publish(t)
        print(state)
        rospy.sleep(sleep)

        i += 1
