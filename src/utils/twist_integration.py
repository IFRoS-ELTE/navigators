import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from utils.common import ODOM_TOPIC


class TwistIntegration:
    def __init__(self):
        self.previous_reading = None

        self.displacement = np.zeros((3, 1))
        self.current_integration_count = 0

        rospy.Subscriber(ODOM_TOPIC, Odometry, self.odom_callback)

    def compute_displacement(self, twist: Twist, dt: float):
        return np.array([[twist.linear.x, 0, twist.angular.z]], dtype=np.float64).T * dt

    def odom_callback(self, odom: Odometry):
        if self.previous_reading is None:
            self.previous_reading = odom
            return

        dt = (odom.header.stamp - self.previous_reading.header.stamp).to_sec()

        local_displacement = self.compute_displacement(odom.twist.twist, dt)
        self.displacement += local_displacement

        self.current_integration_count += 1

        if self.current_integration_count > 1000:
            print("Warning: twist accumulated > 1000 readings.")

        # if self.current_integration_count >= self.target_integration_count:
        #     self.callback_fn(self.displacement)
        #     self.displacement = np.array([[0, 0, 0]]).T
        #     self.current_integration_count = 0

    def get_displacement(self):
        displacement = np.copy(self.displacement)
        self.displacement = np.zeros((3, 1))

        print(f"getting displacement from {self.current_integration_count} values.")
        self.current_integration_count = 0

        return displacement
