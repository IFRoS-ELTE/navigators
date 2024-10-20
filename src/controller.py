import numpy as np

class Controller:
    def __init__(self, k_linear=1.0, k_angular=1.0):
        self.k_linear = k_linear  # Proportional gain for linear velocity
        self.k_angular = k_angular  # Proportional gain for angular velocity

    def compute_velocities(self, robot_position, goal_position):
        x_r, y_r, theta_r = robot_position  # Robot's current position and orientation (in radians)
        x_g, y_g = goal_position  # Goal position

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

        # Calculate linear and angular velocities
        linear_velocity = self.k_linear * distance_to_goal
        angular_velocity = self.k_angular * angle_diff

        return linear_velocity, angular_velocity


robot_controller = Controller(k_linear=1.0, k_angular=2.0)

# Current position of the robot (x, y, theta)
robot_position = (1.0, 1.0, np.pi / 4)  # x_r, y_r, theta_r (45 degrees)

# Goal position (x, y)
goal_position = (4.0, 5.0)  # x_g, y_g

# Compute velocities
linear_velocity, angular_velocity = robot_controller.compute_velocities(robot_position, goal_position)

print("Linear Velocity:", linear_velocity)
print("Angular Velocity:", angular_velocity)
