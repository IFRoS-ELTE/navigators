from typing import List

import numpy as np
import rospy
import tf
from geometry_msgs.msg import (
    Point,
    Pose,
    Pose2D,
    PoseStamped,
    Quaternion,
    TransformStamped,
    Vector3,
)
from std_msgs.msg import ColorRGBA
from utils.common import MAP_FRAME, POSE_TOPIC
from utils.pose import Pose3D
from visualization_msgs.msg import Marker, MarkerArray


def header_map_frame_now():
    return rospy.Header(stamp=rospy.Time.now(), frame_id=MAP_FRAME)


class RVizPublisher:
    def __init__(self):
        self.br = tf.TransformBroadcaster()

        self.pose_pub = rospy.Publisher(POSE_TOPIC, PoseStamped, queue_size=1)

        self.pose_history_pub = rospy.Publisher(
            "pose_history", MarkerArray, queue_size=1
        )

        self.goal_xy_pub = rospy.Publisher("goal_xy", Marker, queue_size=1)

        self.published_poses_max_id = 0

    def publish_pose(self, pose: Pose3D, timestamp):
        """Publish the pose in the map frame in RViz."""
        p = PoseStamped(header=header_map_frame_now())
        p.header.stamp = timestamp
        p.pose.position.x = pose.x
        p.pose.position.y = pose.y

        q = tf.transformations.quaternion_from_euler(0, 0, pose.theta)
        p.pose.orientation = Quaternion(*q)

        self.pose_pub.publish(p)

    def publish_pose_list(self, pose_list: List[Pose3D]):
        """Publish a marker for each pose."""

        namespace = "pose_history"
        purple = ColorRGBA(0.62, 0.35, 1, 1)
        pub = self.pose_history_pub

        # Delete all markers
        deletion_markers = [
            Marker(id=i, action=Marker.DELETE, ns=namespace)
            for i in range(self.published_poses_max_id)
        ]
        pub.publish(MarkerArray(deletion_markers))

        # Create new markers
        pose_markers = [
            pose2D_to_marker(p, id=i, ns=namespace, color=purple)
            for i, p in enumerate(pose_list)
        ]
        pub.publish(MarkerArray(pose_markers))

        self.published_poses_max_id = len(pose_list) - 1

    def publish_goal_xy(self, goal_xy: np.ndarray):
        """Publish a star shape at the given location."""
        x, y = goal_xy.squeeze()
        scale = 1.0
        m = Marker()
        m.header = header_map_frame_now()
        m.ns = "goal_xy"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD

        # Set the scale (applies to the width of the lines)
        m.scale.x = 0.1 * scale

        # Set the color
        m.color = ColorRGBA(145 / 255, 1.0, 0.0, 1.0)

        # Star points relative to the center
        star_points = [
            (0.0, 0.5),
            (0.2, 0.2),
            (0.5, 0.2),
            (0.3, 0.0),
            (0.4, -0.4),
            (0.0, -0.2),
            (-0.4, -0.4),
            (-0.3, 0.0),
            (-0.5, 0.2),
            (-0.2, 0.2),
            (0.0, 0.5),
        ]

        m.points = [Point(x + px * scale, y + py * scale, 0) for px, py in star_points]

        self.goal_xy_pub.publish(m)


def pose3D_to_pose(p: Pose3D):
    """Create a Pose object from a Pose3D object."""
    pose = Pose()
    pose.position.x = p.x
    pose.position.y = p.y
    quaternion = tf.transformations.quaternion_from_euler(0, 0, p.theta)
    pose.orientation = Quaternion(*quaternion)
    return pose


def pose2D_to_marker(pose: Pose3D, **kwargs):
    """Create a Marker from a Pose3D object."""
    return Marker(
        header=header_map_frame_now(),
        action=Marker.ADD,
        type=Marker.ARROW,
        pose=pose3D_to_pose(pose),
        scale=Vector3(0.5, 0.01, 0.01),
        **kwargs,
    )
