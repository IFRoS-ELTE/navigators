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
from utils.common import MAP_FRAME, POSE_TOPIC
from utils.pose import Pose3D


def header_map_frame_now():
    return rospy.Header(stamp=rospy.Time.now(), frame_id=MAP_FRAME)


class RVizPublisher:
    def __init__(self):
        self.br = tf.TransformBroadcaster()

        self.pose_pub = rospy.Publisher(POSE_TOPIC, PoseStamped, queue_size=1)

    def publish_pose(self, pose: Pose3D, timestamp):
        """Publish the pose in the map frame in RViz."""
        p = PoseStamped(header=header_map_frame_now())
        p.header.stamp = timestamp
        p.pose.position.x = pose.x
        p.pose.position.y = pose.y

        q = tf.transformations.quaternion_from_euler(0, 0, pose.theta)
        p.pose.orientation = Quaternion(*q)

        self.pose_pub.publish(p)
