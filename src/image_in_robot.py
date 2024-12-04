#!/usr/bin/python3

import rospy
from rospy import Publisher
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

IMG_TOPIC = "/camera/color/image_raw/compressed"


class RobotImage:

    def __init__(self):
        self.image = None
        rospy.Subscriber(IMG_TOPIC, CompressedImage, self.img_callback)
        rospy.Subscriber("/send_image", String, self.want_image_callback)
        self.image_pub = Publisher("/sent_image", CompressedImage, queue_size=1)

    def img_callback(self, img: CompressedImage):
        print("Got image!")
        self.image = img

    def want_image_callback(self, msg: String):
        if self.image is not None:
            self.image_pub.publish(self.image)


if __name__ == "__main__":
    rospy.init_node("robot")
    RobotImage()
    rospy.spin()
