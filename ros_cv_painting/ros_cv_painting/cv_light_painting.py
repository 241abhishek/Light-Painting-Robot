import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CvLightPainting(Node):
    def __init__(self):
        super().__init__('cv_light_painting')
        
        # initialize the cv_bridge
        self.bridge = CvBridge()

        # create a subscriber to the camera image
        self.sub = self.create_subscription(Image, 'camera/color/image_raw', self.image_callback, 10)


    def image_callback(self, msg):
        # convert the image message to an opencv image
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # show the image
        cv2.imshow('frame', self.frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    cv_light_painting = CvLightPainting()

    rclpy.spin(cv_light_painting)

    cv_light_painting.destroy_node()
    rclpy.shutdown()