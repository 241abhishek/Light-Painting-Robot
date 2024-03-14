import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import numpy as np

# read calibration values from an external txt file
def read_calibration_values(filepath): 
    # create two numpy arrays to store the lower and upper bounds
    lower_bound = np.zeros(3)
    upper_bound = np.zeros(3)
    # open the file in read mode
    with open(filepath, 'r') as file:
        # read the lines from the file
        lines = file.readlines()
        # loop through the lines and store the values in the arrays
        for i, line in enumerate(lines):
            if i == 0:
                lower_bound = np.array([int(x) for x in line.split(',')])
            else:
                upper_bound = np.array([int(x) for x in line.split(',')])
    return lower_bound, upper_bound

class CvLightPainting(Node):
    def __init__(self):
        super().__init__('cv_light_painting')
        
        # declare calibration value filepath parameter
        self.declare_parameter('calibration_values_filepath', "")
        # get the calibration value filepath parameter
        calibration_values_filepath = self.get_parameter('calibration_values_filepath').get_parameter_value().string_value

        # initialize the cv_bridge
        self.bridge = CvBridge()

        # create a subscriber to the camera image
        self.sub = self.create_subscription(Image, 'camera/color/image_raw', self.image_callback, 10)

        # initialize the frame
        self.frame = None

        # read the calibration values from the file
        self.lower_bound, self.upper_bound = read_calibration_values(calibration_values_filepath)

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
    rclpy.shutdown()