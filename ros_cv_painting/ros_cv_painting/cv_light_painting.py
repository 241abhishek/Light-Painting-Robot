import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from std_srvs.srv import Empty

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
        self.calibration_values_filepath = self.get_parameter('calibration_values_filepath').get_parameter_value().string_value

        self.declare_parameter("frequency", 250.0)
        self.param_frequency = (self.get_parameter("frequency").get_parameter_value().double_value)

        # initialize the cv_bridge
        self.bridge = CvBridge()

        # create a subscriber to the camera image
        self.sub = self.create_subscription(Image, 'camera/color/image_raw', self.image_callback, 10)

        # initialize the frame
        self.frame = None

        # read the calibration values from the file
        self.lower_bound, self.upper_bound = read_calibration_values(self.calibration_values_filepath)

        # create a service to start the calibration procedure
        self.calibrate = self.create_service(Empty, 'calibrate_color', self.calibrate_callback)

        # create a service to start the drawing procedure
        self.draw = self.create_service(Empty, 'draw_color', self.draw_color_callback)

        # intialize the calibration flag
        self.calibration = False

        # initialize the draw flag
        self.draw = False

        # initialize the video
        self.video = None

        # create a timer to process the image
        self.timer = self.create_timer(1.0 / self.param_frequency, self.timer_callback)


    def timer_callback(self):
        # check if the frame is not None
        # else return
        if self.frame is None:
            return
        
        # check if the calibration flag is True
        # if True, call the calibrate function
        if self.calibration:
            self.calibrate_color()

        # if the draw flag is True, then draw the color on the frame
        if self.draw:
            self.draw_color()

    def image_callback(self, msg):
        # convert the image message to an opencv image
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def calibrate_callback(self, request, response):
        # set the calibration flag to True
        self.calibration = True

        # Create a window to display the video
        cv2.namedWindow('Video')

        # Create trackbars to select the color to track
        cv2.createTrackbar('Hue Min', 'Video', 0, 179, lambda x: x)
        cv2.createTrackbar('Hue Max', 'Video', 0, 179, lambda x: x)
        cv2.createTrackbar('Sat Min', 'Video', 0, 255, lambda x: x)
        cv2.createTrackbar('Sat Max', 'Video', 0, 255, lambda x: x)
        cv2.createTrackbar('Val Min', 'Video', 0, 255, lambda x: x)
        cv2.createTrackbar('Val Max', 'Video', 0, 255, lambda x: x)

        # Set the trackbar positions to the calibration values
        cv2.setTrackbarPos('Hue Min', 'Video', self.lower_bound[0])
        cv2.setTrackbarPos('Hue Max', 'Video', self.upper_bound[0])
        cv2.setTrackbarPos('Sat Min', 'Video', self.lower_bound[1])
        cv2.setTrackbarPos('Sat Max', 'Video', self.upper_bound[1])
        cv2.setTrackbarPos('Val Min', 'Video', self.lower_bound[2])
        cv2.setTrackbarPos('Val Max', 'Video', self.upper_bound[2])

        # return the response
        return response
    
    def draw_color_callback(self, request, response):
        # set the draw flag to True
        self.draw = True

        # Initialize an empty canvas
        self.canvas = np.zeros((480, 640, 3), dtype=np.uint8)

        # return the response
        return response
    
    def calibrate_color(self):

        # Get the current positions of the trackbars
        h_min = cv2.getTrackbarPos('Hue Min', 'Video')
        h_max = cv2.getTrackbarPos('Hue Max', 'Video')
        s_min = cv2.getTrackbarPos('Sat Min', 'Video')
        s_max = cv2.getTrackbarPos('Sat Max', 'Video')
        v_min = cv2.getTrackbarPos('Val Min', 'Video')
        v_max = cv2.getTrackbarPos('Val Max', 'Video')

        # Create a lower and upper bound for the color to track
        self.lower_bound = np.array([h_min, s_min, v_min])
        self.upper_bound = np.array([h_max, s_max, v_max])

        # create an image from self.frame
        frame = self.frame

        # Convert the frame from BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask to display the color to track
        mask = cv2.inRange(hsv, self.lower_bound, self.upper_bound)

        # overlay the mask on the original frame
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Display the frame
        cv2.imshow('Video', result)

        # Wait for frame rate, and break the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            self.calibration = False
        
        # if key 's' is pressed, then save the color to track
        if cv2.waitKey(1) & 0xFF == ord('s'):
            # log the color to track
            self.get_logger().info(f'Color to track: {self.lower_bound}, {self.upper_bound}')
            # write the color to track to an external file
            with open(self.calibration_values_filepath, 'w') as file:
                file.write(f'{self.lower_bound[0]},{self.lower_bound[1]},{self.lower_bound[2]}\n')
                file.write(f'{self.upper_bound[0]},{self.upper_bound[1]},{self.upper_bound[2]}')
            cv2.destroyAllWindows()
            self.calibration = False

        # if key 'r' is pressed, then reset the trackbars to the default min-max values
        if cv2.waitKey(1) & 0xFF == ord('r'):
            cv2.setTrackbarPos('Hue Min', 'Video', 0)
            cv2.setTrackbarPos('Hue Max', 'Video', 179)
            cv2.setTrackbarPos('Sat Min', 'Video', 0)
            cv2.setTrackbarPos('Sat Max', 'Video', 255)
            cv2.setTrackbarPos('Val Min', 'Video', 0)
            cv2.setTrackbarPos('Val Max', 'Video', 255)

        # if key 'f' is pressed, then read the calibration values from the external file and set the trackbars
        if cv2.waitKey(1) & 0xFF == ord('f'):
            self.lower_bound, self.upper_bound = read_calibration_values(self.calibration_values_filepath)
            cv2.setTrackbarPos('Hue Min', 'Video', self.lower_bound[0])
            cv2.setTrackbarPos('Hue Max', 'Video', self.upper_bound[0])
            cv2.setTrackbarPos('Sat Min', 'Video', self.lower_bound[1])
            cv2.setTrackbarPos('Sat Max', 'Video', self.upper_bound[1])
            cv2.setTrackbarPos('Val Min', 'Video', self.lower_bound[2])
            cv2.setTrackbarPos('Val Max', 'Video', self.upper_bound[2])

    def draw_color(self):
        # read in the calibration values from an external file
        self.lower_bound, self.upper_bound = read_calibration_values(self.calibration_values_filepath)

        # create an image from self.frame
        frame = self.frame

        # resize the video frame
        frame = cv2.resize(frame, (640, 480))

        # Convert the frame from BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask to display the color to track
        mask = cv2.inRange(hsv, self.lower_bound, self.upper_bound)

        # overlay the mask on the original frame
        color_region = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Accumulate the moving color regions over frames to create the final image
        # prevent overwriting the canvas that has already been painted
        # check if canvas region is black, then paint the color region
        # if not, then do not paint the color region
        self.canvas = cv2.bitwise_and(self.canvas, self.canvas, mask=cv2.bitwise_not(mask))
        self.canvas = cv2.add(self.canvas, color_region)

        # remove noise from the canvas
        self.canvas = cv2.medianBlur(self.canvas, 5)
        
        # Average the canvas with the frame to create a moving painting
        final_painting = cv2.addWeighted(frame, 0.5, self.canvas, 0.8, 0)

        # # Display the original frame
        # cv2.imshow('Original Video', frame)

        # # Display the color region
        # cv2.imshow('Color Tracking', color_region)

        # # Display the canvas
        # cv2.imshow('Light Painting', self.canvas)

        # # Display the final painting
        # cv2.imshow('Painting Overlay', final_painting)

        # Create a blank canvas to display the images
        grid = np.zeros((2 * frame.shape[0], 2 * frame.shape[1], 3), dtype=np.uint8)

        # Place the images onto the canvas
        grid[:frame.shape[0], :frame.shape[1]] = frame
        grid[:color_region.shape[0], frame.shape[1]:] =color_region
        grid[frame.shape[0]:, :self.canvas.shape[1]] = self.canvas
        grid[frame.shape[0]:, frame.shape[1]:] = final_painting

        # Display the canvas
        cv2.imshow('Light Painting Grid', grid)

        # Wait for frame rate, and break the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            self.draw = False
        
        # if key 's' is pressed, then save the painting
        if cv2.waitKey(1) & 0xFF == ord('s'):
            # save the painting to an external file
            cv2.imwrite('light_painting.jpg', self.canvas)
            cv2.imwrite('overlay.jpg', final_painting)
            cv2.destroyAllWindows()
            self.draw = False

def main(args=None):
    rclpy.init(args=args)
    cv_light_painting = CvLightPainting()
    rclpy.spin(cv_light_painting)
    rclpy.shutdown()