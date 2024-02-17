"""
Publishes velocity commands to control the turtlebot.

Publishers
----------
    cmd_vel: geometry_msgs/msg/Twist - The velocity of the robot

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mocap4r2_msgs.msg import RigidBodies
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point

from custom_interfaces.srv import Pose
from std_srvs.srv import Empty

import math
import numpy as np
from enum import Enum, auto

class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration

    """

    MOVE_POSE = (auto(),)  # turtle is moving to the desired pose
    MOVE_PATH = (auto(),)  # turtle is moving along a deired path obtained from the gcode file
    STOPPED = (auto(),) # turtle is not moving

def read_coordinates_from_file(filename):
    """
    Read coordinates from a gcode file

    Args:
        filename (_type_): filename to read coordinates from

    Returns:
        _type_: a list of coordinates in the format (g, x, y)
        where g is the path type indicator and x and y are the coordinates
        if g = 1, then the light is turned on, else it is turned off
    """
    coordinates = []
    with open(filename, 'r') as file:
        for line_num, line in enumerate(file):
            if line_num >= 2: # skip the first two lines
                tokens = line.strip().split()
                for token in tokens:
                    if token.startswith('G'):
                        g = float(token[1:])
                    elif token.startswith('X'):
                        x = float(token[1:])
                    elif token.startswith('Y'):
                        y = float(token[1:])
                        coordinates.append((g, x, y))
    return coordinates

def waypoint_parser(coordinates):
    """Convert the gcode coordinates into waypoint poses."""
    waypoints = []
    for i in range(len(coordinates)-1):
        x = coordinates[i][1]
        y = coordinates[i][2]
        del_x = coordinates[i+1][1] - coordinates[i][1]
        del_y = coordinates[i+1][2] - coordinates[i][2]
        theta = math.atan2(del_y, del_x)
        waypoints.append((x,y,theta))
    
    waypoints.append((coordinates[-1][1], coordinates[-1][2], 0.0))
    return waypoints

class TurtleControl(Node):
    """This node publishes velocity commands to control the turtlebot."""

    def __init__(self):
        super().__init__("turtle_control")

        # create publisher to send velocity commands to the robot
        self.velocity = self.create_publisher(Twist, "cmd_vel", 10)

        # create service to send desired pose to the robot
        self.position_srv = self.create_service(
            Pose, "pose", self.pose_srv_callback)

        # create service to load gcode file
        self.load_gcode = self.create_service(Empty, "load_gcode", self.load_gcode_srv_callback)

        # dynamic broadcasting
        self.broadcaster = TransformBroadcaster(self)

        self.transform_stamped = self.create_subscription(RigidBodies, "/rigid_bodies", self.transform_stamped_callback, qos_profile=10)

        # declare parameters
        self.declare_parameter("frequency", 100.0)
        self.param_frequency = (self.get_parameter("frequency").get_parameter_value().double_value)

        self.declare_parameter("filepath", "")
        self.param_filepath = (self.get_parameter("filepath").get_parameter_value().string_value)

        try:
            # read coordinates from the gcode file
            self.coordinates = read_coordinates_from_file(self.param_filepath)
            self.waypoints = waypoint_parser(self.coordinates)
        except:
            self.get_logger().error("Could not read coordinates from the gcode file, please check the filepath parameter")
            self.coordinates = []

        # create the main timer
        self.create_timer(1.0 / self.param_frequency, self.timer_callback)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.ox = 0.0
        self.oy = 0.0
        self.oz = 0.0
        self.ow = 1.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.des_x = 0.0
        self.des_y = 0.0
        self.des_theta = 0.0
        self.state = State.STOPPED
        self.target_reached = True
        self.waypoint_conter = 0

    def timer_callback(self):
        """Call function for the timer."""

        # publish the frame properties for the tb_1
        self.transform_publisher()

        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(x= self.ox, y= self.oy, z= self.oz, w= self.ow)
        # print(f"{self.roll=}, {self.pitch=}, {self.yaw=}")
        # print(f"{self.x=}, {self.y=}")

        # self.get_logger().info(f"{self.coordinates=}")

        if self.state == State.MOVE_POSE:
            self.twist_calculator_pose(self.des_x, self.des_y, self.des_theta)
        if self.state == State.MOVE_PATH:
            self.twist_calculator_pose(self.waypoints[self.waypoint_conter][0], self.waypoints[self.waypoint_conter][1],
                                       self.waypoints[self.waypoint_conter][2])
            self.get_logger().info(f"{self.waypoints[self.waypoint_conter][0]=}, {self.waypoints[self.waypoint_conter][1]=}, \
                                   {self.waypoints[self.waypoint_conter][2]=}")

    def transform_stamped_callback(self, msg: TransformStamped):
        """Set the frame properties for the tb_1."""

        self.x = msg.rigidbodies[0].pose.position.x
        self.y = - msg.rigidbodies[0].pose.position.z
        self.z = msg.rigidbodies[0].pose.position.y
        self.ox = msg.rigidbodies[0].pose.orientation.x
        self.oy = - msg.rigidbodies[0].pose.orientation.z
        self.oz = msg.rigidbodies[0].pose.orientation.y
        self.ow = msg.rigidbodies[0].pose.orientation.w
    
    def transform_publisher(self):
        """Publish the frame properties for the tb_1."""

        world_tb_1_tf = TransformStamped()
        world_tb_1_tf.header.frame_id = "world"
        world_tb_1_tf.child_frame_id = "tb_1"
        world_tb_1_tf.header.stamp = self.get_clock().now().to_msg()
        world_tb_1_tf.transform.translation.x = self.x
        world_tb_1_tf.transform.translation.y = self.y
        world_tb_1_tf.transform.translation.z = self.z
        world_tb_1_tf.transform.rotation.x = self.ox
        world_tb_1_tf.transform.rotation.y = self.oy
        world_tb_1_tf.transform.rotation.z = self.oz
        world_tb_1_tf.transform.rotation.w = self.ow
        self.broadcaster.sendTransform(world_tb_1_tf)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z) # in degrees
    
    def pose_srv_callback(self, req, res):
        self.des_x = req.x
        self.des_y = req.y
        self.des_theta = req.theta
        self.state = State.MOVE_POSE
        self.target_reached = False
        res.success = True
        return res
    
    def load_gcode_srv_callback(self, req, res):
        if len(self.coordinates) > 0:
            self.waypoint_conter = 0
            self.state = State.MOVE_PATH
        else:
            self.get_logger().error("Could not read coordinates from the gcode file, please load the gcode file first")
        return res

    def twist_calculator_pose(self, target_x, target_y, target_theta):
        """Calculate the velocity commands for the turtlebot."""
        
        self.get_logger().info(f"{target_x=}, {target_y=}, {target_theta=}")
        # current heading
        vec_curr = np.array([np.cos(math.radians(self.yaw)), np.sin(math.radians(self.yaw))])
        # print(f"{vec_curr=}")
        vec_curr_unit = vec_curr / np.linalg.norm(vec_curr) # normalize

        # desired heading
        vec_des = np.array([target_x - self.x, target_y - self.y])
        # print(f"{vec_des=}")
        vec_des_unit = vec_des / np.linalg.norm(vec_des) # normalize

        # compute the error in heading wrt the desired movement direction
        heading_error = math.atan2(np.linalg.det([vec_curr_unit,vec_des_unit]),np.dot(vec_des_unit,vec_curr_unit))
        # print(f"{heading_error=}")

        # distance error
        distance_error = math.sqrt((target_x - self.x) ** 2 + (target_y - self.y) ** 2)

        # compute the desired target vector
        vec_tar = np.array([np.cos(math.radians(target_theta)), np.sin(math.radians(target_theta))])
        vec_tar_unit = vec_tar / np.linalg.norm(vec_tar) # normalize
        target_yaw_error = math.atan2(np.linalg.det([vec_curr_unit,vec_tar_unit]),np.dot(vec_tar_unit,vec_curr_unit))
        print(f"{target_yaw_error=}")

        # compute the velocity commands
        t = Twist()
        # check if the heading error is greater than 10 degrees
        # if so, rotate in the desired direction
        if abs(heading_error) > 0.174533 and self.target_reached == False:
            if heading_error > 0:
                t.angular.z = 0.3
            if heading_error < 0:
                t.angular.z = -0.3
        elif distance_error > 0.05:
            t.linear.x = 0.075
            if heading_error > 0:
                t.angular.z = 0.05
            if heading_error < 0:
                t.angular.z = -0.05
        else:
            self.target_reached = True
            if abs(target_yaw_error) > 0.0872665:
                if target_yaw_error > 0:
                    t.angular.z = 0.3
                if target_yaw_error < 0:
                    t.angular.z = -0.3
            else:
                t.linear.x = 0.0
                t.angular.z = 0.0
                self.waypoint_conter += 1
                if self.waypoint_conter >= len(self.waypoints):
                    self.waypoint_conter = 0
                    self.state = State.STOPPED

        self.velocity.publish(t)

def main(args=None):
    rclpy.init(args=args)
    control = TurtleControl()
    rclpy.spin(control)
    rclpy.shutdown()


if __name__ == "__main__":
    main()