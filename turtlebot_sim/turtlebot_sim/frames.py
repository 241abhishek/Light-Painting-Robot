import rclpy
from rclpy.node import Node

import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from mocap4r2_msgs.msg import RigidBodies

from geometry_msgs.msg import TransformStamped

class Frames(Node):
    """This node broadcasts frames for visualising the turtlebot in rviz."""

    def __init__(self):
        super().__init__("frames")

        # dynamic broadcasting
        self.broadcaster = TransformBroadcaster(self)

        self.transform_stamped = self.create_subscription(RigidBodies, "/rigid_bodies", self.transform_stamped_callback, qos_profile=10)

        # declare parameters
        self.declare_parameter("frequency", 250.0)
        self.param_frequency = (self.get_parameter("frequency").get_parameter_value().double_value)

        # create the main timer
        self.create_timer(1.0 / self.param_frequency, self.timer_callback)

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.ox = 0.0
        self.oy = 0.0
        self.oz = 0.0
        self.ow = 1.0

    def timer_callback(self):
        """Call function for the timer."""

        world_tb_1_tf = TransformStamped()
        world_tb_1_tf.header.frame_id = "world"
        world_tb_1_tf.child_frame_id = "tb_1"
        world_tb_1_tf.header.stamp = self.get_clock().now().to_msg()
        world_tb_1_tf.transform.translation.x = self.x
        world_tb_1_tf.transform.translation.y = - self.z
        world_tb_1_tf.transform.translation.z = self.y
        world_tb_1_tf.transform.rotation.x = self.ox
        world_tb_1_tf.transform.rotation.y = - self.oz
        world_tb_1_tf.transform.rotation.z = self.oy
        world_tb_1_tf.transform.rotation.w = self.ow
        self.broadcaster.sendTransform(world_tb_1_tf)

    def transform_stamped_callback(self, msg: TransformStamped):
        """Set the frame properties for the tb_1."""

        r = msg
        
        # self.get_logger().info(f"{r}")

        self.x = r.rigidbodies[0].pose.position.x
        self.y = r.rigidbodies[0].pose.position.y
        self.z = r.rigidbodies[0].pose.position.z
        self.ox = r.rigidbodies[0].pose.orientation.x
        self.oy = r.rigidbodies[0].pose.orientation.y
        self.oz = r.rigidbodies[0].pose.orientation.z
        self.ow = r.rigidbodies[0].pose.orientation.w

def main(args=None):
    rclpy.init(args=args)
    frames = Frames()
    rclpy.spin(frames)
    rclpy.shutdown()