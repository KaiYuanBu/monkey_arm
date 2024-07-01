"""
ROS2 joint state publisher for Monkey Arm.

Written by:
    1. Kean Hao
Last Updated: 10/12/2023
"""

from math import pi

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

import rclpy

class StatePublisher(Node):
    """Joint state publisher of MonKey Arm."""

    def __init__(self):
        """Joint state publisher of MonKey Arm."""
        rclpy.init()
        super().__init__('state_publisher')

        # Create publishers
        qos_profile = QoSProfile(depth=1)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.get_logger().info(f"{self.get_name()} started")

        # Create subscribers
        self.bearing_subscription = self.create_subscription(
            Float64, "bearing_rotation", self.on_bearing_received, 1)
        self.boom1_subscription = self.create_subscription(
            Float64, "boom1_rotation", self.on_boom1_received, 1)
        self.boom2_subscription = self.create_subscription(
            Float64, "boom2_rotation", self.on_boom2_received, 1)

        # loop_rate = self.create_rate(10)

        # Initialize values
        self.bearing_angle = 0
        self.boom1_angle = pi/2
        self.boom2_angle = 0

        # Message declarations
        self.joint_state = JointState()
        self.joint_state.name = ['bearing_base_joint', 'base_boom1_joint', 'boom1_boom2_joint']

        # Create timer to publish joint states
        self.timer = self.create_timer(0.1, self.timer_callback)

    def on_bearing_received(self, msg: Float64):
        """Call when bearing target rotation received."""
        self.get_logger().info(f"Received bearing rotation {msg.data}")
        self.bearing_angle = msg.data

    def on_boom1_received(self, msg: Float64):
        """Call when Arm boom 1 target rotation received."""
        self.get_logger().info(f"Received boom 1 rotation {msg.data}")
        self.boom1_angle = pi/2 + msg.data

    def on_boom2_received(self, msg: Float64):
        """Call when Arm boom 2 target rotation received."""
        self.get_logger().info(f"Received boom 2 rotation {msg.data}")
        self.boom2_angle = -msg.data
    
    def timer_callback(self):
        """Pubblish joint state on triggered based on timer."""
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = [float(self.bearing_angle), float(self.boom1_angle), float(self.boom2_angle)]
        self.joint_pub.publish(self.joint_state)
        

def main():
    """Run when this script is called."""
    state_publisher = StatePublisher()

    try:
        rclpy.spin(state_publisher)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
