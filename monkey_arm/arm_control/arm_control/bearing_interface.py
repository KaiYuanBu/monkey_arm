"""
ROS2 Interface for Monkey Arm bearing control.

Written by:
    1. Kean Hao
Last Updated: 9 September 2023

Usage (send goal):
ros2 action send_goal -f /arm/cylinder_1/SetExtension arm_interfaces/action/SetExtension "{target_extension: 0.1}"
"""

from rclpy.action import ActionServer
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import time
import can
import rclpy
from rclpy.action.server import ServerGoalHandle

from .bearing_driver import BearingDriver
from arm_interfaces.action import SetTarget

class BearingInterface(Node):
    """MonKey Arm bearing control wrapper."""

    def __init__(self):
        """MonKey Arm bearing control node."""
        super().__init__('bearing')

        self.declare_parameter('node_id', 0x0E,
            ParameterDescriptor(description='CAN ID of the target driver.'))
        self.declare_parameter('can_channel', 'can0', # can*, vcan* or /dev/ttyUSB*
            ParameterDescriptor(description='Channel of CAN tranceiver.'))
        self.declare_parameter('can_baudrate', 2000000,
            ParameterDescriptor(description='Baudrate of USB communication.'))
        self.declare_parameter('can_bitrate', 500000,
            ParameterDescriptor(description='Bitrate of CAN communication bus in bit/s.'))
        
        # Get parameters
        self.node_id = self.get_parameter('node_id').value
        can_channel = self.get_parameter('can_channel').value
        baudrate = self.get_parameter('can_baudrate').value
        bitrate = self.get_parameter('can_bitrate').value

        # Create CAN connection
        bus = None
        if (can_channel.startswith('can') or can_channel.startswith('vcan')):
            self.get_logger().info(f"Creating bus with SocketCAN interface in channel \"{can_channel}\"")
            bus = can.ThreadSafeBus(
                interface='socketcan', channel=can_channel, bitrate=bitrate
            )
        elif (can_channel.startswith('/dev/ttyUSB')):
            self.get_logger().info(f"Creating bus with seeedstudio interface in channel \"{can_channel}\"")
            bus = can.ThreadSafeBus(
                interface='seeedstudio', channel=can_channel, baudrate=baudrate, bitrate=bitrate
            )
        else:
            raise Exception(f"Invalid CAN channel \"{can_channel}\"")

        time.sleep(0.5)

        # Initialize driver
        self.get_logger().info(f"Setting up driver \"{self.get_name()}\" with NODE ID \"{self.node_id}\"")
        self.driver = BearingDriver(bus, self.node_id)

        # Declare action server
        self.action_server = ActionServer(self, SetTarget, f'{self.get_name()}/SetTarget', self.target_action_callback)

        # Start message
        self.get_logger().info(f"{self.get_name()} started")

    def target_action_callback(self, goal_handle:ServerGoalHandle):
        """Callback for set extension action."""
        self.get_logger().info("Executing target...")
        
        # Set target to driver
        self.driver.set_target_speed(120)
        self.driver.set_target_position(goal_handle.request.target)

        prev_time = time.time()
        feedback_msg = SetTarget.Feedback()
        result = SetTarget.Result()
        result.success = True

        # Loop until target reached or timeout
        while self.driver.is_running:
            # Get positional feedback
            self.get_logger().info(f'Position: {self.driver.current_position}')
            feedback_msg.current_position = float(self.driver.current_position)
            goal_handle.publish_feedback(feedback_msg)

            # Check for timeout
            if (time.time() - prev_time) >= 8:
                result.success = False
                goal_handle.abort()
                self.driver.is_running = False
                # TODO: Halt bearing rotation
                break

            time.sleep(0.1)

        if result.success:
            goal_handle.succeed()
        
        return result

def main(args=None):
    """Run when this script is called."""
    rclpy.init(args=args)

    #Setup CAN listener
    print("Setting CAN notifier")
    bearing = BearingInterface()

    try:
        rclpy.spin(bearing)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bearing.destroy_node()

if __name__ == '__main__':
    main()