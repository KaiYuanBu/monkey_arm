"""
ROS2 Interface for Monkey Arm cylinder control.

Written by:
    1. Kean Hao
Last Updated: 8 September 2023

Edited by:
    1. Kai Yuan
Last Edited: 24/6/2024

Usage (send goal):
ros2 action send_goal -f /arm/bearing/SetTarget arm_interfaces/action/SetTarget "{target_rotation: 10.0}"
ros2 action send_goal -f /arm/b1/SetTarget arm_interfaces/action/SetTarget "{target: 0.3}"
ros2 action send_goal -f /arm/b2/SetTarget arm_interfaces/action/SetTarget "{target: 0.3}"
"""

from rclpy.action import ActionServer
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

import time
import can
import rclpy
from rclpy.action.server import ServerGoalHandle

from .IDSServoDriver import IDSServoDriver
from arm_interfaces.action import SetTarget
from arm_interfaces.srv import GetPosition, HomeRequest

class CylinderInterface(Node):
    """MonKey Arm cylinder control wrapper for IDSServoDriver."""

    def __init__(self):
        """MonKey Arm cylinder control node."""
        super().__init__('cylinder')

        self.declare_parameter('can_id', 71,
            ParameterDescriptor(description='CAN ID of the target driver.'))
        self.declare_parameter('can_channel', 'can0', # can*, vcan* or /dev/ttyUSB*
            ParameterDescriptor(description='Channel of CAN tranceiver.'))
        self.declare_parameter('can_baudrate', 2000000,
            ParameterDescriptor(description='Baudrate of USB communication.'))
        self.declare_parameter('can_bitrate', 500000,
            ParameterDescriptor(description='Bitrate of CAN communication bus in bit/s.'))
        
        # Get parameters
        self.can_id = self.get_parameter('can_id').value
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
        self.get_logger().info(f"Setting up driver \"{self.get_name()}\" with CAN ID \"{self.can_id}\"")
        self.driver = IDSServoDriver(bus, self.can_id, name=self.get_name())
        time.sleep(0.5)
        self.driver.fault_reset()
        time.sleep(0.5)
        self.driver.set_positional_control_mode()
        time.sleep(0.5)
        self.driver.read_extension()

        # Declare action server
        self.action_server = ActionServer(self, SetTarget, f'{self.get_name()}/SetTarget', self.action_callback)

        # Declare get position service server
        self.service_server = self.create_service(GetPosition, f'{self.get_name()}/GetPosition', self.position_service_callback)

        # Declare home request service server
        self.service_server2 = self.create_service(HomeRequest, f'{self.get_name()}/HomeRequest', self.home_request_callback)
        
        # Start message
        self.get_logger().info(f"{self.get_name()} Initialized")


    def action_callback(self, goal_handle:ServerGoalHandle):
        """Callback for set extension action."""
        self.get_logger().info("Executing goal...")

        self.driver.position_mode_set_accel_decel(1, 1, False)
        self.driver.position_mode_set_speed(3000, False)

        self.driver.set_extension(goal_handle.request.target, 10, False)

        prev_time = time.time()
        feedback_msg = SetTarget.Feedback()
        result = SetTarget.Result()
        result.success = True

        while self.driver.is_running:
            self.get_logger().info(f'Extension: {self.driver.extension}')
            feedback_msg.current_position = float(self.driver.extension)
            goal_handle.publish_feedback(feedback_msg)

            # Check for timeout
            if (time.time() - prev_time) >= 10:
                result.success = False
                goal_handle.abort()
                self.driver.is_running = False
                # TODO: Halt cylinder extension
                break

            time.sleep(0.1)

        if result.success:
            goal_handle.succeed()

        return result

    def position_service_callback(self, request, response):
        """Callback for get position service."""
        self.get_logger().info("Reading extension...")
        response.position = self.driver.extension

        return response
    
    def home_request_callback(self, request, response):
        """Callback for homing request service."""
        self.get_logger().info("Initiating Homing Procedure for Monkey Arm ...")
        self.driver.position_mode_set_accel_decel(3, 3, True)
        self.driver.position_mode_set_speed(800, True)

        self.get_logger().info(f"{self.get_name()} : Calibrating Home Position...")
        self.driver.set_extension(-0.4, 30, True) # wait argument must be True
        time.sleep(0.5)
        self.driver.clear_position()

        ending_pos = self.driver.read_extension()
        self.get_logger().info(f"{self.get_name()}'s current position : {ending_pos}")

        if ending_pos > 0.001 or ending_pos < -0.001:
            response.homing_status = False
            self.get_logger().info(f"{self.get_name()} : Homing Failed")
        else:
            response.homing_status = True
            self.get_logger().info(f"{self.get_name()} : Homing Successful")

        return response

def main(args=None):
    """Run when this script is called."""
    rclpy.init(args=args)

    #Setup CAN listener
    print("Setting CAN notifier")
    cylinder = CylinderInterface()

    try:
        rclpy.spin(cylinder)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cylinder.destroy_node()

if __name__ == '__main__':
    main()