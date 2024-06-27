"""
ROS2 Interface for Monkey Arm inverse kinematics.

Written by:
    1. Kean Hao
Last Updated: 23 Feburary 2023

Use example:
ros2 service call /arm_target arm_interfaces/srv/SetTargetXYZ "{target:{x: 0.1, y: 0.0, z: 0.5}}"
ros2 service call /home std_srvs/srv/Trigger
"""

from math import pi, sin, cos, sqrt, acos, atan, atan2

from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from arm_interfaces.srv import InverseKinematics as IK
from arm_interfaces.action import SetTarget
from std_srvs.srv import Trigger

import rclpy

class InverseKinematics(Node):
    """MonKey Arm inverse kinematics node."""

    def __init__(self):
        """Arm inverse kinematic node."""
        super().__init__('inverse_kinematics')
        
        # self.create_subscription(Point, "arm_target", self.on_target_received, 5)
        self.create_service(IK, "calculate", self.on_target_received)
        # self.create_service(Trigger, "home", self.on_home_trigger)

        self.bearing_publisher = self.create_publisher(Float64, 'bearing_rotation', 5)
        self.boom1_publisher = self.create_publisher(Float64, 'boom1_rotation', 5)
        self.boom2_publisher = self.create_publisher(Float64, 'boom2_rotation', 5)
        # self.cylinder1_publisher = self.create_publisher(Float64, 'cylinder1_extension', 5)
        # self.cylinder2_publisher = self.create_publisher(Float64, 'cylinder2_extension', 5)
        self.cylinder1_client = ActionClient(self, SetTarget, 'arm/b1/SetTarget')
        self.cylinder2_client = ActionClient(self, SetTarget, 'arm/b2/SetTarget')

        self.get_logger().info(f"{self.get_name()} started")

    def on_target_received(self, request, response):
        """Arm end effector target point received."""

        # Global xyz frame
        x0 = request.target.x
        y0 = request.target.y
        z0 = request.target.z
        self.get_logger().info(f"Received target x={x0:.3f}, y={y0:.3f}, z={z0:.3f}")

        # Calculate bearing rotation
        thetha = atan2(y0,x0)
        bearing_rot = thetha if (thetha <= pi) else -(2*pi - thetha)

        # Arm's xy frame
        x1 = sqrt(x0**2 + y0**2)
        y1 = z0

        # Constants
        l1 = 2.67746 # Length of boom1_boom2_link
        l2 = 2.63531 # Length of boom2_end_effector_link

        # Calculation
        q2 = acos((l1**2 + l2**2 - x1**2 - y1**2)/(2*l1*l2)) # Boom 2 rotation
        q1 = pi - atan(y1/x1) - acos((x1**2 + y1**2 + l1**2 - l2**2)/(2*l1*sqrt(x1**2 + y1**2))) # Boom 1 rotation

        # Pubblish results
        bearing_topic = Float64()
        bearing_topic.data = bearing_rot
        self.bearing_publisher.publish(bearing_topic)

        q1_topic = Float64()
        q1_topic.data = q1
        self.boom1_publisher.publish(q1_topic)

        q2_topic = Float64()
        q2_topic.data = q2
        self.boom2_publisher.publish(q2_topic)

        cylinder1_extension = get_cylinder1_extension(q1)
        cylinder1_goal = SetTarget.Goal()
        cylinder1_goal.target = cylinder1_extension
        # cylinder1_topic = Float64()
        # cylinder1_topic.data = cylinder1_extension
        # self.cylinder1_publisher.publish(cylinder1_topic)

        cylinder2_extension = get_cylinder2_extension(q2)
        cylinder2_goal = SetTarget.Goal()
        cylinder2_goal.target = cylinder2_extension
        # cylinder2_topic = Float64()
        # cylinder2_topic.data = cylinder2_extension
        # self.cylinder2_publisher.publish(cylinder2_topic)

        # self.get_logger().info(f"Waiting for {self.cylinder1_client._action_name}")
        # self.cylinder1_client.wait_for_server()
        # self.get_logger().info(f"Waiting for {self.cylinder2_client._action_name}")
        # self.cylinder2_client.wait_for_server()

        self.get_logger().info(f"Executing target [bearing: {thetha*180/pi:.1f}, cylinder_1: {cylinder1_extension:.3f}m, cylinder_2: {cylinder2_extension:.3f}m]")

        # self.cylinder1_client.send_goal_async(cylinder1_goal)
        # self.cylinder2_client.send_goal_async(cylinder2_goal)

        response.success = True
        response.b0_target = bearing_rot
        response.b1_target = cylinder1_extension
        response.b2_target = cylinder2_extension
        return response

    # def on_home_trigger(self, request, response):
    #     """Callback on auto home service."""
    #     goal = SetTarget.Goal()
    #     goal.target = 0.0
    #     self.cylinder1_client.send_goal_async(goal)
    #     self.cylinder2_client.send_goal_async(goal)

    #     response.success = True
    #     response.message = "Home command sent"
    #     return response

def get_cylinder1_extension(rotation: float):
    """Call when Arm boom 1 target rotation received."""
    # Angle of rotation (in radian) of cylinder from reference axis (horizontal axis)
    q1 = rotation

    # Constants
    beta = 0.2328269 # Cylinder offset angle from boom1_boom2_link
    l = 0.819 # Initial cylinder length

    # Coordinate of base_cylinder1_joint
    x1 = 0.1825
    y1 = -0.295

    # Coordinate of origin (center of rotation for boom 1)
    x2 = 0
    y2 = 0
    r2 = 0.97842 # Distance of origin to cylinder1_boom1_joint

    # Calculation
    x3 = x2 + r2*cos(q1-beta)
    y3 = y2 + r2*sin(q1-beta)
    dl = sqrt((x3-x1)**2 + (y3-y1)**2) - l

    # Results
    return dl

def get_cylinder2_extension(rotation: float):
    """Call when Arm boom 2 target rotation received."""
    # Angle of rotation (in radian) of cylinder from reference axis (boom1_boom2_link).
    p = rotation

    # Constants
    theha = 0.0973894 # Cylinder offset angle from boom2_end_effector_link
    l = 0.77254 # Initial cylinder length

    # Coordinate of boom1_cylinder2_joint-
    x1 = -0.186134
    y1 = -0.276372

    # Coordinate of origin (center of rotation for boom 2)
    x2 = 0
    y2 = 0
    r2 = 0.88545 # Distance of origin to cylinder2_boom2_joint

    # Calculation
    x4 = x2 - r2*cos(p-theha)
    y4 = y2 + r2*sin(p-theha)
    dl = sqrt((x4-x1)**2 + (y4-y1)**2) - l

    # Results
    return dl

def main(args=None):
    """Run when this script is called."""
    rclpy.init(args=args)
    inverse_kinematics = InverseKinematics()

    try:
        rclpy.spin(inverse_kinematics)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    inverse_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    