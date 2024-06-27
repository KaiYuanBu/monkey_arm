import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler, TimerAction

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('arm_bt'), 'config', 'mock_hardware.yaml'
    )

    mock_b0_node = Node(
        package='arm_bt',
        executable='mock_actuator',
        name='b0',
        namespace='/arm',
        parameters=[config]
    )

    mock_b1_node = Node(
        package='arm_bt',
        executable='mock_actuator',
        name='b1',
        namespace='/arm',
        parameters=[config]
    )

    mock_b2_node = Node(
        package='arm_bt',
        executable='mock_actuator',
        name='b2',
        namespace='/arm',
        parameters=[config]
    )

    return LaunchDescription([
        mock_b0_node,
        # mock_b1_node,
        # mock_b2_node
    ])
