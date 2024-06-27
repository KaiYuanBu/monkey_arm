import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler, TimerAction

def generate_launch_description():

    # Configurations
    mock_config = os.path.join(
        get_package_share_directory('arm_bt'), 'config', 'mock_hardware.yaml'
    )

    cylinder_config = os.path.join(
        get_package_share_directory('arm_control'), 'config', 'arm_bringup.yaml'
    )

    # Launch Commands
    mock_b0_node = Node(
        package='arm_bt',
        executable='mock_actuator',
        name='b0',
        namespace='/arm',
        parameters=[mock_config]
    )

    cylinder1_node = Node(
        package='arm_control',
        executable='cylinder_interface',
        name='b1',
        namespace='/arm',
        parameters=[cylinder_config]
    )

    cylinder2_node = Node(
        package='arm_control',
        executable='cylinder_interface',
        name='b2',
        namespace='/arm',
        parameters=[cylinder_config]
    )

    cylinder2_delay = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=cylinder1_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[cylinder2_node]
                )
            ],
        )
    )

    ik_node = Node(
        package='arm_control',
        executable='inverse_kinematics',
        name='inverse_kinematics',
        namespace='/arm'
    )

    arm_bt = Node(
        package='arm_bt',
        executable='arm_bt_server',
        name='arm_bt_server',
        namespace='/arm',
    )

    arm_bt_delay = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=cylinder2_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[arm_bt]
                )
            ],
        )
    )

    return LaunchDescription([
        mock_b0_node,
        cylinder1_node,
        cylinder2_delay,
        ik_node,
        arm_bt_delay
    ])
