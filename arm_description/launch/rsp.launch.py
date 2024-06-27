import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # RViz config
    rviz_config_file = os.path.join(
        get_package_share_directory('arm_description'),
        'rviz/arm.rviz'
    )

    # xacro command for model generation
    robot_description = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare('arm_description'), "urdf", "arm_robot.urdf.xacro"]
        )]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz if true'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
        ),

        Node(
            package='arm_control',
            executable='arm_rviz_publisher',
            name='arm_rviz_publisher',
            output='screen'
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz)
        )
    ])
