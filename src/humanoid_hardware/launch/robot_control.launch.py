#!/usr/bin/env python3
"""
Launch file for humanoid robot hardware control.
Starts the servo driver, joint state publisher GUI, robot state publisher, and RViz.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    humanoid_description_pkg = FindPackageShare('humanoid_description')
    humanoid_hardware_pkg = FindPackageShare('humanoid_hardware')

    # Path to URDF file
    urdf_file = os.path.join(
        humanoid_description_pkg.find('humanoid_description'),
        'urdf',
        'humanoid.urdf'
    )

    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for servo board'
    )

    # Robot State Publisher - Publishes robot transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # Joint State Publisher GUI - Interactive control
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Servo Driver - Hardware interface
    servo_driver_node = Node(
        package='humanoid_hardware',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 9600,
            'timeout': 1.0
        }]
    )

    # RViz - 3D Visualization (optional)
    rviz_config_file = os.path.join(
        humanoid_description_pkg.find('humanoid_description'),
        'rviz',
        'display.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        serial_port_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        servo_driver_node,
        rviz_node
    ])
