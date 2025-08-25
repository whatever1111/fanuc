#!/usr/bin/env python3
"""
Launch file for Fanuc joint trajectory streaming node.
Sends motion commands to robot controller via SimpleMessage protocol.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('fanuc_driver')
    
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.100',
        description='IP address of the robot controller'
    )
    
    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='11000',
        description='Port number for motion server (default: 11000)'
    )
    
    j23_factor_arg = DeclareLaunchArgument(
        'J23_factor',
        default_value='0',
        description='J2-J3 coupling factor: -1 (negative), 0 (none), 1 (positive)'
    )
    
    use_bswap_arg = DeclareLaunchArgument(
        'use_bswap',
        default_value='false',
        description='Enable byte swapping for big-endian controllers'
    )
    
    joint_names_arg = DeclareLaunchArgument(
        'joint_names',
        default_value='[joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]',
        description='Names of robot joints'
    )
    
    streaming_rate_arg = DeclareLaunchArgument(
        'streaming_rate',
        default_value='125.0',
        description='Streaming rate in Hz (default: 125Hz)'
    )
    
    # Joint streamer node
    joint_streamer_node = Node(
        package='fanuc_driver',
        executable='joint_streamer_node',
        name='fanuc_joint_streamer',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'robot_port': LaunchConfiguration('robot_port'),
            'J23_factor': LaunchConfiguration('J23_factor'),
            'use_bswap': LaunchConfiguration('use_bswap'),
            'joint_names': LaunchConfiguration('joint_names'),
            'streaming_rate': LaunchConfiguration('streaming_rate'),
        }],
        remappings=[
            ('joint_states', '/joint_states'),
            ('follow_joint_trajectory', '/follow_joint_trajectory'),
            ('joint_path_command', '/joint_path_command'),
        ]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        robot_port_arg,
        j23_factor_arg,
        use_bswap_arg,
        joint_names_arg,
        streaming_rate_arg,
        joint_streamer_node,
    ])