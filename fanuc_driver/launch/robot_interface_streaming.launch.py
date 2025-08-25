#!/usr/bin/env python3
"""
Combined launch file for complete Fanuc robot interface.
Launches both robot state and motion streaming nodes.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    # Include robot state launch
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'robot_state.launch.py')
        ),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'J23_factor': LaunchConfiguration('J23_factor'),
            'use_bswap': LaunchConfiguration('use_bswap'),
        }.items()
    )
    
    # Include motion streaming launch
    motion_streaming_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'motion_streaming_interface.launch.py')
        ),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'J23_factor': LaunchConfiguration('J23_factor'),
            'use_bswap': LaunchConfiguration('use_bswap'),
        }.items()
    )
    
    return LaunchDescription([
        robot_ip_arg,
        j23_factor_arg,
        use_bswap_arg,
        robot_state_launch,
        motion_streaming_launch,
    ])