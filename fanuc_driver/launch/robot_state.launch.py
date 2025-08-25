#!/usr/bin/env python3
"""
Launch file for Fanuc robot state node.
Reads robot joint positions and status from controller via SimpleMessage protocol.
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
        default_value='11002',
        description='Port number for robot state server (default: 11002)'
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
    
    # Robot state node
    robot_state_node = Node(
        package='fanuc_driver',
        executable='robot_state_node',
        name='fanuc_robot_state',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'robot_port': LaunchConfiguration('robot_port'),
            'J23_factor': LaunchConfiguration('J23_factor'),
            'use_bswap': LaunchConfiguration('use_bswap'),
            'joint_names': LaunchConfiguration('joint_names'),
        }],
        remappings=[
            ('joint_states', '/joint_states'),
            ('robot_status', '/robot_status'),
        ]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        robot_port_arg,
        j23_factor_arg,
        use_bswap_arg,
        joint_names_arg,
        robot_state_node,
    ])