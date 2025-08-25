from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('fanuc_driver')
    cmd_params = os.path.join(pkg_share, 'config', 'weld_command.yaml')
    state_params = os.path.join(pkg_share, 'config', 'weld_state.yaml')
    io_map_path = os.path.join(pkg_share, 'config', 'weld_io_map.yaml')

    return LaunchDescription([
        Node(
            package='fanuc_driver',
            executable='weld_command_node',
            name='weld_command_node',
            parameters=[cmd_params, {'io_map_file': io_map_path}],
            output='screen'
        ),
        Node(
            package='fanuc_driver',
            executable='weld_state_node_tcp',
            name='weld_state_node_tcp',
            parameters=[state_params, {'io_map_file': io_map_path}],
            output='screen'
        ),
        # SimpleMessage variant can be enabled by switching executable below
        # Node(
        #     package='fanuc_driver',
        #     executable='weld_state_node_simple',
        #     name='weld_state_node_simple',
        #     parameters=[state_params],
        #     output='screen'
        # )
    ])
