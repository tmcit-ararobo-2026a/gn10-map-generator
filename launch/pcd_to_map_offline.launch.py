import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('gn10_map_generator')
    default_config_path = os.path.join(pkg_dir, 'config', 'pcd_to_map_offline_params.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to parameter YAML file'
    )

    input_pcd_arg = DeclareLaunchArgument(
        'input_pcd_path',
        default_value='fast_lio_map.pcd',
        description='Path to input Fast-LIO PCD file'
    )

    output_json_arg = DeclareLaunchArgument(
        'output_json_path',
        default_value='field_map.json',
        description='Path to output JSON map file'
    )

    pcd_to_map_offline_node = Node(
        package='gn10_map_generator',
        executable='pcd_to_map_offline',
        name='pcd_to_map_offline_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'input_pcd_path': LaunchConfiguration('input_pcd_path'),
                'output_json_path': LaunchConfiguration('output_json_path')
            }
        ]
    )

    return LaunchDescription([
        config_file_arg,
        input_pcd_arg,
        output_json_arg,
        pcd_to_map_offline_node
    ])