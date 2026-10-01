import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('gn10_map_generator')
    default_config_path = os.path.join(pkg_dir, 'config', 'pointcloud_to_map_params.yaml')

    # Launch 引数の定義
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to parameter YAML file'
    )

    input_pcd_topic_arg = DeclareLaunchArgument(
        'input_pcd_topic',
        default_value='/livox/lidar',
        description='Input PointCloud2 topic name'
    )

    # ノードの設定
    pcd_to_map_node = Node(
        package='gn10_map_generator',
        executable='pointcloud_to_map',
        name='pcd_to_map_json_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('input_pointcloud', LaunchConfiguration('input_pcd_topic'))
        ]
    )

    return LaunchDescription([
        config_file_arg,
        input_pcd_topic_arg,
        pcd_to_map_node
    ])