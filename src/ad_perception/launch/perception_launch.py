"""ad_perception 패키지 launch 파일."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ad_perception')
    params_file = os.path.join(pkg_dir, 'config', 'perception_params.yaml')

    perception_node = Node(
        package='ad_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[params_file],
    )

    localization_node = Node(
        package='ad_perception',
        executable='localization_node',
        name='localization_node',
        output='screen',
    )

    return LaunchDescription([
        perception_node,
        localization_node,
    ])
