"""ad_perception 패키지 launch 파일."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ad_perception')
    params_file = os.path.join(pkg_dir, 'config', 'perception_config.yaml')

    perception_node = Node(
        package='ad_perception',
        executable='learned_perception_node',
        name='learned_perception_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        perception_node,
    ])
