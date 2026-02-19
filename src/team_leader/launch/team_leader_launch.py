"""team_leader 패키지 launch 파일."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('team_leader')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    leader_node = Node(
        package='team_leader',
        executable='leader_node',
        name='team_leader',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        leader_node,
    ])
