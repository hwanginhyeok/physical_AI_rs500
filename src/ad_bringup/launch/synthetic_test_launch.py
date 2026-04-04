"""합성 이미지 기반 perception 파이프라인 검증 런치.

Gazebo 없이 합성 이미지 → perception_node → crop_row 토픽 검증.
GPU/렌더링 불필요. Foxglove에서 결과 시각화 가능.

실행:
  ros2 launch ad_bringup synthetic_test_launch.py
  ros2 launch ad_bringup synthetic_test_launch.py scenario:=offset_left
  ros2 launch ad_bringup synthetic_test_launch.py auto_cycle:=true
  ros2 launch ad_bringup synthetic_test_launch.py use_foxglove:=false

검증 포인트:
  1. /sensor/camera/front — 합성 이미지 확인
  2. /perception/crop_row — [row_detected, steering_offset, heading_error, end_detected]
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    perception_dir = get_package_share_directory('ad_perception')
    perception_params = os.path.join(perception_dir, 'config', 'perception_params.yaml')

    # ── Launch 인자 ──
    declare_scenario = DeclareLaunchArgument(
        'scenario', default_value='center',
        description='시나리오: center, offset_left, offset_right, no_rows, end_of_row, single_row',
    )
    declare_auto_cycle = DeclareLaunchArgument(
        'auto_cycle', default_value='false',
        description='시나리오 자동 순환 모드',
    )
    declare_cycle_interval = DeclareLaunchArgument(
        'cycle_interval', default_value='5.0',
        description='자동 순환 간격 (초)',
    )
    declare_publish_rate = DeclareLaunchArgument(
        'publish_rate', default_value='5.0',
        description='이미지 발행 주기 (Hz)',
    )
    declare_use_foxglove = DeclareLaunchArgument(
        'use_foxglove', default_value='true',
        description='Foxglove Bridge 실행 여부',
    )

    scenario = LaunchConfiguration('scenario')
    auto_cycle = LaunchConfiguration('auto_cycle')
    cycle_interval = LaunchConfiguration('cycle_interval')
    publish_rate = LaunchConfiguration('publish_rate')
    use_foxglove = LaunchConfiguration('use_foxglove')

    # ── 1. 합성 이미지 퍼블리셔 ──
    synthetic_publisher = Node(
        package='ad_perception',
        executable='synthetic_image_publisher',
        name='synthetic_image_publisher',
        output='screen',
        parameters=[{
            'scenario': scenario,
            'auto_cycle': auto_cycle,
            'cycle_interval': cycle_interval,
            'publish_rate': publish_rate,
            'topic': '/sensor/camera/front',
        }],
    )

    # ── 2. perception_node ──
    perception_node = Node(
        package='ad_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[perception_params],
    )

    # ── 3. foxglove_bridge (선택) ──
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'send_buffer_limit': 10000000,
            'num_threads': 2,
        }],
        condition=IfCondition(use_foxglove),
    )

    return LaunchDescription([
        declare_scenario,
        declare_auto_cycle,
        declare_cycle_interval,
        declare_publish_rate,
        declare_use_foxglove,

        synthetic_publisher,
        perception_node,
        foxglove_bridge,
    ])
