"""crop_row 감지 Foxglove 검증용 경량 런치 파일.

Nav2/EKF 없이 Gazebo + 카메라 브릿지 + perception_node + foxglove_bridge만 실행.
검증 목적:
  1. /sensor/camera/front 에 나무 이미지 수신 확인
  2. /perception/crop_row 토픽에서 steering_offset 확인
  3. 행 끝(end_detected) 신호 확인

실행:
  ros2 launch ad_bringup crop_row_test_launch.py
  ros2 launch ad_bringup crop_row_test_launch.py spawn_y:=3.0
  ros2 launch ad_bringup crop_row_test_launch.py headless:=false  # GUI 포함
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('ad_bringup')
    perception_dir = get_package_share_directory('ad_perception')

    pear_orchard_world = os.path.join(bringup_dir, 'worlds', 'pear_orchard.sdf')
    default_sdf_model = os.path.join(bringup_dir, 'models', 'ss500', 'model.sdf')
    default_bridge_config = os.path.join(bringup_dir, 'config', 'bridge_config.yaml')
    perception_params = os.path.join(perception_dir, 'config', 'perception_params.yaml')

    # ── Launch 인자 ──
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='true',
        description='GUI 없이 실행 (WSL2 기본값)',
    )
    declare_spawn_x = DeclareLaunchArgument(
        'spawn_x', default_value='0.0',
        description='로봇 스폰 X 좌표',
    )
    declare_spawn_y = DeclareLaunchArgument(
        'spawn_y', default_value='3.0',
        description='로봇 스폰 Y 좌표 (행1-행2 사이: 3.0)',
    )

    headless = LaunchConfiguration('headless')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')

    # ── 1. Gazebo 서버 (headless 렌더링) ──
    gazebo_server = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-s',
            '-r',
            '--headless-rendering',
            pear_orchard_world,
        ],
        output='screen',
    )

    # ── 2. Gazebo GUI (선택) ──
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=UnlessCondition(headless),
    )

    # ── 3. 로봇 스폰 ──
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ss500',
        output='screen',
        arguments=[
            '-name', 'ss500',
            '-file', default_sdf_model,
            '-x', spawn_x, '-y', spawn_y, '-z', '0.5',
        ],
    )

    # ── 4. ros_gz_bridge (카메라 + IMU) ──
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_file': default_bridge_config,
        }],
    )

    # ── 5. perception_node ──
    perception_node = Node(
        package='ad_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[perception_params],
    )

    # ── 6. foxglove_bridge ──
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'port': 8765,
            'send_buffer_limit': 10000000,
            'num_threads': 2,
        }],
    )

    return LaunchDescription([
        declare_headless,
        declare_spawn_x,
        declare_spawn_y,

        gazebo_server,
        gazebo_gui,
        spawn_robot,
        ros_gz_bridge,
        perception_node,
        foxglove_bridge,
    ])
