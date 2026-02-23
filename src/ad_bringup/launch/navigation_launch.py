"""Nav2 + robot_localization + LIO-SAM 통합 내비게이션 launch 파일.

Dual-EKF 위치 추정(로컬/글로벌) + NavSat 변환 + LIO-SAM SLAM +
Nav2 내비게이션 스택을 하나의 launch 파일로 통합 실행한다.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # 패키지 경로 설정
    bringup_dir = get_package_share_directory('ad_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 기본 설정 파일 경로
    default_ekf_params = os.path.join(bringup_dir, 'config', 'dual_ekf.yaml')
    default_nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    default_map_yaml = os.path.join(bringup_dir, 'config', 'map.yaml')
    default_lio_sam_params = os.path.join(
        bringup_dir, 'config', 'lio_sam_params.yaml'
    )

    # Launch 인자 선언
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='시뮬레이션 시간 사용 여부',
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map_yaml,
        description='맵 YAML 파일 경로',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_nav2_params,
        description='Nav2 파라미터 YAML 파일 경로',
    )
    declare_ekf_params_file = DeclareLaunchArgument(
        'ekf_params_file', default_value=default_ekf_params,
        description='robot_localization EKF 파라미터 YAML 파일 경로',
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Nav2 lifecycle 노드 자동 시작 여부',
    )
    declare_use_composition = DeclareLaunchArgument(
        'use_composition', default_value='false',
        description='Nav2 컴포지션 모드 사용 여부',
    )
    declare_use_lio_sam = DeclareLaunchArgument(
        'use_lio_sam', default_value='true',
        description='LIO-SAM SLAM 노드 활성화 여부',
    )
    declare_lio_sam_params_file = DeclareLaunchArgument(
        'lio_sam_params_file', default_value=default_lio_sam_params,
        description='LIO-SAM 파라미터 YAML 파일 경로',
    )

    # Launch 설정값 참조
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_lio_sam = LaunchConfiguration('use_lio_sam')
    lio_sam_params_file = LaunchConfiguration('lio_sam_params_file')

    # robot_localization 노드 (Dual-EKF)
    ekf_local_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_local', output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/local'),
            ('accel/filtered', '/accel/local'),
        ],
    )
    ekf_global_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_global', output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/global')],
    )
    navsat_transform_node = Node(
        package='robot_localization', executable='navsat_transform_node',
        name='navsat_transform', output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('gps/fix', '/sensor/gps'),
            ('imu', '/sensor/imu'),
            ('odometry/filtered', '/odometry/global'),
            ('odometry/gps', '/odometry/gps'),
            ('gps/filtered', '/gps/filtered'),
        ],
    )

    # LIO-SAM SLAM 노드 (조건부 실행)
    lio_sam_imu_preintegration = Node(
        package='lio_sam', executable='lio_sam_imuPreintegration',
        name='lio_sam_imu_preintegration', output='screen',
        parameters=[lio_sam_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu_raw', '/sensor/imu'),
            ('odometry/imu', '/odometry/imu_incremental'),
        ],
        condition=IfCondition(use_lio_sam),
    )
    lio_sam_image_projection = Node(
        package='lio_sam', executable='lio_sam_imageProjection',
        name='lio_sam_image_projection', output='screen',
        parameters=[lio_sam_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('points_raw', '/sensor/lidar'),
            ('imu_raw', '/sensor/imu'),
            ('odometry/imu', '/odometry/imu_incremental'),
        ],
        condition=IfCondition(use_lio_sam),
    )
    lio_sam_feature_extraction = Node(
        package='lio_sam', executable='lio_sam_featureExtraction',
        name='lio_sam_feature_extraction', output='screen',
        parameters=[lio_sam_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_lio_sam),
    )
    lio_sam_map_optimization = Node(
        package='lio_sam', executable='lio_sam_mapOptmization',
        name='lio_sam_map_optimization', output='screen',
        parameters=[lio_sam_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('gps/fix', '/sensor/gps'),
            ('odometry/mapping', '/odometry/slam'),
            ('lio_sam/mapping/cloud_registered', '/slam/cloud_registered'),
            ('lio_sam/mapping/path', '/slam/path'),
        ],
        condition=IfCondition(use_lio_sam),
    )

    lio_sam_group = GroupAction(
        actions=[
            LogInfo(msg='[Launch] LIO-SAM SLAM 노드 그룹 시작'),
            SetParameter('use_sim_time', use_sim_time),
            lio_sam_imu_preintegration,
            lio_sam_image_projection,
            lio_sam_feature_extraction,
            lio_sam_map_optimization,
        ],
        condition=IfCondition(use_lio_sam),
    )

    # Nav2 Bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params_file,
        declare_ekf_params_file,
        declare_autostart,
        declare_use_composition,
        declare_use_lio_sam,
        declare_lio_sam_params_file,

        GroupAction(actions=[
            SetParameter('use_sim_time', use_sim_time),
            ekf_local_node,
            ekf_global_node,
            navsat_transform_node,
        ]),

        lio_sam_group,
        nav2_bringup_launch,
    ])
