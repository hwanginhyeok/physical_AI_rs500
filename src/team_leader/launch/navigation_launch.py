"""Nav2 + robot_localization + LIO-SAM 통합 내비게이션 launch 파일.

Dual-EKF 위치 추정(로컬/글로벌) + NavSat 변환 + LIO-SAM SLAM +
Nav2 내비게이션 스택을 하나의 launch 파일로 통합 실행한다.

구성 요소:
  - ekf_local: IMU + Wheel Odometry 융합 (odom 프레임, 50Hz)
  - ekf_global: EKF local + GPS 융합 (map 프레임, 10Hz)
  - navsat_transform: GPS(WGS84) -> 로컬 직교 좌표 변환
  - LIO-SAM: LiDAR-IMU SLAM (조건부 실행, GPS 불가 시 fallback)
  - Nav2 bringup: 경로 계획 + 경로 추종 + 코스트맵 + 행동 트리
  - map_server: 정적 맵 제공 (SLAM 결과 또는 빈 맵)

LIO-SAM 통합 전략:
  - use_lio_sam:=true 시 LIO-SAM 노드 활성화
  - LIO-SAM은 백그라운드에서 항상 동작하여 SLAM 맵을 구축
  - LocalizationManager(localization_manager.py)가 GPS 상태에 따라
    LIO-SAM 오도메트리와 GPS EKF 오도메트리 간 자동 전환 수행
  - GPS 양호: Dual-EKF 출력 사용 (GPS_EKF 모드)
  - GPS 불가: LIO-SAM 출력 사용 (SLAM_ONLY 모드)
  - GPS 복구: 점진적 전환 (HYBRID 모드)
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
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    # ---------------------
    # 패키지 경로 설정
    # ---------------------
    team_leader_dir = get_package_share_directory('team_leader')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 기본 설정 파일 경로
    default_ekf_params = os.path.join(team_leader_dir, 'config', 'dual_ekf.yaml')
    default_nav2_params = os.path.join(team_leader_dir, 'config', 'nav2_params.yaml')
    default_map_yaml = os.path.join(team_leader_dir, 'config', 'map.yaml')
    default_lio_sam_params = os.path.join(
        team_leader_dir, 'config', 'lio_sam_params.yaml'
    )

    # ---------------------
    # Launch 인자 선언
    # ---------------------
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='시뮬레이션 시간 사용 여부 (Gazebo 연동 시 true)',
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_yaml,
        description='맵 YAML 파일 경로 (SLAM 결과 또는 빈 맵)',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_nav2_params,
        description='Nav2 파라미터 YAML 파일 경로',
    )

    declare_ekf_params_file = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=default_ekf_params,
        description='robot_localization EKF 파라미터 YAML 파일 경로',
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Nav2 lifecycle 노드 자동 시작 여부',
    )

    declare_use_composition = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Nav2 컴포지션 모드 사용 여부',
    )

    # --- LIO-SAM 관련 Launch 인자 ---
    declare_use_lio_sam = DeclareLaunchArgument(
        'use_lio_sam',
        default_value='true',
        description='LIO-SAM SLAM 노드 활성화 여부. '
                    'true: GPS/SLAM 자동 전환 활성, '
                    'false: Dual-EKF만 사용 (GPS 전용)',
    )

    declare_lio_sam_params_file = DeclareLaunchArgument(
        'lio_sam_params_file',
        default_value=default_lio_sam_params,
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

    # ---------------------
    # robot_localization 노드 (Dual-EKF)
    # ---------------------

    # EKF 로컬: IMU + Wheel Odometry 융합 (odom 프레임)
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            # 출력 토픽 리매핑: 글로벌 EKF의 입력으로 사용
            ('odometry/filtered', '/odometry/local'),
            ('accel/filtered', '/accel/local'),
        ],
    )

    # EKF 글로벌: EKF local + GPS 융합 (map 프레임)
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            # 출력 토픽 리매핑
            ('odometry/filtered', '/odometry/global'),
        ],
    )

    # NavSat 변환: GPS(NavSatFix) -> Odometry 변환
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            # GPS 입력
            ('gps/fix', '/sensor/gps'),
            # IMU 입력
            ('imu', '/sensor/imu'),
            # EKF 글로벌 출력을 입력으로 사용
            ('odometry/filtered', '/odometry/global'),
            # GPS 변환 결과 출력 -> 글로벌 EKF의 odom1 입력
            ('odometry/gps', '/odometry/gps'),
            # 필터링된 GPS 출력
            ('gps/filtered', '/gps/filtered'),
        ],
    )

    # ---------------------
    # LIO-SAM SLAM 노드 (조건부 실행)
    # ---------------------
    # use_lio_sam:=true 시에만 LIO-SAM 노드 그룹이 실행된다.
    # LIO-SAM은 4개의 핵심 노드로 구성:
    #   1. imuPreintegration: IMU 사전적분 (고주파 오도메트리)
    #   2. imageProjection: LiDAR 포인트클라우드 -> 레인지 이미지 변환
    #   3. featureExtraction: 엣지/서피스 특징점 추출
    #   4. mapOptmization: 인수 그래프 기반 맵 최적화 + GPS 팩터

    lio_sam_imu_preintegration = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name='lio_sam_imu_preintegration',
        output='screen',
        parameters=[
            lio_sam_params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            # IMU 입력
            ('imu_raw', '/sensor/imu'),
            # 오도메트리 입력 (IMU 사전적분 보조)
            ('odometry/imu', '/odometry/imu_incremental'),
        ],
        condition=IfCondition(use_lio_sam),
    )

    lio_sam_image_projection = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_image_projection',
        output='screen',
        parameters=[
            lio_sam_params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            # LiDAR 입력
            ('points_raw', '/sensor/lidar'),
            # IMU 입력
            ('imu_raw', '/sensor/imu'),
            # 오도메트리 입력 (디스토션 보정용)
            ('odometry/imu', '/odometry/imu_incremental'),
        ],
        condition=IfCondition(use_lio_sam),
    )

    lio_sam_feature_extraction = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_feature_extraction',
        output='screen',
        parameters=[
            lio_sam_params_file,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_lio_sam),
    )

    lio_sam_map_optimization = Node(
        package='lio_sam',
        executable='lio_sam_mapOptmization',
        name='lio_sam_map_optimization',
        output='screen',
        parameters=[
            lio_sam_params_file,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            # GPS 입력 (GPS 팩터로 사용)
            ('gps/fix', '/sensor/gps'),
            # SLAM 오도메트리 출력 -> LocalizationManager 입력
            ('odometry/mapping', '/odometry/slam'),
            # 맵 포인트클라우드 출력
            ('lio_sam/mapping/cloud_registered', '/slam/cloud_registered'),
            # 키프레임 포즈 출력
            ('lio_sam/mapping/path', '/slam/path'),
        ],
        condition=IfCondition(use_lio_sam),
    )

    # LIO-SAM 노드 그룹
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

    # ---------------------
    # Nav2 Bringup (내비게이션 스택)
    # ---------------------
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

    # ---------------------
    # LaunchDescription 조합
    # ---------------------
    return LaunchDescription([
        # Launch 인자 선언
        declare_use_sim_time,
        declare_map,
        declare_params_file,
        declare_ekf_params_file,
        declare_autostart,
        declare_use_composition,
        declare_use_lio_sam,
        declare_lio_sam_params_file,

        # robot_localization 노드 그룹 (위치 추정)
        GroupAction(
            actions=[
                SetParameter('use_sim_time', use_sim_time),
                ekf_local_node,
                ekf_global_node,
                navsat_transform_node,
            ],
        ),

        # LIO-SAM SLAM 노드 그룹 (조건부 실행)
        lio_sam_group,

        # Nav2 내비게이션 스택 (경로 계획 + 추종)
        nav2_bringup_launch,
    ])
