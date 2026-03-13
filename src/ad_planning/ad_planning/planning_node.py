"""Learned Planning ROS2 Node.

Diffusion-based trajectory generation integrated with Safety Guardian.
"""

import rclpy
from rclpy.node import Node
from typing import Optional
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

# ROS2 메시지
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Header

# 커스텀 임포트
import sys
from pathlib import Path as FilePath
sys.path.insert(0, str(FilePath(__file__).parent.parent.parent / 'ad_core'))
sys.path.insert(0, str(FilePath(__file__).parent.parent.parent / 'ad_control'))

from ad_core.hybrid_e2e_types import PerceptionFeatures, TerrainClass
from ad_core.datatypes import Pose2D
from ad_control.safety_guardian import SafetyGuardian, GuardianConfig

from .diffusion_planner import DiffusionPlanner, PlannerConfig


class PlanningNode(Node):
    """Learned Planning Node.
    
    Subscribers:
        - /perception/features (PerceptionFeatures)
        - /vehicle/pose (PoseStamped)
        - /vehicle/speed (Twist)
    
    Publishers:
        - /planning/trajectory (Path)
        - /planning/candidate_paths (Path[]) - debug
        - /planning/guardian_decision (string)
    """
    
    def __init__(self):
        super().__init__('learned_planning_node')
        
        # 파라미터 선언
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('diffusion_steps', 20)
        self.declare_parameter('num_candidates', 3)
        self.declare_parameter('planning_rate', 10.0)  # Hz
        self.declare_parameter('safety_enabled', True)
        self.declare_parameter('use_rule_based_fallback', True)
        
        # 파라미터 읽기
        device = self.get_parameter('device').value
        diffusion_steps = self.get_parameter('diffusion_steps').value
        self.num_candidates = self.get_parameter('num_candidates').value
        planning_rate = self.get_parameter('planning_rate').value
        safety_enabled = self.get_parameter('safety_enabled').value
        
        self.get_logger().info('LearnedPlanning initializing...')
        self.get_logger().info(f'  Device: {device}')
        self.get_logger().info(f'  Diffusion steps: {diffusion_steps}')
        self.get_logger().info(f'  Planning rate: {planning_rate} Hz')
        
        # Planner 초기화
        planner_config = PlannerConfig(
            device=device,
            num_diffusion_steps=diffusion_steps,
            num_waypoints=20,
            prediction_horizon=10.0
        )
        self.planner = DiffusionPlanner(planner_config)
        
        # Safety Guardian 초기화
        if safety_enabled:
            guardian_config = GuardianConfig(
                max_speed=1.5,
                max_slope=0.3,
                min_crop_distance=0.3
            )
            self.guardian = SafetyGuardian(guardian_config)
        else:
            self.guardian = None
        
        # 상태
        self.current_perception = None
        self.current_pose = None
        self.current_speed = 0.0
        self.last_planning_time = self.get_clock().now()
        
        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        # TODO: PerceptionFeatures용 custom msg 필요
        # self.perception_sub = self.create_subscription(
        #     PerceptionFeaturesMsg,
        #     '/perception/features',
        #     self._perception_callback,
        #     qos
        # )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/vehicle/pose',
            self._pose_callback,
            qos
        )
        
        self.twist_sub = self.create_subscription(
            Twist,
            '/vehicle/twist',
            self._twist_callback,
            qos
        )
        
        # Publishers
        self.trajectory_pub = self.create_publisher(
            Path,
            '/planning/trajectory',
            10
        )
        
        self.debug_paths_pub = self.create_publisher(
            Path,
            '/planning/debug/candidates',
            10
        )
        
        # Planning 타이머
        period = 1.0 / planning_rate
        self.timer = self.create_timer(period, self._planning_loop)
        
        self.get_logger().info('LearnedPlanning initialized successfully')
    
    def _pose_callback(self, msg: PoseStamped):
        """위치 수신."""
        self.current_pose = Pose2D(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            yaw=self._quaternion_to_yaw(msg.pose.orientation)
        )
    
    def _twist_callback(self, msg: Twist):
        """속도 수신."""
        self.current_speed = np.sqrt(msg.linear.x**2 + msg.linear.y**2)
    
    def _quaternion_to_yaw(self, q) -> float:
        """쿼터니언에서 yaw 각도 추출."""
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny, cosy)
    
    def _planning_loop(self):
        """주기적 경로 생성."""
        if self.current_pose is None:
            self.get_logger().debug('Waiting for pose...')
            return
        
        # 테스트용 PerceptionFeatures (실제로는 토픽에서 수신)
        if self.current_perception is None:
            self.current_perception = PerceptionFeatures(
                terrain_type=TerrainClass.CROP_FIELD,
                crop_row_confidence=0.8,
                overall_confidence=0.9,
                slope_gradient=0.0
            )
        
        # 경로 생성
        start_time = self.get_clock().now()
        
        try:
            candidate_trajectories = self.planner.generate(
                perception_features=self.current_perception,
                current_pose=self.current_pose,
                current_speed=self.current_speed,
                num_samples=self.num_candidates
            )
            
            # Safety Guardian 검증
            if self.guardian:
                valid_trajectory = self._select_safe_trajectory(
                    candidate_trajectories,
                    self.current_perception
                )
            else:
                valid_trajectory = candidate_trajectories[0] if candidate_trajectories else None
            
            if valid_trajectory:
                # 퍼블리시
                self._publish_trajectory(valid_trajectory)
                self._publish_debug_candidates(candidate_trajectories)
            
            # 처리 시간 로깅
            processing_time = (self.get_clock().now() - start_time).nanoseconds / 1e6
            self.get_logger().debug(f'Planning time: {processing_time:.1f}ms')
            
        except Exception as e:
            self.get_logger().error(f'Planning error: {e}')
    
    def _select_safe_trajectory(
        self,
        candidates: list,
        perception: PerceptionFeatures
    ) -> Optional[dict]:
        """Safety Guardian으로 안전한 경로 선택."""
        from ad_core.hybrid_e2e_types import PlannedTrajectory, TrajectoryPoint
        
        for candidate in candidates:
            # PlannedTrajectory 변환
            waypoints = []
            for wp in candidate['waypoints']:
                waypoints.append(TrajectoryPoint(
                    pose=Pose2D(x=wp['x'], y=wp['y'], yaw=wp['yaw']),
                    velocity=wp['velocity'],
                    acceleration=0.0,
                    curvature=0.0,
                    timestamp=wp['timestamp']
                ))
            
            planned = PlannedTrajectory(
                waypoints=waypoints,
                confidence=candidate['confidence']
            )
            
            # Guardian 검증
            decision = self.guardian.validate(
                planned,
                self.current_pose,
                perception,
                self.current_speed
            )
            
            if decision.is_safe:
                self.get_logger().debug(f'Selected trajectory with score {decision.safety_score:.2f}')
                return candidate
            else:
                self.get_logger().debug(f'Trajectory rejected: {decision.reason}')
        
        self.get_logger().warn('No safe trajectory found!')
        return None
    
    def _publish_trajectory(self, trajectory: dict):
        """경로 퍼블리시."""
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        
        for wp in trajectory['waypoints']:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.position.z = 0.0
            # yaw to quaternion (simplified)
            pose.pose.orientation.w = np.cos(wp['yaw'] / 2)
            pose.pose.orientation.z = np.sin(wp['yaw'] / 2)
            path.poses.append(pose)
        
        self.trajectory_pub.publish(path)
    
    def _publish_debug_candidates(self, candidates: list):
        """디버그용 후보 경로 퍼블리시 (첫 번째만)."""
        if not candidates:
            return
        
        # 첫 번째 후보를 별도 토픽으로
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        
        for wp in candidates[0]['waypoints']:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.position.z = 0.1  # 약간 띄움
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        self.debug_paths_pub.publish(path)
    
    def destroy_node(self):
        """종료 정리."""
        self.get_logger().info('Shutting down LearnedPlanning...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
