"""Waypoint Manager Node.

Foxglove Studio에서 맵 위를 클릭하면 /goal_pose로 PoseStamped가 발행되고,
이 노드가 웨이포인트를 수집한 뒤 Nav2 FollowWaypoints 액션으로 전달한다.

토픽:
    구독: /goal_pose (PoseStamped)
    발행: /waypoint_manager/markers (MarkerArray) — 번호 마커
          /waypoint_manager/status (String) — JSON 상태

서비스:
    /waypoint_manager/start   — 수집된 웨이포인트로 네비게이션 시작
    /waypoint_manager/clear   — 웨이포인트 목록 초기화
    /waypoint_manager/cancel  — 진행 중인 네비게이션 취소
    /waypoint_manager/remove_last — 마지막 웨이포인트 제거

액션 클라이언트:
    /follow_waypoints (FollowWaypoints)
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA
from std_srvs.srv import Trigger, Trigger_Request, Trigger_Response
from nav2_msgs.action import FollowWaypoints, NavigateThroughPoses
from rclpy.action.client import ClientGoalHandle

from builtin_interfaces.msg import Duration


class WaypointManagerNode(Node):
    """Foxglove 클릭 → 웨이포인트 수집 → Nav2 FollowWaypoints 전달."""

    def __init__(self):
        super().__init__('waypoint_manager')

        # 파라미터
        self.declare_parameter('mode', 'follow_waypoints')  # or 'navigate_through_poses'
        self.declare_parameter('marker_scale', 0.5)
        self.declare_parameter('marker_height', 2.0)

        self._mode = self.get_parameter('mode').get_parameter_value().string_value
        self._marker_scale = self.get_parameter('marker_scale').get_parameter_value().double_value
        self._marker_height = self.get_parameter('marker_height').get_parameter_value().double_value

        # 상태
        self._waypoints: list[PoseStamped] = []
        self._navigating = False
        self._current_waypoint_idx = 0

        # 콜백 그룹 (서비스 + 액션 피드백이 동시에 처리되도록)
        self._cb_group = ReentrantCallbackGroup()

        # 구독: Foxglove 3D 패널 클릭 → /goal_pose
        self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_pose_cb, 10,
            callback_group=self._cb_group,
        )

        # 발행
        self._marker_pub = self.create_publisher(
            MarkerArray, '/waypoint_manager/markers', 10,
        )
        self._status_pub = self.create_publisher(
            String, '/waypoint_manager/status', 10,
        )

        # 서비스
        self.create_service(
            Trigger, '/waypoint_manager/start', self._start_cb,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, '/waypoint_manager/clear', self._clear_cb,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, '/waypoint_manager/cancel', self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, '/waypoint_manager/remove_last', self._remove_last_cb,
            callback_group=self._cb_group,
        )

        # 액션 클라이언트
        self._follow_wp_client = ActionClient(
            self, FollowWaypoints, '/follow_waypoints',
            callback_group=self._cb_group,
        )
        self._nav_through_client = ActionClient(
            self, NavigateThroughPoses, '/navigate_through_poses',
            callback_group=self._cb_group,
        )

        # 현재 액션 goal handle
        self._goal_handle = None

        # 주기적 상태 발행 (2Hz)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            f'WaypointManager 시작 (mode={self._mode})'
        )

    # ------------------------------------------------------------------
    # 구독 콜백
    # ------------------------------------------------------------------

    def _goal_pose_cb(self, msg: PoseStamped) -> None:
        """Foxglove 클릭으로 들어온 goal_pose를 웨이포인트로 추가."""
        if self._navigating:
            self.get_logger().warn('네비게이션 진행 중 — 웨이포인트 추가 무시')
            return

        # 프레임 고정 (map)
        msg.header.frame_id = 'map'
        self._waypoints.append(msg)

        idx = len(self._waypoints)
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f'웨이포인트 #{idx} 추가: ({x:.2f}, {y:.2f})')
        self._publish_markers()

    # ------------------------------------------------------------------
    # 서비스 콜백
    # ------------------------------------------------------------------

    def _start_cb(self, request: Trigger_Request, response: Trigger_Response) -> Trigger_Response:
        """수집된 웨이포인트로 네비게이션 시작."""
        if self._navigating:
            response.success = False
            response.message = '이미 네비게이션 진행 중'
            return response

        if not self._waypoints:
            response.success = False
            response.message = '웨이포인트가 없습니다'
            return response

        self._navigating = True
        self._current_waypoint_idx = 0

        if self._mode == 'navigate_through_poses':
            self._send_navigate_through_poses()
        else:
            self._send_follow_waypoints()

        response.success = True
        response.message = f'{len(self._waypoints)}개 웨이포인트 네비게이션 시작'
        return response

    def _clear_cb(self, request: Trigger_Request, response: Trigger_Response) -> Trigger_Response:
        """웨이포인트 목록 초기화."""
        if self._navigating:
            response.success = False
            response.message = '네비게이션 진행 중에는 초기화 불가 (먼저 cancel)'
            return response

        self._waypoints.clear()
        self._publish_markers()
        response.success = True
        response.message = '웨이포인트 초기화 완료'
        self.get_logger().info('웨이포인트 초기화')
        return response

    def _cancel_cb(self, request: Trigger_Request, response: Trigger_Response) -> Trigger_Response:
        """진행 중인 네비게이션 취소."""
        if not self._navigating or self._goal_handle is None:
            response.success = False
            response.message = '취소할 네비게이션 없음'
            return response

        self.get_logger().info('네비게이션 취소 요청')
        self._goal_handle.cancel_goal_async()
        response.success = True
        response.message = '네비게이션 취소 요청 전송'
        return response

    def _remove_last_cb(self, request: Trigger_Request, response: Trigger_Response) -> Trigger_Response:
        """마지막 웨이포인트 제거."""
        if self._navigating:
            response.success = False
            response.message = '네비게이션 진행 중에는 제거 불가'
            return response

        if not self._waypoints:
            response.success = False
            response.message = '제거할 웨이포인트 없음'
            return response

        removed = self._waypoints.pop()
        self._publish_markers()
        x = removed.pose.position.x
        y = removed.pose.position.y
        response.success = True
        response.message = f'웨이포인트 제거: ({x:.2f}, {y:.2f}), 남은 {len(self._waypoints)}개'
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------
    # Nav2 액션 전송
    # ------------------------------------------------------------------

    def _wait_for_action_server(self, client, name: str, 
                                  timeout_sec: float = 5.0, 
                                  retries: int = 3) -> bool:
        """액션 서버 연결을 재시도하며 대기한다.

        Args:
            client: ActionClient 인스턴스.
            name: 액션 서버 이름 (로깅용).
            timeout_sec: 각 시도별 타임아웃 (초).
            retries: 재시도 횟수.

        Returns:
            연결 성공 여부.
        """
        for attempt in range(retries):
            if client.wait_for_server(timeout_sec=timeout_sec):
                return True
            self.get_logger().warn(
                f'{name} 액션 서버 대기 중... ({attempt + 1}/{retries})'
            )
        return False

    def _send_follow_waypoints(self):
        """FollowWaypoints 액션 전송."""
        if not self._wait_for_action_server(
            self._follow_wp_client, 'FollowWaypoints', timeout_sec=5.0, retries=3
        ):
            self.get_logger().error('FollowWaypoints 액션 서버 연결 실패')
            self._navigating = False
            return

        goal = FollowWaypoints.Goal()
        goal.poses = list(self._waypoints)

        self.get_logger().info(
            f'FollowWaypoints 전송: {len(goal.poses)}개 웨이포인트'
        )
        future = self._follow_wp_client.send_goal_async(
            goal, feedback_callback=self._follow_wp_feedback_cb,
        )
        future.add_done_callback(self._follow_wp_goal_response_cb)

    def _follow_wp_goal_response_cb(self, future) -> None:
        """FollowWaypoints goal 응답 처리."""
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('FollowWaypoints goal 거부됨')
            self._navigating = False
            return

        self.get_logger().info('FollowWaypoints goal 수락됨')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._follow_wp_result_cb)

    def _follow_wp_feedback_cb(self, feedback_msg) -> None:
        """FollowWaypoints 피드백 처리."""
        self._current_waypoint_idx = feedback_msg.feedback.current_waypoint
        self._publish_markers()

    def _follow_wp_result_cb(self, future) -> None:
        """FollowWaypoints 결과 처리."""
        result = future.result()
        status = result.status

        # action_msgs/GoalStatus 상수
        # STATUS_SUCCEEDED = 4, STATUS_CANCELED = 5, STATUS_ABORTED = 6
        if status == 4:  # SUCCEEDED
            missed = result.result.missed_waypoints
            if missed:
                self.get_logger().warn(f'완료 (실패 웨이포인트: {list(missed)})')
            else:
                self.get_logger().info('모든 웨이포인트 도달 완료!')
        elif status == 5:  # CANCELED
            self.get_logger().info('네비게이션 취소됨')
        else:
            self.get_logger().error(f'네비게이션 실패 (status={status})')

        self._navigating = False
        self._goal_handle = None
        self._current_waypoint_idx = 0
        self._publish_markers()

    def _send_navigate_through_poses(self):
        """NavigateThroughPoses 액션 전송."""
        if not self._wait_for_action_server(
            self._nav_through_client, 'NavigateThroughPoses', timeout_sec=5.0, retries=3
        ):
            self.get_logger().error('NavigateThroughPoses 액션 서버 연결 실패')
            self._navigating = False
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = list(self._waypoints)

        self.get_logger().info(
            f'NavigateThroughPoses 전송: {len(goal.poses)}개 포즈'
        )
        future = self._nav_through_client.send_goal_async(
            goal, feedback_callback=self._nav_through_feedback_cb,
        )
        future.add_done_callback(self._nav_through_goal_response_cb)

    def _nav_through_goal_response_cb(self, future) -> None:
        """NavigateThroughPoses goal 응답 처리."""
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('NavigateThroughPoses goal 거부됨')
            self._navigating = False
            return

        self.get_logger().info('NavigateThroughPoses goal 수락됨')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_through_result_cb)

    def _nav_through_feedback_cb(self, feedback_msg) -> None:
        """NavigateThroughPoses 피드백 — 남은 포즈 수로 진행률 추정."""
        remaining = feedback_msg.feedback.number_of_poses_remaining
        total = len(self._waypoints)
        self._current_waypoint_idx = max(0, total - remaining)
        self._publish_markers()

    def _nav_through_result_cb(self, future) -> None:
        """NavigateThroughPoses 결과 처리."""
        result = future.result()
        status = result.status

        if status == 4:
            self.get_logger().info('NavigateThroughPoses 완료!')
        elif status == 5:
            self.get_logger().info('NavigateThroughPoses 취소됨')
        else:
            self.get_logger().error(f'NavigateThroughPoses 실패 (status={status})')

        self._navigating = False
        self._goal_handle = None
        self._current_waypoint_idx = 0
        self._publish_markers()

    # ------------------------------------------------------------------
    # 시각화
    # ------------------------------------------------------------------

    def _publish_markers(self):
        """웨이포인트 위치에 번호 매긴 마커를 발행."""
        marker_array = MarkerArray()

        # 이전 마커 삭제
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = 'map'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        marker_array.markers.append(delete_marker)

        for i, wp in enumerate(self._waypoints):
            # 구체 마커 (위치 표시)
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'waypoints'
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = wp.pose
            sphere.pose.position.z = 0.3
            sphere.scale.x = self._marker_scale
            sphere.scale.y = self._marker_scale
            sphere.scale.z = self._marker_scale
            sphere.lifetime = Duration(sec=0, nanosec=0)

            # 색상: 완료=초록, 현재=노랑, 대기=파랑
            if self._navigating and i < self._current_waypoint_idx:
                sphere.color = ColorRGBA(r=0.2, g=0.9, b=0.2, a=0.9)  # 완료 (초록)
            elif self._navigating and i == self._current_waypoint_idx:
                sphere.color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=0.9)  # 현재 (노랑)
            else:
                sphere.color = ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.9)  # 대기 (파랑)

            marker_array.markers.append(sphere)

            # 텍스트 마커 (번호 표시)
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'waypoint_labels'
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = wp.pose
            text.pose.position.z = self._marker_height
            text.scale.z = 0.6  # 텍스트 높이
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = str(i + 1)
            text.lifetime = Duration(sec=0, nanosec=0)
            marker_array.markers.append(text)

            # 웨이포인트 간 연결선 (ARROW로 방향 표시)
            if i > 0:
                arrow = Marker()
                arrow.header.frame_id = 'map'
                arrow.header.stamp = self.get_clock().now().to_msg()
                arrow.ns = 'waypoint_path'
                arrow.id = 1000 + i
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD

                start = self._waypoints[i - 1].pose.position
                end = wp.pose.position
                arrow.points = [
                    start,
                    end,
                ]
                arrow.points[0].z = 0.15
                arrow.points[1].z = 0.15
                arrow.scale.x = 0.08  # 화살표 몸통 지름
                arrow.scale.y = 0.15  # 화살표 머리 지름
                arrow.scale.z = 0.0
                arrow.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.6)
                arrow.lifetime = Duration(sec=0, nanosec=0)
                marker_array.markers.append(arrow)

        self._marker_pub.publish(marker_array)

    def _publish_status(self):
        """JSON 형태로 현재 상태를 발행."""
        status = {
            'navigating': self._navigating,
            'mode': self._mode,
            'total_waypoints': len(self._waypoints),
            'current_index': self._current_waypoint_idx,
            'waypoints': [
                {
                    'index': i + 1,
                    'x': round(wp.pose.position.x, 2),
                    'y': round(wp.pose.position.y, 2),
                }
                for i, wp in enumerate(self._waypoints)
            ],
        }

        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
