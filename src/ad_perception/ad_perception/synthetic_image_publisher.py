"""합성 과수원 이미지 퍼블리셔 — Gazebo 없이 perception 파이프라인 검증.

시나리오별 합성 이미지를 생성하여 /sensor/camera/front 토픽에 발행한다.
crop_row_detector의 HSV 필터링을 실제로 통과하도록 색상값을 설정.

시나리오:
  center      — 두 행 사이 중앙 (steering ≈ 0)
  offset_left — 왼쪽 치우침 (우측 조향 필요)
  offset_right— 오른쪽 치우침 (좌측 조향 필요)
  no_rows     — 빈 필드 (row_detected = 0)
  end_of_row  — 행 끝 (초목 페이드아웃)
  single_row  — 행 1개만

실행:
  ros2 run ad_perception synthetic_image_publisher
  ros2 run ad_perception synthetic_image_publisher --ros-args -p scenario:=offset_left
  ros2 run ad_perception synthetic_image_publisher --ros-args -p auto_cycle:=true
"""

from __future__ import annotations

import time
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# 이미지 크기 (model.sdf 카메라와 동일)
IMG_W = 640
IMG_H = 480

# ── 색상 팔레트 (RGB) ──
# PEAR 프로파일 HSV(25-85, 25-255, 35-255)를 통과하는 초록색
COLOR_CANOPY = (45, 140, 45)     # 진한 초록 (나무 캐노피)
COLOR_CANOPY_LIGHT = (80, 180, 60)  # 밝은 초록
COLOR_TRUNK = (110, 75, 45)      # 갈색 (나무 줄기)
COLOR_SOIL = (165, 110, 70)      # 적갈색 토양 (ExG 음수, 노이즈에도 초목 오탐 방지)
COLOR_SKY = (130, 150, 200)      # 하늘색 (ExG 음수)

# 시나리오 목록
SCENARIOS = ['center', 'offset_left', 'offset_right', 'no_rows', 'end_of_row', 'single_row']


class SyntheticImagePublisher(Node):
    """합성 과수원 이미지를 생성·발행하는 ROS2 노드."""

    def __init__(self):
        super().__init__('synthetic_image_publisher')

        # 파라미터
        self.declare_parameter('scenario', 'center')
        self.declare_parameter('publish_rate', 5.0)  # Hz (SDF 카메라와 동일)
        self.declare_parameter('auto_cycle', False)   # 시나리오 자동 순환
        self.declare_parameter('cycle_interval', 5.0)  # 순환 간격 (초)
        self.declare_parameter('topic', '/sensor/camera/front')

        self._scenario = self.get_parameter('scenario').value
        rate = self.get_parameter('publish_rate').value
        self._auto_cycle = self.get_parameter('auto_cycle').value
        self._cycle_interval = self.get_parameter('cycle_interval').value
        topic = self.get_parameter('topic').value

        # 퍼블리셔
        self._pub = self.create_publisher(Image, topic, 10)
        self._timer = self.create_timer(1.0 / rate, self._publish)

        # 자동 순환
        self._cycle_idx = 0
        self._last_cycle_time = time.monotonic()

        self.get_logger().info(
            f'[합성 이미지] 시작 — 시나리오={self._scenario}, '
            f'{rate}Hz, 토픽={topic}, 자동순환={self._auto_cycle}'
        )

    def _publish(self) -> None:
        """타이머 콜백: 합성 이미지 생성·발행."""
        # 자동 순환
        if self._auto_cycle:
            now = time.monotonic()
            if now - self._last_cycle_time >= self._cycle_interval:
                self._cycle_idx = (self._cycle_idx + 1) % len(SCENARIOS)
                self._scenario = SCENARIOS[self._cycle_idx]
                self._last_cycle_time = now
                self.get_logger().info(f'[합성 이미지] 시나리오 전환 → {self._scenario}')

        # 이미지 생성
        image = generate_orchard_image(self._scenario)

        # ROS2 Image 메시지 생성
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = IMG_H
        msg.width = IMG_W
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = IMG_W * 3
        msg.data = image.tobytes()

        self._pub.publish(msg)


# ══════════════════════════════════════════════════════════════
# 합성 이미지 생성 함수
# ══════════════════════════════════════════════════════════════


def _fill_rect(img: np.ndarray, y1: int, y2: int, x1: int, x2: int,
               color: Tuple[int, int, int]) -> None:
    """이미지 영역을 색상으로 채운다."""
    y1, y2 = max(0, y1), min(img.shape[0], y2)
    x1, x2 = max(0, x1), min(img.shape[1], x2)
    img[y1:y2, x1:x2] = color


def _draw_tree_row(img: np.ndarray, center_x: int, width: int,
                   sky_end: int, ground_start: int,
                   fade: float = 1.0) -> None:
    """나무 행 1열을 그린다 (줄기 + 캐노피).

    Args:
        center_x: 행 중심 X 좌표
        width: 행 전체 폭 (px)
        sky_end: 하늘-나무 경계 Y
        ground_start: 나무-땅 경계 Y
        fade: 투명도 (0=안보임, 1=완전)
    """
    half_w = width // 2
    x1 = center_x - half_w
    x2 = center_x + half_w

    # 줄기 (하단 30%)
    trunk_top = ground_start - int((ground_start - sky_end) * 0.3)
    trunk_w = max(4, width // 5)
    trunk_x1 = center_x - trunk_w // 2
    trunk_x2 = center_x + trunk_w // 2

    # 캐노피 (상단 80%)
    canopy_bottom = trunk_top + int((ground_start - trunk_top) * 0.2)

    if fade < 1.0:
        # 페이드: 배경색과 블렌딩
        canopy_color = tuple(int(c * fade + COLOR_SOIL[i] * (1 - fade))
                             for i, c in enumerate(COLOR_CANOPY))
        trunk_color = tuple(int(c * fade + COLOR_SOIL[i] * (1 - fade))
                            for i, c in enumerate(COLOR_TRUNK))
    else:
        canopy_color = COLOR_CANOPY
        trunk_color = COLOR_TRUNK

    # 캐노피 그리기 (약간의 볼륨감)
    _fill_rect(img, sky_end, canopy_bottom, x1, x2, canopy_color)

    # 캐노피 밝은 부분 (중앙 상단)
    if fade >= 0.5:
        light_w = max(2, width // 3)
        light_x1 = center_x - light_w // 2
        light_x2 = center_x + light_w // 2
        light_top = sky_end + int((canopy_bottom - sky_end) * 0.1)
        light_bottom = sky_end + int((canopy_bottom - sky_end) * 0.5)
        light_color = tuple(int(c * fade + COLOR_SOIL[i] * (1 - fade))
                            for i, c in enumerate(COLOR_CANOPY_LIGHT))
        _fill_rect(img, light_top, light_bottom, light_x1, light_x2, light_color)

    # 줄기 그리기
    _fill_rect(img, trunk_top, ground_start, trunk_x1, trunk_x2, trunk_color)


def _add_noise(img: np.ndarray, intensity: int = 8) -> np.ndarray:
    """이미지에 약간의 노이즈를 추가한다 (센서 시뮬레이션)."""
    rng = np.random.default_rng()
    noise = rng.integers(-intensity, intensity + 1, size=img.shape, dtype=np.int16)
    noisy = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    return noisy


def generate_orchard_image(scenario: str) -> np.ndarray:
    """시나리오에 따른 합성 과수원 이미지 생성.

    Args:
        scenario: center, offset_left, offset_right, no_rows, end_of_row, single_row

    Returns:
        (480, 640, 3) uint8 RGB 이미지
    """
    img = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)

    # 배경: 하늘 (상단 20%) + 토양 (하단 80%)
    sky_end = int(IMG_H * 0.2)  # 96px
    _fill_rect(img, 0, sky_end, 0, IMG_W, COLOR_SKY)
    _fill_rect(img, sky_end, IMG_H, 0, IMG_W, COLOR_SOIL)

    # 나무 영역: sky_end ~ ground_start
    ground_start = int(IMG_H * 0.85)  # 408px
    row_width = 80  # 나무 행 폭 (px)

    if scenario == 'center':
        # 두 행이 이미지 중앙 좌우에 대칭
        _draw_tree_row(img, IMG_W // 2 - 120, row_width, sky_end, ground_start)
        _draw_tree_row(img, IMG_W // 2 + 120, row_width, sky_end, ground_start)

    elif scenario == 'offset_left':
        # 로봇이 왼쪽으로 치우침 → 행이 오른쪽에 몰림
        _draw_tree_row(img, IMG_W // 2 + 20, row_width, sky_end, ground_start)
        _draw_tree_row(img, IMG_W // 2 + 260, row_width, sky_end, ground_start)

    elif scenario == 'offset_right':
        # 로봇이 오른쪽으로 치우침 → 행이 왼쪽에 몰림
        _draw_tree_row(img, IMG_W // 2 - 260, row_width, sky_end, ground_start)
        _draw_tree_row(img, IMG_W // 2 - 20, row_width, sky_end, ground_start)

    elif scenario == 'no_rows':
        # 빈 필드 (나무 없음) — 토양만
        pass

    elif scenario == 'end_of_row':
        # 행 끝: 나무가 완전히 사라진 구간 (토양만 보임)
        # center → end_of_row 전환으로 end_detected 로직 테스트
        # (연속 N프레임 행 미감지 → end_detected=1.0)
        pass

    elif scenario == 'single_row':
        # 행 1개만 (이미지 중앙 약간 왼쪽)
        _draw_tree_row(img, IMG_W // 2 - 80, row_width, sky_end, ground_start)

    else:
        # 알 수 없는 시나리오 → center 기본값
        _draw_tree_row(img, IMG_W // 2 - 120, row_width, sky_end, ground_start)
        _draw_tree_row(img, IMG_W // 2 + 120, row_width, sky_end, ground_start)

    # 센서 노이즈
    img = _add_noise(img)

    return img


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[합성 이미지] 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
