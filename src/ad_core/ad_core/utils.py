"""AD Core 공통 유틸리티 모듈.

수학/기하학 유틸리티 함수들을 제공한다.
"""

import math

# 상수 정의
EPSILON = 1e-9  # 부동소수점 비교용
EPSILON_SQ = 1e-12  # 제곱 연산용
DEFAULT_TOLERANCE = 1e-6  # 일반적 허용 오차


def normalize_angle(angle: float) -> float:
    """각도를 [-pi, pi) 범위로 정규화한다.

    Args:
        angle: 입력 각도 (rad).

    Returns:
        정규화된 각도 (rad).

    Examples:
        >>> normalize_angle(3.5 * math.pi)
        -0.5 * math.pi
        >>> normalize_angle(-4 * math.pi)
        0.0
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def normalize_angle_pi(angle: float) -> float:
    """각도를 [0, 2*pi) 범위로 정규화한다.

    Args:
        angle: 입력 각도 (rad).

    Returns:
        정규화된 각도 (rad), 항상 0 이상.
    """
    angle = normalize_angle(angle)
    if angle < 0:
        angle += 2.0 * math.pi
    return angle


def clamp(value: float, min_val: float, max_val: float) -> float:
    """값을 주어진 범위로 클램프한다.

    Args:
        value: 입력 값.
        min_val: 최소값.
        max_val: 최대값.

    Returns:
        클램프된 값.

    Examples:
        >>> clamp(5.0, 0.0, 10.0)
        5.0
        >>> clamp(-1.0, 0.0, 10.0)
        0.0
        >>> clamp(15.0, 0.0, 10.0)
        10.0
    """
    return max(min_val, min(value, max_val))


def lerp(a: float, b: float, t: float) -> float:
    """선형 보간(Linear Interpolation)을 수행한다.

    Args:
        a: 시작 값.
        b: 종료 값.
        t: 보간 비율 (0.0 ~ 1.0).

    Returns:
        보간된 값: a + t * (b - a)

    Examples:
        >>> lerp(0.0, 10.0, 0.5)
        5.0
    """
    return a + t * (b - a)


def distance_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """2차원 유클리드 거리를 계산한다.

    Args:
        x1: 첫 번째 점의 x 좌표.
        y1: 첫 번째 점의 y 좌표.
        x2: 두 번째 점의 x 좌표.
        y2: 두 번째 점의 y 좌표.

    Returns:
        두 점 사이의 거리.
    """
    return math.hypot(x2 - x1, y2 - y1)


def distance_sq_2d(x1: float, y1: float, x2: float, y2: float) -> float:
    """2차원 유클리드 거리의 제곱을 계산한다.

    sqrt 연산을 피하므로 거리 비교 시 더 효율적이다.

    Args:
        x1: 첫 번째 점의 x 좌표.
        y1: 첫 번째 점의 y 좌표.
        x2: 두 번째 점의 x 좌표.
        y2: 두 번째 점의 y 좌표.

    Returns:
        두 점 사이의 거리의 제곱.
    """
    dx = x2 - x1
    dy = y2 - y1
    return dx * dx + dy * dy
