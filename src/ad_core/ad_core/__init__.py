"""ad_core: 순수 알고리즘 패키지 (ROS2 독립).

제어, 인지, 판단에 사용되는 핵심 알고리즘을 제공한다.
ROS2 의존성 없이 순수 Python + numpy만으로 동작하며,
adtpc(실차 배포용)로 그대로 이식할 수 있다.
"""

from ad_core.utils import (
    normalize_angle,
    normalize_angle_pi,
    clamp,
    lerp,
    distance_2d,
    distance_sq_2d,
    EPSILON,
)

__all__ = [
    "normalize_angle",
    "normalize_angle_pi",
    "clamp",
    "lerp",
    "distance_2d",
    "distance_sq_2d",
    "EPSILON",
]
