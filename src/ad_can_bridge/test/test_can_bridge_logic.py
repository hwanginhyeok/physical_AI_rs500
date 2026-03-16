"""CAN 브릿지 로직 테스트.

C68: Twist → CAN 명령 변환 로직을 ROS2 없이 검증.
- Twist → 좌/우 트랙 속도 → ADT2VCU1 인코딩
- 속도 단위 변환 (m/s → km/h)
- Alive Counter 순환
- 비상정지 처리
"""

import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent / 'ad_core'))
sys.path.insert(0, str(Path(__file__).parent.parent))

from ad_core.skid_steer_model import SkidSteerModel
from ad_can_bridge.ss500_codec import (
    ADT2VCU1,
    SS500Encoder,
    SS500Decoder,
)


class FakeCANBridge:
    """CAN 브릿지 노드의 로직을 ROS2 없이 재현."""

    def __init__(self, track_width=1.4, max_speed=1.5):
        self._skid_steer = SkidSteerModel(
            track_width=track_width, max_speed=max_speed
        )
        self._encoder = SS500Encoder()
        self._alive_counter = 0

    def twist_to_can(self, linear_x: float, angular_z: float) -> bytes:
        """Twist → CAN 바이트 변환."""
        left_ms, right_ms = self._skid_steer.twist_to_tracks(
            linear_x, angular_z
        )

        adt_msg = ADT2VCU1(
            adt_valid=1,
            auto_ctrl_enable=1,
            left_speed_cmd_kmh=left_ms * 3.6,
            right_speed_cmd_kmh=right_ms * 3.6,
            alive_counter=self._alive_counter,
        )

        self._alive_counter = (self._alive_counter + 1) & 0x0F
        return self._encoder.encode_adt2vcu1(adt_msg)

    def decode_speeds_kmh(self, can_data: bytes):
        """CAN 바이트 → (left_kmh, right_kmh)."""
        left_raw = struct.unpack_from('<h', can_data, 2)[0]
        right_raw = struct.unpack_from('<h', can_data, 4)[0]
        return left_raw * 0.01, right_raw * 0.01


class TestTwistToTrackConversion:
    """Twist → 좌/우 트랙 속도 변환."""

    def test_straight_forward(self):
        """직진: 좌우 동일 속도."""
        bridge = FakeCANBridge()
        data = bridge.twist_to_can(1.0, 0.0)
        left_kmh, right_kmh = bridge.decode_speeds_kmh(data)
        assert abs(left_kmh - right_kmh) < 0.1
        assert left_kmh > 0

    def test_straight_backward(self):
        """후진: 좌우 동일 음의 속도."""
        bridge = FakeCANBridge()
        data = bridge.twist_to_can(-0.5, 0.0)
        left_kmh, right_kmh = bridge.decode_speeds_kmh(data)
        assert left_kmh < 0
        assert abs(left_kmh - right_kmh) < 0.1

    def test_left_turn(self):
        """좌회전: 좌측 < 우측."""
        bridge = FakeCANBridge()
        data = bridge.twist_to_can(1.0, 0.5)
        left_kmh, right_kmh = bridge.decode_speeds_kmh(data)
        assert left_kmh < right_kmh, f"좌회전 시 L({left_kmh}) < R({right_kmh}) 이어야 함"

    def test_right_turn(self):
        """우회전: 좌측 > 우측."""
        bridge = FakeCANBridge()
        data = bridge.twist_to_can(1.0, -0.5)
        left_kmh, right_kmh = bridge.decode_speeds_kmh(data)
        assert left_kmh > right_kmh, f"우회전 시 L({left_kmh}) > R({right_kmh}) 이어야 함"

    def test_pivot_left(self):
        """제자리 좌회전: 좌측 음, 우측 양."""
        bridge = FakeCANBridge()
        data = bridge.twist_to_can(0.0, 1.0)
        left_kmh, right_kmh = bridge.decode_speeds_kmh(data)
        assert left_kmh < 0, f"좌 피봇 시 L({left_kmh}) 음수"
        assert right_kmh > 0, f"좌 피봇 시 R({right_kmh}) 양수"

    def test_pivot_right(self):
        """제자리 우회전: 좌측 양, 우측 음."""
        bridge = FakeCANBridge()
        data = bridge.twist_to_can(0.0, -1.0)
        left_kmh, right_kmh = bridge.decode_speeds_kmh(data)
        assert left_kmh > 0
        assert right_kmh < 0

    def test_stop(self):
        """정지: 좌우 모두 0."""
        bridge = FakeCANBridge()
        data = bridge.twist_to_can(0.0, 0.0)
        left_kmh, right_kmh = bridge.decode_speeds_kmh(data)
        assert abs(left_kmh) < 0.01
        assert abs(right_kmh) < 0.01


class TestUnitConversion:
    """m/s → km/h 변환 정확도."""

    def test_1_ms_to_kmh(self):
        """1 m/s = 3.6 km/h."""
        bridge = FakeCANBridge()
        data = bridge.twist_to_can(1.0, 0.0)
        left_kmh, right_kmh = bridge.decode_speeds_kmh(data)
        assert abs(left_kmh - 3.6) < 0.1

    def test_slow_speed(self):
        """0.1 m/s = 0.36 km/h."""
        bridge = FakeCANBridge()
        data = bridge.twist_to_can(0.1, 0.0)
        left_kmh, right_kmh = bridge.decode_speeds_kmh(data)
        assert abs(left_kmh - 0.36) < 0.05


class TestAliveCounter:
    """Alive Counter 순환."""

    def test_counter_increments(self):
        bridge = FakeCANBridge()
        data1 = bridge.twist_to_can(0.0, 0.0)
        data2 = bridge.twist_to_can(0.0, 0.0)
        # alive counter는 byte 7의 upper nibble
        counter1 = (data1[7] >> 4) & 0x0F
        counter2 = (data2[7] >> 4) & 0x0F
        assert counter1 == 0
        assert counter2 == 1

    def test_counter_wraps_at_16(self):
        bridge = FakeCANBridge()
        for _ in range(16):
            bridge.twist_to_can(0.0, 0.0)
        data = bridge.twist_to_can(0.0, 0.0)
        counter = (data[7] >> 4) & 0x0F
        assert counter == 0  # 16 → 0으로 순환


class TestCANFlags:
    """CAN 플래그 설정."""

    def test_valid_and_enable_flags(self):
        """adt_valid=1, auto_ctrl_enable=1 항상 설정."""
        bridge = FakeCANBridge()
        data = bridge.twist_to_can(0.0, 0.0)
        assert data[0] & 0x01 == 1  # adt_valid
        assert (data[0] >> 1) & 0x01 == 1  # auto_ctrl_enable


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
