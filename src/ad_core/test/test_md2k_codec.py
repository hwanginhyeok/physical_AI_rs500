"""MD2K CAN 코덱 단위 테스트."""

import pytest

from ad_core.motor_protection import MotorAlarm
from ad_core.md2k_codec import (
    MD2K_TX_ID,
    MD2K_RX_ID,
    MD2KTxMsg,
    MD2KRxMsg,
    encode_tx,
    decode_tx,
    encode_rx,
    decode_rx,
    rpm_to_track_speed,
    track_speed_to_rpm,
)


class TestConstants:
    def test_can_ids(self):
        assert MD2K_TX_ID == 0x001
        assert MD2K_RX_ID == 0x701


class TestTxCodec:
    def test_roundtrip(self):
        """encode→decode 왕복 일치."""
        msg = MD2KTxMsg(
            mot1_enable=True, mot2_enable=True,
            mot1_vel_req=1500, mot2_vel_req=-800,
        )
        data = encode_tx(msg)
        assert len(data) == 8
        decoded = decode_tx(data)
        assert decoded.mot1_enable == msg.mot1_enable
        assert decoded.mot2_enable == msg.mot2_enable
        assert decoded.mot1_vel_req == msg.mot1_vel_req
        assert decoded.mot2_vel_req == msg.mot2_vel_req

    def test_enable_bits(self):
        """활성화 비트 인코딩."""
        # mot1만 활성
        msg = MD2KTxMsg(mot1_enable=True, mot2_enable=False)
        data = encode_tx(msg)
        assert data[0] & 0x01 == 1
        assert data[0] & 0x02 == 0

        # mot2만 활성
        msg = MD2KTxMsg(mot1_enable=False, mot2_enable=True)
        data = encode_tx(msg)
        assert data[0] & 0x01 == 0
        assert data[0] & 0x02 == 2

    def test_zero_velocity(self):
        """속도 0 인코딩."""
        msg = MD2KTxMsg(mot1_enable=True, mot2_enable=True)
        decoded = decode_tx(encode_tx(msg))
        assert decoded.mot1_vel_req == 0
        assert decoded.mot2_vel_req == 0

    def test_negative_velocity(self):
        """음수 RPM 인코딩."""
        msg = MD2KTxMsg(mot1_vel_req=-2000, mot2_vel_req=-1)
        decoded = decode_tx(encode_tx(msg))
        assert decoded.mot1_vel_req == -2000
        assert decoded.mot2_vel_req == -1

    def test_short_data_raises(self):
        """데이터 부족 시 예외."""
        with pytest.raises(ValueError):
            decode_tx(b"\x00\x00\x00")


class TestRxCodec:
    def test_roundtrip(self):
        """encode→decode 왕복 일치."""
        msg = MD2KRxMsg(
            mot1_vel_act=1200, mot2_vel_act=-600,
            alarm=MotorAlarm.STALL | MotorAlarm.OVER_TEMP,
            temperature=45,
        )
        data = encode_rx(msg)
        assert len(data) == 8
        decoded = decode_rx(data)
        assert decoded.mot1_vel_act == msg.mot1_vel_act
        assert decoded.mot2_vel_act == msg.mot2_vel_act
        assert decoded.alarm == msg.alarm
        assert decoded.temperature == msg.temperature

    def test_no_alarm(self):
        """알람 없음."""
        msg = MD2KRxMsg(alarm=MotorAlarm.NONE)
        decoded = decode_rx(encode_rx(msg))
        assert decoded.alarm == MotorAlarm.NONE

    def test_temperature_clamp(self):
        """온도 0~255 범위 클램프."""
        msg = MD2KRxMsg(temperature=300)
        data = encode_rx(msg)
        decoded = decode_rx(data)
        assert decoded.temperature == 255

    def test_short_data_raises(self):
        with pytest.raises(ValueError):
            decode_rx(b"\x00")


class TestRpmConversion:
    def test_zero(self):
        assert rpm_to_track_speed(0) == 0.0
        assert track_speed_to_rpm(0) == 0.0

    def test_roundtrip(self):
        """RPM→m/s→RPM 왕복."""
        original_rpm = 2000.0
        speed = rpm_to_track_speed(original_rpm)
        recovered_rpm = track_speed_to_rpm(speed)
        assert abs(recovered_rpm - original_rpm) < 0.01

    def test_max_speed(self):
        """2000 RPM → 약 1.111 m/s (RS500 최대 속도)."""
        speed = rpm_to_track_speed(2000.0, gear_ratio=20.0, sprocket_radius=0.106)
        assert abs(speed - 1.111) < 0.01

    def test_custom_params(self):
        """커스텀 감속비/스프로킷 반경."""
        speed = rpm_to_track_speed(1000.0, gear_ratio=10.0, sprocket_radius=0.2)
        expected = (1000.0 * 2 * 3.141592653589793 * 0.2) / (60.0 * 10.0)
        assert abs(speed - expected) < 1e-6

    def test_negative_rpm(self):
        """음수 RPM → 음수 속도."""
        speed = rpm_to_track_speed(-1000.0)
        assert speed < 0
        rpm_back = track_speed_to_rpm(speed)
        assert abs(rpm_back - (-1000.0)) < 0.01
