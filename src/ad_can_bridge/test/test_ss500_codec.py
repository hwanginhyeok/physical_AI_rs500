"""SS500 CAN 코덱 단위 테스트.

ICD 명세 vs 인코딩/디코딩 일치를 검증한다.
"""

import struct
import pytest

from ad_can_bridge.ss500_codec import (
    CAN_ID_ADT2VCU1,
    CAN_ID_VCU2ADT1,
    CAN_ID_VCU2ADT2,
    ADT2VCU1,
    VCU2ADT1,
    VCU2ADT2,
    SS500Encoder,
    SS500Decoder,
)


class TestConstants:
    """CAN ID 상수 검증."""

    def test_can_ids(self):
        assert CAN_ID_ADT2VCU1 == 0x301
        assert CAN_ID_VCU2ADT1 == 0x331
        assert CAN_ID_VCU2ADT2 == 0x332


class TestADT2VCU1Encoder:
    """ADT2VCU1 (0x301) 인코딩 검증."""

    def test_encode_zero(self):
        """모든 필드가 0인 메시지."""
        msg = ADT2VCU1()
        data = SS500Encoder.encode_adt2vcu1(msg)
        assert len(data) == 8
        assert data == b'\x00' * 8

    def test_encode_valid_flag(self):
        """adt_valid=1 설정."""
        msg = ADT2VCU1(adt_valid=1)
        data = SS500Encoder.encode_adt2vcu1(msg)
        assert data[0] & 0x01 == 1

    def test_encode_auto_enable(self):
        """auto_ctrl_enable=1 설정."""
        msg = ADT2VCU1(auto_ctrl_enable=1)
        data = SS500Encoder.encode_adt2vcu1(msg)
        assert (data[0] >> 1) & 0x01 == 1

    def test_encode_return_state(self):
        """return_state=2 설정."""
        msg = ADT2VCU1(return_state=2)
        data = SS500Encoder.encode_adt2vcu1(msg)
        assert (data[0] >> 2) & 0x03 == 2

    def test_encode_go_back_state(self):
        """go_back_state=3 설정."""
        msg = ADT2VCU1(go_back_state=3)
        data = SS500Encoder.encode_adt2vcu1(msg)
        assert (data[0] >> 4) & 0x03 == 3

    def test_encode_drive_condition(self):
        """drive_condition=5, system_fault=10."""
        msg = ADT2VCU1(drive_condition=5, system_fault=10)
        data = SS500Encoder.encode_adt2vcu1(msg)
        assert data[1] & 0x0F == 5
        assert (data[1] >> 4) & 0x0F == 10

    def test_encode_left_speed_positive(self):
        """좌측 속도 2.0 km/h (factor=0.01)."""
        msg = ADT2VCU1(left_speed_cmd_kmh=2.0)
        data = SS500Encoder.encode_adt2vcu1(msg)
        raw = struct.unpack_from('<h', data, 2)[0]
        assert raw == 200  # 2.0 / 0.01 = 200

    def test_encode_right_speed_negative(self):
        """우측 속도 -1.5 km/h (factor=0.01)."""
        msg = ADT2VCU1(right_speed_cmd_kmh=-1.5)
        data = SS500Encoder.encode_adt2vcu1(msg)
        raw = struct.unpack_from('<h', data, 4)[0]
        assert raw == -150  # -1.5 / 0.01 = -150

    def test_encode_warn_msg_and_alive(self):
        """warn_msg=7, alive_counter=12."""
        msg = ADT2VCU1(warn_msg=7, alive_counter=12)
        data = SS500Encoder.encode_adt2vcu1(msg)
        assert data[7] & 0x0F == 7
        assert (data[7] >> 4) & 0x0F == 12

    def test_encode_full_message(self):
        """전체 필드 설정 인코딩."""
        msg = ADT2VCU1(
            adt_valid=1,
            auto_ctrl_enable=1,
            return_state=1,
            go_back_state=2,
            drive_condition=3,
            system_fault=4,
            left_speed_cmd_kmh=1.0,
            right_speed_cmd_kmh=-1.0,
            warn_msg=5,
            alive_counter=15,
        )
        data = SS500Encoder.encode_adt2vcu1(msg)
        assert len(data) == 8

        # Byte 0: 1 | (1<<1) | (1<<2) | (2<<4) = 1+2+4+32 = 0x27
        assert data[0] == 0x27
        # Byte 1: 3 | (4<<4) = 3+64 = 0x43
        assert data[1] == 0x43
        # Byte 2-3: 100 (1.0/0.01)
        assert struct.unpack_from('<h', data, 2)[0] == 100
        # Byte 4-5: -100
        assert struct.unpack_from('<h', data, 4)[0] == -100
        # Byte 7: 5 | (15<<4) = 5+240 = 0xF5
        assert data[7] == 0xF5

    def test_encode_speed_clamping(self):
        """속도 값이 S16 범위를 넘지 않도록 클램핑."""
        msg = ADT2VCU1(left_speed_cmd_kmh=400.0)  # 400/0.01 = 40000 > 32767
        data = SS500Encoder.encode_adt2vcu1(msg)
        raw = struct.unpack_from('<h', data, 2)[0]
        assert raw == 32767


class TestVCU2ADT1Decoder:
    """VCU2ADT1 (0x331) 디코딩 검증."""

    def test_decode_zero(self):
        """모든 바이트 0."""
        data = b'\x00' * 8
        msg = SS500Decoder.decode_vcu2adt1(data)
        assert msg.auto_ctrl_enable_switch == 0
        assert msg.manual_emergency_stop == 0
        assert msg.left_speed_actual_kmh == 0.0
        assert msg.right_speed_actual_kmh == 0.0

    def test_decode_flags(self):
        """플래그 디코딩."""
        # Byte 0: bit0=1, bit1=1, bit4-5=2 (system_fault), bit6=1 (bumper)
        byte0 = 0x01 | 0x02 | (2 << 4) | (1 << 6)  # 0x63
        data = bytearray(8)
        data[0] = byte0
        msg = SS500Decoder.decode_vcu2adt1(bytes(data))
        assert msg.auto_ctrl_enable_switch == 1
        assert msg.manual_emergency_stop == 1
        assert msg.system_fault_state == 2
        assert msg.bumper_detected == 1

    def test_decode_left_speed(self):
        """좌측 실제 속도 1.5 km/h."""
        data = bytearray(8)
        struct.pack_into('<h', data, 2, 150)  # 150 * 0.01 = 1.5
        msg = SS500Decoder.decode_vcu2adt1(bytes(data))
        assert abs(msg.left_speed_actual_kmh - 1.5) < 0.001

    def test_decode_right_speed_negative(self):
        """우측 실제 속도 -2.0 km/h."""
        data = bytearray(8)
        struct.pack_into('<h', data, 4, -200)  # -200 * 0.01 = -2.0
        msg = SS500Decoder.decode_vcu2adt1(bytes(data))
        assert abs(msg.right_speed_actual_kmh - (-2.0)) < 0.001

    def test_decode_max_speed_config(self):
        """최대속도 설정 2.5 km/h (factor=0.1)."""
        data = bytearray(8)
        data[6] = 25  # 25 * 0.1 = 2.5
        msg = SS500Decoder.decode_vcu2adt1(bytes(data))
        assert abs(msg.max_speed_config_kmh - 2.5) < 0.001

    def test_decode_alive_counter(self):
        """alive counter 상위 니블."""
        data = bytearray(8)
        data[7] = 0xA0  # upper nibble = 10
        msg = SS500Decoder.decode_vcu2adt1(bytes(data))
        assert msg.vcu_alive_counter == 10

    def test_decode_short_data_raises(self):
        """8바이트 미만 데이터는 예외."""
        with pytest.raises(ValueError):
            SS500Decoder.decode_vcu2adt1(b'\x00' * 5)


class TestVCU2ADT2Decoder:
    """VCU2ADT2 (0x332) 디코딩 검증."""

    def test_decode_zero(self):
        data = b'\x00' * 8
        msg = SS500Decoder.decode_vcu2adt2(data)
        assert msg.ultrasonic_mid_mm == 0

    def test_decode_ultrasonic(self):
        """초음파센서 거리 1500mm."""
        data = bytearray(8)
        struct.pack_into('<H', data, 0, 1500)
        msg = SS500Decoder.decode_vcu2adt2(bytes(data))
        assert msg.ultrasonic_mid_mm == 1500

    def test_decode_max_ultrasonic(self):
        """초음파센서 최대 거리 65535mm."""
        data = bytearray(8)
        struct.pack_into('<H', data, 0, 65535)
        msg = SS500Decoder.decode_vcu2adt2(bytes(data))
        assert msg.ultrasonic_mid_mm == 65535

    def test_decode_short_data_raises(self):
        """2바이트 미만 데이터는 예외."""
        with pytest.raises(ValueError):
            SS500Decoder.decode_vcu2adt2(b'\x00')


class TestRoundTrip:
    """인코딩-디코딩 라운드트립 검증.

    ADT2VCU1을 인코딩한 후 바이트를 직접 검사하여
    ICD 명세와 일치하는지 확인한다.
    """

    def test_speed_roundtrip_precision(self):
        """속도 값의 인코딩/디코딩 정밀도 검증."""
        test_speeds = [0.0, 0.01, 0.5, 1.0, 2.0, 3.0, -1.0, -3.5]

        for speed in test_speeds:
            msg = ADT2VCU1(left_speed_cmd_kmh=speed, right_speed_cmd_kmh=speed)
            data = SS500Encoder.encode_adt2vcu1(msg)

            # 수동 디코딩
            left_raw = struct.unpack_from('<h', data, 2)[0]
            right_raw = struct.unpack_from('<h', data, 4)[0]
            decoded_left = left_raw * 0.01
            decoded_right = right_raw * 0.01

            assert abs(decoded_left - speed) < 0.01, f"Failed for speed={speed}"
            assert abs(decoded_right - speed) < 0.01, f"Failed for speed={speed}"
