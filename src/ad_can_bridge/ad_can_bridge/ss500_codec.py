"""SS500 CAN ICD 인코더/디코더.

SS500 CAN 프로토콜(Excel 260213 기준)의 메시지를 인코딩/디코딩한다.
Little Endian, 500kbps, CAN2 버스.

핵심 메시지:
  - ADT2VCU1 (0x301): ADT -> VCU 제어 명령 (50ms)
  - VCU2ADT1 (0x331): VCU -> ADT 차량 상태 (50ms)
  - VCU2ADT2 (0x332): VCU -> ADT 초음파센서 (50ms)
"""

import struct
from dataclasses import dataclass
from typing import Optional


# ======================================================================
# CAN ID 상수
# ======================================================================

CAN_ID_ADT2VCU1 = 0x301   # ADT -> VCU 제어 명령
CAN_ID_VCU2ADT1 = 0x331   # VCU -> ADT 차량 상태
CAN_ID_VCU2ADT2 = 0x332   # VCU -> ADT 초음파센서


# ======================================================================
# 데이터 구조
# ======================================================================

@dataclass
class ADT2VCU1:
    """ADT -> VCU 제어 명령 (0x301, 8 bytes, 50ms).

    Attributes:
        adt_valid: 자율주행 시스템 유효 (0=Invalid, 1=Valid).
        auto_ctrl_enable: 자율주행 동작 중 (0=Not, 1=Operating).
        return_state: 복귀 상태 (0~3).
        go_back_state: 회귀 상태 (0~3).
        drive_condition: 주행 조건 (0~15, eADTDrvCnd).
        system_fault: 시스템 고장코드 (0~15).
        left_speed_cmd_kmh: 좌측 속도 명령 (km/h, -4~4).
        right_speed_cmd_kmh: 우측 속도 명령 (km/h, -4~4).
        warn_msg: 경고 메시지 (0~15).
        alive_counter: Alive Counter (0~15, 순환).
    """
    adt_valid: int = 0
    auto_ctrl_enable: int = 0
    return_state: int = 0
    go_back_state: int = 0
    drive_condition: int = 0
    system_fault: int = 0
    left_speed_cmd_kmh: float = 0.0
    right_speed_cmd_kmh: float = 0.0
    warn_msg: int = 0
    alive_counter: int = 0


@dataclass
class VCU2ADT1:
    """VCU -> ADT 차량 상태 (0x331, 8 bytes, 50ms).

    Attributes:
        auto_ctrl_enable_switch: 자율주행 활성화 스위치.
        manual_emergency_stop: 수동 비상정지 (0=OFF, 1=ON).
        low_bat_return_req: 배터리부족 복귀요청.
        tank_level_return_req: 탱크레벨 복귀요청.
        system_fault_state: 시스템고장 (0=정상, 1=운전불가, 2=작업불가).
        bumper_detected: 범퍼센서 감지.
        vehicle_status: 차량 상태 바이트.
        left_speed_actual_kmh: 좌측 실제 속도 (km/h).
        right_speed_actual_kmh: 우측 실제 속도 (km/h).
        max_speed_config_kmh: 최대속도 설정값 (km/h).
        vcu_alive_counter: VCU Alive Counter.
    """
    auto_ctrl_enable_switch: int = 0
    manual_emergency_stop: int = 0
    low_bat_return_req: int = 0
    tank_level_return_req: int = 0
    system_fault_state: int = 0
    bumper_detected: int = 0
    vehicle_status: int = 0
    left_speed_actual_kmh: float = 0.0
    right_speed_actual_kmh: float = 0.0
    max_speed_config_kmh: float = 0.0
    vcu_alive_counter: int = 0


@dataclass
class VCU2ADT2:
    """VCU -> ADT 초음파센서 (0x332, 8 bytes, 50ms).

    Attributes:
        ultrasonic_mid_mm: 중앙 초음파센서 거리 (mm).
    """
    ultrasonic_mid_mm: int = 0


# ======================================================================
# 인코더
# ======================================================================

class SS500Encoder:
    """SS500 CAN 메시지 인코더."""

    @staticmethod
    def encode_adt2vcu1(msg: ADT2VCU1) -> bytes:
        """ADT2VCU1 (0x301) 메시지를 8바이트 CAN 데이터로 인코딩한다.

        바이트 레이아웃 (Little Endian):
          Byte 0: [bit0] valid, [bit1] auto_enable, [bit2-3] return_st,
                  [bit4-5] go_back_st, [bit6-7] reserved
          Byte 1: [bit0-3] drive_condition, [bit4-7] system_fault
          Byte 2-3: left_speed_cmd (S16, factor=0.01)
          Byte 4-5: right_speed_cmd (S16, factor=0.01)
          Byte 6: reserved
          Byte 7: [bit0-3] warn_msg, [bit4-7] alive_counter

        Returns:
            8바이트 CAN 데이터.
        """
        data = bytearray(8)

        # Byte 0: flags
        byte0 = (
            (msg.adt_valid & 0x01)
            | ((msg.auto_ctrl_enable & 0x01) << 1)
            | ((msg.return_state & 0x03) << 2)
            | ((msg.go_back_state & 0x03) << 4)
        )
        data[0] = byte0

        # Byte 1: drive_condition + system_fault
        data[1] = (msg.drive_condition & 0x0F) | ((msg.system_fault & 0x0F) << 4)

        # Byte 2-3: left speed command (S16, factor=0.01, unit=km/h)
        left_raw = int(round(msg.left_speed_cmd_kmh / 0.01))
        left_raw = max(-32768, min(32767, left_raw))
        struct.pack_into('<h', data, 2, left_raw)

        # Byte 4-5: right speed command (S16, factor=0.01, unit=km/h)
        right_raw = int(round(msg.right_speed_cmd_kmh / 0.01))
        right_raw = max(-32768, min(32767, right_raw))
        struct.pack_into('<h', data, 4, right_raw)

        # Byte 6: reserved
        data[6] = 0x00

        # Byte 7: warn_msg + alive_counter
        data[7] = (msg.warn_msg & 0x0F) | ((msg.alive_counter & 0x0F) << 4)

        return bytes(data)


# ======================================================================
# 디코더
# ======================================================================

class SS500Decoder:
    """SS500 CAN 메시지 디코더."""

    @staticmethod
    def decode_vcu2adt1(data: bytes) -> VCU2ADT1:
        """VCU2ADT1 (0x331) 8바이트 CAN 데이터를 디코딩한다.

        바이트 레이아웃 (Little Endian):
          Byte 0: [bit0] auto_ctrl_enable_switch, [bit1] manual_emergency_stop,
                  [bit2] low_bat_return_req, [bit3] tank_level_return_req,
                  [bit4-5] system_fault_state, [bit6] bumper_detected,
                  [bit7] reserved
          Byte 1: vehicle_status
          Byte 2-3: left_speed_actual (S16, factor=0.01)
          Byte 4-5: right_speed_actual (S16, factor=0.01)
          Byte 6: max_speed_config (U8, factor=0.1)
          Byte 7: [bit0-3] reserved, [bit4-7] vcu_alive_counter

        Args:
            data: 8바이트 CAN 수신 데이터.

        Returns:
            디코딩된 VCU2ADT1 구조체.
        """
        if len(data) < 8:
            raise ValueError(f"VCU2ADT1 requires 8 bytes, got {len(data)}")

        msg = VCU2ADT1()

        # Byte 0: flags
        byte0 = data[0]
        msg.auto_ctrl_enable_switch = byte0 & 0x01
        msg.manual_emergency_stop = (byte0 >> 1) & 0x01
        msg.low_bat_return_req = (byte0 >> 2) & 0x01
        msg.tank_level_return_req = (byte0 >> 3) & 0x01
        msg.system_fault_state = (byte0 >> 4) & 0x03
        msg.bumper_detected = (byte0 >> 6) & 0x01

        # Byte 1: vehicle_status
        msg.vehicle_status = data[1]

        # Byte 2-3: left speed actual (S16, factor=0.01)
        left_raw = struct.unpack_from('<h', data, 2)[0]
        msg.left_speed_actual_kmh = left_raw * 0.01

        # Byte 4-5: right speed actual (S16, factor=0.01)
        right_raw = struct.unpack_from('<h', data, 4)[0]
        msg.right_speed_actual_kmh = right_raw * 0.01

        # Byte 6: max speed config (U8, factor=0.1)
        msg.max_speed_config_kmh = data[6] * 0.1

        # Byte 7: alive counter (upper nibble)
        msg.vcu_alive_counter = (data[7] >> 4) & 0x0F

        return msg

    @staticmethod
    def decode_vcu2adt2(data: bytes) -> VCU2ADT2:
        """VCU2ADT2 (0x332) 8바이트 CAN 데이터를 디코딩한다.

        Args:
            data: 8바이트 CAN 수신 데이터.

        Returns:
            디코딩된 VCU2ADT2 구조체.
        """
        if len(data) < 2:
            raise ValueError(f"VCU2ADT2 requires at least 2 bytes, got {len(data)}")

        msg = VCU2ADT2()

        # Byte 0-1: ultrasonic_mid (U16)
        msg.ultrasonic_mid_mm = struct.unpack_from('<H', data, 0)[0]

        return msg
