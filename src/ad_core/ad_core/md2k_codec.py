"""MDROBOT MD2K 듀얼채널 모터 드라이버 CAN 코덱.

MD2K CAN 프로토콜 (50 kbps):
- Tx (호스트→MD2K): CAN ID 0x001
  - Mot1/2 Enable + Vel_Req (RPM)
- Rx (MD2K→호스트): CAN ID 0x701
  - Mot1/2 Vel_Act (RPM) + 알람 비트 + 온도

RPM ↔ m/s 변환 유틸리티 포함.
"""

import struct
from dataclasses import dataclass
from typing import Tuple

from ad_core.motor_protection import MotorAlarm

# CAN ID 상수
MD2K_TX_ID = 0x001  # 호스트 → MD2K
MD2K_RX_ID = 0x701  # MD2K → 호스트


@dataclass
class MD2KTxMsg:
    """MD2K 송신 메시지 (호스트→드라이버).

    Attributes:
        mot1_enable: 채널1 (좌측) 활성화.
        mot2_enable: 채널2 (우측) 활성화.
        mot1_vel_req: 채널1 RPM 명령 (부호 있음).
        mot2_vel_req: 채널2 RPM 명령 (부호 있음).
    """
    mot1_enable: bool = False
    mot2_enable: bool = False
    mot1_vel_req: int = 0  # RPM, signed 16-bit
    mot2_vel_req: int = 0  # RPM, signed 16-bit


@dataclass
class MD2KRxMsg:
    """MD2K 수신 메시지 (드라이버→호스트).

    Attributes:
        mot1_vel_act: 채널1 실제 RPM (부호 있음).
        mot2_vel_act: 채널2 실제 RPM (부호 있음).
        alarm: 알람 비트 플래그.
        temperature: 드라이버 온도 (°C).
    """
    mot1_vel_act: int = 0   # RPM, signed 16-bit
    mot2_vel_act: int = 0   # RPM, signed 16-bit
    alarm: MotorAlarm = MotorAlarm.NONE
    temperature: int = 0    # °C, unsigned 8-bit


def encode_tx(msg: MD2KTxMsg) -> bytes:
    """MD2K Tx 메시지를 8바이트 CAN 데이터로 인코딩한다.

    레이아웃 (8 bytes):
        [0]    : 제어 바이트 (bit0=mot1_en, bit1=mot2_en)
        [1]    : 예약
        [2..3] : mot1_vel_req (int16, big-endian)
        [4..5] : mot2_vel_req (int16, big-endian)
        [6..7] : 예약 (0x00)
    """
    ctrl = (int(msg.mot1_enable) & 1) | ((int(msg.mot2_enable) & 1) << 1)
    vel1 = _clamp_i16(msg.mot1_vel_req)
    vel2 = _clamp_i16(msg.mot2_vel_req)
    return struct.pack(">BxhhH", ctrl, vel1, vel2, 0)


def decode_tx(data: bytes) -> MD2KTxMsg:
    """8바이트 CAN 데이터를 MD2K Tx 메시지로 디코딩한다."""
    if len(data) < 8:
        raise ValueError(f"TX 데이터는 8바이트 필요, {len(data)}바이트 수신")
    ctrl, vel1, vel2, _ = struct.unpack(">BxhhH", data[:8])
    return MD2KTxMsg(
        mot1_enable=bool(ctrl & 0x01),
        mot2_enable=bool(ctrl & 0x02),
        mot1_vel_req=vel1,
        mot2_vel_req=vel2,
    )


def encode_rx(msg: MD2KRxMsg) -> bytes:
    """MD2K Rx 메시지를 8바이트 CAN 데이터로 인코딩한다.

    레이아웃 (8 bytes):
        [0..1] : mot1_vel_act (int16, big-endian)
        [2..3] : mot2_vel_act (int16, big-endian)
        [4]    : alarm (uint8, MotorAlarm 비트맵)
        [5]    : temperature (uint8, °C)
        [6..7] : 예약 (0x00)
    """
    vel1 = _clamp_i16(msg.mot1_vel_act)
    vel2 = _clamp_i16(msg.mot2_vel_act)
    alarm_byte = msg.alarm.value & 0xFF
    temp_byte = max(0, min(255, msg.temperature))
    return struct.pack(">hhBBH", vel1, vel2, alarm_byte, temp_byte, 0)


def decode_rx(data: bytes) -> MD2KRxMsg:
    """8바이트 CAN 데이터를 MD2K Rx 메시지로 디코딩한다."""
    if len(data) < 8:
        raise ValueError(f"RX 데이터는 8바이트 필요, {len(data)}바이트 수신")
    vel1, vel2, alarm_byte, temp_byte, _ = struct.unpack(">hhBBH", data[:8])
    return MD2KRxMsg(
        mot1_vel_act=vel1,
        mot2_vel_act=vel2,
        alarm=MotorAlarm(alarm_byte & 0x7F),  # 7비트 사용
        temperature=temp_byte,
    )


# ------------------------------------------------------------------ #
#  RPM ↔ m/s 변환 유틸리티
# ------------------------------------------------------------------ #

def rpm_to_track_speed(
    rpm: float,
    gear_ratio: float = 20.0,
    sprocket_radius: float = 0.106,
) -> float:
    """모터 RPM을 트랙 속도 (m/s)로 변환한다.

    v = (RPM * 2π * r) / (60 * gear_ratio)
    """
    return (rpm * 2.0 * 3.141592653589793 * sprocket_radius) / (60.0 * gear_ratio)


def track_speed_to_rpm(
    speed_mps: float,
    gear_ratio: float = 20.0,
    sprocket_radius: float = 0.106,
) -> float:
    """트랙 속도 (m/s)를 모터 RPM으로 변환한다.

    RPM = (v * 60 * gear_ratio) / (2π * r)
    """
    return (speed_mps * 60.0 * gear_ratio) / (2.0 * 3.141592653589793 * sprocket_radius)


# ------------------------------------------------------------------ #
#  내부 유틸
# ------------------------------------------------------------------ #

def _clamp_i16(value: int) -> int:
    """int16 범위로 클램프."""
    return max(-32768, min(32767, int(value)))
