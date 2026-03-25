#!/usr/bin/env python3
"""C50 신규 기능 시뮬레이션: MD2K SS/SD, 보호기능, 안전상태.

기존 시나리오 러너와 별개로, C50에서 추가된 기능을 검증한다:
1. SS/SD 가감속 프로파일 vs 기존 1차 지연 비교
2. 전자브레이크 감속 프로파일
3. 비상정지 / 자율주행 모드 / 시스템 폴트 안전 상태
4. 모터 보호 기능 (과전류 Soft fuse, 과온도)
5. MD2K CAN 코덱 왕복 검증
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'ad_core'))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from ad_core.drivetrain_model import DrivetrainConfig, SingleMotorModel, DrivetrainModel
from ad_core.vehicle_dynamics import VehicleDynamics, VehicleDynamicsConfig
from ad_core.skid_steer_model import SkidSteerModel
from ad_core.datatypes import Pose2D
from ad_core.motor_protection import MotorProtection, ProtectionConfig, MotorAlarm
from ad_core.md2k_codec import (
    MD2KTxMsg, MD2KRxMsg, encode_tx, decode_tx, encode_rx, decode_rx,
    rpm_to_track_speed, track_speed_to_rpm,
)


DT = 0.01  # 10ms
SAVE_DIR = os.path.join(os.path.dirname(__file__), '..', 'results', 'c50')


def ensure_dir():
    os.makedirs(SAVE_DIR, exist_ok=True)


# ====================================================================== #
#  1. SS/SD 가감속 프로파일 비교
# ====================================================================== #

def sim_sssd_comparison():
    """SS/SD 가감속 vs 기존 1차 지연 비교."""
    print('\n[1] SS/SD 가감속 프로파일 비교')

    configs = {
        '1차 지연 (tau=0.2s)': DrivetrainConfig(slow_start_time=0.0, slow_down_time=0.0, time_constant=0.20),
        'SS=1s / SD=0.5s': DrivetrainConfig(slow_start_time=1.0, slow_down_time=0.5),
        'SS=2s / SD=1s': DrivetrainConfig(slow_start_time=2.0, slow_down_time=1.0),
        'SS=5s / SD=3s': DrivetrainConfig(slow_start_time=5.0, slow_down_time=3.0),
    }

    duration_accel = 6.0  # 가속 구간 (s)
    duration_decel = 4.0  # 감속 구간 (s)
    total = duration_accel + duration_decel
    cmd_speed = 0.8  # m/s

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle('C50: SS/SD 가감속 프로파일 비교', fontsize=13)

    for label, cfg in configs.items():
        motor = SingleMotorModel(cfg)
        times, vels = [], []
        t = 0.0
        while t < total:
            cmd = cmd_speed if t < duration_accel else 0.0
            motor.update(cmd, DT)
            times.append(t)
            vels.append(motor.actual_velocity)
            t += DT

        times = np.array(times)
        vels = np.array(vels)

        ax1.plot(times, vels, label=label, linewidth=1.5)

        # 가속 시간 측정 (10%~90%)
        max_v = np.max(vels[:int(duration_accel / DT)])
        if max_v > 0.01:
            t10 = times[np.searchsorted(vels[:int(duration_accel / DT)], max_v * 0.1)]
            t90_idx = np.searchsorted(vels[:int(duration_accel / DT)], max_v * 0.9)
            t90 = times[min(t90_idx, len(times) - 1)]
            rise_time = t90 - t10
            print(f'  {label}: 상승시간(10→90%)={rise_time:.2f}s, 최대속도={max_v:.3f} m/s')

    ax1.set_xlabel('시간 (s)')
    ax1.set_ylabel('속도 (m/s)')
    ax1.set_title('가속 + 감속 프로파일')
    ax1.axvline(x=duration_accel, color='r', linestyle='--', alpha=0.3, label='감속 시작')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # SS 시간별 도달 속도 비교
    ss_times = np.arange(0.5, 10.5, 0.5)
    v_at_3s = []
    for ss in ss_times:
        motor = SingleMotorModel(DrivetrainConfig(slow_start_time=ss, slow_down_time=ss / 2))
        for _ in range(int(3.0 / DT)):
            motor.update(cmd_speed, DT)
        v_at_3s.append(motor.actual_velocity)

    ax2.bar(ss_times, v_at_3s, width=0.4, alpha=0.7, color='steelblue')
    ax2.set_xlabel('SS 시간 (s)')
    ax2.set_ylabel('3초 후 속도 (m/s)')
    ax2.set_title('SS 시간별 3초 후 도달 속도')
    ax2.axhline(y=cmd_speed * 0.85, color='r', linestyle='--', alpha=0.5, label=f'목표의 85%')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(SAVE_DIR, 'c50_sssd_comparison.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f'  저장: {path}')
    plt.close(fig)


# ====================================================================== #
#  2. 전자브레이크 감속 프로파일
# ====================================================================== #

def sim_brake():
    """전자브레이크 16Nm 감속 프로파일."""
    print('\n[2] 전자브레이크 감속 프로파일')

    motor = SingleMotorModel(DrivetrainConfig(brake_torque=16.0))

    # 가속
    for _ in range(300):
        motor.update(0.8, DT)
    speed_at_brake = motor.actual_velocity
    print(f'  브레이크 전 속도: {speed_at_brake:.3f} m/s')

    # 브레이크 체결
    motor.set_brake(True)
    times, vels = [0.0], [speed_at_brake]
    t = 0.0
    while motor.actual_velocity > 0.0 and t < 5.0:
        motor.update(0.8, DT)  # 명령 무시
        t += DT
        times.append(t)
        vels.append(motor.actual_velocity)

    stop_time = times[-1]
    print(f'  정지까지: {stop_time:.2f}s')

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(times, vels, 'r-', linewidth=2, label='brake_torque=16Nm')
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.set_xlabel('시간 (s)')
    ax.set_ylabel('속도 (m/s)')
    ax.set_title(f'C50: 전자브레이크 감속 (초기속도={speed_at_brake:.3f} m/s, 정지={stop_time:.2f}s)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(SAVE_DIR, 'c50_brake_profile.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f'  저장: {path}')
    plt.close(fig)


# ====================================================================== #
#  3. 안전 상태 시뮬레이션
# ====================================================================== #

def sim_safety_states():
    """비상정지 / 자율주행 모드 / 시스템 폴트 시나리오."""
    print('\n[3] 안전 상태 시뮬레이션')

    cfg = VehicleDynamicsConfig(mass=800.0, max_speed=1.111, max_accel=0.5, max_decel=1.5)
    dynamics = VehicleDynamics(cfg)

    total_time = 20.0
    events = {
        5.0: ('e-stop ON', lambda d: d.set_emergency_stop(True)),
        8.0: ('e-stop OFF', lambda d: d.set_emergency_stop(False)),
        12.0: ('autonomous OFF', lambda d: d.set_autonomous_mode(False)),
        15.0: ('autonomous ON', lambda d: d.set_autonomous_mode(True)),
        17.0: ('fault ON', lambda d: d.set_system_fault(True)),
        19.0: ('fault OFF', lambda d: d.set_system_fault(False)),
    }

    times, speeds, estop_flags, auto_flags, fault_flags = [], [], [], [], []
    pose = Pose2D()
    t = 0.0

    while t < total_time:
        # 이벤트 체크
        for event_t, (label, action) in events.items():
            if abs(t - event_t) < DT / 2:
                action(dynamics)
                print(f'  t={t:.1f}s: {label}')

        pose = dynamics.step(0.8, 0.8, DT, pose)

        times.append(t)
        speeds.append(dynamics.current_linear_velocity)
        estop_flags.append(1.0 if dynamics.emergency_stop else 0.0)
        auto_flags.append(1.0 if dynamics.autonomous_mode else 0.0)
        fault_flags.append(1.0 if dynamics.system_fault else 0.0)

        t += DT

    times = np.array(times)
    speeds = np.array(speeds)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True,
                                     gridspec_kw={'height_ratios': [3, 1]})
    fig.suptitle('C50: Safety States Simulation', fontsize=13)

    ax1.plot(times, speeds, 'b-', linewidth=1.5, label='speed (m/s)')
    ax1.set_ylabel('speed (m/s)')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # 이벤트 영역 색칠
    for i, t_val in enumerate(times[:-1]):
        if estop_flags[i] > 0:
            ax1.axvspan(t_val, times[i + 1], alpha=0.2, color='red')
        elif fault_flags[i] > 0:
            ax1.axvspan(t_val, times[i + 1], alpha=0.2, color='orange')
        elif auto_flags[i] < 1:
            ax1.axvspan(t_val, times[i + 1], alpha=0.2, color='yellow')

    ax2.step(times, estop_flags, 'r-', label='e-stop', where='post')
    ax2.step(times, [1 - a for a in auto_flags], 'y-', label='manual mode', where='post')
    ax2.step(times, fault_flags, color='orange', label='fault', where='post')
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('flags')
    ax2.set_ylim(-0.1, 1.3)
    ax2.legend(loc='upper right', fontsize=8)
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(SAVE_DIR, 'c50_safety_states.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f'  저장: {path}')
    plt.close(fig)


# ====================================================================== #
#  4. 모터 보호 기능 시뮬레이션
# ====================================================================== #

def sim_motor_protection():
    """과전류 Soft fuse + 과온도 시나리오."""
    print('\n[4] 모터 보호 기능 시뮬레이션')

    prot = MotorProtection(ProtectionConfig(
        rated_current=78.0, max_current=100.0,
        soft_fuse_time=2.0, over_temp_threshold=65.0,
        ambient_temp=25.0,
        winding_resistance=0.05,
        thermal_resistance=0.002,
        cooling_time_constant=60.0,
    ))

    # 시나리오: 정격 근접 → 과부하 → 과온도 폴트 → 냉각 → 복구
    # 열 모델: 정격(78A)→61.5°C 정상상태, 95A→79.2°C (65°C 초과 ~80s)
    total_time = 300.0
    phases = [
        (0.0, 60.0, 75.0, '정격 근접 (75A)'),     # 서서히 가열
        (60.0, 150.0, 95.0, '과부하 (95A)'),       # 과전류+과온도
        (150.0, 250.0, 0.0, '냉각 (전류 0)'),      # 냉각
        (250.0, 300.0, 50.0, '복구 후 정상'),       # 복구
    ]

    times, temps, currents, faults, alarms = [], [], [], [], []
    t = 0.0

    while t < total_time:
        # 현재 phase의 전류 결정
        current = 0.0
        for start, end, curr, label in phases:
            if start <= t < end:
                current = curr
                break

        # 냉각 후 폴트 해제 시도
        if abs(t - 20.0) < DT / 2:
            prot.clear_fault()
            print(f'  t={t:.1f}s: clear_fault() 시도')

        alarm = prot.update(
            command_speed=1.0 if current > 0 else 0.0,
            actual_speed=0.9 if current > 0 else 0.0,
            current=current, voltage=48.0, dt=DT,
        )

        times.append(t)
        temps.append(prot.temperature)
        currents.append(current)
        faults.append(1.0 if prot.is_fault else 0.0)
        alarms.append(alarm.value)

        t += DT

    times = np.array(times)

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle('C50: Motor Protection Simulation (Soft Fuse + Thermal)', fontsize=13)

    axes[0].plot(times, currents, 'b-', linewidth=1)
    axes[0].axhline(y=78, color='orange', linestyle='--', alpha=0.5, label='rated (78A)')
    axes[0].set_ylabel('Current (A)')
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times, temps, 'r-', linewidth=1.5)
    axes[1].axhline(y=65, color='red', linestyle='--', alpha=0.5, label='threshold (65C)')
    axes[1].set_ylabel('Temperature (C)')
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    axes[2].fill_between(times, faults, alpha=0.4, color='red', label='fault')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Fault')
    axes[2].set_ylim(-0.1, 1.3)
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)

    # 위상 구분 표시
    for start, end, curr, label in phases:
        axes[0].axvspan(start, end, alpha=0.05, color='gray')
        axes[0].text((start + end) / 2, max(currents) * 0.9, label,
                     ha='center', fontsize=7, alpha=0.7)

    fig.tight_layout()
    path = os.path.join(SAVE_DIR, 'c50_motor_protection.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f'  저장: {path}')

    # 폴트 발생 시점
    fault_start = None
    for i, f in enumerate(faults):
        if f > 0 and fault_start is None:
            fault_start = times[i]
            break
    if fault_start:
        print(f'  과전류 폴트 발생: t={fault_start:.2f}s')
    print(f'  최고 온도: {max(temps):.1f}°C')

    plt.close(fig)


# ====================================================================== #
#  5. CAN 코덱 검증
# ====================================================================== #

def sim_can_codec():
    """MD2K CAN 코덱 encode/decode 왕복 + RPM 변환."""
    print('\n[5] MD2K CAN 코덱 검증')

    # Tx 왕복
    test_cases_tx = [
        MD2KTxMsg(True, True, 2000, -1500),
        MD2KTxMsg(True, False, 0, 0),
        MD2KTxMsg(False, True, -32768, 32767),
    ]
    for msg in test_cases_tx:
        data = encode_tx(msg)
        decoded = decode_tx(data)
        match = (decoded.mot1_enable == msg.mot1_enable and
                 decoded.mot2_enable == msg.mot2_enable and
                 decoded.mot1_vel_req == msg.mot1_vel_req and
                 decoded.mot2_vel_req == msg.mot2_vel_req)
        status = 'OK' if match else 'FAIL'
        print(f'  TX: vel=({msg.mot1_vel_req},{msg.mot2_vel_req}) → {data.hex()} → '
              f'({decoded.mot1_vel_req},{decoded.mot2_vel_req}) [{status}]')

    # Rx 왕복
    test_cases_rx = [
        MD2KRxMsg(1200, -600, MotorAlarm.STALL | MotorAlarm.OVER_TEMP, 45),
        MD2KRxMsg(0, 0, MotorAlarm.NONE, 25),
    ]
    for msg in test_cases_rx:
        data = encode_rx(msg)
        decoded = decode_rx(data)
        match = (decoded.mot1_vel_act == msg.mot1_vel_act and
                 decoded.mot2_vel_act == msg.mot2_vel_act and
                 decoded.alarm == msg.alarm and
                 decoded.temperature == msg.temperature)
        status = 'OK' if match else 'FAIL'
        print(f'  RX: act=({msg.mot1_vel_act},{msg.mot2_vel_act}) alarm={msg.alarm} temp={msg.temperature}C '
              f'→ {data.hex()} [{status}]')

    # RPM 변환
    print(f'\n  RPM 변환:')
    for rpm in [0, 500, 1000, 1500, 2000]:
        speed = rpm_to_track_speed(rpm)
        rpm_back = track_speed_to_rpm(speed)
        print(f'    {rpm:5d} RPM → {speed:.4f} m/s → {rpm_back:.1f} RPM')


# ====================================================================== #
#  6. 통합 시뮬: SS/SD + 안전상태 + 보호
# ====================================================================== #

def sim_integrated():
    """SS/SD 가감속 + 안전상태 + 보호기능 통합 시나리오."""
    print('\n[6] 통합 시뮬레이션 (SS/SD + 안전상태 + 보호)')

    # SS/SD가 설정된 구동계
    dt_cfg = DrivetrainConfig(slow_start_time=2.0, slow_down_time=1.0, brake_torque=16.0)
    drivetrain = DrivetrainModel(dt_cfg)

    # 동역학
    vd_cfg = VehicleDynamicsConfig(mass=800.0, max_speed=1.111)
    dynamics = VehicleDynamics(vd_cfg)

    # 보호
    prot_l = MotorProtection(ProtectionConfig())
    prot_r = MotorProtection(ProtectionConfig())

    total_time = 25.0
    events = {
        10.0: ('e-stop ON', lambda: dynamics.set_emergency_stop(True)),
        13.0: ('e-stop OFF', lambda: dynamics.set_emergency_stop(False)),
        20.0: ('fault ON', lambda: dynamics.set_system_fault(True)),
        23.0: ('fault OFF', lambda: dynamics.set_system_fault(False)),
    }

    times, speeds_l, speeds_r, speeds_vd = [], [], [], []
    pose = Pose2D()
    t = 0.0

    while t < total_time:
        for event_t, (label, action) in events.items():
            if abs(t - event_t) < DT / 2:
                action()
                print(f'  t={t:.1f}s: {label}')

        # 명령: 직진 0.8 m/s
        cmd = 0.8
        actual_l, actual_r = drivetrain.update(cmd, cmd, DT)

        # 보호 체크
        current_est = 50.0  # 간이 전류 추정
        prot_l.update(cmd, actual_l, current_est, 48.0, DT)
        prot_r.update(cmd, actual_r, current_est, 48.0, DT)

        # 동역학
        pose = dynamics.step(actual_l, actual_r, DT, pose)

        times.append(t)
        speeds_l.append(actual_l)
        speeds_r.append(actual_r)
        speeds_vd.append(dynamics.current_linear_velocity)
        t += DT

    times = np.array(times)

    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    fig.suptitle('C50: Integrated Simulation (SS/SD + Safety + Protection)', fontsize=13)

    axes[0].plot(times, speeds_l, 'b-', linewidth=1, alpha=0.7, label='drivetrain L')
    axes[0].plot(times, speeds_r, 'r-', linewidth=1, alpha=0.7, label='drivetrain R')
    axes[0].set_ylabel('Drivetrain Speed (m/s)')
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times, speeds_vd, 'g-', linewidth=1.5, label='vehicle linear vel')
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Vehicle Speed (m/s)')
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    # 이벤트 표시
    for event_t, (label, _) in events.items():
        for ax in axes:
            ax.axvline(x=event_t, color='gray', linestyle='--', alpha=0.4)
        axes[0].text(event_t + 0.1, max(speeds_l) * 0.95, label, fontsize=7, alpha=0.7)

    fig.tight_layout()
    path = os.path.join(SAVE_DIR, 'c50_integrated.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f'  저장: {path}')
    plt.close(fig)

    print(f'  최종 속도: {dynamics.current_linear_velocity:.3f} m/s')


# ====================================================================== #
#  메인
# ====================================================================== #

def main():
    ensure_dir()
    print('=' * 60)
    print('  C50 시뮬레이션: MD2K + 안전상태 + 보호기능')
    print('=' * 60)

    sim_sssd_comparison()
    sim_brake()
    sim_safety_states()
    sim_motor_protection()
    sim_can_codec()
    sim_integrated()

    print('\n' + '=' * 60)
    print('  C50 시뮬레이션 완료!')
    print('=' * 60)


if __name__ == '__main__':
    main()
