# SS500 CAN 인터페이스 제어 문서 (ICD)

> **문서번호**: SS500-ICD-CAN-001
> **버전**: V0.2
> **작성일**: 2026-02-23
> **작성**: Claude (Engineer_1 역할)
> **검토**: -
> **승인**: -

---

## 1. 문서 개요

### 1.1 목적

본 문서는 PLUVA FRIEND Speed Sprayer SS500(GT-SS500) 차량의 CAN 버스 통신 인터페이스를 정의한다.
VCU(Vehicle Control Unit)를 중심으로 모든 CAN 노드 간 메시지 ID, 시그널 레이아웃, 물리값 변환,
타이밍 요구사항을 일원화하여 개발/검증/양산 단계의 기준 문서로 활용한다.

### 1.2 적용 범위

- CAN1 / CAN2 버스상의 모든 메시지 (Standard + Extended CAN FD)
- VCU ↔ ADT, LCD, BPA(BMS), SNS, Traction Motor(MD2K), Fan Motor, CT Sensor, Debug 통신
- UART 기반 SBUS 리모컨 / UART LCD 프로토콜 (참고용)

### 1.3 변경 이력

| 버전 | 날짜 | 변경 내용 | 작성자 |
|------|------|----------|--------|
| V0.1 | 2026-02-22 | 초안 작성 (DBC 250317 + FW 기반) | Claude |
| **V0.2** | **2026-02-23** | **Excel 260213 기준으로 전면 업데이트. 구동모터 DMKE→MD2K 교체, BMS 0x618~0x61B 재구성, 속도 Factor 0.01 반영, 신규 메시지 13건 추가, DBC 전용 26건 삭제** | **Claude** |

### 1.4 참조 문서

| # | 문서명 | 버전/날짜 | 비고 |
|---|--------|----------|------|
| 1 | **260213_01_SS_Proto_can_spec.xlsx** | **2026-02-13** | **기준 문서 (48개 메시지)** |
| 2 | SSVCU_CANdb_250317_01.dbc | 2025-03-17 | 보조 참조 (61개 메시지, 구버전) |
| 3 | X6_PLUS_CANdb.dbc | - | 팬모터 ESC DBC |
| 4 | VCU FW 소스 (SSVCU branch 20260105) | 2026-01-28 merge | AswCanMsg.h, Adt.h, Lcd.h, Bms.h 등 |
| 5 | 09_CAN_ICD_교차검증_Excel260213_vs_DBC.md | 2026-02-22 | Excel vs DBC 차이점 분석 |

### 1.5 용어 정의

| 약어 | 정의 |
|------|------|
| VCU | Vehicle Control Unit (차량 제어기, STM32F479) |
| ADT | Autonomous Driving Terminal (자율주행 PC, 마음AI) |
| BPA/BMS | Battery Pack Assembly / Battery Management System |
| SNS | Sensor Node (GPS + IMU) |
| LCD | Riverdi 터치 디스플레이 |
| HMI | Human Machine Interface |
| MD2K | MDROBOT MD2K PID 모터드라이버 (구동모터) |
| SBUS | Futaba S.Bus 리모컨 프로토콜 |
| DBC | Database CAN (Vector CANdb++ 형식) |
| DLC | Data Length Code |
| Ext ID | Extended CAN ID (29-bit) |
| Std ID | Standard CAN ID (11-bit) |

---

## 2. 네트워크 아키텍처

### 2.1 CAN 노드 구성

SS500 시스템은 **11개 노드**가 CAN1/CAN2 두 개의 버스에 연결된다.

```
                          ┌─────────────────┐
                          │      ADT PC     │
                          │   (마음AI, Linux)│
                          │ Protobuf↔CAN GW │
                          └────────┬────────┘
                                   │ CAN2
           ┌───────────────────────┼───────────────────────┐
           │                       │                       │
    ┌──────┴──────┐         ┌──────┴──────┐         ┌──────┴──────┐
    │  SNS Node   │         │     VCU     │         │  CT_Sensor  │
    │ (GPS+IMU)   │         │ (STM32F479) │         │ (전류센서)   │
    │ 0x341~0x347 │         │  Master     │         │   0x32C     │
    └─────────────┘         └──────┬──────┘         └─────────────┘
                                   │ CAN1
        ┌──────────┬───────────────┼───────────────┬──────────┐
        │          │               │               │          │
 ┌──────┴────┐ ┌───┴───────┐ ┌────┴─────┐ ┌───────┴──┐ ┌─────┴─────┐
 │   LCD     │ │ BPA(BMS)  │ │ MD2K 좌  │ │ MD2K 우  │ │ FAN Motor │
 │ (Riverdi) │ │ 통합 BMS  │ │ 0x064    │ │ 0x065    │ │ X6 Plus   │
 │ 0x321     │ │ 0x610~61B │ │ 0x764 Fb │ │ 0x765 Fb │ │ Ext CAN ID│
 └───────────┘ └───────────┘ └──────────┘ └──────────┘ └───────────┘
     UART                       MD2K PID     MD2K PID    CAN FD Ext
      │
 ┌────┴─────┐
 │  SBUS    │ ← UART (100kbps, inverted)
 │ 리모컨   │
 └──────────┘
```

### 2.2 노드 상세

| # | 노드명 | 역할 | CAN 버스 | ID 범위 | 프레임 형식 |
|---|--------|------|---------|---------|------------|
| 1 | **VCU** | 차량 제어 마스터 | CAN1+CAN2 | 0x064~0x332 | Standard |
| 2 | **ADT** | 자율주행 제어 PC | CAN2 | 0x301 | Standard |
| 3 | **LCD** | 사용자 인터페이스 | CAN1 (UART) | 0x321 | Standard |
| 4 | **BPA (BMS)** | 배터리 관리 | CAN1 | 0x610~0x61B | Standard |
| 5 | **SNS** | GPS/IMU 센서 | CAN2 | 0x341~0x347 | Standard |
| 6 | **MD2K_L** | 좌측 구동모터 (MDROBOT MD2K) | CAN1 | 0x064(Rx), 0x764(Tx) | Standard |
| 7 | **MD2K_R** | 우측 구동모터 (MDROBOT MD2K) | CAN1 | 0x065(Rx), 0x765(Tx) | Standard |
| 8 | **FAN Motor** | 팬모터 ESC (X6 Plus) | CAN1 | Ext 29-bit | Extended CAN FD |
| 9 | **CT_Sensor** | 외부 전류센서 | CAN2 | 0x32C | Standard |
| 10 | **HMI** | 외부 모니터링 | CAN1 | 0x700 | Standard |
| 11 | **Debug** | 디버그 모니터링 | CAN1 | 0x101~0x104 | Standard |

> **DBC 250317과 차이**: 구동모터가 DMKE CANopen(Node 0x0A/0x0B)에서 MDROBOT MD2K PID(0x064/0x065)로 전면 교체됨. BMS는 6모듈 개별(0x618~0x61D)에서 통합 진단(0x618~0x61B)으로 변경.

### 2.3 CAN 버스 할당

| 버스 | 속도 | 주요 노드 | 비고 |
|------|------|----------|------|
| **CAN1** | 500 kbps | VCU, LCD, BPA, MD2K L/R, FAN, HMI, Debug | 메인 제어 버스 |
| **CAN2** | 500 kbps | VCU, ADT, SNS, CT_Sensor | 자율주행/센서 버스 |

---

## 3. 물리 계층 사양

### 3.1 CAN 물리층

| 항목 | 사양 |
|------|------|
| 프로토콜 | CAN 2.0B + CAN FD (팬모터) |
| Bit Rate | 500 kbps (Nominal) |
| CAN FD Data Rate | 2 Mbps (팬모터 ESC) |
| 종단저항 | 120Ω (버스 양 끝단) |
| 커넥터 | 차량용 방수 커넥터 (상세 TBD) |
| 케이블 | Twisted Pair, 특성 임피던스 120Ω |

### 3.2 바이트 순서

| 항목 | 값 |
|------|-----|
| Byte Order | **Little Endian** (Intel) |
| Bit Numbering | LSB First |

### 3.3 프레임 구성

| 프레임 유형 | ID 길이 | DLC | 사용 노드 |
|------------|---------|-----|----------|
| Standard CAN | 11-bit | 0~8 bytes | VCU, ADT, LCD, BPA, SNS, CT, HMI, Debug, MD2K |
| Extended CAN FD | 29-bit | 2~8 bytes | FAN Motor (X6 Plus) |

---

## 4. 메시지 매트릭스

### 4.1 전체 메시지 목록

> **기준: 260213_01_SS_Proto_can_spec.xlsx (48개 메시지)**
> DBC 250317 전용 메시지(26건)는 삭제됨. 상세는 부록 10.1 참조.

#### 4.1.1 VCU ↔ ADT (자율주행)

| # | 메시지명 | CAN ID | Dec | DLC | 방향 | 주기 | 버스 |
|---|---------|--------|-----|-----|------|------|------|
| 1 | ADT2VCU1 | 0x301 | 769 | 8 | ADT → VCU | 50ms | CAN2 |
| 2 | VCU2ADT1 | 0x331 | 817 | 8 | VCU → ADT | 50ms | CAN2 |
| 3 | **VCU2ADT2** | **0x332** | **818** | **8** | **VCU → ADT** | **50ms** | **CAN2** |

> VCU2ADT2는 Excel 260213에서 신규 추가. 초음파센서 데이터 전달용.

#### 4.1.2 VCU ↔ LCD (사용자 인터페이스)

| # | 메시지명 | CAN ID | Dec | DLC | 방향 | 주기 | 버스 |
|---|---------|--------|-----|-----|------|------|------|
| 4 | VCU2LCD1 | 0x311 | 785 | 8 | VCU → LCD | 100ms | CAN1 |
| 5 | VCU2LCD2 | 0x312 | 786 | 8 | VCU → LCD | 100ms | CAN1 |
| 6 | VCU2LCD3 | 0x313 | 787 | 8 | VCU → LCD | 100ms | CAN1 |
| 7 | LCD2VCU1 | 0x321 | 801 | 8 | LCD → VCU | 50ms | CAN1 |

#### 4.1.3 VCU ↔ BPA (배터리)

| # | 메시지명 | CAN ID | Dec | DLC | 방향 | 주기 | 버스 | 비고 |
|---|---------|--------|-----|-----|------|------|------|------|
| 8 | BPA_Infro | 0x610 | 1552 | 8 | BPA → VCU | - | CAN1 | |
| 9 | BPA_Calc | 0x611 | 1553 | 8 | BPA → VCU | 10ms | CAN1 | |
| 10 | BPA_State | 0x612 | 1554 | 8 | BPA → VCU | 10ms | CAN1 | |
| 11 | BPA_ProtectState | 0x613 | 1555 | 8 | BPA → VCU | 100ms | CAN1 | |
| 12 | BPA_POWER | 0x614 | 1556 | 8 | BPA → VCU | 100ms | CAN1 | |
| 13 | BPA_Volt_state | 0x615 | 1557 | 8 | BPA → VCU | 100ms | CAN1 | |
| 14 | BPA_Temps_state | 0x616 | 1558 | 8 | BPA → VCU | 100ms | CAN1 | |
| 15 | BPA_VT_Position | 0x617 | 1559 | 8 | BPA → VCU | 100ms | CAN1 | |
| 16 | **BPA_Calc2** | **0x618** | **1560** | **8** | **BPA → VCU** | **100ms** | **CAN1** | **DBC: BPA_MD1_State → 완전 교체** |
| 17 | **BPA_MD_Infro** | **0x619** | **1561** | **8** | **BPA → VCU** | **100ms** | **CAN1** | **DBC: BPA_MD2_State → 완전 교체** |
| 18 | **BPA_Cell_Infro** | **0x61A** | **1562** | **8** | **BPA → VCU** | **100ms** | **CAN1** | **DBC: BPA_MD3_State → 완전 교체** |
| 19 | **BPA_COM_State** | **0x61B** | **1563** | **8** | **BPA → VCU** | **100ms** | **CAN1** | **DBC: BPA_MD4_State → 완전 교체** |
| 20 | VCU_RE | 0x600 | 1536 | 8 | VCU → BPA | 100ms | CAN1 | |

> DBC 250317의 BPA_MD5_State(0x61C), BPA_MD6_State(0x61D)는 Excel 260213에서 삭제됨.

#### 4.1.4 SNS → ADT (센서)

| # | 메시지명 | CAN ID | Dec | DLC | 방향 | 주기 | 버스 |
|---|---------|--------|-----|-----|------|------|------|
| 21 | SNS2ADT1 | 0x341 | 833 | 8 | SNS → ADT | 50ms | CAN2 |
| 22 | SNS2ADT2 | 0x342 | 834 | 8 | SNS → ADT | 50ms | CAN2 |
| 23 | SNS2ADT3 | 0x343 | 835 | 8 | SNS → ADT | 50ms | CAN2 |
| 24 | SNS2ADT4 | 0x344 | 836 | 8 | SNS → ADT | 100ms | CAN2 |
| 25 | SNS2ADT5 | 0x345 | 837 | 8 | SNS → ADT | 100ms | CAN2 |
| 26 | SNS2ADT6 | 0x346 | 838 | 8 | SNS → ADT | 100ms | CAN2 |
| 27 | **SNS2ADT7** | **0x347** | **839** | **8** | **SNS → ADT** | **50ms** | **CAN2** |

> SNS2ADT7은 Excel 260213에서 신규 추가. IMU 지자기센서(Magnetometer) 데이터.

#### 4.1.5 기타 Standard CAN

| # | 메시지명 | CAN ID | Dec | DLC | 방향 | 주기 | 버스 |
|---|---------|--------|-----|-----|------|------|------|
| 28 | CT_Sensor | 0x32C | 812 | 8 | CT → VCU | - | CAN2 |
| 29 | HMI | 0x700 | 1792 | 8 | HMI → VCU | - | CAN1 |

#### 4.1.6 구동모터 (MDROBOT MD2K PID, Standard CAN)

| # | 메시지명 | CAN ID | Dec | DLC | 방향 | 주기 | 비고 |
|---|---------|--------|-----|-----|------|------|------|
| 30 | **VCU2MD2K1** | **0x064** | **100** | **8** | **VCU → 좌모터** | **10ms** | PID 명령 |
| 31 | **VCU2MD2K2** | **0x065** | **101** | **8** | **VCU → 우모터** | **10ms** | PID 명령 |
| 32 | **MD2K2VCU1** | **0x764** | **1892** | **8** | **좌모터 → VCU** | **10ms** | 상태 피드백 |
| 33 | **MD2K2VCU2** | **0x765** | **1893** | **8** | **우모터 → VCU** | **10ms** | 상태 피드백 |

> **DBC 250317과 차이**: DMKE CANopen 메시지 17개(Standard 11 + Extended 6)가 MD2K PID 4개로 전면 교체됨.

#### 4.1.7 디버그 (Standard CAN)

| # | 메시지명 | CAN ID | Dec | DLC | 방향 | 주기 | 비고 |
|---|---------|--------|-----|-----|------|------|------|
| 34 | **state_debug_M1** | **0x101** | **257** | **8** | **VCU → Ext** | **10ms** | 좌측 모터 디버그 |
| 35 | **state_debug_M2** | **0x102** | **258** | **8** | **VCU → Ext** | **10ms** | 우측 모터 디버그 |
| 36 | **state_debug_RMT** | **0x104** | **260** | **8** | **VCU → Ext** | **10ms** | 리모컨 디버그 |

> **DBC 250317과 차이**: SS500_Debug_Rx(0x100), SS500_Debug_Tx(0x110)가 state_debug_M1/M2/RMT로 교체됨.

#### 4.1.8 팬모터 (Extended CAN FD)

| # | 메시지명 | Ext CAN ID | DLC | 방향 | 비고 |
|---|---------|------------|-----|------|------|
| 37 | **VCU2FAN1** | **0x004E8401** | 8 | VCU → FAN | 4ch 스로틀 명령 |
| 38 | **FAN2VCU1** | **0x1F4E5201** | 8 | FAN → VCU | RPM/상태 |
| 39 | **FAN2VCU2** | **0x1F4E5301** | 8 | FAN → VCU | 전압/전류 |
| 40 | **FAN2VCU3** | **0x1F4E5401** | 8 | FAN → VCU | 상세 온도 |

> **DBC 250317과 차이**: Extended CAN ID 전면 변경. DBC의 설정 메시지 5건, Legacy MSG 3건은 Excel에 미포함.
> DBC ID: VCU→FAN 0x80640401, FAN→VCU 0xA0640801/0xA0640A01/0xA0640C01

**합계: Excel 260213 기준 40개 메시지** (Standard 36개 + Extended 4개)

> **참고**: Excel CanSpec 시트에는 48개 메시지가 기재되어 있으나, 팬모터 DBC 설정 메시지(5건), Legacy 메시지(3건)를 제외하면 실 운용 메시지는 40개이다.

---

## 5. 시그널 상세 정의

### 5.1 ADT2VCU1 (0x301, 769) — ADT → VCU, 50ms

> 자율주행 PC에서 VCU로 전달하는 제어 명령

| Bit | Signal | 비트폭 | Type | Factor | Offset | Min | Max | 단위 | 설명 |
|-----|--------|-------|------|--------|--------|-----|-----|------|------|
| 0 | CF_ADTValid | 1 | U | 1 | 0 | 0 | 1 | flag | 자율주행 시스템 유효 (0=Invalid, 1=Valid) |
| 1 | CF_ADTAutoCtrlEnblSt | 1 | U | 1 | 0 | 0 | 1 | flag | 자율주행 동작 중 (0=Not, 1=Operating) |
| 2-3 | CF_ADTRtrnSt | 2 | U | 1 | 0 | 0 | 3 | - | 복귀 상태 |
| 4-5 | CF_ADTGoBackSt | 2 | U | 1 | 0 | 0 | 3 | - | 회귀 상태 |
| 6-7 | Reserved | 2 | - | - | - | - | - | - | 예약 |
| 8-11 | CF_ADTDrvCnd | 4 | U | 1 | 0 | 0 | 15 | - | 주행 조건 (eADTDrvCnd 참조) |
| 12-15 | CF_ADTSysFault | 4 | U | 1 | 0 | 0 | 15 | - | 시스템 고장코드 (FC_25~34 매핑) |
| 16-31 | CF_ADTLeftVehSpdCtrl | **16** | **S16** | **0.01** | 0 | -4 | 4 | km/h | 좌측 속도 명령 |
| 32-47 | CF_ADTRightVehSpdCtrl | **16** | **S16** | **0.01** | 0 | -4 | 4 | km/h | 우측 속도 명령 |
| 48-55 | Reserved | 8 | U | - | - | - | - | - | 예약 |
| 56-59 | CF_ADTWarnMsg | 4 | U | 1 | 0 | 0 | 15 | - | 경고 메시지 (WC_1~5 매핑) |
| 60-63 | CF_ADTAliveCnt | 4 | U | 1 | 0 | 0 | 15 | cnt | Alive Counter (0→15→0 순환) |

> **DBC 250317과 차이**: CF_ADTRtrnSt(bit2-3), CF_ADTGoBackSt(bit4-5) 신규 추가. CF_ADTLeftVehSpdCtrl/RightVehSpdCtrl이 8bit→16bit, Factor 0.1→**0.01**로 변경. CF_ADTWarnMsg 위치 bit4-7 → bit56-59로 이동. CF_ADTAliveCnt 위치 bit56-59 → bit60-63으로 이동.

### 5.2 VCU2ADT1 (0x331, 817) — VCU → ADT, 50ms

> VCU에서 자율주행 PC로 전달하는 차량 상태

| Bit | Signal | 비트폭 | Type | Factor | Offset | Min | Max | 단위 | 설명 |
|-----|--------|-------|------|--------|--------|-----|-----|------|------|
| 0 | CF_ADTAutoCtrlEnblSwt | 1 | U | 1 | 0 | 0 | 1 | flag | 자율주행 활성화 스위치 |
| 1 | CF_ManEmgyStopSwt | 1 | S | 1 | 0 | 0 | 1 | flag | 수동 비상정지 (0=OFF, 1=ON) |
| 2 | CF_LowBatRtrnReq | 1 | U | 1 | 0 | 0 | 1 | ON/OFF | 배터리부족 복귀요청 |
| 3 | CF_TankLvlRtrnReq | 1 | U | 1 | 0 | 0 | 1 | flag | 탱크레벨 복귀요청 |
| 4-5 | CF_stSysFault | 2 | U | 1 | 0 | 0 | 3 | - | 시스템고장 (0=정상, 1=운전불가, 2=작업불가) |
| 6 | CF_BumpSnsDet | 1 | U | 1 | 0 | 0 | 1 | flag | 범퍼센서 감지 (0=정상, 1=감지) |
| 7 | Reserved_1 | 1 | U | - | - | - | - | - | 예약 |
| 8-15 | CF_VehicleStatus | 8 | U | 1 | 0 | 0 | 255 | - | 차량 상태 |
| 16-31 | CF_LeftVehSpdAct | 16 | S | **0.01** | 0 | -4 | 4 | km/h | 좌측 실제 속도 |
| 32-47 | CF_RightVehSpdAct | 16 | S | **0.01** | 0 | -4 | 4 | km/h | 우측 실제 속도 |
| 48-55 | CF_LCDVehSpdMaxCfg | 8 | U | **0.1** | 0 | 0.5 | 3 | km/h | 최대속도 설정값 |
| 56-59 | Reserved4 | 4 | U | - | - | - | - | - | 예약 |
| 60-63 | CF_VCUAliveCnt | 4 | U | 1 | 0 | 0 | 15 | cnt | VCU Alive Counter |

> **DBC 250317과 차이**: CF_VehicleStatus(byte1) 신규 추가. CF_LeftVehSpdAct/RightVehSpdAct Factor 0.1→**0.01**. CF_LCDVehSpdMaxCfg Factor 1→**0.1**, Range 0~255→**0.5~3** km/h.

### 5.3 VCU2ADT2 (0x332, 818) — VCU → ADT, 50ms [신규]

> VCU에서 자율주행 PC로 전달하는 초음파센서 데이터

| Bit | Signal | 비트폭 | Type | Factor | Offset | Min | Max | 단위 | 설명 |
|-----|--------|-------|------|--------|--------|-----|-----|------|------|
| 0-15 | CF_UltraSonSns_Mid | 16 | U | 1 | 0 | 0 | 65535 | mm | 중앙 초음파센서 거리 |
| 16-63 | Reserved | 48 | - | - | - | - | - | - | 예약 |

> **DBC 250317과 차이**: Excel 260213에서 신규 추가된 메시지. DBC에 존재하지 않음.

### 5.4 VCU2LCD1 (0x311, 785) — VCU → LCD, 100ms

> VCU에서 LCD로 전달하는 차량 실시간 상태

| Byte | Bit | Signal | 비트폭 | Type | Factor | Min | Max | 단위 | 설명 |
|------|-----|--------|-------|------|--------|-----|-----|------|------|
| 0 | 0-7 | CF_VehSpd | 8 | S | 0.1 | -4 | 4 | km/h | 차량 속도 |
| 1 | 8-12 | CF_SysWarnMsg | **5** | U | 1 | 0 | 31 | - | 시스템 경고 (1~15=ADT, 16~31=VCU) |
| 1 | 13 | CF_ADTValid | 1 | U | 1 | 0 | 1 | flag | ADT 유효 상태 |
| 1 | 14-15 | Reserved8_1 | 2 | - | - | - | - | - | 예약 |
| 2 | 16-18 | CF_ADTOpSt | 3 | U | 1 | 0 | 7 | - | ADT 운영 상태 |
| 2 | 19-23 | Reserved8_2 | 5 | - | - | - | - | - | 예약 |
| 3 | 24-31 | Reserved8_3 | 8 | - | - | - | - | - | 예약 |
| 4-5 | 32-47 | Reserved16_2 | 16 | - | - | - | - | - | 예약 |
| 6 | 48 | CF_StMixer | 1 | U | 1 | 0 | 1 | flag | 교반기 상태 |
| 6 | 49 | CF_StRemoteCtrl | 1 | U | 1 | 0 | 1 | flag | 리모컨 활성 상태 |
| 6 | 50-51 | CF_StPmp | 2 | U | 1 | 0 | 3 | step | 펌프 상태 (0=OFF, 1~3=단계) |
| 6 | 52-53 | CF_StFan | 2 | U | 1 | 0 | 3 | step | 팬 상태 (0=OFF, 1~3=단계) |
| 6 | 54 | CF_StHdLamp | 1 | U | 1 | 0 | 1 | flag | 헤드램프 |
| 6 | 55 | CF_StEmgyStop | 1 | S | 1 | 0 | 1 | flag | 비상정지 상태 |
| 7 | 56-57 | CF_StSolnLD | 2 | U | 1 | 0 | 3 | - | 좌측 솔레노이드 |
| 7 | 58-59 | CF_StSolnRD | 2 | U | 1 | 0 | 3 | - | 우측 솔레노이드 |
| 7 | 60-61 | GPS_St_CarrSoln | 2 | U | 1 | 0 | 2 | - | GPS 반송파 해 |
| 7 | 62-63 | GPS_Info_Idx | 2 | U | 1 | 0 | 3 | - | GPS 정보 인덱스 |

> **DBC 250317과 차이**: CF_SysWarnMsg 8bit→**5bit**, CF_ADTValid(bit13) 신규 추가, CF_ADTOpSt(bit16-18) 신규 추가, GPS_St_CarrSoln(bit60-61) 신규 추가.

**솔레노이드 값 정의:**

| 값 | CF_StSolnLD (좌측) | CF_StSolnRD (우측) |
|----|-------------------|-------------------|
| 0 | 1/2/3/4 전부 개방 | 5/6/7/8 전부 개방 |
| 1 | 1/2 닫힘 (상단) | 7/8 닫힘 |
| 2 | 3/4 닫힘 (하단) | 5/6 닫힘 |
| 3 | 1/2/3/4 전부 닫힘 | 5/6/7/8 전부 닫힘 |

### 5.5 VCU2LCD2 (0x312, 786) — VCU → LCD, 100ms

> 누적 데이터 전달

| Byte | Signal | 비트폭 | Type | Factor | Min | Max | 단위 | 설명 |
|------|--------|-------|------|--------|-----|-----|------|------|
| 0 | CF_TankLvl | 8 | U | 1 | 0 | 100 | % | 탱크 잔량 |
| 1-2 | CF_AccumSprayHour | 16 | U | 1 | 0 | 65535 | h | 누적 분사시간 |
| 3-4 | CF_AccumDist | 16 | U | 1 | 0 | 65535 | km | 누적 주행거리 |
| 5 | CF_AccumSprayHour_pnt | 8 | U | 1 | 0 | 255 | - | 분사시간 소수점 |
| 6[0:3] | CF_AccumDist_pnt | 4 | U | **0.1** | 0 | 1.5 | - | 주행거리 소수점 |
| 6[4:7] | Reserved | 4 | - | - | - | - | - | 예약 |
| 7 | Reserved4 | 8 | - | - | - | - | - | 예약 |

> **DBC 250317과 차이**: CF_AccumSprayHour_pnt(byte5), CF_AccumDist_pnt(byte6 하위4bit) 신규 추가. DBC의 CF_AccumSprayLiter(20bit) 삭제됨.

### 5.6 VCU2LCD3 (0x313, 787) — VCU → LCD, 100ms

> 고장코드 전달 (최대 8개 동시 표시)

| Byte | Signal | 비트폭 | Type | Factor | Min | Max | 단위 | 설명 |
|------|--------|-------|------|--------|-----|-----|------|------|
| 0 | CF_FaultCode1 | 8 | U | 1 | 0 | 255 | - | 고장코드 #1 (eFaultCode enum) |
| 1 | CF_FaultCode2 | 8 | U | 1 | 0 | 255 | - | 고장코드 #2 |
| 2 | CF_FaultCode3 | 8 | U | 1 | 0 | 255 | - | 고장코드 #3 |
| 3 | CF_FaultCode4 | 8 | U | 1 | 0 | 255 | - | 고장코드 #4 |
| 4 | CF_FaultCode5 | 8 | U | 1 | 0 | 255 | - | 고장코드 #5 |
| 5 | CF_FaultCode6 | 8 | U | 1 | 0 | 255 | - | 고장코드 #6 |
| 6 | CF_FaultCode7 | 8 | U | 1 | 0 | 255 | - | 고장코드 #7 |
| 7 | CF_FaultCode8 | 8 | U | 1 | 0 | 255 | - | 고장코드 #8 |

> 고장코드 값은 섹션 6 참조. 고장 없을 경우 0. Excel/DBC 일치.

### 5.7 LCD2VCU1 (0x321, 801) — LCD → VCU, 50ms

> LCD에서 VCU로 전달하는 사용자 제어 입력

| Byte | Bit | Signal | 비트폭 | Type | Factor | Min | Max | 단위 | 설명 |
|------|-----|--------|-------|------|--------|-----|-----|------|------|
| 0 | 0 | CF_LCDMixerCtrl | 1 | U | 1 | 0 | 1 | flag | 교반기 제어 |
| 0 | 1 | Reserved1 | 1 | U | - | - | - | - | 예약 |
| 0 | 2-3 | CF_LCDPmpCtrl | 2 | U | 1 | 0 | 3 | step | 펌프 단계 |
| 0 | 4-5 | CF_LCDFanCtrl | 2 | U | 1 | 0 | 3 | step | 팬 단계 |
| 0 | 6 | CF_LCDHdLampCtrl | 1 | U | 1 | 0 | 1 | flag | 헤드램프 |
| 0 | 7 | **CF_LCDGoBackReq** | **1** | **U** | **1** | **0** | **1** | **flag** | **회귀 요청** |
| 1 | 8-9 | CF_LCDSolnLDCtrl | 2 | U | 1 | 0 | 3 | - | 좌측 솔레노이드 |
| 1 | 10-11 | CF_LCDSolnRDCtrl | 2 | U | 1 | 0 | 3 | - | 우측 솔레노이드 |
| 1 | 12-15 | Reserved4 | 4 | - | - | - | - | - | 예약 |
| 2-3 | 16-31 | CF_LCDBatChrgStatDist | 16 | U | 1 | 0 | 500 | m | 충전소 거리 설정 |
| 4 | 32-39 | CF_LCDVehSpdMaxCfg | 8 | U | **0.1** | **0.5** | **3** | **km/h** | 최대속도 설정 |
| 5-7 | 40-63 | Reserved | 24 | - | - | - | - | - | 예약 |

> **DBC 250317과 차이**: CF_LCDGoBackReq(bit7) 신규 추가. CF_LCDVehSpdMaxCfg Factor 1→**0.1**, Range 0~255→**0.5~3** km/h.

### 5.8 BPA_Infro (0x610, 1552) — BPA → VCU

> 배터리 고정 정보 (부팅 시 1회 전송)

| Byte | Signal | 비트폭 | Type | Factor | Min | Max | 단위 | 기본값 | 설명 |
|------|--------|-------|------|--------|-----|-----|------|-------|------|
| 0 | BPA_Type | 8 | U | 1 | 0 | 255 | type | 03 | 제품 타입 |
| 1 | BPA_SW_Ver | 8 | U | 1 | 0 | 255 | VER | 10 | S/W 버전 |
| 2 | BPA_Serial | 8 | U | 1 | 0 | 255 | S | 14 | 직렬 셀 수 |
| 3 | BPA_Parallel | 8 | U | 1 | 0 | 255 | P | 6 | 병렬 셀 수 |
| 4-5 | BPA_NorVolt | 16 | U | 0.1 | 0 | 900 | V | 51.8 | 정격전압 |
| 6-7 | BPA_Capacity | 16 | U | 0.1 | 0 | 500 | Ah | 300 | 정격용량 |

> Excel/DBC 일치.

### 5.9 BPA_Calc (0x611, 1553) — BPA → VCU, 10ms

> 배터리 핵심 계산값

| Byte | Signal | 비트폭 | Type | Factor | Offset | Min | Max | 단위 | 설명 |
|------|--------|-------|------|--------|--------|-----|-----|------|------|
| 0-1 | BPA_Volt_Total | 16 | S | 0.1 | 0 | 0 | 900 | V | 총 전압 |
| 2-3 | BPA_Curr_Total | 16 | S | 0.1 | 0 | -500 | 500 | A | 총 전류 |
| 4-5 | BPA_SOC | 16 | U | 0.1 | 0 | 0 | 100 | % | 충전 상태 |
| 6-7 | BPA_SOH | 16 | S | 0.1 | 0 | 0 | 100 | % | 건강 상태 |

> Excel/DBC 일치.

### 5.10 BPA_State (0x612, 1554) — BPA → VCU, 10ms

> 배터리 장치/릴레이/시스템 상태 — **Excel 260213에서 대폭 확장 (7→37 시그널)**

| Byte | Bit | Signal | 비트폭 | Type | 설명 |
|------|-----|--------|-------|------|------|
| 0 | 0-7 | BPA_Divice_Status | 8 | U | 0x00=Init, 0x01=Standby, 0x02=Charger, 0x03=Discharger, 0x04=EmgyStop, 0x05=Fault |
| 1 | 8-15 | BPA_Protect_Status | 8 | U | 0x00=Normal, 0x01=Warning, 0x02=Fault, 0x03=Protection |
| 2 | 16 | BPA_Balance | 1 | U | 셀 밸런싱 (0=OFF, 1=ON) |
| 2 | 17 | BPA_Neg_Rly | 1 | U | 음극 릴레이 (0=OFF, 1=ON) |
| 2 | 18 | BPA_Pos_Rly | 1 | U | 양극 릴레이 (0=OFF, 1=ON) |
| 2 | 19 | BPA_PreChar_Rly | 1 | U | 프리차지 릴레이 |
| 2 | 20 | BPA_MSD_AUX | 1 | U | AUX 퓨즈 (0=정상, 1=비정상) |
| 2 | 21 | BPA_ChargerEn | 1 | U | 충전기 활성화 |
| 2 | 22 | BPA_KillSW | 1 | U | 킬스위치 상태 |
| 2 | 23 | BPA_PWRBMS_Hold | 1 | U | BMS 전원 유지 |
| 3 | 24-31 | Reserved | 8 | - | 예약 |
| 4 | 32-34 | Sys_SysSeqState | 3 | U | 시스템 시퀀스 상태 |
| 4 | 35-37 | SyS_RlySeqState | 3 | U | 릴레이 시퀀스 상태 |
| 4 | 38-39 | Sys_SocSeqState | 2 | U | SOC 시퀀스 상태 |
| 5 | 40 | Sys_INITOK | 1 | U | 초기화 완료 |
| 5 | 41 | CANCOMEnable | 1 | U | CAN 통신 활성화 |
| 5 | 42 | VCURlyWakeUp | 1 | U | VCU 릴레이 Wakeup |
| 5 | 43 | ChargerWakeUp | 1 | U | 충전기 Wakeup |
| 5 | 44 | Sys_SocMode | 1 | U | SOC 모드 |
| 5 | 45 | Sys_BalaMode | 1 | U | 밸런싱 모드 |
| 5 | 46 | Sys_BalanceEn | 1 | U | 밸런싱 활성화 |
| 5 | 47 | Sys_DisCharMode | 1 | U | 방전 모드 |
| 6 | 48 | Sys_Aalarm | 1 | U | 시스템 알람 |
| 6 | 49 | Sys_Fault | 1 | U | 시스템 고장 |
| 6 | 50 | Sys_Prtct | 1 | U | 시스템 보호 |
| 6 | 51 | Sys_RlyDO_NRly | 1 | U | 릴레이 DO: 음극 |
| 6 | 52 | Sys_RlyDO_PRly | 1 | U | 릴레이 DO: 양극 |
| 6 | 53 | Sys_RlyDO_PreRly | 1 | U | 릴레이 DO: 프리차지 |
| 6 | 54 | Sys_CellVoltOk | 1 | U | 셀 전압 정상 |
| 6 | 55 | Sys_CellTempsOk | 1 | U | 셀 온도 정상 |
| 7 | 56 | Sys_MSDERR | 1 | U | MSD 에러 |
| 7 | 57 | Sys_RlyERR | 1 | U | 릴레이 에러 |
| 7 | 58 | Sys_CANCOMERR | 1 | U | CAN 통신 에러 |
| 7 | 59 | Sys_ISOSPICOMERR | 1 | U | ISOSPI 통신 에러 |
| 7 | 60 | Sys_IMDRegErr | 1 | U | 절연저항 에러 |
| 7 | 61 | Sys_HMICOMMode | 1 | U | HMI 통신 모드 |
| 7 | 62 | Sys_HMIBalanceMode | 1 | U | HMI 밸런싱 모드 |
| 7 | 63 | Reserved | 1 | - | 예약 |

> **DBC 250317과 차이**: byte2에 BPA_ChargerEn/KillSW/PWRBMS_Hold 추가. byte4~7 전체가 BPA_Ah(16bit)에서 시스템 상태 비트필드(30개 시그널)로 완전 교체됨.

### 5.11 BPA_ProtectState (0x613, 1555) — BPA → VCU, 100ms

> 배터리 보호/경고 상태 — **Excel 260213에서 3계층(Warning/Fault/Protection) 구조로 변경**

**Warning 플래그 (Byte 0-1, bit 0~15):**

| Byte | Bit | Signal | 설명 |
|------|-----|--------|------|
| 0 | 0 | BPA_Wn_OC | 과전류 경고 |
| 0 | 1 | BPA_Wn_Chager_OC | 충전 과전류 경고 |
| 0 | 2 | BPA_Wn_SOC_OV | SOC 과충전 경고 |
| 0 | 3 | BPA_Wn_SOC_UV | SOC 저충전 경고 |
| 0 | 4 | BPA_Wn_OV | 팩 과전압 경고 |
| 0 | 5 | BPA_Wn_UV | 팩 저전압 경고 |
| 0 | 6 | BPA_Wn_Cell_OV | 셀 과전압 경고 |
| 0 | 7 | BPA_Wn_Cell_UV | 셀 저전압 경고 |
| 1 | 8 | BPA_Wn_Cell_UnbalV | 셀 전압 불균형 경고 |
| 1 | 9 | BPA_Wn_DisCharger_Cell_OT | 방전 셀 과온 경고 |
| 1 | 10 | BPA_Wn_Charger_Cell_OT | 충전 셀 과온 경고 |
| 1 | 11 | BPA_Wn_DisCharger_Cell_UT | 방전 셀 저온 경고 |
| 1 | 12 | BPA_Wn_Charger_Cell_UT | 충전 셀 저온 경고 |
| 1 | 13 | BPA_Wn_Cell_UnbalT | 셀 온도 불균형 경고 |
| 1 | 14 | BPA_Wn_Discharger_UnBPAWR | 방전 전력 불균형 경고 |
| 1 | 15 | BPA_Wn_Charger_UnBPAWR | 충전 전력 불균형 경고 |

**Fault 플래그 (Byte 2-3, bit 16~31) [신규 계층]:**

| Byte | Bit | Signal | 설명 |
|------|-----|--------|------|
| 2 | 16 | BPA_FLT_OC | 과전류 Fault |
| 2 | 17 | BPA_FLT_Chager_OC | 충전 과전류 Fault |
| 2 | 18 | BPA_FLT_SOC_OV | SOC 과충전 Fault |
| 2 | 19 | BPA_FLT_SOC_UV | SOC 저충전 Fault |
| 2 | 20 | BPA_FLT_OV | 팩 과전압 Fault |
| 2 | 21 | BPA_FLT_UV | 팩 저전압 Fault |
| 2 | 22 | BPA_FLT_Cell_OV | 셀 과전압 Fault |
| 2 | 23 | BPA_FLT_Cell_UV | 셀 저전압 Fault |
| 3 | 24 | BPA_FLT_Cell_UnbalV | 셀 전압 불균형 Fault |
| 3 | 25 | BPA_FLT_DisCharger_Cell_OT | 방전 셀 과온 Fault |
| 3 | 26 | BPA_FLT_Charger_Cell_OT | 충전 셀 과온 Fault |
| 3 | 27 | BPA_FLT_DisCharger_Cell_UT | 방전 셀 저온 Fault |
| 3 | 28 | BPA_FLT_Charger_Cell_UT | 충전 셀 저온 Fault |
| 3 | 29 | BPA_FLT_Cell_UnbalT | 셀 온도 불균형 Fault |
| 3 | 30 | BPA_FLT_Discharger_UnBPAWR | 방전 전력 불균형 Fault |
| 3 | 31 | BPA_FLT_Charger_UnBPAWR | 충전 전력 불균형 Fault |

**Protection 플래그 (Byte 4-7, bit 32~53):**

| Byte | Bit | Signal | 설명 |
|------|-----|--------|------|
| 4 | 32 | BPA_Prtct_OC | 과전류 보호 |
| 4 | 33 | BPA_Prtct_Chager_OC | 충전 과전류 보호 |
| 4 | 34 | BPA_Prtct_SOC_OV | SOC 과충전 보호 |
| 4 | 35 | BPA_Prtct_SOC_Un | SOC 저충전 보호 |
| 4 | 36 | BPA_Prtct_OV | 팩 과전압 보호 |
| 4 | 37 | BPA_Prtct_UV | 팩 저전압 보호 |
| 4 | 38 | BPA_Prtct_Cell_OV | 셀 과전압 보호 |
| 4 | 39 | BPA_Prtct_Cell_UV | 셀 저전압 보호 |
| 5 | 40 | BPA_Prtct_Cell_UnbalV | 셀 전압 불균형 보호 |
| 5 | 41 | BPA_Prtct_DisCharger_Cell_OT | 방전 셀 과온 보호 |
| 5 | 42 | BPA_Prtct_Charger_Cell_OT | 충전 셀 과온 보호 |
| 5 | 43 | BPA_Prtct_DisCharger_Cell_UT | 방전 셀 저온 보호 |
| 5 | 44 | BPA_Prtct_Charger_Cell_UT | 충전 셀 저온 보호 |
| 5 | 45 | BPA_Prtct_Cell_UnbalT | 셀 온도 불균형 보호 |
| 5 | 46 | BPA_Prtct_Discharger_UnBPAWR | 방전 전력 불균형 보호 |
| 5 | 47 | BPA_Prtct_Charger_UnBPAWR | 충전 전력 불균형 보호 |
| 6 | 48 | BPA_Prtct_VCUCom_Err | VCU 통신 에러 보호 |
| 6 | 49 | BPA_Prtct_CharCom_Err | 충전기 통신 에러 보호 |
| 6 | 50 | BPA_Prtct_CTCom_Err | CT센서 통신 에러 보호 |
| 6 | 51 | BPA_Prtct_Rly_Err | 릴레이 고장 보호 |
| 6 | 52 | BPA_Prtct_Discharger_PwrErr | 방전 전력 에러 보호 |
| 6 | 53 | BPA_Prtct_Charger_PwrErr | 충전 전력 에러 보호 |
| 6-7 | 54-63 | Reserved | 예약 |

> **DBC 250317과 차이**: Fault 계층(bit16~31) 전체 신규 추가. Protection에 통신/전력 오류 항목 추가 (Prtct_VCUCom_Err, Prtct_CharCom_Err, Prtct_CTCom_Err, Prtct_Discharger_PwrErr, Prtct_Charger_PwrErr). DBC의 2계층→Excel의 3계층으로 변경.

### 5.12 BPA_POWER (0x614, 1556) — BPA → VCU, 100ms

> 전력 제한값

| Byte | Signal | 비트폭 | Type | Factor | Min | Max | 단위 | 설명 |
|------|--------|-------|------|--------|-----|-----|------|------|
| 0-1 | BPA_Charge_Cont_PL | 16 | U | 0.1 | 0 | 20 | kW | 충전 연속 전력 제한 |
| 2-3 | BPA_Discharge_Cont_PL | 16 | U | 0.1 | 0 | 20 | kW | 방전 연속 전력 제한 |
| 4-5 | BPA_Charge_Peak_PL | 16 | U | 0.1 | 0 | 20 | kW | 충전 피크 전력 제한 |
| 6-7 | BPA_Discharge_Peak_PL | 16 | U | 0.1 | 0 | 20 | kW | 방전 피크 전력 제한 |

> Excel/DBC 일치.

### 5.13 BPA_Volt_state (0x615, 1557) — BPA → VCU, 100ms

| Byte | Signal | 비트폭 | Type | Factor | Min | Max | 단위 | 설명 |
|------|--------|-------|------|--------|-----|-----|------|------|
| 0-1 | BSA_Cell_MaxV | 16 | U | 0.001 | 0 | 4.5 | V | 셀 최대전압 |
| 2-3 | BPA_Cell_MinV | 16 | U | 0.001 | 0 | 4.5 | V | 셀 최소전압 |
| 4-5 | BPA_Cell_AVGV | 16 | U | 0.001 | 0 | 4.5 | V | 셀 평균전압 |
| 6-7 | BPA_Cell_DeviV | 16 | U | 1 | 0 | 1000 | mV | 셀 전압편차 |

> Excel/DBC 일치.

### 5.14 BPA_Temps_state (0x616, 1558) — BPA → VCU, 100ms

| Byte | Signal | 비트폭 | Type | Factor | Min | Max | 단위 | 설명 |
|------|--------|-------|------|--------|-----|-----|------|------|
| 0-1 | BPA_Cell_MaxT | 16 | S | 0.1 | -30 | 100 | deg C | 셀 최대온도 |
| 2-3 | BPA_Cell_MinT | 16 | S | 0.1 | -30 | 100 | deg C | 셀 최소온도 |
| 4-5 | BPA_Cell_AVGT | 16 | S | 0.1 | -30 | 100 | deg C | 셀 평균온도 |
| 6-7 | BPA_Cell_DeviT | 16 | U | 0.1 | 0 | 100 | deg C | 셀 온도편차 |

> Excel/DBC 일치.

### 5.15 BPA_VT_Position (0x617, 1559) — BPA → VCU, 100ms

| Byte | Signal | 비트폭 | Type | Factor | Min | Max | 단위 | 설명 |
|------|--------|-------|------|--------|-----|-----|------|------|
| 0-1 | BPA_Max_Volt_Pos | 16 | U | 1 | 0 | 200 | Pos | 최대전압 셀 위치 |
| 2-3 | BPA_Min_Volt_Pos | 16 | U | 1 | 0 | 200 | Pos | 최소전압 셀 위치 |
| 4-5 | BPA_Max_Temps_Pos | 16 | U | 1 | 0 | 200 | Pos | 최대온도 셀 위치 |
| 6-7 | BPA_Min_Temps_pos | 16 | U | 1 | 0 | 200 | Pos | 최소온도 셀 위치 |

> Excel/DBC 일치.

### 5.16 BPA_Calc2 (0x618, 1560) — BPA → VCU, 100ms [신규]

> 배터리 용량/절연저항 정보 — **DBC의 BPA_MD1_State를 완전 대체**

| Byte | Signal | 비트폭 | Type | Factor | Min | Max | 단위 | 설명 |
|------|--------|-------|------|--------|-----|-----|------|------|
| 0-1 | BPA_BatCapacity | 16 | U | 0.1 | 0 | 6553 | Ah | 배터리 용량 |
| 2-3 | BPA_BatISORegs | 16 | U | 0.1 | 0 | 6553 | MOhm | 절연저항 |
| 4-5 | BPA_CellRegsMax | 16 | U | 0.1 | 0 | 6553 | mOhm | 셀 최대내부저항 |
| 6-7 | BPA_BatSocOldCapacity | 16 | U | 0.1 | 0 | 6553 | Ah | NVRAM 저장 SOC 용량 |

> **DBC 250317과 차이**: 동일 CAN ID(0x618)이지만 메시지 내용이 완전 교체됨. DBC에서는 BPA_MD1_State(모듈1 전압/온도).

### 5.17 BPA_MD_Infro (0x619, 1561) — BPA → VCU, 100ms [신규]

> 배터리 모듈 정보 — **DBC의 BPA_MD2_State를 완전 대체**

| Byte | Signal | 비트폭 | Type | Factor | Min | Max | 단위 | 설명 |
|------|--------|-------|------|--------|-----|-----|------|------|
| 0-1 | BPA_BMNum | 16 | U | 0.1 | 0 | 6553 | - | 모듈 번호 |
| 2-3 | BPA_BM_Volt_Total[Num] | 16 | U | 0.1 | 0 | 6553 | V | 모듈 전압 |
| 4-5 | BPA_BM1_Cell_AVGV[Num] | 16 | U | 0.001 | 0 | 65.5 | V | 셀 평균전압 |
| 6-7 | BPA_BM1_Cell_AVGT[Num] | 16 | U | 0.1 | 0 | 6553 | deg C | 셀 평균온도 |

> **DBC 250317과 차이**: 동일 CAN ID(0x619)이지만 메시지 내용이 완전 교체됨. [Num]은 BPA_BMNum에 의한 인덱싱.

### 5.18 BPA_Cell_Infro (0x61A, 1562) — BPA → VCU, 100ms [신규]

> 셀별 전압/온도/저항 정보 — **DBC의 BPA_MD3_State를 완전 대체**

| Byte | Signal | 비트폭 | Type | Factor | Min | Max | 단위 | 설명 |
|------|--------|-------|------|--------|-----|-----|------|------|
| 0-1 | BPA_CellNum | 16 | U | 1 | 0 | 384 | Num | 셀 번호 |
| 2-3 | BPA_CellVoltage[Num] | 16 | U | 0.001 | 0 | 65.5 | V | 셀 전압 |
| 4-5 | BPA_CellTemperature[Num] | 16 | U | 0.1 | 0 | 6553 | deg C | 셀 온도 |
| 6-7 | BPA_CellRegs[Num] | 16 | U | 0.1 | 0 | 6553 | mOhm | 셀 내부저항 |

> **DBC 250317과 차이**: 동일 CAN ID(0x61A)이지만 메시지 내용이 완전 교체됨.

### 5.19 BPA_COM_State (0x61B, 1563) — BPA → VCU, 100ms [신규]

> 통신 상태 카운터 — **DBC의 BPA_MD4_State를 완전 대체**

| Byte | Signal | 비트폭 | Type | Factor | Min | Max | 단위 | 설명 |
|------|--------|-------|------|--------|-----|-----|------|------|
| 0 | BPA_SBNum | 8 | U | 1 | 0 | 255 | Num | Slave BMS 번호 |
| 1 | BPA_ISOSPIErrCount[Num] | 8 | U | 1 | 0 | 255 | Cnt | ISOSPI 에러 카운트 |
| 2 | CT_CanCount | 8 | U | 1 | 0 | 255 | Cnt | CT센서 CAN 수신 카운트 |
| 3 | VCU_CanCount | 8 | U | 1 | 0 | 255 | Cnt | VCU CAN 수신 카운트 |
| 4 | CHA_CanCount | 8 | U | 1 | 0 | 255 | Cnt | 충전기 CAN 수신 카운트 |
| 5 | HMI_CanCount | 8 | U | 1 | 0 | 255 | Cnt | HMI CAN 수신 카운트 |
| 6-7 | Reserved | 16 | - | - | - | - | - | 예약 |

> **DBC 250317과 차이**: 동일 CAN ID(0x61B)이지만 메시지 내용이 완전 교체됨.

### 5.20 VCU_RE (0x600, 1536) — VCU → BPA, 100ms

> VCU에서 BPA로 전달하는 리셋/제어 명령

| Byte | Bit | Signal | 비트폭 | 설명 |
|------|-----|--------|-------|------|
| 0 | 0-7 | Reserved0 | 8 | 예약 |
| 1 | 8 | Ifcu_BSAReset | 1 | BAT 보호 상태 리셋 (0=No Action, 1=Reset) |
| 1 | 9 | Ifcu_BSAHldShutdown | 1 | BMS 셧다운 보류 |
| 1 | 10-15 | Reserved | 6 | 예약 |
| 2-7 | - | Reserved | 48 | 예약 |

> **DBC 250317과 차이**: Ifcu_BSAHldShutdown(bit9) 신규 추가.

### 5.21 SNS2ADT1 (0x341, 833) — SNS → ADT, 50ms

> IMU 가속도

| Byte | Signal | 비트폭 | Type | Factor | 단위 | 설명 |
|------|--------|-------|------|--------|------|------|
| 0-1 | IMU_Ac_X | 16 | S | 0.001 | g | X축 가속도 |
| 2-3 | IMU_Ac_Y | 16 | S | 0.001 | g | Y축 가속도 |
| 4-5 | IMU_Ac_Z | 16 | S | 0.001 | g | Z축 가속도 |
| 6-7 | Reserved16 | 16 | S | - | - | 예약 |

> Excel/DBC 일치.

### 5.22 SNS2ADT2 (0x342, 834) — SNS → ADT, 50ms

> IMU 각속도

| Byte | Signal | 비트폭 | Type | Factor | 단위 | 설명 |
|------|--------|-------|------|--------|------|------|
| 0-1 | IMU_Av_X | 16 | S | 0.01 | deg/s | X축 각속도 |
| 2-3 | IMU_Av_Y | 16 | S | 0.01 | deg/s | Y축 각속도 |
| 4-5 | IMU_Av_Z | 16 | S | 0.01 | deg/s | Z축 각속도 |
| 6-7 | Reserved16 | 16 | S | - | - | 예약 |

> Excel/DBC 일치.

### 5.23 SNS2ADT3 (0x343, 835) — SNS → ADT, 50ms

> IMU 자세각

| Byte | Signal | 비트폭 | Type | Factor | 단위 | 설명 |
|------|--------|-------|------|--------|------|------|
| 0-1 | IMU_Roll_deg | 16 | S | 0.01 | deg | 롤각 |
| 2-3 | IMU_Pitch_deg | 16 | S | 0.01 | deg | 피치각 |
| 4-5 | IMU_Yaw_deg | 16 | S | 0.01 | deg | 요각 |
| 6-7 | Reserved16 | 16 | S | - | - | 예약 |

> **DBC 250317과 차이**: Excel에서 Roll(byte0-1), Pitch(byte2-3) 순서. DBC에서는 Pitch(byte0-1), Roll(byte2-3) 순서.

### 5.24 SNS2ADT4 (0x344, 836) — SNS → ADT, 100ms

> GPS 상태 및 속도/방위

| Byte | Bit | Signal | 비트폭 | Type | Factor | 단위 | 설명 |
|------|-----|--------|-------|------|--------|------|------|
| 0-1 | 0-15 | GPS_Heading_Motion_deg | 16 | U | 0.01 | deg | GPS 이동 방위각 |
| 2-3 | 16-31 | GPS_Ground_Speed | 16 | S | 0.01 | km/h | GPS 지면 속도 |
| 4 | 32-33 | GPS_St_FixType | 2 | U | 1 | - | Fix 유형 (0=No, 2=2D, 3=3D, 5=TimeOnly) |
| 4 | 34 | GPS_St_FixedOK | 1 | U | 1 | - | Fix 유효 (0=No, 1=Valid) |
| 4 | 35 | GPS_St_DiffSoln | 1 | U | 1 | - | 보정 적용 여부 |
| 4 | 36-37 | GPS_St_CarrSoln | 2 | U | 1 | - | 반송파 해 (0=No, 1=Float, 2=Fixed) |
| 4 | 38-39 | Reserved2 | 2 | - | - | - | 예약 |
| 5 | 40-47 | GPS_St_numSV | 8 | U | 1 | - | 위성 수 |
| 6-7 | 48-63 | SNS_Heading_deg | 16 | U | 0.01 | deg | 보정 방위각 |

> Excel/DBC 일치.

### 5.25 SNS2ADT5 (0x345, 837) — SNS → ADT, 100ms

> GPS 원시 좌표

| Byte | Signal | 비트폭 | Type | Factor | 단위 | 설명 |
|------|--------|-------|------|--------|------|------|
| 0-3 | GPS_Latitude | 32 | S | 1E-7 | deg | 위도 |
| 4-7 | GPS_Longitude | 32 | S | 1E-7 | deg | 경도 |

> Excel/DBC 일치.

### 5.26 SNS2ADT6 (0x346, 838) — SNS → ADT, 100ms

> 보정 좌표

| Byte | Signal | 비트폭 | Type | Factor | 단위 | 설명 |
|------|--------|-------|------|--------|------|------|
| 0-3 | SNS_Latitude | 32 | S | 1E-7 | deg | 보정 위도 |
| 4-7 | SNS_Longitude | 32 | S | 1E-7 | deg | 보정 경도 |

> Excel/DBC 일치.

### 5.27 SNS2ADT7 (0x347, 839) — SNS → ADT, 50ms [신규]

> IMU 지자기센서 (Magnetometer)

| Byte | Signal | 비트폭 | Type | Factor | 단위 | 설명 |
|------|--------|-------|------|--------|------|------|
| 0-1 | IMU_Mag_X | 16 | S | 0.01 | uT | X축 자기장 |
| 2-3 | IMU_Mag_Y | 16 | S | 0.01 | uT | Y축 자기장 |
| 4-5 | IMU_Mag_Z | 16 | S | 0.01 | uT | Z축 자기장 |
| 6-7 | Reserved16 | 16 | S | - | - | 예약 |

> **DBC 250317과 차이**: Excel 260213에서 신규 추가된 메시지. DBC에 존재하지 않음.

### 5.28 CT_Sensor (0x32C, 812) — CT → VCU

> 외부 전류센서

| Byte | Signal | 비트폭 | Type | Factor | Min | Max | 단위 | 설명 |
|------|--------|-------|------|--------|-----|-----|------|------|
| 0-1 | CT_Sensor | 16 | U | 0.1 | -500 | 500 | A | 전류 측정값 |

> Excel/DBC 일치.

### 5.29 HMI (0x700, 1792) — HMI → VCU

> 외부 모니터링 요청

| Byte | Bit | Signal | 비트폭 | 설명 |
|------|-----|--------|-------|------|
| 0 | 0 | HMI1_COM_EN | 1 | BMS 통신 활성화 |
| 0 | 1 | HMI1_Cell_Volt_Request | 1 | 셀 전압 데이터 요청 |
| 0 | 2 | HMI1_Cell_Temps_Request | 1 | 셀 온도 데이터 요청 |
| 4-5 | 32-47 | HMI1_Cell_AGVT | 16 | 셀 평균 온도 (0.1 deg C) |
| 6-7 | 48-63 | HM1I_Cell_MinV | 16 | 셀 최소 전압 (0.001V) / 정규화 요청 |

> Excel/DBC 일치.

### 5.30 VCU2MD2K1 (0x064, 100) — VCU → 좌모터, 10ms [신규]

> MDROBOT MD2K PID 명령 (좌측)

| Byte | Signal | 비트폭 | Type | Factor | 단위 | 설명 |
|------|--------|-------|------|--------|------|------|
| 0 | PID | 8 | U | 1 | - | Parameter IDentification number |
| 1-2 | D1D2 | 16 | S | 1 | - | PID에 따른 16bit Signed 데이터 |
| 3 | D3 | 8 | U | 1 | - | 확장 데이터 3 |
| 4 | D4 | 8 | U | 1 | - | 확장 데이터 4 |
| 5 | D5 | 8 | U | 1 | - | 확장 데이터 5 |
| 6 | D6 | 8 | U | 1 | - | 확장 데이터 6 |
| 7 | D7 | 8 | U | 1 | - | 확장 데이터 7 |

> **DBC 250317과 차이**: DMKE CANopen RPDO1(0x22B, DLC=2)에서 MD2K PID CAN(0x064, DLC=8)으로 전면 교체.

### 5.31 VCU2MD2K2 (0x065, 101) — VCU → 우모터, 10ms [신규]

> MDROBOT MD2K PID 명령 (우측). 시그널 구조는 VCU2MD2K1과 동일.

> **DBC 250317과 차이**: DMKE CANopen RPDO1(0x22A, DLC=2)에서 MD2K PID CAN(0x065, DLC=8)으로 전면 교체.

### 5.32 MD2K2VCU1 (0x764, 1892) — 좌모터 → VCU, 10ms [신규]

> MD2K 좌측 모터 상태 피드백

| Byte | Signal | 비트폭 | Type | Factor | 단위 | 설명 |
|------|--------|-------|------|--------|------|------|
| 0 | PID | 8 | U | 1 | - | Return Type (PID) |
| 1 | Fault | 8 | U | 1 | - | Fault 비트필드 (하단 참조) |
| 2-3 | RPM | 16 | S | 1 | RPM | 모터 회전수 |
| 4-5 | DC_Current | 16 | U | 0.1 | A | DC 전류 |
| 6-7 | Duty | 16 | S | 0.1 | - | Duty (-1024~1024) |

**Fault 비트필드 (Byte 1):**

| Bit | Signal | 설명 |
|-----|--------|------|
| 0 | stall | 모터 정지 |
| 1 | inv_vel | 역방향 감지 |
| 2 | hall_fail | 홀센서 고장 |
| 3 | over_load | 과부하 |
| 4 | over_temp | 과온도 |
| 5 | over_volt | 과전압 |
| 6 | ctrl_fail | 제어 실패 |
| 7 | alarm | 알람 |

> **DBC 250317과 차이**: DMKE CANopen TPDO1~4(4개 메시지)가 MD2K 단일 피드백 메시지로 통합.

### 5.33 MD2K2VCU2 (0x765, 1893) — 우모터 → VCU, 10ms [신규]

> MD2K 우측 모터 상태 피드백. 시그널 구조는 MD2K2VCU1과 동일.

### 5.34 state_debug_M1 (0x101, 257) — VCU → Ext, 10ms [신규]

> 좌측 모터 디버그

| Byte | Signal | 비트폭 | Type | Factor | 단위 | 설명 |
|------|--------|-------|------|--------|------|------|
| 0 | Fault | 8 | U | 1 | - | MD2K Fault 비트필드 (5.32 참조) |
| 1 | CL_F | 8 | U | 1 | - | 전류 제한 플래그 |
| 2-3 | Spd_Reqramp | 16 | S | 1 | RPM | RPM 램프 목표 |
| 4-5 | Spd_Feed | 16 | S | 1 | RPM | RPM 피드백 |
| 6 | vVCU_FaultCode1 | 8 | U | 1 | - | VCU 고장코드 1 |
| 7 | vVCU_FaultCode2 | 8 | U | 1 | - | VCU 고장코드 2 |

> **DBC 250317과 차이**: SS500_Debug_Rx(0x100)/SS500_Debug_Tx(0x110)를 대체. CAN ID 0x101로 변경.

### 5.35 state_debug_M2 (0x102, 258) — VCU → Ext, 10ms [신규]

> 우측 모터 디버그. 시그널 구조는 state_debug_M1과 동일.

### 5.36 state_debug_RMT (0x104, 260) — VCU → Ext, 10ms [신규]

> 리모컨 디버그

| Byte | Signal | 비트폭 | Type | Factor | 단위 | 설명 |
|------|--------|-------|------|--------|------|------|
| 0-7 | (TBD) | 64 | - | - | - | 리모컨 관련 디버그 데이터 |

> **DBC 250317과 차이**: Excel 260213에서 신규 추가. 상세 시그널 정의는 추후 확정.

### 5.37 VCU2FAN1 (0x004E8401, Ext) — VCU → FAN

> 4채널 스로틀 명령

| Bit | Signal | 비트폭 | Type | 범위 | 설명 |
|-----|--------|-------|------|------|------|
| 0-13 | Throttle_1 | 14 | S | -8192~8191 | 채널 1 스로틀 |
| 14-27 | Throttle_2 | 14 | S | -8192~8191 | 채널 2 스로틀 |
| 28-41 | Throttle_3 | 14 | S | -8192~8191 | 채널 3 스로틀 |
| 42-55 | Throttle_4 | 14 | S | -8192~8191 | 채널 4 스로틀 |
| 56-63 | Tail_Byte | 8 | U | - | 끝 바이트 |

> **DBC 250317과 차이**: CAN ID 0x80640401→**0x004E8401**. 시그널 구조 동일, 시그널명에서 X6_ prefix 제거.

### 5.38 FAN2VCU1 (0x1F4E5201, Ext) — FAN → VCU

> eRPM, 쓰로틀, 상태 플래그

| Bit | Signal | 비트폭 | Type | Factor | 설명 |
|-----|--------|-------|------|--------|------|
| 0-15 | eRPM | 16 | U | 1 | 전기적 RPM |
| 16-31 | Input_Throttle_Value | 16 | U | 0.000122 | 입력 쓰로틀 |
| 32-47 | Status_Flags | 16 | U | 1 | 상태 플래그 (비트필드) |
| 48-55 | Tail_Byte | 8 | U | - | 끝 바이트 |

**Status_Flags 비트필드 (bit 32~47):**

| Bit | Signal | 설명 |
|-----|--------|------|
| 32 | High_Voltage_F | 고전압 고장 |
| 33 | Low_Voltage_F | 저전압 고장 |
| 34 | OC_F | 과전류 고장 |
| 35 | Positioning_Complete_F | 위치결정 완료 |
| 36 | Throttle_Loss_F | 쓰로틀 손실 |
| 37 | Throttle_NO_Zero_F | 쓰로틀 미영점 |
| 38 | FET_OTP_F | FET 과열 보호 |
| 39 | CAP_OTP_F | 캐패시터 과열 |
| 40 | Motor_Block_F | 모터 잠김 |
| 41 | Upper_FET_Open_F | 상부 FET 개방 |
| 42 | FET_Short_F | FET 단락 |
| 43 | Lower_FET_Open_F | 하부 FET 개방 |
| 44 | OPamp_Exception_F | OP앰프 이상 |
| 45 | Communication_F | 통신 고장 |
| 46 | Direction_F | 방향 고장 |
| 47 | Throttle_Signal_F | 쓰로틀 신호 이상 |

> **DBC 250317과 차이**: CAN ID 0xA0640801→**0x1F4E5201**. 시그널명에서 X6_ prefix 제거.

### 5.39 FAN2VCU2 (0x1F4E5301, Ext) — FAN → VCU

> 전기적 상태

| Byte | Signal | 비트폭 | Type | Factor | 단위 | 설명 |
|------|--------|-------|------|--------|------|------|
| 0-1 | Voltage | 16 | U | 0.1 | V | 입력 전압 |
| 2-3 | Current | 16 | S | 0.1 | A | 입력 전류 |
| 4 | Temp | 8 | U | 1 | deg C | 온도 |
| 5 | Tail_Byte | 8 | U | - | - | 끝 바이트 |

> **DBC 250317과 차이**: CAN ID 0xA0640A01→**0x1F4E5301**. 시그널명에서 X6_/X6_Input_ prefix 제거.

### 5.40 FAN2VCU3 (0x1F4E5401, Ext) — FAN → VCU

> 상세 온도

| Byte | Signal | 비트폭 | Type | Factor | 단위 | 설명 |
|------|--------|-------|------|--------|------|------|
| 0 | MOS_Temp | 8 | U | 1 | deg C | MOS FET 온도 |
| 1 | Capacitor_Temp | 8 | U | 1 | deg C | 캐패시터 온도 |
| 2 | Motor_Temp | 8 | U | 1 | deg C | 모터 온도 |
| 7 | Tail_Byte | 8 | U | - | - | 끝 바이트 |

> **DBC 250317과 차이**: CAN ID 0xA0640C01→**0x1F4E5401**. 시그널명에서 X6_ prefix 제거.

---

## 6. 고장코드 / 경고코드

### 6.1 Fault Code (eFaultCode)

> 출처: FaultCode.h + 06_SS기_개발사양.md 교차검증
> VCU2LCD3 메시지의 CF_FaultCode1~8에 전달됨 (최대 8개 동시 활성)

| 값 | 이름 | 카테고리 | 설명 | MR Off | 구동금지 | 작업금지 | 자율주행금지 |
|----|------|---------|------|--------|---------|---------|-----------|
| 0 | FC_NONE | - | 고장 없음 | - | - | - | - |
| **1** | FC_BPA_Prtct_Dischager_OC | BMS | 배터리 방전 과전류 | - | - | - | - |
| **2** | FC_BPA_Prtct_Chager_OC | BMS | 배터리 충전 과전류 | - | - | - | - |
| **3** | FC_BPA_Prtct_SOC_OV | BMS | 배터리 SOC 과전압 | - | - | - | - |
| **4** | FC_BPA_Prtct_SOC_Un | BMS | 배터리 SOC 저전압 | - | - | O | O |
| **5** | FC_BPA_Prtct_OV | BMS | 배터리 과전압 | - | - | - | - |
| **6** | FC_BPA_Prtct_UV | BMS | 배터리 저전압 | - | - | O | O |
| **7** | FC_BPA_Prtct_Cell_OV | BMS | 배터리 셀 과전압 | - | - | - | - |
| **8** | FC_BPA_Prtct_Cell_UV | BMS | 배터리 셀 저전압 | - | - | O | O |
| **9** | FC_BPA_Prtct_Cell_UnbalV | BMS | 배터리 셀전압 불안정 | - | - | - | - |
| **10** | FC_BPA_Prtct_DisCharger_Cell_OT | BMS | 배터리 셀 방전 과온도 | - | - | O | O |
| **11** | FC_BPA_Prtct_Charger_Cell_OT | BMS | 배터리 셀 충전 과온도 | - | - | - | - |
| **12** | FC_BPA_Prtct_DisCharger_Cell_UT | BMS | 배터리 셀 방전 저온도 | - | - | O | O |
| **13** | FC_BPA_Prtct_Charger_Cell_UT | BMS | 배터리 셀 충전 저온도 | - | - | - | - |
| **14** | FC_BPA_Prtct_Cell_UnbalT | BMS | 배터리 셀 온도 불안정 | - | - | - | - |
| **15** | FC_BPA_Prtct_Discharger_UnBPAWR | BMS | 배터리 방전 불균형 | - | - | - | - |
| **16** | FC_BPA_Prtct_Charger_UnBPAWR | BMS | 배터리 충전 불균형 | - | - | - | - |
| **17** | FC_BPA_Prtct_Rly_Err | BMS | 배터리 릴레이 고장 | **O** | **O** | **O** | **O** |
| **18** | FC_BPA_Prtct_ISOSPI_Err | BMS | 배터리 SPI통신 에러 | - | - | O | O |
| **19** | FC_BPA_Prtct_CAN_COM_Err | BMS | 배터리 CAN통신 에러 | - | - | O | O |
| **20** | FC_BPA_Prtct_CT_COM_Err | BMS | 배터리 전류센서 에러 | - | - | O | O |
| **21** | FC_BPA_Prtct_ISORESIS_Err | BMS | 배터리 절연파괴 에러 | - | - | O | O |
| 22-24 | FC_BPA_22~24 | BMS | (예약) | - | - | - | - |
| **25** | FC_ADTSysFault_FrontCamera | ADT | 전면 카메라 고장 | - | - | O | - |
| **26** | FC_ADTSysFault_LeftCamera | ADT | 좌측 카메라 고장 | - | - | - | - |
| **27** | FC_ADTSysFault_RightCamera | ADT | 우측 카메라 고장 | - | - | - | - |
| **28** | FC_ADTSysFault_GPS_NotConnected | ADT | GPS 감지 불가 | - | O | O | - |
| **29** | FC_ADTSysFault_GPS_Unstable | ADT | GPS 신호 불량 | - | O | O | - |
| **30** | FC_ADTSysFault_Network_NotConnected | ADT | 네트워크 연결 없음 | - | O | O | - |
| **31** | FC_ADTSysFault_Network_Unstable | ADT | 네트워크 불안정 | - | O | O | - |
| **32** | FC_ADTSysFault_General | ADT | 자율주행 일반 고장 | - | O | O | - |
| **33** | FC_ADTSysFault_Control1 | ADT | 마음AI 제어 고장(#9) | - | - | - | - |
| **34** | FC_ADTSysFault_Control2 | ADT | 마음AI 제어 고장(#10) | - | - | - | - |
| 35-40 | FC_ADTSysFault_35~40 | ADT | (예약) | - | - | - | - |
| **41** | FC_CanBusOff | VCU | CAN Bus Off | **O** | **O** | **O** | - |
| **42** | FC_ADT_CanMsgTimeout | VCU | ADT 통신 Timeout | - | O | - | O |
| **43** | FC_BMS_CanMsgTimeout | VCU | BMS 통신 Timeout | - | - | - | - |
| **44** | FC_LCD_CanMsgTimeout | VCU | LCD 통신 Timeout | - | - | - | - |
| **45** | FC_LeftMotor_CanMsgTimeout | VCU | 좌측 구동모터 Timeout | **O** | **O** | **O** | - |
| **46** | FC_RightMotor_CanMsgTimeout | VCU | 우측 구동모터 Timeout | **O** | **O** | **O** | - |
| **47** | FC_FanMotor_CanMsgTimeout | VCU | 팬모터 Timeout | - | - | O | - |
| **48** | FC_SNS_CanMsgTimeout | VCU | GPS센서 Timeout | - | - | - | - |
| 49 | FC_VCU_49 | VCU | (예약) | - | - | - | - |
| **50** | FC_MainRLY_Err | VCU | 메인릴레이 고장 | **O** | **O** | **O** | - |
| **51** | FC_HeadLamp_Err | VCU | 헤드램프 고장 | - | - | - | - |
| **52** | FC_CoreTemp_High | VCU | 코어 온도 높음 | - | - | - | - |
| **53** | FC_PC_power_Err | VCU | PC 부팅 불가 | - | - | - | - |
| **54** | FC_PC_StatPin_Err | VCU | PC 진단핀 단선 | - | - | - | - |
| 55-60 | FC_VCU_55~60 | VCU | (예약) | - | - | - | - |
| **61** | FC_LeftMotor_61 | Motor | 좌측 모터 고장 | **O** | **O** | **O** | - |
| **62** | FC_RightMotor_62 | Motor | 우측 모터 고장 | **O** | **O** | **O** | - |
| 63-70 | FC_fault_63~70 | - | (예약) | - | - | - | - |
| **71** | FC_FanMotor_71 | Motor | 팬모터 고장 | - | - | O | - |
| **72** | FC_PumpMotor_72 | Motor | 펌프모터 고장 | - | - | O | - |
| 73-79 | FC_fault_73~79 | - | (예약) | - | - | - | - |
| **80** | FC_fault_80 | Test | S_BUS RxFrame Timeout 테스트 | - | - | - | - |

### 6.2 Warning Code (eWarningCode)

> CF_SysWarnMsg(VCU2LCD1 bit 8-12)에 전달. 값 1~15는 ADT 경고, 16~31은 VCU 경고.

| 값 | 이름 | 카테고리 | 설명 | 주요 액션 |
|----|------|---------|------|----------|
| 0 | WC_NONE | - | 경고 없음 | - |
| **1** | WC_ImpossibleToAvoidObstacleDetected | ADT | 회피 불가능 장애물 감지 | 펌프OFF, 팬OFF, 교반기OFF |
| **2** | WC_TemporaryObstacleDetected | ADT | 일시적 장애물 감지 (3m 이내) | 펌프OFF, 팬OFF, 교반기OFF |
| **3** | WC_TemporaryGPS | ADT | 일시적 GPS 신호 불량 | - |
| **4** | WC_OutofWorkBoundary | ADT | 자율주행 영역 이탈 | 펌프OFF, 팬OFF, 교반기OFF |
| **5** | WC_ObstacleDetected | ADT | 장애물 감지 (상시) | 구동금지 |
| 6-15 | WC_ADT_6~15 | ADT | (예약) | - |
| **16** | WC_ReturnToStartPoint_SOC | VCU | 배터리 부족 복귀 | 펌프OFF, 팬OFF, 교반기OFF |
| **17** | WC_ReturnToStartPoint_TANK | VCU | 약제통 부족 복귀 | 펌프OFF, 팬OFF, 교반기OFF |
| **18** | WC_SOC | VCU | 배터리 잔량 부족 | 충전 권고 |
| **19** | WC_TANK | VCU | 약제통 잔량 부족 | - |
| **20** | WC_ObstacleDetectedByUltrasonicSensor | VCU | 범퍼(초음파) 장애물 감지 | 구동금지 |
| **21** | WC_REMOTE_CONTROL_BATTERY_LOW | VCU | 리모컨 배터리 부족 | 구동금지 |
| **22** | WC_BATTERY_CHARGER_MODE | VCU | 배터리 충전 모드 | - |
| **23** | WC_AUTO_RCUSER_FAR | VCU | 리모컨 조작자 원거리 | - |
| **24** | WC_AUTOMODE_RCSPEED_STOP | VCU | 리모컨 조향 → 자율주행 정지 | 작업기 정지 |

### 6.3 CF_ADTDrvCnd — 자율주행 주행 조건

> ADT2VCU1의 CF_ADTDrvCnd 시그널 값 정의 (eADTDrvCnd, Dm.h)

| 값 | 이름 | 설명 | VCU 액션 |
|----|------|------|----------|
| 0 | None | 기본 | - |
| 1 | WhenBothSidesAreLargeFruitTrees | 양쪽 대형 과수목 | Sol 1,4번 제어 |
| 2 | WhenThereAreNoFruitTreesOnTheLeftLine | 좌측 과수목 없음 | Sol 1,2번 제어 |
| 3 | WhenThereAreNoFruitTreesOnTheRightLine | 우측 과수목 없음 | Sol 3,4번 제어 |
| 4 | WhenMovingLineHasAlreadyBeenWorked | 기작업 라인 이동 | 펌프OFF, 팬OFF, 교반기OFF |
| 5 | WhenTurningLeftAtEndOfLine | 라인 끝 좌회전 | 펌프OFF, 팬OFF, 교반기OFF |
| 6 | WhenTurningRightAtEndOfLine | 라인 끝 우회전 | 펌프OFF, 팬OFF, 교반기OFF |

---

## 7. UART 프로토콜

### 7.1 SBUS 리모컨 (UART4)

> Futaba S.Bus 프로토콜. VCU와 SBUS 수신기 간 UART 통신.

| 항목 | 사양 |
|------|------|
| 물리층 | UART, 100kbps, 8E2 (Inverted) |
| 프레임 크기 | 25 bytes |
| 채널 수 | 16ch (각 11bit, 0~2047) |
| 유효 범위 | 272 ~ 1712 (Raw 값) |
| 데드존 | 891 ~ 1093 (중립) |
| 타임아웃 | DISCONNECTED_TH = 200 (FreeRTOS ticks) |

**채널 매핑:**

| CH# | 이름 | 조작부 | 기능 | 값 범위 |
|-----|------|--------|------|---------|
| 0 | DIR_CH | 우측 조이스틱 좌우 | 방향 제어 | 272~1712 |
| 2 | VH_SPD_CH | 좌측 조이스틱 상하 | 속도 제어 | +/-3.0 kph |
| 4 | LAMP_CH | 좌측 3단 SW | 헤드램프 ON/OFF | 임계값 기반 |
| 5 | AUTOMODE_CH | 우측 3단 SW | 자율주행 모드 | 임계값 기반 |
| 6 | EMGY_CH | 가운데 3단 SW | 비상정지 | 임계값 기반 |
| 7 | MIX_CH | A 버튼 | 교반기 ON/OFF | 토글 |
| 8 | HLD_VH_CH | B 버튼 | 속도 고정(크루즈) | 토글 |
| 9 | LD_SOL_CH | C 버튼 | 좌측 솔레노이드 | 4단계 순환 |
| 10 | RD_SOL_CH | D 버튼 | 우측 솔레노이드 | 4단계 순환 |
| 11 | PMP_CH | 좌측 센터링 다이얼 | 펌프 단계 | 3단계 |
| 12 | FAN_CH | 우측 센터링 다이얼 | 팬 단계 | 3단계 |
| 13-15 | RESERVED | - | 예약 | - |

### 7.2 UART LCD 프로토콜 (UART5)

> VCU <-> RC-LCD 보드 간 UART 통신. CAN 메시지를 UART 프레임으로 변환하여 LCD에 전달.

**Tx 프레임 (VCU -> LCD, 32 bytes payload):**

| 항목 | 사양 |
|------|------|
| Header | 0xAA 0x44 0xB5 0x55 (4 bytes) |
| Length | 2 bytes (Little Endian) |
| Payload | 32 bytes (VCU2LCD1 + VCU2LCD2 + VCU2LCD3 + BPA_Calc + GPS_Info) |
| CRC32 | 4 bytes (0x990951BA) |

**Rx 프레임 (LCD -> VCU, 5 bytes payload):**

| 항목 | 사양 |
|------|------|
| Header | 0x55 0x44 0xB5 0xAA (4 bytes, 역순) |
| Length | 2 bytes |
| Payload | 5 bytes |
| CRC32 | 4 bytes (0xA6BC5767) |

**Rx Payload 구조:**

| Byte | Signal | 설명 |
|------|--------|------|
| 0-1 | CF_ADTStrtLocnRtrnDist | 복귀 최단거리 (uint16_t) |
| 2 | CF_RMTBattLvl | 리모컨 배터리 레벨 |
| 3 | CF_RMTDrvCnd | 리모컨 주행 조건 |
| 4[0] | CF_RMTUserAway | 리모컨 사용자 원거리 여부 |
| 4[1:7] | Reserved | 예약 |

---

## 8. 타이밍 / Timeout

### 8.1 송신 주기

| 메시지 | 주기 | 소스 |
|--------|------|------|
| ADT2VCU1 | 50ms | Adt.h |
| VCU2ADT1, VCU2ADT2 | 50ms | Adt.h |
| LCD2VCU1 | 50ms | Lcd.h |
| VCU2LCD1/2/3 | 100ms | Lcd.h |
| BPA_Calc, BPA_State | 10ms | Bms.h |
| BPA_ProtectState~BPA_COM_State | 100ms | Bms.h |
| VCU_RE | 100ms | Bms.h |
| SNS2ADT1~3 | 50ms | Sns.h |
| SNS2ADT4~7 | 100ms | Sns.h |
| VCU2MD2K1/2 | 10ms | FW 10ms task |
| MD2K2VCU1/2 | 10ms | MD2K 모터드라이버 |
| state_debug_M1/M2/RMT | 10ms | FW 10ms task |

### 8.2 Timeout 임계값

> 모든 Timeout은 **2000ms (2초)** 기준. 팬모터만 5000ms.

| 대상 | Timeout | 계산식 | 소스 |
|------|---------|--------|------|
| ADT2VCU1 | 2000ms | 2000/50 = 40 카운트 @ 50ms | Adt.h |
| LCD2VCU1 | 2000ms | 2000/50 = 40 카운트 @ 50ms | Lcd.h |
| BPA (10ms msgs) | 2000ms | 2000/10 = 200 카운트 @ 10ms | Bms.h |
| BPA (100ms msgs) | 2000ms | 2000/100 = 20 카운트 @ 100ms | Bms.h |
| SNS (50ms msgs) | 2000ms | 2000/50 = 40 카운트 @ 50ms | Sns.h |
| SNS (100ms msgs) | 2000ms | 2000/100 = 20 카운트 @ 100ms | Sns.h |
| MD2K Motor | 2000ms | FreeRTOS ticks | CommMsg.h |
| FAN Motor | **5000ms** | Start delay 4000 + 마진 1000 | CommMsg.h |

### 8.3 Timeout 발생 시 고장코드 매핑

| Timeout 대상 | Fault Code | 값 | MR Off |
|-------------|------------|-----|--------|
| ADT | FC_ADT_CanMsgTimeout | 42 | - |
| BMS | FC_BMS_CanMsgTimeout | 43 | - |
| LCD | FC_LCD_CanMsgTimeout | 44 | - |
| MD2K_L | FC_LeftMotor_CanMsgTimeout | 45 | **O** |
| MD2K_R | FC_RightMotor_CanMsgTimeout | 46 | **O** |
| FAN | FC_FanMotor_CanMsgTimeout | 47 | - |
| SNS | FC_SNS_CanMsgTimeout | 48 | - |

### 8.4 CAN Bus Off 복구

| 항목 | 사양 |
|------|------|
| 감지 | HAL_CAN_ERROR_BOF (STM32 HAL) |
| 고장코드 | FC_CanBusOff (41) |
| 액션 | MR Off, 구동금지, 작업금지 |
| 복구 | 자동 복구 (128 x 11 recessive bits 후 Error Active 복귀) |

### 8.5 VCU FW 태스크 주기

| 태스크 | 주기 | 처리 내용 |
|--------|------|----------|
| 10ms | 10ms | BMS Rx/Tx, SBUS 처리, MD2K 명령/피드백, Debug Tx |
| 50ms | 50ms | ADT Rx/Tx, LCD Rx, SNS Rx/Tx |
| 100ms | 100ms | LCD Tx, BMS ProtectState, DM 판단, 고장 처리 |

---

## 9. CAN <-> Protobuf 매핑 (ADT 인터페이스)

### 9.1 개요

ADT PC(자율주행 컴퓨터)는 내부적으로 **Protobuf + MQTT** 기반 core-service를 사용하며,
CAN 메시지와 Protobuf 간 변환은 ADT PC 내부의 CAN Gateway 프로세스가 담당한다.

### 9.2 CAN -> Protobuf 방향 (VCU -> ADT -> core-service)

| CAN 메시지 | Protobuf 필드 (추정) | 설명 |
|-----------|---------------------|------|
| VCU2ADT1.CF_ADTAutoCtrlEnblSwt | vehicle.autonomous_enable | 자율주행 활성화 |
| VCU2ADT1.CF_ManEmgyStopSwt | vehicle.emergency_stop | 비상정지 상태 |
| VCU2ADT1.CF_LeftVehSpdAct | vehicle.left_speed_actual | 좌측 실제 속도 |
| VCU2ADT1.CF_RightVehSpdAct | vehicle.right_speed_actual | 우측 실제 속도 |
| VCU2ADT1.CF_stSysFault | vehicle.system_fault | 시스템 고장 |
| VCU2ADT1.CF_BumpSnsDet | vehicle.bumper_detected | 범퍼 감지 |
| VCU2ADT2.CF_UltraSonSns_Mid | vehicle.ultrasonic_mid | 초음파센서 거리 |

### 9.3 Protobuf -> CAN 방향 (core-service -> ADT -> VCU)

| Protobuf 필드 (추정) | CAN 메시지 | 설명 |
|---------------------|-----------|------|
| control.valid | ADT2VCU1.CF_ADTValid | 시스템 유효 |
| control.auto_enabled | ADT2VCU1.CF_ADTAutoCtrlEnblSt | 자율주행 동작 |
| control.left_speed_cmd | ADT2VCU1.CF_ADTLeftVehSpdCtrl | 좌측 속도 명령 |
| control.right_speed_cmd | ADT2VCU1.CF_ADTRightVehSpdCtrl | 우측 속도 명령 |
| control.drive_condition | ADT2VCU1.CF_ADTDrvCnd | 주행 조건 |
| control.return_status | ADT2VCU1.CF_ADTRtrnSt | 복귀 상태 |
| control.goback_status | ADT2VCU1.CF_ADTGoBackSt | 회귀 상태 |
| control.warning | ADT2VCU1.CF_ADTWarnMsg | 경고 메시지 |
| control.alive_count | ADT2VCU1.CF_ADTAliveCnt | Alive 카운터 |

> **주의**: Protobuf 필드명은 Notion 문서 기반 추정. 정확한 `.proto` 파일은 ADT 개발팀(마음AI) 확인 필요.

### 9.4 SNS 데이터 흐름

```
GPS/IMU 센서 -> [CAN2] -> SNS2ADT1~7 -> ADT PC -> Protobuf -> core-service
                                                    |
                                          자율주행 알고리즘
                                                    |
                                          ADT2VCU1 -> [CAN2] -> VCU
```

---

## 10. 부록

### 10.1 Excel 260213 vs DBC 250317 차이 요약

> 상세: `보고서/09_CAN_ICD_교차검증_Excel260213_vs_DBC.md` 참조

#### 삭제된 DBC 메시지 (26건)

| 카테고리 | 건수 | CAN ID 범위 | 사유 |
|---------|------|------------|------|
| DMKE CANopen (Standard) | 11건 | 0x0AA, 0x1AA~0x4AB, 0x22A~0x22B | MD2K PID로 전면 교체 |
| DMKE CANopen (Extended) | 6건 | 0x880003AA~0x880003BD | MD2K PID로 전면 교체 |
| FAN 설정 메시지 | 5건 | 0x90A0xxxx, 0x98A0xxxx | Excel 미포함 (초기 설정용) |
| FAN Legacy | 3건 | 0x80000A62~0x80000A64 | 신규 CAN ID로 교체 |
| Debug | 1건 | 0x110 | state_debug_M1/M2/RMT로 교체 |

#### 신규 추가 메시지 (13건)

| 메시지명 | CAN ID | 사유 |
|---------|--------|------|
| VCU2ADT2 | 0x332 | 초음파센서 데이터 |
| SNS2ADT7 | 0x347 | IMU 지자기센서 |
| VCU2MD2K1/2 | 0x064/0x065 | MD2K 모터 PID 명령 |
| MD2K2VCU1/2 | 0x764/0x765 | MD2K 모터 피드백 |
| state_debug_M1/M2/RMT | 0x101/0x102/0x104 | 모터/리모컨 디버그 |
| BPA_Calc2 | 0x618 | 배터리 용량/절연 (MD1_State 대체) |
| BPA_MD_Infro | 0x619 | 배터리 모듈 정보 (MD2_State 대체) |
| BPA_Cell_Infro | 0x61A | 셀별 정보 (MD3_State 대체) |
| BPA_COM_State | 0x61B | 통신 상태 (MD4_State 대체) |

#### CRITICAL 시그널 변경 (3건)

| 항목 | 내용 |
|------|------|
| 구동모터 프로토콜 | DMKE CANopen(17개 메시지) -> MDROBOT MD2K PID(4+3개 메시지) |
| BMS 0x618~0x61B | 모듈별 상태(6개) -> 통합 진단(4개), 0x61C/0x61D 삭제 |
| 속도 Factor | ADT2VCU1/VCU2ADT1 속도 시그널 Factor 0.1->**0.01**, 8bit->**16bit** |

### 10.2 제어권 상태 (typCrtlOwner)

> Dm.h에 정의된 VCU 제어권 Owner

| 값 | 이름 | 설명 |
|----|------|------|
| 0 | CTRL_NONE | 미사용 |
| 1 | CTRL_RC | 리모컨 제어 |
| 2 | CTRL_LCD | LCD 터치 제어 |
| 3 | CTRL_ADT | 자율주행 제어 |
| 4 | CTRL_TEST | 테스트 모드 |

### 10.3 CAN Filter Bank 설정

> AswCanMsg.h: `FILTER_BANK_MAX = 9`

VCU는 STM32F479의 CAN Filter Bank를 사용하여 수신 메시지를 필터링한다.
CAN1(FIFO0)과 CAN2(FIFO1)에 대해 최대 9개 필터 뱅크가 할당된다.

> **참고**: MD2K 모터(0x764/0x765) 수신을 위한 필터 뱅크 추가 필요. DMKE CANopen 필터(0x1AA~0x4AB)는 삭제 필요.

### 10.4 FW 송신 함수 매핑

| 함수명 | 주기 | 송신 메시지 |
|--------|------|------------|
| AswCanMsg_TxCanMsg_10ms() | 10ms | VCU2MD2K1/2, BMS 관련, state_debug_M1/M2/RMT |
| AswCanMsg_TxCanMsg_50ms() | 50ms | VCU2ADT1, VCU2ADT2, SNS 관련 |
| AswCanMsg_TxCanMsg_100ms() | 100ms | VCU2LCD1/2/3, VCU_RE |

### 10.5 DBC 파일 목록

| # | 파일명 | 위치 | 용도 | 메시지 수 |
|---|--------|------|------|----------|
| 1 | SSVCU_CANdb_250317_01.dbc | `4)CAN/` | 메인 VCU CAN DB (구버전) | 61 |
| 2 | X6_PLUS_CANdb.dbc | `4)CAN/FAN_MOTOR/` | 팬모터 ESC | ~10 |
| 3 | DMKE_CANdb.dbc | `4)CAN/TRACTION_MOTOR/` | 구동모터 (구버전, DMKE) | ~10 |
| 4 | DMD1KT.dbc | `2)부품/1)MDAS/MDAS_CAN/CANoe/` | MDAS 모터드라이버 | ~4 |

> **권고**: DBC 파일을 Excel 260213 기준으로 업데이트 필요. 특히 DMKE -> MD2K 교체, BMS 0x618~0x61B 재정의, 속도 Factor 변경 반영 필수.

### 10.6 용어집

| 용어 | 설명 |
|------|------|
| Alive Counter | 메시지 유효성 확인용 순환 카운터 (0->15->0) |
| Bus Off | CAN 에러 카운터 초과로 인한 통신 중단 상태 |
| DBC | Vector CANdb++ Database 파일 형식 |
| Factor | 물리값 = Raw x Factor + Offset |
| MD2K | MDROBOT MD2K PID 모터드라이버 프로토콜 |
| PID | Parameter IDentification number (MD2K 프로토콜) |
| RTK | Real-Time Kinematic (cm 단위 GPS 보정) |
| SOC | State of Charge (배터리 충전 상태, %) |
| SOH | State of Health (배터리 건강 상태, %) |

---

> **문서 끝**
> 본 문서는 V0.2로, 260213_01_SS_Proto_can_spec.xlsx 기준으로 업데이트됨.
> DBC 250317과의 차이점은 각 시그널 상세 섹션 및 부록 10.1에 기재.
> ADT Protobuf 스키마 확인 및 DBC 파일 갱신 후 V1.0으로 승격 예정.

