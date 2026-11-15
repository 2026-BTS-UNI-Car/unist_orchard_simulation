# 시스템 구조와 제어 흐름

- 작성: 2026-11-13 (작업 DOC-05). 도식 6개를 한곳에 모았다. 각 도식 아래에 출처 표시와 구현 상태(구현됨 / 설계만 / 계획)를 적었다.
- 바탕 문서: 설계 [`DESIGN_v1_KO.md`](steering/DESIGN_v1_KO.md)(2절 두 단계 SIM, 3절 차량 모델, 6절 제어기),
  개정 [`DESIGN_v2_KO.md`](steering/DESIGN_v2_KO.md)(V2-1 입력, V2-2 시작·정지, V2-3 네 층, V2-4 속도, V2-6 조향 하드웨어),
  운영자 규약 [`OPERATOR_API_KO.md`](steering/OPERATOR_API_KO.md), ESP 규약 [`ESP_PROTOCOL_V3_DRAFT_KO.md`](steering/ESP_PROTOCOL_V3_DRAFT_KO.md).
- 모든 수치는 자리값이다. 조향 장치의 실제 사양이 오면 바뀐다([`HARDWARE_KO.md`](HARDWARE_KO.md)).
- 2026-11-16 고침: 실차의 하위 보드가 ESP32-S3 하나(BTS7960 세 개, 펌웨어 ABS-HOLD-8-REBOUND-LIMITS)로 바뀌었다(1.1절). 2026-11-15까지의
  설계(ESP8266 + 펌웨어 V3)는 1.2절에 그대로 남긴다. SIM은 V3 직렬 규약을 그대로 쓰고 조향 법칙만 새 보드의 것으로 흉내 낼 수 있다
  (4절, [`ABS_HOLD8_SIM_KO.md`](steering/ABS_HOLD8_SIM_KO.md)).

## 1. 시스템 구성 (실차)

### 1.1 지금 (2026-11-16~)

```mermaid
flowchart LR
  subgraph SENS["센서"]
    LI["RPLIDAR S2<br/>10 Hz, 3240빔<br/>꼭대기 0.57 m · 앞차축 0.21 m 뒤"]
    IM["BNO085 IMU<br/>raw 33 Hz (I2C)"]
    GN["u-blox F9P GNSS<br/>약 1 Hz (USB)"]
    CA["UVC 카메라<br/>표시·기록만"]
  end
  subgraph JET["Jetson Orin Nano"]
    DRV["ROS 2 센서 드라이버<br/>/scan · /imu/data_raw · /gps/fix"]
    EXE["실행기 (20 Hz)<br/>추정기 L0~L3 + 열 추종 제어기<br/>운영자 HTTP"]
    BR["새 보드용 조향 브리지<br/>(대시보드 저장소, 작업 중)"]
    DB["대시보드 서버<br/>수동 조종 · 자율 시작/STOP"]
  end
  subgraph ESP["ESP32-S3 (펌웨어 ABS-HOLD-8-REBOUND-LIMITS)"]
    HOLD["조향 절대 각 유지<br/>PWM 100 펄스 50/80 ms → 300 ms 끔<br/>→ 조용한 가변저항 값으로 판정"]
    MAN["수동 모터 명령<br/>n F/R pwm · n S · STOP"]
  end
  subgraph ACT["BTS7960 드라이버 3개"]
    D1["1 조향<br/>일반 DC 모터"]
    D2["2 왼쪽 구동<br/>앞뒤 모터"]
    D3["3 오른쪽 구동<br/>앞뒤 모터"]
    POT["가변저항(벨트)<br/>GPIO6 12비트 ADC"]
  end
  OP["운영자 휴대폰·노트북"]
  LI --> DRV
  IM --> DRV
  GN --> DRV
  CA --> DRV
  DRV --> EXE
  OP <--> DB
  DB -->|"자율 시작 · STOP 중계"| EXE
  EXE -->|"목표 조향각 · 구동 duty"| BR
  DB -->|"수동 조종 · STOP"| BR
  BR <-->|"네이티브 USB (Jetson USB-C)<br/>글자 명령 ANGLE · STATUS · STOP · 'n F pwm'"| ESP
  HOLD --> D1
  MAN --> D1
  MAN --> D2
  MAN --> D3
  D1 -->|"감속·벨트·링크"| POT
  POT --> HOLD
```

[도식] 구현 상태: 보드·드라이버·펌웨어는 **실기에 있음**(사용자, 2026-11-16 Jetson에서 STATUS 응답만 확인, 움직임 없음). 새 보드용
Jetson 브리지는 **작업 중**(대시보드 저장소). 펌웨어 명령과 규칙은 [`HARDWARE_KO.md`](HARDWARE_KO.md) 2절·6.1절. 실기 펌웨어는 구동 명령이
끊겨도 서는 장치가 없어 호스트 쪽에서 막아야 한다. SIM은 이 보드를 V3 규약 위의 흉내로 돈다(2절, 4절).

### 1.2 2026-11-15까지의 설계 (ESP8266 + 펌웨어 V3, 기록)

```mermaid
flowchart LR
  subgraph SENS["센서"]
    LI["RPLIDAR S2<br/>2D LiDAR 10 Hz, 3240빔<br/>지면 약 0.6 m"]
    IM["BNO085 IMU<br/>raw 33 Hz (I2C)"]
    GN["u-blox F9P GNSS<br/>약 1 Hz (USB)"]
    CA["UVC 카메라<br/>표시·기록만"]
  end
  subgraph JET["Jetson Orin Nano"]
    DRV["ROS 2 센서 드라이버<br/>/scan · /imu/data_raw · /gps/fix"]
    EXE["실행기 (20 Hz)<br/>추정기 L0~L3 + 열 추종 제어기<br/>운영자 HTTP"]
    BR["조향 브리지 V3<br/>lease · STOP 확인 · 래치"]
    DB["대시보드 서버<br/>수동 조종 · 자율 시작/STOP"]
    REC["기록기<br/>bag · 조향 사이드카"]
  end
  subgraph ESP["ESP8266 (펌웨어 V3)"]
    DRAMP["구동: 램프 255/s<br/>TTL · watchdog"]
    SERVO["조향 위치 루프 500 Hz<br/>가변저항 → 조향 모터 PWM"]
  end
  subgraph ACT["구동기"]
    DMD["구동 드라이버 2개<br/>좌·우 DC 모터"]
    SMD["조향 모터 드라이버<br/>일반 DC 모터"]
    POT["가변저항<br/>벨트로 조향축과 연결"]
  end
  OP["운영자 휴대폰·노트북<br/>(차의 무선 AP)"]
  LI --> DRV
  IM --> DRV
  GN --> DRV
  CA --> DRV
  DRV --> EXE
  DRV --> REC
  DRV --> DB
  OP <--> DB
  DB -->|"자율 시작 · STOP 중계"| EXE
  DB -->|"수동 조종 · STOP"| BR
  EXE -->|"구동 duty · 목표 조향각"| BR
  BR <-->|"USB 직렬 115200<br/>V3 ASCII 줄, CRC-8"| ESP
  DRAMP --> DMD
  SERVO --> SMD
  SMD -->|"감속·벨트·링크"| POT
  POT -->|"A0 10비트 ADC"| SERVO
```

[도식] 구현 상태(2026-11-15 기준 기록): Jetson 쪽 소프트웨어(드라이버, 실행기, 브리지 V3, 기록기)와 ESP 펌웨어 V3는 **SIM에서 구현됨**
(펌웨어는 PC용 흉내 층과 ESP8266 컴파일·링크까지, 실기 쓰기 없음). 2026-11-16에 실차 보드가 1.1절로 바뀌어 이 ESP8266 구성은 실차에
쓰이지 않는다. 펌웨어 V3는 SIM의 직렬 규약·안전 껍데기로 남는다. 휠 엔코더는 없다.

## 2. SIM 스택 컨테이너 (G-SIM, `compose.ubts-steer.yaml`)

```mermaid
flowchart TB
  subgraph NS["sim 네트워크 이름공간 (ROS 도메인 43, GZ 파티션 ubts-steer)"]
    SIMC["sim<br/>Gazebo Harmonic + 조향 차량 SDF<br/>Embree LiDAR → 자르기 → S2 형식 /scan<br/>IMU 오차 모형 · 가상 GNSS · 카메라"]
    STEER["steer<br/>ESP 에뮬레이터(펌웨어 V3 네이티브, pty;<br/>조향 법칙 V3 또는 ABS-HOLD-8 흉내)<br/>+ 조향 브리지 V3"]
    RECC["recorder<br/>기록기 + 조향 사이드카"]
    DASH["dashboard<br/>대시보드 사본 + SIM 확장<br/>127.0.0.1:18090"]
    EXEC["executor (profile executor)<br/>실행기 + 열 추종 제어기<br/>운영자 HTTP 127.0.0.1:8097"]
    CHK["check (profile check)<br/>검사 대본 · 평가(참값은 여기만)"]
  end
  VOL[("볼륨: runtime · control · cache · hooks")]
  BROWSER["브라우저"] --> DASH
  SIMC -->|"/scan · /imu/data_raw · /gps/fix"| EXEC
  EXEC -->|"control.sock"| STEER
  DASH -->|"control.sock"| STEER
  DASH -->|"실행기 중계 + 토큰"| EXEC
  STEER -->|"바퀴각 목표 · 구동 duty (lockstep)"| SIMC
  SIMC --> RECC
  STEER --> RECC
  SIMC -.->|"참값 (평가 전용)"| CHK
  NS --- VOL
```

[도식] 구현 상태: **구현됨**. 이미지 `ubts-steer-sim:wf4c` 기준(대시보드 새 화면은 그다음 이미지부터, [`SIM_DASHBOARD_KO.md`](steering/SIM_DASHBOARD_KO.md)).
메모리 예산 6 GiB(sim 3.5, dashboard 1, steer 0.5, recorder 0.5, executor 0.5). 빠른 SIM(F-SIM, Docker 없음)은 같은 실행기·브리지
코어·펌웨어 로직을 한 프로세스의 SIM 시계로 묶는다(T1 장치, [`T1_RIG_KO.md`](steering/T1_RIG_KO.md)). 실행기와 추정기에는 참값,
`/odom`, `/joint_states`, `/sim/truth/*`가 들어가지 않는다(검사 A13).

## 3. 자료 흐름

```mermaid
flowchart LR
  S1["/scan<br/>LaserScan 3240빔 10 Hz"] --> EST
  S2["/imu/data_raw<br/>raw IMU 33 Hz"] --> EST
  S3["/gps/fix<br/>약 1 Hz"] --> L3G["L3 GNSS 층<br/>기록 · 지오펜스"]
  ST["MCU 상태 줄 20 Hz<br/>적용 duty · 가변저항 각 · 결함"] --> BRC
  BRC["브리지 응답<br/>(drive 응답 100 ms마다)"] --> EST
  EST["추정기 스냅샷 20 Hz<br/>횡오차 · 방향 오차 · 폭<br/>열 끝까지 거리 · 앞 여유 · 속도"] --> CTL["열 추종 제어기<br/>(throttle, steer_mrad)"]
  CTL --> GUARD["SteerOutputGuard<br/>한계 · 변화율 · GO 규칙"]
  GUARD -->|"op drive (lease 보유자)"| BRIDGE["조향 브리지 V3<br/>20 ms 루프"]
  BRIDGE -->|"C L R S ttl"| MCU["ESP8266 V3"]
  MCU --> ST
  CTL --> LOG["실행기 기록<br/>틱 · 이유 · 상한"]
  BRIDGE --> SIDE["조향 사이드카<br/>steering.jsonl 20 Hz"]
  L3G --> LOG
```

[도식] 구현 상태: **구현됨**(SIM). GNSS 층은 기본 꺼짐(`gnss_mode` `off`), 켜도 통로 추정과 명령에 들어가는 길이 없다
([`GNSS_LAYER_KO.md`](steering/GNSS_LAYER_KO.md)). `C` 줄의 L·R은 좌·우 구동 duty(−255..255), S는 보정 반범위의 천분율(−1000..1000)이다.

## 4. 제어 루프: 실행기 바깥 루프 + ESP 조향 위치 루프

```mermaid
flowchart LR
  subgraph OUT["바깥 루프 (Jetson 실행기, 20 Hz)"]
    E0["추정기 L0<br/>통로 band"] --> PP["주시점 추종<br/>L_d = base + k·v<br/>y_L = −k_d·횡오차 + k_ψ·L_d·sin(방향 오차)<br/>κ = 2·y_L / L_d²"]
    PP --> DEL["δ = atan(L·κ) + trim<br/>(trim: 느린 적분, 한정)"]
    DEL --> LIM["한계 · 저역 통과 · 변화율"]
    E0 --> GOV["속도 조절기<br/>v_ref = min(순항, 스캔, 지연, 정지 거리,<br/>곡률, 횡오차, 조향, 품질, 열 끝)"]
    GOV --> DUTY["duty = 표⁻¹(v_ref) + PI(v_ref − LiDAR 속도)"]
  end
  subgraph IN["안쪽 루프 (ESP 에뮬레이터: V3 서보 500 Hz 또는 ABS-HOLD-8 펄스)"]
    TGT["목표 S (천분율)<br/>→ 보정표 → 목표 위치<br/>(ABS-HOLD-8: ANGLE = −S × 20 / 1000)"] --> SV["V3: P 제어 + 데드밴드 · 최소/최대 PWM<br/>ABS-HOLD-8: PWM 100 펄스 50/80 ms,<br/>300 ms 끔, 허용 10 · 다시 시작 16 count"]
    SV --> MOT["조향 모터 드라이버 → DC 모터<br/>→ 감속 · 링크 → 앞바퀴"]
    MOT --> PT["가변저항 (벨트)<br/>A0 1 kHz, 중앙값 5"]
    PT --> SV
    RAMP["구동 램프 255/s<br/>C TTL 300 ms · watchdog 500 ms"]
  end
  LIM -->|"steer_mrad"| TGT
  DUTY -->|"L = R = duty"| RAMP
  MOT --> CAR["차량 운동<br/>(외력 · 미끄럼 · 쏠림)"]
  RAMP --> CAR
  CAR -->|"LiDAR · IMU"| E0
```

[도식] 구현 상태: 안쪽 루프는 프로필의 `steer_fw_mode`로 고른다. v0.x 프로필은 V3 서보, `front_steer_v1_spec_20261116`(실제 치수,
2026-11-16)은 실기 보드 펌웨어의 펄스 유지 흉내(`sim/ubts/steering/native/abs_hold8.cc`)다. 바깥 루프 **구현됨**
(`sim/ubts/closed_loop/steer_control.py`, 기본 제어기 `row_follow`). 요레이트 안쪽 루프 이득
`k_r`은 걸이만 있고 0이다. ESP 위치 루프는 펌웨어 V3로 **구현됨**(SIM에서 네이티브 빌드로 돔). 서보 이득·데드밴드·최소 PWM 등은
프로필 v0.1~v0.3의 자리값이다. 외란 아래 직진 유지(ST-11)에서 ESP 위치 루프만으로는 이어지는 힘을 다 되돌리지 못해, 직진 유지는
바깥 루프가 맡는다([`STEER_HOLD_RESULT_KO.md`](steering/STEER_HOLD_RESULT_KO.md)).

## 5. 추정기 층 (L0~L3)

```mermaid
flowchart TB
  SC["LiDAR 스캔"] --> L0
  IMU["raw IMU (gyro)"] --> L0
  STAT["MCU 상태 줄<br/>적용 duty · 조향각"] --> L1
  GPS["GNSS fix"] --> L3
  L0["L0 빠른 층 (주 입력)<br/>키프레임 ICP · 통로 band<br/>횡·방향 오차 · 폭 · 열 끝 · 여유"]
  L1["L1 구동기 사전값<br/>표(duty)·배율 → 속도<br/>v·tan δ / L → 요레이트"]
  L2["L2 국소 격자<br/>20 × 10 m, 0.10 m 셀, 2 스캔마다<br/>여러 스캔에 남는 셀 → 두 열 선"]
  L3["L3 GNSS 약한 층<br/>품질 · 큰 어긋남 · 열 범위"]
  L1 -->|"ICP 시드 · 짧은 예측 · 감시<br/>(가중 기본 0)"| L0
  L0 --> L2
  L2 -->|"낮은 이득 융합 (기본 0)"| L0
  L0 --> CTRL["조향 · 속도 · 열 끝 판정"]
  L2 --> SHOW["화면 · 교차 확인"]
  L3 --> LOGG["기록 · 지오펜스 (기본 꺼짐)"]
```

[도식] 구현 상태: L0 **구현됨**(주 입력). L1·L2·L3 **구현됨, 기본 꺼짐**: L1 가중 0(front_steer 는 2026-11-14부터 가중 0 감시가
기본으로 켜져 온라인 배율만 낸다, PW-10; [`ACTUATOR_PRIOR_KO.md`](steering/ACTUATOR_PRIOR_KO.md) 4절), L2 `local_grid_enabled` false·융합 이득 0
([`LOCAL_GRID_KO.md`](steering/LOCAL_GRID_KO.md)), L3 `gnss_mode` off([`GNSS_LAYER_KO.md`](steering/GNSS_LAYER_KO.md)).
조향 명령과 열 끝 판정은 L0에서만 나온다. 일반 GNSS 오차(10/10 기록 수평 정확도 p50 5.3~10.0 m)가 열 간격(3.6~4.1 m)보다 커서
GNSS는 횡 위치에 쓰지 않는다. PWM은 엔코더가 아니다.

## 6. 실행기 run 상태 (`run_state`, 한 번 눌러 시작)

```mermaid
stateDiagram-v2
  [*] --> idle
  idle --> ready : Start (새 run)
  ended --> ready : Start (새 run)
  ready --> armed : Arm (브리지 lease)
  armed --> running : go (입력 1 s 신선, 정지, 통로 잡힘)
  armed --> ready : 기다리는 중 lease 잃음 (다시 Arm)
  running --> paused : 회복 가능한 멈춤 (stale_lidar, stale_imu, 운영자 신호 1.0 s 끊김(R60), 속도 상한, 과속, 예측 한도)
  paused --> running : 새 go
  paused --> ended : 멈춤 3회 넘음 또는 end
  running --> ended : row_end, obstacle_ahead, corridor_lost 등
  ready --> latched : STOP 또는 결함
  armed --> latched : STOP 또는 결함
  running --> latched : STOP 또는 결함
  paused --> latched : STOP 또는 결함
  latched --> ended : ack (래치 확인)
```

[도식] 구현 상태: **구현됨**(`tools/row_executor/executor_main.py` `_run_state`). 상태 이름과 조건은
[`OPERATOR_API_KO.md`](steering/OPERATOR_API_KO.md) 2~4절이다. 멈춤(paused)에서는 구동 0을 그 틱에 보내고 조향은 그 자리에 둔다.
자동 재출발은 없다. STOP은 세션과 상관없이 누구나 누를 수 있고 run을 `operator_stop`으로 끝낸다.

## 7. 도식의 이름과 원본 대조

| 도식의 이름 | 원본(코드·규격) |
|---|---|
| `/scan`, `/imu/data_raw`, `/gps/fix` | 실행기 구독 허용 목록(검사 A13, 개정 30): `sim/ubts/steering/truth_isolation.py`, `tools/row_executor/executor_main.py` |
| `C L R S ttl`, 상태 줄 20 Hz, CRC-8, 115200 | [`ESP_PROTOCOL_V3_DRAFT_KO.md`](steering/ESP_PROTOCOL_V3_DRAFT_KO.md), 펌웨어 `firmware/ubts_steer_v3/` |
| 조향 브리지 V3, lease, 래치, `control.sock` | `dashboard/scripts/steer_bridge_core.py`, `steer_rover_bridge.py`, [`HOST_BRIDGE_V3_KO.md`](steering/HOST_BRIDGE_V3_KO.md) |
| 열 추종 제어기, 주시점, trim, 속도 상한 이름 | `sim/ubts/closed_loop/steer_control.py` 머리말 |
| L0~L3, 설정 키(`prior_enabled`, `local_grid_enabled`, `gnss_mode`) | `sim/ubts/closed_loop/estimator.py`, `tools/row_executor/config.py`, 설계 v2 V2-3 |
| `run_state` 값, `/api/start`·`/api/arm`·`/api/go`·`/api/stop`·`/api/ack` | `tools/row_executor/operator_http.py`, `executor_main.py`, [`OPERATOR_API_KO.md`](steering/OPERATOR_API_KO.md) |
| 컨테이너 이름, 포트 18090·8097, 도메인 43 | `compose.ubts-steer.yaml` 머리말 |
| 핀(D8·D5·D3·D0 구동, D6·D7 조향 PWM, D2 조향 EN, A0 가변저항) | ESP 규격 8.3절(배치 A안, 옛 ESP8266 설계) |
| ESP32-S3, BTS7960 1·2·3, `ANGLE`·`STATUS`·`STOP`·`n F pwm`, GPIO6 | 실기 펌웨어 `BTS7960_Serial_Control.ino`(레포 밖), [`HARDWARE_KO.md`](HARDWARE_KO.md) 6.1절 |
| 조향 법칙 `abs_hold_8`, ANGLE 눈금 대응 | `sim/ubts/steering/native/abs_hold8.cc`, `sim/ubts/steering/abs_hold.py`, [`ABS_HOLD8_SIM_KO.md`](steering/ABS_HOLD8_SIM_KO.md) |
