# 하드웨어와 제품 사양 (알려진 것과 자리값)

- 작성: 2026-11-13 (작업 DOC-06). 이 저장소의 SIM이 쓰는 차량 값과, 사용자가 알려 준 사양·과거 측정을 한 문서에 모았다.
- 2026-11-16 고침: 사용자가 실제 치수를 재 주었고 조향 보드가 ESP32-S3 하나(BTS7960 드라이버 세 개, 펌웨어 ABS-HOLD-8-REBOUND-LIMITS)로
  바뀌었다. 옛 ESP8266 보드는 뺐다. 측정값은 1절·3절·4.1절, 새 보드는 2절·6절. 2026-11-15까지의 설계(ESP8266 + 펌웨어 V3)는 6.2절과
  [`ARCHITECTURE_KO.md`](ARCHITECTURE_KO.md) 1.2절에 기록으로 남긴다.
- 실차 배선·핀·설치의 원본은 대시보드 저장소다([`DOCS_POLICY_KO.md`](DOCS_POLICY_KO.md) 1절). 여기 핀 표는 펌웨어 V3 규격의 배치 안이다.
- **조향 장치의 값은 모두 자리값이다.** 사용자가 실측·사양을 주면 [`SPEC_UPDATE_KO.md`](steering/SPEC_UPDATE_KO.md)의 절차로 바꾼다.
  묻는 질문 목록은 [`VEHICLE_SPEC_REQUEST_KO.md`](steering/VEHICLE_SPEC_REQUEST_KO.md)(Q1~Q29)다.
- 이 저장소의 SIM 수치는 자리값 기반 SIM 결과이며 현장 성능이 아니다.

출처 표시: **제품표기**(라벨·판매 설명), **사용자**(사용자 설명·어림값, 날짜), **측정**(기록에서 잰 값, 날짜), **설정**(Jetson·실행기 설정),
**펌웨어**(펌웨어 기본값), **자리값**(SIM이 정한 가정. 프로필의 `SPEC_PENDING`·`SIM_ASSUMED`), **미정**(아무 값도 없음).

## 1. 차량

| 항목 | 값 | 출처 |
|---|---|---|
| 바탕 차량 | 2인승 어린이용 4륜 전동차를 개조. 상부를 문 높이까지 잘라냄 | 사용자 |
| 개조 전 크기 | 1.22 × 0.70 × 0.75 m, 하중 60 kg, 배터리 명목 12 V | 제품표기(개조 전. 지금 발자국·휠베이스·윤거가 아님) |
| 조향 | 앞바퀴 조향(2026-11-08~). 조향 모터는 서보가 아닌 일반 DC 모터, BTS7960 드라이버로 돌림. 조향 장치와 가변저항을 벨트로 연결하고 ESP32-S3가 가변저항 값으로 짧은 펄스를 주어 목표 각을 붙잡는다(펌웨어 ABS-HOLD-8, 2절) | 사용자(2026-11-08, 11-11, 11-16), 펌웨어 소스 |
| 조향 각 한계 | 좌우 약 20°까지 돌고 그 이상 안 감. 0°에 두어도 축 유격으로 좌우 오차가 조금 있다(크기 미측정) | 사용자 측정 2026-11-16 |
| 구동 | 4륜, 한쪽 앞뒤 모터가 드라이버 하나를 나눠 씀(왼쪽 = 드라이버 2, 오른쪽 = 드라이버 3) | 사용자·펌웨어 소스(2026-11-16) |
| 모터 드라이버 | BTS7960 모듈 3개: 1 조향, 2 왼쪽 구동, 3 오른쪽 구동 | 사용자·펌웨어 READY 줄(2026-11-16) |
| 바퀴 | 지름 0.26 m(반경 0.13), 두께 0.095 m | 사용자 측정 2026-11-16 |
| 휠베이스 | 0.657 m(앞뒤 축 간격) | 사용자 측정 2026-11-16 |
| 윤거 | 바퀴 바깥-바깥 0.622 m → 바퀴 중심 사이 0.527 m(앞뒤 구별 없이 준 값) | 사용자 측정 2026-11-16 |
| 휠 엔코더 | 없음. 속도·이동거리는 LiDAR 스캔 정합으로 재고 PWM은 한정된 사전값으로만 쓴다 | 사용자 |
| 리밋 스위치 | 없는 것으로 본다 | 사용자 설명에서 추정(Q25) |
| 조향 속도 | 명령 0 → ±20 약 3.4~3.6 s(마지막 긴 펄스까지), ±20 → 0 약 2.2~2.9 s(서 있는 차, 보드 시험). 명령 눈금의 물리 각은 미측정 | 측정(보드 시험 기록) 2026-11-16 |
| 질량, 무게중심, 발자국(차체 길이·폭), 킹핀 폭, Ackermann | 실측 없음. SIM은 4절의 자리값 | 미정 |

## 2. 컴퓨터와 보드

| 항목 | 값 | 출처 |
|---|---|---|
| 상위 컴퓨터 | NVIDIA Jetson Orin Nano Super 개발 키트(aarch64, Ubuntu 22.04 / JetPack 6 계열). 센서 드라이버는 ROS 2 Iron 컨테이너 | 설정(과거 설치 기록) |
| 하위 보드(2026-11-16~) | ESP32-S3 하나, 네이티브 USB(USB JTAG/serial)로 Jetson USB-C에 연결(Jetson `/dev/ttyACM*`, 115200 줄 단위 글자 명령). BTS7960 세 개(조향, 왼쪽 구동, 오른쪽 구동)를 같은 보드에서 돌린다 | 사용자·Jetson 확인(2026-11-16) |
| 보드 펌웨어(실기) | `ABS-HOLD-8-REBOUND-LIMITS`(`BTS7960_Serial_Control.ino`, 이 저장소 밖). 명령: `ANGLE n`(절대 조향, −왼쪽/+오른쪽, ±20 명령도), `STATUS`, `STOP`, `n F pwm` / `n R pwm` / `n S`(모터 번호 1~3, 방향, PWM 0~255), `POT`, `CENTER`·`LEFT n`·`RIGHT n`. 조향 수동 명령은 1초 한도, ANGLE 유지는 보정마다 구동 1초 한도. 구동 모터(2, 3)는 명령이 끊겨도 서는 장치가 없다 | 펌웨어 소스(2026-11-16) |
| 옛 하위 보드(2026-11-15까지) | ESP8266 NodeMCU 계열(USB 직렬 115200, CP2102). 뺐다 | 사용자 |
| SIM 펌웨어 | 조향 펌웨어 V3(`firmware/ubts_steer_v3/`). 실기에 쓴 적이 없다. SIM은 PC용 흉내 층으로 이 소스를 돌리며 직렬 규약·ARM·TTL·STOP·구동 껍데기로 쓴다. 프로필 `steer_fw_mode abs_hold_8`이면 조향 법칙만 실기 펌웨어의 펄스 유지로 바꾼다([`ABS_HOLD8_SIM_KO.md`](steering/ABS_HOLD8_SIM_KO.md)) | 펌웨어 |
| SIM 실행 | Windows Docker Desktop + WSL2, ROS 2 Jazzy, Gazebo Harmonic(gz-sim 8). 스택 메모리 예산 6 GiB | 설정 |

## 3. 센서

| 센서 | 값 | 쓰임 | 출처 |
|---|---|---|---|
| 2D LiDAR | SLAMTEC RPLIDAR S2, 360° 단일 평면 ToF, 10 Hz. 실제 한 스캔 3240빔(DenseBoost). 최대 거리 흰 면 30 m / 검은 면 10 m | 주 입력(통로, 열 끝, 앞 여유) | 측정 2026-10-10(빔 수·주기), 제품표기(거리) |
| LiDAR 장착 | 꼭대기가 지면에서 0.57 m, 앞차축에서 0.21 m 뒤(뒤차축에서 0.447 m 앞), 좌우는 바퀴 기준 가운데. 스캔 평면 높이는 꼭대기 − 0.02 m(자리값: S2 도면 미확인). 주행축 관례 `lidar_yaw_offset_deg` 179.08° | | 사용자 측정 2026-11-16(위치), 측정 2026-10-10 기반 추정(yaw, ±10°). 옛 값: 약 0.60 m(사용자 어림) |
| IMU | Adafruit BNO085, I2C. raw `/imu/data_raw`를 그대로 쓴다(소프트웨어 회전·0점 보정 없음). Jetson 설정 33 Hz(10/10 기록은 약 20 Hz) | 주 입력(gyro) | 설정, 측정 2026-10-10 |
| IMU 장착 | LiDAR와 x·y가 거의 같음. 기울기·높이는 미측정 | | 사용자 / 미정 |
| GNSS | u-blox ZED-F9P 보드, USB. RTK 보정은 쓰지 않음. 10/10 기록 수평 정확도 p50 5.3~10.0 m(열 간격 3.6~4.1 m보다 큼) | 약한 층(기록, 지오펜스. 기본 꺼짐) | 측정 2026-10-10 |
| 카메라 | USB UVC, 최대 1920 × 1080 @ 30 fps | 표시·기록만. 위치 추정에 쓰지 않음 | 제품표기 |
| 조향 가변저항 | 벨트로 조향축과 연결, ESP32-S3 GPIO6(12비트 ADC, 11 dB 감쇠). 보드 보정: 오른쪽 1681 / 가운데 1900 / 왼쪽 2180 count = 명령 +20 / 0 / −20(오른쪽으로 갈수록 값이 준다). 조용한 가운데 값 1896~1903. 종류·저항값·회전 범위·벨트 비 미정 | 조향 위치 되먹임 | 펌웨어 소스·보드 시험(2026-11-16) / 미정 |

## 4. SIM 자리값 표 (프로필)

- 기본 프로필은 `front_steer_v0_1_placeholder`다. 아래 표는 그다음 판 `front_steer_v0_3_placeholder`(스키마 v3, v0.2와 같은 값)의
  값이며 v0.1과 다른 칸은 괄호에 적었다. v0.3은 정상 대본 판정(개정 43 ②)이 불합격이라 아직 기본이 아니다([`RESULTS_KO.md`](RESULTS_KO.md)).
- 파일: `sim/ubts/steering/profiles/`. v0.3의 매개변수는 176개이고 출처 태그는 SPEC_PENDING 47, SIM_ASSUMED 45, FIRMWARE 26,
  CONFIG 19, DERIVED 11, ESTIMATED 10, MEASURED_20261010 5, SIM_NUMERIC 4, MEASURED_20261010_OLD_DRIVETRAIN 4, GENERIC_SPEC 3,
  DEFINITION 1, USER_APPROX 1이다. 측정값(MEASURED)은 조향 장치를 달기 전 차에서 잰 것이다.
- 태그 뜻: `SPEC_PENDING` 사양을 기다리는 자리값, `SIM_ASSUMED` SIM 가정, `FIRMWARE` 펌웨어 기본값, `CONFIG` 설정,
  `USER_APPROX` 사용자 어림값, `ESTIMATED` 근거에서 추정, `GENERIC_SPEC` 부품 일반 자료, `MEASURED_<날짜>` 측정.

| 항목 | 프로필 키 | 값 | 단위 | 시험 범위 | 출처 태그 |
|---|---|---|---|---|---|
| 휠베이스 | `wheelbase_m` | 0.76 | m | 0.65 ~ 0.9 | SPEC_PENDING |
| 앞 윤거 | `track_front_m` | 0.56 | m | 0.5 ~ 0.65 | SPEC_PENDING |
| 뒤 윤거 | `track_rear_m` | 0.56 | m | 0.5 ~ 0.65 | SPEC_PENDING |
| 킹핀 사이 거리 | `kingpin_width_m` | 0.46 | m | 0.4 ~ 0.6 | SPEC_PENDING |
| 바퀴 반경 | `wheel_radius_front_m` | 0.14 | m | 0.1 ~ 0.16 | SPEC_PENDING |
| 바퀴 폭 | `wheel_width_m` | 0.12 | m | 0.08 ~ 0.16 | SPEC_PENDING |
| 발자국 길이 | `footprint_length_m` | 1.22 | m | 1 ~ 1.5 | SPEC_PENDING |
| 발자국 폭 | `footprint_width_m` | 0.7 | m | 0.6 ~ 0.85 | SPEC_PENDING |
| 기준점→앞범퍼 | `front_bumper_x_in_base_m` | 0.61 | m | 0.5 ~ 0.75 | SPEC_PENDING |
| 기준점→뒤차축 | `rear_axle_x_in_base_m` | -0.38 | m | -0.5 ~ -0.25 | SPEC_PENDING |
| 차체 질량(바퀴 제외) | `mass_kg` | 24 | kg | 20 ~ 60 | SIM_ASSUMED |
| 조향 형식 | `steer_type` | linkage | - | linkage/axle_pivot | SPEC_PENDING |
| Ackermann 비율 | `steer_ackermann_ratio` | 0.5 | - | 0 ~ 1 | SPEC_PENDING |
| 바퀴 기계 한계 | `steer_mech_limit_deg` | 28 (네 방향 같음) | deg | 20 ~ 35 | SPEC_PENDING |
| 소프트 한계 여유 | `steer_soft_margin_deg` | 3 | deg | 2 ~ 5 | CONFIG |
| 조향 속도(주행) | `steer_rate_max_deg_s` | 30 | deg/s | 10 ~ 60 | SPEC_PENDING |
| 조향 속도(정지) | `steer_rate_standstill_deg_s` | 15 | deg/s | 5 ~ 30 | SPEC_PENDING |
| 링크 유격 | `steer_backlash_deg` | 2 | deg | 0 ~ 6 | SPEC_PENDING |
| 조향 0점 오차 | `steer_zero_offset_deg` | 평균 0, sd 0.7 (run마다) | deg | -2 ~ 2 | SPEC_PENDING |
| 자기 잠금 | `steer_self_locking` | 아니오 | - | False/True | SPEC_PENDING |
| 리밋 스위치 | `steer_limit_switch` | none | - | none/both | SPEC_PENDING |
| 벨트 종류 | `belt_type` | timing | - | timing/flat/round | SPEC_PENDING |
| 가변저항 쪽 풀리 이 수 | `belt_pulley_teeth_pot` | 20 | teeth | 10 ~ 60 | SPEC_PENDING |
| 벨트 비(가변저항:조향) | `pot_to_steer_ratio` | 1 | - | 0.5 ~ 5 | SPEC_PENDING |
| 가변저항 회전 범위 | `pot_range_deg` | 270 | deg | 240 ~ 340 | SPEC_PENDING |
| 가변저항 위치 | `pot_location` | motor | - | motor/wheel | SPEC_PENDING |
| 비례 측정 ADC | `adc_ratiometric` | 아니오 | - | False/True | GENERIC_SPEC |
| ESP 서보 주기 | `servo_hz` | 500 | Hz | 500/1000 | FIRMWARE |
| 서보 비례 이득 | `servo_kp_pct_per_deg` | 15 | %/deg | 5 ~ 40 | FIRMWARE |
| 서보 데드밴드 | `servo_deadband_deg` | 0.5 | deg | 0.2 ~ 1.5 | FIRMWARE |
| 서보 최소 출력 | `servo_min_pwm_pct` | 45 (v0.1: 40) | % | 10 ~ 50 | FIRMWARE |
| 조향 모터 정지 토크 | `steer_motor_stall_torque_nm` | 5 | N*m | 2 ~ 15 | SPEC_PENDING |
| 설계 외란 토크 | `steer_disturbance_design_nm` | 2.5 | N*m | 0.5 ~ 5 | SIM_ASSUMED |
| C 줄 TTL | `c_ttl_max_ms` | 300 | ms | 150 ~ 500 | FIRMWARE |
| 링크 watchdog | `watchdog_ms` | 500 | ms | 300 ~ 2000 | FIRMWARE |
| MCU 상태 줄 | `status_hz` | 20 | Hz | 10 ~ 50 | FIRMWARE |
| 구동 램프 | `mcu_ramp` | pwm_per_s 255, reversal_dwell_ms 120 | PWM/s; ms | (칸별) | FIRMWARE |
| 구동 바퀴 | `drive_wheels` | 4wd_lr_pairs | - | 4wd_lr_pairs/rear2/front2 | SPEC_PENDING |
| 좌·우 구동 | `drive_lr_mode` | equal_duty | - | equal_duty/electronic_diff | CONFIG |
| 목표 속도 | `v_target_m_s` | 0.4 | m/s | 0.2 ~ 0.6 | CONFIG |
| LiDAR 앞 위치 | `lidar_x_in_base_m` | 0.18 | m | 0 ~ 0.35 | SIM_ASSUMED |
| LiDAR 높이 | `lidar_z_m` | 0.6 | m | 0.55 ~ 0.65 | USER_APPROX |
| LiDAR yaw 관례 | `lidar_yaw_offset_deg` | 179.08 | deg | 169.08 ~ 189.08 | ESTIMATED |
| LiDAR 빔 수 | `lidar_beams` | 3240 | beams | 3240 ~ 3240 | MEASURED_20261010 |
| LiDAR 주기 | `lidar_hz` | 10 | Hz | 10 ~ 10 | MEASURED_20261010 |
| IMU 주기 | `imu_rate_hz` | 33 | Hz | 20/33 | CONFIG |
| IMU 위치 | `imu_pose_in_base` | x 0.18, y 0, z 0.56 | m | (칸별) | SIM_ASSUMED |
| 카메라 위치 | `camera_pose_in_base` | x 0.4, y 0, z 0.48 | m | (칸별) | SIM_ASSUMED |
| GNSS 안테나 위치 | `gnss_antenna_pose_in_base` | x 0, y 0, z 0.9 | m | (칸별) | SIM_ASSUMED |

- 목표 속도는 실행기 설정에서 0.40 m/s(범위 0.20~0.50, 최대 0.50 잠금)다(설계 v2 V2-7). 프로필의 범위(0.2~0.6)는 민감도 시험용이다.
- 펌웨어 고장 판정 문턱(극성, 드라이버 고착, 열 예산, 추종)은 v0.2에서 바꿨다: [`STEER_PROFILE_V02_KO.md`](steering/STEER_PROFILE_V02_KO.md).
- 프로필 모형의 식(구동기 L0/L1, 가변저항 측정 사슬, 외란)은 설계 v1 3.3~3.6절과 [`STEER_EXT_MODEL_KO.md`](steering/STEER_EXT_MODEL_KO.md)에 있다.

### 4.1 실제 치수 프로필 `front_steer_v1_spec_20261116` (2026-11-16)

위 표(v0.3)의 바뀐 칸만 적는다. 만든 법과 모든 칸의 출처는 [`ABS_HOLD8_SIM_KO.md`](steering/ABS_HOLD8_SIM_KO.md) 2절이다. 이 판도
SPEC_PENDING 34칸이 남아 결과는 자리값 기반 SIM 결과다. 기본 프로필(v0.1)은 바꾸지 않았다.

| 항목 | 프로필 키 | 값 | 출처 태그 |
|---|---|---|---|
| 휠베이스 | `wheelbase_m` | 0.657 | MEASURED_20261116 |
| 앞·뒤 윤거 | `track_front_m`, `track_rear_m` | 0.527 | MEASURED_20261116 |
| 바퀴 반경·폭 | `wheel_radius_front_m`, `wheel_radius_rear_m`, `wheel_width_m` | 0.13, 0.095 | MEASURED_20261116 |
| 바퀴 기계 한계 | `steer_mech_limit_deg` | 20(네 방향) | MEASURED_20261116 |
| 기준점→뒤차축 | `rear_axle_x_in_base_m` | −0.3285 | DEFINITION(SIM 규약: 두 차축이 발자국 가운데에 대칭) |
| LiDAR 위치 | `lidar_x_in_base_m`, `lidar_y_in_base_m` | 0.1185, 0 | MEASURED_20261116(앞차축 0.21 m 뒤) |
| LiDAR 스캔 평면 높이 | `lidar_z_m` | 0.55 | ESTIMATED(꼭대기 0.57 측정 − 0.02 자리값) |
| 조향 모형 | `steer_rate_max_deg_s`, `steer_min_pwm_pct`, `steer_motor_tau_s` | 100, 10, 0.02 | ESTIMATED / SIM_ASSUMED(PWM 100 펄스에 맞춤) |
| 조향 법칙 | `steer_fw_mode` | `abs_hold_8` | CONFIG |
| 보드 펌웨어 상수 | `abs_hold_fw` | 보정 1681/1900/2180, 펄스 PWM 100·80/50 ms, 끔 300 ms, 허용 10, 다시 시작 16 등 | FIRMWARE |
| 계단 응답 | `steer_step_time_measured_s` | 바깥 3.5 s, 가운데로 2.5 s | MEASURED_20261116 |

## 5. 구동 응답 (조향 장치를 달기 전 차의 측정)

2026-10-03·10-10 수동 주행 14개에서 LiDAR 스캔 정합으로 잰 직진 속도다. 조향 장치를 단 차의 값은 아직 없다. 전체 표와 방법은
[`drive_response/README_KO.md`](drive_response/README_KO.md)다.

| duty | 속도 m/s | 등급 |
|---|---|---|
| 40 | 0.373 | 참고(구간 1) |
| 70 | 0.641 | 측정(GNSS) |
| 88 | 0.771 | 측정 |
| 95 | 0.864 | 측정 |
| 100 | 0.928 | 측정(322구간) |

- 자율 주행에 쓸 25~60 %는 duty마다 구간 1~3개뿐이고 40 % 미만은 자료가 없다(외삽, ±20 % 넘게 틀릴 수 있다).
- 정지에서 100 %로 출발하면 1초에 0.34 m, 2초에 1.29 m를 갔다. 0.8~0.9 m/s에서 키를 놓으면 약 0.55 m 더 갔다.
- MCU 램프는 255 PWM/s(= 100 %/s)다(펌웨어. 10/10 측정 p50 249/s).

## 6. 보드 핀

### 6.1 ESP32-S3 (2026-11-16, 펌웨어 소스의 표)

| 드라이버 | R_EN | RPWM | L_EN | LPWM | 비고 |
|---|---|---|---|---|---|
| 1 조향 | 16 | 17 | 18 | 8 | F(RPWM) = 오른쪽. 가변저항 와이퍼 GPIO6 |
| 2 왼쪽 구동 | 46 | 3 | 9 | 10 | GPIO3·46은 부팅 스트래핑 핀 |
| 3 오른쪽 구동 | 12 | 11 | 13 | 14 | |

출처: `BTS7960_Serial_Control.ino`의 `motors[]`(2026-11-16 받은 소스, 레포 밖). PWM 20 kHz, 8비트.

### 6.2 ESP8266 (배치 A안, 옛 보드·SIM 펌웨어 V3)

| NodeMCU | GPIO | 역할 |
|---|---|---|
| D8 | 15 | 왼쪽 전진 PWM |
| D5 | 14 | 왼쪽 후진 PWM |
| D3 | 0 | 오른쪽 전진 PWM |
| D0 | 16 | 오른쪽 후진 PWM |
| D1 | 5 | 구동 EN(드라이버 2개의 EN 4선을 묶음, 자리값: 배선 확인 Q5) |
| D2 | 4 | 조향 EN |
| D6 | 12 | 조향 PWM A |
| D7 | 13 | 조향 PWM B |
| A0 | ADC | 가변저항 와이퍼 |

출처: [`ESP_PROTOCOL_V3_DRAFT_KO.md`](steering/ESP_PROTOCOL_V3_DRAFT_KO.md) 8.3절. 구동 핀 4개는 지금 실차(V2)와 같다.

## 7. 아직 모르는 것 (답이 오면 SIM 값을 바꾼다)

- 벨트 종류와 양쪽 풀리 크기, 가변저항이 걸린 위치(모터 쪽/바퀴 쪽), 가변저항 종류·저항값·회전 범위(Q1, Q2, Q22~Q24).
- 조향 모터 정격·정지 전류·감속비, 자기 잠금 여부, 조향 드라이버 종류와 핀(Q11, Q26).
- 질량, 발자국, 킹핀 폭, Ackermann(Q2, B1 측정). 휠베이스·윤거·바퀴·조향 한계는 2026-11-16에 받았다.
- 명령 눈금(±20)의 물리 바퀴 각, 0° 기계 오차의 크기, 주행 중 조향 속도(B3, B5, F1).
- S2 스캔 평면이 꼭대기에서 얼마나 아래인지(제품 도면), IMU 장착 위치·자세(Q14).
- 새 보드 펌웨어의 구동 쪽 안전 장치(명령이 끊기면 서는 시간 한도가 없다), 새 보드용 Jetson 브리지(아직 없음).
- 조향 장치를 단 뒤의 duty→속도(Q3, Q29, F3 측정).
- 배터리 종류·용량, 전압 기록 가능 여부(Q13, Q27). 물리 비상정지가 조향 전원도 끊는지(Q15).
- 실물 사진(차량 네 면, 조향 장치, 배선)은 아직 없다(DOC-16). 판매 사이트의 제품 사진은 저작권 때문에 쓰지 않는다.
