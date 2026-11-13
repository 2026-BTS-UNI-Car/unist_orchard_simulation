# 하드웨어와 제품 사양 (알려진 것과 자리값)

- 작성: 2026-11-13 (작업 DOC-06). 이 저장소의 SIM이 쓰는 차량 값과, 사용자가 알려 준 사양·과거 측정을 한 문서에 모았다.
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
| 조향 | 앞바퀴 조향으로 바꾸는 중(2026-11-08~). 조향 모터는 서보가 아닌 일반 DC 모터, 모터 드라이버로 돌림. 조향 장치와 가변저항을 벨트로 연결하고 가변저항 값이 목표에 오도록 모터를 돌림 | 사용자(2026-11-08, 11-11) |
| 구동 | 4륜, 한쪽 앞뒤 모터가 드라이버 하나를 나눠 씀, 좌·우 같은 duty(조향 차에서도 그대로인지는 미정, Q3) | 사용자·설정 |
| 모터 드라이버 | 구동용 BTS7960 계열 모듈 2개. 조향용 드라이버 종류는 미정(Q26) | 사용자 / 미정 |
| 휠 엔코더 | 없음. 속도·이동거리는 LiDAR 스캔 정합으로 재고 PWM은 한정된 사전값으로만 쓴다 | 사용자 |
| 리밋 스위치 | 없는 것으로 본다 | 사용자 설명에서 추정(Q25) |
| 질량, 무게중심, 휠베이스, 윤거, 바퀴 지름, 조향각 한계, 조향 속도 | 실측 없음. SIM은 4절의 자리값 | 미정 |

## 2. 컴퓨터와 보드

| 항목 | 값 | 출처 |
|---|---|---|
| 상위 컴퓨터 | NVIDIA Jetson Orin Nano Super 개발 키트(aarch64, Ubuntu 22.04 / JetPack 6 계열). 센서 드라이버는 ROS 2 Iron 컨테이너 | 설정(과거 설치 기록) |
| 하위 보드 | ESP8266 NodeMCU 계열(USB 직렬 115200). 구동 PWM과 조향 위치 루프를 같은 보드에서 돈다(배치 A안) | 사용자·설계(Q5 기본값) |
| ESP 펌웨어 | 조향 펌웨어 V3(`firmware/ubts_steer_v3/`, build id `04353B49`). SIM은 PC용 흉내 층으로 같은 소스를 돌린다. ESP8266 코어 3.1.2로 컴파일·링크만 했고 실기에 쓰지 않았다 | 펌웨어 |
| SIM 실행 | Windows Docker Desktop + WSL2, ROS 2 Jazzy, Gazebo Harmonic(gz-sim 8). 스택 메모리 예산 6 GiB | 설정 |

## 3. 센서

| 센서 | 값 | 쓰임 | 출처 |
|---|---|---|---|
| 2D LiDAR | SLAMTEC RPLIDAR S2, 360° 단일 평면 ToF, 10 Hz. 실제 한 스캔 3240빔(DenseBoost). 최대 거리 흰 면 30 m / 검은 면 10 m | 주 입력(통로, 열 끝, 앞 여유) | 측정 2026-10-10(빔 수·주기), 제품표기(거리) |
| LiDAR 장착 | 지면에서 약 0.60 m, 옛 핸들 근처(차체 중앙보다 조금 앞). 주행축 관례 `lidar_yaw_offset_deg` 179.08° | | 사용자(높이·위치), 측정 2026-10-10 기반 추정(yaw, ±10°) |
| IMU | Adafruit BNO085, I2C. raw `/imu/data_raw`를 그대로 쓴다(소프트웨어 회전·0점 보정 없음). Jetson 설정 33 Hz(10/10 기록은 약 20 Hz) | 주 입력(gyro) | 설정, 측정 2026-10-10 |
| IMU 장착 | LiDAR와 x·y가 거의 같음. 기울기·높이는 미측정 | | 사용자 / 미정 |
| GNSS | u-blox ZED-F9P 보드, USB. RTK 보정은 쓰지 않음. 10/10 기록 수평 정확도 p50 5.3~10.0 m(열 간격 3.6~4.1 m보다 큼) | 약한 층(기록, 지오펜스. 기본 꺼짐) | 측정 2026-10-10 |
| 카메라 | USB UVC, 최대 1920 × 1080 @ 30 fps | 표시·기록만. 위치 추정에 쓰지 않음 | 제품표기 |
| 조향 가변저항 | 벨트로 조향축과 연결, ESP A0(10비트 ADC, 3.3 V 급전, 비례 측정 아님). 종류·저항값·회전 범위·벨트 비 미정 | 조향 위치 되먹임과 고장 감시, 한정된 사전값 | 사용자 / 미정 |

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

## 6. ESP8266 핀 (배치 A안, 실기 배선 확인 전)

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
- 휠베이스, 윤거, 바퀴 지름, 조향각 한계, 조향 속도, 질량(Q2, B1 측정).
- 조향 장치를 단 뒤의 구동 바퀴 구성과 duty→속도(Q3, Q29, F3 측정), LiDAR·IMU 장착 변화(Q14).
- 배터리 종류·용량, 전압 기록 가능 여부(Q13, Q27). 물리 비상정지가 조향 전원도 끊는지(Q15).
- 실물 사진(차량 네 면, 조향 장치, 배선)은 아직 없다(DOC-16). 판매 사이트의 제품 사진은 저작권 때문에 쓰지 않는다.
