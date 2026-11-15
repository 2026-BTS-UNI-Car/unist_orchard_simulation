# 실행기와 ESP32-S3 브리지 (실차 보드, 2026-11-16)

- 작성: 2026-11-16 (작업 jetson-autonomy). 대상: 2026-11-16 차에 달린 ESP32-S3 보드(펌웨어 `ABS-HOLD-8-REBOUND-LIMITS`, BTS7960 3개)와
  그 Jetson 브리지(대시보드 레포 `scripts/esp32_steer_bridge.py`, `ROVER_ESP_KIND=esp32_abs_hold`).
- 앞 문서: [`EXECUTOR_STEER_KO.md`](EXECUTOR_STEER_KO.md)(V3 브리지와 실행기), [`HOST_BRIDGE_V3_KO.md`](HOST_BRIDGE_V3_KO.md), [`OPERATOR_API_KO.md`](OPERATOR_API_KO.md),
  [`ABS_HOLD8_SIM_KO.md`](ABS_HOLD8_SIM_KO.md)(새 보드 펌웨어의 SIM 흉내와 SIM 호스트 정책).
- 수치는 자리값, SIM 제안, 또는 2026-11-16 보드 시험 값이다. 가짜 보드로 확인한 것이며 차에서 확인하지 않았다.

## 1. 요약

실행기는 V3 조향 브리지와 말하도록 만들었다. 새 보드의 브리지는 같은 제어 소켓과 같은 조종 규칙(lease, 한 번 쓰는 ticket, seq + 1, deadman, 입력 기한
0.75 s, 자동 재무장 없음)을 쓰지만, 조향 눈금, 상태 칸, 펌웨어 감시가 다르다. 브리지를 다시 설치하지 않도록 **실행기 쪽 어댑터**
(`tools/row_executor/bridge_client.py`의 `Esp32AbsHoldAdapter`, `Esp32SteerBridgeClient`, `esp32_arm_precheck`)를 더했다. 설정 키
`steer_bridge_kind: esp32_abs_hold`가 그것을 고른다(기본 `v3`: SIM 과 V3 는 그대로).

## 2. 다른 점과 어댑터가 하는 일

| 실행기가 기대한 것(V3) | ESP32 브리지 | 어댑터 / 실행기 |
| --- | --- | --- |
| `steer_mrad` = 등가 바퀴각(왼쪽 +) | `steer_mrad` = 펌웨어 명령 눈금(왼쪽 +, ±20 명령도 = ±349), 브리지가 `ANGLE = −도`로 보냄 | 등가 ↔ 명령 눈금을 `steer_esp32_mark_deg`(명령 20의 등가각)로 바꾼다: 명령도 = −등가도 × 20 / mark. SIM 과 같은 식(ABS_HOLD8_SIM 3.3절). mark 17.831 은 SIM 가정(기계 한계 − 1°), 현장 B3 로 바꾼다. 측정각도 거꾸로 바꿔 실행기에 준다 |
| C 줄 20 Hz, MCU 서보 | 새 ANGLE 마다 펌웨어가 300 ms 기다리고 1000 ms 보정 예산을 다시 시작 | SIM 호스트 정책(`abs_hold_host`)을 어댑터가 한다: 새 목표는 마지막 것과 1 명령도 넘게 다르고, 마지막 뒤 1.0 s 가 지났거나 보드가 목표에 닿았거나(10 count) 유지가 없을 때만. 그 뒤 브리지 규칙(1 명령도 단위, 0.5 s 간격, 20° 넘는 이동은 나눔)이 따른다 |
| 취소 뒤 같은 ANGLE 다시 보냄(SIM 정책) | 취소된 목표는 요청이 바뀔 때까지 다시 보내지 않음 | 그대로 둔다(브리지 규칙이 더 보수적). 대신 아래 추종 감시가 남은 오차를 잡는다. SIM 과 다른 점 |
| Arm 전 점검: 펌웨어 V3, `cal_id`, `simulated`, 극성 기록 | `firmware` `ABS-HOLD-8-REBOUND-LIMITS`, CAL 줄, `drive.direction_verified`, `drive.firmware_watchdog_ms` | `esp32_arm_precheck`: 상태 ok·연결, `esp_kind`, 펌웨어 이름, front_steer, 브리지 한계 349, **구동 방향 확인됨**, **펌웨어 구동 감시**(설정 `steer_esp32_require_fw_watchdog`, 기본 true), 보정됨, **CAL 줄 = 설정 `steer_esp32_cal`**(cal_id 대신), 결함 없음, 상태 신선(0.3 s), 측정각 유한, STOP 확인, 아무도 lease 없음, 정지, 스냅샷 2 s 안. 거절이면 arm 을 보내지 않는다 |
| MCU ARM 확인(`arm_state` confirmed) | MCU ARM 없음(`arm_state` armed) | 브리지가 armed 이면 `confirmed`로 바꿔 보인다(그때부터 결함을 센다) |
| MCU 결함 비트(TRACK, POT_*, STALL …)와 호스트 래치 | `pot_invalid`, `steering_status_stale`, 래치 `hardware_fault`, `mcu_reset_detected`, `firmware_mismatch`, `stop_unconfirmed`, 사건 `steer_hold_cancelled`, `steer_stop` | 정지 사유 표에 `pot_invalid` → steering_feedback_invalid, `firmware_mismatch` → link_identity_mismatch, `mcu_drive_watchdog` → link_watchdog 를 더했다. 펌웨어 사건 둘은 결함이 아니라 `events`로 옮긴다(추종 감시가 판단) |
| 호스트 래치 `steering_not_following`(3°, 1.5 s) | 없음 | 어댑터 감시: lease 를 쥔 동안 요청 목표와 측정각 차가 **등가 3° 넘게 2.0 s**(자리값: V3 1.5 s + ANGLE 한 번) 이어지면 래치, 그 자리에서 op stop, 실행기는 그 사유로 run 을 끝낸다 |
| 호스트 래치 `steering_ineffective`(자이로 vs v·tanδ/L) | 없음, op `motion` 도 없음(보내면 ticket 없는 요청이라 lease 를 잃음) | op `motion`을 어댑터가 받아(브리지로 보내지 않음) V3 규칙을 그대로 돌린다: v > 0.2 m/s, \|δ\| > 3°, \|r − v·tanδ/L\| > max(3°/s, 0.5·예측) 가 1.5 s. L = `steer_wheelbase_m` |
| STOP 확인 계수(`counters.stop_sent/confirmed`) | `stop_state`만 | 어댑터가 보낸 stop 을 세고, 브리지 상태가 그 stop 의 이유(또는 0.1 s 뒤 스냅샷)로 `stop_state` confirmed 이면 확인으로 센다(실행기의 STOP 감시 사건) |
| 실행기 컨트롤러 휠베이스 0.76(자리값) | — | 설정 `steer_wheelbase_m`(측정 0.657)를 기본 컨트롤러(row_follow)에 넣는다. SIM 재측정(`follow_realdims`)과 같은 값 |

- 실행기 상태 신선도(0.3 s), 운영자 heartbeat 기한(1.0 s → 일시정지, 자동 재출발 없음), STOP 우선, 래치 뒤 확인 → Start → Arm → go, 출력 가드(조향 속도, 소프트 한계) 는 바뀌지 않았다.
- SHADOW 에서는 어댑터도 전송도 만들지 않는다(브리지 소켓에 닿지 않는다).

## 3. 차량 설정

`tools/row_executor/config.vehicle_esp32_shadow.json` = `python -B -m steering.vehicle_executor_config`(sim/ubts)의 출력(시험이 같은지 본다).
front_steer, 단일열, **SHADOW**, 소켓 `/run/ubts-rover/control.sock`, 운영자 API `127.0.0.1:8097`, 현장 조정 파일 `/var/lib/row_executor/field_tuning.json`.

| 값 | 설정 | 출처 |
| --- | --- | --- |
| 휠베이스 | 0.657 | MEASURED_20261116 |
| 소프트 한계 | 281 mrad(16.14°) | 프로필 조향 한계 20°(측정) − 소프트 여유(CONFIG) |
| 조향 변화율 한도 | 4.075 °/s | 0.8 × 보드 계단 응답 5.094 °/s(MEASURED_20261116, SIM 눈금) |
| 데드밴드 | 0.814° | 펌웨어 목표 허용 10 count |
| CAL | [1681, 1900, 2180, 20] | 펌웨어 |
| mark | 17.831° | SIM_ASSUMED(현장 B3 로 바꾼다) |
| LiDAR | x 0.1185(측정), yaw 179.08(ESTIMATED) | 프로필 |
| 몸체 | 1.22 × 0.70 | SPEC_PENDING |

비워 둔 값(현장에서 잴 때까지 null/false, 그래서 MOTOR 는 설정 검사에서 거절): C2 `breakaway_duty_pct`, C3 `v_measured_m_s`, C7 `measured_stop_distance_m`,
B5 `steer_measured_rate_*`(GO 문턱 결정과 함께; 5.09 °/s 는 문턱 8 °/s 에 걸린다), B2/B3 `steer_range_measured`·`steer_wheel_soft_margin_deg`, B4 `steer_min_pwm_pct`, F1 `steer_zero_offset_deg`,
B0 `steer_polarity_checked`. `steer_esp32_require_fw_watchdog` 는 true(보드 펌웨어에 구동 감시가 없어 MOTOR Arm 이 거절된다. 패치를 넣거나 결정으로 false).

## 4. 안전 규칙이 실차 보드에서 어떻게 지켜지나, 남는 것

| 위험 | V3 에서 막던 것 | ESP32 에서 막는 것 | 남는 것 |
| --- | --- | --- | --- |
| 실행기가 죽음·멈춤 | 브리지 입력 기한 0.75 s + MCU C TTL 300 ms | 브리지 입력 기한 0.75 s(→ STOP)와 브리지 자체 구동 TTL 0.75 s. 실행기 감시견(0.35 s)이 살아 있으면 구동 0 | MCU 쪽 300 ms TTL 이 없어 끊긴 뒤 멈춤까지 최대 약 0.75~0.85 s(가짜 보드: 0.85 s 한도 안) |
| 브리지가 죽음 | MCU watchdog 500 ms + C TTL | 브리지 SIGTERM/atexit 는 STOP(시험). SIGKILL·멈춤은 없음 | **펌웨어가 마지막 구동을 계속한다.** 펌웨어 구동 감시 패치(대시보드 기록 `runs/esp32_bridge_20261116/firmware_watchdog/`) 전에는 물리 비상정지. 그래서 Arm 기본은 감시 필요 |
| 운영자 화면 끊김 | 실행기 heartbeat 1.0 s → 일시정지(구동 0), 재출발 없음 | 같다(시험: 1.15 s 안에 `2 S`/`3 S`, 다시 이어져도 구동 없음) | — |
| STOP | op stop → 다음 틱 STOP, 확인·재전송 | op stop → 브리지 다음 단계(20 ms)에 STOP, 확인될 때까지 다시 씀, 0.6 s 넘으면 `stop_unconfirmed` 래치 | — |
| 조향이 안 따라옴 | MCU TRACK + 호스트 래치 | 어댑터 추종 감시(3°, 2 s) | 문턱은 자리값, 차에서 재지 않음. 펄스 사이 되밀림(외란)은 SIM 에서도 넓은 조건 실패 원인(`STEER_FOLLOW_REALDIMS_RESULT_KO.md`) |
| 조향이 먹지 않음(미끄럼) | 호스트 래치(자이로) | 어댑터가 같은 규칙 | 차에서 재지 않음 |
| 가변저항 고장 | MCU POT_* 결함 | 브리지 `pot_invalid`(STATUS INVALID, 취소 INVALID POT) → 정지, 실행기 steering_feedback_invalid | 펌웨어의 고장 감지는 V3 보다 적다(급변·잡음 감시 없음) |
| 보드 리셋 | boot_id, n 역행 | 배너로 `mcu_reset_detected` 래치(브리지) | 리셋이 늘 배너를 내는지는 한 번만 봄 |
| 구동 방향·조향 눈금 틀림 | 보정 기록(cal_id, 극성) | Arm 에 방향 확인과 CAL 줄 필요 | 극성(B0)과 눈금(B3)은 사람이 바퀴 든 시험으로 확인해야 한다 |
| 실행기 없이 움직임 | SHADOW 는 소켓을 만들지 않음 | 같다 + Jetson SHADOW 설치는 `/run/ubts-rover`를 마운트하지 않음 | — |

## 5. Jetson 실행 환경

- 실행 이미지: Jetson 에 이미 있는 ROS Iron arm64 이미지 `sha256:a59689051b2e…`(native 관찰 서비스용; Python 3.10.12, numpy 1.21.5, scipy 1.8.0, rclpy). 센서 발행자(sllidar_node, BNO085 드라이버)도 Iron 이다. SIM 의 Jazzy 이미지는 쓰지 않는다(배포판 섞임을 피함; 실행기는 순수 Python 이라 Iron 에서 돈다). 새 이미지 빌드·pull 없음.
- 토픽(2026-11-16 읽기 점검): `/scan` 발행 RELIABLE, `/imu/data_raw` 발행 BEST_EFFORT. 실행기 구독은 BEST_EFFORT(깊이 2 / 50)라 둘 다 맞는다. `ROS_DOMAIN_ID` 0, network·IPC host.
- 설치 묶음: 대시보드 레포 `deploy/jetson_row_executor/`(점검 기본, `--apply`, `--rollback`, `--mode shadow|motor`). 운영자 토큰은 `--token-file`로 대시보드 runtime 폴더에 쓰고 `--no-print-token`으로 로그에 남기지 않는다. 대시보드는 연결 파일(`runtime/row_executor/dashboard_link.json`)을 읽는다.

## 6. 시험

| 시험 | 내용 | 환경 |
| --- | --- | --- |
| `tools/row_executor/tests/test_esp32_adapter.py` | 눈금·부호, 정규화, 정책(1 s 간격, 닿으면 다음 목표), motion 이 브리지에 가지 않음, 추종·미끄럼 감시와 stop 한 번, 다른 사람 lease 에는 stop 없음, STOP 확인 계수, Arm 전 점검 규칙 하나씩, V3 점검이 이 상태를 거절, 클라이언트 lease 잃음 → 래치, 설정 규칙, 기본 digest 그대로, 휠베이스 | 대시보드 없이 |
| `tests/test_esp32_executor_e2e.py` | 실행기(ExecutorLoop, MOTOR, one_press) → 어댑터 → **대시보드의 Esp32Bridge·Esp32SteerMotor 그대로** → 가짜 펌웨어, 한 가상 시계, JSON 왕복. go 전 구동 줄 없음(Start만, Arm 뒤 기다림), 방향 미확인·감시 없음·CAL 다름 → Arm 거절(arm 안 보냄), 왼쪽 100 mrad → `ANGLE -6.0`, 가변저항 왼쪽, 측정 되돌림, 목표 변경 수, 화면 STOP → 40 ms 안 구동 0 · operator_stop · 래치 · 재출발 없음, heartbeat 끊김 → 1.15 s 안 구동 0 · 다시 이어져도 구동 없음, 실행기 멈춤 → 0.85 s 안 STOP, 가변저항 고착 → steering_not_following, INVALID POT → 정지 | `UBTS_DASH_DIR`=대시보드 레포 |
| `tests/test_esp32_executor_linux.py` | 실시간: 실행기 → 실제 Unix 소켓 → **브리지 프로세스**(`python -m scripts.esp32_steer_bridge`, 실제 USB 전송 코드, 가짜 udev) → pty 의 가짜 펌웨어. go 뒤 구동·왼쪽 ANGLE, 화면 STOP 0.2 s 안 구동 0, 실행기 멈춤 1.0 s 안 구동 0, 브리지 SIGTERM → 구동 0 · run 끝 | Linux + `UBTS_DASH_DIR` |
| `sim/ubts/test_steering_vehicle_executor_config.py` | 레포의 차량 설정 = 생성기 출력, 값과 출처, MOTOR 거절, SHADOW 루프에 전송 없음 | sim/ubts |

결과(2026-11-16, 이 브랜치): 7절.

## 7. 결과와 남은 일

결과와 Jetson 설치 상태는 작업 기록(`steering-sim-20261108/runs/jetson_autonomy_20261116/NOTES_KO.txt`)과 대시보드 레포
`deploy/jetson_row_executor/README_KO.md`에 적는다. 남은 일: 현장 첫 조향 시험(B0, B2/B3, B5, F1, 구동 방향), 구동 측정(C2, C3, C7), GO 문턱과 펌웨어
감시 결정, 그 뒤 MOTOR 설정(대시보드 레포 `docs/FIELD_STEER_AUTONOMY_TEST_KO.md`). 어댑터의 정책·감시 수치는 SIM 이 아닌 가짜 보드로만 확인했다(SIM T1 은 V3 링크 위의 abs_hold8 흉내라 이 어댑터를 지나지 않는다).
