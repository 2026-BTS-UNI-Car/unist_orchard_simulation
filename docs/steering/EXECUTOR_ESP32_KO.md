# 실행기와 ESP32-S3 브리지 (실차 보드, 2026-11-16)

- 작성: 2026-11-16 (작업 jetson-autonomy). 대상: 2026-11-16 차에 달린 ESP32-S3 보드(펌웨어 `ABS-HOLD-8-REBOUND-LIMITS`, BTS7960 3개)와
  그 Jetson 브리지(대시보드 레포 `scripts/esp32_steer_bridge.py`, `ROVER_ESP_KIND=esp32_abs_hold`).
- 앞 문서: [`EXECUTOR_STEER_KO.md`](EXECUTOR_STEER_KO.md)(V3 브리지와 실행기), [`HOST_BRIDGE_V3_KO.md`](HOST_BRIDGE_V3_KO.md), [`OPERATOR_API_KO.md`](OPERATOR_API_KO.md),
  [`ABS_HOLD8_SIM_KO.md`](ABS_HOLD8_SIM_KO.md)(새 보드 펌웨어의 SIM 흉내와 SIM 호스트 정책).
- 수치는 자리값, SIM 제안, 또는 2026-11-16 보드 시험 값이다. 가짜 보드로 확인한 것이며 차에서 확인하지 않았다.
- 고침 2026-11-16 밤(작업 field-steer-config): 차량 설정의 보드 값(CAL, 명령 눈금, 조향 속도)을 **현장 조향 기록**에서 읽는다(3절).
  그 값은 앞바퀴를 띄운 정지 상태에서 조향만 움직여 잰 현장 측정이다. 주행 결과가 아니다.

## 1. 요약

실행기는 V3 조향 브리지와 말하도록 만들었다. 새 보드의 브리지는 같은 제어 소켓과 같은 조종 규칙(lease, 한 번 쓰는 ticket, seq + 1, deadman, 입력 기한
0.75 s, 자동 재무장 없음)을 쓰지만, 조향 눈금, 상태 칸, 펌웨어 감시가 다르다. 브리지를 다시 설치하지 않도록 **실행기 쪽 어댑터**
(`tools/row_executor/bridge_client.py`의 `Esp32AbsHoldAdapter`, `Esp32SteerBridgeClient`, `esp32_arm_precheck`)를 더했다. 설정 키
`steer_bridge_kind: esp32_abs_hold`가 그것을 고른다(기본 `v3`: SIM 과 V3 는 그대로).

## 2. 다른 점과 어댑터가 하는 일

| 실행기가 기대한 것(V3) | ESP32 브리지 | 어댑터 / 실행기 |
| --- | --- | --- |
| `steer_mrad` = 등가 바퀴각(왼쪽 +) | `steer_mrad` = 펌웨어 명령 눈금(왼쪽 +, ±20 명령도 = ±349), 브리지가 `ANGLE = −도`로 보냄 | 등가 ↔ 명령 눈금을 `steer_esp32_mark_deg`(명령 20의 등가각)로 바꾼다: 명령도 = −등가도 × 20 / mark. SIM 과 같은 식(ABS_HOLD8_SIM 3.3절). mark 는 가장 새 현장 조향 기록의 값(2026-11-16 밤 18.8°, 추정. 3.2절), 각도계 측정(B3)이 생기면 새 기록으로 바꾼다. 측정각도 거꾸로 바꿔 실행기에 준다 |
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
값은 두 곳에서 온다. 차 치수(휠베이스, 소프트 한계, LiDAR, 몸체)는 프로필 `front_steer_v1_spec_20261116`, 보드 값(CAL, 명령 눈금, 조향 속도,
데드밴드, 극성·범위 확인 여부)은 **가장 새 현장 조향 기록**이다.

### 3.1 현장 조향 기록 (`sim/ubts/steering/field_records/`)

- 파일 하나가 현장 측정 한 번이다: `field_steer_<YYYYMMDD>_<HHMM>.yaml`, 스키마 `ubts.field_steer_record.v1`, 읽기·검사
  `sim/ubts/steering/field_steer_record.py`. 값마다 출처 태그(`MEASURED_<날짜>_LIFTED` 바퀴 든 정지 측정, `FIRMWARE`, `FIRMWARE_NVS` 보드 NVS
  저장 값, `USER_VISUAL` 사용자 눈 확인, `ESTIMATED`, `USER_DECISION`, `DEFINITION`, `NOT_MEASURED`)와 근거를 적고, 측정 조건(바퀴 든 채/땅,
  정지, 구동 상한, 가변저항 축전기)을 적는다.
- 버전 파일이다. 커밋한 기록은 고치지 않고, 다시 재면 새 파일을 더한다. 생성기는 `measured_at`이 가장 늦은 기록을 쓰고 옛 기록은 그대로
  둔다. 파일 이름의 날짜·시각은 `measured_at`(오프셋 포함)과 같아야 하고, 같은 순간의 기록 둘은 거절한다.
- 검사: 펌웨어 이름이 실행기가 기대하는 `ABS-HOLD-8-REBOUND-LIMITS`, 명령 눈금 ±20, CAL 이 RIGHT < CENTER < LEFT 이고 펌웨어 하드 한계와
  찾은 끝 안쪽, 전 범위(명령 0 ↔ ±20) 계단이 하나 이상, 확인 안 한 항목(false)은 `NOT_MEASURED`.
- 새 기록 반영: 기록 파일을 더한다 → `python -B -m steering.vehicle_executor_config --check ../../tools/row_executor/config.vehicle_esp32_shadow.json`
  (sim/ubts)이 다르다고 알린다 → `--out <새 파일>`로 쓴 것을 레포 파일과 바꾼다 → `python -B tools/row_executor/write_sha256sums.py` → 시험.
  `--sources`는 값마다 출처(기록 id·sha256 포함)를 보인다. `--no-field-record`는 프로필만으로 만든다(11/16 오전 판과 같은 바이트).
- 시험(`sim/ubts/test_steering_vehicle_executor_config.py`)은 기록 값을 글자로 박지 않는다: 설정 = 가장 새 기록의 값, 그리고 어느 기록에나
  서는 규칙(변화율 한도·GO 문턱 ≤ 0.8 × 측정 속도, 보드 명령 눈금으로도 같음; 극성·범위 확인 false; 구동 값 null; 펌웨어 감시 있음;
  소프트 한계가 브리지 범위 안). 기록이 극성(B0)이나 범위(B2/B3) 확인을 담으면 그 규칙 시험을 일부러 고친다.

### 3.2 첫 기록 `field_steer_20261116_2212` (2026-11-16 밤)

조건: 사용자가 차체를 받쳐 **앞바퀴를 띄움**, 정지, 조향만, 브리지 구동 PWM 상한 0(구동 명령 없음), 가변저항에 축전기를 단 뒤.
펌웨어 `ABS-HOLD-8-REBOUND-LIMITS F=RIGHT POT_DECREASES DRIVE_WD=300 CAL_RUNTIME=1`(실시간 보정판, 22:06 app0 에 씀, 사용자 허락).
원격(Jetson 브리지 보정·조향 요청)으로 하고 사용자가 차 옆에서 눈으로 봤다. **바퀴 든 정지 상태의 현장 측정이며 주행 결과가 아니다.**

| 무엇 | 값 | 어떻게 | 태그 |
| --- | --- | --- | --- |
| 왼쪽 끝 | 가변저항 2493 | 보정 끝 찾기(PWM 100 짧은 펄스) 1943 → 2493, 펄스 7번, 하드 한계 2500 앞에서 GUARD 로 멈춤. 사용자가 물리 최대라고 눈으로 확인 | MEASURED_20261116_LIFTED, USER_VISUAL |
| 오른쪽 끝 | 1439 | 끝 찾기 2501 → 1439, 펄스 14번, 되밀림(REBOUND)으로 멈춤 | MEASURED_20261116_LIFTED |
| CAL (보드 NVS) | RIGHT 1469 / CENTER 1900 / LEFT 2463, MAX_COMMAND 20 | 두 끝에서 30 count 안쪽을 `CAL SET`, `CAL SAVE`(STORE=NVS). 가운데는 그대로 | FIRMWARE_NVS |
| 곧은 바퀴 | 명령 0 에서 1891~1894 | 사용자가 앞바퀴가 곧다고 눈으로 확인(각도계 아님) | USER_VISUAL |
| 계단 응답 (명령도, + 왼쪽, 목표 1.5 명령도 안에 든 시각) | 0→+10 1.63 s, +10→+20 1.78 s, +20→0 3.22 s, 0→−10 1.45 s, −10→−20 1.29 s, −20→0 2.59 s. 끝 +19.66 / −19.89, 결함 0 | 브리지 조향 요청(throttle 0) | MEASURED_20261116_LIFTED |
| 가변저항 잡음 (정지) | 1851~1867 (p2p 16), 표준편차 3.0, 266 표본 / 30 s | 축전기 단 뒤, 모터 꺼짐, 브리지 상태의 raw(펌웨어 중앙값 필터 뒤) 약 10 Hz | MEASURED_20261116_LIFTED |
| 명령 20 의 등가각 (mark) | 18.8° | 가변저항이 선형이고 두 끝이 약 20° 라고 보면 20 × 431 / 461 = 18.70°(오른쪽), 20 × 563 / 593 = 18.99°(왼쪽)의 평균 | ESTIMATED |

### 3.3 설정 값 (생성기 출력)

| 값 | 설정 | 출처 |
| --- | --- | --- |
| 휠베이스 | 0.657 | 프로필 MEASURED_20261116 |
| 소프트 한계 | 281 mrad (16.14°, 명령 약 17.2 명령도) | 프로필 조향 한계 20°(측정) − 소프트 여유(CONFIG) |
| CAL | [1469, 1900, 2463, 20] | 기록 FIRMWARE_NVS. Arm 전 점검에서 브리지가 보고하는 CAL 줄과 같아야 한다 |
| mark | 18.8° | 기록 ESTIMATED |
| 측정 조향 속도 (주행·정지 둘 다) | 5.838 °/s (등가 바퀴각) | 18.8° / 3.22 s(가장 느린 전 범위 계단 +20 → 0), 0.001 내림. 명령 눈금으로는 20 / 3.22 = 6.21 명령도/s |
| 변화율 한도 (주행·정지) | 4.67 °/s | 0.8 × 측정 속도(사용자 규칙 todo 다-6, 내림). 보드에는 4.67 × 20 / 18.8 = 4.97 명령도/s ≤ 0.8 × 6.21 |
| GO 문턱 `steer_rate_min_deg_s` | 4.67 °/s | 같은 규칙. 측정 5.838 ≥ 4.67 이라 조향 속도 GO 는 통과한다(예전 자리값 8 은 버림) |
| 데드밴드 | 0.436° | 펌웨어 목표 허용 10 count ÷ 21.55 count/명령도(count 가 적은 오른쪽) × 18.8 / 20 |
| LiDAR | x 0.1185(측정), yaw 179.08(ESTIMATED) | 프로필 |
| 몸체 | 1.22 × 0.70 | SPEC_PENDING |

- 단위: 조정 메모의 "측정 6.2 °/s, 한도·문턱 4.9 °/s"는 **명령도** 기준이다. 실행기는 등가 바퀴각으로 명령하고 어댑터가 mark 로 명령도로
  바꾸므로, 같은 0.8 규칙을 보드에서 지키려면 등가각으로 바꿔야 한다(한도를 4.9 등가 °/s 로 두면 보드에서 5.21 명령도/s = 측정의 0.84 배).
  11/16 오전 생성기도 같은 식(mark / 계단 시간)으로 등가각 속도를 냈다. mark 가 바뀌면 속도·한도도 같이 바뀐다.
- 비워 둔 값과 아직 없는 것(그래서 MOTOR 는 설정 검사에서 거절, SHADOW 상태 'unknown'):
  - 극성 B0: 왼쪽 명령에 바퀴가 왼쪽으로 도는지 사용자가 아직 확인하지 않음 → `steer_polarity_checked` false.
  - 각도계 범위 B2/B3: mark 18.8° 는 추정이다. 20° 가 바퀴각이면(프로필 `steer_mech_limit_deg`의 해석, Ackermann 0.5 는 SPEC_PENDING) 등가각
    끝은 18.83°라 mark 는 약 17.7° 다. → `steer_range_measured` false, `steer_wheel_soft_margin_deg` null.
  - 0점 오차 F1: 눈 확인(1891~1894)뿐 → `steer_zero_offset_deg` null. 조향 시작 PWM B4 → `steer_min_pwm_pct` null.
  - 조향 속도의 조건: 바퀴를 띄운 정지 값을 주행 중 값에도 넣었다. 땅에 닿으면(특히 정지 중) 더 느릴 수 있다. 10 명령도 계단은 같은 식으로
    5.6~7.8 명령도/s 이고 +10 → +20(5.6)은 기준으로 쓴 전 범위 계단보다 느리다. 기준(전 범위 계단 가운데 가장 느린 것)은 정의다.
  - 구동 방향, C2 `breakaway_duty_pct`, C3 `v_measured_m_s`, C7 `measured_stop_distance_m`: 사용자가 앞뒤 구동을 금지해 하지 않음 → null,
    브리지 `direction_verified` false 라 Arm 도 거절된다.
- `steer_esp32_require_fw_watchdog` 는 true. 보드 펌웨어가 `DRIVE_WD=300`을 보고해 브리지 상태의 `drive.firmware_watchdog_ms` 가 300 이다.
- 보드 NVS 의 표시가 지워지거나 읽히지 않으면 펌웨어는 기본 표시 1681 / 1900 / 2180(STORE=DEFAULT)으로 돌아간다. 그때 CAL 줄이 설정과
  달라 Arm 은 `calibration_mismatch`로 거절된다(가짜 보드 시험). 보정을 다시 하면 새 기록과 새 설정이 필요하다.
- Jetson 의 설정은 이 변경으로 바뀌지 않는다. 설치는 대시보드 레포의 실행기 묶음 절차로 따로 한다.

## 4. 안전 규칙이 실차 보드에서 어떻게 지켜지나, 남는 것

| 위험 | V3 에서 막던 것 | ESP32 에서 막는 것 | 남는 것 |
| --- | --- | --- | --- |
| 실행기가 죽음·멈춤 | 브리지 입력 기한 0.75 s + MCU C TTL 300 ms | 브리지 입력 기한 0.75 s(→ STOP)와 브리지 자체 구동 TTL 0.75 s. 실행기 감시견(0.35 s)이 살아 있으면 구동 0 | MCU 쪽 300 ms TTL 이 없어 끊긴 뒤 멈춤까지 최대 약 0.75~0.85 s(가짜 보드: 0.85 s 한도 안) |
| 브리지가 죽음 | MCU watchdog 500 ms + C TTL | 브리지 SIGTERM/atexit 는 STOP(시험). SIGKILL·멈춤은 없음 | 원본 펌웨어는 마지막 구동을 계속했다. 2026-11-16 보드에 쓴 구동 감시(300 ms)가 멈춘다(실제 보드에서는 아직 재지 않음). Arm 기본은 감시 필요, 물리 비상정지는 그대로 마지막 수단 |
| 운영자 화면 끊김 | 실행기 heartbeat 1.0 s → 일시정지(구동 0), 재출발 없음 | 같다(시험: 1.15 s 안에 `2 S`/`3 S`, 다시 이어져도 구동 없음) | — |
| STOP | op stop → 다음 틱 STOP, 확인·재전송 | op stop → 브리지 다음 단계(20 ms)에 STOP, 확인될 때까지 다시 씀, 0.6 s 넘으면 `stop_unconfirmed` 래치 | — |
| 조향이 안 따라옴 | MCU TRACK + 호스트 래치 | 어댑터 추종 감시(3°, 2 s) | 문턱은 자리값, 차에서 재지 않음. 펄스 사이 되밀림(외란)은 SIM 에서도 넓은 조건 실패 원인(`STEER_FOLLOW_REALDIMS_RESULT_KO.md`) |
| 조향이 먹지 않음(미끄럼) | 호스트 래치(자이로) | 어댑터가 같은 규칙 | 차에서 재지 않음 |
| 가변저항 고장 | MCU POT_* 결함 | 브리지 `pot_invalid`(STATUS INVALID, 취소 INVALID POT) → 정지, 실행기 steering_feedback_invalid | 펌웨어의 고장 감지는 V3 보다 적다(급변·잡음 감시 없음) |
| 보드 리셋 | boot_id, n 역행 | 배너로 `mcu_reset_detected` 래치(브리지) | 리셋이 늘 배너를 내는지는 한 번만 봄 |
| 구동 방향·조향 눈금 틀림 | 보정 기록(cal_id, 극성) | Arm 에 방향 확인과 CAL 줄(현장 기록의 NVS 표시) 필요 | CAL 은 2026-11-16 밤 바퀴 든 보정으로 잡았다. 극성(B0)은 사용자 확인 전, 눈금(B3)은 추정(각도계 없음), 구동 방향은 안 함 |
| 실행기 없이 움직임 | SHADOW 는 소켓을 만들지 않음 | 같다 + Jetson SHADOW 설치는 `/run/ubts-rover`를 마운트하지 않음 | — |

## 5. Jetson 실행 환경

- 실행 이미지: Jetson 에 이미 있는 ROS Iron arm64 이미지 `sha256:a59689051b2e…`(native 관찰 서비스용; Python 3.10.12, numpy 1.21.5, scipy 1.8.0, rclpy). 센서 발행자(sllidar_node, BNO085 드라이버)도 Iron 이다. SIM 의 Jazzy 이미지는 쓰지 않는다(배포판 섞임을 피함; 실행기는 순수 Python 이라 Iron 에서 돈다). 새 이미지 빌드·pull 없음.
- 토픽(2026-11-16 읽기 점검): `/scan` 발행 RELIABLE, `/imu/data_raw` 발행 BEST_EFFORT. 실행기 구독은 BEST_EFFORT(깊이 2 / 50)라 둘 다 맞는다. `ROS_DOMAIN_ID` 0, network·IPC host.
- 설치 묶음: 대시보드 레포 `deploy/jetson_row_executor/`(점검 기본, `--apply`, `--rollback`, `--mode shadow|motor`). 운영자 토큰은 `--token-file`로 대시보드 runtime 폴더에 쓰고 `--no-print-token`으로 로그에 남기지 않는다. 대시보드는 연결 파일(`runtime/row_executor/dashboard_link.json`)을 읽는다.

## 6. 시험

| 시험 | 내용 | 환경 |
| --- | --- | --- |
| `tools/row_executor/tests/test_esp32_adapter.py` | 눈금·부호, 정규화, 정책(1 s 간격, 닿으면 다음 목표), motion 이 브리지에 가지 않음, 추종·미끄럼 감시와 stop 한 번, 다른 사람 lease 에는 stop 없음, STOP 확인 계수, Arm 전 점검 규칙 하나씩, V3 점검이 이 상태를 거절, 클라이언트 lease 잃음 → 래치, 설정 규칙, 기본 digest 그대로, 휠베이스 | 대시보드 없이 |
| `tests/test_esp32_executor_e2e.py` | 실행기(ExecutorLoop, MOTOR, one_press) → 어댑터 → **대시보드의 Esp32Bridge·Esp32SteerMotor 그대로** → 가짜 펌웨어(실시간 보정판, NVS 에 설정의 CAL 표시; 대시보드 1463c8b 이후), 한 가상 시계, JSON 왕복. go 전 구동 줄 없음(Start만, Arm 뒤 기다림), 방향 미확인·감시 없음·CAL 다름·NVS 표시 잃음(기본 표시) → Arm 거절(arm 안 보냄), NVS 표시 = 설정이면 Arm, 왼쪽 100 mrad → `ANGLE -6.0`, 가변저항 왼쪽, 측정 되돌림, 목표 변경 수, 화면 STOP → 40 ms 안 구동 0 · operator_stop · 래치 · 재출발 없음, heartbeat 끊김 → 1.15 s 안 구동 0 · 다시 이어져도 구동 없음, 실행기 멈춤 → 0.85 s 안 STOP, 가변저항 고착 → steering_not_following, INVALID POT → 정지 | `UBTS_DASH_DIR`=대시보드 레포 |
| `tests/test_esp32_executor_linux.py` | 실시간: 실행기 → 실제 Unix 소켓 → **브리지 프로세스**(`python -m scripts.esp32_steer_bridge`, 실제 USB 전송 코드, 가짜 udev) → pty 의 가짜 펌웨어. go 뒤 구동·왼쪽 ANGLE, 화면 STOP 0.2 s 안 구동 0, 실행기 멈춤 1.0 s 안 구동 0, 브리지 SIGTERM → 구동 0 · run 끝 | Linux + `UBTS_DASH_DIR` |
| `sim/ubts/test_steering_vehicle_executor_config.py` | 레포의 차량 설정 = 생성기 출력(가장 새 현장 기록), 보드 값 = 그 기록의 값, 고정 규칙(0.8 × 측정, 보드 눈금으로도; 극성·범위 false; 구동 null; 감시; 브리지 범위), 새 기록이 이기고 옛 기록은 남음, 같은 순간 거절, 잘못된 기록 18가지 거절, 속도 내림, 기록 없이 만들면 프로필 값, MOTOR 거절, SHADOW 루프에 전송 없음 | sim/ubts |

결과(2026-11-16, 이 브랜치): 7절.

## 7. 결과와 남은 일

결과와 Jetson 설치 상태는 작업 기록(`steering-sim-20261108/runs/jetson_autonomy_20261116/NOTES_KO.txt`)과 대시보드 레포
`deploy/jetson_row_executor/README_KO.md`에 적는다. 남은 일: 현장 첫 조향 시험(B0, B2/B3, B5, F1, 구동 방향), 구동 측정(C2, C3, C7), GO 문턱과 펌웨어
감시 결정, 그 뒤 MOTOR 설정(대시보드 레포 `docs/FIELD_STEER_AUTONOMY_TEST_KO.md`). 어댑터의 정책·감시 수치는 SIM 이 아닌 가짜 보드로만 확인했다(SIM T1 은 V3 링크 위의 abs_hold8 흉내라 이 어댑터를 지나지 않는다).

2026-11-16 밤 현장 조향 기록 반영(브랜치 field-steer-config-20261116, 작업 기록 `steering-sim-20261108/runs/field_steer_config_20261116/NOTES_KO.txt`):
- 시험: Windows Python 3.13 실행기 시험 전체 611 통과 35 건너뜀(ESP32 끝단은 대시보드 1463c8b 작업 트리로), WSL Python 3.10.12·numpy 1.21.5
  에서 ESP32 시험 34 통과(Linux 실시간 끝단 3 포함), sim/ubts 관련 시험 121 통과 1 건너뜀. 모두 가짜 보드이며 차에서 확인한 것이 아니다.
- SIM 은 바뀌지 않는다(SIM 은 현장 기록과 차량 설정을 읽지 않는다). main bde5474 와 이 브랜치에서 같은 잠금 세션으로: T1 안전 대본 둘 다 71/71,
  직렬·trace digest 71/71 같음. 빠른 SIM nominal(자리값 v0.1 5 run, 측정 치수 v1 5 run) 둘 다 5/5, 설정 digest·직렬·trace 5/5 같음, 실행기
  기록은 시간 칸(ms, loop_ms, scan.proc_ms)과 tuning.file(트리 경로)만 다름. 자리값 기반 SIM 결과이며 판정이 아니다.
- 남은 일: 극성 확인(B0), 각도계 범위(B2/B3)로 mark 확정, 땅에 닿은 조향 속도, 0점(F1), 구동 방향과 C2/C3/C7. 끝 찾기·유지 방식을 다시 하면
  새 기록 파일을 더하고 설정을 다시 만든다(3.1절). Jetson 의 실행기 설정은 따로 설치해야 바뀐다.
