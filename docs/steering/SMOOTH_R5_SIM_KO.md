# 차량 조향 펌웨어 SMOOTH r5 흉내와 현장 프로필 (2026-11-17)

- 작성: 2026-11-17 (작업 sim-smooth-model). 계획의 방향 "측정한 차를 SIM 에"(승인됨).
- 차 조향 보드(ESP32-S3, BTS7960 3개)의 펌웨어가 2026-11-16 밤 펄스 유지(`ABS-HOLD-8-REBOUND-LIMITS`, [`ABS_HOLD8_SIM_KO.md`](ABS_HOLD8_SIM_KO.md))에서
  **SMOOTH 유지 r5**(같은 소스의 `SMOOTH=1`, SM MODE=SMOOTH 를 보드 NVS 에 저장)로 바뀌었다. 이 문서는 그 유지 법칙을 SIM 의 새 펌웨어 모형
  `steer_fw_mode smooth_r5` 와 새 프로필 `front_steer_v2_field_20261116` 으로 넣은 것을 적는다.
- 옛 프로필(v0.x, `front_steer_v1_spec_20261116`)과 기본 프로필(v0.1), 그 결과는 그대로다(7절의 동일성 확인). 사용자 화면용 Gazebo 스택
  (`compose.ubts-steer.measured.yaml`)은 아직 v1 을 고른다. 바꾸는 것은 따로 정한다.
- 이 프로필로 만든 수치는 **자리값 기반 SIM 결과**다. 현장 성능이 아니다.

## 1. 근거

| 무엇 | 출처 | 날짜 |
|---|---|---|
| r5 소스 `BTS7960_Serial_Control.ino`(CAL_RUNTIME=1, SMOOTH=1, 앱 sha256 f448b2db…) | 레포 밖 기록 `runs/steer_smooth_20261116/fw/build_r5/source_r5.ino`(make_smooth.py + r2/r4/r5 바꿈) | 2026-11-16 23:20 |
| 보드에 저장된 값: CAL 1475 / 2017 / 2513, SM PWM_MIN 60 PWM_MAX 80 SLOW 200 STOP_TOL 6 RESTART_TOL 16 RAMP 12 SETTLE_MS 120 STALL_MS 200 BOOST 15 MAX_RUN_MS 5000 BRAKE_MS 40 QUIET_MS 15 QUIET_US 1500 | 현장 기록 [`field_steer_20261116_2321.yaml`](../../sim/ubts/steering/field_records/field_steer_20261116_2321.yaml) | 2026-11-16 |
| r5 계단 응답(바퀴 든 정지 상태, 한 번의 연속 구동 시간) | 같은 기록 `step_response` | 2026-11-16 23:21 |
| 호스트 규칙: 0.25 명령도, 200 ms, 취소된 목표 다시 안 보냄 | 대시보드 da39e8a 브리지, 실행기 어댑터 dad6a93(`EXECUTOR_ESP32_KO.md`) | 2026-11-16~17 |

## 2. SIM 안에서의 자리

- 펄스 유지와 같은 틀이다: 펌웨어 V3(`firmware/ubts_steer_v3`)의 조향 법칙 걸이(`fw_steer.h steer_law_hook`)에 `sim/ubts/steering/native/smooth_r5.cc`
  를 끼운다. 시작 줄 블록 `steer_law` 의 `kind` 가 `smooth_r5` 일 때만이다(`steer_config.cc parse_smooth_law`, `fw_session.cc`). V3 의 직렬 규약·
  ARM·TTL·watchdog·STOP·구동·결함 감시는 그대로다. V3 펌웨어 소스는 바꾸지 않았다(빌드 id 878B5F56 그대로).
- 걸이가 돌려주는 것: 출력(−255..255, + 왼쪽), `cmd`(보낸 ANGLE 의 permille, 상태 줄 `cmd`), `release`(1 = EN 끔, 0 = EN 켬·PWM 0 = 제동).
- 펌웨어 법칙의 백엔드는 네이티브 하나다(Python 펌웨어 모형은 없다). T1 리그(빠른 SIM)와 Gazebo ESP 에뮬레이터(`gz_esp_emulator.py`)가 같은
  네이티브 세션을 쓴다. Gazebo 에서 이 프로필로 돌린 적은 없다.
- 파이썬 쪽: `steering/smooth_hold.py`(블록, 목표 count, 실행기 값), `vehicle_profile.py` 스키마 v5, `native/lockstep_v2.firmware_startup_config`,
  적합성 실행기 `startup_config`(법칙 블록이 있는 프로필만; V3 대본 92개의 시작 줄은 같다).

## 3. 펌웨어에서 그대로 옮긴 것

(괄호는 r5 소스의 함수. count 는 보드 12비트 ADC, + 왼쪽 = 값이 커짐, F = 오른쪽.)

| 규칙 | 값 |
|---|---|
| 가변저항(`readPotValue`) | 5 ms 마다 표본, 5개 중앙값, 두 중앙값이 20 안이면 평균 확정, r4 변화율 제한 max(10, 2 × 지난 ms) count, 100 ms 넘게 확정 없으면 무효. 유효 창 1200 < v < 2700(하드 한계 1300/2600 ± 100) |
| 목표(`moveSteeringToAngle`) | CENTER + lroundf(span × \|a\| / 20.0f)(float), 같은 목표 무시. 구동 중 같은 방향·STOP_TOL 보다 먼 새 목표는 멈추지 않고 이어받음(구동 횟수 1, 구동 시간 한도 다시 시작). 그 밖은 멈추고 새 목표 |
| 구동(`updateSmoothHold` 상태 1) | 위치 = 조용한 읽기(QUIET_MS 마다 드라이버 끔, 변화율 제한). STOP_TOL 안이나 지나치면 끝. CAL 표지 + 15 넘으면 끝. 10 ms 마다 PWM = 바닥 + (PWM_MAX − 바닥) × min(1, (남은 − STOP_TOL)/SLOW), 오를 때 RAMP 까지. 진척 4 count 없이 STALL_MS 면 바닥 + BOOST, 이미 PWM_MAX 면 STALL 취소. MAX_RUN_MS 면 취소 |
| 끝(`smEndRun`, 상태 2) | BRAKE_MS 동안 EN 켬·PWM 0(아래쪽 스위치 둘 = 단락 제동), 그다음 EN 끔. 처음 100 ms 되튐 정점, SETTLE_MS 뒤 판정: 오차 > RESTART_TOL 이고 되튐 ≥ 20 이면 REBOUND 취소 |
| 쉼(상태 0) | 오차 ≤ STOP_TOL, 또는 닿은 뒤 ≤ RESTART_TOL 이면 유지(출력 없음). 4번 구동해도 못 닿으면 NO CONVERGENCE. 방향 바꿈은 드라이버가 꺼진 뒤 60 ms 쉼 |
| 취소(`smCancel`, `stopMotor`) | 모든 취소는 '다시 밀지 않음'. 무효 가변저항도 취소 |

## 4. SIM 에서 다른 것과 자리값

- 판정 주기: 실기는 루프마다, SIM 은 V3 서보 격자다. 새 프로필은 `servo_hz` 1000(1 ms)이라 5 ms 표본·10 ms 램프·15 ms 조용한 주기가 실기 시각에
  떨어진다.
- 조용한 읽기: 실기는 같은 루프 안에서 EN 을 1.5 ms 끄고 다섯 번 읽은 중앙값. SIM 은 서보 한 주기(1 ms) EN 을 끄고 다음 주기의 ADC 값 하나를
  쓴다(16 ms 마다 1 ms 끔). 그동안 플랜트의 모터는 실제로 놓인다.
- ADC: SIM 플랜트는 V3 의 10비트 ADC 모형이라, 걸이가 V3 보정 표지를 거쳐 12비트 count 로 옮긴다(펄스 유지와 같다). 이 보정에서 1 raw 는 약
  7~8 count 로 STOP_TOL 6 보다 거칠고, 잡음 sd 1 raw(자리값)가 약 7 count 다.
- SIM ADC 경로 자리값 `smooth_adc_sim`(기본 0 = 끔): 드라이버가 PWM 으로 켜져 있던 동안의 ADC 값에만 움직임 반대쪽 어긋남 `powered_offset_counts`
  와 전류 묶음 `burst_*` 을 더한다. 현장에서 모터 켬 잡음이 시간에 따라 커졌으나(원값 40 count 넘게 어긋남 0.4 → 19.5 %, r5 시험 때 2.1 %)
  크기·부호·전류 관계는 재지 않았다. 적합성 대본에서만 켠다(50 count, 110/20/170 ms: 호스트 시험 모형).
- 호스트: 실기는 어댑터 + 브리지 두 겹, SIM 은 걸이 안의 호스트 정책 하나(`smooth_host`: 200 ms 마다 0.25 명령도로 0 에서 먼 쪽 반올림, 바뀌면 보냄,
  취소된 목표는 요청이 바뀔 때까지 안 보냄). 브리지의 '첫 목표가 지금 자리면 건너뜀'은 넣지 않았다.
- 실기 보드에 없는 V3 감시는 남는다: 목표에 1 s 넘게 못 닿으면 V3 TRACK(실행기의 `steering_not_following` 은 2 s), 빠른 가변저항 변화의 POT_NOISE·
  POT_JUMP. SIM 은 결함이 더 날 수만 있다.
- 흉내 내지 않은 것: 수동 조향(`1 F 100`)과 1000 ms 수동 시간 한도, 한계 확인 횟수(수동 조향 전용), CAL FIND, TRACE, SM SET/SAVE, 펄스 유지로의
  실행 중 전환.
- 플랜트(모터): L1 식 `w = rate (u − min)/(1 − min)` 의 rate 를 r5 계단 8개에 맞췄다(80 °/s, 5절). 시작 PWM 10 %, 유격 2°, 되밀림, 가운데로 미는
  토크 없음, 땅에 닿은 조향·주행 중 조향 값은 v1 자리값 그대로다.

## 5. 새 프로필 `front_steer_v2_field_20261116` (스키마 v5)

v1 파일의 바이트에 아래만 바꿨다(시험 `test_v2_differs_from_v1_only_in_these_records`). 만든 스크립트는 레포 밖 `runs/sim_smooth_model_20261117/scripts/make_profile_v2.py`.
`apply_spec.py` 의 사양 반영 경로(v0.3 바탕)로 만든 판이 아니다.

| 칸 | 값 | provenance |
|---|---|---|
| `steer_fw_mode` | `smooth_r5`(선택지 v3_servo, abs_hold_8, smooth_r5) | CONFIG |
| `smooth_fw` | CAL 1475/2017/2513, SM 값 13개(1절), 상수(진척 4, 구동 4번, 방향 바꿈 60 ms, 램프 틱 10 ms, 되튐 20, 되튐 창 100, 가드 15, 표본 5 ms, 중앙값 5, 짝 20, 신선도 100, 변화율 10·2/ms, 하드 한계 1300/2600, 여유 100) | FIRMWARE |
| `smooth_host` | step 0.25°, deadband 0.25°, 간격 200 ms, poll 200 ms | CONFIG |
| `smooth_adc_sim` | 모두 0(끔) | SIM_ASSUMED |
| `smooth_step_time_measured_s` | outward_20 1.439 s, return_20 1.087 s | MEASURED_20261116(바퀴 듦) |
| `abs_hold_fw` | 보정만 1475/2017/2513(펄스 상수 그대로; SM OFF 비교용) | FIRMWARE |
| `servo_hz` | 1000 | CONFIG |
| `steer_rate_max_deg_s`, `..._standstill` | 80(v1 100) | ESTIMATED |

모터 속도 맞춤(T1 리그, 서 있는 차, 같은 순서의 계단; 20 대신 소프트 한계 약 18 명령도; 첫 연속 구동 시간):

| 계단(+ 왼쪽) | 현장 r5 | SIM rate 80 |
|---|---|---|
| 0 → 20 | 1.439 s | 1.039 s(18.1 명령도) |
| 20 → −20 | 2.127 s | 1.887 s(36.1) |
| −20 → 20 | 2.431 s | 1.935 s(36.1) |
| 20 → 0 | 1.087 s | 1.007 s(18.1) |
| 0 → −10, −10 → 0, 0 → 10, 10 → 0 | 0.591, 0.703, 0.543, 0.655 s | 0.607, 0.591, 0.639, 0.623 s |

명령도당 시간으로 견주면 rate 60/70/75/80/90/100 가운데 80 이 가장 가깝다(평균 −4 %, rms 12 %). 맞추지 못한 것: 현장은 9 움직임 모두 한 번에 끝났고
(보정 구동 0) SIM 은 8 계단 가운데 2번 뒤에 15~47 ms 짧은 보정 구동이 났다(10비트 분해능과 잡음 자리값). 가운데로 돌아오면 바퀴가 0.9~1.1° 남는다
(유격 2° 자리값의 절반). 첫 움직임이 늦게 보이는 0.17 s 는 호스트 poll(200 ms)이다.

실행기 '측정' 조향 값(`smooth_hold.executor_values`, `abs_hold.executor_values` 로 부른다): 조향 속도 = 보정 표지 각 17.83° / 1.439 s = 12.39 °/s
(0.01 아래로 내림: 실행기의 명령 변화율 한도 round(0.8 × 속도, 3)가 0.8 × 속도를 넘지 않게), 데드밴드 = STOP_TOL 6 count(왼쪽, 1 count 가 큰 쪽)
= 0.216°, 다시 시작 0.575°. 조향 GO 규칙(8 °/s)을 넘으므로 v1 처럼 문턱을 바꿀 필요가 없다.

## 6. 적합성 대본 (`sim/ubts/steering/conformance/scripts_smooth_r5/`, 11개)

V3 대본 92개와 같은 형식(`ubts.steer_conformance.v1`)이고, 대본 ADC(`adc_set`)로 가변저항 값을 정해 핀(`steer_pwm`, `steer_en`)과 상태 줄 `cmd`
를 틱 단위로 본다. 기대 틱은 .ino 글을 보고 따로 쓴 보조 계산(레포 밖 `runs/sim_smooth_model_20261117/scripts/smooth_ref.py`, 생성기
`gen_smooth_conformance.py`)이 냈다. 같은 작업이 모형도 썼으므로 '규격만 보고 따로 쓴 대본'은 아니다.

| 대본 | 보는 것 |
|---|---|
| `smooth_run_ramp_quiet_brake` | 출발 PWM 60, 10 ms 마다 +12(72, 80), 16 ms 마다 1 틱 EN 끔, SLOW 구역에서 바닥 60 쪽으로, STOP_TOL 안에서 제동 40 ms 뒤 EN 끔, 정착 뒤 출력 없음 |
| `smooth_takeover_same_direction` | 구동 중 같은 방향 새 목표: PWM 80 그대로(멈추고 60 으로 다시 출발하지 않음) |
| `smooth_reversal_pause` | 반대 방향 목표: EN 끄고 60 ms 쉰 뒤 −60 으로 출발, 그 뒤 STALL |
| `smooth_stall_cancel_no_retry` | 막힌 바퀴: PWM_MAX 에서 STALL 취소, 호스트가 같은 요청을 다시 안 보냄, 요청이 바뀌면 다시 출발 |
| `smooth_floor_boost_heavy` | SLOW 구역 PWM 67 로 안 움직임 → 200 ms 뒤 바닥 75, PWM 77 → 움직여 목표 |
| `smooth_powered_offset_quiet_on` / `_off` | 50 count 어긋남: QUIET 15 면 제 틱에 끝, QUIET 0 이면 EN 이 계속 켜지고 목표를 못 봐 바닥 올림 뒤 STALL |
| `smooth_current_bursts` | 110 count 묶음(20/170 ms): 거짓 취소 없이 제 틱에 끝 |
| `smooth_brake_off` | BRAKE_MS 0: 끝에서 바로 EN 끔 |
| `smooth_drift_correction` | 닿은 뒤 10 count 흐름은 구동 없음, 41 count 밀림은 같은 방향 보정 구동 |
| `smooth_soft_limit_clamp` | 요청 1000 ‰ → V3 소프트 한계 935 → −18.75(cmd 938) |

```
cd sim/ubts
python -B -m steering.conformance.run_conformance --scripts-dir steering/conformance/scripts_smooth_r5
python -B -m steering.conformance.run_conformance --scripts-dir steering/conformance/scripts_smooth_r5 --transport backend
python -B -m pytest -q -p no:cacheprovider test_steering_smooth_hold.py test_steering_smooth_conformance.py
```

## 7. 확인 결과

- 2026-11-17 뒤: v1 넓은 조건의 실패 분석, 차의 감시 묶음을 맞춘 v2.1(스키마 v6)과 SIM 제안 r6p(v2.2)는
  [`SMOOTH_R5_DISTURBANCE_KO.md`](SMOOTH_R5_DISTURBANCE_KO.md), 결정 R103.

[`STEER_FOLLOW_SMOOTH_RESULT_KO.md`](STEER_FOLLOW_SMOOTH_RESULT_KO.md)(사전등록 [`PREREG_STEER_FOLLOW_SMOOTH_KO.md`](PREREG_STEER_FOLLOW_SMOOTH_KO.md)):
열 추종 `matrix` 15/15, 넓은 조건 52/60(외란 두 조건의 V3 감시와 운영자 연결 기준; v1 은 51/60), 실기 r5 소스와의 차이 시험에서 조용한 읽기를 끄면
핀 단위로 같음, 옛 프로필 동일성(T1 71, nominal 두 프로필의 trace 같음). 모두 자리값 기반 SIM 결과다.
