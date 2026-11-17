# SMOOTH r5 넓은 조건 실패 분석, 차의 감시 묶음 맞춤(v2.1)과 SIM 제안 r6p(v2.2)

- 작성: 2026-11-17 (작업 sim-smooth-wide). 앞 문서: [`SMOOTH_R5_SIM_KO.md`](SMOOTH_R5_SIM_KO.md), 결과 [`STEER_FOLLOW_SMOOTH_RESULT_KO.md`](STEER_FOLLOW_SMOOTH_RESULT_KO.md)(v1).
- 모든 수치는 **자리값 기반 SIM 결과**다. 현장 성능이 아니다. 조향 모터 정지 토크(5 N·m), 외란 크기, 되밀림 배율, 유격은 자리값이다.
- **제안 r6p 는 SIM 제안이다. 차의 펌웨어가 아니며 보드에 쓴 적이 없다.**
- 기록 폴더(레포 밖): `steering-sim-20261108/runs/sim_smooth_wide_20261117/`(NOTES_KO.txt, scripts/fault_window.py, measure/dev_*).

## 1. 요약

| 실패(v1 `wide`) | 멈춘 직접 이유 | 실기 스택에 그 감시가 있나 | 밑의 원인 | 판정 |
|---|---|---|---|---|
| `dist_step_p2p5` 0/3 | V3 POT_NOISE → MCU 무장 풀림 → `steering_feedback_invalid` | 없다(r5 보드에 결함 감시 없음) | SMOOTH 유지가 PWM_MAX 80(31 %)으로 2.5 N·m(등가 duty 50 %)를 못 버티고 STALL 취소로 바퀴를 놓음 → 바퀴가 끝까지 감 | 모형 차이 + **유지 법칙 자체의 행동** |
| `dist_terrain` 1/3 | V3 TRACK → `steering_not_following` | 없다(실행기 어댑터의 3 deg / 2 s 규칙은 있다) | 지면 밀기·충격에 PWM 80 으로 못 버팀, STALL·REBOUND 취소 뒤 바퀴가 풀림 | 모형 차이 + **유지 법칙 자체의 행동** |
| `operator_link_lost` 0/3 | 등록 기준 0.55 s 대비 1.05 s | 해당 없음 | 기준이 결정 R60(2026-11-13, 연결 기한 1.0 s) 전 값 | **등록 기준이 낡음**(조향과 무관) |

- 고침 1(모형): 새 프로필 `front_steer_v2_1_field_20261117`(스키마 v6)이 차의 감시 묶음(`steer_monitor_stack esp32_r5`)을 흉내 낸다(3절).
- 고침 2(SIM 제안): `front_steer_v2_2_proposal_20261117` 이 SMOOTH 유지에 외력 대응 네 가지를 더한다(제안 r6p, 4절).
- 개발 run(판정 아님, seed 100~104): v2.1 에서 `dist_step_p2p5` 0/5, `dist_terrain` 1/5(모두 `steering_not_following`), v2.2 에서 10/10(5절).
- 판정은 새 사전등록 `steer_follow_smooth_r5_v2`(판정 run 전에 커밋, [`PREREG_STEER_FOLLOW_SMOOTH_V2_KO.md`](PREREG_STEER_FOLLOW_SMOOTH_V2_KO.md)). v1 결과는 그대로 둔다.

## 2. v1 실패 run 의 기록 (새 run 없이 기록만 봄)

기록: `runs/sim_smooth_model_20261117/measure/smooth_wide/runs/<run>/`(SERIAL.jsonl 의 상태 줄 `S`, TRACE.npz). pos·cmd 는 V3 ‰(약 56 ‰/deg),
sp 는 조향 핀 PWM(−: 오른쪽), 바퀴 각은 참값(truth_delta_eq).

### 2.1 `dist_step_p2p5`(6.0 s 부터 +2.5 N·m 3 s, 왼쪽으로 미는 토크)

| run | 6.0 s 뒤 | 출력이 0 이 된 시각 | 그 뒤 | 결함 줄 |
|---|---|---|---|---|
| smooth_wide_dist_step_p2p5_s4000 | sp −66 → −80(PWM_MAX)로 오른쪽으로 미는데 pos −44 → 574 ‰ | 6.47 s 상태 줄 sp 0 | pos 868(6.52), 1103 ‰(왼쪽 끝) | `F POT_NOISE 2600 6485` |
| smooth_wide_dist_step_p2p5_s4001 | sp −60 → −80, 바퀴 0.25 → 10.4°(6.43 s) | 6.43 s 무렵(트레이스 PWM 0) | 14.3°(6.475), 18.8°(끝) | `F POT_NOISE 2520 6488` |
| smooth_wide_dist_step_p2p5_s4002 | sp −60 → −80, 바퀴 0.1 → 9.9°(6.43 s) | 6.43 s 무렵 | 13.8°(6.475), 18.8° | `F POT_NOISE 2597 6490` |

- PWM 80 으로 미는 동안에도 바퀴는 약 14 °/s 로 밀렸다. r5 규칙으로는 PWM_MAX 에 닿은 뒤 200 ms(STALL_MS) 진척이 없으면 **STALL 취소**(no force
  retry)다. 시각이 그와 맞는다(구동 시작 약 6.03 s, PWM 80 도달 약 6.23 s, 출력 0 약 6.43 s).
- 출력이 끊긴 뒤 바퀴는 약 80 °/s 로 끝까지 갔고(드라이버 꺼짐 = 되밀림 배율 3), 그 빠른 raw 변화가 V3 POT_NOISE(50 ms 창 분산 > (5.0 raw)²,
  인자 2520~2600 > 2500)를 켰다. MCU 가 무장을 풀고 실행기는 `steering_feedback_invalid` 로 6.65 s 에 멈췄다(약 1.3 m 주행).
- 외란 모형: 외력 / 조향 모터 정지 토크(`steer_motor_stall_torque_nm` 5 N·m, SPEC_PENDING) = 등가 duty([`STEER_EXT_MODEL_KO.md`](STEER_EXT_MODEL_KO.md)).
  2.5 N·m = 50 %, SMOOTH 의 PWM_MAX 80/255 = 31 %. 이 자리값에서 r5 의 SMOOTH 유지가 버틸 수 있는 조향축 외력은 약 1.6 N·m 아래다.

### 2.2 `dist_terrain`(지면 충격 1 /m·sd 1 N·m 50 ms + 밀기 sd 0.5 N·m, 상관 길이 2 m)

- smooth_wide_dist_terrain_s4001: 6.2~6.7 s 목표 −3.6° 를 PWM 80 으로 붙잡는 동안 pos −88 → −162 ‰ 로 거의 못 감, 6.72 s 출력 0(STALL), 바퀴가
  0.4 s 만에 +11° 로 밀림. 호스트가 바뀐 목표를 보낼 때마다(7.12, 7.52 s) PWM 65~80 구동이 약 200 ms 뒤 다시 끊김. 바퀴는 왼쪽 끝 18.8°.
  7.79 s `F TRACK -1524`(cmd − pos = −1524 ‰, 약 −27°) → `steering_not_following`.
- smooth_wide_dist_terrain_s4002: 같은 모양으로 13.46 s `TRACK`. s4000 은 통과(밀기가 작았다).

### 2.3 `operator_link_lost`

- 세 run 모두: 마지막 화면 신호 19.95 s, 구동 0 요청 21.00 s(실행기 기록 `EXECUTOR.jsonl`, 21.00 s 에 PAUSED `operator_link_lost`). 실행기 설정
  `operator_timeout_s` 1.0, `control_hz` 20 → 1.05 s = 기한 1.0 s + 한 틱 0.05 s. 트레이스의 구동 0 은 1.06 s.
- 등록 기준 `link_drive_zero_s` 0.55 s 는 `steer_follow_wide_v2`(기한 0.5 s 때)의 값이다. 결정 R60(2026-11-13)이 기한을 1.0 s 로 바꾸고 합격 기준 A17
  개정 60 이 "구동 0 ≤ `operator_timeout_s` + 1틱"으로 바꿨다. 펄스 유지 v1(`STEER_FOLLOW_REALDIMS_RESULT_KO.md`)도 같은 1.05 s 로 실패했고, 그 문서가 이미
  "다음 사전등록에서 R60 에 맞춘다"고 적었다. 조향 펌웨어와 상관없다.

## 3. 차의 감시 묶음과 SIM 의 차이, 그리고 v2.1

| 감시 | 차(r5 보드 + ESP32 브리지 + 실행기 ESP32 어댑터) | SIM v2(스키마 v5) | SIM v2.1(`steer_monitor_stack esp32_r5`) |
|---|---|---|---|
| POT_JUMP, POT_NOISE, POLARITY, DRV_STUCK, STALL(MCU), TRACK(MCU), THERMAL | 없다(r5 소스 `source_r5.ino`: 결함 비트 없음) | V3 펌웨어가 돈다 | CFG 값을 닿을 수 없는 끝으로(`fw_eeprom.BOARD_R5_MONITORS_OFF`) |
| POT_OPEN / SHORT | 무효 가변저항 → 유지 취소, 브리지에 유효 각 없음 | V3 결함 → 무장 풀림 | 그대로(둘 다 멈춤) |
| 유지 취소(STALL, REBOUND, NO CONVERGENCE, RUN TIME LIMIT, QUIET OPERATING LIMIT, ADC STALE/INVALID) | 있다(결함 아님, 'no force retry') | `smooth_r5.cc` 가 흉내 | 그대로 |
| `steering_not_following` | 어댑터: \|보낸 목표 − 측정\| > 3 deg 가 2 s(`ESP32_TRACK`), 램프 조건 없음 | V3 호스트: \|요청 − 측정\| > 3 deg 가 3 s, MCU 램프가 목표에 닿은 상태 줄만 셈 | 호스트 프로필 `track_rule esp32_adapter`: 상태 줄 cmd(보낸 목표)와 측정의 차, 램프 조건 없음, 3 deg / 2 s |
| `steering_ineffective` | 어댑터: V3 규칙과 같은 값 | V3 호스트 규칙 | 그대로 |

- v2 의 호스트 규칙은 SMOOTH 에서 거의 세지 않는다: 걸이의 cmd(보낸 ANGLE, 0.25 명령도로 맞춤)와 V3 tgt 가 대부분 달라 '램프가 닿음' 조건이 서지 않는다.
  곧 v2 는 MCU 쪽은 실기보다 엄하고 호스트 쪽은 실기보다 무르다. v2.1 은 둘 다 실기 쪽으로 맞춘다.
- V3 펌웨어 소스는 바꾸지 않았다(빌드 id 878B5F56). 끄는 값: POT_JUMP 1023 raw(0..1023 범위라 넘을 수 없음), POT_NOISE sd 100 raw(조향 전체가 약 140 raw),
  POLARITY·DRV_STUCK 1 ms 안 1000 ‰, STALL PWM 255 와 움직임 < 0, TRACK 2000 ‰ 10 s(호스트 2 s 가 먼저), THERMAL PWM 255 로 60 s 전부.
- 옛 프로필(v0.x, v1, v2)은 호스트 프로필 바이트, 시작 줄 법칙 블록, CFG 값이 main 54f9c34 와 같다(시험 `test_steering_smooth_monitors.py`).

## 4. SIM 제안 r6p (`steer_fw_mode smooth_r6p`, 프로필 v2.2) — 차의 펌웨어가 아니다

r5 규칙에 조향축 외력 대응 네 가지를 더했다(네이티브 `smooth_r5.cc` 의 kind `smooth_r6p`; kind `smooth_r5` 는 비트 단위로 그대로).

| 바꿈 | r5 | 제안 r6p(값은 SIM 자리값) | 겨냥한 실패 |
|---|---|---|---|
| `hold_pwm_max` 200 | STALL 바닥 올림과 STALL 취소는 PWM_MAX(80)에서 | 바닥 올림의 상한과 STALL 취소가 200(78 %)에서. 바닥이 PWM_MAX 위면 그 바닥이 PWM(감속 구역에서 바닥 아래로 내리지 않음) | 힘 부족(2.1) |
| `push_counts` 24 | 없음 | 구동 중 진척 기준에서 24 count(약 1°) 거꾸로 밀리면(막힘이 아니라 밀림) 바닥을 곧바로 200 으로(램프 규칙은 그대로) | 밀리는 동안 200 ms 씩 기다리는 바닥 올림 |
| `rest_brake` 1 | 끝 제동 BRAKE_MS(40 ms) 뒤 EN 끔 | 목표를 쥔 동안(정착·쉼) 단락 제동 유지(EN 켬, PWM 0) | 놓인 바퀴가 3배 빨리 밀림 |
| `hold_no_cancel` 1 | 정착 뒤 되튐 ≥ 20 이면 REBOUND 취소, 4번 못 닿으면 NO CONVERGENCE 취소(다시 안 밂) | 목표에 한 번 닿은 뒤에는 둘 다 취소하지 않고 보정 구동 | 밀린 뒤 풀린 채 남음(2.1, 2.2) |

- 남는 안전 장치: PWM 200 에서도 진척·밀림이 없으면(막힘) STALL 취소, 한 구동 5 s(RUN TIME LIMIT), 무효 가변저항, CAL 표지 가드, 호스트의 3 deg / 2 s.
- 한계와 위험(현장에서 따져야 할 것):
  - PWM 200 은 r5 의 SM_LIMITS(PWM_MAX 40..160) 밖이다. 모터·드라이버 전류와 열, 벨트 미끄럼을 재지 않았다. 필요한 값은 실제 정지 토크(B5)와 땅에 닿은
    조향의 외력에 달렸다.
  - REBOUND 취소는 현장에서 링크 되튐(오른쪽 최대 119 count)과 싸우지 않으려고 넣은 규칙이다. 제안은 닿은 목표에서 그 취소를 없애므로 끝 부근에서 되튐과
    보정이 번갈아 날 수 있다(SIM 에 되튐 모형은 자리값).
  - 서 있는 차(T1, 2.5 N·m 3 s): 바퀴가 끝까지 가지 않지만 1.2~6.5° 사이를 약 0.9 s 주기로 오간다(구동이 목표에 닿아 제동·정착하는 동안 다시 밀림).
    호스트 규칙은 3° 아래로 자주 돌아와 걸리지 않는다. 고칠 거리: 밀린 기억이 있으면 보정 구동을 바닥 200 에서 시작, 정착 중 밀림에 바로 보정.
  - 정착·쉼 제동은 전류가 없는 단락 제동이지만, 실기 BTS7960 에서 EN 을 계속 켜 두는 것의 부작용(발열, 수동 조향 감)은 보지 않았다.

## 5. 개발 run (판정 아님, seed 100~104; 판정 seed 4000~4002 는 쓰지 않음)

| 프로필 | 무엇 | 결과 |
|---|---|---|
| v2 그대로 | `dist_terrain` 5 | 1/5(TRACK 3, guard_heading 1) |
| v2 + V3 감시만 끔(호스트는 V3 램프 조건 규칙 그대로) | `dist_step_p2p5` 5, `dist_terrain` 5 | 0/5(모두 guard_heading 25°), 2/5(guard_heading 2, obstacle_ahead 1: 횡 1.16 m) |
| v2 + 감시 끔 + PWM_MAX 160(r5 의 SM 상한) | 같은 10 | 0/5(row_end 3 이지만 방향 RMSE·최대 넘음, guard_heading 2), 3/5 |
| **v2.1**(차의 감시 묶음) | 같은 10 | **0/5**(모두 `steering_not_following`, 외란 뒤 약 2.1~2.2 s), **1/5**(`steering_not_following` 4: 하나는 출발 전 밀기로 끝까지 감) |
| **v2.2**(제안 r6p 200/24/1/1) | 같은 10 | **10/10**(열 끝, 횡 RMSE 0.035~0.094 m, 방향 최대 2.6~14.9°) |
| v2.2 | matrix 6조건(nominal, heading_p8, side_pull, steer_zero_n2, lidar_stale_1p0, obstacle) + wide 5조건(curve_l30, speed_0p5, w2_042301_c03, operator_link_lost, imu_stale_1p0) × seed 100, 101 | 22 가운데 20 통과, `operator_link_lost` 2 는 개발 스크립트가 v1 기준 0.55 s 로 판정해서(1.05 s) |

- 제안 값(200/24/1/1)은 처음 고른 값 그대로다. 개발 run 을 보고 고치지 않았다.
- 같은 묶음을 v2.1 로 돌리던 개발 run 은 다른 작업의 무거운 측정(잠금)과 겹치지 않게 도중에 멈췄다(판정하지 않음). 판정 run 이 대신한다.
- 결론: v1 의 두 외란 실패는 '실기에 없는 감시가 멈춘 것'만이 아니다. 실기 감시 묶음(v2.1)에서도 SMOOTH r5 는 이 자리값 외란을 못 버티고 실행기 어댑터 규칙으로
  멈춘다. SIM 에서는 제안 r6p 가 같은 외란을 버텼다(서 있는 차의 떨림은 남음). 실기 판단에는 정지 토크·땅에 닿은 조향·전류의 측정이 먼저다.
