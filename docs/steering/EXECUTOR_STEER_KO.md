# 앞바퀴 조향 실행기와 T1 실행기 루프 (설계 8.3 P4: p4_exec)

- 작성: 2026-11-09 (3단계 WF-3a, 작업 p4_exec. 같은 날 검토 처리 fix_wf3a, 11절)
- 설계: `DESIGN_v1_KO.md` 2.5(T1), 6.2, 6.5, 6.6, 6.7, 8.3 P4. 합격 기준: `SIM_ACCEPTANCE_KO.md` A5, A8, A11, A13, A17, A19, A22(바꾸지 않았다).
- 앞 단계: `EXECUTOR_IMPORT_KO.md`(p4_import, 실행기 가져오기), `T1_RIG_KO.md`(T1 장치), `HOST_BRIDGE_V3_KO.md`, `FIRMWARE_V3_KO.md`.
- 모든 수치는 자리값 프로필로 만든 **자리값 기반 SIM 결과**다. 현장 검증이 아니다. 이 문서에는 GNSS 좌표, 지도 원점, 실제 기록의 궤적 절대값이 없다.

## 1. 요약

1. 실행기 출력이 (throttle, steer_mrad)가 됐다. 안쪽 바퀴 규칙은 앞바퀴 조향에서 쓰지 않는다. 두 바퀴는 같은 듀티를 받는다.
2. `SteerOutputGuard`가 마지막 검사를 한다: 듀티는 0..상한 정수, 조향 목표는 정수 mrad, 등가 소프트 한계 안, 변화 속도 한계 안(주행 중·정지 중 따로).
3. 조향 상태가 실행기 입력이 됐다(`SteerMonitor`). 0.3 s보다 오래되면 멈춘다. MCU 고장 이름과 호스트 래치는 설계 6.6 정지 사유로 바뀐다.
4. 회전 중심은 뒤차축(기준점 뒤 0.38 m, 6.7)이다. 차동 피벗 값은 스냅샷에서 빠진다. 추정기의 피벗 값(`EstimatorConfig.pivot_icr_behind_lidar_m`, `pivot_icr_uncertainty_m`)을 바꿔도 결정과 스냅샷이 같다는 시험이 있다(추정기 자체 값은 달라지는 것도 확인한다. 검토 P4R-09).
5. 정지(PAUSED)나 끝(ENDED) 뒤에는 조향 목표가 바뀌지 않는다(6.5). 정지 상태에서 제어기가 조향을 바꾸면 그 run을 끝낸다.
6. `SteerBridgeClient`가 V3 브리지와 말한다. ARM 전에 펌웨어 V3, 차량 프로필, 보정, cal_id, 극성, 고장, 상태 신선도를 본다. `simulated` 장치는 SIM 설정에서만 허용한다. 실제 설정은 SIM 장치를 거부하고, SIM 설정은 실제 장치를 거부한다.
7. `tools/row_executor/sim_in_loop_steer.py`가 실행기 → 클라이언트 → 브리지 코어 → V3 메모리 전송 → MCU 펌웨어·액추에이터 → 조향 플랜트 → 세계·센서를 한 시계로 묶는다. 같은 시드면 직렬 로그, trace, 실행기 로그가 바이트까지 같다.
8. 대본 61개(정상 6개, RF-03 2개 포함, 검토 처리로 10개 더함). RF-03 두 개는 오탐률 run으로만 돈다. 나머지 59개와 오탐률 run 60개를 돌렸고, 59개가 모두 각자의 기대를 지켰다. A5, A8, A13, A17, A22는 합격이다. **A19는 불합격이다**: MCU 고장 15종의 판정 틱은 모두 명세 6.7 재계산과 같고(차이 0 틱) 호스트 래치 16종을 모두 T1 대본이 일으켰지만, 정상 대본 40회 중 1회에서 POLARITY 오탐이 났다(2.5 %, 7절). 검토 지적 처리는 11절.
9. 발견한 것: 자리값 프로필의 정지 중 조향이 느리다(약 4°/s, 7.1절). 정지 중 큰 조향 반전은 호스트 추종 규칙을 건다. DRV_STUCK 규칙은 고장 방향에 따라 놓칠 수 있다(7.3절).

## 2. 실행기 바뀐 곳

### 2.1 설정 블록 (`config.py`)

- 새 키는 `single_row_initial_press_grace_s` 뒤, turn 블록 앞에 있다. `TURN_KEYS`가 위치로 잘리기 때문이다.
- `vehicle_profile`: `differential`(기본) 또는 `front_steer`.
- 조향 키(모두 `steer_` 접두사, `STEER_KEYS` 21개):

| 키 | 기본 | 뜻 |
|---|---|---|
| `steer_limit_left_mrad`, `steer_limit_right_mrad` | 409 | 등가 소프트 한계. 호스트 프로필과 같아야 ARM한다 |
| `steer_rate_limit_deg_s` / `_standstill_deg_s` | 24 / 12 | 가드의 변화 속도 한계(주행 중 / 정지 중) |
| `steer_measured_rate_deg_s` / `_standstill_deg_s` | None | 측정 조향 속도(B5, F5). 한계는 이 값의 0.8배 이하 |
| `steer_rate_min_deg_s` | 8 | 측정 속도 하한(GO 규칙, P8에서 다시 정함) |
| `steer_range_measured` | False | 보정 기록의 측정 범위(B2, B3) |
| `steer_wheel_soft_margin_deg` | None | 바퀴별 소프트 한계와 기계 한계 사이(≥ 2°) |
| `steer_min_pwm_pct` | None | 조향 모터 시작 PWM(B4) |
| `steer_zero_offset_deg` | None | 영점 오차(F1, ±1.5° 안) |
| `steer_servo_deadband_deg` | None | 서보 데드밴드(≤ 1°) |
| `steer_polarity_checked` | False | 극성 검사(B0) |
| `steer_cal_id` | None | 브리지가 보고해야 하는 보정 세트 번호 |
| `steer_allow_sim` | False | SIM 장치 허용(SIM 설정만 True) |
| `steer_status_max_age_s` | 0.3 | 조향 상태 신선도 한계 |
| `steer_authority_fraction`, `steer_authority_s` | 0.95, 1.5 | D2 자리값 규칙(2.4절) |
| `steer_rotation_centre_x_m`, `_sd_m` | −0.38, 0.05 | 회전 중심(뒤차축)과 그 불확실성 |

- 손대지 않은 블록은 `to_dict()`에서 빠진다. 기본 설정과 단일열 SHADOW 설정의 digest가 546fa8c와 같다(`700ee41c…`, `594143ca…`, 시험이 고정값으로 비교).
- 정적 규칙(`steer_rule_problems`): `front_steer`이면 bench 프로필, turn assist, 보정 run 모터 출력을 거부한다(모두 좌우 듀티 표다). 정지 중 한계 ≤ 주행 중 한계. 상태 신선도 한계 ≥ 응답 주기 + 틱 하나(0.05 + 0.05 s). 회전 중심은 발자국 안.
- GO 규칙(`front_steer_go_problems`, 6.6): 측정 범위, 측정 속도와 0.8배 규칙, 속도 하한, 소프트 여유 ≥ 2°, 시작 PWM, 영점 ±1.5°, 데드밴드 ≤ 1°, 극성 검사, cal_id. SHADOW에서는 'unknown', 모터 출력이면 문제. 구동 쪽 C2, C3, C7은 그대로 둔다. 안쪽 바퀴 규칙과 C5 yaw 권한 규칙은 `front_steer`에서 쓰지 않는다.

### 2.2 정지 사유 표

MCU 고장 이름(상태 줄 `fl`)과 호스트 래치를 사유로 바꾼다. 조향 표가 먼저, 연결 표가 다음이다. 같은 표 안에서는 위가 먼저다.

| 이름 | 사유 | 분류 |
|---|---|---|
| POLARITY | steering_polarity_fault | CONTROL_AUTHORITY |
| DRV_STUCK, THERMAL | steering_driver_fault | CONTROL_AUTHORITY |
| STALL | steering_stalled | CONTROL_AUTHORITY |
| TRACK, 래치 steering_not_following | steering_not_following | CONTROL_AUTHORITY |
| 래치 steering_ineffective | steering_ineffective | CONTROL_AUTHORITY |
| POT_OPEN, POT_SHORT, POT_JUMP, POT_NOISE | steering_feedback_invalid | CONTROL_AUTHORITY |
| NOCAL, 래치 calibration_mismatch | steering_uncalibrated | CONTROL_AUTHORITY |
| 래치 steering_status_stale, status_seq_gap | steering_feedback_stale | CONTROL_AUTHORITY |
| 래치 stop_unconfirmed | stop_unconfirmed | SYSTEM_FAULT |
| 래치 mcu_reset_detected | mcu_reset | SYSTEM_FAULT |
| WATCHDOG, CRC, TTL | link_watchdog, link_bad_lines, link_ttl_expired | SYSTEM_FAULT |
| 래치 c_rejected, hardware_fault | link_command_rejected, link_hardware_fault | SYSTEM_FAULT |
| 래치 mcu_state_mismatch, arm_unconfirmed, arm_rejected, mcu_disarmed | 같은 이름 | SYSTEM_FAULT |
| 래치 sim_identity_mismatch, hardware_profile_mismatch | link_identity_mismatch | SYSTEM_FAULT |
| 래치 internal_error | bridge_internal_error | SYSTEM_FAULT |

- BOOT, SOFTLIMIT은 고장이 아니다. 사유가 없다.
- 실행기가 스스로 내는 사유: steering_feedback_stale(상태 없음·0.3 s 초과), steering_uncalibrated, steering_feedback_invalid(측정값이 없거나 한계의 1.5배 밖), steering_authority_insufficient.
- 연결 사유의 분류(SYSTEM_FAULT)는 제안이다. 조향 사전등록에서 정한다. TTL은 run을 끝낸다. 다음 C 줄이 조작자 없이 구동을 다시 시작할 수 있기 때문이다.
- TTL 사건은 상태 줄의 TTL 비트만으로는 놓칠 수 있다. 연결이 돌아온 뒤 처음 보낸 C가 다음 상태 줄보다 먼저 MCU에 닿으면 비트가 지워져 어느 상태 줄에도 나오지 않고, MCU는 TTL 제동 120 ms 뒤 다시 구동한다(규격 6.3, 8.2; 검토 RS-01). 그래서 `SteerMonitor`는 호스트가 센 F TTL 줄 수(`steering_host.counters.mcu_fault_TTL`, ARM 확인 전 값이 기준)도 본다(`link_events()`). 실행기는 drive 응답에서 이 사건을 보면 같은 틱에 run을 끝내고 STOP을 보낸다(`ExecutorLoop._link_event_after_send`).
- 남은 틈: 실행기는 호스트 카운터를 drive 응답에서만 본다(send_hz 10, 100 ms마다). 브리지는 한 단계에서 직렬 줄을 먼저 읽고 요청 하나를 처리한다. F 줄이 아직 오는 중(수 ms)일 때 처리한 drive의 `C`가 TTL 비트를 지우면 그 응답에는 사건이 없다. 실행기는 다음 drive 응답(≤ 100 + 20 ms 뒤)에서 사건을 보고, STOP은 그다음 브리지 단계(≤ 20 ms)에 나간다. 분석상 STOP은 TTL 제동 + 120 ms보다 최대 약 25 ms 늦을 수 있다(실행기 틱이 늦으면 더). 그동안 MCU는 휴지 120 ms 뒤 255/s 램프로 다시 출발할 수 있다(25 ms면 듀티 약 6/255). T1에서 손실 시작(4.95–5.05 s, 10 ms 간격), 손실 길이(320, 400 ms), 부팅 지연(55–75 틱), 브리지 위상(0, 10 ms)의 220조합을 돌렸다: TTL 220회, 상태 줄에 비트가 안 보인 TTL 8회, TTL 뒤 구동 증가 0회, `link_ttl_expired`로 끝나지 않은 run 0회. 이 조합들에서는 위 틈이 생기지 않았다. 실차 지연으로 재지는 못했다. 이 틈을 없애려면 호스트가 F TTL 줄을 읽은 단계에서 스스로 STOP해야 한다. 설계 4.1(TTL 뒤 `C`가 오면 다시 움직임)과 다르므로 결정으로 남긴다(11절).

### 2.3 출력 가드 (`guard.py` `SteerOutputGuard`)

- throttle: Python int, 0 ≤ throttle ≤ 듀티 상한(후진 없음).
- steer_mrad: Python int(bool, float, NaN 거부), −limit_right ≤ steer ≤ limit_left.
- 변화 속도: |steer − 이전| ≤ rate × dt + 1 mrad. 이 명령이나 이전 명령의 throttle이 0이 아니면 주행 중 한계, 아니면 정지 중 한계(MCU rmov/rstd와 같은 규칙).
- 위반이면 (0, 이전 조향, 위반 이름)을 돌려준다. Runner는 `guard_violation:<이름>`으로 run을 끝낸다(즉시 STOP).
- `hold()`는 throttle 0과 붙잡은 조향을 기록한다. 그래서 정지 뒤 첫 명령도 속도 검사를 받는다.

### 2.4 Runner (`core.py`)

- `Decision`에 `throttle`, `steer_mrad`가 생겼다(차동에서는 None). `front_steer`에서 left = right = throttle.
- `front_steer`는 조향 제어기 공장이 꼭 있어야 한다(vendor 제어기는 좌우 듀티를 낸다). P8 제어기 전에는 SIM 대본 제어기(4절)가 들어간다.
- `feed_steering(t, block, host)`: 브리지 응답의 조향 블록. run의 첫 측정값이 붙잡은 조향 목표가 된다(6.5: 조향은 있던 자리에 있다).
- 스냅샷: `rotation_centre_x_m` = −0.38, `rotation_centre_sd_m`, `steer_measured_mrad`, `steer_status_age_s`, `steer_command_mrad`를 더하고 `pivot_clearance_m`, `pivot_radius_required_m`을 뺀다.
- 누름: ARM 뒤(`steer_required`)에는 조향 상태가 쓸 수 있어야 누름을 받는다. 아니면 `press_ignored_steering_feedback`.
- ACTIVE 틱 순서: 조향 상태 검사 → 제어기 → HOLD 처리 → 정지 상태의 출력·조향 변화 금지 → 회랑 대기 → 기하 가드 → 조향 권한(D2 자리값) → 무응답 예산(N9) → 출력 가드.
- 조향 권한 규칙(D2 자리값): 주행 중 유효 회랑에서 |조향| ≥ 0.95 × 그쪽 한계가 1.5 s 넘게 이어지고 그동안 |횡오차|가 줄지 않으면 `steering_authority_insufficient`. 회랑이 없으면 평가하지 않는다.
- MCU 고장이나 래치는 ARM 확인 뒤에만 센다. 확인 전 블록의 고장은 지난 일이다(ARM이 지운다).

### 2.5 브리지 클라이언트 (`bridge_client.py`)

- `steer_arm_precheck(status, …)`가 처음 걸리는 문제를 돌려준다: `status_not_ok`, `esp_not_connected`, `firmware_v3_required`, `vehicle_profile_mismatch`, `sim_identity_mismatch`, `steer_limit_mismatch`, `steering_status_missing`, `steering_uncalibrated`, `calibration_id_not_configured`, `calibration_mismatch`, `polarity_check_missing`(POLARITY·DRV_STUCK 비트), `steering_fault:…`, `steering_status_stale`, `steering_feedback_invalid`, `stop_unconfirmed`, `bridge_armed_by_other_client`/`bridge_state_*`, `bridge_reports_moving`, `bridge_snapshot_stale`.
- BOOT, TTL, SOFTLIMIT 비트는 ARM을 막지 않는다.
- 조작자가 래치를 확인한 뒤의 재ARM(`acknowledge_latch()` → `arm()`)은 고장 비트와 래치를 넘긴다. 그것은 ARM이 지우는 것이고, 조건이 남아 있으면 브리지가 거부한다. 극성 비트는 재ARM에서도 막는다.
- 기록의 극성 플래그(`pol=ok`)는 호스트 상태에 없다. 브리지의 ARM 차단기가 그것을 본다(거부 코드 `polarity_check_missing`).
- `drive(throttle, steer_mrad)`, `keepalive()`(목표가 아직 없을 때), `motion(yaw, speed)`(브리지의 steering_ineffective 감시용 센서 표본, 래치하지 않음), `steering()`, `fault_reason()`.

### 2.6 실행 루프 (`executor_main.py`)

- 조향 모드에서 브리지 응답마다 조향 블록을 Runner에 넣는다(`_feed_steering`).
- 출력: 비상 사유(`STEER_EMERGENCY_REASONS`: 조향 정지 사유 중 권한 부족을 뺀 것 + 연결 사유)는 즉시 op stop. 그 밖의 끝은 throttle 0과 붙잡은 조향을 정지할 때까지(또는 `terminal_stop_wait_s`) 보낸 뒤 op stop.
- op stop 뒤에는 브리지의 STOP 확인을 1 s 지켜본다(`stop_confirmed`, `stop_unconfirmed`, `stop_watch_timeout` 사건).
- 감시견: 조향 모드에서는 drive(0, 마지막 조향)를 보낸다.
- drive 뒤마다 `motion` 표본(자이로 yaw rate, 스캔 정합 속도, 오도메트리가 유효할 때만)을 보낸다.
- drive 응답의 조향 상태에 호스트가 센 TTL 사건이 있으면 그 틱에 run을 끝내고 STOP을 보낸다(2.2절, 검토 RS-01). 사건 `bridge_stop`에 `same_tick: true`가 붙는다.

## 3. T1 실행기 루프 (`tools/row_executor/sim_in_loop_steer.py`)

- 사슬: 실제 `ExecutorLoop` + `Runner`(front_steer, MOTOR, 단일열 모드) ↔ 실제 `SteerBridgeClient` ↔ T1 장치의 제어 API(`RigClient`) ↔ `SteerBridgeCore` ↔ V3 메모리 전송 ↔ 펌웨어 V3 MCU + C++ 액추에이터·전위차계 ↔ `SteeredPlant` ↔ W0 평지 또는 W2 개발 회랑 ↔ 센서.
- 센서: 장치의 3200빔 스캔을 `s2_format.RealFormat`으로 실제 S2 형식으로 바꾼다(3240빔, angle_min float32(−π), 증분 0.0019398535 rad, 원래 순서, 장착 각 규칙, 0 = 반사 없음, 1 mm). IMU는 (0, 0, gyro_z)와 센서 축 가속도.
- 시계: 실행기는 20 Hz로 장치의 10 ms 격자 위에서 돈다. 늦은 틱은 건너뛴다. 실행기와 클라이언트는 장치 시계를 쓴다.
- 조작자 흉내: 0 s 시작, 0.5 s ARM, 1.2 s 누름. 정지(await_hold 아님) 뒤에는 놓고 다시 누른다(최대 20회). 끝 뒤에는 누르지 않는다. 자동 재ARM은 없다.
- 동작(대본의 actions): stop, release, hold_lost, press, end, ack, arm, start, executor_stall, client2_arm/drive/stop, late_drive, foreign_arm.
- SIM 실행기 설정(`steer_executor_config`): 차량 프로필 자리값(SIM_ASSUMED). 한계는 호스트 프로필 값, 속도 한계는 프로필 최대 속도의 0.8배, 회전 중심은 뒤차축, `steer_allow_sim` True, cal_id는 장치 값. `max_yaw_rate_deg_s` 3.6은 vendor SingleRowConfig 관계 때문이다(앞바퀴 조향은 이 차동을 쓰지 않는다).
- 산출물: `TRACE.npz`(장치 trace + `ctrl_*` 칸), `SERIAL.jsonl`, `EXECUTOR.jsonl`, `RESULT.json`(세 파일 정규 내용의 SHA-256).
- `truth_hold_position_rad`는 MCU가 거른 전위차계 위치다. hold_measured STOP이 붙잡는 값이다(R13-3). 장치가 직접 쓴다(같은 값의 `truth_mcu_pos_rad`도 있다. 검토 P4R-16). 평가기가 이것을 정지 뒤 조향 기준으로 쓴다.
- 전송: 실행기는 `RigSocket`(RigClient 감쌈)을 전송으로 받는다. 브리지 프로세스가 없으면(장치 고장 `bridge_restart`) 소켓 전송처럼 `TransportError('socket_error:…')`가 난다.
- 봉인 회랑(042301 c01/c04/c07 양방향)은 장면 명세와 장치가 둘 다 거부한다.

## 4. 대본 제어기 (`sim/ubts/steering/scripted_control.py`)

- 행 추종 제어기가 아니다(그것은 P8). 개발용 운전자다. 스냅샷만 읽는다(A13 역할 decision).
- `StraightHold`: 정지 중 정렬 → 주행. 유효 회랑이면 δ = −k_d × 횡오차 + k_psi × 방향 오차(0.4, 0.9), 아니면 출발 때의 자이로 방향과 추측 횡오차. 최대 8°. 거리, 추정기 열 끝(−0.61 m), 앞 여유 0.6 m에서 멈춘다.
- `SegmentScript`: 구간 목록(시간, 듀티, 조향 각·'hold'·sine·triangle, raw). 끝은 STOP 또는 끝 붙잡기. 만드는 함수: steps, circle, zigzag, stop_test, standstill_sweeps.
- `HoldStill`: 정지 붙잡기.
- 조향 명령은 `SteerShaper`를 지난다: 실행기 한계의 0.75배 속도, 0.9배 소프트 한계(대본이 `rate_margin`, `limit_margin`을 (0, 1]에서 바꿀 수 있다).
- 멈춤: STOP_REQUESTED(throttle 0, 조향 유지) → 0.4 s 정지가 보이면 HOLD.

## 5. 안전 대본 (`sim/ubts/steering/safety_scripts/`)

- `catalogue.py`: 대본마다 에피소드 명세와 기대(`expect`). `measure.py`: 산출물에서 판정 틱, STOP 시간, 재전송 격자, 붙잡기 오차, A22 창을 계산한다. 판정 틱은 MCU 채널(P, cmd, 출력, 모드, ADC)과 CFG 값으로 명세 6.7을 다시 계산한다. 펌웨어 코드는 쓰지 않는다. `acceptance.py`: 대본을 돌리고 점검표를 만든다.
- 세계: W0 시험장(1.5 m 간격 기둥 두 줄, 3.5 m 폭, 60개) 또는 개발 회랑 042301 c03 fwd.

| 묶음 | 대본 |
|---|---|
| 안전 | mid_row_stop_w2, mid_row_stop_w0, deadman_release, hold_lost, c_line_loss_ttl, c_line_loss_ttl_status_phase, c_line_loss_ttl_catalogue_phase60, c_line_loss_ttl_bridge_phase, link_loss_watchdog_brake, executor_stall, bridge_stall_watchdog, cal_id_mismatch_executor, stale_steering_status, stop_unconfirmed, power_cut |
| 고장 | pot_open, pot_short, pot_jump, pot_noise, polarity_reversed, driver_stuck, driver_stuck_isolated, stall_jam, track_jam_small, thermal, nocal_boot, softlimit |
| 09-19 필수 | mcu_reset, two_screens_and_cli, link_loss_while_held, link_loss_lateral_5deg, bridge_forced_exit, bridge_forced_exit_port_reset, usb_unplug_replug, usb_unplug_replug_reset, half_frame, crc_error_single, crc_error_burst, range_error, status_line_loss, late_drive_after_stop, clock_change(기준 clock_reference) |
| 호스트 | cal_id_mismatch_host, host_not_following, host_ineffective, host_seq_gap, host_state_mismatch, arm_unconfirmed, host_hw_profile_mismatch, host_sim_identity_mismatch, host_arm_rejected, host_internal_error |
| 정상 | normal_straight_w0, normal_straight_w2, normal_zigzag, normal_arcs, normal_steps_standstill, clock_reference |
| RF-03 | rf03_sweeps_duty0, rf03_sweeps_duty4 |
| 발견 | finding_standstill_reversal |

- 고장 하나만 걸리게 CFG를 넓힌 대본: track_jam_small(stm 60), driver_stuck_isolated(polp 1000), host_not_following(trk 2000, stm 0), thermal과 RF-03(호스트 추종 시간 60 s). 넓힌 이유는 대본 설명에 있다.
- 정상 묶음은 원 아크 대신 S자 두 아크다. 열 안에서는 기하 가드(방향 25°)가 원을 끝내기 때문이다.
- 검토 처리로 더한 대본(10개, 11절):
  - `c_line_loss_ttl_status_phase`(손실 4.97 s부터 0.32 s, 부팅 지연 60틱): 연결이 돌아온 뒤 첫 `C`가 다음 상태 줄보다 먼저 MCU에 닿아 TTL 비트가 어느 상태 줄에도 나오지 않는 위상이다(RS-01). `c_line_loss_ttl_catalogue_phase60`(부팅 지연 60틱), `c_line_loss_ttl_bridge_phase`(브리지 위상 10 ms)는 같은 손실의 다른 위상이다. 모두 `link_ttl_expired`로 끝나고 다시 구동하지 않아야 한다.
  - `link_loss_watchdog_brake`(wd 200, 손실 0.5 s): TTL(300 ms)보다 watchdog이 먼저 구동을 세운다. A19 WATCHDOG 판정 틱은 이 대본으로 잰다(브레이크가 watchdog에서 나오는 경우).
  - `bridge_forced_exit`: 장치 고장 `bridge_restart`(브리지 프로세스가 죽고 2 s 뒤 새 프로세스가 포트를 연다). 실행기는 소켓 오류로 끝나고, 새 프로세스는 lease가 없고 C 줄을 보내지 않는다. `bridge_forced_exit_port_reset`은 새 프로세스가 포트를 열 때 MCU가 리셋되는 경우다(새 boot, 7.0 s 뒤 BOOT).
  - `usb_unplug_replug_reset`: 다시 꽂을 때 MCU가 리셋되는 경우(RS-13). `hardware_fault` 래치, 새 boot, 다시 ARM하지 않는다.
  - 호스트 래치 4종: `host_hw_profile_mismatch`(호스트가 기대하는 하드웨어 프로필이 다름), `host_sim_identity_mismatch`(호스트가 실차 링크를 기대하는데 SIM MCU), `host_arm_rejected`(POT_OPEN이 남은 채 조작자가 확인하고 새 run을 ARM하면 MCU가 ARM을 거부하고(`N`, fault) 호스트가 `arm_rejected`를 래치), `host_internal_error`(장치 고장 `host_defect`: 호스트 코어 루프 안 예외 한 번).

## 6. 점검 결과 (검토 처리 뒤 최종 코드, Windows Python 3.13.2)

- `checks_final_w6`(작업자 6, 120 s)과 `checks_final_w1`(작업자 1, 419 s). 두 기록에서 대본 59개의 직렬 로그·trace·실행기 로그 digest가 모두 같고, A17·A19·A22 표와 오탐률 표도 같다. 다른 것은 A11의 시간 값뿐이다. 대본 59개가 모두 각자의 기대를 지켰다. 기록은 runs 폴더 `fix_wf3a/`에 있다.
- 앞선 checks_3(대본 49개)과는 대본, 판정 규칙, trace 칸이 달라(11절) digest를 비교하지 않는다.

| 점검 | 판정 | 내용 |
|---|---|---|
| A5(F) | 합격 | 3240빔, angle_min, 증분, frame laser, 0.05–30 m, 반사 없음 0.0(최소 2347빔), 10 Hz, 정면 기둥 원래 빔 8에서 5.70 m(기대 5.70), 거리 잡음 0.01496 m(프로필 0.015), 자이로 잡음 0.002574 rad/s(모델 0.002485), IMU 33.002 Hz(33) |
| A8 | 합격 | 같은 시드 두 번 직렬 로그·trace·실행기 로그 같음, 다른 시드 다름, 작업자 2개 풀도 같음(대본 2개) |
| A11 | 보고 | 작업자 1: 실행기 한 걸음 p50 1.26 ms, p99 12.1 ms(대본별 중앙값), p99 최악 82.3 ms, p99.9 최악 125 ms, 최대 137 ms, MCU FW_STEP 평균 0.18 ms, 대본 벽시계 0.5–18.1 s(중앙 1.4 s, 합 127 s). 작업자 6: 실행기 한 걸음 p50 1.63 ms, p99 18.0 ms(대본별 중앙값), p99 최악 64.3 ms, p99.9 최악 129 ms, 최대 142 ms, MCU FW_STEP 평균 0.27 ms, 대본 벽시계 0.8–37.8 s(중앙 2.0 s). 다른 작업과 같은 PC다 |
| A13(F) | 합격 | 178개 파일, 오류 0, 면제 12개 모두 쓰임(쓰지 않은 면제 0). 실행기 구독 /scan, /imu/data_raw, /rover/control/requested(허용 목록 안) |
| A17(F) | 합격 | 195개 항목(판정 189, 해당 없음 6). 필수 15개 대본 모두 합격 |
| A19(F) | **불합격** | 고장 15종 모두 판정 틱 차이 0. 호스트 래치 16종 모두 T1 대본으로 확인. 정상 40회 중 POLARITY 오탐 1회 |
| A22(F) | 합격 | 18개 끝 순간, 모두 관찰 3 s가 다 있다. 명령 조향 최대 이동 0.75°(허용 1°), 실행기 명령 이동 최대 0.17°, 자동 중앙 복귀 0 |

### 6.1 A17 수치

- STOP: 요청 → STOP 바이트 0 ms(같은 브리지 틱), MCU 해석 → 구동 0은 0 틱, 확인 25 ms(한계 100 ms). 대본 mid_row_stop_w0, mid_row_stop_w2, two_screens_and_cli, late_drive_after_stop(앞 두 항목은 stop_unconfirmed도).
- STOP 확인 실패(stop_unconfirmed, MCU 0.8 s 멈춤): 재전송 100, 160, 200, 260, 300 ms(수정 7 격자와 같음), 래치 400 ms. MCU 멈춤 안에 온 STOP은 멈춤 끝(6800 틱)에 해석되고 그 틱에 구동 0.
- TTL: 마지막으로 받은 C의 도착 + ttl과 F TTL 틱이 같다(9개 대본). 구동 0은 ttl + 1 틱 안.
- 감시견: 마지막으로 받은 입력 + wd + 1과 W 틱이 같다(7개 대본). 그 뒤 DISARMED. MCU가 거절한 줄은 입력으로 세지 않는다(규격 6.2). 'watchdog → 구동 0'은 W 전에 구동이 0이 아니었던 link_loss_watchdog_brake에서 판정한다(W 5164, 구동 0 ≤ 5165). 나머지 6개는 TTL 브레이크로 이미 0이라 해당 없음이다.
- 고장 → 조향 출력 0: 1 틱 안(EN과 PWM, 10개 대본). 구동은 브레이크 고장이면 1 틱 안, THERMAL만이면 풀림 경사(255/s, 10 ms 격자) 한계 안.
- 붙잡기 오차(정지 뒤 2 s, 참 바퀴): 7개 대본(STOP 해석, W, F CRC 틱부터), 최대 0.13°(crc_error_burst, mid_row_stop_w0), 한계 1°.
- 자동 재시작 없음(43개 항목): 정지 순간부터 구동이 커지지 않고, 1.5 s 뒤 0이며, ARM 줄이 없다.
- 연결 끊김 중 옆 이동(link_loss_lateral_5deg, 4.75°, 0.43 m/s): SIM 0.0116 m, 해석 0.0078 m(뒤차축 호 0.0016 + 기준점 0.38 m × sin(κs) 0.0062), 차이 0.0038 m(허용 ±10 % 또는 ±0.01 m). 설계 목표 ≤ 0.05 m도 맞다. 남은 차이는 플랜트의 무작위 옆 미끄럼이다.
- 두 화면과 CLI: 두 번째 화면의 arm은 `control_in_use`, drive는 `control_not_owned`, CLI STOP은 받아들여진다. 실행기는 래치한다.
- 늦은 drive: `stop_barrier`로 거부.
- 브리지 강제 종료(bridge_forced_exit, bridge_forced_exit_port_reset): 5.0 s에 프로세스가 끝나고 7.0 s에 새 프로세스가 선다. 그동안 실행기 요청 3번이 소켓 오류다. 새 프로세스는 lease 0, 세션 STOP 1줄, C 줄 0. 포트 리셋 경우 7000 ms에 새 boot.
- USB를 다시 꽂을 때 리셋(usb_unplug_replug_reset): 6000 ms에 새 boot, `hardware_fault` 래치, 다시 ARM하지 않는다.
- 시계 변경(+1 h): clock_change와 clock_reference의 trace와 직렬 로그가 같다.

### 6.2 A19 표

| 고장 | 대본 | 판정 틱 | 재계산 | 차이 |
|---|---|---|---|---|
| NOCAL | nocal_boot | 70 | 70(setup 틱, 상태 비트) | 0 |
| POT_OPEN / POT_SHORT / POT_JUMP | pot_open / pot_short / pot_jump | 5002 | 5002 | 0 |
| POT_NOISE | pot_noise | 5021 | 5021 | 0 |
| POLARITY | polarity_reversed | 1669 | 1669 | 0 |
| DRV_STUCK | driver_stuck_isolated | 5302 | 5302 | 0 |
| SOFTLIMIT | softlimit | 6502 | 6502(첫 P > soft_l) | 0 |
| STALL | stall_jam | 3936 | 3936 | 0 |
| TRACK | track_jam_small | 4368 | 4368 | 0 |
| THERMAL | thermal | 15361 | 15361 | 0 |
| TTL | c_line_loss_ttl 외 8 | 5263 | 5263 | 0 |
| WATCHDOG | link_loss_watchdog_brake 외 6 | 5164 | 5164 | 0 |
| CRC | crc_error_burst | 5103 | 5103(crcn번째 나쁜 줄 도착) | 0 |
| BOOT | mcu_reset | 5070 | 5070(리셋 + 부팅 지연) | 0 |

- 호스트 래치 16종을 모두 T1 대본으로 확인했다: arm_rejected, arm_unconfirmed, c_rejected, calibration_mismatch, hardware_fault, hardware_profile_mismatch, internal_error, mcu_disarmed, mcu_reset_detected, mcu_state_mismatch, sim_identity_mismatch, status_seq_gap, steering_ineffective, steering_not_following, steering_status_stale, stop_unconfirmed. 앞서 T1에서 만들지 않던 4종은 대본 4개로 만든다(5절, 검토 P4R-15).
- 오탐률(정상 4개 대본 × 시드 10): STALL 0/40, steering_ineffective 0/40, **POLARITY 1/40**(normal_steps_standstill 시드 101).
- RF-03 THERMAL(정지 중 ±20° 삼각 스윕 30 s, 시드 10): 듀티 0에서 10회 모두 THERMAL 창이 차고 THERMAL 0/10. 듀티 4 %(구동 데드밴드 안)에서는 차가 움직이지 않아 실행기 무응답 예산(N9, 6 s)이 run을 약 7 s에 끝낸다(10/10). THERMAL 창이 차기 전이라 **재지 못했다**(비율 없음, 검토 P4R-07·RS-11). 듀티 4 %로는 긴 정지 스윕을 할 수 없다.

## 7. 발견한 것

### 7.1 정지 중 조향이 느리다

- 자리값 프로필의 정지 중 조향: 서보 이득 15 %/deg, 정지 부하 3 N·m, 정지 시작 PWM 35 %. 오차 3°의 서보 출력은 약 45 %다. 이때 정지 중 실제 속도는 약 4°/s다. 반전에는 백래시(2°) 때문에 약 0.8 s의 죽은 시간이 있다.
- 프로필의 정지 조향 속도 상한은 15°/s다. 실행기 한계는 그 0.8배인 12°/s다.
- 결과: ±10° 정지 반전은 대본 속도(0.75배부터 0.3배까지, 9–3.6°/s) 어디서나 호스트 추종 규칙(3°, 1.5 s)을 걸었다(`finding_standstill_reversal`). 그래서 정상 대본의 정지 단계는 ±5°다.
- 뜻: SIM의 '측정' 정지 속도를 프로필 상한에서 가져오면 안 된다. B5/F5처럼 장치에서 재야 한다. 정지 중 추종 규칙과 백래시 죽은 시간도 함께 정해야 한다.

### 7.2 POLARITY 오탐

- normal_steps_standstill 시드 101, 8831 틱. 정지 중 −5°로 가는 중이었다. 서보 출력은 최소(−102)이고 실제 바퀴는 약 1°/s로 기어갔다.
- 거른 전위차계 값이 ±2 원시 카운트(약 ±21 permille) 흔들렸다. 100 틱 사이 P 차이가 +42 permille가 되어 polp 40을 넘었다.
- 뜻: 자리값 polp 40 permille(약 1°)는 거른 전위차계 잡음 폭과 가깝다. polp를 키우거나(예: 3 × 잡음 폭) 창을 늘리는 결정이 필요하다. 펌웨어 자리값이라 이 작업에서 바꾸지 않았다.

### 7.3 DRV_STUCK 규칙의 경쟁

- DRV_STUCK은 출력이 0(브레이크 단계)일 때 기준을 잡고, 위치가 기준에서 db보다 멀어진 뒤에야 센다. 서보 데드밴드도 같은 db다.
- 고장 난 드라이버가 cmd 쪽으로 밀면, 위치가 기준에서 db만큼 벗어나기 전에 서보가 반대로 밀기 시작한다. 그러면 계수기가 처음으로 돌아가고 DRV_STUCK은 걸리지 않는다. 대신 TRACK이 나중에 걸린다(run에 따라 POLARITY).
- driver_stuck(60 %)는 POLARITY로 끝났다. driver_stuck_isolated는 POLARITY 규칙을 넓히고 오른쪽(−60 %)으로 밀게 해서 DRV_STUCK을 확인했다.

## 8. 바꾼 다른 파일

- `sim/ubts/steering/t1_rig.py`(막힘 결함, 뒤 호환): `RigConfig.report_adc`(기본 False). True일 때만 펌웨어 시작 줄에 `report_adc`를 넣는다. POT_NOISE 판정 틱 재계산에 원시 ADC가 필요하다. 기본 시작 줄은 그대로다.
- `sim/ubts/steering/evaluate_steer.py`: 실행기 trace 평가(`evaluate_executor_trace`, `evaluate_executor_run`, CLI `executor`), 붙잡기 기준 칸 `HOLD_POSITION_COLUMNS`(`truth_hold_position_rad` 먼저), 연결 사유 분류. 자세한 것은 `EVALUATE_STEER_KO.md` 8절.
- `sim/ubts/steering/truth_isolation.py`: 새 파일의 역할(대본 제어기 decision, 안전 대본 evaluator, T1 장치 plant, s2_format shared, ident 등), 실행기 구독 검사(`subscription_check`), 경로·함수·싱크까지 맞춰야 하는 면제(`SINK_EXEMPTIONS`).
- `tools/row_executor/core.py`: 위 2.4절. 초기 누름이 기다림 시간을 넘으면 원인이 조향 상태일 때 `press_ignored_steering_feedback`을 쓴다.
- `tools/row_executor/SHA256SUMS`, `vendor/SOURCE_MANIFEST.json`: `regen_vendor --worktree`로 다시 만들었다.
- 검토 처리(fix_wf3a)에서 더 바꾼 파일(자세한 것은 11절):
  - `sim/ubts/steering/t1_rig.py`: 장치 고장 `bridge_restart`, `host_defect`. 설정 `host_hw_profile`, `host_simulated`. trace 칸 `truth_mcu_pos_rad`, `truth_hold_position_rad`(장치가 직접 쓴다. 예전 `truth_servo_view_rad` 이름은 쓰지 않는다). `T1_RIG_KO.md` 3.1, 6.2, 7절.
  - `tools/row_executor/sim_in_loop_steer.py`: `RigSocket`(브리지 프로세스가 없으면 소켓 오류).
  - `tools/row_executor/core.py`: `SteerMonitor.link_events()`, vendor import(`_SimTreeNames`). `executor_main.py`: `_link_event_after_send`.
  - `sim/ubts/steering/evaluate_steer.py`: 붙잡기 기준 다시 계산(`measured_hold_rows`, `hold_reference`). `EVALUATE_STEER_KO.md` 8절.
  - `sim/ubts/steering/truth_isolation.py`: 세 칸 면제 키, 장치 채널 규칙, `tools/field_calib` 범위. `EVALUATE_STEER_KO.md` 5절.
  - `sim/ubts/steering/safety_scripts/`: 대본 10개, 판정 규칙(11절).
  - `tools/steer_sim/regen_vendor.py`, `check_vendor_sync.py`, vendor 폴더 이름: `EXECUTOR_IMPORT_KO.md` 13절.

## 9. 돌리는 방법

작업 트리 바깥 폴더에 결과를 쓴다. 환경: `UBTS_STEER_BUILD_DIR`(저장소 밖 빌드 폴더), `UBTS_STEER_CXX`(Windows g++), `UBTS_STEER_PATHS`(W2 지도).

```
python -B tools/row_executor/sim_in_loop_steer.py episode --spec <spec.json> --out <폴더>
python -B tools/row_executor/sim_in_loop_steer.py checks --out <폴더> --workers 6
cd sim/ubts && python -B -m steering.evaluate_steer executor --run <run 폴더>
```

- `checks`는 대본 59개(RF-03 2개는 오탐률 run으로만 돈다), 오탐률 run 60개, A5, A8, A13을 돌리고 `P4_CHECKS.json`을 쓴다. `--workers`는 A11 보고에 적힌다. A11은 작업자 1과 N으로 두 번 돌린다.
- `--only 이름|묶음|a5|a8|a13|rates`로 줄일 수 있다. 그러면 돈 점검만 판정하고 `complete`가 false다. 대본이 없으면 A17, A19, A22, A11이 없고, 오탐률 run이 없으면 A19가 없다.

## 10. 남은 일

- A19 불합격(7.2절)의 결정: polp 또는 POLARITY 창.
- SIM 측정 정지 속도와 정지 중 추종 규칙(7.1절).
- DRV_STUCK 규칙의 경쟁(7.3절)을 받아들일지, 기준을 바꿀지.
- 보정 run(C5, C3b, C7)은 `front_steer`에서 거부한다. 조향 식별은 `tools/field_calib`(p4_ident)이다.
- 연결 사유 분류(SYSTEM_FAULT)와 D2 권한 규칙 숫자는 조향 사전등록에서 정한다.
- TTL 사건의 남은 틈(2.2절): 호스트가 F TTL 줄을 읽은 단계에서 스스로 STOP할지 정해야 한다(설계 4.1과 다름).
- F-SIM hold_measured(검토 RS-12): 붙잡기 목표는 측정 공간(거른 전위차계)이고 시험 서보의 되먹임은 참 공간이다. 기본 프로필에서 차이는 1° 안이다. P8 전에 모형을 정한다.
- RF-03 듀티 4 %의 THERMAL 오탐률은 재지 못했다(6.2절). 무응답 예산(N9) 안에서 THERMAL 창을 채우는 다른 대본이 필요하다.
- 끝 순간 붙잡기와 TTL 지연은 T1 장치(한 SIM 시계)에서만 쟀다. 실차 지연은 재지 않았다.

## 11. 검토 지적 처리 (fix_wf3a, 2026-11-09)

검토 review_p4(P4R-01–16)와 review_safety(RS-01–13)를 처리했다. 등록 합격 기준은 바꾸지 않았다. 지적마다 옛 동작을 잡는 시험을 더하거나 고쳤다(대본은 고친 판정 규칙으로 돈다). 검토 목록은 RS-13의 세부 설명 중간에서 잘려 전달됐다. 그 뒤 항목이 있었다면 보지 못했다.

| 지적 | 고친 것 | 시험 |
|---|---|---|
| RS-01 | 호스트가 센 F TTL 줄 수(`mcu_fault_TTL`)를 `SteerMonitor.link_events()`가 본다. drive 응답에서 보면 그 틱에 run을 끝내고 STOP(2.2절). TTL 위상 대본 3개. 남은 틈은 2.2절, 결정은 10절 | `test_ttl_event_counted_by_the_host_ends_the_run`, `test_ttl_event_without_a_status_bit_ends_the_run_before_a_restart`, 대본 `c_line_loss_ttl_status_phase` 외 2 |
| RS-02, P4R-11 | '자동 재출발 없음'은 정지 순간부터 구동이 커지지 않는지 본다(1.5 s 뒤 0, ARM 줄 0도 그대로) | `test_restart_inside_the_first_1500_ms_is_found` |
| RS-03 | 정지 전에 구동이 이미 0이면 'watchdog → 구동 0'은 해당 없음. `link_loss_watchdog_brake`(wd 200)가 watchdog 브레이크를 잰다. A19 WATCHDOG도 이 대본 | `test_watchdog_brake_item_only_when_moving`, 대본 |
| RS-04, P4R-05 | 장치 고장 `bridge_restart`: 브리지 프로세스가 죽고 새 프로세스가 포트를 연다(`port_open_reset` 선택). `bridge_forced_exit`, `bridge_forced_exit_port_reset` | `test_bridge_restart_is_a_new_process`, `test_bridge_restart_is_a_new_process_without_a_lease` |
| RS-05 | A13 엄격 역할에 장치 채널 규칙(`rig_reference`, `rig_channel`, `scene_truth`). 식별 기록 작성기의 엄격 시험에서 `ident/bench.py`(트리 역할 plant, 벤치 장치를 직렬 포트로 씀)는 `rig_reference`만 뺀다(`IDENT_KO.md` 11절) | `test_rig_channel_reads_in_decision_code_are_caught` 외 2 |
| RS-06, P4R-03 | 평가기가 fast_sim trace(trace_every 1)의 pot_raw로 `SteeredPlant.measured_hold_rad`를 다시 계산해 붙잡기 기준으로 쓴다(`measured_hold_recomputed`) | `test_recomputed_hold_reference_is_the_plant_hold_measured_target`, L1 brake 시험 |
| RS-07, P4R-02 | vendor 꾸러미 이름을 `closed_loop_copy`, `orchard_perception_copy`로 바꾸고 `sys.path`에 넣지 않는다. `check_pins` OK(`EXECUTOR_IMPORT_KO.md` 13절) | `test_vendor_and_sim_tree_live_in_one_process`, `test_regen_moves_a_vendored_file_and_prunes_the_old_folder` |
| RS-08 | 규격 13.5의 instant 넘침 문장을 12.1-67(고리 255 B, 가장 옛 바이트를 버림)과 맞췄다 | 문서 |
| RS-09 | 붙잡기 오차 항목을 정지가 있는 대본 7개에 쓴다(STOP 해석, W, F CRC 틱부터) | 대본 |
| RS-10 | watchdog·TTL 기대 틱 재계산에서 MCU가 거절한 줄(`N`)은 입력으로 세지 않는다(규격 6.2, 6.3) | `test_refused_lines_are_no_input` |
| RS-11, P4R-07 | 오탐률 표가 THERMAL 창을 채우지 못한 run을 '재지 못함'으로 적는다(듀티 4 %: 10/10 재지 못함, 비율 없음) | `test_thermal_rate_counts_only_filled_windows` |
| RS-12 | 고치지 않았다. 모형 결정(10절) | — |
| RS-13 | `usb_unplug_replug_reset`: 다시 꽂을 때 MCU 리셋(새 boot, 6.0 s), `hardware_fault` 래치, 다시 ARM하지 않음 | `test_usb_replug_with_a_port_open_reset_boots_again`, 대본 |
| P4R-01 | A13 면제 키가 (경로, 함수, 세부) 세 칸이고 한 번만 쓴다. 쓴 면제와 안 쓴 면제를 보고한다 | `test_a_new_leak_in_an_exempted_function_is_an_error`, `test_tree_report_lists_the_exemptions` |
| P4R-04 | `P4_CHECKS.json`과 A11 보고에 작업자 수. 작업자 1과 6으로 돌렸다(6절) | 실행 기록 `checks_final_w1`, `checks_final_w6` |
| P4R-06 | A22는 관찰 3 s가 다 들어 있어야 판정한다. `normal_steps_standstill`을 20 s로 늘렸다 | `test_a22_window_must_be_complete` |
| P4R-08 | `IDENT_KO.md`에 A18(F) 불합격 판정 | 문서 |
| P4R-09 | 피벗 값 시험이 `EstimatorConfig` 피벗 값을 바꾸고, 추정기 값은 달라지는데 결정·스냅샷은 같음을 본다 | `test_steer_executor.py` 피벗 시험 |
| P4R-10 | `--only` 실행은 돈 점검만 판정하고 `complete` false. 대본이 없으면 A17 등이 없다 | `test_partial_runs_report_only_what_ran` |
| P4R-12 | `check_vendor_sync`: git이 manifest 커밋이나 경로를 모르면 PROBLEM | `CheckGitTest` |
| P4R-13 | `tools/field_calib`을 A13 범위에 넣었다(시험은 test, 나머지는 shared) | `test_field_calib_files_are_checked_as_shared` |
| P4R-14 | 6절의 A13 파일 수를 이번 기록(`checks_final_w6`)에서 옮겼다 | 문서 |
| P4R-15 | 호스트 래치 4종을 T1 대본으로 만든다(`host_hw_profile_mismatch`, `host_sim_identity_mismatch`, `host_arm_rejected`, `host_internal_error`). 16종 모두 대본으로 확인 | `test_host_link_settings_and_host_defect`, 대본 |
| P4R-16 | 장치 trace의 MCU 위치 칸을 `truth_mcu_pos_rad`로 바꾸고 `truth_hold_position_rad`를 장치가 쓴다 | `test_stop_while_slewing_holds_the_filtered_position` |

- TTL 위상 쓸기(저장소 밖 스크립트, 2.2절): 220조합, 재구동 0, `link_ttl_expired`로 끝나지 않은 run 0.
- A18 다시 돌림: 수치가 a18_full_v5와 같다(`IDENT_KO.md` 6절).
