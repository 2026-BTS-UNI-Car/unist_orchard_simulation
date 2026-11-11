# 조향 네이티브 프로세스 lockstep 규약 v1

- 작성: 2026-11-08 (P1 공유 코어)
- 근거: `DESIGN_v1_KO.md` 2.3.1, 3.3, 3.4, 4.1과 `DECISIONS_KO.md` R1
- Python 쪽 구현: `sim/ubts/steering/backend_api.py` (struct 배치, 인코드·디코드, 시작 줄 생성)
- 상태: v1 규격이다. 네이티브 쪽(C++)은 `sim/ubts/steering/native/`에 있다(구현 세부: `NATIVE_ACTUATOR_MODEL_KO.md`). 이 문서, `backend_api.py`, C++ 구현은 같아야 한다. `sim/ubts/test_steering_backend_api.py`가 이 문서의 hex 예·시작 줄 키 목록·level 목록·참값 칸 목록·종료 코드를 코드와 대조한다.
- 고친 곳(2026-11-09, 검토 반영): C++ 구현과 다르던 곳(풀림 복원 식, L1b·L0 제한, L0의 applied 뜻, 종료 8, 기계 한계 키 이름)을 구현에 맞췄다. `applied_steer_pwm`을 참값 칸으로 옮겼다. 정지 중 값 순서와 가변저항 이득 검사를 시작 줄 검사에 더했다.
- v2(firmware 모드, P3 규격, 2026-11-09)는 11절이다. 1~10절의 v1 규격은 그대로 유효하다. 11절의 구현 위치는 11절 머리의 상태 줄에 있다. ESP 규약은 `ESP_PROTOCOL_V3_DRAFT_KO.md`다.

조향 구동기·가변저항 모델(C++)과 P3부터의 MCU 로직은 한 하위 프로세스에서 돈다. 차체 플랜트(Python)와는 플랜트 한 스텝(보통 10 ms)마다 한 번 바꾼다. 이 문서는 그 교환의 바이트 형식, 순서, 오류 처리, 시작 줄 필드의 뜻을 정한다. 구동기 동역학 식(L0, L1, L1b)은 설계 3.3·3.4를 따른다. 여기서는 필드의 뜻과 각 변환의 정의만 정한다.

---

## 1. 수명

1. 부모(Python)가 프로세스를 띄운다. stdin과 stdout은 이진 파이프다. stderr는 진단 문자열에만 쓴다.
2. 부모가 시작 줄 한 줄을 쓴다(2절). ASCII JSON이고 `\n`으로 끝난다. 64 KiB 이하다.
3. 프로세스는 시작 줄을 검사한다. 통과하면 READY 프레임 하나를 쓴다. 실패하면 stderr에 진단 한 줄을 쓰고 종료 코드 2로 끝난다.
4. READY 직후 상태는 `RESET(0.0)` 직후와 같다.
5. 그 뒤 부모가 요청 프레임 하나를 쓰고 프로세스가 응답 프레임 하나를 쓴다. 이것을 엄격히 번갈아 한다. 부모는 응답을 받기 전에 다음 요청을 쓰지 않는다. Python 연결(`NativeSteeringBackend`)은 요청 쓰기 하나나 응답 하나가 `response_timeout_s`(기본 30 s)를 넘으면 프로세스를 죽이고 오류(`timed_out`)를 낸다.
6. 부모가 프레임 경계에서 stdin을 닫으면(EOF) 프로세스는 종료 코드 0으로 끝난다.
7. 시작 줄의 첫 바이트 전에 EOF가 오면 아무것도 하지 않고 0으로 끝난다.

## 2. 시작 줄

- 한 줄짜리 JSON 객체다. 단위는 SI다: m, rad, s. PWM과 duty는 비율(−1..1)이다. ADC 값은 count다.
- Python 생성 함수: `native_startup_config(profile, seed, mode, mcu_tick_hz, injections)`와 `startup_line(config)`.
- 모르는 키, 빠진 키, 형이 틀린 값은 종료 코드 2다. 예외: `profile_overrides`의 내용과 `mcu` 블록은 v1이 해석하지 않는다(형만 본다).
- v1 구현이 시작 줄에서 거부하는 것(종료 2, 8절): `mode` `firmware`, `actuator.level` `L1b`, `L0`인데 `mode`가 `test_servo`가 아님, `L0`에서 드라이버 고장 주입(`driver_dead`, `driver_stuck_on`, `polarity_reversed`), `rate_standstill_rad_s` > `rate_max_rad_s`, `min_pwm_standstill` < `min_pwm`, 가변저항 이득 2^adc_bits × to_steer_ratio × (1 + gain_error) / range_rad가 유한하지 않거나 (0, 1e9] count/rad 밖, 등가 기계 한계가 바퀴별 한계와 맞지 않음, 0점 오차가 등가 기계 한계 밖. Python `native_startup_config()`는 `L1b` 시작 줄도 만든다. 거부는 네이티브가 한다.
- 각 값은 차량 프로필(`sim/ubts/steering/profiles/*.yaml`)에서 나온다. 괄호 안은 프로필 이름이다.

맨 위 키:

| 키 | 형 | 뜻 |
|---|---|---|
| `protocol` | 문자열 | `"ubts.steer_lockstep.v1"`. 다르면 종료 2 |
| `mode` | 문자열 | `direct_pwm`, `test_servo`, `firmware`. v1 구현은 `firmware`이면 종료 2 |
| `seed` | 정수 0..2^64−1 | 네이티브 잡음 흐름의 seed |
| `mcu_tick_hz` | 정수 | MCU 틱 주기(기본 `sim_mcu_tick_hz` 1000). `test_servo.servo_hz`의 배수 |
| `profile_id`, `profile_sha256` | 문자열 | 기록용 식별값 |
| `profile_overrides` | null 또는 객체 | 장면 덮어쓰기 정보. 기록용. 해석하지 않는다 |
| `geometry`, `actuator`, `pot`, `test_servo`, `mcu` | 객체 | 아래 표 |
| `injections` | 배열 | 고장 주입(9절) |

`geometry`:

| 키 | 뜻 (프로필 이름) |
|---|---|
| `steer_type` | `linkage` 또는 `axle_pivot` (`steer_type`) |
| `wheelbase_m` | 휠베이스 (`wheelbase_m`) |
| `kingpin_width_m` | 킹핀 폭 (`kingpin_width_m`) |
| `ackermann_ratio` | Ackermann 비율 a (`steer_ackermann_ratio`) |
| `wheel_mech_limit_rad` | 바퀴·방향별 기계 한계 4개, 양수 크기 (`steer_mech_limit_deg`). 키는 정확히 `left_wheel_toward_left`, `left_wheel_toward_right`, `right_wheel_toward_left`, `right_wheel_toward_right` |
| `eq_mech_limit_left_rad`, `eq_mech_limit_right_rad` | 등가각 기계 한계(계산값). 끝 멈춤 위치다 |
| `eq_soft_limit_left_rad`, `eq_soft_limit_right_rad` | 등가각 소프트 한계(계산값). 기록·검사용 |
| `link_table_rad` | 가변저항 쪽 각 → 바퀴 쪽 등가각 표 `[[x, y], ...]`, 두 열 모두 증가 (`steer_link_table`) |

`actuator`:

| 키 | 뜻 (프로필 이름) |
|---|---|
| `level` | `L0`, `L1`, `L1b` (`steer_actuator_level`). v1 구현은 `L1b`를 거부한다(종료 2). `L0`는 `test_servo` 모드에서만 되고 드라이버 고장 주입을 받지 않는다 |
| `rate_max_rad_s` | 주행 중 조향 속도 상한 (`steer_rate_max_deg_s`) |
| `rate_standstill_rad_s` | 정지 중 조향 속도 상한 (`steer_rate_standstill_deg_s`). `rate_max_rad_s` 이하 |
| `tau_s` | 모터 속도 1차 지연 (`steer_motor_tau_s`) |
| `min_pwm`, `min_pwm_standstill` | 모터가 움직이기 시작하는 PWM, 주행 중·정지 중 (`steer_min_pwm_pct`, `steer_min_pwm_standstill_pct`). 정지 중 값이 주행 중 값 이상 |
| `backlash_rad` | 놀이 전체 폭 b (`steer_backlash_deg`) |
| `backlash_wander_sd_rad`, `backlash_wander_corr_length_m` | 주행 중 놀이 안 떠돎 OU(거리 기준) (`steer_backlash_wander`) |
| `self_locking` | 조향 기어 자기 잠금 (`steer_self_locking`) |
| `release_scale`, `release_centering_per_s_per_m_s`, `release_noise_sd_rad`, `release_noise_corr_length_m` | 비자기잠금이고 구동 토크가 없을 때 복원·잡음 (`steer_release_restoring`) |
| `lr_asymmetry` | 좌우 조향 이득 차 (`steer_lr_asymmetry`) |
| `stall_torque_nm`, `load_moving_nm`, `load_standstill_nm` | L1b 토크 모델 값 |
| `zero_offset_rad` | 이 run의 0점 오차. 프로필 분포에서 seed로 뽑은 값 (`steer_zero_offset_deg`) |

`pot`:

| 키 | 뜻 (프로필 이름) |
|---|---|
| `location` | `motor`(백래시 앞) 또는 `wheel`(백래시 뒤 링크 쪽) (`pot_location`) |
| `range_rad` | 전기 회전 범위 (`pot_range_deg`) |
| `to_steer_ratio` | 가변저항 축 각 / 조향 각 (`pot_to_steer_ratio`) |
| `center_raw` | 측정각 0일 때 이상 raw (`pot_center_raw`) |
| `adc_bits` | 10 (`adc_bits`) |
| `ratiometric` | 비례 측정 여부. 기본 false (`adc_ratiometric`) |
| `supply_sag_per_pwm` | 조향 PWM 1.0당 급전 처짐 비율 (`pot_supply_sag_per_pwm`) |
| `noise_sd_counts` | 표본 잡음 sd (`pot_noise_sd_counts`) |
| `linearity_frac` | 직선성 오차 비율 (`pot_linearity_pct`/100) |
| `gain_error` | 이득 오차 (`pot_gain_error`) |
| `drift_rad_per_s` | 서서히 표류 (`pot_drift_deg_per_h`) |
| `cal_residual_rad` | 이 run의 직진 보정 잔차. `firmware` 모드(P3)에서만 쓴다 (`pot_cal_residual_deg`) |

`test_servo` (시험용 P 서보. 펌웨어가 아니다):

| 키 | 뜻 (프로필 이름) |
|---|---|
| `servo_hz` | 서보 갱신 주기 (`servo_hz`) |
| `kp_per_rad` | PWM 비율 / rad (`servo_kp_pct_per_deg` 환산) |
| `deadband_rad` | 데드밴드 (`servo_deadband_deg`) |
| `min_pwm`, `max_pwm` | 데드밴드 밖 PWM 크기의 하한·상한 (`servo_min_pwm_pct`, `servo_max_pwm_pct`) |

`mcu`: `firmware` 모드(P3)용 예약 블록이다. 서보 값, TTL, watchdog, 유지 시간, 상태 줄 주기, 구동 램프, 가변저항 필터, 결함 판정값(`faults`)이 들어 있다. 내용과 이름은 `backend_api.native_startup_config()`가 기준이다. v1 구현은 객체인지만 보고 해석하지 않는다.

## 3. 프레임 머리 (16 B)

모든 정수와 실수는 little-endian이다. 실수는 IEEE-754 binary64다. 구조체 채움(padding)은 없다. 오프셋은 바이트 단위다.

| 오프셋 | 형 | 이름 | 값 |
|---|---|---|---|
| 0 | char[4] | magic | `USTP` (55 53 54 50) |
| 4 | u16 | version | 1 |
| 6 | u16 | frame_type | 아래 표 |
| 8 | u32 | seq | READY는 0. 요청은 1부터 하나씩 늘린다(STEP·RESET 모두). 응답은 요청의 seq를 그대로 쓴다 |
| 12 | u32 | payload_len | 머리 뒤 바이트 수 |

| frame_type | 이름 | 방향 | payload_len |
|---|---|---|---|
| 0x0001 | STEP_REQUEST | 부모 → 프로세스 | 48 + serial_in_len |
| 0x0002 | RESET_REQUEST | 부모 → 프로세스 | 8 |
| 0x8000 | READY | 프로세스 → 부모 | 8 + build_id_len |
| 0x8001 | STEP_RESPONSE | 프로세스 → 부모 | 80 + serial_out_len |
| 0x8002 | RESET_RESPONSE | 프로세스 → 부모 | 80 + serial_out_len |

## 4. 페이로드

### 4.1 READY (seq 0)

| 오프셋 | 형 | 이름 | 뜻 |
|---|---|---|---|
| 0 | u32 | mcu_tick_hz | 시작 줄 값과 같다 |
| 4 | u32 | build_id_len | 0..256 |
| 8 | u8[] | build_id | ASCII. 예: 소스 SHA 앞 8자리 + 컴파일러 이름·버전 |

### 4.2 STEP_REQUEST (48 B + serial_in) ↔ `SteerStepInput`

| 오프셋 | 형 | 이름 | 규칙 |
|---|---|---|---|
| 0 | f64 | dt_s | 0 < dt_s ≤ 1 |
| 8 | f64 | steer_pwm | −1..1. `direct_pwm`에서만 0이 아닐 수 있다 |
| 16 | f64 | target_delta_eq_rad | \|값\| < π/2. `test_servo`에서만 0이 아닐 수 있다 |
| 24 | f64 | vehicle_speed_m_s | 유한값. 플랜트 참값(앞 +) |
| 32 | f64 | supply_scale | 0 < 값 ≤ 2. 플랜트 쪽 급전 배율(구동 PWM 처짐, 급전 고장) |
| 40 | u8 | mode | 0 `direct_pwm`, 1 `test_servo`, 2 `firmware`. 시작 줄 mode와 같아야 한다 |
| 41 | u8 | steer_en | 0 또는 1. `firmware`에서는 1이어야 한다(MCU가 정한다) |
| 42 | u8 | vehicle_stopped | 0 또는 1. 플랜트 참값 |
| 43 | u8 | reserved | 0 |
| 44 | u32 | serial_in_len | 0..4096. `firmware`가 아니면 0 |
| 48 | u8[] | serial_in | 호스트 → MCU 직렬 바이트(P3) |

### 4.3 RESET_REQUEST (8 B)

| 오프셋 | 형 | 이름 | 규칙 |
|---|---|---|---|
| 0 | f64 | delta_eq_rad | 바퀴 쪽 등가각(0점 오차 포함). 등가각 기계 한계 안 |

### 4.4 STEP_RESPONSE / RESET_RESPONSE (80 B + serial_out) ↔ `SteerStepOutput`

| 오프셋 | 형 | 이름 | 뜻 |
|---|---|---|---|
| 0 | f64 | sim_time_s | ticks_total / mcu_tick_hz |
| 8 | f64 | delta_left_rad | 참 왼쪽 바퀴각 |
| 16 | f64 | delta_right_rad | 참 오른쪽 바퀴각 |
| 24 | f64 | delta_eq_rad | 참 바퀴 쪽 등가각 |
| 32 | f64 | motor_side_angle_rad | 참 모터 쪽 각(백래시 앞, 등가각 눈금) |
| 40 | f64 | applied_steer_pwm | 참값. 드라이버의 실제 출력 PWM, −1..1. EN·드라이버 고장(`driver_dead`, `driver_stuck_on`) 반영, 배선 극성 반영 전. L0에서는 PWM이 아니라 정규화한 노력(한 틱 이동 / (rate × dt)). MCU가 아는 값이 아니다 |
| 48 | f64 | drive_duty_left | 예약(`firmware`). 그 밖에는 0 |
| 56 | f64 | drive_duty_right | 예약(`firmware`). 그 밖에는 0 |
| 64 | u32 | ticks | 이 스텝에서 돈 MCU 틱 수. RESET 응답은 0 |
| 68 | u16 | pot_raw | 마지막 ADC 표본, 0..1023(거르기 전) |
| 70 | u16 | fault_bits | MCU 결함 비트(9절). `firmware`가 아니면 0 |
| 72 | u16 | model_flags | 참값 쪽 상태 비트(9절) |
| 74 | u8 | steer_en | 실제 조향 EN, 0 또는 1 |
| 75 | u8 | drive_en | 예약(`firmware`). 그 밖에는 0 |
| 76 | u32 | serial_out_len | 0..4096. `firmware`가 아니면 0 |
| 80 | u8[] | serial_out | MCU → 호스트 직렬 바이트(P3) |

참값 칸(`delta_left_rad`, `delta_right_rad`, `delta_eq_rad`, `motor_side_angle_rad`, `applied_steer_pwm`, `model_flags`; `backend_api.TRUTH_OUTPUT_FIELDS`)은 플랜트·센서 합성·평가에만 간다. 실행기·추정기와 MCU식 상태(`SteeredPlant.steer_status()`)에는 주지 않는다(설계 1.4 규칙 1, 2.6, A13). MCU식 상태의 PWM은 명령 PWM이다.

### 4.5 예 (hex)

`direct_pwm`, dt 0.01 s, steer_pwm 0.5, EN 1, 차속 0.4 m/s, 주행 중, supply 1.0, seq 1:

```
55 53 54 50 01 00 01 00 01 00 00 00 30 00 00 00
7b 14 ae 47 e1 7a 84 3f 00 00 00 00 00 00 e0 3f
00 00 00 00 00 00 00 00 9a 99 99 99 99 99 d9 3f
00 00 00 00 00 00 f0 3f 00 01 00 00 00 00 00 00
```

RESET 0.0, seq 1: `55 53 54 50 01 00 02 00 01 00 00 00 08 00 00 00 00 00 00 00 00 00 00 00`

## 5. 시간과 틱

- 프로세스는 RESET 뒤로 받은 dt_s의 합 `plant_time`(double, 받은 순서대로 더함)과 `ticks_total`을 센다.
- STEP마다: `plant_time += dt_s`, `target = llround(plant_time × mcu_tick_hz)`, `ticks = target − ticks_total`, 틱 `ticks`개를 돌린다, `ticks_total = target`.
- dt_s × mcu_tick_hz가 정수이고 dt가 일정하면 매 스텝 `ticks = round(dt_s × mcu_tick_hz)`다. F-SIM 10 ms, 1 kHz면 10이다. G-SIM 1/120 s, 1 kHz면 8, 8, 9가 반복된다(설계 P6a). 틱 수를 매 스텝 따로 반올림하면 G-SIM에서 MCU 시계가 약 4 % 늦어진다. 그래서 누적 규칙을 쓴다.
- 한 틱의 순서: (1) 조향 명령 PWM을 정한다(`test_servo`는 서보 주기마다 다시 계산), (2) 구동기를 1/mcu_tick_hz만큼 적분한다, (3) ADC 표본 하나를 만든다.
- 응답 값은 마지막 틱 뒤 상태다. 틱이 0개인 스텝은 상태를 그대로 돌려준다.
- RESET: `plant_time = 0`, `ticks_total = 0`. 모든 상태를 처음으로 되돌리고 잡음 흐름을 시작 줄 seed로 다시 시작한다(같은 seed면 같은 run). 고장 주입의 `at_s`는 RESET 시각부터 센다. 모터 쪽 각 = 링크 쪽 각(놀이 가운데), 모터 속도 0, 가변저항 상태를 새로 만든다.

## 6. 모드

- `direct_pwm`: 드라이버 명령은 요청의 steer_pwm과 steer_en이다.
- `test_servo`: 하네스의 시험용 P 서보가 서보 주기마다 PWM을 정한다. 펌웨어 로직이 아니다.
  - 측정각 m = T(가변저항 쪽 참각)(잡음 없음, 0점 오차 모름). 오차 e = target − m.
  - \|e\| ≤ deadband_rad이면 0. 아니면 부호(e) × min(max_pwm, max(min_pwm, kp_per_rad × \|e\|)).
  - steer_en은 요청 값을 쓴다.
  - 모터는 \|u\| > 구동기 최소 PWM일 때만 움직인다(7절 L1). 데드밴드 바로 밖의 가장 작은 출력 max(min_pwm, kp_per_rad × deadband_rad)가 구동기 최소 PWM(주행 중·정지 중) 이하이면 서보는 데드밴드 밖에서 선다. 프로필 `warnings()`가 이 경우를 알린다.
- `firmware`: P3용 예약이다. serial_in 바이트를 MCU UART로 넣고 MCU가 낸 바이트를 serial_out으로 돌려준다. 바이트를 틱에 나누는 방법은 P3에서 정하고 이 규약의 v2로 올린다. v1 구현은 시작 줄에서 거부한다(종료 2).

## 7. 각과 측정의 정의 (v1)

모든 각은 조향 등가각 눈금(rad, 왼쪽 +)이다.

- θ_m: 모터 쪽 각(감속기 출력, 백래시 앞). 응답의 `motor_side_angle_rad`.
- θ_l: 링크 쪽 각. 놀이 폭 b에서 \|θ_m − θ_l\| ≤ b/2. 모터가 놀이 끝에서 밀 때만 링크가 따라간다. 주행 중에는 놀이 안에서 떠돈다(backlash_wander, 거리 기준 OU).
- 바퀴 쪽 등가각: δ_eq = T(θ_l) × (1 + lr_asymmetry × s) + zero_offset_rad. T는 `link_table_rad` 조각 선형(끝 밖은 끝 구간으로 연장), s = T(θ_l)의 부호(0이면 0).
- 끝 멈춤: δ_eq ∈ [−eq_mech_limit_right_rad, +eq_mech_limit_left_rad]. 끝에 닿으면 링크가 멈춘다. 모터 쪽은 놀이 안에서만 더 움직인다.
- 좌우 바퀴각: `linkage`이면 k = kingpin_width_m / (2 × wheelbase_m), δ_L,ideal = atan2(sin δ, cos δ − k sin δ), δ_R,ideal = atan2(sin δ, cos δ + k sin δ), δ_i = δ + a(δ_i,ideal − δ) (δ = δ_eq, a = ackermann_ratio). `axle_pivot`이면 δ_L = δ_R = δ_eq. Python 기준: `geometry.wheel_angles()`.
- 구동 방향: 배선이 맞으면 양의 applied_steer_pwm이 θ_m을 + 쪽으로 민다. `polarity_reversed` 주입이면 반대다.
- 속도 상한과 최소 PWM: vehicle_stopped = 1이면 `rate_standstill_rad_s`, `min_pwm_standstill`, 아니면 `rate_max_rad_s`, `min_pwm`. `slow` 주입은 두 속도 상한에 rate_scale을 곱한다.
- L1: 정상 모터 속도 = 부호(u) × rate × (\|u\| − min) / (1 − min) (\|u\| > min일 때), 아니면 0. 1차 지연 tau_s. u는 applied_steer_pwm에 배선 극성을 반영한 값이다.
- L0(`test_servo` 전용): 가변저항 쪽 참각이 T⁻¹(목표)를 1차 지연 tau_s로 따른다. 한 틱 이동은 rate × dt로 자른다. 백래시·끝 멈춤·0점 오차·좌우 차는 그대로 둔다. L1b는 v1에서 거부한다(종료 2).
- 구동 토크가 없을 때(EN 0, applied_steer_pwm 0, `driver_dead`):
  - `self_locking` false: 바퀴가 외력으로 움직인다. 주행 중 dθ_l/dt = −release_scale × release_centering_per_s_per_m_s × \|v\| × (θ_l − θ_s) + 잡음. θ_s는 바퀴가 곧은(δ_eq = 0) 링크 각이다(0점 오차가 0이면 0, 아니면 0이 아니다). 잡음은 거리 기준 OU 과정(sd release_noise_sd_rad, 상관 거리 release_noise_corr_length_m)의 증분이다. 거리 ds마다 θ_l ← θ_s + (θ_l − θ_s) × exp(−k × ds) + Δn, k = release_scale × release_centering_per_s_per_m_s. 모터 쪽은 놀이 끝에서 역구동으로 같이 움직인다. 서 있으면 움직이지 않는다.
  - `self_locking` true: 모터 쪽이 그대로다. 링크 쪽은 놀이 안에서만 움직인다.
- 가변저항 측정각 θ_p: location `motor`이면 θ_m, `wheel`이면 θ_l. 여기에 drift_rad_per_s × sim_time을 더한다. `pot_reversed`는 θ_p의 부호를 바꾼다. `pot_slip`은 offset_rad를 더한다. `pot_drift`는 rate_rad_per_s × (t − at_s)를 더한다.
- raw (설계 3.4의 식을 전기 시작점에서 잰 각으로 쓴 것):
  - N = 2^adc_bits, g = N × to_steer_ratio × (1 + gain_error) / range_rad
  - r0 = center_raw + g × θ_p, r1 = r0 + linearity_frac × N × sin(2π (r0 − center_raw) / N)
  - s_v = supply_scale × (1 − supply_sag_per_pwm × \|applied_steer_pwm\|)
  - ratiometric false: r2 = s_v × r1. true: r2 = r1. 1 % 처짐이면 가운데(512) 근처에서 약 5 LSB 바뀐다.
  - linearity 항은 전 범위(N count)에 사인 한 주기다. 조향 범위(자리값 ±28°, 약 ±106 count)에서는 굽음보다 국소 이득 변화로 크게 나타난다: 중앙 기울기 × (1 + 2π × linearity_frac), 2 %면 약 1.126배다. 그래서 linearity_frac은 gain_error와 섞인 항으로 해석한다.
  - raw = clamp(round(r2 + ε), 0, N − 1), ε ~ N(0, noise_sd_counts) 표본마다. `pot_noise_x10`은 sd에 scale을 곱한다. `pot_open`은 0, `pot_short`은 N − 1, `pot_stuck`은 at_s의 값을 그대로, `pot_jump`는 duration_s 동안 counts를 더한다.

## 8. 오류와 종료 코드

오류가 나면 프로세스는 stderr에 진단 한 줄(`steer_native: <이름>: <내용>`)을 쓰고 아래 코드로 끝난다. 부모는 같은 규칙으로 응답을 검사한다(`backend_api.decode_*`가 `ProtocolError.exit_code`로 같은 번호를 낸다).

| 코드 | 이름 | 때 |
|---|---|---|
| 0 | ok | 프레임 경계에서 stdin EOF. 시작 줄 첫 바이트 전 EOF |
| 2 | bad_startup | 시작 줄 JSON 오류, 모르는·빠진 키, 형·범위 오류, protocol 불일치, v1이 못 하는 mode·level(`firmware`, `L1b`), `L0`의 제한(`test_servo` 전용, 드라이버 고장 주입 거부), 정지 중 값 순서 위반, 가변저항 이득 범위 밖, 모르는 주입 종류, 시작 줄 도중 EOF, 64 KiB 초과 (2절) |
| 3 | bad_header | magic, version, 모르는 frame_type, 요청 자리에 온 응답·READY 종류 |
| 4 | sequence_mismatch | seq가 기대값과 다름 |
| 5 | bad_length | payload_len이 종류에 맞지 않음, serial 길이 4096 초과, 길이 칸과 실제 길이 불일치 |
| 6 | bad_value | 유한하지 않은 값, 범위 밖 값, 시작 줄과 다른 mode, reserved ≠ 0, 0·1이 아닌 불 칸, 규칙을 어긴 칸(4.2), RESET 각이 기계 한계 밖 |
| 7 | truncated_frame | 머리나 페이로드 도중 EOF |
| 8 | io_error | stdin 읽기 실패, stdout 쓰기 실패(파이프 끊김 등) |
| 9 | internal_error | 모델이 NaN을 내는 등 내부 오류(가변저항 계산값이 수가 아님 포함. 시작 줄 검사가 막으므로 정상 입력으로는 생기지 않는다) |

## 9. 비트와 주입

`fault_bits` (MCU 결함, 설계 4.1 결함 표, 상태 줄 `fl`과 같은 배치. `firmware` 모드에서만 0이 아닐 수 있다):

| 비트 | 이름 | 래치 |
|---|---|---|
| 0 | NOCAL | 예 |
| 1 | POT_OPEN | 예 |
| 2 | POT_SHORT | 예 |
| 3 | POT_JUMP | 예 |
| 4 | POT_NOISE | 예 |
| 5 | POLARITY | 예 |
| 6 | DRV_STUCK | 예 |
| 7 | SOFTLIMIT | 아니오(경고) |
| 8 | STALL | 예 |
| 9 | TRACK | 예 |
| 10 | THERMAL | 예 |
| 11 | TTL | 아니오(계수) |
| 12 | WATCHDOG | 예 |
| 13 | CRC | 예 |
| 14 | BOOT | 정보 |
| 15 | 예약 | 0 |

`model_flags` (참값 쪽, 플랜트와 평가 전용):

| 비트 | 이름 | 뜻 |
|---|---|---|
| 0 | END_STOP_LEFT | 바퀴 쪽 등가각이 왼쪽 기계 한계에 닿음 |
| 1 | END_STOP_RIGHT | 오른쪽 기계 한계에 닿음 |
| 2 | IN_BACKLASH | 모터 쪽이 놀이 안에 있고 링크를 밀지 않음 |
| 3 | FREE_WHEELING | 구동 토크가 없고 자기 잠금도 아님 |
| 4 | INJECTION_ACTIVE | 고장 주입이 하나 이상 켜져 있음 |
| 5 | POT_CLIPPED | 마지막 ADC 표본이 0 또는 N − 1로 잘림 |
| 6..15 | 예약 | 0 |

`injections` 항목은 `{"kind": 문자열, "at_s": 수(RESET부터 s, ≥ 0), "params": 객체}`다. 켜지면 RESET 때까지 유지한다(`pot_jump`만 duration_s 뒤 꺼진다).

| kind | params | 뜻 |
|---|---|---|
| `jam` | 없음 | 모터 쪽 각이 at_s부터 멈춤(PWM은 그대로 나감) |
| `slow` | `rate_scale` (0 < 값 ≤ 1) | 조향 속도 상한에 곱함 |
| `driver_dead` | 없음 | 드라이버 출력이 늘 0 |
| `driver_stuck_on` | `pwm` (−1..1) | EN이 켜져 있는 동안 출력이 이 값에 고착 |
| `polarity_reversed` | 없음 | 모터 배선 반대 |
| `pot_open` | 없음 | raw 0 |
| `pot_short` | 없음 | raw N − 1 |
| `pot_stuck` | 없음 | raw가 at_s 값에 멈춤 |
| `pot_noise_x10` | `scale` (> 0) | 잡음 sd × scale |
| `pot_jump` | `counts`(정수), `duration_s` (> 0) | duration 동안 raw + counts |
| `pot_slip` | `offset_rad` | 측정각 + offset(축 미끄럼) |
| `pot_drift` | `rate_rad_per_s` | 측정각 + rate × (t − at_s) |
| `pot_reversed` | 없음 | 측정각 부호 반대 |

## 10. 구현 메모

- Windows(MinGW)에서는 stdin·stdout을 이진 모드로 바꾼다(`_setmode(_fileno(stdin), _O_BINARY)`, stdout도). 그러지 않으면 0x0A 바이트가 CRLF로 바뀌어 프레임이 깨진다.
- 응답 프레임마다 stdout을 flush한다.
- 읽기는 정한 바이트 수를 다 받을 때까지 되풀이한다. 값은 `memcpy`로 옮긴다. 구조체 배치에 기대지 않는다.
- 결정성(A8): 같은 빌드·같은 플랫폼에서 같은 시작 줄과 같은 요청열이면 응답 바이트가 같아야 한다.
- 잡음 흐름은 seed에서 만든다. 잡음 원천마다(가변저항 잡음, 놀이 떠돎, 복원 잡음) 흐름을 따로 두면 한 원천을 바꿔도 다른 원천의 값이 밀리지 않는다.
- 프레임 배치를 바꾸면 version과 protocol 문자열을 올린다(v2).

## 11. v2: firmware 모드

- 작성: 2026-11-09 (P3 규격). 근거: `ESP_PROTOCOL_V3_DRAFT_KO.md`(이하 "V3 규격"), 설계 2.3.1·2.5, 이 문서 1~10절.
- 상태: 규격이다. 구현은 `sim/ubts/steering/native/`(`lockstep_main.cc`, `fw_session.cc`, `arduino_shim/`)와 Python 연결 `actuator_link.FirmwareSteeringBackend`, `native/lockstep_v2.py`다. 구현 메모는 `FIRMWARE_V3_KO.md`다. v1(1~10절)은 그대로 유효하다. v2는 v1을 넓히고 v1 모드(`direct_pwm`, `test_servo`)의 페이로드 뜻을 바꾸지 않는다.
- v2에서 MCU 펌웨어(V3 규격의 C++ 소스)는 같은 하위 프로세스에서 SIM HAL(`millis`, `analogRead`, `Serial`, `EEPROM`, `ESP.*` 흉내) 위에 돈다. 구동기·가변저항 모형은 v1 그대로다.

### 11.1 판과 호환

- 시작 줄 `protocol`이 `"ubts.steer_lockstep.v1"`이면 v1 규칙 그대로다(머리 version 1, `firmware` 모드 거부). v2를 아는 구현도 v1 시작 줄을 받는다.
- `"ubts.steer_lockstep.v2"`이면 모든 프레임 머리의 version이 2다. 시작 줄과 다른 version의 프레임은 종료 3이다.
- v2의 `direct_pwm`, `test_servo`: 프레임 종류와 페이로드는 v1과 같다(머리 version만 2). 동작도 v1과 같다.
- v2의 `firmware`: RESET_REQUEST(0x0002)와 새 프레임 FW_STEP_REQUEST(0x0003), FW_STEP_RESPONSE(0x8003)를 쓴다. STEP_REQUEST(0x0001)는 쓰지 않는다.
- 종료 코드는 8절 표 그대로다(11.9절에 v2 사례).

### 11.2 시작 줄 (v2)

- 맨 위 키 = v1 키 전부 + `firmware`. v2에서는 `firmware`가 꼭 있어야 한다: `mode`가 `firmware`이면 객체, 아니면 `null`. 모르는 키는 종료 2.
- `mode`가 `firmware`이면 `mcu_tick_hz`는 1000이어야 한다(MCU 틱 = 1 ms = `millis()` 1). 아니면 종료 2.
- `test_servo` 블록은 v1처럼 있어야 한다. firmware 모드에서는 쓰지 않는다.
- `mcu` 블록: firmware 모드에서 네이티브가 읽는 것은 `mcu.baud`뿐이다. 펌웨어에 컴파일된 baud(V3 규격 10.6절)와 다르면 종료 2. 나머지 `mcu` 값은 펌웨어에 주지 않는다. 실기 MCU가 아는 것은 EEPROM, 컴파일 상수, 받은 명령뿐이기 때문이다. 그 값은 부모가 EEPROM 이미지를 만들 때 쓴다(11.11절).
- `pot.cal_residual_rad`도 네이티브가 쓰지 않는다. 부모의 EEPROM 이미지 만들기가 중앙 표시에 쓴다(11.11절).

`firmware` 객체(키는 모두 필수):

| 키 | 형 | 뜻 |
|---|---|---|
| `expect_build` | null 또는 대문자 16진 8자리 문자열 | null이 아니면 컴파일된 펌웨어 build(V3 규격 10.5절)와 같아야 한다. 다르면 종료 2 |
| `boot_seed` | 정수 0..2^64−1 | SIM `ESP.random()` 흐름 seed(V3 규격 10.1절) |
| `chip_id` | 정수 0..16777215 | `ESP.getChipId()` 값 |
| `eeprom_hex` | null 또는 512자 16진 문자열 | RESET 때 EEPROM 256 B 이미지. null이면 모두 0xFF(빈 EEPROM) |
| `initial_delta_eq_rad` | 수 | READY 상태 = RESET(이 각). RESET 각과 같은 검사(유한, 등가 기계 한계 안) |
| `boot_delay_ticks` | 정수 0..1000 | 리셋 순간부터 `setup()`까지 틱 수 |
| `millis_start` | 정수 0..4294967295 | 리셋 순간의 `millis()` |
| `boot_garbage_hex` | 짝수 길이 16진 문자열, 0..512자 | 리셋·전원 켬마다 리셋 틱에 내는 바이트(부트 롬 출력 흉내). 빈 문자열이면 없음 |
| `adc_mode` | `"model"` 또는 `"script"` | 처음 ADC 출처 |
| `adc_script_raw` | 정수 0..1023 | 처음 대본 ADC 값(`adc_mode`가 `script`일 때 쓴다) |
| `motor_power` | 불 | 처음 조향 모터 급전 |
| `report_adc` | 불 | 참이면 변화 목록에 `adc_read` 채널을 싣는다 |
| `report_servo` | 불 | 참이면 변화 목록에 서보 평가 채널 11~14를 싣는다(11.6절). 2026-11-09 3단계(T1 장치)에서 더했다. 거짓이면 응답 바이트는 더하기 전과 같다 |

### 11.3 프레임 종류 (v2)

| frame_type | 이름 | 방향 | 쓰는 모드 | payload_len |
|---|---|---|---|---|
| 0x0001 | STEP_REQUEST | 부모 → 프로세스 | `direct_pwm`, `test_servo` | 48 + serial_in_len(v1, serial_in_len 0) |
| 0x0002 | RESET_REQUEST | 부모 → 프로세스 | 모두 | 8 |
| 0x0003 | FW_STEP_REQUEST | 부모 → 프로세스 | `firmware` | 32 + 8·event_count + 4·rx_count |
| 0x8000 | READY | 프로세스 → 부모 | 모두 | 8 + build_id_len |
| 0x8001 | STEP_RESPONSE | 프로세스 → 부모 | `direct_pwm`, `test_servo` | 80(v1) |
| 0x8002 | RESET_RESPONSE | 프로세스 → 부모 | 모두 | 80(v1 배치, serial_out_len 0) |
| 0x8003 | FW_STEP_RESPONSE | 프로세스 → 부모 | `firmware` | 96 + 8·change_count + 12·tx_count |

- 모드에 맞지 않는 요청 종류(firmware 모드의 0x0001, 다른 모드의 0x0003)는 종료 3이다.
- seq 규칙, 엄격한 번갈아 하기, EOF, 응답 기한은 v1(1·3절)과 같다.

### 11.4 FW_STEP_REQUEST 페이로드

| 오프셋 | 형 | 이름 | 규칙 |
|---|---|---|---|
| 0 | f64 | dt_s | 0 < dt_s ≤ 0.1 |
| 8 | f64 | vehicle_speed_m_s | 유한값. 플랜트 참값(앞 +) |
| 16 | f64 | supply_scale | 0 < 값 ≤ 2. 가변저항 급전 배율(v1 4.2절과 같은 뜻) |
| 24 | u8 | vehicle_stopped | 0 또는 1 |
| 25 | u8 | reserved | 0 |
| 26 | u16 | event_count | 0..64 |
| 28 | u32 | rx_count | 0..4096 |
| 32 | 사건 × event_count | events | 항목 8 B: u16 tick_offset, u16 kind, u32 arg |
| 32 + 8·E | 수신 × rx_count | rx | 항목 4 B: u16 tick_offset, u8 byte, u8 reserved(0) |

- 이 스텝의 틱 수 N은 v1 5절 누적 규칙으로 정해진다(부모도 같은 규칙으로 계산한다). tick_offset은 0..N−1이다. 스텝의 첫 틱 번호 a = 스텝 전 ticks_total, 항목의 틱 = a + tick_offset.
- events와 rx는 각각 tick_offset이 줄지 않는 순서다. 같은 틱의 rx는 적힌 순서가 도착 순서다. N = 0이면 두 목록 모두 비어 있어야 한다.

사건 종류:

| kind | 이름 | arg | 뜻 |
|---|---|---|---|
| 1 | `mcu_reset` | 리셋 원인 1..6(V3 규격 10.3절) | 그 틱에 MCU 리셋. RAM 상태를 잃는다. EEPROM은 남는다. 핀이 뜬다 |
| 2 | `power_cycle` | 꺼진 틱 수 1..60000 | 그 틱부터 arg 틱 동안 MCU 전원 꺼짐. 그다음 틱에 전원 켬 리셋(원인 0). EEPROM은 남는다 |
| 3 | `port_open_reset` | 0 | `mcu_reset`(원인 6)과 같다. 대본에서 뜻을 드러내려고 따로 둔다 |
| 4 | `motor_power` | 0 끔, 1 켬 | 그 틱부터 조향 모터 급전. 끄면 핀과 상관없이 드라이버 출력 0 |
| 5 | `mcu_stall` | 틱 수 1..60000 | 그 틱부터 arg 틱 동안 `loop()`를 부르지 않는다(`millis()`는 흐른다, 핀은 그대로) |
| 6 | `adc_set` | raw 0..1023 | 그 틱부터 `analogRead`가 이 값을 돌려준다(대본 모드) |
| 7 | `adc_model` | 0 | 그 틱부터 `analogRead`가 가변저항 모형 값을 돌려준다 |

### 11.5 틱 처리 순서 (firmware)

스텝 안의 틱 n(= a + i, i = 0..N−1)마다:

1. 사건: tick_offset이 i인 사건을 적힌 순서로 적용한다.
   - 리셋(종류 1, 3, 2의 켜는 틱): 그 틱이 리셋 틱 r이다. MCU 상태를 지운다. 핀 값은 모두 0이고 EN은 0이다(풀다운, V3 규격 8.3절). RX 버퍼를 비운다. 아직 다 나가지 않은(done_tick ≥ r) MCU 송신 바이트를 버리고 송신선을 비운다. `boot_garbage_hex` 바이트를 틱 r에 송신 줄에 넣는다(플래그 bit0). `setup()` 틱 = r + `boot_delay_ticks`. 부팅 수(`boot_count`)는 리셋이 아니라 `setup()`을 부를 때 1 오른다(11.6절. `boot_delay_ticks` 동안 다시 리셋되면 두 리셋에 `setup()`은 한 번이다). 진행 중인 `mcu_stall`은 끝난다.
   - 전원 끔(종류 2): 틱 n..n+arg−1은 꺼짐이다. 핀 0, EN 0, 수신 바이트는 사라진다, 송신 없음. 틱 n+arg에 원인 0 리셋. 이 전원 켬 리셋은 그 틱의 사건보다 먼저 적용한다. 꺼짐 동안의 종류 1, 3, 5는 효과가 없고, 다시 온 종류 2는 꺼짐을 그 틱부터 새로 센다.
   - 종류 4, 6, 7은 그 틱부터 적용한다.
2. 수신: tick_offset이 i인 rx 바이트를 MCU RX 고리(256 B 배열, 255 B까지 담음)에 넣는다. MCU가 꺼짐이거나 리셋 뒤 `setup()` 전이면 버린다(`setup()` 틱에 도착한 바이트도 버린다: 이 2단계가 3단계의 `setup()`보다 앞이다). 고리가 차 있으면 가장 옛 바이트를 버리고 새 바이트를 넣는다(ESP8266 코어 3.1.2 `uart.cpp`는 `UART_DISCARD_NEWEST`를 정의하지 않는다. V3 규격 12.1-67). 버린 바이트는 `rx_lost_total`에 센다. 하드웨어 RX FIFO(128 B)가 ISR 전까지 더 담는 몫은 흉내 내지 않는다[가정].
3. MCU: 틱 n이 `setup()` 틱이면 `setup()` 다음 `loop()`. 아니면 `setup()`을 지났고 멈춤(`mcu_stall`) 중이 아니면 `loop()`. `setup()` 틱이 멈춤 안에 들면 `setup()`만 부르고 `loop()`는 부르지 않는다. `millis()` = (n − r + `millis_start`) mod 2^32.
4. 구동기: 조향 명령 = (D6 − D7) / 255, EN = D2. MCU가 꺼짐·리셋 중이면 0과 0. 조향 모터 급전이 꺼져 있으면 드라이버 출력은 0이다. v1 7절 구동기가 한 틱(1 ms)을 적분한다(v1 주입, 정지 여부, 차속을 그대로 쓴다).
5. ADC: v1 순서대로 틱 끝 표본 하나(표본 번호 n + 1)를 만든다. 다음 틱의 `analogRead`는 ADC 모드가 `model`이면 이 표본, `script`이면 대본 값을 돌려준다. 틱 0(RESET 뒤 첫 틱)의 `analogRead`는 RESET 표본(번호 0)이다.
6. 송신: 이 틱에 펌웨어가 쓴 바이트(`setup()`, `loop()`)의 queued_tick은 n이다. 송신 시각은 11.7절이다.

- 구동 출력은 플랜트(SteeredPlant)가 쓴다: 구동 duty 왼쪽 = (D8 − D5) / 255, 오른쪽 = (D3 − D0) / 255, 구동 EN = D1. 네이티브는 구동 동역학을 돌리지 않는다.
- RESET_REQUEST(firmware 모드): v1 5절 RESET에 더해 EEPROM = 시작 줄 이미지, `ESP.random()` 상태 = `boot_seed`, 부팅 수 0, ADC 모드·값·조향 모터 급전 = 시작 줄 값, 송수신 줄 비움, 채널 값 처음으로(11.6절). MCU는 틱 0에 원인 0으로 리셋된 상태다(부트 쓰레기는 틱 0, `setup()`은 틱 `boot_delay_ticks`). RESET_RESPONSE는 v1 배치이고 ticks 0, 구동·조향 출력 0, fault_bits 0, serial_out_len 0이다.
- READY 직후 상태는 RESET(`initial_delta_eq_rad`) 직후와 같다.
- SIM 단순화: lockstep 세션의 송신은 UART FIFO 128 B 한도를 흉내 내지 않는다. SIM의 `Serial.availableForWrite()`는 16384 − (아직 다 나가지 않은 송신 바이트 수)다. 펌웨어는 12단계(V3 규격 7.1절)에서 `availableForWrite()` − 1 바이트까지만 옮기므로(12.1-66) 송신 고리는 보통 그 틱에 다 비고, 바이트는 `mcu_stall` 중에도 이어서 나간다. EEPROM commit은 0틱이다(V3 규격 12.1절 26). SIM의 `EEPROM.commit()`은 ESP8266 코어처럼 바뀐 바이트가 있을 때만 플래시를 쓰고 그때만 `eeprom_commits`를 올린다.
- 시험 선택(C++ 단위 시험만, lockstep 시작 줄에는 없다): `SimMcu::set_tx_fifo_model(true)`는 ESP8266 코어 3.1.2의 송신 규칙(FIFO 128 B, `availableForWrite()` = 128 − 채운 수, 채운 수가 127 이상이면 `Serial.write`가 기다림)을 흉내 낸다. 기다림은 실제로 돌지 않고 틱 안 CPU 시간에 더해 `tx_block_events()`·`tx_block_units()`로 세며, 그 틱 뒤의 핀 쓰기 시각(`pin_write_units()`)에 나타난다. `set_eeprom_commit_fail(true)`는 바뀐 바이트의 commit을 실패시킨다(플래시 그대로). `fw_unit_test.cc`의 `test_tx_fifo_model`, `test_eeprom_commit_failure`가 쓴다.

### 11.6 FW_STEP_RESPONSE 페이로드

| 오프셋 | 형 | 이름 | 뜻 |
|---|---|---|---|
| 0..79 | v1 4.4절 배치 | 기본 칸 | 아래 뜻 |
| 80 | u16 | change_count | 0..4096 |
| 82 | u16 | tx_count | 0..4096 |
| 84 | u8 | mcu_state | 마지막 틱 뒤 MCU 상태 0..3, 255는 돌지 않음(꺼짐, 리셋 뒤 `setup()` 전) |
| 85 | u8 | run_flags | bit0 전원, bit1 `setup()` 지남, bit2 멈춤 중, bit3 조향 모터 급전. 나머지 0 |
| 86 | u16 | boot_count | RESET 뒤 `setup()`을 부른 수 |
| 88 | u32 | mcu_millis | 마지막 틱의 `millis()`. 돌지 않으면 0xFFFFFFFF |
| 92 | u32 | rx_lost_total | RESET 뒤 버린 수신 바이트 수 |
| 96 | 변화 × change_count | changes | 항목 8 B: u16 tick_offset, u8 channel, u8 reserved(0), i32 value |
| 96 + 8·C | 송신 × tx_count | tx | 항목 12 B: u32 queued_tick, u32 done_tick, u8 byte, u8 flags, u16 reserved(0) |

기본 칸의 firmware 모드 뜻:

- `sim_time_s`, `ticks`, `delta_left_rad`, `delta_right_rad`, `delta_eq_rad`, `motor_side_angle_rad`, `applied_steer_pwm`, `model_flags`: v1과 같다(참값 칸). `applied_steer_pwm`은 EN, 드라이버 고장, 조향 모터 급전을 반영한 실제 드라이버 출력이다.
- `drive_duty_left`, `drive_duty_right`: 마지막 틱의 (D8 − D5) / 255, (D3 − D0) / 255. `drive_en`: 마지막 틱의 D1. `steer_en`: 마지막 틱의 D2. 돌지 않으면 모두 0.
- `pot_raw`: 다음 틱 `analogRead`가 돌려줄 값(모형 표본 또는 대본 값).
- `fault_bits`: 마지막 틱 뒤 MCU 결함 마스크(V3 규격 5.3절 `fl`). 돌지 않으면 0.
- `serial_out_len`(오프셋 76): 0. MCU 송신 바이트는 tx 목록에 있다.

변화 채널:

| 번호 | 이름 | 값 |
|---|---|---|
| 1 | `drive_l` | D8 − D5, −255..255 |
| 2 | `drive_r` | D3 − D0 |
| 3 | `drive_en` | D1, 0 또는 1 |
| 4 | `steer_pwm` | D6 − D7, −255..255 |
| 5 | `steer_en` | D2 |
| 6 | `mcu_state` | 0..3, 255(돌지 않음) |
| 7 | `fault_bits` | 0..0xFFFF(돌지 않으면 0) |
| 8 | `gpio_writes` | RESET 뒤 `analogWrite`·`digitalWrite` 호출 수(u32, 감김) |
| 9 | `eeprom_commits` | RESET 뒤 EEPROM commit 수 |
| 10 | `adc_read` | 그 틱 `analogRead`가 돌려준 값(`report_adc`가 참일 때만) |
| 11 | `servo_pos` | 펌웨어 RAM의 거른 위치 P[n](천분율, `report_servo`가 참일 때만) |
| 12 | `servo_tgt` | 서보 목표 tgt(천분율, 같음) |
| 13 | `servo_cmd` | 속도 제한 뒤 내부 목표 cmd(천분율, 같음) |
| 14 | `steer_mode` | 조향 모드 0 OFF, 1 ACTIVE, 2 HOLD, 3 JOG, 4 POL(같음) |

- 틱의 값은 그 틱 처리(3단계 MCU, 꺼짐·리셋이면 강제 0) 뒤 값이다. 곧 그 틱 동안 구동기에 걸린 값이다. 강제 0은 핀 채널(1~5), `fault_bits`, `adc_read`에 적용하고 `mcu_state`는 255다. `gpio_writes`, `eeprom_commits`는 RESET 뒤 누적값이라 꺼짐·리셋 중에도 그대로다. 멈춤(`mcu_stall`) 중에는 핀과 `adc_read`가 앞 값을 유지한다.
- RESET 직후 채널 값은 모두 0이고 `mcu_state`만 255다. 각 틱에서 앞 틱(스텝 첫 틱이면 앞 스텝 마지막 틱, RESET 뒤 첫 틱이면 RESET 값)과 다른 채널마다 항목 하나를 싣는다. 순서는 tick_offset, 그다음 채널 번호다.
- tx 목록: done_tick이 이 스텝의 틱 [a, a + N) 안에 드는 바이트를 송신 순서대로 싣는다. queued_tick ≤ done_tick이다(앞 스텝에 넣은 바이트일 수 있다). flags bit0 = 부트 쓰레기 바이트(펌웨어가 쓴 것이 아님), 나머지 0.
- 변화 채널과 run 칸은 평가·적합성 전용이다(참값 칸과 같은 취급). 실행기·추정기에는 주지 않는다. 호스트는 직렬 바이트만 받는다. `mcu_state`, `fault_bits`, `gpio_writes`, `eeprom_commits`, `adc_read`는 MCU 내부 값이라 실기에서는 직렬 줄로만 보인다.
- 채널 11~14(`report_servo`)는 네이티브 세션이 틱 처리 뒤 펌웨어 RAM(`fw.pos`, `fw.tgt`, `fw.cmd`, `fw.mode`)을 읽어 낸다. 펌웨어 소스는 바꾸지 않았다(build id 그대로). 꺼짐·리셋 중에는 0이고, 멈춤 중에는 앞 값을 유지한다. 실기에서는 상태 줄의 `pos`, `tgt`, `cmd`로만 보인다(20 Hz). 쓰는 곳: T1 장치(`T1_RIG_KO.md`)의 평가 칸(유지 위치 칸 `truth_servo_view_rad`). 상태 줄이 나간 틱에서는 상태 줄 값과 같다(`test_steering_firmware.py::test_report_servo_channels`). 적합성 대본은 이 채널을 쓰지 않는다(실기 벤치에서 볼 수 없다).

### 11.7 UART 시각 규칙

- 시간 단위 u = 1/576000 s. 1틱(1 ms) = 576 u. 바이트 하나(10비트) = 5,760,000 / baud u: 115200이면 50 u, 230400이면 25 u.
- MCU 송신: 송신 순서의 바이트 k(queued_tick q_k)는 E_k = max(q_k · 576, E_{k−1}) + 50에 끝난다(RESET과 MCU 리셋 때 E는 그 틱 시작으로 돌아간다). done_tick_k = ceil(E_k / 576) − 1, 곧 그 바이트가 끝나는 틱이다.
- 호스트 → MCU(부모가 도착 틱을 정하는 규칙): 호스트가 바이트 k를 시각 h_k(u)에 쓰고 전송 지연이 d(u, `serial_delay_ms` × 576)이면 E_k = max(h_k + d, E_{k−1}) + 50. 도착 틱 = ceil(E_k / 576). 그 틱이 든 스텝의 요청에 tick_offset = 도착 틱 − a로 넣는다. 도착 틱에 그 바이트는 MCU `loop()` 4단계(V3 규격 7.1절) 전에 RX 버퍼에 있다.
- 예: 상태 줄 85 B를 틱 100에 넣으면 첫 바이트 done_tick 100, LF done_tick 107. 호스트가 틱 0 시작(h = 0, d = 0)에 `C` 32 B를 쓰면 도착 틱은 바이트 0..10이 1, 11..22가 2, 23..31이 3이다.

### 11.8 READY (v2)

- 배치는 v1 4.1절과 같다. build id: `ubts-steer-native/2 src=h<소스 SHA 앞 12자리> fw=<펌웨어 build 8자리> cxx=gcc-<버전> bits=<32|64>`. 네이티브 실행 파일에 펌웨어가 들어 있으므로 v2 시작 줄이면 모든 모드에서 `fw=`를 싣는다. v1 시작 줄이면 v1 형식(`ubts-steer-native/1 src=h… cxx=… bits=…`, `fw=` 없음)을 그대로 쓴다(1.~10절 v1 규칙). `src=`의 소스 SHA는 네이티브 소스·흉내 층·펌웨어 소스를 모두 덮는다. 펌웨어 소스 폴더가 없는 곳(예: `sim/ubts`만 복사한 이미지)에서 빌드한 실행 파일은 firmware 모드가 없고 `fw=none`을 싣는다. 이때 v1 모드와 v2 direct_pwm·test_servo 모드의 응답 바이트는 펌웨어가 들어 있는 실행 파일과 같다(READY build id만 다르다).

### 11.9 오류 (v2 사례)

| 코드 | v2에서 더한 경우 |
|---|---|
| 2 | `firmware` 키 없음·형 틀림, firmware 모드인데 `mcu_tick_hz` ≠ 1000, `mcu.baud` ≠ 컴파일 값, `expect_build` 불일치, `eeprom_hex` 길이·문자 오류, `boot_garbage_hex` 오류, 펌웨어 소스 없이 빌드한 실행 파일에 firmware 모드 시작 줄 |
| 3 | 머리 version ≠ 시작 줄 판, 모드에 맞지 않는 요청 종류 |
| 5 | payload_len ≠ 32 + 8·event_count + 4·rx_count, 개수 한도 초과 |
| 6 | dt_s가 (0, 0.1] 밖, tick_offset ≥ N, 순서가 줄어듦, N = 0인데 항목 있음, 모르는 사건 종류, arg 범위 밖, reserved ≠ 0, 차속·급전·정지 칸 오류 |
| 9 | 펌웨어 내부 오류(펌웨어 assert 등) |

### 11.10 결정성

- 같은 빌드, 같은 플랫폼, 같은 시작 줄, 같은 틱별 입력(사건, 수신 바이트, 차속·정지·급전)이면 응답 바이트가 같다(A8).
- 틱을 스텝으로 나누는 방법만 다르고 틱별 입력이 같으면 틱별 결과(채널 변화, 송신 바이트와 두 틱, 참값)가 같아야 한다. 적합성 실행기가 이것을 시험한다(V3 규격 13.9절).

### 11.11 EEPROM 이미지 만들기 (부모 쪽 권장 규칙)

firmware 모드 SIM run은 "보정을 마친 차"로 시작하는 것이 보통이다. 부모는 아래 규칙으로 `eeprom_hex`를 만든다. 결과는 V3 규격 9.6절 예와 바이트까지 같은 형식이어야 한다(만드는 코드의 시험).

1. 표시 각(바퀴 쪽 등가각): 왼쪽 = +(`eq_mech_limit_left_rad` − 1°), 오른쪽 = −(`eq_mech_limit_right_rad` − 1°), 중앙 = 0 + 이 run의 `pot.cal_residual_rad`(중앙을 조금 틀리게 맞춘 것).
2. 각 표시 각에서 raw: v1 7절 식으로 잡음 없이 계산한다(급전 배율 1, 놀이 가운데: θ_m = θ_l, 링크 각은 δ_eq 식을 거꾸로 풀어 얻음, round는 0에서 먼 쪽). 이 값이 raw_l, raw_c, raw_r이다.
3. 눈금 s(‰/°) = 1000 / 평균(\|왼쪽 표시 각\|, \|오른쪽 표시 각\|)(°). 자리값 프로필에서 약 39.8이다.
4. soft_l = round(1000 × `eq_soft_limit_left_rad` / 왼쪽 표시 각), soft_r = −round(1000 × `eq_soft_limit_right_rad` / \|오른쪽 표시 각\|).
5. 변수 41개: `mcu` 블록(= 프로필 D·D2표)을 V3 규격 4.10절 식과 눈금 s로 바꾼다. `movms`는 300, `svhz`는 `servo_hz`.
6. cal_id는 장면 명세가 정한다(기본 1). pol_ok = 1. mdir = +1(처음부터 극성이 반대인 차를 흉내 낼 때만 −1. 주행 중 배선이 바뀌는 고장은 v1 `polarity_reversed` 주입으로 만든다).
7. V3 규격 9.2절 배치 + CRC-32로 106 B 레코드, 뒤에 0xFF 150 B → 512자 대문자 16진.
