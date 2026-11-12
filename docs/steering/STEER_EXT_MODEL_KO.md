# 조향 확장 모형: 벨트 가변저항·조향 급전·외란 토크 (ST-10, ST-11)

- 작성: 2026-11-13 (묶음 S3)
- 근거: 설계 v2 V2-6(벨트로 이은 가변저항, 일반 DC 모터, 리밋 스위치 없음, 직진 유지), 결정 R36, 사양 요청서 8절(Q22~Q29), `NATIVE_ACTUATOR_MODEL_KO.md`(L1 모형)
- 상태: 모든 값은 자리값이다. 이 모형으로 낸 결과는 자리값 기반 SIM 결과이며 현장 검증이 아니다.
- 용어: 조향 장치는 **조향 모터(일반 DC 모터)와 위치 제어**다. 조향 모터를 ESP가 모터 드라이버로 돌리고, 벨트로 이은 가변저항 값으로 위치 루프를 닫는다.

## 1. 어디에 있나

| 층 | 파일 | 내용 |
|---|---|---|
| 프로필 | `sim/ubts/steering/profiles/front_steer_v0_2_placeholder.yaml` | 스키마 v3. 새 칸은 맨 아래 I(벨트·리밋·급전), J(외란) 묶음 |
| 읽기·검사 | `sim/ubts/steering/vehicle_profile.py` | 스키마 v2(v0, v0.1)와 v3를 모두 읽는다. v3만 가변저항 범위 검사를 거부로 한다 |
| 시작 줄 | `sim/ubts/steering/backend_api.py` `ext_startup_block()` | v3 프로필이면 시작 줄에 선택 블록 `steer_ext`를 붙인다. v2 프로필이면 붙이지 않는다 |
| 네이티브 | `sim/ubts/steering/native/{steer_config,actuator,potentiometer,steer_model}.{h,cc}` | `steer_ext` 읽기, 외란 토크, 벨트 |
| 고장 주입 | `steer_torque`(새 종류), 도우미 `belt_skip_injection`·`belt_break_injection` | 대본용 |

`steer_ext`가 없으면(v2 프로필) 네이티브 모형은 이전과 바이트 하나 다르지 않다. 블록이 있어도 항목이 모두 꺼져 있으면 같다(시험 `test_neutral_ext_block_is_bit_identical`). 펌웨어 소스는 바꾸지 않았다(build id `04353B49` 그대로). 적합성 대본 92개가 이 변경 뒤에도 모두 통과했다.

빠른 SIM, ESP 펌웨어 모형(lockstep firmware 모드, T1 장치, Gazebo의 ESP 에뮬레이터)은 모두 같은 `native_startup_config()`로 시작 줄을 만든다. 그래서 같은 프로필 파일을 주면 같은 벨트·외란 모형이 들어간다. Gazebo 모델 생성(`gz_model.py`)은 기하·관절 한계를 같은 프로필에서 읽는다(7절).

## 2. 벨트 가변저항

잰 축(가변저항 위치 `pot_location`이 motor면 모터 쪽 각, wheel이면 링크 쪽 각, 조향 등가각 눈금 θ_meas)과 가변저항 축 사이에 벨트가 있다.

1. **측정 쪽 유격** `belt_pot_play_deg`(b_p): 가변저항 축 각 θ_s는 θ_meas를 폭 b_p의 놀이 연산자로 따른다. θ_s ← clamp(θ_s, θ_meas − b_p/2, θ_meas + b_p/2). 링크 백래시(`steer_backlash_deg`)와 따로다. 반전할 때 b_p만큼 가변저항이 늦게 움직인다.
2. **누적 미끄럼(마찰 벨트)** `belt_creep_sd_deg_per_100deg`(c): 가변저항 축이 움직인 거리 Δ(rad)마다 0점에 N(0, k²Δ)를 더한다. k = c / √(100°)이라 100° 움직이면 sd c°의 무작위 걸음이다. 타이밍 벨트는 0.
3. **이 건너뜀(타이밍 벨트)** `belt_skip_per_10000deg`(λ): 움직인 거리 Δ마다 확률 1 − e^(−λΔ)로 0점이 ±한 이(`belt_tooth_step_deg` = 360 / 가변저항 풀리 잇수 / 벨트 비, 자리값 18°) 옮겨 간다. 부호는 반반. `belt_type`이 timing일 때만 0이 아닐 수 있다.
4. 그 뒤는 예전 가변저항 식 그대로다(규약 7절): 이득 = 2^bits × 벨트 비 × (1 + 이득 오차) / 회전 범위, 직선성, 급전 처짐, 잡음, 고장.

난수는 새 흐름(`kStreamBeltSlip`)에서만 뽑고, 항이 꺼져 있으면 뽑지 않는다. 그래서 예전 잡음 흐름이 밀리지 않는다.

**끊어짐·벗겨짐**은 가변저항이 멈춘 것과 같다: `belt_break_injection(at_s)` = `pot_stuck`. 대본용 **이 건너뜀 하나**는 `belt_skip_injection(profile, at_s, teeth)` = `pot_slip`(offset = 이 수 × 이 각).

### 2.1 벨트 비 검사 (프로필 읽기)

`pot_to_steer_ratio`(벨트 비 = 가변저항 축 각 / 조향 각)의 범위는 0.5~5다(설계 v2 V2-6.5). 범위 안이어도 다음 둘 중 하나면 v3 프로필 읽기가 거부한다.

| 코드 | 조건 | 뜻 |
|---|---|---|
| `pot_range_exceeded` | 끝 멈춤의 이론 raw(유격·이득·직선성 포함, `pot_end_stop_raws`)가 `cal_end_raw_window`(60~963) 밖 | 조향 전 범위 × 비가 가변저항 유효 범위를 넘는다. 비를 줄이거나 회전 범위가 넓은 가변저항 |
| `pot_cal_half_width_small` | 보정 표지(등가 기계 한계 − 1°와 0)의 가운데에서의 거리 < 50 count | 펌웨어 `marks_valid`가 보정을 거부한다. 비를 키운다 |

자리값(기계 한계 28°, 270° 가변저항, 10비트)에서 받는 비는 약 0.53~4.2다. 0.5는 47.6 count라 거부된다. 계산값 `pot_travel_deg`(끝에서 끝까지 가변저항 축 회전, 자리값 54.2°)와 `pot_cal_half_width_counts`(자리값 95.2)를 프로필 표에서 본다. 스트레스 스윕은 `with_overrides(..., allow_out_of_range=True)`로 받고 경고(`warnings()`)로 남긴다.

## 3. 리밋 스위치와 조향 급전

- `steer_limit_switch`: 기본 none. both는 읽기가 거부한다(`not_modelled`). ESP8266에 남는 핀이 D4 하나이고 펌웨어 V3가 스위치를 읽지 않는다. 끝 보호는 소프트 한계, 기계 끝 멈춤, STALL·TRACK이다(설계 v2 V2-6.4).
- `steer_supply_shared_with_drive`(기본 true): 조향 모터가 구동 배터리를 같이 쓰면 `battery_voltage_scale`을 조향 속도(주행 중·정지 중)에 곱한다.

## 4. 외란 토크와 되밀림

조향축 외력 토크 τ(N·m, + 왼쪽) = 주입 `steer_torque`의 합 + 지면 충격 + 미는 토크.

- **지면 충격** `steer_terrain_impacts`: 주행 거리에서 포아송(rate_per_m)으로 시작하는 사각 펄스. 크기 N(0, torque_sd_nm), 길이 duration_s. 새 충격이 오면 이전 것을 바꾼다. 서 있으면 생기지 않는다.
- **미는 토크** `steer_terrain_push`: 거리 기준 OU 과정(sd_nm, corr_length_m). 바퀴자국·옆 경사.
- v0.2 기본은 둘 다 꺼짐이다. ST-11 대본이 켠다.

τ는 L1 속도 식에 **등가 duty** d = τ / `steer_motor_stall_torque_nm`(자리값 5 N·m)로 들어간다. min은 주행 중·정지 중 조향 시작 PWM(정지 마찰)이다.

| 상태 | 식 |
|---|---|
| 구동 중(드라이버 출력 u ≠ 0) | u_eff = u + d |
| 제동(EN 켬, 출력 0) | 비자기잠금: u_eff = d. 자기 잠금: \|τ\|가 `steer_self_lock_breakaway_nm`(자리값 8)를 넘은 만큼만 |
| 관성(EN 끔, 전원 끊김, `driver_dead`) | 위와 같고 속도에 `steer_backdrive_coast_gain`(자리값 3)을 곱한다(전기 제동 없음) |

w_ss = sign(u_eff) × rate × (\|u_eff\| − min) / (1 − min) (\|u_eff\| > min일 때), 최대 4 × rate. τ ≠ 0이면 링크는 τ 쪽 놀이 끝에 붙는다(백래시가 닫힌다). τ = 0이면 모든 계산이 예전과 같다.

읽는 법(자리값, 주행 중 min 0.25):
- 1 N·m(d 0.2)은 정지 마찰보다 작다. 바퀴는 백래시 반쪽(1°)만 움직인다.
- 2.5 N·m(d 0.5)은 출력 0일 때 (0.5 − 0.25) / 0.75 × 30 = 10°/s로 민다. 서보 최소 출력 0.45가 걸리면 u_eff 0.05로 멈춘다. 되돌리려면 \|u\| > 0.75, 곧 서보 오차 5°(kp 15 %/deg)가 필요하다. 그래서 펌웨어의 데드밴드 안 제동 유지(0.2 s) 동안 밀린 만큼(약 2°) 오차가 남고 외란이 끝나야 돌아온다.
- 5 N·m(d 1.0)은 서보가 버티려면 \|u\| ≥ 0.75(오차 5°)가 필요하다. TRACK(3°, 1.0 s)이나 STALL(60 % 이상, 0.5 s)로 선다.

## 5. 시험 (`sim/ubts/test_steering_profile_v3.py`, 30개)

v0.2 읽기와 계산값, v2 파일 그대로 읽기, v0.1과 다른 칸이 정한 5개뿐인지, 벨트 비 받음 0.55/1/2/4와 거부 0.3/5.5/0.5/4.5/5.0, 범위 밖 덮어쓰기는 경고, 파일 수준 거부, 리밋 스위치 거부, 이 건너뜀은 timing만, 스키마별 매개변수 집합, Q29까지 식별 표지, 시작 줄 블록 값과 배터리 배율, 주입 도우미와 검사, 꺼진 블록의 바이트 동일, 벨트 유격의 반전 지연, 이 건너뜀이 이 수의 배수, 마찰 아래 토크는 백래시만, 제동·관성 되밀림 속도, 구동이 버티는 경우와 밀리는 경우, 자기 잠금 이탈 토크, L0·정지 토크 0 거부, 지면 충격은 주행 중에만·결정적, 급전 배율.

## 6. 한계

- 토크는 조향축에 걸린 순 토크로 본다. 타이어 복원 토크는 예전 풀림 모형(비자기잠금, 출력 0, 주행 중)이 따로 맡는다. 외란이 있는 틱에는 링크가 외란 쪽 놀이 끝에 붙어 풀림 이동이 덮인다.
- DC 모터 토크-속도 직선과 정지 마찰(시작 PWM)만 있다. 감속기 효율, 전류 제한, 온도는 없다(L1b는 여전히 TODO).
- 벨트 미끄럼은 가변저항 쪽 0점 이동만 모형화한다. 벨트가 조향 힘을 전달하지는 않는다(벨트는 센서용).
- 값은 모두 자리값이다. 정지 토크, 시작 PWM, 되밀림 배율이 오면 결과가 크게 바뀐다(`SPEC_UPDATE_KO.md`).

## 7. 같은 프로필을 읽는 곳 (확인)

| 경로 | 읽는 법 | 확인 |
|---|---|---|
| 빠른 SIM(`fast_sim`, `steered_plant`) | 장면의 프로필 → `NativeSteeringBackend` → `native_startup_config` | 시작 줄에 `steer_ext` |
| ESP 펌웨어 모형(lockstep firmware, T1, 적합성) | `lockstep_v2.firmware_startup_config` → 같은 함수. 펌웨어 CFG는 `fw_eeprom.params_from_profile` | T1 장치가 v0.2로 무장·주행(개발 확인) |
| Gazebo ESP 에뮬레이터(`gz_esp_emulator --profile`) | 같은 함수 | 프로필 인자 |
| Gazebo 모델(`gz_model.py --profile`) | 기하·관절 한계·질량을 같은 프로필에서 | `SPEC_UPDATE_KO.md`의 도구가 SDF를 만들어 본다 |
| 호스트 브리지(`host_profile_export --profile`) | 소프트 한계·문턱 | 같은 프로필 |
