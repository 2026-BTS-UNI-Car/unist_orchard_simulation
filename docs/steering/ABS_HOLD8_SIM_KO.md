# 실제 치수 프로필과 조향 펌웨어 ABS-HOLD-8 흉내 (2026-11-16)

- 작성: 2026-11-16 (작업 sim-realdims). 사용자 요청: SIM을 지금 잰 실제 차와 실제 조향 펌웨어에 맞추고 열 추종을 다시 잰다.
- 새 프로필: `sim/ubts/steering/profiles/front_steer_v1_spec_20261116.yaml`(스키마 v4). 입력 파일:
  `profiles/steer_spec_input_20261116.yaml`, 도구 `tools/steer_sim/apply_spec.py`(`SPEC_UPDATE_KO.md`).
- 기본 프로필(`load_profile()`)은 여전히 v0.1이다. 예전 결과가 그대로 다시 만들어지도록 바꾸지 않았다. 새 프로필은 이름으로 고른다.
- 이 프로필로 만든 수치는 **자리값 기반 SIM 결과**다(남은 SPEC_PENDING 자리값 34칸: 킹핀 폭, Ackermann, 유격, 발자국, 가변저항 사양, IMU 자세 등. 질량 같은 SIM 가정값도 남는다). 현장 성능이 아니다.

## 1. 근거

| 무엇 | 출처 | 날짜 |
|---|---|---|
| 바퀴 지름 0.26 m·두께 0.095 m, 휠베이스 0.657 m, 바퀴 바깥 폭 0.622 m, LiDAR 꼭대기 0.57 m·앞차축 0.21 m 뒤·좌우 가운데, 조향 좌우 약 20°까지(그 이상 안 감), 0°에도 축 유격으로 좌우 오차 조금 | 사용자 측정 | 2026-11-16 |
| 조향 펌웨어 `ABS-HOLD-8-REBOUND-LIMITS`(ESP32-S3, BTS7960 세 개) 소스 `BTS7960_Serial_Control.ino` | 사용자가 준 소스(레포 밖 기록) | 2026-11-16 |
| 보정 RIGHT 1681 / CENTER 1900 / LEFT 2180 = 명령 +20 / 0 / −20, 펄스 시험 기록 | 보드 시험 기록 `REBOUND_CALIBRATION.md`, `verification_quiet_full.log`, `measurement_serial.log`, `rebound_serial.log`(레포 밖) | 2026-11-16 |

## 2. 새 프로필의 값

`apply_spec.py`가 v0.3 자리값 프로필 위에 입력을 넣어 만들었다. 같은 입력이 같은 바이트를 만든다(시험 `test_the_20261116_input_gives_the_installed_profile`).

| 칸 | 값 | provenance | 비고 |
|---|---|---|---|
| `wheelbase_m` | 0.657 | MEASURED_20261116 | |
| `track_front_m`, `track_rear_m` | 0.527 | MEASURED_20261116 | 바깥-바깥 0.622 − 두께 0.095. 앞뒤 구별 없이 준 값 |
| `wheel_radius_front_m`, `wheel_radius_rear_m` | 0.13 | MEASURED_20261116 | 지름 0.26 |
| `wheel_width_m` | 0.095 | MEASURED_20261116 | |
| `steer_mech_limit_deg` | 바퀴마다 20° | MEASURED_20261116 | 어느 바퀴·등가각인지 구별이 없어 네 방향 같게 둠. 등가 기계 한계 18.83°, 소프트 한계 16.14°(여유 3°, CONFIG), 보정 표지 17.83° |
| `rear_axle_x_in_base_m` | −0.3285 | DEFINITION | **SIM 규약**: base 원점 = 발자국 가운데, 두 차축이 원점에 대칭(실측 아님). 앞차축 x = +0.3285 |
| `lidar_x_in_base_m` | 0.1185 | MEASURED_20261116 | 앞차축 x 0.3285 − 0.21. 차축 기준 위치가 측정값이고 base 원점은 위 규약 |
| `lidar_y_in_base_m` | 0.0 | MEASURED_20261116 | |
| `lidar_z_m`(스캔 평면) | 0.55 | ESTIMATED | 꼭대기 0.57(측정) − 0.02(**자리값**: S2 제품 도면을 레포에서 확인하지 못함) |
| `steer_self_locking` | false | ESTIMATED | 보드 시험에서 출력을 끈 뒤 12~26 count 되튐 → 되밀리는 감속기로 봄 |
| `steer_rate_max_deg_s`, `..._standstill` | 100 | ESTIMATED | L1 식(`w = rate (u − min) / (1 − min)`)을 PWM 100 펄스에 맞춘 값(4절). 100 % 속도는 재지 않음 |
| `steer_min_pwm_pct`, `..._standstill` | 10 % | SIM_ASSUMED | 맞춤에 쓴 가정. 시험이 서 있는 차에서였으므로 주행 중 값도 같게 둠 |
| `steer_motor_tau_s` | 0.02 | ESTIMATED | 700 ms 펄스를 끄자 10 ms 안에 멈춤 |
| `steer_fw_mode` | `abs_hold_8` | CONFIG | 3절 |
| `abs_hold_fw` | 펌웨어 상수(3절) | FIRMWARE | |
| `abs_hold_host` | step 0.5°, deadband 1.0°, 최소 간격 1000 ms, poll 200 ms | CONFIG | SIM 호스트 정책(3.3절) |
| `steer_step_time_measured_s` | outward_20 3.5 s, return_20 2.5 s | MEASURED_20261116 | 0→−20 3.34 s, 0→+20 약 3.6 s(마지막 긴 펄스 기준; 허용 10 count 안까지는 3.7 / 4.4 s), +20→0 2.17 s, −20→0 2.90 s |

그대로 둔 자리값 가운데 이 일과 관련 있는 것: 0° 정렬 오차 `steer_zero_offset_deg`(정규 sd 0.7°, ±2°로 자름). 펌웨어의 조용한 가운데 값 1896~1903은 약 ±0.3 명령도이고 축 유격 크기는 모른다. 그래서 더 넓은 자리값 분포를 그대로 둔다. 유격 `steer_backlash_deg` 2°도 그대로다(가운데로 돌아오면 바퀴가 가변저항보다 약 1° 덜 온다: 4절 그림의 남는 각).

## 3. 펌웨어 흉내 (`steer_fw_mode`)

### 3.1 실기 펌웨어가 하는 일

`ANGLE a`(명령도, + 오른쪽, ±20)가 오면 목표 = CENTER + round(span × |a| / 20)(왼쪽 span = LEFT − CENTER = 280, 오른쪽 = RIGHT − CENTER = −219 count). 같은 목표가 다시 오면 무시하고(예산 그대로), 다른 목표면 모터를 끄고 구동 예산·무진척 횟수를 0으로 하고 300 ms 기다린 뒤 첫 판정을 한다.

| 규칙 | 값 |
|---|---|
| 가변저항 | 12비트 ADC, 5 ms마다 한 개, 5개 중앙값, 두 중앙값이 20 count 안이면 확정(평균). 100 ms 넘게 확정 없으면 유지 취소 |
| 판정 | 펄스 뒤 300 ms 끈 상태(EN 끔, 되밀림 가능)의 조용한 값. 끈 뒤 처음 100 ms는 되튐 정점에 넣는다 |
| 멈춤 / 다시 시작 | \|오차\| ≤ 10 count면 멈춤(예산 0으로), 보정이 돌지 않을 때 ≤ 16 count면 무시 |
| 펄스 | PWM 100/255, \|오차\| > 30 count면 80 ms, 아니면 50 ms. 운용 한계 15 count 너머면 일찍 끊음 |
| 취소 | 되튐 ≥ 20 count, 진척 < 3 count가 3번, 한 보정의 구동 합 1000 ms, 운용 한계에서 바깥쪽 펄스 |

1 count는 오른쪽 0.091 명령도, 왼쪽 0.071 명령도다. 목표 허용 10 count = 0.91 / 0.71 명령도, 다시 시작 16 count = 1.46 / 1.14 명령도.

### 3.2 SIM에서

- `native/abs_hold8.cc`가 위 규칙을 옮겼다(실기 펌웨어 소스는 바꾸지 않았다). 펌웨어 V3(`firmware/ubts_steer_v3`)에 비어 있는 법칙 걸이 `steer_law_hook`(`fw_steer.h`)을 더했고, 네이티브 SIM 세션이 그 걸이에 끼워 V3 서보 법칙 자리에서 돈다. 걸이가 비어 있으면(모든 v0.x 프로필, `steer_fw_mode v3_servo`, 그리고 기기에서는 펌웨어가 걸이를 설정하지 않으므로 늘) V3 법칙 그대로다. SIM 전용 `#if` 분기는 두지 않았다(펌웨어 규칙, `test_steering_firmware.py`). 걸이 때문에 펌웨어 빌드 id가 바뀌어 배너의 build 값이 다르다. V3 동작이 같은지는 열 추종 15회의 trace를 main과 견줘 확인한다(재측정 결과 문서).
- 시작 블록 `steer_law`(`native/steer_config`)는 스키마 v4 프로필이 `abs_hold_8`일 때만 붙는다(`native/lockstep_v2.firmware_startup_config`, `steering/abs_hold.py`). T1 리그(빠른 SIM)와 Gazebo의 ESP 에뮬레이터(`gz_esp_emulator.py`)가 같은 네이티브 세션을 쓰므로 둘 다 같다.
- V3에서 그대로 남는 것: 직렬 규약(C 줄, 상태 줄, CRC), ARM, TTL, watchdog, STOP과 정지 유지, 구동 램프, 결함 감시(POT, TRACK 등). 실기 펌웨어에는 이 감시들이 없다. SIM에서는 결함이 더 날 수만 있다(보수적).
- 가변저항: SIM 모형의 10비트 값을 V3 보정 표지(±1000 permille)를 거쳐 실기 12비트 눈금으로 옮긴다(+1000 = LEFT 2180, −1000 = RIGHT 1681). 그래서 SIM의 count 잡음·분해능은 10비트 모형의 것이다(1 count10 ≈ 3~4 count12; 실기 조용한 잡음은 ±3 정도).
- 판정 주기: 실기는 루프마다, SIM은 V3 서보 격자(500 Hz, 2 ms)다. 펄스 길이는 2 ms 안에서 맞다.

### 3.3 명령 눈금 대응 (실행기 → 펌웨어)

```
실행기 steer_mrad (등가 바퀴각, + 왼쪽)
  → 호스트 보정표(host_profile_export: SIM 보정 표지 = 등가 기계 한계 − 1° = 17.83°가 ±1000 permille)  → S permille (+ 왼쪽)
  → ANGLE a = −S × 20 / 1000                       (명령도, + 오른쪽; ±20 = 보드 운용 기준 RIGHT 1681 / LEFT 2180)
  → SIM 호스트 정책(abs_hold_host)이 보낼 때를 정한다
  → 펌웨어 목표 count = 1900 + round(span × |a| / 20)
```

- SIM에서 ANGLE ±20은 보정 표지 각 17.83°(등가)다. 1 명령도 ≈ 0.89°. 실기 명령 눈금의 물리 각은 재지 않았다(펌웨어도 "physical degrees unverified"라고 낸다). 실기 브리지는 B2·B3으로 잰 표지 각으로 같은 식을 쓰면 된다.
- 실행기 한계는 소프트 한계 16.14°(281 mrad) = ANGLE 약 18.1이다. 그 너머 목표는 V3 쪽에서 소프트 한계로 자른다.

SIM 호스트 정책(새 보드용 Jetson 브리지는 아직 없다. 이 정책은 SIM의 제안이다): poll 200 ms마다 실행기 목표를 0.5 명령도로 반올림해 마지막 ANGLE과 1.0 명령도 이상 다르면 보낸다. 단 마지막 ANGLE 뒤 1000 ms가 지났거나, 보드가 목표에 닿았거나(오차 ≤ 10 count), 유지가 취소되었을 때만. 취소 뒤 오차가 16 count보다 크면 같은 ANGLE을 다시 보낸다. 까닭: 새 목표마다 펌웨어가 300 ms 기다림과 구동 예산을 다시 시작하므로, 20 Hz로 조금씩 다른 각을 보내면 펄스가 한 번도 나가지 않는다. 실기 브리지도 이런 묶음이 필요하다.

### 3.4 실행기의 '측정' 조향 값

SIM 실행기 설정(`sim_in_loop_steer.steer_executor_config`, G-SIM `exec_container_main.gsim_executor_config`)은 `abs_hold_8`이면 모터의 100 % 속도가 아니라 보드의 계단 응답을 쓴다(`abs_hold.executor_values`).

| 값 | SIM 값 | 계산 |
|---|---|---|
| 측정 조향 속도(주행·정지) | 5.09 °/s | 표지 각 17.83° / outward_20 3.5 s |
| 명령 변화율 한도 | 4.08 °/s | 0.8 × 측정 속도(설계 6.2) |
| 서보 데드밴드 | 0.81° | 목표 허용 10 count, 오른쪽(1 count가 더 큰 쪽) |

**발견**: 실행기의 조향 GO 규칙(측정 조향 속도 ≥ `steer_rate_min_deg_s` 8 °/s, 자리값, 설계 6.6 "P8에서 정함")이 이 값을 거부한다. 설정 검사(`config.validate`)에서 걸려 실행기가 서지 않는다. 열 추종 재측정(`PREREG_STEER_FOLLOW_REALDIMS_KO.md`)은 이 문턱을 4.0 °/s로 낮춘 설정을 문서화해 쓴다.

## 4. 구동기 맞춤과 계단 응답 (T1, 서 있는 차)

보드 시험의 80 ms 펄스 한 번은 바깥쪽으로 30~47 count(오른쪽 2.9~4.3, 왼쪽 2.1~3.2 명령도), 가운데 쪽으로 51~72 count를 옮겼다. 700 ms 연속 펄스는 처음 약 100 ms 거의 안 움직이고 그 뒤 약 600 count/s였고, 끄자 곧 멈췄다. SIM L1 모형은 펄스 하나의 이동이 펄스 길이에 비례한다(죽은 시간 없음). 그래서 80 ms 펄스(대부분의 펄스)에 맞췄다: rate 100 °/s, 시작 PWM 10 %, τ 0.02 s → PWM 100에서 약 33 °/s, 80 ms에 약 2.6°.

| 계단(T1, 서 있는 차, 정상 대본 아님) | SIM | 실기(명령 20) |
|---|---|---|
| 0 → 왼쪽 소프트 한계 16.2° | 1° 안까지 3.01 s | 0 → −20: 마지막 긴 펄스 3.34 s, 허용 안 3.7 s |
| 16.2° → 0 | 2.45 s, 바퀴 +1.0°에서 멈춤(가변저항은 허용 안, 유격 2°의 절반) | +20 → 0 2.17 s, −20 → 0 2.90 s |
| 0 → 오른쪽 소프트 한계 | 2.35 s, −14.6°에서 멈춤(가변저항 쪽 허용 안 + 유격) | 0 → +20 약 3.6 s(되튐 12~18 count) |

(`test_steering_abs_hold.py`가 펄스 모양(PWM 100, 50/80 ms, 사이 ≥ 300 ms, 그사이 EN 끔)과 이 범위를 확인한다.)

맞추지 못한 것:
- 50 ms 펄스: SIM 약 1.6°, 실기 1.0~1.5 명령도. 작은 보정이 SIM에서 조금 크다.
- 바깥쪽 / 가운데 쪽 차이(가운데로 약 1.6배 빠름): 가운데로 미는 토크 모형이 없어 SIM은 양쪽이 같다. 가운데로 돌아오는 것이 실기보다 느리다(보수적).
- 정지 상태에서 첫 펄스가 덜 움직이는 것(실기 8~16 count): 모형 없음.
- 주행 중 조향(서 있을 때보다 보통 쉽다): 잰 적 없어 서 있을 때 값을 쓴다.

## 5. 짧은 확인의 외란 결과 (발견)

`apply_spec.py`의 짧은 확인 D1(+2.5 N·m 외란, 개정 41 대본, seed 0)이 이 프로필에서 실패했다: 펄스 사이에 드라이버가 꺼져 있어(실기와 같다) 외란이 조향을 끝(18.8°)까지 민다. PWM 100은 정지 토크의 39 %(자리값 정지 토크 5 N·m에서 약 2 N·m)라 2.5 N·m를 되돌리지 못한다. 빠른 움직임에 V3의 POT_NOISE 감시(실기에는 없음)가 걸려 MCU가 무장을 풀었다. 외란 크기·정지 토크·되밀림 배율은 모두 자리값이라 현장에서 이만큼 밀리는지는 모른다. V3 서보(제동 유지)는 같은 대본에서 버텼다(`STEER_HOLD_RESULT_KO.md`).

## 6. Gazebo

- 바퀴·차축·조향 관절 한계는 프로필을 따른다(`gz_model.wheel_layout`: 앞뒤 차축 x ±0.3285, 윤거 0.527, 반경 0.13, 두께 0.095, 관절 ±20°).
- LiDAR: 장면 조립(`gz_assemble.profile_lidar_mount`)이 생성 모델의 laser 링크를 프로필 위치(0.1185, 0, 0.55)에 두고, 감독(`supervisor_steer`)이 Embree LiDAR(`cpu_lidar.py`)와 clip 노드에 같은 장착 값을 담은 런타임 `rover.yaml` 사본을 준다. 고정 파일(`rover.yaml`, `cpu_lidar.py`, `generate_world.py`)은 그대로다. 예전 프로필은 값이 `rover.yaml`과 같아 SDF가 그대로다.
- Gazebo의 조향 관절은 위치 제어라 자기 잠금 가정이다(`GZ_MODEL_KO.md`). 펄스 사이의 되밀림은 빠른 SIM에서만 있다.
- 이 작업에서 Gazebo 측정은 하지 않았다.

## 7. 다시 만드는 법

```
python -B tools/steer_sim/apply_spec.py --spec sim/ubts/steering/profiles/steer_spec_input_20261116.yaml --out <저장소 밖>
cd sim/ubts && python -B -m pytest -q -p no:cacheprovider test_steering_abs_hold.py test_steering_apply_spec.py
```
