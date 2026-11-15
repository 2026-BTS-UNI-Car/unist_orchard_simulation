# 실제 치수·실기 조향 펌웨어로 열 추종 다시 재기 사전등록 (steer_follow_realdims_v1)

- 작성: 2026-11-16. 등록 번호 `steer_follow_realdims_v1`. 기계가 읽는 사본: `PREREG_STEER_FOLLOW_REALDIMS.json`(`follow_realdims.py --prereg`의 출력과 같아야 한다. 시험 `sim/ubts/test_steering_follow_realdims.py`가 본다).
- 이 문서는 판정에 쓰는 run을 돌리기 전에 커밋한다. 이 커밋까지 새 프로필로 돌린 것(개발 확인, 판정 아님): 등록 밖 seed 99의 `nominal` 1회(열 끝 정지, 기능 확인), `gate_as_is` 기능 확인 1회(설정 거부), T1 정지 계단(`ABS_HOLD8_SIM_KO.md` 4절)과 단위 시험, `apply_spec.py`의 짧은 확인(N1 직진 통과, D1 외란 실패, 10° 원).
- 결과는 **자리값 기반 SIM 결과**다. 실차 성능이 아니다. 새 프로필도 SPEC_PENDING 자리값이 34칸 남는다.
- 근거: 실제 치수와 펌웨어 흉내 `ABS_HOLD8_SIM_KO.md`, 앞 등록 `PREREG_STEER_FOLLOW_KO.md`(15회), `PREREG_STEER_FOLLOW_V2_KO.md`(넓은 조건, 2차 규칙).

## 1. 무엇으로 재나

| 항목 | 값 |
|---|---|
| 시뮬레이터 | F-SIM T1 리그, 실행기 포함(`tools/row_executor/sim_in_loop_steer.py`). 앞 두 등록과 같다 |
| 프로필 | `front_steer_v1_spec_20261116`(스키마 v4). 휠베이스 0.657, 윤거 0.527, 바퀴 반경 0.13·두께 0.095, 조향 한계 바퀴마다 20°, LiDAR (0.1185, 0, 0.55) m. `steer_fw_mode abs_hold_8`: ESP 에뮬레이터가 실기 펌웨어 ABS-HOLD-8의 펄스 유지를 돈다 |
| 실행기 설정 | 리그가 만드는 설정(`steer_executor_config`). 조향 '측정' 값은 보드 계단 응답: 측정 조향 속도 5.09 °/s, 명령 변화율 한도 4.08 °/s, 서보 데드밴드 0.81°(`abs_hold.executor_values`) |
| GO 문턱 바꿈 | `steer_rate_min_deg_s` 8.0 → **4.0** °/s. 까닭: 8 °/s는 자리값(설계 6.6, "P8에서 정함")이고 보드는 5.09 °/s라 그대로면 설정이 거부된다(`gate_as_is`에서 기록). 다른 안전 한도와 GO 규칙은 그대로다 |
| 제어기 | `{"kind": "row_follow", "wheelbase_m": 0.657}`. 휠베이스만 프로필 값(기본 0.76은 옛 자리값). 다른 값은 기본값 |
| 발자국 | 1.22 × 0.70 m(프로필 자리값 그대로, 앞범퍼 0.61 m) |

## 2. 부분

| 부분 | run | 판정 |
|---|---|---|
| `gate_as_is` | `nominal` seed 0 하나, 실행기 설정 그대로 | 판정 안 함. 기대: 조향 GO 규칙이 설정을 거부한다(`config.validate`: 측정 조향 속도 8 °/s 아래). 결과를 그대로 적는다 |
| `matrix` | `steer_follow_v1`의 11조건 15회(아래 표 A) | 15회 모두 통과면 통과 |
| `wide` | `steer_follow_wide_v2`의 20조건 × seed 4000, 4001, 4002 = 60회(아래 표 B) | 조건마다 통과 수를 적는다. 60회 모두 통과면 통과 |
| `geometry_only` | `matrix`와 같은 15회, `steer_fw_mode v3_servo`(새 치수 + V3 서보, 맞춘 모터 속도) | 판정 안 함. 펌웨어 법칙만의 차이를 보려는 비교 |

표 A(`matrix`, 앞 등록 2절 그대로): `nominal` seed 0~4, `lateral_p040`, `lateral_n040`, `heading_p8`, `heading_n8`, `steer_zero_p2`, `steer_zero_n2`, `side_pull`, `lidar_dropout_0p2`, `lidar_stale_1p0`(기대 `stale_lidar`), `obstacle`(기대 `obstacle_ahead`) 각 seed 0. 세계·재는 값·기준은 `PREREG_STEER_FOLLOW_KO.md` 1~4절과 같다(`follow_matrix.measure`, `judge`).

표 B(`wide`, 앞 등록의 조건 정의와 2차 재는 규칙 그대로; `follow_wide.measure`, `judge`):

| 무리 | 조건 | 기대 |
|---|---|---|
| 열 기하 | `curve_l30`, `curve_r30`, `gap3_both`, `endpost_2p0`, `width_3p0` | 열 끝 정지 |
| W2 통로 | `w2_042301_c03`, `w2_040927_c06` | 열 끝 정지(W2 횡오차는 참 통로 가운데 기준) |
| 출발 | `heading_p10`, `heading_n10` | 열 끝 정지 |
| 차량 | `pull_left`, `pull_right` | 열 끝 정지 |
| 속도 | `speed_0p2`, `speed_0p5` | 열 끝 정지 |
| 구동 표 | `table_x070`, `table_x130` | 열 끝 정지 |
| 조향 외란 | `dist_step_p2p5`(6 s부터 +2.5 N·m 3 s), `dist_terrain`(지면 충격·밀기 과정) | 열 끝 정지, 조향 결함 없음. 앞 등록은 v0.3 프로필이었고 여기서는 새 프로필(외력 모형 있음)이다 |
| 센서 | `imu_stale_1p0` | `stale_imu`로 일시정지 |
| GNSS | `gnss_nominal` | 열 끝 정지 |
| 조작 | `operator_link_lost` | `operator_link_lost` |

기준(두 앞 등록과 같은 수): 접촉 없음(여유 > 0 m), 끝에 정지(≤ 0.02 m/s), guard 위반 종결 없음, A22 규칙. 열 끝 run은 `row_end`, 주행 뒤 일시정지 0, 횡 RMSE ≤ 0.20 m, p95 ≤ 0.35 m, 최대 ≤ 0.50 m, 방향 RMSE ≤ 5°, 최대 ≤ 15°, 참 속도 최대 ≤ 0.50 m/s(`speed_0p5`는 0.55), e −1.0 … +1.0 m. 출발 어긋남 조건과 W2는 RMSE·p95를 6.0 m 뒤부터 잰다. 일시정지 조건은 앞 등록의 규칙(구동 0까지 시간, 다시 출발 안 함).

## 3. 고칠 수 있는 것 (반복)

- 반복은 최대 2번. 고치기 전에 이 문서에 개정(무엇을, 왜, 어느 run을 다시)을 적고 커밋한다.
- 고칠 수 있는 것: 현장 조정 값(`field_tuning`의 basic·advanced 칸) 또는 문서와 시험을 갖춘 제어기 바꿈. 바꾸지 않는 것: 이 문서의 기준, 실행기 guard·신선도 한도·속도 한도, 위 문턱 말고 다른 GO 규칙, 펌웨어 흉내와 프로필 값.
- 반복마다 전(before)과 후(after)를 같은 표로 적는다. 판정 run은 등록한 seed 그대로다.

## 4. 비교로 적는 것 (판정 아님)

- 앞 결과: `steer_follow_v1` 15/15(v0.1, V3, `STEER_FOLLOW_RESULT_KO.md`), 넓은 조건 2차 판정(`STEER_FOLLOW_WIDE_RESULT_KO.md`, seed 4000~4004, v0.1.2·v0.3).
- `geometry_only`: 새 치수에서 V3 서보로 바꾸면 어떻게 되는지.
- V3 동일성: 걸이를 더한 뒤 기본 프로필(v0.1, V3)의 15회와 T1 안전 대본이 main과 같은 trace를 내는지(결과 문서에 적는다).

## 5. 이 등록이 보이지 않는 것

현장 성능, 실기 브리지(새 보드용 Jetson 브리지는 아직 없고 SIM 호스트 정책은 제안이다), 서 있을 때만 잰 조향 속도의 주행 중 차이, 가운데로 미는 토크, 실기 펌웨어에 없는 V3 감시의 차이, 구동 쪽 watchdog(실기 펌웨어에는 구동 명령이 끊겨도 서는 장치가 없다. SIM은 V3 TTL·watchdog를 그대로 쓴다), Gazebo.
