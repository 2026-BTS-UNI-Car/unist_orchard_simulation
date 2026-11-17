# SMOOTH r5 조향 펌웨어 SIM 흉내: 확인 결과와 열 추종 (steer_follow_smooth_r5_v1)

- 작성: 2026-11-17 (작업 sim-smooth-model). 모형과 프로필: [`SMOOTH_R5_SIM_KO.md`](SMOOTH_R5_SIM_KO.md). 사전등록:
  [`PREREG_STEER_FOLLOW_SMOOTH_KO.md`](PREREG_STEER_FOLLOW_SMOOTH_KO.md)(커밋 f67b495, 판정 run 전). 기계가 읽는 결과: `STEER_FOLLOW_SMOOTH_RESULT.json`.
- 모든 수치는 **자리값 기반 SIM 결과**다. 현장 성능이 아니다. 측정한 코드: 브랜치 HEAD 41ca526(모형 783df78 + 사전등록 f67b495 + 문서, main aa7f2f4 위).
- **작업 중(WIP)**: 브랜치를 main b8fef2e 위로 옮겼다(같은 내용의 커밋 e228693, eb45a01, 8122eb1; 충돌 없음, 조향 시험 1,347 통과 28 건너뜀).
  b8fef2e 대비 동일성(T1 71 기록, nominal 두 프로필)과 등록 matrix 15 의 재확인은 아직 하지 않았다. 그 전에는 main 에 넣지 않는다.
- 측정은 무거운 측정 잠금 안에서 했고, 시작 전 호스트 부하 확인을 통과했다(전체 CPU 30 s 평균 20.5 %, E 등급 평균 24.4 %, `STACK_OPERATIONS_KO.md` 2.1).
  기록 폴더(레포 밖) `steering-sim-20261108/runs/sim_smooth_model_20261117/`(measure/, logs/, NOTES_KO.txt).

## 1. 요약

| 무엇 | 결과 |
|---|---|
| 사전등록 `matrix` 15회 | **15/15 통과**(열 끝 13회 모두 row_end, 기대 정지 2회 맞음) |
| 사전등록 `wide` 60회 | **52/60**(불합격): `dist_step_p2p5` 0/3(V3 POT_NOISE), `dist_terrain` 1/3(V3 TRACK), `operator_link_lost` 0/3(정지 시간 기준). 나머지 17조건 51/51 |
| 실기 r5 소스와의 차이(레포 밖 차이 시험) | QUIET_MS 0: 7개 상황 모두 핀이 ms 마다 같음(11,900/11,900 ms). QUIET_MS 15: 같은 목표·같은 취소 수, 끝 위치 차 2 count 안 |
| 적합성 | SMOOTH 대본 11/11(두 전송), V3 대본 92/92(두 전송, 시험 안에서) |
| 동일성(main aa7f2f4 대비) | T1 71/71 통과, trace 71/71 같음; nominal v0.1·v1 각 5/5, 설정·trace 5/5 같음. 직렬 로그는 머리 줄의 네이티브 빌드 id 만 다르다(2절) |

## 2. 옛 프로필이 그대로인지 (동일성)

같은 잠금 세션(09:23~09:29)에서 브랜치를 돌려 main aa7f2f4 의 결과(다른 작업이 같은 날 09:05~09:10 에 돌린 `field_steer_config_20261116/measure/base3_*`,
그리고 dad6a93 의 `executor_kit_20261116/measure/*`)와 견줬다.

| 묶음 | 통과 | trace | 설정 digest | 직렬 로그 |
|---|---|---|---|---|
| T1 안전 대본 71 | 71/71(A17, A22 true) | 71/71 같음 | - | digest 는 71 모두 다름: 머리 줄의 `native_build_id`(네이티브 소스 해시, smooth_r5.cc 를 더해 바뀜). 머리 줄을 뺀 기록 digest(일꾼 1, 분리 작업 트리 aa7f2f4 와 같은 잠금 안에서 따로 돌림): 71/71 같음 |
| 빠른 SIM nominal, v0.1 | 5/5 | 5/5 같음 | 5/5 같음 | 머리 줄의 `native_build_id` 만 다르고 나머지 바이트가 같다(5/5) |
| 빠른 SIM nominal, v1(펄스 유지) | 5/5 | 5/5 같음 | 5/5 같음 | 같은 판정(5/5) |

- 실행기 기록은 시간 칸(`ms`, `loop_ms`, `ph`, `scan.proc_ms`)과 튜닝 파일 경로(`tuning.file`, 작업 트리 이름)만 다르다(앞 작업들과 같다).
- 펌웨어 빌드 id 878B5F56 은 그대로다(V3 펌웨어 소스를 바꾸지 않았다).

## 3. 실기 r5 소스와 SIM 이식의 차이 시험 (레포 밖)

`runs/sim_smooth_model_20261117/diff/diff_harness.cpp`: 실기 `source_r5.ino`(호스트 시험의 stub 로 컴파일)와 SIM 의 `SmoothBoard`(smooth_r5.cc)를 같은
자리값 플랜트 두 벌에 물리고, 보드 NVS 값(CAL 1475/2017/2513, SM 값)과 같은 ANGLE 을 같은 ms 에 준다. ms 마다 조향 핀(EN, RPWM, LPWM)을 견준다.

| 상황 | QUIET_MS 0: 핀 같은 ms | QUIET_MS 15: 핀 같은 ms(실기가 조용한 읽기로 시계를 넘긴 ms 제외) | QUIET_MS 15: 끝 위치 A / B, 취소 A / B |
|---|---|---|---|
| free(왼 20 → 오른 20 → 0 → ±10) | 11900/11900 | 8112/10808 | 2012.0 / 2013.3, 0 / 0 |
| retarget(이어받기, 방향 바꿈) | 11900/11900 | 10150/11455 | 1885.3 / 1887.2, 0 / 0 |
| heavy(PWM 75 아래 안 움직임, 바닥 올림) | 11900/11900 | 9040/10768 | 2096.7 / 2096.6, 0 / 0 |
| stuck(막힘: STALL 취소 두 번) | 11900/11900 | 11818/11840 | 2017.0 / 2017.0, 2 / 2 |
| offset(켜진 동안 50 count 처짐) | 11900/11900(둘 다 NO CONVERGENCE 2번) | 9017/11085 | 2013.4 / 2012.2, 0 / 0 |
| bursts(110 count, 20/170 ms) | 11900/11900 | 9017/11085 | 2013.4 / 2012.2, 0 / 0 |
| noise(±6 count) | 11900/11900 | 8507/11015 | 2192.5 / 2191.3, 0 / 0 |

- 조용한 읽기를 끄면 이식은 실기 소스와 핀 단위로 같다. 켜면 차이는 조용한 읽기의 방식(실기 1.5 ms 한 루프 안, SIM 1 ms 한 틱)에서 나오는 시각
  차이뿐이고, 닿는 목표·취소·끝 위치(2 count 안)가 같다.
- offset 상황: QUIET_MS 0 이면 실기와 SIM 모두 목표를 못 보고 NO CONVERGENCE 로 두 번 취소한다(호스트 시험 r5 의 '놓치거나 취소'와 같은 결론).

## 4. 사전등록 판정 (`matrix`, 15회)

| 조건 | 결과 | 끝 | 횡 RMSE / 최대 (m) | 방향 RMSE / 최대 (°) |
|---|---|---|---|---|
| nominal s0~s4 | 5/5 | row_end, e +0.045 ~ +0.068 m | 0.030 ~ 0.050 / 0.073 ~ 0.097 | 0.98 ~ 1.54 / 1.9 ~ 3.7 |
| lateral_p040, lateral_n040 | 2/2 | row_end | 0.038, 0.047 / 0.40(출발 어긋남) | 1.15, 1.34 / 10.9, 11.0 |
| heading_p8, heading_n8 | 2/2 | row_end | 0.067, 0.041 / 0.149, 0.077 | 1.75, 1.04 / 8.0 |
| steer_zero_p2, steer_zero_n2 | 2/2 | row_end | 0.033, 0.056 / 0.077, 0.138 | 0.78, 1.56 / 1.8, 4.1 |
| side_pull, lidar_dropout_0p2 | 2/2 | row_end | 0.060, 0.051 / 0.114, 0.110 | 1.93, 1.57 / 3.5, 3.1 |
| lidar_stale_1p0 | 1/1 | 20.3 s stale_lidar(기대) | - | - |
| obstacle | 1/1 | obstacle_ahead(기대) | - | - |

조향 결함 줄 0, 접촉 0, 참 속도 최대 0.456 m/s. 열 끝 13회 평균 횡 RMSE 0.046 m(v1 펄스 유지 0.054), 방향 RMSE 평균 1.30°(v1 1.90°).

## 5. 사전등록 판정 (`wide`, 20조건 × 3 seed = 60회): 52/60, 불합격

| 조건 | 통과 | 실패 내용 |
|---|---|---|
| curve_l30, curve_r30, gap3_both, endpost_2p0, width_3p0, w2_042301_c03, w2_040927_c06, heading_p10, heading_n10, pull_left, pull_right, speed_0p2, speed_0p5, table_x070, table_x130, gnss_nominal | 각 3/3 | - |
| imu_stale_1p0 | 3/3 | 20.4 s stale_imu(기대) |
| dist_step_p2p5(6 s 부터 +2.5 N·m 3 s) | 0/3 | 6.49 s 에 V3 POT_NOISE → MCU 무장 풀림 → steering_feedback_invalid 로 멈춤(약 −22 m 앞) |
| dist_terrain(지면 충격·밀기) | 1/3 | s4001 7.79 s, s4002 13.46 s 에 V3 TRACK → steering_not_following 으로 멈춤 |
| operator_link_lost | 0/3 | 운영자 신호가 끊긴 뒤 구동 0 까지 1.05 s(기준 0.55 s). 멈춤 자체는 `operator_link_lost` 로 맞음 |

- 열 끝 49회 평균 횡 RMSE 0.047 m(v1 0.061), 최대 횡 0.368 m(W2 통로, 참 통로 가운데 기준), 방향 RMSE 평균 1.36°(v1 2.01°).
- v1(펄스 유지, 같은 60회) 은 51/60 이었고 같은 세 조건에서 실패했다(외란 6, 운영자 연결 3). SMOOTH 는 `dist_terrain` s4000 하나를 더 통과했다.
- 실패 원인(판정 밖 메모, 고치지 않음): 두 외란 조건의 결함은 실기 보드에 없는 V3 감시(POT_NOISE: 50 ms 창의 가변저항 분산, TRACK: 1 s 동안 목표에
  못 닿고 진척 없음)다. 외란 크기·정지 토크·되밀림 배율은 자리값이다. SMOOTH 는 PWM_MAX 80(31 %)으로 밀어 펄스 유지(PWM 100 펄스 사이 놓임)보다
  덜 밀리지만, 2.5 N·m 계단은 여전히 조향을 빠르게 끝까지 민다. `operator_link_lost` 의 1.05 s 는 v1 과 같은 값이라 펌웨어 법칙과 상관없는 측정 기준
  문제로 보인다(앞 작업의 진단 `sim-realdims-diag`).

## 6. 시험과 검사 (커밋 뒤)

- 단위 시험(조향 시험 전부 `test_steering_*.py`, Windows Python 3.13, b8fef2e 위로 옮긴 뒤): 1,347 통과 28 건너뜀(Linux 전용·외부 자료 없음), 실패 0. 처음 돌릴 때 실패였던 2개를 고침:
  `test_steering_profile` 의 SHAPES 범위(v5 이름), `test_steering_fast_sim` 의 identity(aa7f2f4 에서도 `abs_hold.py` 로 실패하던 것; `abs_hold.py`,
  `smooth_hold.py` 를 IDENTITY_FILES 에 더함).
- 새 시험: `test_steering_smooth_hold.py` 12, `test_steering_smooth_conformance.py` 7, `test_steering_follow_smooth.py` 4.
- 레포 검사: check_pins OK(고정 115 중 113 같음, 허용 2), check_vendor_sync OK, check_import_manifest OK, check_assets OK, 바뀐 파일 좌표 lint 판정 0.

## 7. 남는 것

- 땅에 닿은 조향(바퀴 든 계단으로 맞춤), 주행 중 조향 속도, 가변저항 어긋남의 크기·부호(기본 0), 10비트 ADC 분해능(1 raw ≈ 7~8 count), 실기에 없는
  V3 감시, 어댑터·브리지 두 겹 정책을 하나로 합친 것, Gazebo 에서 이 프로필로 돌린 적 없음.
- 등록 판정과 화면용 스택의 기본 프로필을 v2 로 바꿀지는 따로 정한다(R102).
