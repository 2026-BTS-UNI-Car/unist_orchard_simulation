# 시스템 참고: 인터페이스, 주기, 추정·제어, 평가, 기록

README와 [`SIMULATION_KO.md`](SIMULATION_KO.md)를 코드 기준으로 자세히 적은 문서다. 값은 2026-11-16 main 기준이며, 파일 이름은 저장소 맨 위에서의 경로다.
표지: **코드**(코드에서 읽은 값), **설정**(compose·yaml·json 값), **측정**(기록된 run의 수치, 날짜·조건 함께), **설계**(설계 문서에만 있고
코드에 없음). 수치는 SIM 결과이며 조향 장치 값 대부분은 자리값이다. 예외는 4절의 Jetson 줄과 6절 끝의 현장 설정으로, 2026-11-19·20 현장 기록에서 읽은 값이다(2026-11-22 더함).

## 1. Gazebo 스택의 노드와 토픽

모든 서비스가 sim 컨테이너의 네트워크·IPC 이름공간을 같이 쓴다. ROS 도메인 43, 발견 LOCALHOST, FastDDS 공유 메모리 + UDPv4 127.0.0.1만
(설정, `compose.ubts-steer.yaml`, `sim/ubts/fastdds.xml`). QoS 약어: BE = BEST_EFFORT, REL = RELIABLE, KLn = KEEP_LAST 깊이 n.

| 노드 (컨테이너) | 구독 | 발행 (형, QoS, 주기) | 근거 |
|---|---|---|---|
| `gz sim` (sim) | `/sim/act/*` | `/clock` 물리 걸음마다(설정 120 Hz), `/sim/imu` 33 Hz, `/sim/camera` 640×360 30 Hz, 참값 `/sim/ground_truth`·`/sim/truth/joint_states` | 설정·코드, `sim/ubts/steering/gz_assemble.py`, `gz_model.py`, `sim/ubts/rover.yaml` |
| `ros_gz_bridge` (sim) | ROS `/sim/act/*` Float64 | `/clock`, `/sim/imu`, `/ground_truth/odom`, `/sim/truth/joint_states`, `/camera/image_raw`, `/camera/camera_info` | 설정, `sim/ubts/steering/bridge_steer.yaml` |
| `ubts_cpu_lidar` (sim) | `/ground_truth/odom` BE KL1 | `/sim/scan` LaserScan BE KL1, SIM 시간 0.1 s 격자(10 Hz), 3200빔, 0.05~30 m, 잡음 0.01 m | 코드, `sim/ubts/cpu_lidar.py` |
| `ubts_steer_scan_clip` (sim, 10/10 장면만) | `/sim/scan`, `/ground_truth/odom` | `/sim/scan_clipped`: 지도에서 관측되지 않은 칸 뒤 반사를 지움 | 코드, `sim/ubts/steering/scan_unobserved_clip.py` |
| `ubts_steer_s2_format` (sim) | `/sim/scan` 또는 `/sim/scan_clipped` BE KL10 | **`/scan`** LaserScan **REL KL10**, 입력마다 1개(10 Hz), RPLIDAR S2 형식 3240빔, 1 mm 반올림, 0 = 반사 없음 | 코드, `sim/ubts/steering/s2_format_node.py`, `s2_format.py` |
| `ubts_steer_sensor_contract` (sim) | `/clock`, `/sim/imu`, `/ground_truth/odom`, 조향 상태 | **`/imu/data_raw`** Imu(오차 모형 + 게임 회전 벡터 자세) 33 Hz; **`/gps/fix`** NavSatFix·`/gps/data` String 약 5 Hz(잡음·끊김 없음) | 코드, `sim/ubts/steering/sensor_contract_steer.py` |
| `gz_esp_emulator` (steer) | `/clock` REL KL20, `/ground_truth/odom`(속도만) | 관절 명령 `/sim/act/steer_*_cmd`·바퀴 `_cmd` Float64(바뀔 때 + 0.1 s마다 전부), `/sim/truth/steering`(평가 전용) | 코드, `sim/ubts/steering/gz_esp_emulator.py` |
| `steer_rover_bridge` V3 (steer) | control.sock, pty 직렬 | `/rover/steering/state` MCU 상태 줄마다(기본 20 Hz, 줄이 없으면 0.5 s마다 stale 재발행), `/rover/status`·`/rover/control/requested`·`/rover/control/applied` 0.5 s마다 | 코드, `dashboard/scripts/steer_rover_bridge.py` |
| `data_recorder_service` (recorder) | 기록 토픽(7절) | HTTP 8790 | 코드, `dashboard/scripts/data_recorder_service.py` |
| `steering_sidecar` (recorder) | `/rover/steering/state` REL KL100 | 파일(7절) | 코드, `sim/ubts/steering/steering_sidecar.py` |
| `ubts_row_executor` (executor) | `/scan` BE KL2, `/imu/data_raw` BE KL50, `/rover/control/requested`; SIM 시계 모드에서만 `/clock`; `gnss_mode`가 off가 아닐 때만 `/gps/fix` BE KL5 | ROS 발행 없음. 출력은 control.sock 요청과 운영자 HTTP | 코드, `tools/row_executor/executor_main.py`, `sim/ubts/steering/exec_container_main.py` |
| WS 브리지 4개 (sim) | `/scan`, `/imu/data_raw`, `/gps/data`, 카메라 | 대시보드용 WebSocket(LiDAR 최대 10 Hz, IMU 최대 20 Hz, 카메라 30 fps) | 코드·설정, `sim/ubts/steering/supervisor_steer.py` |
| 대시보드 (dashboard) | 상태 파일, control.sock, 실행기 HTTP | 컨테이너 8080 → 호스트 `127.0.0.1:18090` | 설정, `compose.ubts-steer.yaml` |

- `/scan`은 RELIABLE로 발행되고 실행기는 BEST_EFFORT로 받는다(맞물림 정상).
- 컨테이너별 자식 프로세스: sim = gz sim → ros_gz_bridge → cpu_lidar → (clip) → s2_format → sensor_contract → WS 브리지,
  steer = 프로필 내보내기 → ESP 에뮬레이터(pty) → 조향 브리지, recorder = 기록기 → 사이드카, executor = 실행기 한 프로세스
  (제어기 `row_follow`). 메모리: sim 3.5, dashboard 1, steer·recorder·executor 각 0.5 GiB(설정).
- 실행기 컨테이너는 runtime 볼륨과 에뮬레이터 hook을 마운트하지 않는다. 참값 파일에 닿지 않는다(설정).
- 시계 모드 `UBTS_STEER_CLOCK`: 기본 `wall`. `sim`이면 브리지와 실행기가 둘 다 SIM 시간을 쓴다(사용자 스택과 최종 확인은 SIM 시계).

## 2. ESP 에뮬레이터, 조향 브리지, 제어 소켓

- 에뮬레이터: `/clock` 하나 = Gazebo 한 걸음. MCU 펌웨어(`firmware/ubts_steer_v3`, `UBTS_STEER_SIM=1` 네이티브 빌드)가 1 ms 틱으로 돈다.
  MCU 상태 줄 기본 20 Hz, 링크 감시 500 ms, 명령 유효시간(TTL) 최대 300 ms, 서보 500 Hz(코드, `fw_params.cpp`, `fw_link.cpp`).
- 측정 치수 프로필(`steer_fw_mode: abs_hold_8`)은 서보 법칙만 실차 보드의 펄스 유지 흉내로 바꾸고, 링크·arm·TTL·감시·STOP은 V3 그대로다.
  현장 프로필 `front_steer_v2_field_20261116`(`steer_fw_mode: smooth_r5`, 2026-11-17)은 같은 자리에 그 보드의 SMOOTH 유지 r5 흉내를 끼우고
  서보 격자를 1000 Hz 로 둔다([`SMOOTH_R5_SIM_KO.md`](steering/SMOOTH_R5_SIM_KO.md)).
  실차 ESP32-S3 펌웨어의 구동 감시(300 ms, 2026-11-16 보드에 씀)는 SIM에 짝이 없다.
- 제어 소켓 `control.sock`(Unix 소켓): 연결 하나에 JSON 한 줄(≤ 4096 B), 답 한 줄.
  - 요청: `arm {owner, vehicle_profile}`, `drive {owner, lease, ticket, seq = 앞 + 1, deadman, throttle 0~100, steer_mrad}`, `keepalive`,
    `motion {yaw_rate_rad_s, speed_m_s}`, `status`, `stop {reason}`, `release`.
  - 답: 성공 `{ok:true, lease, ticket, seq, ticket_ttl_s:1.5, status:{…}}`, 실패 `{ok:false, error}`.
  - 브리지 기한: 입력 0.75 s, ticket 1.5 s, owner 30 s. MCU 상태 줄이 150 ms 끊기면 `steering_status_stale` 래치.
  - 브리지 루프 20 ms. 구동 명령 줄은 새 요청 때 바로, 입력이 유효한 동안 20 Hz, TTL = min(300 ms, 남은 입력 시간).
    STOP은 100 ms 뒤부터 50 ms마다 5번 다시 보내고, 400 ms까지 확인이 없으면 래치.

## 3. 실행기

### 3.1 한 틱 (20 Hz, `tools/row_executor/executor_main.py` `ExecutorLoop.step`)

| 순서 | 하는 일 |
|---|---|
| 0 | 주기: 벽시계 모드는 50 ms 절대 일정(늦으면 놓친 틱을 건너뛰고 `missed_ticks`로 셈), SIM 시계 모드는 SIM 시간 50 ms 격자 |
| 1 | 운영자 요청 처리(HTTP 스레드가 넣은 start·arm·go·ack·params) |
| 2 | STOP 처리 |
| 3 | 입력: IMU는 모두 순서대로, 스캔은 시각만 붙이고 가장 새것 하나만 처리(나머지는 `scans_dropped`) |
| 4 | 감시 스레드가 건 일시정지 반영 |
| 5 | 조향 상태가 이 걸음 처리로 0.15 s보다 낡았으면 브리지 `status`를 한 번 다시 읽음 |
| 6 | `Runner.tick`: 시계 역행·틱 틈(> 0.35 s) 검사 → 추정기 스냅숏 → 신선도(LiDAR ≤ 0.45 s, IMU ≤ 0.40 s) → 조향 결함 → 제어기 |
| 7 | 출력: 주행 중 `drive(throttle, steer_mrad)` 10 Hz(throttle이 0이 되는 순간은 바로), 그 뒤마다 `motion`; 비상 끝은 즉시 `stop`과 래치; 정상 끝은 정지 확인 또는 1.5 s까지 `drive(0, 조향 유지)` 뒤 `stop` |
| 8 | 틱 기록(JSONL) 한 줄과 상태 스냅숏(`/api/status`) |

나란히 도는 스레드: ROS 콜백(받은 메시지를 큐에 넣기만, 스캔 16·IMU 400개 한도), 감시(50 ms마다, 주 루프가 0.35 s 넘게 안 끝나면
`drive(0, 마지막 조향)` 한 번과 `stall_guard` 일시정지), HTTP(STOP은 이 스레드에서 바로 브리지로), 기록 쓰기.

### 3.2 한도와 기본값 (`tools/row_executor/config.py`, 코드)

| 값 | 기본 |
|---|---|
| `control_hz` | 20 (고정) |
| `lidar_max_age_s` / `imu_max_age_s` / `steer_status_max_age_s` | 0.45 / 0.40 / 0.3 s |
| `stall_guard_s`, 허용 횟수 | 0.35 s, 3 |
| `fresh_recover_s`, 낡음 허용 횟수 | 1.0 s, 3 |
| `operator_timeout_s` (한 번 눌러 시작) | 1.0 s (결정 R60, 2026-11-13에 0.5 s에서 늘림) |
| `send_hz`, `terminal_stop_wait_s`, `bridge_timeout_s` | 10 Hz, 1.5 s, 0.6 s |
| `scan_latency_s`, 스캔 시각 방식 | 0.08 s, header_offset |
| `corridor_every_n_scans` | 3 (추정기 자체 기본은 2) |
| `gnss_mode` | off |

상태: Runner `IDLE`/`ACTIVE`/`PAUSED`/`ENDED`, 브리지 클라이언트 `DISARMED`/`ARMED`/`LATCHED`. 회복 가능한 멈춤(PAUSED)은 새 go로만 다시
출발하고, 멈춤이 3회를 넘으면 run을 끝낸다.

### 3.3 운영자 HTTP (`tools/row_executor/operator_http.py`)

`GET /api/status`(화면 2 Hz), `POST /api/start`, `/api/arm`, `/api/go`, `/api/heartbeat`(화면 100 ms마다), `/api/stop`, `/api/ack`,
`/api/end`, `/api/params`, `/api/calib_start`. 토큰 헤더 `X-Row-Token`, 본문 4 KiB, Host·Origin 검사. Gazebo에서는 127.0.0.1:8097.
대시보드는 토큰을 서버에서만 읽어 이 API를 중계한다(시간 한도 heartbeat 0.8 s, stop 2.5 s, 그 밖 2.0 s).

## 4. 잰 처리 시간과 지연 (측정)

`loop_ms`(한 걸음 계산 시간)와 `proc_ms`(스캔 처리)는 두 시계 모드 모두 **벽시계**다. 틱 시각, 입력 나이, 틱 틈은 SIM 시계 모드에서 **SIM 시간**이다.

| 수치 | 값 | 조건 | 출처 |
|---|---|---|---|
| 추정기 스캔 처리, 10/10 기록 재생 | P 코어 p50 13.4~20.0 / p99 48.4~50.3 ms, E 코어 p99 33.5~36.2 ms | 2026-11-13, Core Ultra 7 155H, Windows, Python 3.13, 코어 고정 | `docs/steering/EXECUTOR_PERF_KO.md` |
| 빠른 SIM 실행기 걸음 | p50 3.3~4.2 / p99 20.5~22.9 ms, 스캔 처리 p99 18.6~21.0 ms | 같은 날, P 코어 | 같은 문서 |
| Gazebo 8b단계(합격) | 스캔 처리 p99 11.0~19.0 ms(최대 69.1), 실행기 걸음 p99 20.5~21.4 ms, 조향 상태 나이 최대 0.142 s(한도 0.3) | 2026-11-15, 이미지 wf11, SIM 시계, 호스트 스레드 정책 '성능 코어 선호'(임시) | `docs/steering/STEER_FOLLOW_GSIM_FINAL_KO.md` |
| Gazebo 3단계(불합격 예) | 실행기 걸음 p50/p99/최대 18.3/209/323 ms, 스캔 처리 p99 105.6 ms, 조향 상태 나이 0.308~0.367 s → `steering_feedback_stale` | 2026-11-13, wf6, SIM 시계, steer 컨테이너 CPU 1.5 | 같은 문서 |
| 대시보드 → 실행기 상태 중계 | p50 18 / p95 124 / p99 276 / 최대 356 ms (155회, 정지 중) | 2단계, 벽시계 | 같은 문서 |
| STOP (Gazebo, 실행기 없음) | 요청 → STOP 바이트 8.8~13.5 ms, 확인 40~63 ms, 미확인 래치 400.9 ms | 2026-11-09, 벽시계 | `docs/RESULTS_KO.md` |
| STOP (빠른 SIM) | 요청 → STOP 바이트 0 ms(같은 브리지 틱), 확인 25 ms | T1 | `docs/steering/EXECUTOR_STEER_KO.md` |
| Jetson 실행기, 과수원 사과 2행(100 m 열) 주행 틱 | 틱 간격 p50 54.6 / p90 114.5 / p99 141.8 / 최대 199.1 ms(100 ms 넘음 19.2 %), 한 틱 계산 p50 17.4 / p99 108.6 ms, 스캔 처리 p50 13.4 / p90 67.3 / p99 88.9 / 최대 124.8 ms, LiDAR 나이 p50 144 / p99 204 ms | 현장 기록 2026-11-20, Jetson Orin Nano, 실행기 키트 `ee5804e`, 벽시계, 5번(15:23~16:52) | `docs/steering/AUTONOMY_RUNS_ANALYSIS_KO.md` 7.5절 |
| Jetson 실행기, 과수원 배 1행(40 m 열) 구역 주행 틱 | 틱 간격 p50 56.4 / p99 90.6 / 최대 117.3 ms(100 ms 넘음 0.13 %), 한 틱 계산 p99 58.4 ms, 스캔 처리 p50 9.1 / p99 40.4 / 최대 62.6 ms | 현장 기록 2026-11-20, 13번(17:08~18:00) | 같은 문서 |
| Jetson 실행기, 실내 복도 주행 틱 | 틱 간격 p50 60.0 / p99 121.8 / 최대 167.8 ms(100 ms 넘음 6.55 %), 한 틱 계산 p99 86.5 ms, 스캔 처리 p50 15.0 / p99 73.2 / 최대 116.9 ms | 현장 기록 2026-11-19 KST, 15번 | 같은 문서 |
| Jetson 스캔 처리, 틱 기록 한 파일 | 스캔 처리 시간이 남은 주행 틱 12,179개: p50 13.2 / p90 68.1 / p99 89.3 / 최대 124.8 ms. 같은 파일 모든 틱의 간격 p50 51.4 / p99 137 / 최대 192 ms | 현장 기록 2026-11-20, 16:13에 시작한 실행기의 첫 파일(사과 2행 16:26~16:52 네 번) | [`AUTONOMY_ALGORITHM_KO.md`](AUTONOMY_ALGORITHM_KO.md) 12.1절 |

- Jetson 줄의 스캔 처리는 근거리 확인과 추정기(`on_scan`)의 벽시계 시간이고 국소 격자 갱신은 들지 않는다. 2026-11-16에 적었던 추정(60~90 ms)은 사과 2행의 p90~p99 크기와 맞았다.
- 30 ms 넘은 느린 틱은 대부분 스캔 처리 단계에서 나왔다. 한 틱 계산이 150 ms를 넘은 주행 틱은 6개다(저녁 브리지 쓰기 실패 5번의 그 순간 407~433 ms, 16:44 주행의 스캔 처리 123 ms 한 번).
- 틱 간격이 0.35 s를 넘어 걸리는 감시(stall_guard)와 신선도 일시정지는 두 날 모두 0번이었다.
- 화면 버튼에서 바퀴 구동 0까지를 한 번에 잰 지연 분포는 없다(차량 시험을 마쳐 재지 못했다).

## 5. 행간 중심선 추정 L0 (`sim/ubts/closed_loop/estimator.py`)

실행기가 쓰는 값 = 추정기 기본값 + 실행기 덮어쓰기(`ExecutorConfig.estimator_overrides()`). 모두 SIM에서 정한 값이다.

1. **전처리**: LiDAR 장착 방향 보정(179.08°, 추정), 0·NaN·범위 밖 → 없음, 차체 마스크 안 반사 제거, 2 m 안 반사는 앞 두 스캔의 0.35 m 안에
   반사가 있어야 통과.
2. **IMU**: 요레이트 = 자이로의 위쪽 성분(비력 EMA, τ 1 s). 자이로 저역(0.25 s)과 정지 중 편향 학습.
3. **주행 거리계**: 12 m 안 점을 0.12 m 칸으로 줄여 키프레임 대비 점-대-점 ICP(12회, 짝 ≥ 25). 회전 초기값은 자이로 적분, 이동 초기값은
   LiDAR 속도 × dt. 맞음 비율 ≥ 0.5일 때만 위치 갱신, 실패면 자이로 회전만. 속도 = 최근 5개 스캔 단계 속도의 중앙값. 키프레임은 0.4 m·8°마다.
4. **통로 잡기**: 진행 방향 ±30° 안 2° 간격 축마다 옆 0.2 m × 앞 0.75 m 칸 점유를 세어 양쪽에서 가장 가까운 띠(|d| ≥ 0.5 m)를 찾는다.
   폭 1.2~8 m, 두 선 사이에 다른 반사가 있으면(사이 열) 거부, 다른 축이 거의 같은 세기면 '모호'로 거부. 같은 후보가 3번 이어지면 통로 생성.
5. **두 열 선**: 각 선 ±0.3 m, 앞뒤 8 m 안 점에 공통 기울기 최소제곱(잔차 0.18 m 넘는 점 빼고 두 번). 추적 중에는 통로 검출기
   (`sim/ubts/orchard_perception/scan_corridor.py`) 결과를 이 맞춤으로 다듬는다.
6. **갱신**: 방향 차 < 8°, 두 쪽: 중심 차 < 0.35 m·폭 차 < 0.5 m면 방향·폭에 이득 0.4, 중심에 0.25로 섞는다. 한쪽: 예측 선과 0.3 m 안이면
   배운 폭을 유지해 중심을 놓는다. 2 s 갱신이 없으면 통로를 잃는다.
7. **출력**: 기준점 = 차 발자국 중심. 횡오차 + = 차가 중심선 왼쪽. 방향 오차 = 행 축 − 차 방향(왼쪽 +). 통로 폭, 유효 여부와 이유, 갱신 나이.
   (평가의 방향 오차는 차 − 행 축으로 부호가 반대다.)
8. **열 끝**: 쪽마다 선 근처 지지점의 97 % 분위를 끝으로 둔다(6 m 이상 빈칸에서 자름). 굽은 열은 두 선 공통 포물선 d = d₀ + c·s²에서 잰다.
   통로를 가로지르는 구조물은 지지로 치지 않는다. 두 쪽이 다 끝난 갱신이 2.5 s 안에 두 번 이어지고 알린 끝이 1.0 m 안에서 맞으면
   `row_end_ahead_m`을 낸다.
9. **다른 층**: L1 구동기 사전값(켜짐, 가중 0: 기록·감시만), L2 국소 격자(20 × 10 m, 0.10 m 칸, 꺼짐), L3 GNSS(off; RTK 약한 제약
   `weak_constraint`는 RTK FIXED일 때 제어기 입력 횡오차만 ±0.10 m·0.05 m/s 안에서 당김, [`GNSS_RTK_KO.md`](steering/GNSS_RTK_KO.md)).

## 6. 열 추종 제어기 (`sim/ubts/closed_loop/steer_control.py`)

입력은 추정기 스냅숏과 조향 상태뿐이다(참값·GNSS·duty 적분 거리 없음).

```
v     = max(최소 속도, LiDAR 속도)
L_d   = clamp(1.5 + 1.25·v, 1.5, 3.0)                       0.40 m/s에서 2.0 m
y_L   = −k_d·횡오차 + k_ψ·L_d·sin(방향 오차)                  k_d = k_ψ = 1.0
κ     = 2·y_L / L_d²
δ_raw = atan(L·κ)                                           L: 기본 0.76 m, 측정 차 0.657 m
trim += 0.05·y_L·dt  (주행 중, 한계 안일 때만, |trim| ≤ 3°)
δ     = clamp(δ_raw + trim, ±12°) → 저역 τ 0.25 s → ±0.9 × 소프트 한계 → 변화율 ≤ 0.75 × 실행기 한도
```

부호는 + = 왼쪽 조향. 요레이트 안쪽 루프(`k_r`)는 자리만 있고 0이다. 측정 차의 변화율은 4.075 × 0.75 ≈ 3.06 °/s다(옛 SIM 프로필 값, 2026-11-20 현장 설정은 7.88 °/s).

2026-11-20 현장 설정(코드 기본값과 다른 것, 그날 틱 기록의 `config` 사건): 주시거리 바닥·상한 2.5 / 4.0 m, `speed_window_s` 1.5 s(위 식의 v = max(LiDAR 속도, 1.5 s 경로 거리 변화율)), 목표 속도 오후 0.45·저녁 0.40 m/s, 속도 배율 0.93, 조향 상태 한도 0.6 s, 국소 격자 켬(4 스캔마다, 융합 이득 0). 전체 표는 [`AUTONOMY_ALGORITHM_KO.md`](AUTONOMY_ALGORITHM_KO.md) 16절이다.

**속도 상한** (v_ref = 최솟값)

| 상한 | 식 | 기본 |
|---|---|---|
| 순항 | 목표 속도 | 0.40 m/s (0.20~0.50) |
| 스캔 | 0.05 m / 스캔 간격 p90 | 10 Hz면 0.50 m/s |
| 연산 지연 | 0.15 m / τ_loop (스캔 나이 p90 + 0.24 s + 틱 늦음) | 약 0.44 m/s |
| 정지 거리 | v·(τ_loop + 0.25) + v²/(2·1.07) ≤ 앞 여유 − 0.35 m | 감속 1.07 m/s²(추정) |
| 곡률 | √(0.3 / \|κ\|) | 옆 가속 0.3 m/s² |
| 횡오차 | \|횡\| 0.15 → 0.40 m에서 순항 → 최소 | |
| 조향 요구 | \|δ\|/12°가 0.8 → 1.0에서 순항 → 최소 | |
| 품질 | 한쪽 갱신, 갱신 나이 > 0.6 s, 예측 중이면 최소 | |
| 신선도 | 최신 스캔 나이 > 0.30 s면 최소 | |
| 열 끝 | 남은 거리 ≤ 1.5 m + 정지 거리면 접근 속도 | 0.20 m/s |

v_ref가 0.20 m/s 아래면 `speed_cap_below_min`(회복 가능 멈춤). LiDAR 속도가 0.55 m/s를 1 s 넘게 넘으면 `overspeed`.

**duty**: 표⁻¹(v_ref / speed_scale) + 30·(v_ref − v_LiDAR) + I(15·∫e, ±15 %), 올림 5 %/s·내림 15 %/s, 1~70 %. LiDAR 속도가 목표보다
빠르면 올리지 않는다. 기본 속도 표는 (25 %, 0.239), (80, 0.709), (90, 0.715), (100, 0.899) m/s로 조향 장착 전 차의 기록에서 온 자리값이다.

**단계**: ALIGN(서서 통로를 잡고 첫 목표각까지 조향, 6 s 안에 못 잡으면 끝) → FOLLOW → APPROACH_END → STOP(throttle 0, 조향 유지) → HOLD.
열 끝 정지 = 남은 거리(`row_end_ahead_m` − 앞범퍼 0.61 m) ≤ 정지 거리. 기하 가드: |방향 오차| > 25° 또는 차가 통로 밖이면 끝.

**설계와 코드가 다른 곳(설계)**: 뒤차축 기준 횡오차, `k_r` 0.3, 조향 반전 히스테리시스는 설계 문서(`DESIGN_v1_KO.md` 6.2, `DESIGN_v2_KO.md`
V2-5)에만 있고 코드에는 없다.

## 7. 기록과 재현

| 기록 | 위치·형식 | 내용 |
|---|---|---|
| 실행기 JSONL | `row_executor_<UTC>_<pid>_<n>.jsonl`(Gazebo `/run/ubts-steer/exec/log/`, 차 `/var/log/row_executor`), 100 MB마다 새 파일 | 틱 줄 `k:'tick'`: 시각, run 상태, 이유, 출력 L·R·조향, 통로(횡·방향·폭·끝·앞 여유), 속도, 요레이트, 입력 나이, 스캔 처리(`proc_ms`, 나이, 버림), `loop_ms`, 조향 상태. 사건 줄 `k:'event'`(config, start, stop, request 등) |
| 실행기 상태 | Gazebo `/run/ubts-steer/exec/status.json` 1 s마다, HTTP `/api/status` | 모드, 링크, 출발 막힘 이유, 정지 이유 |
| 기록기 | `/runtime/recordings/<UTC>_<kind>/rosbag2/` + metadata, timeline, gps.jsonl, motor.jsonl | `/rover/status`, `/rover/control/requested`·`applied`, `/odom`, `/tf`, `/clock`, IMU·GNSS(선택), `/scan`, 카메라(종류에 따라) |
| 조향 사이드카 | 기록 폴더 안 `steering/steering.jsonl`(+ mcap) | MCU 상태 줄마다: 가변저항 raw, 측정·목표 조향각, 조향 PWM, 좌우 구동, TTL 남은 시간, 결함 |
| 에뮬레이터·브리지 | `/runtime/steer/emulator_log.jsonl`, `bridge_events.jsonl` | 직렬 줄, 요청·답, 상태 전이 |
| Gazebo 열 추종 검사 | `GSIM_FOLLOW_RUN.json`, `GSIM_FOLLOW_TRACE.json` | 참값 경로, 실행기 틱 요약, 장면 기하(검사 컨테이너만 참값을 읽음) |
| 빠른 SIM | `TRACE.npz`(10 ms마다 1행), `SERIAL.jsonl`, `EXECUTOR.jsonl`, `RESULT.json` | 참값·명령, 직렬 줄, 실행기 기록, 요약과 각 파일 SHA-256 |

run의 매개변수 신원: 실행기 `config` 사건에 설정 전체·`config_digest`(정규 JSON의 SHA-256)·소스 digest·조정 파일 요약, 현장 조정 파일의
`digest`와 `file_sha256`, 빠른 SIM `RESULT.json`의 파일 해시. 프로필은 고치지 않고 새 판을 만든다.

그림 명령:

```bash
python -B tools/steer_sim/gsim_follow_figures.py --runs <run> [--top <run>] [--out DIR]     # Gazebo 경로·오차 그림
cd sim/ubts && python -B -m steering.safety_scripts.follow_matrix --out <dir>               # 빠른 SIM 행렬
python -B -m steering.safety_scripts.follow_figures --runs <matrix> [--animation]          # 행렬 그림
python -B -m steering.evaluate_steer executor --run <run>                                   # 실행기 run 평가
python -B tools/row_executor/sim_in_loop_steer.py episode --spec <spec.json> --out <dir>    # 빠른 SIM 한 run
```

## 8. 평가 정의

- 참값 격리: 정적 검사(`sim/ubts/steering/truth_isolation.py`)가 결정 코드의 참값 읽기를 막는다. Gazebo 실행기는 `/scan`, `/imu/data_raw`,
  조종 요청만 구독한다.
- `steer_follow_v1`(빠른 SIM, W0; `docs/steering/PREREG_STEER_FOLLOW_KO.md`): 주행 구간 = 구동 명령 > 0인 첫 trace 행 ~ 마지막 행.
  횡오차 = 참 발자국 중심의 y(중심선 y = 0), 방향 오차 = 참 방향각. RMSE = √mean(y²), p95 = |y|의 95 % 분위, 최대 = max|y|
  (출발 어긋남 조건은 RMSE·p95를 6 m 뒤부터). e = 정지 때 앞범퍼 x − 24.0 m.
  합격: 접촉 없음, 끝 속도 ≤ 0.02 m/s, 끝 이유 `row_end`, 멈춤 0, 횡 RMSE ≤ 0.20·p95 ≤ 0.35·최대 ≤ 0.50 m, 방향 RMSE ≤ 5°·최대 ≤ 15°,
  속도 ≤ 0.50 m/s, e ∈ [−1, +1] m, 끝난 뒤 3 s 조향 명령 불변.
- `steer_follow_wide_v2`(`PREREG_STEER_FOLLOW_V2_KO.md`): 굽은 열은 중심선 호 좌표 (s, d). e 기준 = 행 줄의 마지막 구조물(결정 R50).
  W2는 2차 판정부터 횡오차 기준선 = 참 통로 중심(s ±2 m, |d| 0.3~3.5 m 반사 칸의 양쪽 10 % 분위 가운데).
- `steer_follow_gsim_v1`(Gazebo W1; `PREREG_STEER_FOLLOW_GSIM_KO.md`): 중심선 = 출발 통로 양쪽 나무 줄기에 맞춘 두 최소제곱 직선의 평균,
  e = 앞범퍼 s − 두 열 줄기의 최대 s. 실시간 비율 < 0.9 또는 `/clock` 틈 > 100 ms인 run은 무효.

## 9. 매개변수의 근거

- **측정**: 2026-11-16 치수(휠베이스, 윤거, 바퀴, 조향 한계, LiDAR 위치)와 보드 계단 응답. 구동 duty → 속도는 조향 장치를 달기 전 차의
  2026-10-03·10-10 수동 주행 14개를 오프라인 LiDAR ICP로 잰 값(100 % 0.928 m/s, 90 % 0.707 m/s; 직선 근사 v = 0.00855·duty + 0.025).
  25~60 %는 duty마다 구간이 1~3개뿐이고 40 % 아래는 자료가 없다([`drive_response/README_KO.md`](drive_response/README_KO.md)).
- **자리값**: 제어기 값 전부, 현장 조정 기본값, 실행기 조향 GO 문턱 8 °/s, 정지 감속 1.07 m/s², 연산 지연 고정 몫 0.24 s, 조향 장치 사양 34칸.
- **SIM에서 맞춤**: 한 행 추정기 옵션, 열 끝 일치 1.0 m, 굽은 열 끝 맞춤, 측정 차 GO 문턱 4.0 °/s(사전등록에 문서화한 변경), 펄스 유지 흉내의
  구동기 값.
- **현장 조정 파일**(`tools/row_executor/field_tuning.default.json`, 결정 R35): basic(화면에서 정지 중에만: 목표 속도, 주시거리, k_d, k_ψ,
  조향 변화 비율, 속도 배율), advanced(파일만: 속도 표, PI, duty 범위, 열 끝 값, trim 등), locked(파일만, 재시작: 최대 속도 0.50, duty 상한 75,
  신선도 한도, 운영자 1.0 s, 과속, 기하 가드). 바꿀 때마다 변경 기록이 남는다.

## 10. 알려진 문서·코드 차이

- `sim/ubts/steering/exec_container_main.py` 머리말은 제어기 칸을 'SIM 대본'이라 하지만 compose는 `row_follow`로 띄운다.
- `compose.ubts-steer.yaml` 머리말의 대시보드 사본 커밋(a92cfec)은 옛 이미지 기준이다. 사본은 `dashboard/DASHBOARD_SOURCE.json`이 기준이고,
  compose가 `dashboard/`를 마운트하지 않으므로 새 사본은 이미지를 다시 만들어야 돈다.
- `docs/steering/P6A_HEADLESS_KO.md`는 초기 스택 기준이라 `/scan` 노드·IMU 자세·상태 주기가 지금 코드와 다르다.
- `sim/ubts/rover.yaml`의 IMU 20 Hz·GNSS 원점은 조향 스택이 33 Hz·원점 0/0으로 덮어쓴다.
- LiDAR 잡음은 Gazebo 0.01 m, 프로필·빠른 SIM 0.015 m다(코드에 알려진 차이로 적혀 있음).
- 방향 오차 부호는 추정기(행 축 − 차)와 평가(차 − 행 축)가 반대다.
