# S2 형식 노드와 실행기 컨테이너 (P6a 나머지, 작업 표지 s2exec)

- 작성: 2026-11-09 (4단계 WF-4, 빌드 단계 s2exec).
- 설계: `DESIGN_v1_KO.md` 2.2~2.6, 3.7, 4.3, 8.3 P6a('S2 형식 노드', '실행기 컨테이너'). 결정: `DECISIONS_KO.md` R16, R18, R19, R20.
- 합격 기준: `SIM_ACCEPTANCE_KO.md` A5, A13(개정 11), A17(7.2.1, 개정 4·7·10), A19(G, 개정 17). 기준은 바꾸지 않았다.
- 모든 값은 자리값 프로필 v0.1(`front_steer_v0_1_placeholder`)에서 온다. 결과는 **자리값 기반 SIM 결과**다. 현장 검증이 아니다. 이 문서에는 GNSS 좌표, 지도 원점, 실제 기록의 궤적 절대값이 없다.
- 이 단계의 스택 측정은 개발 값이다. 시간 민감 판정(실차 시계 A17 STOP, A10 W2)은 R20대로 다른 단계가 돌지 않는 최종 자격 검사에서 다시 한다.
- **Docker 검증 전(2026-11-12).** 4단계 작업 흐름이 2026-11-09에 멈췄을 때 이 단계는 스택 측정 중이었다. 그 변경을 새 저장소로 옮겼고(`DECISIONS_KO.md` R22) Docker 없이 도는 시험(`test_steering_s2_node.py`, `test_steering_exec_container.py`, `test_steering_gz_model.py`, `test_steering_truth_isolation.py`)만 확인했다. 7~12절은 스택 측정(ST-05) 뒤에 채운다. compose 기본 이미지는 아직 `ubts-steer-sim:p6a`(실행기 패키지 없음)이고, 새 이미지는 ST-05가 커밋한 트리로 만든다. 스택을 올리고 내리는 규칙은 `STACK_OPERATIONS_KO.md`다.

## 1. 요약

1. S2 형식 노드 `sim/ubts/steering/s2_format_node.py`(노드 `ubts_steer_s2_format`)를 만들었다. `/sim/scan`(field1010에서는 `/sim/scan_clipped`)을 받아 `/scan`을 낸다. 변환은 F-SIM과 같은 코드(`s2_format.RealFormat`)다. 감독기가 기본으로 켠다(`--s2-node none`이면 끈다).
2. 실행기 컨테이너 `sim/ubts/steering/exec_container_main.py`와 compose 서비스 `executor`(프로필 `executor`)를 만들었다. 실제 실행기(`tools/row_executor`)가 대본 제어기를 슬롯에 넣고 MOTOR 모드로 돈다. 제어 소켓만으로 브리지와 말한다. `--clock wall|sim` 두 모드가 있다.
3. 감독기(`supervisor_steer.py`)에 S2 노드 준비 확인, 옛 상태 파일 정리(감사 G06), steer 역할의 브리지 신호 걸이를 더했다.
4. 검사 모듈 `sim/ubts/steering/exec_checks.py`를 만들었다: A13(G) 실행기 구독, A5(G) `/scan`·IMU·카메라·가상 GPS, W2 감지 사슬, A17(G) 실행기 고리 대본 22개.
5. `docker/Dockerfile.ubts-steer`가 `tools/row_executor`를 복사하고 새 모듈을 import해 본다. 새 이미지는 만들지 않았다(이미지 ID 상한은 통합 단계 몫). 이 단계의 스택은 `ubts-steer-sim:p6a`에 스냅숏 소스를 읽기 전용으로 붙여 돌렸다(9절).

## 2. S2 형식 노드

### 2.1 토픽 계약

| 항목 | 값 |
|---|---|
| 입력 | `--input`(감독기: 자르기 노드가 있으면 `/sim/scan_clipped`, 없으면 `/sim/scan`). LaserScan, best effort, keep last 10 |
| 입력 규약 | cpu_lidar `ScanSynthesizer`: 3200빔, 2π, angle_min −π, 증분 2π/3200, 몸체 좌표, frame `laser`. 다르면 변환하지 않고 이유별로 센다(`beam_count`, `angle_min`, `angle_increment`, `frame_id`, `stamp_invalid`). 모르는 형식을 짐작해 바꾸지 않는다 |
| 출력 | `/scan`, LaserScan, reliable, keep last 10(실제 드라이버 sllidar_ros2와 같은 QoS) |
| ranges | 3240빔, raw 순서, float32, 1 mm 반올림. 반사 없음(+inf, NaN, 30 m 이상)은 0.0 |
| angle_min / 증분 / angle_max | float32(−π) = `RAW_AMIN`, `RAW_INC`, angle_min + 3239 × 증분 |
| frame_id | `laser`. raw 빔 j는 몸체 각 wrap(angle_min + j × 증분 + `lidar_yaw_offset_deg`)를 본다(프로필 값 179.08°) |
| range_min / range_max | 0.05 m / 30 m |
| scan_time / time_increment | 0.1 s / 0(F-SIM `scan_message`와 같다). intensities 없음 |
| header.stamp | 입력 stamp 그대로(cpu_lidar가 쓴 자세의 SIM 시각. 자르기 노드도 그대로 둔다). F-SIM처럼 실행기는 SIM 센서 stamp를 본다 |
| 한 스캔 | 입력 하나에 출력 하나. 합치거나 다시 쓰거나 몰래 버리지 않는다 |

- 노드는 참값을 읽지 않는다(자세, `/ground_truth/*`, `/sim/truth/*` 구독 없음). `p6a_graph.S2_NODE`는 `/sim/scan`·`/sim/scan_clipped`의 허용 구독자이고 참값 허용 목록에는 없다.
- 변환 코드는 F-SIM `sim_in_loop_steer`가 쓰는 `s2_format.RealFormat` 하나다(R16). 시험이 바이트 동일을 확인한다.

### 2.2 계수와 상태 파일

- `--status-file`(감독기: `/runtime/s2_node_status.json`), 약 1 s마다: state(waiting, running, input_rejected, stopped), 계수(입력, 출력, 거부와 이유, stamp 틈, 빠진 스캔 추정, 줄지 않은 stamp, 반사 있음·없음 빔 수), 변환 시간과 콜백 시간(last, mean, max ms), 마지막 입력·출력 stamp, 감독기의 실행 토큰, 프로필 id, `lidar_yaw_offset_deg`, 출력 형식.
- stamp 틈: 이웃 입력 stamp 간격이 주기(0.1 s)의 1.5배를 넘으면 틈 1개, 빠진 스캔 round(간격/주기) − 1개로 센다. 앞 단계(cpu_lidar, 자르기 노드, 전송)에서 빠진 스캔이 조용히 묻히지 않게 한다.

## 3. 감독기 변경 (`supervisor_steer.py`)

### 3.1 S2 노드 연결

- `--s2-node`(환경 변수 `UBTS_STEER_S2_NODE`) 기본값은 `steering.s2_format_node`다. `none`, `off`, `0`이면 켜지 않는다(P6a처럼 `/scan` 없음).
- 명령: `python3 -B -m MODULE --input <입력> --output /scan --status-file <runtime>/s2_node_status.json --run-token <토큰> [--profile P]`(`s2_node_command`). 시험이 이 명령을 노드 자신의 인자 해석기로 읽어 맞춘다.
- 준비: 상태 파일이 이번 실행 토큰이고 waiting 또는 running이며 10 s 안에 갱신됐다(`s2_node_ready`, 60 s 한도).
- 순서: gz → ros_gz_bridge → cpu_lidar → [자르기 노드] → S2 노드 → 센서 계약(`--s2-present`: `/scan`을 구독하고 `sensors_alive`에 `scan`을 더한다) → WebSocket 다리.

### 3.2 옛 상태 파일 정리 (감사 G06)

- runtime 볼륨은 실행 사이에 남는다. 그래서 앞 실행의 상태 파일이 뒤 실행의 결과에 섞였다(W1 결과에 field1010 자르기 노드 값).
- 각 역할은 자식을 켜기 전에 자기 자식들의 상태 파일(`ROLE_STATUS_FILES`)을 `<runtime>/steer/retired/<이번 실행 토큰>/`으로 옮긴다. 이번 장면이 돌리지 않는 노드(W0의 자르기 노드, `--s2-node none`의 S2 노드)와 앞 실행의 파일이 모두 빠진다. 준비 확인이 옛 파일을 이번 실행 것으로 읽는 일도 없어진다.
  - sim: `simulation_status.json`, `cpu_lidar_status.json`, `scan_clip_status.json`, `s2_node_status.json`, `camera_driver_status.json`, `imu_driver_status.json`, `lidar_bridge_status.json`, `camera_bridge_status.json`, `imu_bridge_status.json`, `gps_bridge_status.json`
  - steer: `steering_status.json`, `rover_bridge_status.json`
  - recorder: `data_recorder_status.json`
- 옮긴 목록은 `supervisor_<역할>.json`의 `retired_status_files`와 사건 `RETIRED_STATUS`에 있다. retired 폴더는 최근 5개만 남긴다.
- 검사 쪽 규칙(이번 실행 토큰 뒤에 갱신된 파일만 넣기)은 `p6a_checks`(g_gaps) 몫이다.

### 3.3 감독기 걸이 (steer 역할, SIM 전용)

- 소켓 `--supervisor-hook-socket`(기본 `/run/ubts-steer-emu/supervisor.sock`, hooks 볼륨. 실행기 컨테이너는 이 볼륨을 붙이지 않는다). 한 연결에 JSON 한 줄.
- `{"op": "status"}`: 자식과 pid. `{"op": "signal", "child": "bridge", "signal": "KILL" | "TERM" | "STOP" | "CONT", "resume_after_s": s}`: 브리지 래퍼 프로세스에 신호를 보낸다. `resume_after_s`는 STOP에만 쓰고(0 < s ≤ 30) 그 뒤 CONT를 보낸다.
- 쓰는 곳: 실행기를 고리에 넣은 '브리지 강제 종료'(KILL, 감독기가 1 s 뒤 다시 켠다)와 '브리지 멈춤'(STOP 1.5 s). 종료할 때 감독기는 TERM 뒤 CONT도 보낸다(멈춘 자식도 끝난다).
- 같은 목적의 에뮬레이터 걸이 `kill_bridge`를 g_gaps가 따로 만드는 중이다. 이 걸이는 STOP/CONT도 다룬다.

## 4. 실행기 컨테이너 (`exec_container_main.py`)

### 4.1 서비스와 자원

| 항목 | 값 |
|---|---|
| compose 서비스 | `executor`, 프로필 `executor`(그냥 `up`으로는 켜지지 않는다) |
| 이미지 | 같은 기반 이미지(`UBTS_STEER_IMAGE`). WF-4 Dockerfile이 `tools/row_executor`를 `/opt/ubts/tools/row_executor`로 복사한다. p6a 이미지에는 없어 override로 붙인다 |
| 네트워크·IPC | `service:sim`(ROS 도메인 43, FastDDS 공유 메모리, sim 네트워크 이름공간의 127.0.0.1) |
| 볼륨 | `control:/run/ubts-steer`만. runtime 볼륨(SIM 상태·참값 파일)과 에뮬레이터 hooks 볼륨은 붙이지 않는다 |
| 메모리·CPU | 512m, 1.5 CPU |
| 시계 | `UBTS_STEER_CLOCK`(브리지 래퍼와 같은 값) |
| 재시작 | no, stop_grace_period 20 s |

- 6 GiB 예산: sim 3.5 + 대시보드 1 + steer 0.5 + recorder 0.5 + 실행기 0.5 = 6.0(설계 P6a 자원). 검사 서비스(0.5)는 따로 자리를 쓴다. 실행기와 함께 검사할 때는 대시보드 없이 켠다(sim + steer + recorder + 실행기 5.0 + 검사 0.5). 시험이 compose 파일에서 세 조합을 확인한다.

### 4.2 설정

- `gsim_executor_config`: T1 설정(`sim_in_loop_steer.steer_executor_config`)과 같은 값에 SIM 제어 소켓(`/run/ubts-steer/control.sock`)과 운영자 페이지 주소(127.0.0.1:8097)를 더한다. 차량 프로필 `front_steer`, 단일열 모드, MOTOR, 프로필에서 온 SIM '측정'값(SIM_ASSUMED 자리값), `steer_allow_sim` 참, `steer_cal_id` 1(steer 역할이 내보내고 MCU가 보고하는 값).
- 시험이 두 설정을 견준다: 소켓·주소 세 칸 말고는 같다.
- `--config-overrides`(또는 `UBTS_STEER_EXEC_OVERRIDES`)로 칸을 덮을 수 있고 `validate()`를 통과해야 한다. 실제 설정은 `/run/ubts-steer/exec/config.json`에 쓴다(digest 포함).
- 소스 검사: 실차처럼 `verify_sources()`(SHA256SUMS)와 `verify_vendor()`를 돈다. `--skip-source-check`(개발 스냅숏용)는 config.json과 상태에 적힌다.

### 4.3 제어기 슬롯

- Runner의 `controller_factory`는 대본 제어기(`scripted_control`) 하나다. 행 추종 제어기는 P8이다.
- `--controller`(또는 `UBTS_STEER_EXEC_CONTROLLER`) JSON, 기본 `{"kind": "straight_hold", "drive_pct": 40, "distance_m": 8.0}`. 재개마다 새 제어기를 만든다. SIM 걸이 `controller`로 run 사이에 바꿀 수 있다(ACTIVE 중에는 거부).

### 4.4 시계 모드

| 부분 | `--clock wall` | `--clock sim` |
|---|---|---|
| ROS 입력 | `executor_main.RosInput` 그대로(수신 시각 `time.monotonic()`) | `SimRosInput`: 같은 토픽·QoS + `/clock`. 수신 시각은 콜백 때의 SIM 시각 |
| 루프 | `executor_main.run_loop` 그대로(단조 시계 20 Hz) | `run_loop_sim`: 2 ms(벽시계)마다 보고 SIM 시각이 다음 50 ms 격자점에 닿으면 한 번 돈다. 여러 격자점을 건너뛰면 한 번만 돌고 건너뛴 수를 센다(따라잡지 않음). 예외 처리는 MOTOR의 run_loop 규칙 |
| 실행기 시계(정지 감시 0.35 s, 누름 만료 0.5 s, 조향 상태 나이, 클라이언트 시각) | 단조 시계 | SIM 시각 |
| 벽시계(기록 'wall', 스냅숏 나이 검사) | time.time() | time.time() |
| 브리지 기한(입력 0.75 s, release 1.1 s) | 브리지 래퍼의 시계(같은 모드) | 같음 |

- 실차와 같은 기한을 쓴다: 브리지 코어 `SteerControl.RELEASE_TTL` 1.1 s, 입력 기한 0.75 s(옛 SIM의 release 유예 10 s는 이 스택에 없다). 시험이 값을 확인한다.
- SIM 시계 모드의 실행기 노드만 `/clock`을 구독한다(개정 11).

### 4.5 운영자 페이지

- `operator_http`를 127.0.0.1:8097(sim 네트워크 이름공간)에 연다. 토큰은 `/run/ubts-steer/exec/log/operator_token.txt`(0600). 검사(`exec_checks`)가 운영자처럼 쓴다: Start, Arm, 누름 심장 박동 100 ms마다, 놓기, STOP, ack.
- 대시보드 컨테이너는 다른 네트워크 이름공간이라 이 페이지에 닿지 않는다(P6b 화면 연결은 ui·통합 단계 몫).

### 4.6 SIM 걸이 (`/run/ubts-steer/exec/hooks.sock`, SIM 전용)

| op | 뜻 |
|---|---|
| `status` | 시계, Runner 상태·사유, 클라이언트 스냅숏, 계수, 걸이 상태 |
| `controller {spec}` | 다음 재개의 대본 제어기(ACTIVE 중 거부) |
| `stall {duration_s}` | 주 루프가 그 시간 동안 돌지 않는다. 정지 감시 스레드는 돈다 |
| `freeze {duration_s}` | 프로세스 전체 SIGSTOP. 도우미 프로세스가 시간 뒤 SIGCONT |
| `kill` | 실행기 프로세스 SIGKILL(조종 프로세스 강제 종료). 컨테이너는 다시 켜지지 않는다 |
| `wall_offset {seconds}` | 실행기의 벽시계만 뛴다(09-19 '시스템 시각 변경'의 실행기 쪽) |

- 응답을 보낸 뒤 freeze·kill을 한다.

### 4.7 파일

- `/run/ubts-steer/exec/`(control 볼륨): `config.json`, `status.json`(1 s), `log/`(실행기 JSONL, 최근 30개 유지, 운영자 토큰), `hooks.sock`. 앞 실행의 `status.json`은 시작 때 `retired/<토큰>/`으로 옮긴다(G06).

### 4.8 명령줄

```
python3 -B -m steering.exec_container_main --clock wall|sim [--controller JSON] [--config-overrides JSON]
    [--profile YAML] [--cal-id N] [--exec-dir DIR] [--socket PATH] [--bind-port N] [--max-log-mb MB]
    [--skip-source-check] [--max-runtime-s S]
```

## 5. compose 사용법

```
python -I <tools>/docker_budget.py acquire --label <표지> --gib 6
# 전체 스택(6.0 GiB): sim, dashboard, steer, recorder, executor
docker compose -p ubts-steer -f compose.ubts-steer.yaml --profile executor up -d
# 실행기와 검사(5.5 GiB): 대시보드 없이
docker compose -p ubts-steer -f compose.ubts-steer.yaml --profile executor up -d sim steer recorder executor
docker compose -p ubts-steer -f compose.ubts-steer.yaml --profile executor run --rm --no-deps check \
    python3 -B -m steering.exec_checks <검사> --out /runs/<폴더>
docker compose -p ubts-steer -f compose.ubts-steer.yaml --profile executor down -t 20     # -v 없이
python -I <tools>/docker_budget.py release --label <표지>
```

- `run`에는 `--no-deps`와 `up` 때와 같은 환경 값을 쓴다(P6A 6절).
- p6a 이미지에는 `tools/row_executor`가 없다. 새 이미지 전까지는 override로 `/opt/ubts/tools/row_executor`(그리고 커밋할 `sim/ubts`, `dashboard/scripts`)를 읽기 전용으로 붙인다(9절).

## 6. 검사 (`exec_checks.py`)

| 검사 | 내용 |
|---|---|
| `graph` | A13(G): 모든 토픽의 발행·구독(`ROS_GRAPH.json`, `GRAPH_LISTING.txt`), p6a_graph R1~R5, 실행기 노드 R6(개정 11: `/scan`, `/imu/data_raw`, `/rover/control/requested`, `/parameter_events`, `/rosout`, SIM 시계 모드에서만 `/clock`), 참값·금지 토픽 0, S2 노드는 입력 하나만 읽고 `/scan`을 낸다 |
| `scan` | A5(G) `/scan`: 형식 항목, 0 = 반사 없음, 10 Hz(헤더 stamp), F-SIM 변환과 바이트 동일(같은 stamp의 SIM 스캔), W0 기둥으로 `lidar_yaw_offset_deg` 관례(참값 자세, 평가), S2 노드 계수(입력 = 출력, 거부 0, 틈 0), 정지 거리 잡음(보고) |
| `imu` | A5(G) `/imu/data_raw` 정지: 주기(프로필 ±10 %), 자이로·가속도 잡음 sd(이번 실행 IMU 오차 모델, ±10 %, A5(F)와 같은 허용), frame, 자세 미제공 |
| `display` | A5(G) 표시: `/camera/image_raw` 640×360·30 Hz, 카메라 WebSocket(대시보드가 중계하는 흐름)의 JPEG를 640×360으로 해독, `/gps/fix` 0 N 0 E 근처, GPS WebSocket 스냅숏 |
| `chain` | W2 감지 사슬(field1010): 헤더 stamp로 짝지어 단계별 지연(앞 단계 수신 대비, 첫 단계는 SIM 시계가 stamp에 닿은 벽시각 대비), SIM·벽시계 율, 단계별 빠진 스캔, 노드 계수. 개발 값 |
| `safety` | A17(G) 실행기 고리 대본 22개(7절 표). 항목마다 측정값과 등록 기준. 실차 시계에서는 대본마다 A10 유효성(개정 10). 몰아 받은 A19(G) 정상 대본 오검출 |
| `drive` | 대본 제어기 run 하나와 실행기의 입력 수용(개발) |

## 7. 결과

(작성 중: 스택 측정 뒤 채운다.)

## 8. 발견한 것

(작성 중)

## 9. 이미지와 스냅숏

(작성 중)

## 10. 시험하지 않은 것

(작성 중)

## 11. 다른 단계에 넘기는 것

(작성 중)

## 12. 시험

(작성 중)
