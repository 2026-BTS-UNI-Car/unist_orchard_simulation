# P6a 헤드리스 G-SIM 스택 (실행기 없음)

- 작성: 2026-11-09. 설계 8.3 P6a 기록이다. 작업 표지 p6a(WF-3b).
- 코드(새 파일): `sim/ubts/steering/gz_esp_emulator.py`(ESP 에뮬레이터), `supervisor_steer.py`(감독기), `sensor_contract_steer.py`(센서 계약), `verify_steer.py`(통합 확인), `p6a_client.py`(대본 클라이언트), `p6a_checks.py`(검사), `p6a_graph.py`(ROS 그래프 규칙), `p6a_imu_model.py`(IMU 오차 모델), `bridge_steer.yaml`, 저장소 루트 `compose.ubts-steer.yaml`. 브리지 래퍼 `dashboard/scripts/steer_rover_bridge.py`를 고쳤다(SIM 시계 격자, 고정 20 ms 격자, 이벤트 기록).
- 시험: `sim/ubts/test_steering_p6a.py`, `sim/ubts/test_steering_p6a_emulator.py`, `dashboard/scripts/test_steer_rover_bridge.py`.
- 이미지: `ubts-steer-sim:p6a`(14절). 최종 검사는 모두 이 이미지의 자기 소스로 돌렸다(소스를 붙이지 않음).
- 모든 값은 자리값 프로필 v0.1(`front_steer_v0_1_placeholder`)에서 온다. 결과는 "자리값 기반 SIM 결과"다. 현장 검증이 아니다.
- 실행기 컨테이너와 S2 형식 노드는 다음 단계다. 이 스택에는 `/scan` 발행자가 없다(5절).
- 2026-11-09 리뷰(review_g RG-01~13, review_world RW-01)를 반영해 고쳤다. 고친 내용은 15절, 다시 잰 결과는 9.9절이다. 9.0~9.8절은 처음 측정이다. 그 가운데 9.6절의 "모두 통과"는 리뷰에서 근거가 부족하다고 확인됐다(정상 대본 오검출 미측정, 판정 시각 허용이 설계값보다 넓음). 지우지 않고 남기며, 판정은 9.9절을 따른다.

## 1. 프로세스 그래프

컨테이너 넷(compose 서비스)과 그 안의 프로세스다. 감독기(`supervisor_steer.py`)가 역할마다 하나씩 돈다. 자식은 순서대로 켜고, 준비 확인(gz가 본 토픽, 상태 파일, pty 링크)을 시간 한도 안에 통과해야 다음으로 간다. 끌 때는 거꾸로 된 순서로 모든 자식에 SIGTERM을 보내고(기다리지 않고 이어서 보냄), 6 s 뒤 남은 것에 SIGKILL을 보낸다.

- 실행 표지(RG-10): sim 감독기는 시작할 때 새 토큰을 만든다. 장면 READY 표지(JSON)와 센서 계약의 `simulation_status.json`(`sim_run`)에 같은 토큰을 넣는다. steer 감독기는 READY 표지와 토큰이 같고, 3 s 안에 갱신됐고, `clock_alive`가 참인 상태 파일을 본 뒤에야 자식(에뮬레이터, 브리지)을 켠다. 그래서 SIGKILL로 남은 이전 표지를 쓰지 않고, gz·Embree BVH·센서 계약의 시작 부하가 끝난 뒤에 브리지가 켜진다.

```
sim 컨테이너 (supervisor --role sim)            steer 컨테이너 (--role steer, sim의 네트·IPC 공유)
  1 gz_assemble: 장면 w0_force | w1_force |      0 sim 역할의 READY 표지 + 같은 토큰의 살아 있는 /clock 대기
    field1010(field world 명세)                    1 host_profile_export -> /runtime/steer/host_profile.json
  2 gz sim -r -s (서버, GUI 없음)               2 gz_esp_emulator
  3 ros_gz_bridge (bridge_steer.yaml,               pty /run/ubts-steer/esp-tty (슬레이브 링크)
    장면 manifest와 대조한 실행 사본)               MCU 펌웨어 + 구동기·가변저항 모델(lockstep v2
  4 cpu_lidar.py (고정 파일): /sim/scan             펌웨어 모드)을 /clock에 맞춰 돌림
  5 자르기 노드: field1010 장면만(기본               3 steer_rover_bridge (pty, SIM 역할,
    steering.scan_unobserved_clip)
  6 [S2 형식 노드 걸이: 다음 단계]                   --clock wall|sim, /run/ubts-steer/control.sock)
  7 sensor_contract_steer: /imu/data_raw,            종료되면 다시 켬(60 s에 5번까지)
    /gps/*, /tf_static, 상태 파일
  8 대시보드 WebSocket 다리 4개                   recorder 컨테이너 (--role recorder)
    (카메라, LiDAR(/scan), IMU, GPS)               기존 data_recorder_service

dashboard 컨테이너: 기존 SIM 대시보드 코드(옛 화면), 127.0.0.1:18090
check 컨테이너(프로필 check, up으로 켜지 않음): 검사 스크립트, sim의 네트·IPC 공유
```

- Gazebo 물리는 120 Hz(스텝 8,333,333 ns)다. `/clock`은 스텝마다 하나 나온다.
- 조향 명령 경로: 클라이언트 → 제어 소켓 → 브리지 래퍼(V3 규약) → pty → 에뮬레이터 안 MCU 펌웨어 → 구동기 모델 → `/sim/act/*` → ros_gz_bridge → Gazebo 관절. 옛 `SimMotor`, 옛 제어 소켓, `/sim/cmd_vel`은 없다.
- 가변저항 값은 공유 모델의 각에서만 나온다. `/sim/truth/joint_states`를 읽지 않는다. 에뮬레이터는 차량 속도와 정지 여부만 참값(`/ground_truth/odom`)에서 읽는다(설계 2.6, plant 쪽).

## 2. 토픽

최종 실행(W0, 실차 시계)에서 잰 ROS 그래프다(`checks/final/w0/a13/ROS_GRAPH.json`). `/parameter_events`, `/rosout`은 뺐다. 토픽 표의 노드 이름은 앞의 `ubts_`를 뺐다.

| 토픽 | 형식 | 발행 | 구독 | 비고 |
|---|---|---|---|---|
| `/camera/camera_info` | CameraInfo | ros_gz_bridge | camera_ws_bridge |  |
| `/camera/image_jpeg` | CompressedImage | 없음 | camera_ws_bridge |  |
| `/camera/image_raw` | Image | ros_gz_bridge | camera_ws_bridge, data_recorder | 640×360, 30 Hz |
| `/clock` | Clock | ros_gz_bridge | ros_gz_bridge, steer_esp_emulator, steer_p6a_check, steer_sensor_contract | Gazebo 물리 스텝마다(120 Hz) |
| `/gps/data` | String | steer_sensor_contract | data_recorder, gps_websocket_bridge | 가상 GPS 대시보드 형식 |
| `/gps/driver_status` | String | steer_sensor_contract | data_recorder, gps_websocket_bridge |  |
| `/gps/fix` | NavSatFix | steer_sensor_contract | 없음 | 가상 GPS, 원점 0 N 0 E, 표시 전용 |
| `/ground_truth/odom` | Odometry | ros_gz_bridge | cpu_lidar, steer_esp_emulator, steer_p6a_check, steer_sensor_contract | 참값: 차체 자세·속도 |
| `/imu/data` | Imu | data_recorder | 없음 | 녹화기의 기존 재발행 |
| `/imu/data_raw` | Imu | steer_sensor_contract | data_recorder, imu_ws_bridge | IMU 오차 모델 적용, frame imu_link, 자세 미제공 |
| `/imu/mag` | MagneticField | 없음 | data_recorder, imu_ws_bridge |  |
| `/rover/control/applied` | String | steer_rover_bridge | 없음 | 2 Hz, MCU 보고 핀 |
| `/rover/control/requested` | String | steer_rover_bridge | 없음 | 2 Hz |
| `/rover/status` | String | steer_rover_bridge | data_recorder, steer_p6a_check | 2 Hz |
| `/rover/steering/state` | String | steer_rover_bridge | steer_p6a_check, steer_sensor_contract | MCU 상태 줄마다(20 Hz) |
| `/scan` | LaserScan | 없음 | data_recorder, lidar_ws_bridge | 발행자 없음(S2 형식 노드: 다음 단계) |
| `/sim/imu` | Imu | ros_gz_bridge | steer_sensor_contract | Gazebo 이상 IMU(센서 계약만 읽음) |
| `/sim/scan` | LaserScan | cpu_lidar | 없음 | Embree 3200빔(참값 자세에서), 실행기 입력 아님 |
| `/sim/truth/joint_states` | JointState | ros_gz_bridge | 없음 | 참값: 관절 |
| `/sim/truth/steering` | String | steer_esp_emulator | steer_p6a_check, steer_sensor_contract | 참값: δL, δR, δeq, 모터 쪽 각, 핀, MCU 상태 |
| `/tf_static` | TFMessage | steer_sensor_contract | 없음 | 정적 TF만 |
| `/sim/act/*` 6개 (left_front_wheel_cmd, left_rear_wheel_cmd, right_front_wheel_cmd, right_rear_wheel_cmd, steer_left_cmd, steer_right_cmd) | Float64 | steer_esp_emulator | ros_gz_bridge | 조향 관절 목표 각, 구동 바퀴 속도 목표(duty × ω0) |

노드별 발행·구독(같은 실행):

| 노드 | 발행 | 구독 |
|---|---|---|
| `ros_gz_bridge` | `/camera/camera_info`, `/camera/image_raw`, `/clock`, `/ground_truth/odom`, `/sim/imu`, `/sim/truth/joint_states` | `/clock`, `/sim/act/*`(6) |
| `ubts_camera_ws_bridge` | 없음 | `/camera/camera_info`, `/camera/image_jpeg`, `/camera/image_raw` |
| `ubts_cpu_lidar` | `/sim/scan` | `/ground_truth/odom` |
| `ubts_data_recorder` | `/imu/data` | `/camera/image_raw`, `/gps/data`, `/gps/driver_status`, `/imu/data_raw`, `/imu/mag`, `/rover/status`, `/scan` |
| `ubts_gps_websocket_bridge` | 없음 | `/gps/data`, `/gps/driver_status` |
| `ubts_imu_ws_bridge` | 없음 | `/imu/data_raw`, `/imu/mag` |
| `ubts_lidar_ws_bridge` | 없음 | `/scan` |
| `ubts_steer_esp_emulator` | `/sim/truth/steering`, `/sim/act/*`(6) | `/clock`, `/ground_truth/odom` |
| `ubts_steer_p6a_check` | 없음 | `/clock`, `/ground_truth/odom`, `/rover/status`, `/rover/steering/state`, `/sim/truth/steering` |
| `ubts_steer_rover_bridge` | `/rover/control/applied`, `/rover/control/requested`, `/rover/status`, `/rover/steering/state` | 없음 |
| `ubts_steer_sensor_contract` | `/gps/data`, `/gps/driver_status`, `/gps/fix`, `/imu/data_raw`, `/tf_static` | `/clock`, `/ground_truth/odom`, `/rover/steering/state`, `/sim/imu`, `/sim/truth/steering` |

- 참값 토픽(`/ground_truth/odom`, `/sim/truth/*`)의 구독자는 허용 목록(LiDAR 광선 프로세스, 센서 계약, 에뮬레이터, 녹화기, 검사 관찰자, ros_gz_bridge)뿐이다(A13, 9.2절).
- `/odom` 발행자와 동적 `/tf` 발행자는 없다. `/tf_static`만 센서 계약이 낸다.
- `/scan`은 구독자(녹화기, LiDAR WebSocket 다리)만 있고 발행자가 없다(5절).
- `/imu/data`는 녹화기가 `/imu/data_raw`를 그대로 다시 내는 기존 동작이다.

## 3. 시계 모드

| 부분 | `--clock wall`(기본) | `--clock sim` |
|---|---|---|
| Gazebo | 실시간 목표로 돈다(`-r`) | 같음 |
| 에뮬레이터 | `/clock`마다 MCU를 `clock + 1스텝`까지 돌린다(두 모드 같음). 1/120 s는 8, 9, 8틱이고 누적 틱 수는 정확하다. 0.1 s를 넘는 틈은 0.1 s 이하 FW_STEP으로 나눈다 | 같음 |
| 브리지 래퍼 | 단조 시계의 고정 20 ms 격자. 한 주기 넘게 늦은 틱은 격자를 다시 맞추고 센다(`skipped`) | 2 ms(벽시계)마다 ROS를 돌리고, `/clock` 표본이 다음 20 ms 격자점에 닿으면 코어를 한 번 돌린다. 여러 격자점을 건너뛰면 한 번만 돌리고 건너뛴 수를 센다. STOP(코어의 비상 표시: op stop, 큐 가득 참)이 오면 격자를 기다리지 않고 다음 폴에서 지금 SIM 시각으로 코어를 한 번 더 돌린다(`urgent_steps`). STOP이 걸려 있는 동안 ROS 돌리기는 `/clock`을 더 받지 않는다 |
| 래퍼 이벤트 기록 | 요청, 전송 쓰기, 상태 변화. STOP이 확인되거나 포기될 때까지 루프 틱마다 `tick`(코어 단계 전 시각) | 같음 |
| 래퍼 이벤트 기록의 시각 | 단조 시계 | SIM 시각 |
| 검사의 시각 | 에뮬레이터 틱(SIM 1 ms), 래퍼 기록, 검사 프로세스 벽시계 | 같음 |

- 고친 것(2026-11-09): 실차 시계 루프가 틱마다 20 ms를 쉬어서 주기가 20 ms보다 길어졌다(A17 STOP 지연 21.7 ms까지). 고정 격자로 바꿨다. 시험 `test_wall_loop_keeps_a_fixed_20ms_grid`.
- SIM 시계 모드에서 브리지는 `/clock` 표본(8.33 ms 간격)에서만 움직일 수 있다. 처음 판은 STOP도 20 ms 격자점을 기다려 요청에서 송신까지가 8.3, 16.7, 25.0 ms로 나왔다(9.5절). 격자 단계는 /clock 표본 간격 때문에 16.67 ms와 25 ms로 번갈아 나오므로 최대 대기가 25 ms다(RG-04에서 120 Hz 표본으로 확인). 고친 판은 STOP을 다음 폴에서 바로 처리한다. STOP 대기는 최대 /clock 한 표본(8.33 ms)이다. C 재전송, STOP 재전송 격자, 상태 주기 같은 주기 일은 그대로 20 ms 격자에 둔다(시험 `test_sim_clock_stop_does_not_wait_for_the_grid`). 실차 시계 모드(실차와 같은 코드 경로)는 바꾸지 않았다.
- 언제 무엇을 쓰나: A10에 따라 실차 시계에서 RTF < 0.9인 구간의 시간 민감 결과는 무효다. 그때는 SIM 시계 모드로 돌린다. 이번 측정에서는 호스트 부하가 높을 때 W0·W1 모두 0.9 아래 창이 있었다(9.0, 9.3절). 그 묶음의 시간 민감 검사는 다시 돌렸다.
- SIM 시계 모드도 에뮬레이터 자체의 멈춤은 막지 못한다. 에뮬레이터가 늦으면 `/clock` 메시지를 건너뛰고(SIM 시각이 한 번에 크게 나아감) 그 사이 MCU 상태 줄이 한꺼번에 나온다. 그러면 SIM 시계 브리지도 상태 줄 공백을 본다(9.0절).

## 4. 신원 분리 (A14)

| 항목 | 조향 SIM(이 스택) | 옛 SIM 스택 | 실차 |
|---|---|---|---|
| ROS 도메인 | 43 | 42 | 0 |
| GZ 파티션 | `ubts-steer` | `ubts-sim` | 없음 |
| 대시보드 | 127.0.0.1:18090 | 18080 | 실차 대시보드 |
| 제어 소켓 | `/run/ubts-steer/control.sock`(볼륨 `ubts-steer_control`) | 옛 소켓 | `/run/ubts-rover/control.sock` |
| ESP 장치 | pty `/run/ubts-steer/esp-tty`(Unix98 pty 슬레이브만) | 없음 | udev by-id 직렬(ID_SERIAL 확인) |
| 정적 TF `base_link -> laser` | z축 회전 = 프로필 `lidar_yaw_offset_deg`(179.08°, S2 형식 `/scan` 관례) | | 실차 대시보드 소스에는 laser 정적 TF 발행자가 없다. 실행기는 자기 설정의 `lidar_yaw_offset_deg`를 쓴다 |
| 고장 걸이 소켓 | `/run/ubts-steer-emu/hooks.sock`(제어 볼륨 밖, 볼륨 `ubts-steer_hooks`) | 없음 | 없음 |
| 배너 | `sim=1`, 장치 ID(seed에서), `hw=front_steer_v0` | | `sim=0` |
| 호스트 프로필 cal_id | null(아무 값) 허용 | | 고정값 필수 |
| `--clock sim` | 허용 | | 거부 |

- 래퍼는 전송 방식으로 역할을 정한다. `serial`이면 실차 래퍼다. `sim=1` 배너를 거부하고, `/run/ubts-steer` 아래 경로를 거부하고, `/run/ubts-steer` 아래 제어 소켓을 거부한다. `pty`·`memory`면 SIM 래퍼다. `sim=0`을 거부하고, 장치가 `/run/ubts-steer` 밖이면 시작하지 않으며, `/run/ubts-rover` 아래 제어 소켓을 거부한다.
- 한 포트 세션에서 `sim=0` 배너를 한 번 받으면 그 세션은 끝까지 거부 상태다(`EspSteerV3`). 포트를 다시 열어야(USB 분리·재연결) 새 세션이 된다. 검사도 이 동작을 확인했다(9.4절).

## 5. 상태 파일과 `/scan`

| 파일 | 쓰는 쪽 | 내용 |
|---|---|---|
| `/runtime/simulation_status.json` | 센서 계약(0.5 s마다) | 고정 `health.py`가 읽는 기존 키(`updated_at`, `sim_time`, `clock_alive`, `sensors_alive`, `counts`) + `schema` `ubts.steer_simulation_status.v1`, `scan_contract`, `control`, `odometry`, `imu_model`, `gps`, `lidar`, `camera_health`, `rtf_5s`, `scene` |
| `/runtime/steering_status.json` | 에뮬레이터(0.5 s마다) | 에뮬레이터 상태(세션, 보정, 카운터, 참값 각, 마지막 배너·상태 줄, 시계 틈 통계) + 브리지 상태 파일 사본 |
| `/runtime/rover_bridge_status.json` | 브리지 래퍼 | 브리지 스냅숏(조향 블록 포함) + `bridge_loop`(시계 모드, 단계 수, 건너뜀) |
| `/runtime/camera_driver_status.json`, `imu_driver_status.json` | 센서 계약 | 대시보드 표시용 |
| `/runtime/steer/supervisor_<역할>.json`, `/runtime/steer/logs/` | 감독기 | 자식 상태, 자식 출력 |
| `/runtime/steer/emulator_log.jsonl` | 에뮬레이터 | 직렬 줄(양방향, 틱), 핀 변화, 걸이, 포트·세션 사건(64 MB에서 돌림) |
| `/runtime/steer/bridge_events.jsonl` | 브리지 래퍼(`--event-log`) | 제어 요청·응답, 전송 쓰기, 래치·상태 변화, STOP이 걸려 있는 동안의 루프 틱(32 MB에서 돌림) |
| `/runtime/steer/a19_record_<사례>.json` | 에뮬레이터(`record` 걸이, `report_adc`) | A19 사례의 FW_STEP 입력·출력(틱마다 analogRead 값). `p6a_fw_rules`가 규칙 틱을 다시 계산한다 |
| `/runtime/scan_clip_status.json` | 자르기 노드(field1010) | 스캔 수, 발행 수, 자세 없는 스캔 수, 잘린 광선 수, 처리 ms |

- `sensors_alive` 키: `sim_scan`(Embree 프로세스 상태 파일), `imu`, `camera`, `camera_info`, `gps`, `steering`(`/rover/steering/state`가 stale이 아님). S2 노드가 켜지면 `scan`이 더해진다.
- `/scan` 처리: 이 단계에서 `/scan`은 없다. `scan_contract`에 "absent: the S2 format node is the next phase"로 적는다. 고정 `health.py`는 `sensors_alive`의 값이 모두 참인지와 `updated_at`만 본다. 그래서 `/scan` 없이도 그대로 통과한다. `/sim/scan`(3200빔, 참값 자세에서 만든 이상 스캔)은 실행기 입력이 아니다.

## 6. compose 사용법

```
python -I <tools>/docker_budget.py acquire --label p6a-stack --gib 6
docker compose -p ubts-steer -f compose.ubts-steer.yaml up -d
docker compose -p ubts-steer -f compose.ubts-steer.yaml run --rm --no-deps check python3 -B -m steering.verify_steer --out /runs/verify
docker compose -p ubts-steer -f compose.ubts-steer.yaml down -t 20    # -v 없이: 볼륨은 남는다(stop_grace_period 20 s)
python -I <tools>/docker_budget.py release --label p6a-stack
```

- `run`에는 `--no-deps`를 쓰고, 환경 변수는 `up` 때와 같은 값을 준다. 다르면 compose가 sim 서비스를 다시 만든다. 그러면 sim의 네트를 공유하는 steer·recorder·check 컨테이너가 ROS 그래프를 잃는다(개발 실행에서 두 번 겪음).
- check 서비스의 `/runs`는 `UBTS_STEER_RUNS_DIR`(저장소 밖 폴더)를 붙인다. 없으면 시작하지 않는다(`create_host_path: false`).
- sim 감독기가 자식을 모두 끄는 데 약 6.5 s가 걸린다. 처음 판은 compose 기본값으로 sim이 SIGKILL되어 137로 끝났고, 그때 READY 표지가 볼륨에 남았다(9.8절, RG-10). 고친 판은 모든 서비스에 `stop_grace_period: 20s`를 두고, compose 머리말의 `down` 예시에도 `-t 20`을 적었다. 남은 표지는 실행 토큰으로 거른다(1절).

| 환경 변수 | 기본 | 뜻 |
|---|---|---|
| `UBTS_STEER_IMAGE` | `ubts-steer-sim:p6a` | 모든 서비스의 이미지 |
| `UBTS_STEER_SCENE` | `w0_force` | `w0_force`, `w1_force`, `field1010` |
| `UBTS_STEER_FIELD_SPEC` | 빈 값 | field1010의 field world 명세(JSON 문자열 또는 파일 경로). 카탈로그 회랑이면 지도와 카탈로그를 sim 서비스에 읽기 전용으로 붙이는 override가 필요하다 |
| `UBTS_STEER_CLOCK` | `wall` | 브리지 시계 모드 `wall` / `sim` |
| `UBTS_STEER_SEED` | 0 | 에뮬레이터·IMU 모델 뽑기 |
| `UBTS_STEER_NO_CAMERA` | 0 | 1이면 장면에서 카메라를 뺀다 |
| `UBTS_STEER_REPORT_ADC` | 0 | 1이면 틱마다 analogRead 값을 기록한다(A3) |
| `UBTS_STEER_LEAD_STEPS` | 1 | MCU를 `/clock`보다 몇 스텝 앞까지 돌릴지 |
| `UBTS_STEER_PORT_OPEN_RESET` | 1 | 호스트가 pty를 열면 MCU 리셋(NodeMCU 자동 리셋) |
| `UBTS_STEER_S2_NODE`, `UBTS_STEER_CLIP_NODE` | 빈 값 | S2 노드 걸이(다음 단계), 자르기 노드 모듈(field1010만, 기본 `steering.scan_unobserved_clip`; 다른 장면에 주면 감독기가 거절) |
| `UBTS_STEER_SOURCE_COMMIT`, `UBTS_STEER_SOURCE_STATUS_SHA256` | `unknown` | 결과 파일의 출처 |
| `UBTS_STEER_RUNS_DIR` | 없음 | check 서비스의 출력 폴더 |

| 서비스 | 메모리 | CPU |
|---|---|---|
| sim | 3.5 GiB(3584m) | 4 |
| dashboard | 1 GiB | 1 |
| steer(에뮬레이터, 래퍼) | 0.5 GiB | 1.5 |
| recorder | 0.5 GiB | 1 |
| 남김(다음 단계 실행기) | 0.5 GiB | |
| check(P6a에서 실행기 자리를 씀, up으로 켜지 않음) | 0.5 GiB | 1 |

- 카메라: `compose.ubts.gpu.yaml`과 같은 WSLg GPU 경로(D3D12, Ogre 1, `/dev/dxg`). LiDAR: Embree CPU 프로세스. 소프트웨어 렌더링 변수는 compose 머리말에 있다.
- 건강 확인: sim 서비스의 healthcheck가 고정 `health.py`를 그대로 돌린다(start_period 240 s).

## 7. 조종 소켓 클라이언트 (`p6a_client.py`)

제어 소켓의 규약은 실차와 같다(한 연결에 JSON 한 줄, 응답 한 줄, 4096 B 이하).

| 호출 | 요청 | 뜻 |
|---|---|---|
| `ControlClient(owner)` | | 소켓 `ROVER_CONTROL_SOCKET`(기본 `/run/ubts-steer/control.sock`). lease, ticket, seq를 들고 있다 |
| `status()` | `op status` | 브리지 스냅숏 |
| `arm(vehicle_profile='front_steer')` | `op arm` | lease를 받는다. `front_steer`가 아니면 `steer_profile_required` |
| `drive(throttle, steer_mrad, deadman=True)` | `op drive` | 입력 기한 0.75 s. 좌우 값(`left`/`right`)은 `differential_command_rejected` |
| `keepalive()`, `release()` | `op keepalive` / `op release` | release: throttle 0(MCU 램프), 조향 목표 유지 |
| `motion(yaw_rate_rad_s, speed_m_s)` | `op motion` | 실행기의 센서 운동 표본(`steering_ineffective`용). 운동 입력이 아니다. 이 op은 WF-3a가 고치는 중인 코어(작업 트리, 커밋 전)에 있고, 이미지의 HEAD 판 코어에는 없다(미시험) |
| `stop(mode)` | `brake`: `op stop`(lease 불필요), `release`: `op release`, `expire`: 요청 없음(입력 기한 → C TTL) | 정지 방식 |
| `DriveLoop(client, throttle, steer)` | | 0.1 s마다 drive를 다시 보내는 스레드. `set()`, `stop()` |
| `HookClient()` | 에뮬레이터 걸이 | `status`, `mark`, `event`, `adc_pattern`, `adc_ramp`, `link`, `status_loss`, `corrupt`, `half_frame`, `inject_line`, `banner_rewrite`, `restart`, `unplug`, `record` |
| `SimObserver()` | | 평가 쪽 노드(`ubts_steer_p6a_check`). 참값을 읽으므로 평가 소비자다. 제어에 쓰지 않는다 |

## 8. 다음 단계 연결점

- S2 형식 노드: `--s2-node MODULE`(또는 `UBTS_STEER_S2_NODE`). 감독기가 `python3 -B -m MODULE --input <토픽> --output /scan`으로 켠다. 입력은 `/sim/scan`(자르기 노드가 있으면 `/sim/scan_clipped`). 켜지면 센서 계약이 `/scan`을 구독하고 `sensors_alive`에 `scan`을 더한다. LiDAR WebSocket 다리와 녹화기는 이미 `/scan`을 듣는다. `verify_steer`의 `scan_absent_until_s2_node`는 이때 바꾼다.
- 자르기 노드(RW-01, 고침): field1010 장면에서 감독기가 켠다(기본 `steering.scan_unobserved_clip`). 같은 stamp의 자세가 아직 오지 않은 스캔은 자세를 기다린다(최대 8개, 15.4절). 명령은 `--input /sim/scan --output /sim/scan_clipped --raster <장면>/field_world/raster.npz --status-file /runtime/scan_clip_status.json --noise-sd SD --noise-seed N`이다(`supervisor_steer.clip_node_command`, 노드 인자 해석과 맞추는 시험 있음). 처음 판은 감독기가 `--raster` 없이 켜서 노드가 인자 오류(종료 2)로 끝나고 sim 역할 전체가 멈췄다. 자르기 노드가 있으면 cpu_lidar는 잡음 0인 rover.yaml 사본으로 돌고, 자르기 노드가 자른 뒤 rover.yaml 잡음을 더한다(F-SIM 순서, RW-04, `WORLDS_KO.md` 5절).
- 실행기 컨테이너: compose 서비스로 더한다. 메모리 0.5 GiB 자리, `network_mode`·`ipc`는 `service:sim`, 제어 볼륨을 `/run/ubts-steer`에 붙이고 `ROVER_CONTROL_SOCKET`을 쓴다. 노드 이름은 `ubts_row_executor`로 둔다. 그러면 `p6a_graph`의 R6(구독 허용 목록 `/scan`, `/imu/data_raw`, `/rover/control/requested`)이 그 노드를 검사한다. 지금은 노드가 없어 R6은 "없음"으로 보고된다.
- 실행기가 있어야 하는 A17 항목: 실행기를 고리에 넣은 STOP·deadman·TTL, 실행기의 `SteerOutputGuard`와 `motion` op(`steering_ineffective`), 열 전체 주행.

## 9. 검사 결과

최종 실행: 이미지 `ubts-steer-sim:p6a`, 2026-11-09. 출력은 저장소 밖 `<runs>/wf3b/p6a/checks/final/`(검사 JSON), `logs/`(콘솔, docker 상태). 모든 결과 파일의 `provenance.sources_match_image`가 참이다(검사 프로세스가 읽은 `sim/ubts`, `dashboard/scripts`, `firmware`의 모든 파일이 이미지 manifest의 SHA와 같음).

### 9.0 실행 조건과 부하

| 묶음 | 장면, 시계 | 검사 | 호스트 CPU 부하 |
|---|---|---|---|
| `w0` | W0, 실차 시계 | verify, A13, A14, A17, A19, A10(120 s) | 높음(다른 작업 흐름의 Windows Python 프로세스 6개가 각 약 1코어, 전체 82~100 %) |
| `a3` | W0, 실차 시계, ADC 기록 | A3 | 높음 |
| `sim` | W0, SIM 시계 | verify, A17, A13 | 높음 |
| `drive_w0_1..5`, `drive_w1_1..5` | W0·W1, 실차 시계 | A9, A10(W1 180 s) | 높음에서 낮음으로 바뀜 |
| `w0b` | W0, 실차 시계 | verify, A17, A19, A10(60 s), 사건 기록 사본 | 74~100 % |
| `simb` | W0, SIM 시계 | A19, A17 | 3~19 % |
| `w0c` | W0, 실차 시계 | A17 | 10 % |
| `bridge_kill` | W0, 실차 시계 | 필수 장애 시험 "브리지 강제 종료" | 낮음 |

- 메모리 예산(6 GiB)은 다른 작업 흐름과 나눠 쓴다. CPU는 나누지 않는다. 처음 묶음들은 다른 작업 흐름이 무거운 계산을 돌리던 때였다. 그때 에뮬레이터의 `/clock` 처리 사이 벽시계 틈이 100 ms를 넘은 것이 37번(최대 8.7 s), W0 A10 창의 최소 RTF가 0.18이었다.
- 에뮬레이터가 100 ms 넘게 멈추면 브리지는 상태 줄을 150 ms 못 받아 `steering_status_stale`를 래치하고 조종을 멈춘다(설계대로의 안전 동작). 이때 진행 중이던 대본 단계가 끊긴다. `w0b`의 사건 기록에서 A17 첫 STOP 시험 직전에 이 래치가 걸린 것을 확인했다(`checks/final/w0b/eventlogs/w0b_safety/`).
- A10 규칙대로 RTF < 0.9 구간(그리고 같은 원인의 멈춤 구간)의 실차 시계 시간 민감 결과는 무효로 보고 다시 돌렸다. 아래 표는 각 검사의 처음 결과와 다시 돌린 결과를 함께 적는다. 처음 결과를 지우지 않는다.

### 9.1 통합 확인 (`verify_steer`)

| 항목 | `w0`(부하 높음) | `w0b` | `sim`(SIM 시계) |
|---|---|---|---|
| 건강(모든 `sensors_alive`) | 통과 | 통과 | 통과 |
| `/imu/data_raw` 율(프로필 33 Hz ±10 %) | 32.58 Hz | 32.35 Hz | 32.34 Hz |
| frame, 자세 미제공(covariance[0] = −1) | 통과 | 통과 | 통과 |
| `/sim/scan` 빔 수 | 3200 | 3200 | 3200 |
| 가상 GPS(0 N 0 E 근처) | 통과 | 통과 | 통과 |
| `/tf`·`/odom` 발행자 없음, `/tf_static`만 | 통과 | 통과 | 통과 |
| 왼쪽 원 +175 mrad: R, R/R_geom(참 δ), 참 δ | 5.091 m, 1.022, 8.68° | 5.083 m, 1.021, 8.68° | 5.093 m, 1.023, 8.68° |
| 오른쪽 원 −175 mrad | **실패**: 조종이 끊겨 차가 서 있었음(요레이트 0) | 5.213 m, 1.022, −8.47° | 5.218 m, 1.022, −8.47° |
| 요레이트 부호 | 왼쪽만 맞음 | 맞음 | 맞음 |
| STOP: 호스트 확인, 정지, 3 s 정지 유지, 자동 재출발 없음 | 통과(정지 거리 0.055 m) | 통과(0.055 m) | 통과(0.061 m) |
| deadman(입력 기한 0.75 s) | `input_expired` | `input_expired` | `input_expired` |
| 판정 | 실패(오른쪽 원) | **통과** | **통과** |

- 명령 10.03°에 참 δ가 8.68°인 것은 자리값 데드밴드·백래시·0점 오차·보정 잔차 뽑기 때문이다. R/R_geom(참 δ) 1.02는 A1(G) 힘 모드 값(10°에서 약 1.03)과 같은 쪽이다.
- `w0`의 오른쪽 원: 대본 반복 요청이 `control_not_owned` 1번, `client_has_no_lease` 255번이었다. 부하 높을 때의 상태 줄 멈춤 래치와 같은 모양이다(이 묶음의 사건 기록은 남기지 못했다. 다음 묶음이 기록 파일을 새로 열었다).

### 9.2 A13 그래프 (참값 격리)

- `w0`·`sim` 모두 통과. 노드 11개(검사 관찰자 포함). 규칙 R1~R5 위반 0: 참값 토픽 구독자는 허용 목록뿐, `/odom` 없음, 동적 `/tf` 없음, `/sim/act/*` 발행자는 에뮬레이터뿐, `/sim/scan` 구독자는 허용 목록뿐.
- R6(실행기 구독 허용 목록 `/scan`, `/imu/data_raw`, `/rover/control/requested`): 실행기 노드 `ubts_row_executor`가 없어 "없음"으로 보고했다. 합성 그래프 시험은 통과(`test_steering_p6a.py`).
- `/scan` 발행자 0(보고).
- 토픽·노드 표는 2절.

### 9.3 A9 반복 분산, A10 실시간 비율

A9: 같은 장면 5번, 실행마다 스택을 새로 켬(같은 seed). 대본: W0 throttle 40, 직진 15 s 뒤 +175 mrad 10 s, STOP. W1 throttle 60, 직진 20 s 뒤 +50 mrad 8 s, STOP. 위치는 출발 자세 기준(SIM 지역 좌표).

| | 최종 x [m] | 최종 y [m] | 최종 방향 [°] | 직진 끝 옆 벗어남 [m] | 실행별 RTF 중앙값 |
|---|---|---|---|---|---|
| W0 1~4 | 7.833~7.839 (sd 0.0025) | 1.139~1.150 (sd 0.0046) | 34.95~35.13 (sd 0.075) | 0.0037~0.0052 | 0.978~1.000 |
| W0 5 | 7.300 | 0.765 | 27.71 | 0.0035 | 0.956 |
| W0 5번 모두 | 평균 7.729, sd 0.240 | 평균 1.069, sd 0.170 | 평균 33.57, sd 3.28 | 평균 0.0042, sd 0.0007 | |
| W1 5번 | 평균 12.771, sd 0.020, 범위 0.050 | 평균 0.197, sd 0.010 | 평균 7.80, sd 0.17 | 평균 −0.053, sd 0.004 | 0.941~1.000 |

- W0 5번째 실행은 회전 중 조종이 끊겨(`control_not_owned` 1, `client_has_no_lease` 19) 약 2 s 일찍 섰다. 부하 높을 때의 상태 줄 멈춤 래치와 같은 모양이다. 그 실행만 빼면 W0 분산은 W1보다 작다.
- A10 W1(카메라 640×360, GUI 없음, 정지 180 s): RTF 중앙값 0.965, 하위 10 % 0.913, 최소 창 0.847, 전체 0.965. 토픽 율(SIM 초당): 카메라 30.27, IMU 33.34, GPS 4.80, 조향 상태 20.00. W1 주행 중(28 s 대본) 창 최소 0.903~1.000.
- A10 W0(전체 스택): `w0` 120 s 중앙값 0.988, 하위 10 % 0.936, 최소 창 0.181(멈춤), 전체 0.908. `w0b` 60 s 중앙값 1.000, 최소 0.958, 전체 0.997.
- 판정: A9는 보고. A10 W1은 보고(옛 W1 기록 0.68배속보다 높음). W0 중앙값 ≥ 0.9는 두 번 모두 넘었지만, 부하가 높을 때 최소 창이 0.9 아래로 내려간다.

### 9.4 A14 신원 분리 (`w0`, 통과)

| 항목 | 결과 |
|---|---|
| SIM 래퍼가 `sim=0` 장치를 거부(배너 고침 걸이, MCU 리셋) | `sim_identity_mismatch`, arm 거부 |
| 같은 포트 세션에서 `sim=1` 배너가 다시 와도 거부 유지 | 거부. USB 분리·재연결 뒤 새 세션 |
| 하드웨어 프로필이 다르면 arm 거부(`hw=front_steer_x9`) | `hardware_profile_mismatch` |
| SIM 래퍼 프로세스가 `/run/ubts-steer` 밖 장치를 거부 | 종료 코드 2, "SIM pty device must be under /run/ubts-steer" |
| 실차 래퍼 프로세스(`serial`, cal_id 고정 프로필)를 SIM pty에 연결 | 시작은 하지만 포트를 열지 않음: `serial_open` false, "SIM device path refused by the real-hardware transport" |
| 실차 전송 계층이 pty를 거부(검사 컨테이너, steer 컨테이너) | 링크 경로: 경로 규칙으로 거부. pty 슬레이브 `/dev/pts/0`: udev 자료 없음으로 거부(steer 컨테이너, 포트를 열지 않음) |
| 실차 역할 링크가 `sim=1` 배너를 거부 | `sim_identity_mismatch` |
| 옛 조종 화면(대시보드 HTTP, 옛 요청)으로 arm | `steer_profile_required` |
| 옛 화면의 좌우 drive, 소켓의 좌우 drive | `differential_command_rejected`, 구동 핀 변화 0 |
| ROS 도메인 | 43에서 자기 토픽이 보임(양성 대조), 0과 42에서는 0개 |

### 9.5 A17 안전 동작

| 항목(기준) | `w0` | `w0b` | `w0c` | `sim`, `simb`(SIM 시계) |
|---|---|---|---|---|
| STOP 요청 → STOP 바이트(≤ 20 ms) | 4.0~17.1 ms | 2.3~18.1 ms | 4.4~15.9 ms | 8.3~25.0 ms, 16.7~25.0 ms (**25.0 ms가 기준 넘음**) |
| MCU STOP → 구동 0(≤ 1 서보 틱) | 0틱 ×5 | 0틱 ×4, 첫 시험은 차가 서 있었음 | 0틱 ×5 | 0틱(`sim` 첫 시험은 차가 서 있었음) |
| STOP 확인 상태 줄(≤ 2주기 100 ms) | 2~35 ms | 2~44 ms | 2~36 ms | 2~44 ms |
| 유지 정지의 조향 유지 오차(≤ max(1°, 2·데드밴드)) | ≤ 0.72° | ≤ 0.58° | ≤ 0.57° | ≤ 0.63° |
| 확인 실패: 재전송 ≤ 5, 50 ms 격자 뒤 첫 루프 틱, 래치 | 5번, 100/160/200/260/300 ms, 래치 400.3 ms | 5번, 120/160/219/259/320 ms, 래치 419.8 ms | 5번, 120/160/220/260/320 ms, 래치 400.2 ms | 5번, 100/158/200/250~258/300~317 ms, 래치 400.0 ms |
| C TTL 만료 → 구동 0(≤ ttl + 1틱) | +0틱 | +0틱 | +0틱 | +0틱 |
| 링크 watchdog → 구동 0, DISARMED(≤ wd + 1틱) | +0틱 | +0틱 | +0틱 | +0틱 |
| 끊김 동안 옆 벗어남(호 계산 ±10 % 또는 0.01 m) | 0.0013 m(호 0.0013) | 0.0022 m(호 0.0023) | 0.0012 m(호 0.0013) | 0.0026, 0.0020 m(호 0.0026, 0.0021) |
| 결함 정지(POT_OPEN) → 조향 출력 0(≤ 1틱), 구동 브레이크 | 조향 0틱, **구동: 차가 이미 서 있었음** | 0틱, 0틱 | 0틱, 0틱 | 0틱, 0틱 |
| 자동 재출발 | 0 | 0 | 0 | 0 |
| 판정 | 실패(결함 정지 사례) | 실패(첫 STOP 시험) | **통과** | 실패(STOP 지연 양자화, `sim` 첫 시험) |

- 실차 시계 판정: 부하가 낮은 `w0c`에서 모든 기준 통과. `w0`·`w0b`의 실패는 둘 다 상태 줄 멈춤 래치로 시험 직전에 조종이 끊긴 경우다(9.0절). 판정 기준 자체를 넘은 값은 없었다.
- SIM 시계 판정: STOP 지연이 `/clock` 표본(8.33 ms)에 묶여 8.3, 16.7, 25.0 ms로만 나온다. 최악 25.0 ms가 20 ms 기준을 넘는다(10절 7, 제안). 나머지 기준은 `simb`에서 모두 통과.
- 필수 장애 시험(실행기 없이 되는 것, `w0c` 보고): 두 클라이언트(두 번째의 arm `control_in_use`, drive `control_not_owned`, STOP은 누구나 됨), STOP 직후 늦은 drive(`stop_barrier`), 반쪽 프레임(MCU `N 1 nocrc`, 호스트 `c_rejected`), CRC 오류 한 줄(MCU `N - crc`, 계속 무장), 범위 오류(MCU `N … range`, `c_rejected`), 상태 줄 120 ms 유실(무장 유지), 400 ms 유실(`steering_status_stale`), 주행 중 MCU 리셋(BOOT, `mcu_reset_detected`, 재출발 없음), USB 분리·재연결(pty 다시 열기, `hardware_fault`, 재출발 없음).
- 브리지 강제 종료(`bridge_kill`, 통과): 주행 중(throttle 30) 래퍼를 SIGKILL했다. 마지막 C 도착 + 300 ms(TTL)에 구동 0(+0틱), 마지막 입력 + 500 ms에 watchdog `W`(+1틱), 감독기가 1 s 뒤 래퍼를 다시 켬, 포트를 다시 열자 MCU 리셋(`rst=ext_sys`), 자동 재출발 0, 호스트는 무장 상태가 아님(`fault`: BOOT, `steering_status_stale`).
- 실행기가 있어야 하는 항목(다음 단계): 실행기를 고리에 넣은 STOP·deadman·TTL, 실행기의 `SteerOutputGuard`와 `motion` op(`steering_ineffective`), 열 전체 주행. 시스템 시각 변경, 누른 채 통신 끊김은 미시험(11절).

### 9.6 A19 결함 코드 (`w0b` 실차 시계, `simb` SIM 시계: 모두 통과)

판정 사례마다 걸이로 결함을 만들고, 판정 틱(MCU `F` 줄의 틱) − 기대 틱과 결과 상태를 본다.

| 사례 | 걸이 | 판정 − 기대(허용) `w0b` / `simb` | 결과 |
|---|---|---|---|
| POT_OPEN | `adc_set 0`, 주행 중 | 0(±1) / 0 | DISARMED, 호스트 래치. 먼저 POT_NOISE가 같은 틱에 걸림(10절 1) |
| POT_SHORT | `adc_set 1023` | 0(±1) / 0 | 같음 |
| POT_JUMP | 현재 raw + 4·jmp | 0(±1) / 0 | 같음, POT_NOISE 한 틱 먼저 |
| POT_NOISE | ±(jmp − 2) 교대 무늬, DISARMED | +9(±25, nsw 창 안) / +9 | 래치 |
| SOFTLIMIT | 느린 ADC 램프, DISARMED | 상태 비트 0x0080 | 상태 변화 없음(조건 비트만) |
| POLARITY | v1 주입 `polarity_reversed`, 목표 250 | −289 / −369(첫 밀기 + polt 이후 1.5 s 창 안) | DISARMED |
| DRV_STUCK | ADC 램프 9틱에 1카운트(적합성 대본과 같은 증상) | 시각 기준 없음(보고): 걸이 뒤 약 60틱 | DISARMED |
| STALL | 모터 전원 끔 + ADC 고정 + 계단 350 | 0(±2) / 0 | DISARMED |
| TRACK | maxp 150 + 같은 계단 | 0(±5) / 0 | DISARMED |
| THERMAL | CFG tha 50, thw 1, 계단 380 | 0(±2) / 0 | 조향 끔 |
| TTL | 링크 끊김(to_mcu 400 ms) | 0(±1) / 0 | 구동 0, 무장 유지 |
| WATCHDOG | 링크 끊김 900 ms | 0(±1) / 0 | `W`, DISARMED |
| CRC | 다음 5줄 CRC 오류 | 0(±1) / 0 | DISARMED |
| BOOT | `mcu_reset` | 배너, 조건 비트 0x4000 | 호스트 `mcu_reset_detected` |
| NOCAL | 빈 EEPROM 세션 | 조건 비트 0x0001 | arm 거부 `steering_uncalibrated` |
| 호스트 `steering_status_stale` | 상태 줄 400 ms 유실 | | 래치 |
| 호스트 `calibration_mismatch` | cal_id 2 세션 | | arm 거부 |
| 호스트 `c_rejected` | 다음 C를 범위 오류로 | | 래치 |
| 호스트 `steering_not_following` | 느린 조향(`rate_scale` 0.3) | | 래치 |

- 보고 사례(판정 없음): `drv_stuck_model`(모델의 구동기 고착 → POLARITY가 먼저), `stall_model_noise`(모델 ADC 잡음으로 STALL), `track_model_noise`(TRACK 없이 호스트 `steering_not_following`), `pot_noise_armed`(무장 서보 → DRV_STUCK이 먼저).
- `w0`(부하 높음)의 첫 결과: MCU 결함 사례는 모두 통과했다. 마지막 호스트 사례 4개(NOCAL, 보정 불일치, `c_rejected`, `steering_not_following`)가 멈춤 때문에 arm이 `control_busy_stopped`로 끝나거나 watchdog이 먼저 걸려 실패했다. `w0b`·`simb`에서 모두 통과했다.
- 실행기가 있어야 하는 것: `steering_ineffective`(실행기의 `motion` op). 신원·하드웨어 불일치는 A14, `stop_unconfirmed`는 A17에서 봤다.

### 9.7 A3 (G 가변저항 raw = F 모델 raw, 통과)

- 실차 시계 + ADC 기록(`UBTS_STEER_REPORT_ADC=1`). 20 SIM 초: 정지 → 주행 중 조향 ±200 mrad 세 번 → v1 주입 `pot_jump`(40카운트, 0.3 s).
- 기록한 FW_STEP 입력을 새 펌웨어 백엔드(F-SIM·T1 경로)에 다시 넣었다: 프레임 2,380개, 틱 20,000, 응답 불일치 0, analogRead 값 비교 14,516개, 같은 네이티브 빌드(`fw=04353B49`).
- 정지 raw sd 1.13카운트(보고, 프로필 `pot_noise_sd_counts` 1.0). 무장 상태의 서보 보정이 섞인 표본이라 순수 잡음 측정이 아니다.

### 9.8 건강 확인, 시작·종료, 두 시계 모드

- sim 서비스 healthcheck(고정 `health.py`)는 매 실행에서 통과했다. 예산을 바로 잡은 실행에서 `up`부터 healthy까지 26~40 s였다(W1이 더 김).
- 종료: `stop -t 20`(`w0b`, `simb`, `w0c`)에서 모든 감독기가 0으로 끝났다. sim 감독기는 약 6.5 s 걸렸다: `gps_ws_bridge`, `lidar_ws_bridge`가 SIGTERM에 끝나지 않아 6 s 뒤 SIGKILL, `gz`는 종료 때 SIGSEGV(−11), `cpu_lidar` 1, `ros_gz_bridge` −15. 이 compose(v5.6.1)의 기본 `stop`은 sim에 SIGTERM 뒤 약 3 s에 SIGKILL을 보내 sim이 137로 끝났다. 그래서 `down -t 20`을 쓴다(6절, 제안). 대시보드는 SIGTERM에 143으로 끝난다(기존 코드).
- 남은 것: 매 `down` 뒤 이 프로젝트의 컨테이너 0개, 네트워크 0개. 볼륨 4개(`ubts-steer_runtime`, `_control`, `_cache`, `_hooks`)는 설계대로 남는다.
- 두 시계 모드: 실차 시계(`w0`, `w0b`, `w0c`, 주행)와 SIM 시계(`sim`, `simb`)를 모두 돌렸다. SIM 시계에서 브리지는 SIM 20 ms 격자로 돌았다(예: `simb` 10,922단계, 건너뜀 217).

### 9.9 리뷰 수정 뒤 다시 잰 결과 (fix_wf3b)

- 2026-11-09. 결과 파일은 저장소 밖 `<runs>/wf3b/fix_wf3b/checks/<묶음>/`(검사 JSON, 검사마다 에뮬레이터·브리지 사건 기록과 상태 파일 사본 `eventlogs/`), 콘솔과 묶음 기록은 `logs/`(`phases.txt`)에 있다.
- 이미지는 `ubts-steer-sim:p6a` 그대로다(새 이미지 ID 없음). 고친 소스의 스냅숏(HEAD + WF-3b 파일의 작업 트리 판, 저장소 밖 `make_snapshot_fix.py`)을 모든 서비스의 `/opt/ubts/sim/ubts`와 `/opt/ubts/dashboard/scripts` 위에 읽기 전용으로 붙였다. 그래서 결과의 `provenance.sources_match_image`는 거짓이고, 이미지와 다른 파일 목록이 결과에 있다. 스냅숏 기록은 `logs/SNAPSHOT_v1.json`~`_v5.json`이다.
- 스냅숏: v1(묶음 A), v2(B, C, D, A2: 15.1절), v3(E, F, G: 15.4절의 고침), v4(H, I: a25 계단 크기), v5(마지막 소스: 자르기 노드 보간, 시험 컨테이너만). 등록 기준은 바꾸지 않았다.
- 실차 시계 묶음의 시간 민감 단계는 A10 무효 규칙으로 거른다: 단계를 덮는 5 s 창 RTF < 0.9, 단계 안 에뮬레이터 `/clock` 벽시계 틈 > 100 ms, 단계 안 SIM 쪽 원인의 호스트 래치(v3부터, 단계 창은 ARM 앞부터) 가운데 하나라도 있으면 그 단계는 판정하지 않는다(무효, 다시 돌림).
- 호스트 부하: 다른 작업 흐름이 같은 PC를 쓴다. 묶음마다 검사 시작 때 호스트 CPU를 `phases.txt`에 적었다. E 이후는 83~99 %였다. C의 처음 약 60 s는 이 작업의 Windows 시험(7분)과 겹쳤다.

| 묶음 | 장면, 시계, 스냅숏 | 검사 | 검사 시작 때 호스트 CPU |
|---|---|---|---|
| A | W0, 실차, v1 | startup, verify, A13, A14, a25, A17, A19, drive | 0~60 % |
| B | W0, SIM, v2 | startup, A17, A19, a25 | 28~76 % |
| C | W1, 실차, v2 | startup, A10 240 s, drive | 43~65 % |
| D | W2 field1010, 실차, v2 | startup, A13, A10 240 s | 22~60 % |
| A2 | W0, 실차, v2 | A와 같음 | 0~85 % |
| E | W0, 실차, v3 | startup, a25, A17 | 3~93 % |
| F | W0, SIM, v3 | startup, a25 | 97~99 % |
| G | W2 field1010, 실차, v3 | startup, A10 90 s(자르기 노드) | 99 % |
| H | W0, 실차, v4 | startup, a25, A17 | 90~99 % |
| I | W0, SIM, v4 | startup, a25 | 56~82 % |

#### 9.9.1 A17 안전 동작 (RG-04, RG-05, RG-06)

| 항목(기준) | B(SIM 시계, v2) | A2(실차, v2) | E(실차, v3) | H(실차, v4) |
|---|---|---|---|---|
| STOP 요청 → STOP 바이트(≤ 20 ms) | 0.0 ms ×5(SIM 시각, STOP이 걸리면 격자를 기다리지 않음) | 4.3~17.7 ms | 4.6~9.3 ms(유효 4번) | 5.0~12.6 ms(유효 4번) |
| MCU STOP → 구동 0(≤ 1 서보 틱) | 0틱 ×5 | 0틱 ×4, STOP 2는 차가 서 있었음 | 0틱 ×4 | 0틱 ×4 |
| STOP 바이트 → 호스트 확인(≤ 100 ms) | 33~67 ms | 20~41 ms | 20~61 ms | 41~96 ms |
| 유지 정지의 조향 유지 오차(≤ 1°) | ≤ 0.52° | ≤ 0.75° | ≤ 0.53° | ≤ 0.55° |
| 확인 실패: 재전송 5번, 격자 뒤 첫 루프 틱(기록으로 확인), 래치 | 116.7/158.3/216.7/258.3/316.7 ms, 래치 416.7 ms | 100.2/160.4/220.0/260.0/320.2 ms, 래치 420.2 ms | 100.2/160.0/200.1/260.0/300.2 ms, 래치 400.3 ms | 100.2/160.2/200.3/260.0/300.2 ms, 래치 400.5 ms |
| C TTL → 구동 0(≤ ttl + 1틱) | +0틱 | +0틱 | +0틱 | +0틱 |
| watchdog: 마지막 입력 + wd + 1 이하 | 501(= wd + 1) | 501 | 501 | 501 |
| 끊김 옆 벗어남 vs 해석 호(±10 % 또는 0.01 m) | 0.0012 vs 0.0016 m | 0.0011 vs 0.0014 m | 0.0024 vs 0.0017 m | 0.0012 vs 0.0015 m |
| 정지 거리 SIM vs 해석(보고) | 0.167 vs 0.192 m | 0.161 vs 0.182 m | 0.238 vs 0.199 m | 0.174 vs 0.186 m |
| 결함 정지 → 조향 0, 구동 브레이크(≤ 1틱) | 0틱, 0틱 | 0틱, 0틱 | 0틱, 0틱 | 0틱, 0틱 |
| 판정 | **통과** | 실패(STOP 2) | STOP 5번 항목은 판정 안 함(STOP 1 무효), 나머지 통과 | STOP 5번 항목은 판정 안 함(STOP 1 무효: 5 s RTF 0.882), 나머지 통과 |

- B: SIM 시계에서 STOP 지연이 0이 됐다. 처음 판의 25.0 ms(9.5절, 10절 7)는 브리지가 STOP을 다음 20 ms 격자점까지 기다렸기 때문이다.
- A2 STOP 2: STOP 1.0 s 전 주행 중 에뮬레이터 멈춤(123 ms) 뒤 `steering_status_stale`이 걸려 차가 섰다. v2는 STOP 단계의 창을 STOP 순간부터 잡아 이 단계를 유효로 보고 판정했다(15.4절). v3 규칙이면 무효다.
- E STOP 1: 주행 중 에뮬레이터 멈춤(340 ms)과 SIM 쪽 래치로 무효(v3 규칙대로 판정 안 함). 다른 STOP 4번은 모든 기준 안이다. 실차 시계에서 5번 모두 유효한 묶음은 아직 없다.
- 끊김 해석 호: 끊김 때 속도 × C TTL 남은 시간 + 정지 모드 거리(`drive.stop_modes`, TTL 만료 모드 brake). G-SIM의 정지 거리는 해석 모델과 −0.04~+0.04 m 다르다(보고). 옆 벗어남 판정은 0.01 m 바닥으로 통과했다.
- 필수 장애 시험(보고): B·A2·E에서 두 클라이언트, STOP 직후 늦은 drive `stop_barrier`, CRC 한 줄, 범위 오류, 상태 줄 400 ms 유실, 주행 중 MCU 리셋, USB 분리·재연결은 9.5절과 같은 반응이었고 자동 재출발은 0이었다. 두 항목은 묶음마다 달랐다(모두 거절이나 래치로 끝나는 안전한 반응, 판정 항목 아님). 반쪽 프레임은 B에서 MCU `N 1 nocrc`와 호스트 `c_rejected`, A2·E에서 다음 줄과 붙어 MCU `N - crc`(무장 유지)였다. 상태 줄 120 ms 유실은 B(SIM 시계)에서 무장 유지, A2·E(실차 시계)에서 `steering_status_stale`였다. 유실 창 앞뒤의 상태 줄 간격이 150 ms 문턱을 넘을 수 있기 때문으로 본다. 처음 문장("9.5절과 같은 반응")은 이 두 항목에서 틀렸다(2026-11-09 qualify_wf3b 대조, 같은 날 최종 자격 묶음 A·B도 같은 갈래).

#### 9.9.2 A19 결함 코드: 판정 시각 (RG-02)

| 묶음 | 판정 − 규칙 틱 | 무효 사례 | TRACK(추정, 보고) |
|---|---|---|---|
| A(실차, v1) | POT_OPEN, POT_SHORT, POT_JUMP, POT_NOISE, POLARITY, DRV_STUCK, STALL, THERMAL, TTL, WATCHDOG, CRC 모두 0 | 없음 | +1 |
| B(SIM, v2) | 모두 0 | 없음 | 0 |
| A2(실차, v2) | 모두 0 | POT_JUMP(사례 안 에뮬레이터 멈춤, 값은 0) | 0 |

- SOFTLIMIT: 규칙이 조건을 낸 틱 뒤 첫 상태 줄에 조건 비트가 있었다(A 3472 → 3520, B 3474 → 3520, A2 3586 → 3620). 그보다 이른 줄에 비트는 없었다.
- 허용은 ±1틱이다(처음 판의 +9, −289, −369틱은 규칙을 다시 계산하지 않고 넓은 창을 허용했기 때문이다). 23개 사례 모두 결과 상태(DISARMED, 래치 등)가 맞았다. 보고 사례 4개는 9.6절과 같다.

#### 9.9.3 A25 에뮬레이터 경로 (RG-03)

| 묶음 | 같은 SIM 시각 관절 − 공유 모델 p99 / 최대 | 지연 1·2·3스텝 p99 | 가장 큰 \|δ_eq\| | 관절 속도 최대(한계) | `/clock` 빠짐 | 판정 |
|---|---|---|---|---|---|---|
| B(SIM, v2) | 1.1e-15° | 같음 | 움직임 없음 | 3e-16 rad/s | 0 | 판정 무의미(`stale_input`, 15.4절) |
| A2(실차, v2) | 5.3e-14° / 7.5e-14° | 지연과 무관(같음) | (v2: 기록 없음) | 0.24(0.785) rad/s | 20 | 판정 무의미: SIM 쪽 래치(340 ms 멈춤) 뒤 조향이 거의 서 있었음 |
| E(실차, v3) | 4.2e-11° / 7.4e-11° | 0.104°, 0.205°, 0.308° | 11.48° | 0.24 rad/s | 0 | 실패: 호스트 `steering_not_following` 뒤 drive 거부 230번 |
| F(SIM, v3) | 4.2e-11° / 2.2e-10° | 0.100°, 0.201°, 0.297° | 10.93° | 0.42 rad/s | 424 / 3,325 | 실패: 같은 래치 |
| H(실차, v4) | 1.41° / 2.58° | 1.24°, 1.23°, 1.18° | 7.51° | 0.71 rad/s | 480 / 4,110 | 실패: p99, 호스트 `steering_not_following`(requested −85, measured −32 mrad) 뒤 drive 거부 28번. 호스트 CPU 99 % |
| I(SIM, v4) | 2.4e-11° / 0.10° | 0.032°, 0.083°, 0.125° | 5.62° | 0.19 rad/s | 480 / 4,057 | 실패: 호스트 `steering_status_stale`("no status line for 158 ms") 뒤 drive 거부 303번, MCU `F TTL` |

- 에뮬레이터가 따라갈 때(E, F, I) 관절 각은 같은 SIM 시각의 공유 모델 각과 1e-10° 안에서 같다(에뮬레이터가 `/clock` + 1스텝 앞의 모델을 내고 Gazebo가 다음 스텝에 맞춘다). 지연 1스텝마다 약 0.1°씩 커지므로 시각 맞춤이 맞다.
- H(실차 시계, 호스트 CPU 99 %)에서는 에뮬레이터가 `/clock`의 11.7 %를 빠뜨렸고, 빠진 스텝 동안 관절이 마지막 명령에 머물러 모델보다 늦었다(조향 속도 × 틈, p99 1.41°). Gazebo의 RTF는 0.99였으므로 RTF 규칙으로는 무효가 아니다(15.3절 제안). 이 묶음의 A25 실패는 에뮬레이터 경로의 과부하 결과로 적는다.
- E·F의 래치: v3 대본의 +250 → −250 mrad 정지 계단에서 호스트가 1.5 s 뒤 "requested −250 mrad, measured 27 mrad"로 `steering_not_following`을 걸었다(호스트 추종 기준 3°·1.5 s, 맞는 검출). 대본 문제라 v4에서 계단을 0에서 ±150 mrad 이하로, 사인을 100 mrad·8 s로 줄였다.
- F의 `/clock` 빠짐 424개(12.8 %)는 호스트 CPU 97~99 %일 때다. SIM 시계라 판정에는 영향이 없지만 빠진 스텝의 참값 행이 없어 짝 수가 줄었다.

#### 9.9.4 정상 대본 오검출 (A19(G), RG-01)

| 정상 대본 | 결과 파일(묶음) | 오검출 | 따로 적은 SIM 쪽 래치 |
|---|---|---|---|
| 스택 시작 | B, C, D, A2, E, F, G, H, I | D 1(`steering_status_stale`, v2: 느린 구간 기록 없음) | C, E, G, H 각 1 |
| verify | A2 | 0(deadman의 TTL은 기대) | 0 |
| A9 주행 | A2, C | 0 | 0 |
| A17 STOP 창 | B, A2, E, H | 0 | E 1, H 1 |
| a25 | A2, H, I | H 1(`steering_not_following`), I 2(`F TTL`, `steering_status_stale`) | A2 1 |
| G-SIM lockstep 펌웨어 모드(gz_probe `a25_firmware`) | P5 결과 | 1(THERMAL, `steps_standstill`) | |
| 합(정상 SIM 시간 918 s) | | **5**: stale 2, not_following 1, TTL 1, THERMAL 1. A19(G)의 "정상 대본 오검출 0" 기준 **실패** | 7 |

- STALL 오검출 0(정상 SIM 시간당 0). `steering_ineffective` 오검출률은 실행기의 motion op가 있어야 해서 재지 않았다.
- H·I a25의 오검출은 호스트 CPU 99 %·74 %(다른 작업 흐름)일 때 에뮬레이터가 `/clock`의 약 12 %를 빠뜨린 실행이다. SIM 시계 모드도 Gazebo는 실시간으로 돈다: 에뮬레이터가 따라가지 못하면 상태 줄이 브리지의 SIM 시각보다 늦고 C 명령이 TTL을 넘는다. 실차 시계의 무효 규칙(RTF, 멈춤, 느린 구간)에 걸리지 않아 셌다. 이런 경우를 SIM 쪽 원인으로 가르는 규칙은 제안이다(15.3절).
- 요약에서 뺀 것: E·F a25(v3 대본이 호스트 추종 기준을 넘는 계단 +250 → −250 mrad를 줌, v4로 다시 잼), B a25(움직임 없음), 묶음 A(v1, 창 자르기 오류).
- 결과 파일: `checks/summary/a19g_normal.json`. 시작 검사의 SIM 시간은 검사 자신의 길이(약 0.7 s)만 셌다.

#### 9.9.5 A13·A14·verify·A9

- A13(A, A2, D): 규칙 R1~R5 위반 0. 실행기 노드가 없어 R6은 "없음". D(field1010)에서 자르기 노드 `ubts_steer_scan_clip`이 `/sim/scan`과 `/ground_truth/odom`을 구독하고 `/sim/scan_clipped`를 낸다(구독자 0: S2 노드는 다음 단계). 참값 구독자는 허용 목록 안이다.
- A14(A, A2): 통과. 검사 컨테이너의 pty 슬레이브(udev 자료 없음, 다른 ID_SERIAL로 위조한 자료) 둘 다 실차 전송이 거부했다. 에뮬레이터 슬레이브는 다른 devpts라 검사 컨테이너에서 보이지 않는다(보고).
- verify(A, A2): 통과. 원 R/R_geom 1.0226/1.0220(A), 1.0228/1.0217(A2), IMU 33.32/33.33 Hz. A의 오검출 1은 창으로 자른 브리지 기록의 오류였다(15.4절). A2 0.
- A9 drive(보고): W0(A2) 최종 x 7.836 m, y 1.145 m, 방향 35.01°. W1(C) x 12.763 m, y 0.223 m, 방향 7.73°, 직진 끝 옆 −0.013 m(처음 5번 평균 −0.053 m). `/clock` 큐를 50으로 늘려 에뮬레이터가 `/clock`을 빠뜨리지 않게 되면서(RG-03) 에뮬레이터의 갱신 주기가 바뀐 것과 맞는다. 한 번씩이라 분산은 재지 않았다.

#### 9.9.6 A10 실시간 비율 (RG-09, RG-11)

| 묶음 | 정의 | 중앙 | 하위 10 % | 최소 | 0.9 아래 창 | 토픽 율(SIM 초당) |
|---|---|---|---|---|---|---|
| C W1 | 240 s, 앞 20 s 제외, 5 s 창 43개 | 0.99997 | 0.995 | 0.965 | 0 | 카메라 30.29, IMU 33.33, GPS 4.80, 조향 상태 20.00, cpu_lidar 10.005 |
| D W2 | 같음 | 0.981 | 0.606 | 0.410 | 8(한 덩어리 약 40 s) | 카메라 30.28, IMU 33.34, GPS 4.80, 조향 20.00, cpu_lidar 10.02, 자르기 노드 약 9.3 |
| G W2 | 90 s, 앞 10 s 제외 | 0.343 | 0.280 | 0.262 | 15(전부) | 호스트 CPU 99 %, 비교하지 않음 |

- RTF 출처는 검사 노드의 `/clock` 수신과 단조 시계다(P5는 gz 서버의 WorldStatistics). 렌더 경로: GPU D3D12(WSLg `/dev/dxg`), ogre, 카메라 640×360, GUI 없음.
- 모든 검사 결과에 5 s 창 RTF 요약(`rtf`)과 에뮬레이터 `/clock` 계수(`emulator_clock`: 틈 수, 받은 수, SIM 시간으로 기대한 수, 빠진 수)를 적었다. 시간 민감 단계는 단계마다 `validity`를 적는다.
- W2 결과는 `WORLDS_KO.md` 12.4절에도 적었다.

#### 9.9.7 시작·종료 (RG-10)

- 모든 묶음(A~I)의 `stop -t 20` 종료 코드: sim 0, steer 0, recorder 0, dashboard 143(SIGTERM으로 끝남, 기존 대시보드 코드, SIGKILL 아님). 매 `down` 뒤 이 프로젝트의 컨테이너 0개, 네트워크 0개.
- startup 오검출(정상 대본: 스택 시작): A 실패 1(v1, 멈춤 귀속 없음), B 0, C 0(SIM 쪽 래치 1을 따로 적음: 340 ms 멈춤), D 실패 1(v2, 느린 SIM 귀속 없음), A2 0, E 0(따로 1: 340.7 ms 멈춤), F 0, G 0(따로 1: 느린 구간, 받은 SIM 속도 0.084배), H 0(따로 1: 335 ms 멈춤), I 0.
- steer 역할은 sim 역할의 모든 자식이 준비된 뒤(`all_ready`, 같은 실행 토큰) 켜진다. 그래도 실차 시계에서 시작 1~2 s 뒤 약 340 ms 멈춤이 자주 보였다(C, E, A2 중간 등). C에서는 같은 컨테이너의 브리지도 같은 순간 334 ms 멈췄고, 곧이어 벽시계(CLOCK_REALTIME)가 +0.55 s, +0.67 s 뛰었다(브리지 기록의 단조 시각과 벽시각 비교). 원인은 확인하지 못했다(VM 수준 정지 또는 컨테이너 CPU 몫 의심, 15.3절 제안). H·I에서 검사마다 steer·sim 컨테이너의 cgroup `cpu.stat`을 앞뒤로 남기려 했으나, 저장소 밖 도구의 MSYS 경로 변환 때문에 컨테이너 안 경로를 읽지 못해 자료가 없다.

## 10. 발견 사항

개발 실행과 최종 실행에서 찾은 것이다. 1~6은 펌웨어·모델 상호작용(A19), 7~14는 스택 동작이다. 등록 기준은 바꾸지 않았다. 결정이 필요한 것은 제안으로 넘긴다(이 문서 밖, 작업 결과).

1. 계단형 가변저항 고장(개방·단락·점프)에서 POT_NOISE가 같은 틱이나 한 틱 먼저 걸린다. POT_NOISE는 raw 값 nsw개의 분산을 보고, 큰 계단 두 표본이면 sd가 nsd/10(5카운트)를 넘는다. 개방·단락·점프는 중앙값(med 5)을 거쳐 2틱 뒤에 보인다. 그래서 호스트는 코드 두 개를 받는다. 판정 시각 자체는 설계값과 같았다(9.6절).
2. 가변저항 잡음과 STALL·TRACK 진행 기준이 가깝다. 기본 보정(raw_l 609, raw_c 512, raw_r 418)에서 1카운트가 약 10.3‰이고 프로필 잡음 sd는 1카운트다. stm은 20‰(약 2카운트)다. 모델 잡음을 그대로 두면 STALL은 늦게 걸리고, TRACK은 stt 창 안의 잡음을 "진행"으로 읽어 걸리지 않는다. 그 전에 호스트의 `steering_not_following`(1.5 s)이 먼저 래치한다(보고 사례 `stall_model_noise`, `track_model_noise`). 판정 사례는 적합성 대본처럼 ADC를 그 값에 고정했다.
3. 모델의 구동기 고착(v1 `driver_stuck_on`)은 DRV_STUCK보다 POLARITY가 먼저 걸린다(보고 사례 `drv_stuck_model`). DRV_STUCK 판정 사례는 적합성 대본 `fault_drv_stuck_default`처럼 ADC 램프(9틱에 1카운트)로 증상을 만들었다.
4. 기본 CFG(tha 5000, thw 30)로는 서보가 포화해도 THERMAL보다 호스트의 `steering_not_following`(1.5 s)이 먼저 온다. THERMAL 사례는 적합성 대본 `fault_thermal_release`처럼 CFG tha 50, thw 1로 돌렸다. PWM이 thp를 들락날락하므로 기대 시각은 "hot 틱의 (tha + 1)번째 다음 틱"으로 셌다.
5. 무장한 서보에 POT_NOISE 무늬(±6카운트 교대)를 주면 위치가 브레이크 중 떠도는 것처럼 보여 DRV_STUCK이 먼저 걸릴 때가 있다(개발 실행 한 번). 판정 사례는 DISARMED에서 돌리고, 무장 사례는 보고로 둔다.
6. TRACK의 기대 시각은 cmd가 P가 아니라 계단 전 cmd(cmd0)에서 출발한다는 점을 넣어야 맞는다: `C 도착 + ceil(1000 (P + trk + 1 − cmd0) / rstd) + trt − 1`. 처음 식(trk / rstd)은 23틱 일렀다.
7. SIM 시계 모드에서 브리지는 `/clock` 표본에서만 움직인다. 120 Hz `/clock`과 20 ms 격자에서 STOP 요청 → 송신은 8.3, 16.7, 25.0 ms로 나오고, 최악 25 ms가 A17의 20 ms를 넘는다(9.5절, 제안).
8. 가상 GPS 안테나 높이: 고정 `sim_telemetry`의 0.70 m를 그대로 쓴다(표시 전용). 프로필 `gnss_antenna_pose_in_base`는 0.9 m다. `gps_link` 정적 TF도 0.70 m다.
9. 실차 시계 모드 시작 때 첫 `/clock` 틈(약 0.13 s)으로 `steering_status_stale`가 한 번 래치될 수 있다. 다음 무장이 지운다. 브리지가 처음 연 포트로 보낸 INFO 바이트는 포트 열기 리셋 때문에 잃는다(호스트가 다시 보냄).
10. `EspSteerV3`는 한 포트 세션에서 `sim=0` 배너를 받으면 세션 끝까지 거부한다. 같은 세션에서 `sim=1` 배너가 와도 풀리지 않는다. 안전한 동작이고, 검사가 확인했다(포트를 다시 열면 새 세션).
11. IMU 33 Hz는 120 Hz 물리 스텝에서 3·4스텝 간격으로 나온다. 헤더 시각으로 잰 율은 실행마다 30.7~33.3 Hz였다(9.1절).
12. 에뮬레이터는 CPU 약 1코어를 쓴다(lockstep, 120 Hz `/clock`, 기록, 참값 발행). 호스트가 바쁘면 `/clock` 처리 사이 벽시계 틈이 100 ms를 넘고(첫 묶음 37번, 최대 8.7 s), 그때 MCU 상태 줄이 150 ms 넘게 늦어 호스트가 `steering_status_stale`를 래치한다. 안전 동작은 맞지만 시간 민감 검사가 끊긴다(9.0절). 메모리 예산 도구는 CPU를 나누지 않는다.
13. 감독기 종료: 기존 `gps_ws_bridge`, `lidar_ws_bridge`는 SIGTERM에 끝나지 않아 6 s 뒤 SIGKILL된다. `gz sim`은 끝날 때 SIGSEGV(−11)를 낸다. sim 감독기 종료에 약 6.5 s가 걸려 compose 기본 `stop`으로는 sim이 137로 끝난다(9.8절).
14. 이미지의 브리지 코어는 HEAD 판이다. WF-3a가 작업 트리에서 고치는 중인 `op motion`은 이미지에 없다. P6a 검사는 `op motion`을 쓰지 않는다.

## 11. 시험하지 않은 것

- 실행기 컨테이너, S2 형식 노드, `/scan`: 다음 단계다. A13의 실행기 구독 허용 목록(R6)은 합성 그래프 시험만 했다.
- field1010(W2) 장면, 자르기 노드: P7이다. 감독기의 `gz_assemble` 호출은 이미지의 wf3 판 `gz_assemble.py`로만 돌려 봤다. P7이 고치는 판과의 맞춤은 미시험이다.
- GUI 클라이언트(gz gui, RViz): 켜지 않았다.
- P6b 조향 화면, 녹화 내용(A23): P6b다. 녹화기 서비스는 떠 있었지만 녹화 1회 검사는 하지 않았다.
- 필수 장애 시험 중 시스템 시각 변경(컨테이너에 CAP_SYS_TIME 없음; P3 단위 시험이 단조 시계를 확인), 누른 채 통신 끊김(조향 화면 필요), 대시보드 강제 종료(이 단계의 대시보드는 조종 경로 밖): 미시험.
- 몇 시간 단위 연속 실행: 에뮬레이터의 기록 상한(BoundedFirmwareBackend)은 단위 시험(3 s)만, 로그 돌림(64 MB, 32 MB)은 돌림 크기까지 가 보지 않았다.
- 여러 스택 동시 실행: 하지 않았다. 다른 작업과 CPU를 나눈 상태는 이번에 우연히 겪었다(9.0절). 통제된 부하 시험은 하지 않았다.
- W1의 SIM 시계 모드: 돌리지 않았다.
- 실차 래퍼가 udev 신원으로 실제 USB 장치를 고르는 경로: 실기 없이 할 수 없다. 이번에는 pty와 경로 규칙으로 거부되는 것만 봤다.
- 실차, Jetson, 펌웨어 쓰기, 장착 보정: 하지 않는다(사용자 현장 작업).
- 리뷰 수정 뒤(9.9절) 남은 것: 실차 시계에서 STOP 5번이 모두 유효한 A17 묶음, 조용한 호스트에서 v4 대본의 A25 에뮬레이터 경로(실차·SIM 시계), 자르기 노드의 자세 보간(스냅숏 v5)의 스택 실행, W2 RTF 재측정, 약 340 ms 멈춤의 원인(cgroup 기록 수집 실패). 모두 미시험이다.

## 12. 개발 실행 이력

- 개발 실행은 `ubts-steer-sim:wf3` 위에 스냅숏 소스를 읽기 전용으로 붙여 돌렸다(`runs/wf3b/p6a/checks/dev_*`). 최종 결과(9절)와 섞지 않는다.
- A13: 첫 그래프 잡기에서 노드가 덜 보였다(`_NODE_NAME_UNKNOWN_`). 같은 그래프가 두 번 잡히고 기대 노드가 모두 보일 때까지(최대 40 s) 다시 잡도록 고쳤다.
- A17: 링크 끊김 옆 벗어남을 처음에 `base_footprint`로 쟀다. 회전 중 옆 미끄럼이 섞여 호 계산과 맞지 않았다. 뒤차축 중심 궤적으로 바꿨다.
- A19: 첫 판에서 POLARITY, DRV_STUCK, STALL, TRACK, THERMAL이 설계 시각과 맞지 않았다. 원인과 고침은 10절. 기대 시각을 못 찾으면 시각 기준을 건너뛰지 않고 실패로 적도록 바꿨다.
- 스택 도구: `compose run`이 환경 변수 차이로 sim 서비스를 다시 만들었다(두 번). 6절의 `--no-deps`와 같은 환경 값으로 막았다.
- 최종 실행 첫 묶음(`w0`)의 에뮬레이터·브리지 사건 기록은 다음 묶음이 기록 파일을 새로 열면서 없어졌다. `w0b`부터 검사마다 사본을 남겼다(`checks/final/<묶음>/eventlogs/`).

## 13. 시험

| 명령(요지) | 환경 | 결과 |
|---|---|---|
| `pytest test_steering_p6a.py test_steering_p6a_emulator.py test_steering_gz_model.py` | Windows Python 3.13 | 86 통과, 2 건너뜀(pty는 Linux 전용, W1 조립 시험은 SIM 컨테이너 전용) |
| `pytest scripts/test_steer_*.py`(대시보드) | Windows Python 3.13 | 109 통과, 10 건너뜀(Unix 소켓 등) |
| `unittest test_steering_p6a test_steering_p6a_emulator test_steering_gz_model` | WSL Python 3.10 | 88개 실행, 1 건너뜀(W1 조립 시험), 나머지 통과 |
| `unittest scripts.test_steer_rover_bridge` | WSL Python 3.10 | 15개 통과 |
| 같은 pytest 세 파일 | 이미지 `ubts-steer-sim:p6a` Python 3.12 | 86 통과, 2 건너뜀 |
| `pytest scripts/test_steer_rover_bridge.py … test_steer_bridge_equivalence.py`(6파일) | 이미지 Python 3.12 | 115 통과 |
| `python -I tools/steer_sim/check_pins.py` | Windows | FAIL: 고정 파일 변경 7개 모두 허용 목록 안(예상 밖 0), 그러나 WF-3a의 `tools/row_executor/vendor/{closed_loop,orchard_perception}`가 모듈 이름을 가림(이 작업 파일 아님) |

- Windows: `cd sim/ubts; python -B -m pytest -q -p no:cacheprovider --basetemp=<runs>/pt test_steering_p6a.py test_steering_p6a_emulator.py test_steering_gz_model.py`(UBTS_STEER_PATHS, UBTS_STEER_BUILD_DIR, UBTS_STEER_CXX=MinGW g++). 대시보드: `cd dashboard; python -B -m pytest ... scripts/test_steer_*.py`.
- WSL 3.10: `python3 -B -m unittest test_steering_p6a test_steering_p6a_emulator test_steering_gz_model`(네이티브 빌드는 runs 폴더), `cd dashboard; python3 -B -m unittest scripts.test_steer_rover_bridge`.
- 이미지 3.12: `docker run --rm --network none ... ubts-steer-sim:p6a`에서 같은 pytest.
- 시험은 작업 트리에 파일을 만들지 않는다(임시 폴더). 에뮬레이터 시험은 네이티브 프로그램이 없으면 건너뛴다(`UBTS_STEER_REQUIRE_FIRMWARE`가 있으면 실패).

리뷰 수정 뒤(fix_wf3b, 2026-11-09, 마지막 소스 = 스냅숏 v5):

| 명령(요지) | 환경 | 결과 |
|---|---|---|
| `pytest test_steering_*.py test_field_world.py`(조향 시험 전부: 1·2단계와 WF-3a 시험 포함) | Windows Python 3.13 | 934 통과, 3 건너뜀, 실패 0(5분 36초). 같은 날 앞선 실행에서는 WF-3a의 `test_steering_ident` 1개(벤치 도구의 엄격 검사)가 실패했다가 마지막 실행에서 통과했다(이 작업의 파일 아님) |
| `pytest scripts/test_steer_*.py`(대시보드 7파일) | Windows Python 3.13 | 111 통과, 10 건너뜀 |
| `unittest test_steering_p6a test_steering_p6a_emulator test_steering_gz_model`, `unittest scripts.test_steer_rover_bridge`, 단순 실행기(`test_steering_worlds`, `test_field_world`) | WSL Python 3.10 | 106개 실행 1 건너뜀 나머지 통과, 17 통과, 37 통과 1 건너뜀(scipy 필요, 결정 R10) |
| `pytest test_steering_p6a.py test_steering_p6a_emulator.py test_steering_gz_model.py test_steering_worlds.py test_field_world.py test_steering_scene_spec.py`, `pytest scripts/test_steer_rover_bridge.py` | 이미지 `ubts-steer-sim:p6a` Python 3.12(스냅숏 v5를 붙임, 2 CPU·2 GiB) | 190 통과 4 건너뜀, 17 통과 |
| `python -I tools/steer_sim/check_pins.py` | Windows | OK(고정 423개: 그대로 416, 바뀜 7개 모두 허용 목록 안, 예상 밖 0) |

- WSL의 브리지 시험은 TMPDIR을 `/mnt/d`(drvfs)에 두면 Unix 소켓을 못 만들어 2개가 오류였다. 기본 `/tmp`로 다시 돌려 통과했다(시험 코드 문제 아님).
- 스택 검사(9.9절)는 Gazebo가 도는 이미지 안의 Python 3.12다.

## 14. 이미지와 스냅숏

- 이미지 `ubts-steer-sim:p6a`, ID `sha256:e3b4059d7a6102e47e0bfb6be866926c21ba4838799e3cabdf8930f05ded1cb0`. 기반 `ubts-sim-clean-tenth:20261008-…@sha256:6209168a3507…`(빌드 전후 같음). 이 작업 흐름(WF-3b)의 두 번째이자 마지막 새 이미지 ID다. `ubts-steer-sim:p5`(e440445947ff)와 `ubts-steer-sim:wf3`는 그대로 둔다. 새 dangling 이미지는 없다.
- 빌드 문맥은 작업 트리를 그대로 쓰지 않았다. 작업 트리는 WF-3a와 p7이 함께 고치는 중이다. 문맥 = HEAD(`0c1311e`) + wf3 이미지 판 `gz_assemble.py`·`gz_probe.py` + p6a가 맡은 파일 19개(스냅숏 도구는 저장소 밖 `runs/wf3b/p6a/make_snapshot_p6a.py`, 기록 `snap_p6a/SNAPSHOT.json`). 다른 사람이 고치는 중인 파일 50개는 HEAD 판으로 들어가거나 빠졌다(목록은 SNAPSHOT.json). 그래서 이미지 안의 WF-3a 파일(`steer_bridge_core.py`, `native/*` 등)은 HEAD 판이다. 개발 실행도 같은 규칙의 스냅숏으로 돌렸다.
- manifest(`/opt/ubts/STEER_IMAGE_MANIFEST.json`): `source_commit` `0c1311e2…`, `source_status_sha256` `3f858fb1…`(p6a 파일의 `git status --porcelain` 줄), 파일 SHA 합 `3b2a3f4ae7bcbda1…(스냅숏의 이미지 파일 SHA 합과 같음, 917파일)`, 네이티브 빌드 `firmware_build_id` 04353B49.
- Dockerfile 고침: `IMAGE_TAG_HINT` 기본값 `ubts-steer-sim:p6a`, P6a 모듈 import 확인 줄. apt·pip 없음.
- 빌드: `docker buildx build --builder desktop-linux -f ctx/docker/Dockerfile.ubts-steer --build-arg BASE_IMAGE_ID=… --build-arg SOURCE_COMMIT=… --build-arg SOURCE_STATUS_SHA256=… --build-arg IMAGE_TAG_HINT=ubts-steer-sim:p6a --load -t ubts-steer-sim:p6a ctx`(예산 4 GiB를 잡고 풂). 기록 `runs/wf3b/p6a/logs/build_p6a.log`.
- 리뷰 수정(fix_wf3b)은 새 이미지를 만들지 않았다. 스택과 시험 컨테이너는 이 이미지에 고친 소스 스냅숏을 읽기 전용으로 붙여 돌렸다(9.9절). 그래서 그 결과의 `sources_match_image`는 거짓이고, 소스 판별은 스냅숏 기록(`runs/wf3b/fix_wf3b/logs/SNAPSHOT_v*.json`, 파일별 SHA와 출처)이다. 이미지 manifest의 `source_status_sha256`(`3f858fb1…`)과 스냅숏의 `5db6d4a2…`는 서로 다른 파일 묶음의 `git status` 줄이다.

## 15. 2026-11-09 리뷰 수정 (작업 표지 fix_wf3b)

리뷰 review_g의 지적 13건과 review_world의 RW-01을 고쳤다. 다시 잰 결과는 9.9절이다. 다시 재면서 찾은 결함과 고침은 15.4절이다. 등록 기준은 바꾸지 않았다. 기준 문구에 없는 것을 넣어야 할 때는 제안으로만 적는다(15.3).

### 15.1 고친 것

| 지적 | 고친 것 | 파일 |
|---|---|---|
| RG-01 정상 대본 오검출 | 정상 대본(스택 시작, verify, drive, A17의 STOP 5번, a25)의 기록 창에서 기대하지 않은 `F`·`W` 줄, 새 결함 비트(BOOT 제외), 새 호스트 래치를 센다(`false_detections`, R15-3 정의). verify의 deadman 단계 TTL만 기대한다. `a19g_normal` 요약이 결과 파일과 G-SIM lockstep 결과(gz_probe `a25_firmware`)를 모아 A19(G) 오검출과 STALL 오검출률(정상 SIM 시간당)을 낸다. `steering_ineffective` 오검출률은 실행기의 motion op가 있어야 잴 수 있어 미측정으로 적는다 | `p6a_checks.py`, `verify_steer.py` |
| RG-02 판정 시각 | 사례를 `record` 걸이로 시작한다(같은 세션 값 + `report_adc`). 기록에서 틱마다 analogRead 값과 조향 핀을 꺼내 펌웨어 감시 규칙(`fw_monitor.cpp`)을 그대로 다시 계산한다(`p6a_fw_rules.py`: 중앙값 필터, `raw_to_permille`, POT_NOISE 분산 창, POLARITY·DRV_STUCK·STALL 실행 카운터, THERMAL 창, SOFTLIMIT 조건). POT_NOISE, POLARITY, DRV_STUCK, STALL, THERMAL은 규칙 틱 ±1, SOFTLIMIT은 조건이 시작된 뒤 첫 상태 줄에 비트가 있는지로 판정한다. TRACK은 서보 램프 cmd가 기록에 없어 추정값이라 시각을 판정하지 않고 보고한다. 에뮬레이터 핵심 시험에서 실제 펌웨어 F 줄과 규칙 틱이 6개 코드 모두 차이 0이었다 | `p6a_fw_rules.py`(새), `p6a_checks.py`, `gz_esp_emulator.py`(`record`·`restart`의 `report_adc`, `record_finish` 걸이) |
| RG-03 A25 에뮬레이터 경로 | 새 검사 `a25`: `/sim/truth/joint_states`의 조향 관절 각과 같은 SIM 시각의 공유 모델 δL·δR(`/sim/truth/steering`)을 비교한다(p99, 최대, 1/120 s 스텝 지연별 오차, 관절 속도 튐). 모든 검사 결과에 에뮬레이터 `/clock` 틈 수와 빠진 표본 수를 적는다(`emulator_clock`). `/clock`과 `/sim/truth/joint_states`의 ROS 발행 큐를 1에서 50으로 늘렸다 | `p6a_checks.py`, `p6a_client.py`, `bridge_steer.yaml` |
| RG-04 SIM 시계 STOP | STOP이 걸리면 다음 폴에서 격자를 기다리지 않고 코어를 돌린다. 주석을 고쳤다 | `dashboard/scripts/steer_rover_bridge.py` |
| RG-05 A17 허용 | STOP 송신 ≤ 20 ms(1 ms 여유 없앰). watchdog은 마지막 입력 + wd + 1 이하. STOP 재전송은 브리지의 루프 틱 기록으로 "격자 시각 뒤 첫 루프 틱"인지 정확히 본다(실차 시계 1 ms, SIM 시계 1 µs 여유). STOP 확인은 STOP 바이트에서 호스트 확인까지로 판정하고, MCU 쪽 상태 줄은 보고로 둔다 | `p6a_checks.py`, `steer_rover_bridge.py`(`tick` 기록) |
| RG-06 끊김 옆 벗어남 | 해석 호 길이 = 끊김 때 속도 × C TTL 남은 시간 + 정지 모드 거리(`drive.stop_modes`, TTL 만료 모드 `ttlmode`). SIM 경로 길이로 낸 값은 보조 보고 | `p6a_checks.py` |
| RG-07 R6 허용 목록 | 등록 문구대로 판정한다. `/clock`, `/parameter_events`, `/rosout`도 허용 목록 밖이면 실패다. 기반 토픽은 따로 적고, 개정이 받아들여지면 나올 판정(`passed_if_infrastructure_allowed`)을 보고한다 | `p6a_graph.py` |
| RG-08 A14 pty | 검사 컨테이너에서 직접 만든 pty 슬레이브로 실차 전송을 시험한다(udev 자료 없음, 다른 ID_SERIAL로 위조한 udev 자료 둘 다 거부해야 통과). 에뮬레이터 슬레이브(다른 devpts)는 보고로만 둔다 | `p6a_checks.py` |
| RG-09 A10(W1) 정의 | `rtf` 기본 240 s, 앞 20 s 제외, 5 s 창 중앙값. RTF 출처(검사 노드의 /clock 수신)와 렌더링 경로를 결과에 적는다 | `p6a_checks.py` |
| RG-10 시작·종료 | 모든 서비스에 `stop_grace_period: 20s`, 머리말 `down -t 20`. 실행 토큰으로 옛 READY 표지를 거르고, steer 역할은 같은 토큰의 살아 있는 `/clock`을 본 뒤 켠다 | `compose.ubts-steer.yaml`, `supervisor_steer.py`, `sensor_contract_steer.py` |
| RG-11 A10 유효성 | 시간 민감 단계마다 그 단계를 덮는 5 s 창 RTF와 단계 동안 에뮬레이터 `/clock` 벽시계 틈(100 ms 초과)을 적는다. 실차 시계에서 RTF < 0.9이거나 그런 틈이 있으면 그 단계의 판정을 하지 않는다(무효 표시). 모든 결과에 5 s 창 RTF 요약을 적는다 | `p6a_checks.py` |
| RG-12 laser TF | `base_link -> laser` 회전 = `lidar_yaw_offset_deg`(S2 형식 `/scan` 관례). 실차 대시보드 소스에는 laser 정적 TF 발행자가 없다. `/sim/scan`(고정 cpu_lidar, frame 'laser'지만 몸체 좌표)은 이 TF로 그리면 안 된다 | `sensor_contract_steer.py` |
| RG-13 틱 순서 | "8, 8, 9" → "8, 9, 8"(주석, 3절) | `gz_esp_emulator.py`, 이 문서 |
| RW-01 자르기 노드 연결 | 감독기에 field1010 장면과 field world 명세(`--field-spec`, `UBTS_STEER_FIELD_SPEC`), 자르기 노드 명령(`--raster` 포함), 노드 이름 `ubts_steer_scan_clip`(그래프 허용 목록과 같게) | `supervisor_steer.py`, `scan_unobserved_clip.py`, `p6a_graph.py` |

### 15.2 시험

- 새 시험: `test_steering_p6a.py`(R6 엄격 판정, 자르기 노드 그래프 규칙, laser TF yaw, compose 유예, 감독기 명령·토큰·잡음 사본, 오검출 집계, 첫 루프 틱 규칙, 해석 호, A25 보조 함수, 계수 차이, 무효 표시, `a19g_normal` 요약), `test_steering_p6a_emulator.py`(`record`의 `report_adc`, `record_finish`, 6개 코드의 규칙 틱 = 펌웨어 F 줄), `dashboard/scripts/test_steer_rover_bridge.py`(SIM 시계 STOP이 격자를 기다리지 않음, STOP 동안 틱 기록).
- 고치기 전 판에서 새 시험은 실패한다: R6(`/parameter_events`를 받던 판), STOP 대기(격자점까지 기다리던 판), 자르기 노드 인자(`--raster` 없음), 규칙 틱 함수·걸이(없던 기능).
- 15.4절의 시험: a25 가짜 스택(첫 `/clock` → ARM → drive 순서, 움직임 없는 실행은 실패), `a25_summary`, `a25_motion`, 느린 `/clock` 구간(멈춤 뒤 몰아 받기, 반속 SIM), 느린 구간 래치 귀속, 단계 유효성(멈춤, SIM 쪽 래치, SIM 시계), STOP 창 블록 합치기와 처음 판 A17 파일의 요약, 자르기 노드의 자세 기다림(늦은 자세, 잃은 자세, 8개 초과).

### 15.4 다시 재면서 찾아 고친 것

9.9절의 스택 실행에서 처음 판(스냅숏 v1, v2)의 결함이 드러났다. 고친 판은 스냅숏 v3~v5다(v4: a25 계단, v5: 자르기 노드 보간). 처음 결과 파일은 지우지 않았다.

| 드러난 것 | 고친 것 | 파일 |
|---|---|---|
| A17 `link_loss`의 기준 문구에 `%` 형식 오류(`10 %,`)가 있어 안전 검사 전체가 예외로 끝났다(묶음 A) | `%%`. 모든 `문자열 % 값` 식을 AST로 훑어 다른 곳이 없음을 확인했다 | `p6a_checks.py` |
| a25가 시작 시각을 `/clock`을 받기 전에 읽어 예외로 끝났다(묶음 A) | 첫 `/clock`을 기다린 뒤 시작 시각을 잡는다 | `p6a_checks.py` |
| a25가 ARM 뒤 첫 `/clock`을 기다리느라 첫 drive가 2.3 s 늦었고, 브리지가 `stale_input`으로 무장을 풀어 조향이 전혀 움직이지 않았는데도 "통과"했다(묶음 B: 0과 0을 비교) | 첫 `/clock`을 ARM 전에 받는다. 판정 항목 `steering_moved_deg`(공유 모델의 가장 큰 \|δ_eq\|가 가장 큰 계단 목표의 절반 이상)와 `drive_loop_refusals`(거부 0)를 더했다. 가짜 스택 시험이 순서와 "움직임 없음 = 실패"를 확인한다 | `p6a_checks.py`, `test_steering_p6a.py` |
| 정상 대본 검사가 창으로 자른 브리지 기록을 받아, 창 앞에서 이미 걸려 있던 래치를 새 래치로 셌다(묶음 A verify의 오검출 1) | 브리지 기록은 처음부터 넘긴다(창 앞의 래치는 "알던 것") | `p6a_checks.py`, `verify_steer.py` |
| A17 STOP 단계의 창이 STOP 순간부터라, STOP 전 3 s 주행 중 에뮬레이터 멈춤(123 ms)이 래치를 걸어 차가 이미 서 있었는데도 단계가 유효로 판정되어 A17이 실패했다(묶음 A2 STOP 2) | 시간 민감 단계(STOP, 확인 실패, 끊김, 결함 정지)의 창을 ARM 앞부터 잡는다. 단계 안에 SIM 쪽 원인의 호스트 래치가 있으면 그 단계는 무효(A10)다 | `p6a_checks.py`(`step_start`, `step_sim_latches`, `step_validity`) |
| 시작 때 SIM이 실시간보다 느려(field1010) 상태 줄이 150 ms 넘게 늦어 걸린 래치를 SIM 쪽 원인으로 가를 근거가 없었다(묶음 D startup 실패) | 에뮬레이터가 받은 SIM 시각을 단조 시계와 견줘 0.3 s 창에서 0.9배보다 느린 구간을 기록한다(`clock_slow`). 실차 시계 모드에서 그 구간 안(끝 뒤 1 s까지)의 `steering_status_stale`은 멈춤과 같이 따로 적고 세지 않는다. 브리지 기록의 `t`(실차 시계 모드의 단조 시계, 같은 컨테이너)로 맞춘다 | `gz_esp_emulator.py`, `p6a_checks.py`, `verify_steer.py` |
| A17 결과의 오검출 블록에 `by_code`가 없어 요약이 STOP 창의 코드를 빠뜨릴 수 있었다 | STOP 창 블록을 합친다(`merge_false_detections`). 요약은 처음 판 A17 파일도 읽는다 | `p6a_checks.py` |
| 자르기 노드가 같은 stamp의 자세보다 스캔을 먼저 받으면 스캔을 버렸다(묶음 D: 2,211개 중 75개, 3.4 %). 단일 스레드 실행기는 기다리는 구독마다 메시지 하나씩 꺼낸다 | 스캔이 자세를 기다린다(최대 8개). 더 새 자세가 먼저 오면(제 자세를 잃음) 또는 8개를 넘으면 버리고 센다 | `scan_unobserved_clip.py`, `test_field_world.py` |
| startup 결과의 에뮬레이터 계수가 한 번도 오르지 않은 항목을 null로 적었다 | 0으로 적는다 | `p6a_checks.py` |
| a25(v3)의 정지 계단 +250 → −250 mrad에서 1.5 s 뒤에도 −250 요청에 +27 mrad라 호스트가 `steering_not_following`을 걸었다(묶음 E·F, 맞는 검출: 대본이 호스트 추종 기준 3°·1.5 s를 넘는 계단을 줌) | 계단을 0에서 ±150 mrad 이하로(정지 150, 0, −150, 0, 80, 0, −80, 0, 주행 150, 0, −150, 0), 사인을 100 mrad·8 s로(스냅숏 v4). 시험이 계단 크기를 확인한다 | `p6a_checks.py`, `test_steering_p6a.py` |
| 자르기 노드의 남은 버림(묶음 G: 383개 중 13개, 더 새 자세가 먼저 온 경우로 봄: best effort 자세를 잃음) | 잃은 자세를 앞뒤 자세(50 ms 이내) 사이로 보간한다(위치 선형, 쿼터니언 정규화 선형). 스냅숏 v5, 단위 시험만(스택 미시험) | `scan_unobserved_clip.py`, `test_field_world.py` |

### 15.3 제안 (등록 문서는 통합 담당이 고친다)

- A13 개정 제안: 실행기 노드 구독 허용 목록에 `/clock`(SIM 시계 모드, 시각만 담음)을 더한다. `/parameter_events`, `/rosout`은 rclpy가 만드는 기반 토픽으로 따로 둔다. 개정 전까지 R6은 엄격 판정이다.
- A19(G) 판정 시각: TRACK은 펌웨어 cmd(서보 램프)가 HEAD 네이티브 기록에 없어 규칙 틱을 정확히 낼 수 없다. `report_servo` 채널(WF-3a 작업 트리)이 들어간 이미지에서 다시 판정한다.
- A10(실차 시계)의 무효 규칙을 판정 코드에 넣었다: 5 s 창 RTF < 0.9, 에뮬레이터 `/clock` 벽시계 틈 > 100 ms, 단계 안 SIM 쪽 원인의 호스트 래치(멈춤 또는 받은 SIM 시각이 0.3 s 창에서 0.9배보다 느린 구간). 100 ms, 0.3 s 값은 이 작업이 정한 것이다(설계값 아님). 단계 창은 ARM 앞부터다.
- (제안) 에뮬레이터 경로의 과부하: 호스트가 포화되면(다른 작업 흐름, CPU 74~99 %) Gazebo의 RTF는 1 근처인데 에뮬레이터가 `/clock`의 약 12 %를 빠뜨리고, 관절 − 모델 오차가 p99 1.41°가 되거나(실차 시계 H) 호스트 래치가 걸린다(SIM 시계 I). RTF 규칙에 걸리지 않는다. 단계 안 에뮬레이터 `/clock` 빠짐이 몇 % 넘으면 무효로 할지(값은 결정 필요), 또는 시간 민감 검사 동안 Gazebo를 에뮬레이터와 lockstep으로 묶을지 정해야 한다. SIM 시계 모드는 브리지의 시계만 바꾸므로 부하에 강하지 않다.
- (제안) A25 에뮬레이터 경로 판정: 에뮬레이터가 `/clock`을 빠뜨리지 않은 실행(E: 빠짐 0, p99 4.2e-11°)에서는 관절이 공유 모델과 같다. 판정은 위 무효 규칙을 정한 뒤 조용한 호스트에서 다시 한다.
- (제안) 실차 시계 시작 직후와 실행 중의 약 340 ms 멈춤: 같은 컨테이너의 에뮬레이터와 브리지가 함께 멈추고 벽시계가 뛴 경우가 있다. 컨테이너 CPU 몫(steer 1.5)과 VM 수준 정지를 가르려면 cgroup `cpu.stat`(throttled)과 VM 시계 기록이 필요하다(이번 수집 실패).

## 16. 4단계 g_gaps 소스 변경 (2026-11-09 작업, Docker 검증 전)

- 2026-11-09 작업 흐름 WF-4의 g_gaps 단계가 사용 한도로 중간에 멈췄다. 그 변경을 새 저장소로 옮겼다(`DECISIONS_KO.md` R22). 이 절의 내용은 **Docker 검증 전**이다: Gazebo 스택에서 아직 돌리지 않았고, Docker 없이 도는 단위 시험(`test_steering_p6a_emulator.py`, `test_steering_p6a_checks_units.py`)만 통과를 확인했다. 단계가 남긴 개발 기록(부하가 있는 호스트에서 잰 값, 판정 아님)은 저장소 밖 `runs/wf4/g_gaps/`에 있다. 최종 결과와 판정은 ST-04가 이 절과 9절에 넣는다.
- 들어온 것:
  - ESP 에뮬레이터: /clock이 SIM 시간 1 s 동안 고르게 온 뒤에 pty 링크를 만든다(3단계 시작 때의 호스트 래치 대응, 늦어도 첫 /clock 뒤 벽시계 20 s). 바뀌지 않은 `/sim/act/*` 값은 12스텝(0.1 s)마다만 다시 보낸다(펌웨어가 보는 것은 같다는 것이 단계의 주장이고 아직 확인하지 않았다). `--truth-every`. 상태 파일에 에뮬레이터 자신의 부하(CPU, /clock 콜백 시간 분포, 발행 수). 직렬 줄에 단조 시각. 훅 `record`가 `report_servo`를 켠다(TRACK 판정 틱), 훅 `kill_bridge`(필수 장애 시험 '브리지 강제 종료').
  - 검사(`p6a_checks.py`): `bridge_kill`(브리지 SIGKILL → TTL 구동 0 → watchdog → 감독기 재시작 → 포트 열기 리셋 → 재출발 없음), `a3noise`(DISARMED, `report_adc`, 정지 10,000틱 이상, 개정 15), `clip`(자르기 노드 ROS 출력과 래스터 DDA 비교, 스탬프→발행 지연), 실행 범위(run token) 밖의 옛 상태 파일 무시, 호스트 래치 시각을 HOST_BRIDGE 설계값(±1 루프)과 BOOT/NOCAL 시각으로 판정, 모든 요약에 출처 칸(커밋, 상태 SHA, 프로필 id와 SHA, 코드 SHA; `field_world/provenance.py`).
  - `p6a_fw_rules.py`: 평가 전용 채널(`servo_pos`, `servo_tgt`, `servo_cmd`, `steer_mode`)로 TRACK을 추정이 아니라 판정한다.
  - `p6a_imu_model.py`: G-SIM IMU 오차 모델과 F-SIM 모델의 일치 비교(`compare_with_fsim`).
  - `sensor_contract_steer.py`: 가상 GPS 레버암과 `gps_link` 정적 TF를 프로필 `gnss_antenna_pose_in_base`에서 계산한다(고정 파일 `sim_telemetry.py`는 그대로).
  - `gz_direct_drive.py`: 데모의 '뜬 바퀴' 규칙을 토크 단위(DC 선의 순 토크)로 바꿨다(R19 결정 14). `gz_probe.py`: fdir 이방 진단에 상대 허용을 더했다(옛 절대 기준 결과도 남긴다). W1 바퀴 중심 높이·옆면 접촉력을 직접 재는 일은 이 변경에 없다.
  - `p6a_stall_probe.py`: 조향 컨테이너의 약 335 ms 멈춤을 조사하는 탐침(`/proc` 프로세스 카운터, 스레드 상태, cgroup v2 cpu.stat·PSI). 컨테이너 안에서 `python3 -B -m steering.p6a_stall_probe --seconds 600 --out ...`. 아직 스택에서 돌리지 않았다.
- 들어오지 않은 것: 자르기 노드의 빠른 경로(`scan_unobserved_clip.py`의 블록 여유 + 제한된 DDA, `field_world/raster_ops.py`의 `first_stop_steps`, `chebyshev_rings`). ST-04 계획대로 따로 떼어 저장소 밖 패치(`runs/steer_phase4_20261112/held/g_gaps_fast_clip.patch`)로 보류했다. `test_field_world` 단위 시험과 A7 재측정을 통과할 때만 넣는다. 지금 자르기 노드는 3단계의 `first_stop` 경로 그대로다.
- 하지 않은 문서 일(감사 G02, G05): 3단계 최종 자격 결과를 9절·`WORLDS_KO.md`에 넣기, A12 등록 형식 수를 6개 중 3개로 고치기(R19 결정 15). ST-04에서 한다.
