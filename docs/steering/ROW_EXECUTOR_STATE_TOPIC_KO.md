# 실행기 상태 토픽 `/row_executor/state` (schema `ubts.row_executor.state.v1`)

- 작성: 2026-11-19. 근거: 사용자 요청(2026-11-19) "자율주행 녹화를 보면 계산 과정의 시각 자료도 같이 보이게", "그리드맵의 중심선과 실제 이동 경로의 오차를 실시간으로".
- 코드: `tools/row_executor/executor_main.py`(`state_message`, `ExecutorLoop._emit_state`, `RosInput`의 발행자, `--state-topic`), `core.py`(`Runner.state_geometry`), `run_metrics.py`(4절의 `metrics`). 시험: `tools/row_executor/tests/test_field1119.py`(`StateMessageTest`).
- 표시·녹화 전용이다. 실행기 안의 어떤 결정도 이 토픽을 읽지 않고, 발행이 실패해도 틱은 멈추지 않는다(실패는 `counts.state_errors`, 처음 세 번은 `state_topic_error` 사건).
- 이 문서의 수치는 자리값이거나 실행기 자기 추정이다. 참값이 아니다.

## 1. 발행

| 항목 | 값 |
|---|---|
| 토픽 | `/row_executor/state`(실행기 인자 `--state-topic`, 빈 값이면 발행하지 않음) |
| 형식 | `std_msgs/msg/String`, `data` = JSON 객체 하나(공백 없는 구분자, NaN 없음, 숫자는 반올림) |
| QoS | 기본(신뢰, 깊이 10) |
| 주기 | run이 열려 있는 동안(PAUSED·ACTIVE) 약 10 Hz(0.1 s), 그 밖(run 없음, run 끝남)에는 약 1 Hz(1.0 s). 실행기 20 Hz 틱에서 보낸다 |
| 크기 | 보통 1~1.5 KB |
| 보내는 쪽 | 실행기 주 루프 스레드(같은 rclpy 노드 `ubts_row_executor`) |

녹화기(대시보드 레포)는 'executor' 센서 묶음을 고르면 이 토픽을 rosbag에 넣는다. 대시보드는 자율 탭과 자율 녹화 재생에서 중심선, 경로, 앞봄점, 격자를 이 토픽으로 그린다.

## 2. 좌표

- `frame` = `run_odometry`: 이번 run의 추정기 odometry 좌표(run의 첫 스캔이 원점 근처, 지도·GNSS 아님). Start마다 새 좌표다.
- `pose`: 차체 기준점(base, 실행기 `lidar_x_in_base_m` 의 기준)의 odometry 위치와 방향 `{x, y, yaw_deg}`(yaw + = 왼쪽으로 돎). 지연 보정한 값(추정기 `base_pose`).
- 차체 좌표(`base`): x = 앞, y = 왼쪽, 원점 = `pose`.
- 경로는 연속된 메시지의 `pose`를 이으면 된다(메시지에 경로를 싣지 않는다).

## 3. 칸

| 칸 | 형식 | 뜻 |
|---|---|---|
| `schema` | 문자열 | `ubts.row_executor.state.v1` |
| `stamp` | 수 | 벽시계 시각[s, epoch](실행기 `t_wall`) |
| `t` | 수 | 실행기 단조 시계[s] |
| `run_id` | 정수 | run 번호(실행기 시작 뒤 0부터) |
| `mode` | 문자열 | `SHADOW` / `MOTOR` |
| `run_state` | 문자열 | `idle`, `ready`, `armed`, `running`, `paused`, `ended`, `latched`(`OPERATOR_API_KO.md` 4절) |
| `state`, `reason` | 문자열 | Runner 상태(`IDLE`/`PAUSED`/`ACTIVE`/`ENDED`)와 그 이유 |
| `phase` | 문자열/null | 제어기 단계(`ALIGN`, `FOLLOW`, `APPROACH_END`, `STOP`, `STOPPED`) |
| `stop_reason` | 문자열/null | 멈춤 분류(`OPERATOR_API_KO.md` 4절) |
| `gnss` | `{mode, state}` | 이 run의 GNSS 층(`OPERATOR_API_KO.md` 4.2절) |
| `frame` | 문자열 | `run_odometry` |
| `pose` | `{x, y, yaw_deg}`/null | 2절. run 전에는 null |
| `lat_m`, `head_deg`, `width_m` | 수/null | 추정 횡오차(+ = 차가 중심선 왼쪽), 방향 오차(통로 축 − 차 방향, < 0 = 코가 왼쪽), 통로 폭 |
| `corridor_ok`, `corridor_reason` | 참/거짓, 문자열 | 추정기 통로가 유효한가, 그 이유 |
| `centreline` | 객체/null | 추정기 통로 중심선(제어기가 따르는 선): `odom` = [[x, y], [x, y]](odometry 좌표 선분: 차 기준점의 선 위 위치에서 뒤 2 m ~ 앞 8 m), `base` = 같은 두 점의 차체 좌표, `axis_deg` = 선 방향(odometry), `width_m` = 통로 폭. 두 열 선은 중심선을 왼쪽 법선으로 ±`width_m`/2 옮긴 선이다. 통로가 없으면 null |
| `grid_centreline` | 객체/null | 국소 점유 격자의 두 선 맞춤 가운데선(같은 형식). `local_grid_enabled`이고 맞춤이 1 s 안일 때만 |
| `lookahead` | 객체/null | 조향 제어기의 앞봄점(ACTIVE일 때): `base` = [L_d, y_L](차체 좌표: 앞봄 거리, 그 거리의 옆 목표; 곡률 = 2 y_L / L_d²), `odom` = 같은 점의 odometry 좌표 |
| `steer_cmd_mrad`, `steer_meas_mrad` | 정수/수/null | 조향 명령, 측정 조향(등가 바퀴각 mrad, + = 왼쪽) |
| `throttle_pct` | 정수 | 구동 명령 duty[%] |
| `speed_mps` | 수/null | 추정기 LiDAR 속도(5개 스캔 정합 걸음의 중앙값). 복도에서 실제보다 낮게 읽힌 기록이 있다(`FIELD_CORRIDOR_20261119_KO.md`) |
| `target_speed_mps` | 수/null | 이번 틱의 속도 목표(가장 낮은 상한) |
| `dist_m` | 수/null | 추정기 경로 거리 |
| `front_m` | 수/null | 앞 여유 |
| `metrics` | 객체/null | `run_id`와 `OPERATOR_API_KO.md` 4.3절의 칸 중 `ref, samples, distance_m, lat_rms_m, lat_mean_m, lat_max_abs_m, within_0_05_pct, within_0_10_pct, head_rms_deg, head_max_abs_deg, grid_samples, grid_lat_rms_m, grid_lat_mean_m, grid_lat_max_abs_m, grid_within_0_05_pct, grid_within_0_10_pct, grid_head_rms_deg, grid_head_max_abs_deg` |

- 값이 없거나 수가 아니면(무한대 등) null이다.
- 칸을 더할 때는 schema 이름을 그대로 두고(읽는 쪽은 모르는 칸을 무시한다), 뜻을 바꿀 때는 `v2`로 올린다.

## 4. 예(값은 시험 값)

```json
{"schema":"ubts.row_executor.state.v1","stamp":1795021500.25,"t":10620.3,"run_id":3,"mode":"MOTOR",
 "run_state":"running","state":"ACTIVE","reason":"follow_blind","phase":"FOLLOW","stop_reason":null,
 "gnss":{"mode":"off","state":"off"},"frame":"run_odometry","pose":{"x":5.012,"y":0.31,"yaw_deg":-1.2},
 "lat_m":0.12,"head_deg":0.8,"width_m":3.12,"corridor_ok":true,"corridor_reason":"tracked_full_candidate",
 "centreline":{"odom":[[3.0,0.19],[13.0,0.05]],"base":[[-2.01,-0.1],[7.99,-0.05]],"axis_deg":-0.8,"width_m":3.12},
 "grid_centreline":null,"lookahead":{"base":[2.93,-0.083],"odom":[7.94,0.166]},"steer_cmd_mrad":-21,
 "steer_meas_mrad":-18.0,"throttle_pct":38,"speed_mps":0.18,"dist_m":5.03,"front_m":15.2,"target_speed_mps":0.34,
 "metrics":{"ref":"estimator","samples":412,"distance_m":4.9,"lat_rms_m":0.061,"lat_mean_m":0.032,"lat_max_abs_m":0.21,
  "within_0_05_pct":48.1,"within_0_10_pct":83.0,"head_rms_deg":1.3,"head_max_abs_deg":4.9,"grid_samples":0,
  "grid_lat_rms_m":null,"grid_lat_mean_m":null,"grid_lat_max_abs_m":null,"grid_within_0_05_pct":null,
  "grid_within_0_10_pct":null,"grid_head_rms_deg":null,"grid_head_max_abs_deg":null,"run_id":3}}
```
