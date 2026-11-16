# 실차 주행 평가 방법

- 작성: 2026-11-16(계획). 고침: 2026-11-16 — 지도 기반 오프라인 LiDAR 위치 추정과 평가 스크립트를 `tools/field_eval/`로 만들었다(5절).
  **빠른 SIM 기록(참값 있음)으로만 확인했고, 실차 기록으로는 아직 한 번도 돌리지 않았다.** 바닥 자국 측정표, 줄기 측량 절차, RTK 입력
  경로는 아직 계획이다.
- 목적: SIM과 같은 지표(README 7절: 횡오차 RMSE·p95·최대, 방향 오차, 열 끝 정지 위치 e, 속도)를 실차에서 재는 방법을 정한다.
  SIM은 참값(진짜 위치)을 바로 읽지만, 실차에는 참값이 없으므로 기준 중심선과 차의 실제 궤적을 따로 재야 한다.
- 원칙: 실행기가 쓰는 추정(LiDAR 통로 band)과 **다른 방법**으로 참값을 만든다. 같은 추정을 정답으로 쓰면 오차가 보이지 않는다.

## 1. 무엇을 기록하나 (이미 있는 것)

| 기록 | 내용 | 있는 곳 |
|---|---|---|
| 실행기 JSONL | 틱마다 온라인 추정(횡·방향 오차, 통로 폭, 열 끝 거리), 출력(throttle, 목표 조향), 입력 나이 | `tools/row_executor/executor_main.py` |
| 대시보드 기록기 rosbag2 | `/scan`, IMU, `/gps/fix`·`/gps/data`, 조종 요청·적용, 카메라(선택) | 대시보드 저장소 `scripts/data_recorder_service.py` |
| 기록기 sidecar | `mapping_source/sensors.jsonl.gz`: `/scan`, `/imu/data_raw`(JSON 줄, 수신 시각 포함) | 같은 기록 폴더(`tools/field_calib/records.py`가 읽음) |
| 조향 사이드카 | 목표·측정 조향각, 가변저항, PWM | `sim/ubts/steering/steering_sidecar.py`(SIM), 실차는 브리지 상태 기록 |
| 오프라인 LiDAR 주행 거리 | 기록의 스캔을 다시 정합한 이동 속도·경로 | `tools/drive_response/` |

모든 기록은 시각으로 맞춘다. 실행기와 기록기는 같은 Jetson 시계를 쓴다. 실행기 틱 시각은 `time.monotonic`이고 사건 줄에는 벽시계
(`wall`)가 같이 있어, 도구는 사건 줄의 `wall - t` 중앙값으로 틱을 기록기 시각(벽시계)에 옮긴다.

## 2. 단계 1: 실내 평지 (직선)

1. **기준선.** 바닥에 곧은 중심선을 줄자로 표시하고, 그 양쪽 1.75 m에 기둥(상자·콘)을 1.5 m 간격으로 둔다(SIM W0와 같은 배치). 기둥
   위치를 줄자로 적는다(5.1절 지도 파일).
2. **참 궤적 A (수동).** 차의 발자국 중심 아래에 펜·분필을 달아 바닥에 자국을 남기고, 주행 뒤 1 m마다 중심선과의 거리를 줄자로 잰다.
   정밀도는 약 1 cm, 시간 정보는 없다. 측정값을 CSV(`x_m, y_m`)로 적으면 도구가 같은 지표를 낸다(`--trajectory`). **[측정표 양식: 계획]**
3. **참 궤적 B (LiDAR, 지도 기준).** 적어 둔 기둥 위치를 지도로 삼아 기록의 스캔을 오프라인으로 그 지도에 맞춘다(지도 기반 위치 추정).
   실행기의 통로 band와는 다른 계산이며, A와 비교해 정확도를 먼저 확인한다. **[도구: `tools/field_eval`(5절). SIM으로만 확인, 실차 미확인]**
4. **비교.** 참 궤적으로 README 7절의 지표를 계산하고, 실행기 JSONL의 온라인 추정과 참 궤적의 차(추정 오차)를 따로 그린다(도구가 함).
5. **함께 볼 그래프.** 목표 속도 대 LiDAR 속도, 목표 조향각 대 측정 조향각, duty 대 속도(도구가 그림). 조건(속도, 배터리 전압, 바닥)과
   현장 조정 값(`config_digest`, 도구 보고서에 들어감)을 run마다 같이 적는다.

## 3. 단계 2: 과수원 한 행

1. **기준 중심선.** 출발 행 양쪽 줄기 위치를 잰다(줄자로 행 시작 기준 거리와 옆 거리, 또는 RTK 측량 막대). 두 줄기 열에 각각 직선을 맞추고
   그 평균을 중심선으로 둔다. Gazebo 평가(`steer_follow_gsim_v1`)와 같은 정의다. 열 끝 기준은 행 줄의 마지막 구조물이다(결정 R50).
   **[줄기 측량 절차: 계획]**
2. **참 궤적.** RTK GNSS(기지국 또는 NTRIP, FIXED만 사용)의 안테나 위치를 발자국 중심으로 옮긴다(안테나 레버암을 줄자로 잼).
   방향은 IMU와 RTK 궤적에서 구한다. 위 단계 1의 지도 기반 LiDAR 위치 추정(줄기 지도)을 교차 확인으로 쓴다. 행 국소 좌표로 옮긴 RTK
   궤적은 `--trajectory` CSV로 넣을 수 있다.
   **[RTK 입력 경로 없음: 지금 Jetson GNSS 드라이버는 FIXED와 FLOAT를 가르지 않는다(`GNSS_RTK_KO.md`)]**
3. **지표와 기록**은 단계 1과 같다. 위경도는 기록·문서에 남기지 않고 행 시작 기준 국소 좌표로만 쓴다(도구는 위경도처럼 보이는 이름과
   2 km를 넘는 좌표 값을 거부한다).

## 4. 만들어야 할 것

| 항목 | 상태 |
|---|---|
| 실내 기준선·기둥 배치 절차, 바닥 자국 측정표 | 계획(측정값을 CSV로 넣는 입력은 구현) |
| 지도 기반 오프라인 LiDAR 위치 추정 도구(기둥·줄기 지도) | **구현** `tools/field_eval/localise.py`. 빠른 SIM 참값으로 확인(5.6절). 실차 기록 미확인 |
| 기록(rosbag2 + 실행기 JSONL)에서 지표와 그래프를 만드는 평가 스크립트 | **구현** `tools/field_eval/evaluate.py`, `cli.py`. 지표 코드는 `follow_matrix.measure`와 같은 값을 냄(시험, SIM 6 run) |
| rosbag2 읽기(표준 라이브러리) | **구현** sqlite3, MCAP(압축 없음; zstd 청크는 `zstandard` 모듈이 있을 때). Jetson이 쓴 실제 bag으로는 미확인 |
| RTK FIXED를 가르는 GNSS 입력 경로, 안테나 레버암 | 계획(사용자: 기지국·NTRIP 준비) |
| 줄기 위치 측량 절차 | 계획 |

이 문서의 단계는 실차 조향 시험(바퀴 든 상태, 대시보드 저장소 `docs/FIELD_STEER_AUTONOMY_TEST_KO.md`)과 식별 시험(B0~B7, F1~F5,
[`IDENT_KO.md`](IDENT_KO.md)) 뒤에 한다.

## 5. 평가 도구 `tools/field_eval/` (2026-11-16 구현)

오프라인 도구다. 표준 라이브러리와 numpy만 쓰고 Python 3.10 이상용으로 썼다(3.10으로는 문법만 확인, 시험은 3.13 + numpy 2.2).
Jetson에서도 돌릴 수 있게 matplotlib 없이 SVG로 그린다.
실행기 패키지(`tools/row_executor`)에 넣지 않았고 실행기는 이 도구를 부르지 않는다. 시뮬레이션·실행기 동작은 바뀌지 않았다.
참값은 PC 전용 `sim_verify.py`만 읽는다.

### 5.1 입력

| 입력 | 형식 |
|---|---|
| 기준 지도 | JSON(`ubts.field_eval.map.v1`) 또는 CSV. **행 국소 미터**(원점은 행 시작 근처, x는 행 방향, y는 왼쪽). 기둥·줄기마다 `row`, `x`, `y`, 선택 `radius_m`(없으면 `default_radius_m`, 기본 0.05 m), 또는 줄마다 두 끝점(`row_lines`; 중심선만, 위치 추정은 못 함). 줄이 셋 이상이면 `lane_rows`로 출발 통로의 두 줄을 고른다. 선택 `start_pose` [x, y, 방향°] |
| 실행기 JSONL | 파일 하나, 회전된 여러 파일, 또는 폴더(`*.jsonl`, `*.jsonl.gz`). run이 여럿이면 구동 명령이 있는 마지막 run(또는 `--run`) |
| 스캔 출처 | ① F-SIM 캡처 폴더(`SCANS.npz`, `IMU.npz`; `sim_capture.py`가 씀) ② 기록 폴더의 `mapping_source/sensors.jsonl.gz` ③ rosbag2 폴더(`*.db3` 또는 `*.mcap`; `/scan` LaserScan, `/imu/data_raw` Imu를 CDR에서 직접 읽음) |
| 차량 값 | 기본은 실행기 `config` 사건의 `lidar_x_in_base_m`, `lidar_yaw_offset_deg`, `footprint_length_m`, `footprint_width_m`, `imu_yaw_sign`. `--vehicle` JSON(같은 이름과 `lidar_y_in_base_m`)이 덮어쓴다. 보고서에 값마다 출처를 적는다 |
| 외부 기준 궤적(선택) | `--trajectory` CSV: `x_m, y_m`(지도 좌표)와 선택 `t_s`, `yaw_deg`. 바닥 자국 측정값이나 RTK 궤적용 |

CSV 지도 예:

```text
id,row,x_m,y_m,radius_m
R01,right,0.00,-1.74,0.06
L01,left,0.02,1.77,0.06
```

### 5.2 기준 중심선과 열 끝

`gsim_follow.row_geometry`와 같은 정의다: 줄마다 최소제곱 직선 y = a + b x, 중심선은 두 직선의 평균(a, b의 평균), 행 축은 그 방향,
열 끝 s_E = 두 줄 구조물의 행 방향 좌표 최댓값(결정 R50). 기울어진 두 줄에서 `gsim_follow`와 같은 값이 나오고, W0에서는 `follow_matrix`의
중심선 y = 0, 열 끝 24.0 m와 같다(시험).

### 5.3 지도 기반 LiDAR 위치 추정 (`localise.py`)

실행기의 통로 band를 쓰지 않는다. 스캔마다:

1. 빔 j의 몸체 각 = angle_min + j·증분 + `lidar_yaw_offset_deg`(실행기와 같은 S2 장착 규약). 0.30~15 m 밖과 차체(발자국 + 0.10 m) 위 점은 버린다.
2. 이웃 점 간격이 0.10 m + 2·r·Δθ보다 크면 끊어 군집을 만들고, 점 3개 이상·폭 0.60 m 이하만 기둥·줄기 후보로 둔다.
3. 군집 중심 = 지도 반지름의 원을 점에 맞춘 중심(반지름 고정 Gauss-Newton).
4. 예측 자세로 옮긴 중심을 가장 가까운 지도 기둥에 붙인다(문 0.50 m, 첫 풀이 뒤 0.25 m, 기둥 하나에 군집 하나).
5. 짝지은 중심으로 SE(2)를 가중 최소제곱(닫힌 꼴)으로 풀고, 잔차가 max(0.10 m, 중앙값×3)보다 큰 짝을 빼고 다시 푼다.
6. 다음 스캔 예측: 지난 자세 + 평활한 속도 × 방향 + IMU 요레이트 적분(가속도로 구한 '위' 방향에 자이로를 투영, 출발 전 정지 구간에서 bias).

시작: 주어진 시작 자세 둘레(행 방향 ±1.0 m, 옆 ±0.5 m, ±10°)를 격자로 찾아 "지도 기둥 가까이 있는 군집 수 − 8 m 안에 보여야 하는데
군집이 없는 기둥 수"가 가장 큰 자세에서 시작한다. 품질(스캔마다): **ok**(짝 4개 이상이 두 줄 모두에, 잔차 RMS ≤ 0.05 m, 예측과 차 ≤ 0.30 m),
**weak**(짝 1개 이상), **lost**(짝 없음: 예측값), **rejected**(예측과 0.75 m 넘게 다른 풀이: 버림). 지표는 ok·weak만 쓴다.

**행 방향 한 칸 어긋남.** 기둥 지도는 행 방향으로 주기적이라, 시작 위치가 한 간격쯤 틀리면 풀이가 한 칸 어긋난 채 모든 스캔이 'ok'로
나올 수 있다(SIM에서 실제로 봄: 1.5 m 틀린 시작에서 횡오차는 맞고 e만 +1.5 m). 그래서 전체 궤적을 ±1 간격 옮긴 점수를 비교하고
(`along_check`), 옮긴 쪽이 나으면 그 시작으로 다시 풀어 더 나은 쪽을 남기며 보고서 `warnings`에 적는다. 행 끝(첫 기둥 줄, 마지막 기둥 줄)이
보여야 가려진다.

### 5.4 지표와 출력 (`metrics.py`, `evaluate.py`)

지표는 `follow_matrix.measure`(SYSTEM_REFERENCE 8절)를 행 좌표로 옮긴 것이다: 주행 구간 = 실행기 틱 T > 0인 처음~끝, 횡오차 = 발자국
중심의 d, 방향 오차 = 방향 − 행 축, RMSE·p95(출발 어긋남이면 `--settle-m 6` 뒤부터), 최대, e = 정지 자세의 행 방향 좌표 + 앞범퍼·cos ψ − s_E
(정지 = 주행 끝 + 5 s 안의 마지막 측정 자세), 최대 속도(기준 궤적 ±0.5 s 직선 맞춤). 실행기 추정 오차 = 틱의 `lat` − 기준 d,
(−`head`) − 기준 ψ. 10 ms 참값에 이 코드를 쓰면 등록 지표와 같은 값이 나온다(SIM 6 run, 7지표 모두 같음).

출력 폴더: `REPORT.json`(입력 파일마다 SHA-256, 실행기 `config_digest`, 차량 값과 출처, 위치 추정 설정·품질·`along_check`, 지표,
SIM 한도와 나란히 놓은 값(판정 아님), 추정 오차, `warnings`), `TRAJECTORY.csv`(스캔마다 자세, 행 좌표, 속도, 품질), 그림(SVG):
`fig_trajectory.svg`(전체 관측 범위 1:1 + '상세' 횡 확대), `fig_lateral_vs_distance.svg`(횡·방향 오차와 실행기 추정 오차 대 거리),
`fig_speed.svg`(목표 대 LiDAR 속도), `fig_steering.svg`(목표 대 측정 조향각), `fig_duty_speed.svg`. 그림 아래에 출처(결과 종류, 지도 SHA-256)를 적는다.

### 5.5 사용법

저장소 맨 위에서(출력 폴더는 저장소 밖):

```bash
# 실차 기록 하나 (기록 폴더: rosbag2/ 나 mapping_source/ 가 있는 곳)
python -B tools/field_eval/cli.py evaluate --map <row_map.json> --executor <실행기 jsonl 또는 폴더> \
    --scans <기록 폴더> --out <출력 폴더> [--start-pose X,Y,YAW_DEG] [--vehicle <vehicle.json>] [--settle-m 6]
# 바닥 자국 측정값이나 RTK 궤적을 기준으로
python -B tools/field_eval/cli.py evaluate --map <row_map.json> --executor <...> --trajectory <ref.csv> --out <...>
python -B tools/field_eval/cli.py map-info --map <row_map.json>     # 중심선·열 끝 확인
# PC 전용: 빠른 SIM 캡처와 참값 비교
python -B tools/field_eval/sim_capture.py --out <폴더> --only nominal lateral_p040 steer_zero_p2 --seeds 0
python -B tools/field_eval/cli.py verify-sim --run-dir <캡처 run 폴더> --out <폴더> [--start-error=A,L,DEG] [--map-noise-m 0.02]
python -B -m unittest discover -s tools/field_eval/tests           # 시험 20개(합성 자료)
```

음수로 시작하는 값은 `--start-error=-0.4,0.25,-6`처럼 `=`로 붙인다. 시작 자세는 `--start-pose`, 없으면 지도의 `start_pose`를 쓴다.

### 5.6 SIM 확인 (자리값 기반 SIM 결과, 2026-11-16)

빠른 SIM W0(기둥 지름 0.24 m, 1.5 m 간격, LiDAR 잡음 1.5 cm) 6 run: `follow_matrix` nominal s0·s1, lateral_p040, steer_zero_p2,
heading_p8(프로필 v0.1)과 `follow_realdims` nominal s0(측정 치수 프로필). `sim_capture.py`가 실행기에 들어간 S2 스캔과 IMU를 저장했다
(nominal s0은 같은 작업 트리의 `follow_matrix` run과 세 digest가 모두 같고, 나머지 follow_matrix 4 run은 다른 작업 트리의 이전 run과
trace·직렬 digest가 같다; 실행기 기록 digest는 기록에 든 현장 조정 파일 경로만 다르다). 도구는 실차와 같은 입력(지도, 실행기 JSONL, 스캔·IMU)만
받았고, 결과를 TRACE 참값과 비교했다. 기록: 작업 폴더 `runs/field_eval_tool_20261116/`(capture, verify2).

| 경우 | 위치 오차 p50 / p95 / 최대 | 지표 차(도구 − 등록 지표) |
|---|---|---|
| 기본 6 run(정확한 지도, 참 시작 자세) | 0.9 / 1.8~1.9 / 2.5~3.3 mm, 방향 p95 0.02° | 횡 RMSE −0.1~+0.2 mm, p95 −0.5~+0.7 mm, 최대 0~+1.4 mm, 방향 RMSE −0.004~+0.002°, 방향 최대 −0.014~+0.009°, e −1~0 mm, 최대 속도 −9~−1 mm/s |
| 시작 자세 오차 (0.35, −0.2 m, 5°), (−0.4, 0.25 m, −6°), 행 방향 1.0 m | 기본과 같음(시작 탐색이 찾음) | 기본과 같음 |
| 시작 자세 행 방향 1.5 m(한 간격) | 첫 풀이가 1.5 m 어긋남 → `along_check`가 찾아 다시 풀고 기본과 같음 | 기본과 같음(경고 남김) |
| 지도 위치 잡음 σ 2 cm(줄자 오차 모형, 4경우) | 2.5~5.8 / 4.2~8.6 / 5.0~10.1 mm | 횡 RMSE ±2.2 mm, p95·최대 ±4.8 mm, e ±7 mm, 방향 RMSE −0.014° 이하 |
| 지도 위치 잡음 σ 5 cm | 14.1 / 21.5 / 23.0 mm, 모든 스캔 'weak'(잔차 > 5 cm, 경고) | 횡 RMSE −3.6 mm, p95·최대 −7.8 mm, e +18 mm |
| 지도 기둥 20 % 뺌 | 1.0 / 2.1 / 3.6 mm | 기본과 같은 크기 |

실행기 온라인 추정 오차를 도구로 잰 값과 참값으로 잰 값의 차: 횡 RMSE ±0.1 mm, 평균 ±0.2 mm, 방향 RMSE +0.001° 이하(기본 6 run;
지도 잡음 2 cm에서 횡 RMSE +0.9 mm, 방향 RMSE +0.008° 이하). 최대 속도가 낮게 나오는 것은 ±0.5 s 직선 맞춤 때문이다.
PC에서 run 하나(약 700 스캔) 처리 약 10 s. Jetson 처리 시간은 재지 않았다.

**이 정확도는 실차 정확도가 아니다.** SIM 기둥은 완전한 원기둥이고 지도가 정확하며 지면·풀·가지·차체 기울기·스캔 중 움직임이 없다.
실차에서는 줄기 모양, 줄자 지도 오차, 풀·가지 반사, 경사, 차체 흔들림이 더해진다. 실차 정확도는 단계 1의 바닥 자국(A)과 비교해 처음 정한다.

### 5.7 한계와 남은 일

- 실차 기록·실제 Jetson rosbag2로 돌린 적이 없다. rosbag2 읽기는 이 도구가 쓴 bag으로만 시험했다(필드 이름·정렬이 다르면 첫 실제 bag에서
  드러난다). MCAP zstd·lz4 청크는 `zstandard`·`lz4` 모듈이 없으면 읽지 못하고 이유를 알린다. 파일 단위 압축 bag은 읽지 않는다.
- 시작 자세가 필요하다(줄자로 잼). 행 방향 오차는 약 1 m까지 시작 탐색이 찾고, 한 간격 어긋남은 `along_check`로 고친다. 행 끝이 보이지 않는
  곳에서 시작하면 가리지 못할 수 있다.
- 가림(occlusion)은 모형화하지 않았다(보여야 할 기둥 검사는 8 m 안만). 스캔 한 바퀴(0.1 s) 동안의 움직임은 보정하지 않는다.
- 지도에 없는 줄기·물체는 지도 기둥에서 0.25 m보다 멀 때만 무시된다. 기둥 바로 옆의 잡초·가지는 짝을 잘못 지을 수 있다(잔차·품질에 드러남).
- Gazebo W1(나무가 있는 장면)과 실제 과수원 줄기에는 아직 돌리지 않았다.
- 남은 절차: 바닥 자국 측정표 양식, 줄기 측량 절차, RTK FIXED 입력 경로(4절).
