# GNSS 약한 층 L3 (PC-12)

- 작성: 2026-11-13. 작업 번호 PC-12. 설계 `DESIGN_v2_KO.md` V2-1 3항(L3), 결정 R31, 합격 기준 개정 30(A13).
- 코드: `tools/row_executor/gnss_layer.py`(층), `perception_settings.py`(설정, 표준 라이브러리만), `perception_layers.py`(Runner 연결), `config.py`의 `gnss_*` 묶음, `core.py`(`Runner.feed_gnss`, geofence 끝), `executor_main.py`(입력 대기열, `/gps/fix` 구독). SIM: `sim/ubts/steering/gnss_sim.py`(가상 GNSS), `t1_rig.py`(`RigConfig.gnss`), `tools/row_executor/sim_in_loop_steer.py`(GNSS 패킷), `safety_scripts/perception_compare.py`(개발 비교).
- 기본은 꺼짐(`gnss_mode` `'off'`)이다. 꺼져 있으면 층을 만들지 않고 설정 digest와 실행기 기록이 그대로다(5.1절).
- SIM 수치는 모두 **자리값 기반 SIM 결과**다. 가상 GNSS의 잡음 모형은 자리값이다(10/10 수신기의 짧은 시간 흔들림은 잰 적이 없다). 이 문서와 코드·기록에 GNSS 좌표는 없다. 실행기 기록에는 run의 첫 fix 기준 국소 미터만 남는다.

## 1. 왜 약한 층인가

10/10 기록의 수신기는 수평 정확도 p50 5.3~10.0 m, 속도 정확도 p50 0.43~0.56 m/s였다(`TIMING_BASELINE_KO.md` 6.2절). 열 간격은 3.6~4.1 m, 목표 속도는 0.40 m/s다. 그래서 일반 GNSS로는

- 행간 안의 위치(횡오차)도, 어느 열인지도 알 수 없다.
- 목표 속도에서 GNSS 속도·진행 방향은 쓸 수 없다. 위치 차분으로 맞춘 진행 방향도 SIM 잡음 모형에서 run 정렬 오차 p50 2.1~2.4°, 최대 9.4°이고, 15 m 창은 그 정렬에서 최대 24°까지 벗어났다(3.3절). 통로 방향 오차(목표 ±1.5° 수준)에 쓸 수 있는 값이 아니다.

따라서 L3는 통로 추정(횡오차, 방향 오차, 폭, 열 끝)과 조향·구동 명령에 들어가는 길이 **없다**. 할 수 있는 것은 넓은 여유를 둔 점검뿐이다: 진행 방향의 큰 어긋남, 위치의 큰 어긋남, 열 범위를 크게 벗어남.

## 2. 모드

| 모드 | 하는 일 | 차에 미치는 영향 |
|---|---|---|
| `off`(기본) | 아무것도 하지 않는다. `/gps/fix`를 구독하지 않는다 | 없음 |
| `log_only` | 아래 3절의 판정을 모두 하고 기록·상태에만 낸다 | 없음. 같은 seed에서 명령 열이 `off`와 같다(5.2절) |
| `geofence` | `log_only` + 열 범위 펜스가 확인되면 run을 끝낸다(`gnss_geofence`, 분류 `OPERATOR_OR_BACKSTOP`) | 펜스 밖으로 확인된 fix가 이어질 때만 정지 |

- 설계의 열린 선택 4는 현장 기본을 `log_only`로 제안한다. 실행기 설정의 기본값은 기존 설정의 digest를 지키려고 `'off'`로 두었다. 현장 설정 파일에서 `"gnss_mode": "log_only"`를 켠다.
- 설계에 적힌 `weak_constraint`는 만들지 않았다(RTK 보정 계획이 없음, 계획 4절 다7). 설정 검사가 거부한다.
- GNSS가 끊기거나 품질이 낮거나 튀면 'GNSS 없음'으로 처리한다. 그것만으로 서지 않는다(`log_only`, `geofence` 모두).

## 3. 판정 규칙 (`gnss_layer.GnssLayer`)

모든 값은 자리값이다. 설정 이름은 `ExecutorConfig`의 키다.

### 3.1 품질과 순서

- 받아들이는 fix: 수신기가 fix라고 하고(`NavSatFix.status.status ≥ 0`), 수평 정확도를 알고(공분산 형식 ≠ 0), 그 값이 `gnss_max_hacc_m`(10 m) 이하, 앞 fix보다 늦게 받은 것.
- 'GNSS 있음'은 마지막으로 받아들인 fix가 `gnss_max_age_s`(2.5 s)보다 새것일 때다.
- 결과 이름: `accepted`, `restarted`(끊김 뒤 다시 시작), `reanchored`(지속 계단 뒤 다시 잡음), `quality`, `no_fix`, `order`, `invalid`, `jump`.

### 3.2 튐 기각

- 마지막으로 받아들인 fix에서 실행기 odometry로 움직인 만큼 옮긴 예측과 새 fix의 차이(혁신)가 `gnss_jump_gate_m`(3 m)을 넘으면 기각한다(`jump`).
- 정렬(3.3) 전에는 두 좌표계 사이의 회전을 모르므로 크기만 비교한다: | |ΔGNSS| − |Δodometry| |. 정렬 뒤에는 벡터 차이를 쓴다.
- 서로 맞는 기각 fix가 연달아 3개(`reanchor_after`)면 수신기 해의 계단으로 보고 다시 잡는다(`reanchored`). 5 s(`jump_max_gap_s`)보다 긴 끊김 뒤에는 검사 없이 다시 시작한다(`restarted`).
- 한 fix짜리 튐은 기각 하나로 끝난다. 그래서 SIM의 단발 튐 조건에서 '기각 수 = 주입 수'를 볼 수 있다(5.3절).

### 3.3 진행 방향 점검(정렬과 course drift)

- run의 odometry 경로가 `gnss_course_baseline_m`(15 m)를 넘으면, 받아들인 (odometry, GNSS) 쌍 전체로 두 좌표계 사이의 회전을 맞춘다(2D Kabsch: 두 센서가 본 진행 방향). 경로가 기준의 두 배(30 m)가 될 때까지 다시 맞추고, 그 뒤에는 고정한다(기준 정렬).
- 마지막 15 m 창만으로 맞춘 회전과 기준 정렬의 차이가 course drift다. `gnss_course_tolerance_deg`(45°)를 넘으면 `course_disagree`를 표시한다(표시만).
- 이것은 heading 입력이 아니다. odometry 방향이 크게 무너졌을 때(예: 정합이 계속 틀려 회전이 쌓임)만 잡는 거친 점검이다. 합성 시험(`test_gnss_layer.py`)에서 60° 어긋남은 잡는다.
- 가상 GNSS 잡음 모형으로 0.4 m/s 직진을 seed 50개씩 돌린 값(SIM 모형만, 실행기 없이 층만; `runs/perception_layers_20261113/scripts/gnss_course_check.py`): 28.5 m 열은 정렬 오차 p50 2.38°, 최대 9.43°, course drift 최대 13.5°, 위치 불일치 최대 3.85 m. 60 m 열은 2.11°, 9.33°, 24.3°, 8.01 m. 표시(course·위치)와 펜스 작동은 모두 0이었다.

### 3.4 위치 불일치와 열 범위 펜스

- 정렬 뒤 받아들인 fix를 기준 정렬로 odometry 좌표에 옮겨, 추정기 기준점과의 거리를 낸다(`disagreement_m`). max(`gnss_fence_margin_m` 10 m, 3 × 정확도)를 넘으면 `position_disagree`를 표시한다.
- 펜스(열 범위): 여유 M = max(10 m, 3 × 수평 정확도).
  - 정렬 전: run의 첫 fix를 중심으로 반지름 (열 길이 + M)인 원.
  - 정렬 뒤: run 시작 자세에서 열 축(정렬할 때의 추정기 band 방향, 없으면 시작 방향)을 따라 [−M, 열 길이 + M], 옆으로 ±M인 사각형.
  - 열 길이는 단일열 모드에서 `single_row_max_row_length_m`(120 m), 그 밖에는 `max_row_len_m`. 운영자가 입력한 열 길이는 쓰지 않는다(넓게 잡아 거짓 정지를 줄임).
  - 받아들인 fix가 연달아 3개(`gnss_fence_confirm`) 펜스 밖이면 펜스가 걸린다. `geofence` 모드에서만 run이 끝난다.
- 정렬 회전이 몇 도 틀려도 120 m 열의 끝에서 10 m 남짓이다. 그래서 여유를 10 m보다 좁히지 않는다(설계 열린 선택 8).

### 3.5 실차 입력

- `executor_main.RosInput`이 `gnss_mode`가 `off`가 아닐 때만 `--gnss-topic`(기본 `/gps/fix`, `NavSatFix`)을 구독한다. 위도·경도는 `gnss_layer.LocalFrame`이 프로세스의 첫 fix 기준 국소 미터로 바꾼다. 원점은 메모리에만 있고 기록·상태·내보내기에 나가지 않는다.
- 수평 정확도는 `position_covariance`의 수평 분산 중 큰 쪽의 제곱근이다. 공분산 형식이 0(모름)이면 정확도를 모르는 fix로 보고 쓰지 않는다. 실제 드라이버가 공분산을 채우는지는 확인하지 않았다(남은 일).
- A13 개정 30: 정적 검사(`steering/truth_isolation.py`)가 실행기 노드 파일(`executor_main.py`)의 `/gps/fix` 문자열과 구독만 허용한다. 다른 결정 코드의 `/gps/*`, 같은 파일의 `/gps/data`는 여전히 오류다(`test_steering_truth_isolation_gnss.py`).

### 3.6 상태와 기록

- Runner 요약과 실행기 틱 기록에 `gnss` 블록: 모드, 있음/없음, 나이, 수평 정확도, 마지막 결과, 결과별 수, 표시별 수, 정렬 여부, course drift, 위치 불일치, 펜스 안/밖, 펜스 작동, run 첫 fix 기준 국소 위치(0.1 m 반올림).
- 대시보드는 이 블록을 그대로 보여 주면 된다('GNSS 없음'도 정상 상태로 보인다, V2-2).

## 4. SIM 가상 GNSS (`gnss_sim.py`)

- T1 리그가 참 안테나 위치(프로필 `gnss_antenna_pose_in_base`)에서 합성한다. 실행기는 fix 패킷만 받는다(참값 격리 A13: `gnss_sim.py`는 plant 역할).
- 1 Hz(10 ms 격자의 230 ms 위상), 지연 0.30 s, 보고 정확도 7.0 m(± 0.3 m 흔들림).
- 오차 = run마다 고정인 편향(축마다 σ 3 m) + 1차 Gauss-Markov 떠돎(σ 1.5 m, τ 60 s) + 흰 잡음(σ 0.4 m). 좌표계는 SIM 세계보다 30° 돌아 있다(실제 열이 동쪽과 나란하지 않음).
- 조건(장면의 `rig.gnss`):

| 이름 | 내용 |
|---|---|
| `gnss_nominal` | 위 기본값 |
| `gnss_dropout_60s` | 10~70 s 동안 fix 없음(조용한 수신기) |
| `gnss_jump_5m`, `gnss_jump_15m` | 30, 45, 60 s에 한 fix짜리 5 m / 15 m 튐(방향은 매번 다름) |
| `gnss_hacc_x3` | 10~90 s 동안 보고 정확도와 떠돎·흰 잡음 × 3(21 m: 품질 문턱 10 m를 넘음) |
| `gnss_bias_20m` | 좌표계 원점을 (20, −10) m 옮김, 편향 0(run 전체에 같은 큰 오프셋) |

- 리그의 GNSS 난수 흐름은 따로(목적 번호 301)라 GNSS를 켜도 플랜트·LiDAR·IMU 잡음은 같다. GNSS가 없는 장면은 바뀐 것이 없다.

## 5. 측정 (빠른 SIM, 개발 측정)

- **판정이 아니다**(사전등록 없음). 도구 `safety_scripts/perception_compare.py`, 커밋 `187b1f4`의 내보낸 트리, 2026-11-13 06:28~06:40 KST, 작업자 4, 잠금 안에서. 결과 원본 `runs/perception_layers_20261113/measure_187b1f4/sim/PERCEPTION_COMPARE.json`. 자세한 방법은 `LOCAL_GRID_KO.md` 6절.
- 세계는 W0(평지 기둥 두 줄, 28.5 m 열), 열 추종 제어기 `row_follow`, 목표 0.40 m/s. GNSS 조건은 nominal 열에 seed 0·1·2.

### 5.1 끄면 그대로, `log_only`이면 명령이 그대로

| 확인 | 결과 |
|---|---|
| 기본 설정 digest | 바꾸기 전과 같음 |
| 기준 에피소드 E1~E6, R1~R4(차동 실행기) | `main` 506eff3과 행·trace·하네스 trace·기록 10/10 같음 |
| 열 추종 행렬 15회 `off` | `main`과 trace·직렬·실행기 기록 15/15 같음 |
| `gnss_log` vs `off`(행렬 15회 + GNSS 조건 18회) | 명령 열(스로틀·조향) 33/33 같음, trace 33/33 같음. 실행기 기록만 `gnss` 블록만큼 다름(설계 개정 30의 '`log_only`에서 명령 차이 0') |

### 5.2 조건별

`all` 변형(격자 융합 0.2 + GNSS `geofence`)과 `gnss_log`의 결과. 횟수는 seed 0·1·2의 범위.

| 조건 | 받아들인 fix | 튐 기각 / 주입 | 품질 미달 | 끊김 뒤 다시 시작 | course·위치 표시 | 펜스 | 끝 |
|---|---|---|---|---|---|---|---|
| `gnss_nominal` | 69~73 | 0 / 0 | 0 | 0 | 0 | 안 걸림 | `row_end` 6/6 |
| `gnss_dropout_60s` | 10~12 | 0 / 0 | 0 | 0~1 | 0 | 안 걸림 | `row_end` 6/6 |
| `gnss_jump_5m` | 66~70 | **3 / 3**(6 run 모두) | 0 | 0 | 0 | 안 걸림 | `row_end` 6/6 |
| `gnss_jump_15m` | 66~70 | **3 / 3**(6 run 모두) | 0 | 0 | 0 | 안 걸림 | `row_end` 6/6 |
| `gnss_hacc_x3` | 10 | 0 / 0 | 59~63 | 0 | 0 | 안 걸림 | `row_end` 6/6 |
| `gnss_bias_20m` | 69~73 | 0 / 0 | 0 | 0 | 0 | 안 걸림 | `row_end` 6/6 |

- 끊김 60 s 동안 GNSS 탓 정지 0건, 정확도 × 3 구간 정지 0건(PC-12 완료 기준의 '끊김 60 s 동안 GNSS 탓 정지 0건'을 이 SIM 조건에서 충족).
- 튐 기각 수 = 주입 수: 단발 튐 조건 12 run 모두 3 = 3(PC-12 완료 기준).
- `geofence` 모드에서 거짓 정지 0(33 run). 다만 이 SIM 열은 28.5 m라 펜스(120 m + 여유)에 다가가지도 않는다. 펜스가 실제로 거는 경우는 합성 시험(`test_gnss_layer.py`, `test_perception_runner.py`)만 확인했다.
- course drift(run 끝 값)는 −13.3~+0.5°, 위치 불일치는 0.7~3.1 m였다(45°, max(10 m, 3 × 정확도) 문턱보다 훨씬 작음).
- 그림 `figures/perception/fig05_gnss_sim.png`(조건별 위치 불일치, GNSS 없음 구간, 튐 주입 시각).

### 5.3 비용

GNSS 층은 1 Hz에 상수 개의 연산과 run당 최대 600쌍의 Kabsch 맞춤(경로 30 m까지, 그 뒤 15 m 창)이다. 따로 시간을 재지 않았다. 실행기 한 걸음 p99 최대(작업자 4 부하): `off` 191 ms, `gnss_log` 115 ms로 부하에 묻혀 구분되지 않는다.

## 6. 한계와 남은 일

- 가상 GNSS의 떠돎·흰 잡음은 자리값이다. 10/10 기록의 GNSS를 LiDAR odometry 참조와 맞대어 짧은 시간 흔들림과 정렬 오차를 재는 일이 남았다(좌표를 문서에 남기지 않는 방식으로).
- 실제 `/gps/fix`의 공분산 형식·값, 수신기 지연은 확인하지 않았다. 정확도를 모르면 이 층은 fix를 쓰지 않는다.
- 펜스의 열 축은 정렬 시점의 band 방향 하나다. 열이 휘면 넓은 여유만이 거짓 정지를 막는다.
- G-SIM(Gazebo)의 가상 GPS(`sensor_contract_steer.py`)는 아직 이 층과 이어 보지 않았다. 그쪽은 위도·경도(가상 원점)를 내므로 `LocalFrame` 경로를 탄다.
- 넓은 사전등록 행렬(PC-14, PC-15)의 GNSS 조건 판정은 다른 묶음의 일이다. 이 문서의 수치는 그 판정이 아니다.
