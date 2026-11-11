# 조향 평가기·출발 조건 가능성(A24)·참값 격리(A13 F 정적)

- 작성: 2026-11-09 (설계 8.3 P2 뒷부분, 작업 표지 p2_eval)
- 설계: `DESIGN_v1_KO.md` 5.3, 6.5, 6.6, 7.1, 7.2(A13, A21, A24), 7.3. 결정: `DECISIONS_KO.md`. 합격 기준: `SIM_ACCEPTANCE_KO.md`(바꾸지 않았다).
- 코드: `sim/ubts/steering/evaluate_steer.py`, `sim/ubts/steering/entry_feasibility.py`, `sim/ubts/steering/truth_isolation.py`. 시험: `sim/ubts/test_steering_evaluate.py`, `test_steering_entry_feasibility.py`, `test_steering_truth_isolation.py`.
- 모든 수치는 자리값 프로필(`front_steer_v0_placeholder`, SHA-256 `561681518c2de6eb…`)로 만든 **자리값 기반 SIM 결과**다. 현장 검증이 아니다.
- 이 문서에는 GNSS 좌표, 지도 원점, 실제 기록의 궤적 절대값이 없다. 회랑 값은 회랑 좌표(s, d, 방향 오차)와 여유뿐이다.

## 1. 무엇을 만들었나

| 파일 | 하는 일 | 설계 |
|---|---|---|
| `evaluate_steer.py` | fast_sim trace 한 개를 단일열 사전등록 지표 + 조향 지표로 평가한다. run 폴더 전체 평가, A21 평가기 사례 | 7.1, 6.5, 6.6, A21 |
| `entry_feasibility.py` | 회랑 × 출발 조건마다 이상 제어의 기구학 궤적을 만들고 이탈 판정에 걸리면 INFEASIBLE로 적는다 | 5.3, A24 |
| `truth_isolation.py` | 제어기·추정기·실행기 쪽 코드가 SIM 참값을 읽지 않는지 AST로 검사한다. 플랜트·평가기 쪽은 허용 목록 | 2.6, A13(F 정적) |

- 단일열 평가 코드는 고치지 않았다. `closed_loop/raster_corridor.py`의 사전등록 도구(`_stats`, `_runs`, `stop_category`, `time_cap_s`, `ROW_END_WINDOWS`, 정지·이탈·시간 상수, 셀 분류, `CLEARANCE_GROUPS`)를 import해서 쓴다. 이 모듈은 import 때 scipy가 필요 없어서 Python 3.10에서도 돈다.
- `closed_loop/single_row_eval.py`의 거리 가중 백분위(`_weighted_percentile`)는 같은 식을 `evaluate_steer.weighted_percentile`로 옮겼다. 그 모듈은 하네스(→ scipy)를 import해서 3.10에서는 import할 수 없기 때문이다(결정 R10). 시험이 두 함수의 값이 같은지 비교한다(scipy가 되는 3.12·3.13에서 실행, 3.10에서는 이유와 함께 건너뜀).

## 2. 평가기 (`evaluate_steer.py`)

### 2.1 입력과 좌표

- 입력: fast_sim trace(`ubts.steer_sim.fast_sim_trace.v1`)의 칸. 필수 칸은 `t_s`, `truth_x_m`, `truth_y_m`, `truth_yaw_rad`, `truth_v_m_s`, `truth_delta_left_rad`, `truth_delta_right_rad`다. 나머지(`cmd_steer_target_mrad`, `steer_target_rad`, `steer_measured_rad`, `drive_request_pct`, `cmd_throttle_pct`, `drive_applied_pct`, `cmd_stop_mode`, `truth_distance_m` …)가 없으면 해당 규칙을 "평가 안 함"으로 적는다.
- 세계: W2는 래스터 회랑 세계(`raster_corridor`, 카탈로그 중심선의 곡선 좌표), W0은 평지(기둥만 있음). 세계가 없거나 W0이면 세계 좌표가 곧 field다(s = x, d = y). W2 trace를 세계 없이 넣으면 거부한다(`EvaluationError`).
- 점 투영: 뒤차축, 앞범퍼, 발자국 꼭짓점을 세계 좌표로 바꾼 뒤 회랑 좌표로 투영한다(`CurvilinearFrame.project_many`). 그래서 휜 중심선에서도 정확하다.
- 표본 격자: 사전등록 지표는 20 Hz 표본(t가 0.05 s의 배수, 하네스 제어 틱과 같다)으로 계산한다. REVERSE, 바퀴 한계, `STEER_AFTER_TERMINAL`, 정지 반응은 trace의 모든 행(100 Hz)으로 계산한다.
  - 2026-11-09 고침(리뷰 R11): trace 행이 20 Hz 격자를 주지 못하면(`trace_every`가 1이나 5가 아니면 간격이 10 Hz, 6.7 Hz, 5 Hz, 2.9 Hz …로 바뀐다) `EvaluationError`로 거부한다. `require_grid=False`면 굵은 격자로 평가한다. 결과에 실제 간격(`grid_dt_s`), 요청 간격(`grid_dt_requested_s`), 일치 여부(`grid_matches`)를 적는다. 고치기 전에는 바뀐 격자로 계산하고 0.05 s라고 적었다.
- 발자국: `geometry.SteeringGeometry.footprint_polygons(δ_L, δ_R)` = 차체 사각형 + 실제(참) 바퀴각의 두 앞바퀴 + 뒷바퀴, 브래킷 여유 포함. 자리값 프로필에서 차체 사각형은 사전등록의 1.22 × 0.70 m 사각형과 같다. 바퀴각은 캐시를 위해 1e-5 rad로 반올림한다(꼭짓점 변화 1e-6 m 미만).

### 2.2 지표 (설계 7.1)

| 지표 | 정의 | 출력 키 |
|---|---|---|
| 횡오차 | base의 d. 열 안 창(첫 움직임 ~ 종결 정지, s ∈ [0, s_E])에서 RMSE, \|d\| p95, 최대. 뒤차축·앞범퍼 d도 같은 통계. 거리 가중 RMSE·p95(보고) | `lateral_m.base / rear_axle / front_bumper / base_distance_weighted` |
| 방향 오차 | ψ = wrap(yaw − 가장 가까운 선분의 접선). RMSE, 최대 | `heading_deg` |
| 완주·정지 위치 | e = 정지 때 앞범퍼 s − s_E. FALSE(e < −2) / EARLY / OK(−1 ~ +1) / LATE / MISSED. 넘어감 = max(0, 앞범퍼 s 최대 − s_E) | `progress`, `stop_position_e_m`, `row_end_outcome`, `overshoot_m` |
| 조향 포화 | 구동 중(전진 구동 명령) 표본 가운데 \|δ_cmd\| ≥ 0.95 × 그쪽 등가 소프트 한계인 비율, 가장 긴 포화 시간. 실제 조향 속도가 속도 한계(주행 중 또는 정지 중)의 0.95배 이상인 비율. 서보 목표가 주행 중 속도 한계보다 빨리 바뀐 비율 | `steering.saturation` |
| 조향 활동량 | 명령 조향의 거리당 총변화(°/m), 10 m당 부호 변화(히스테리시스 ± 서보 데드밴드 0.5°), 명령·실제 조향 속도 p95. 실제 조향의 총변화도 보고 | `steering.activity` |
| 조향 추종 | \|서보 목표 − 측정각\| p95, \|측정각 − 참 바퀴 쪽 등가각\| p95(SIM 전용) | `steering.tracking` |
| 바퀴 한계(A21) | 참 바퀴각이 바퀴별·방향별 기계 한계를 넘은 행 수, 최대 초과. 바퀴가 차체 반폭 밖으로 나온 최대 거리(브래킷 여유 포함) | `steering.wheel_limits`, `steering.sweep` |
| 접촉 | 스윕 발자국과 세계: 래스터 셀(셀 중심 부호 거리 − res/√2, 사전등록 계산)이나 W0 기둥(정확한 원). 충돌·주행 중 접촉·정지 중 접촉·가지·미관측 겹침, 최소 여유. 차체만의 값과 "바퀴 스윕 때문에만 생긴 충돌 표본"도. 표본마다 닿는 거리는 2 m까지 본다 | `contacts` |
| 정지 여유(설계 7.1, 사전등록 `obstacle_wall_clearance`) | 종결 정지 표본(정지 시작 ~ 끝, 20 Hz)에서 스윕 발자국과 ADDED_HARD 셀(장면의 장애물 상자·벽)의 최소 여유. 6 m 도달(`raster_corridor.AT_REST_REACH_M`)로 본다. 2 m 접촉 도달로는 열 끝 정지 앞 3 m 벽이 "없음"이 된다(단일열 1차 개발에서 고친 결함과 같다). 몸체만의 값도 낸다. 래스터 세계가 아니거나 종결 정지가 없거나 6 m 안에 추가 셀이 없으면 None. 2026-11-09 더함(리뷰 R3) | `obstacle_wall_clearance_at_rest_m`, `obstacle_wall_clearance_at_rest_body_m`, `obstacle_wall_reach_m` |
| 안전 사건 | 아래 2.3절 규칙의 결과 | `fail_codes`, `forbidden` |
| 정지 반응 | 정지 계기(정지 모드 시작, 또는 구동 명령이 0으로 떨어짐)마다 구동 출력 0까지, 정지까지 시간, 거리, 계기 때 방향 기준 옆 벗어남, 정지 동안 참 조향 변화. 모드별 묶음 | `stop_response` |
| 정지 분류 | 종결 이유 → 분류. 등록된 표(`raster_corridor.stop_category`) 다음에 조향 새 이유 표(제안) | `stop_category`, `stop_table_source` |
| 정지 직전 속도 비(보고, 사전등록 `report_only`) | 종결 정지 앞에서 구동 명령이 0이 된 첫 표본의 속도 ÷ 순항(s_start + 5 m ~ s_정지 − 5 m, 움직임 중) 속도 중앙값. 정지·구동 칸·순항이 없으면 None. 2026-11-09 더함(리뷰 R3) | `report_only.pre_stop_speed_ratio` |

- 회랑 지표(횡·방향 오차, 안전 띠)는 회랑이 있을 때만 뜻이 있다. W2 세계이거나 W0의 폐루프 운전자(직선 y = 0)일 때 계산한다. W0의 개방 루프 대본(원, 사인, 정지 시험)은 "해당 없음"으로 적고 실패 코드를 내지 않는다(`evaluate_run`이 정한다).
- "끊김 동안 옆 벗어남"(7.1)은 통신 끊김 표지가 trace에 생기는 P4부터 계산할 수 있다. 지금은 정지 반응의 옆 벗어남(정지 계기 → 정지)만 있다.

### 2.3 규칙

| 규칙 | 정의 |
|---|---|
| 종료(사전등록, 사후 적용) | 표본 순서대로 COLLISION(스윕 발자국의 hard 여유 ≤ 0), MISSED_ROW_END(앞범퍼 s > s_E + 2.5), CORRIDOR_DEPARTURE(아래), LEFT_FIELD(s < −2.0), s > s_E + 6.0이면 MISSED, TIME_CAP(120 s + (s_E − s_start + 2.5)/0.075), NO_PROGRESS(60 s 동안 0.5 m 미만). 없으면 마지막 1 s 이상 정지로 TERMINAL_STOP, 없으면 OPEN. 입력이 없는 규칙(s_E, 간격, 세계)은 건너뛰고 `rules_evaluated`에 적는다 |
| 안전 띠 — 사전등록 형태 | \|d_base\| ≤ band. band = 3.7/2 − 0.60(수관) − 반폭 − 0.30(여유). 반폭은 프로필 `footprint_width_m`/2라서 자리값에서 0.60 m(사전등록과 같다). 첫 움직임 ~ 정지의 모든 표본 |
| 안전 띠 — 조향 형태(설계 7.1) | 스윕 발자국이 회랑 법선 방향으로 띠 끝 0.95 m(= 3.7/2 − 0.60 − 0.30)를 넘지 않는다. 방향 0, 바퀴가 차체 안이면 사전등록 형태와 같다. 방향 오차와 큰 조향의 바퀴 스윕이 들어간다. 두 형태를 모두 내고, 어느 것으로 판정할지는 PREREG_STEER에서 정한다 |
| 이탈 판정(설계 7.1) | 스윕 발자국이 열 선에 닿음: 발자국 최대 d > 간격(s)/2 또는 최소 d < −간격(s)/2. 간격 = 카탈로그 프로필의 왼쪽 열 d − 오른쪽 열 d(base의 s에서 보간). 사전등록 몸체 형태(\|d_base\| > 간격/2 − 반폭)도 보고 |
| REVERSE | 구동 명령 < 0인 행이 하나라도 있으면(설계 7.1: PIVOT_OR_REVERSE → REVERSE) |
| AUTO_RESTART(구동 기준) | 첫 움직임 뒤 차가 서 있고 구동 명령이 0인 상태가 0.5 s 이상 이어진 다음, 구동 명령이 0이 아니게 되면. 정지 중 조향 명령은 재출발로 세지 않는다 |
| STEER_AFTER_TERMINAL(설계 6.5, A22 "종결 뒤 명령 조향 이동 0") | 종결 시점(호출자가 준 `terminal_t_s`, 없으면 첫 움직임 뒤의 STOP 명령(정지 모드) 시작마다, 그것도 없으면 마지막 정지 시작) 행 i부터 구동이 다시 시작되거나 trace가 끝날 때까지 본다. 명령 조향은 서보 목표다. 허용 오차는 max(1°, 2 × 서보 데드밴드). 사건은 둘 중 하나다. (a) 종결 뒤 명령이 움직임: \|목표[k] − 목표[i]\| > 허용. (b) 정지 자체가 명령을 옮김: \|목표[i] − 목표[i−1]\| > 허용이고, 새 목표가 정지 순간의 조향 위치가 아님(\|목표[i] − 위치[i−1]\| > 허용). 위치는 trace의 `truth_servo_view_rad`(플랜트의 hold_measured가 잡는 서보 쪽 각, 정확)다. 그 칸이 없는 옛 trace는 가변저항 해석값(`steer_measured_rad`), 그것도 없으면 직전 명령을 쓴다(창마다 `hold_position_source`). 그래서 hold_measured가 목표를 지금 위치로 옮기는 것은 허용하고, center 정책(목표 0)은 사건이다. 가변저항 해석값은 (a)의 기준으로 쓰지 않는다. `auto_centre`(A22): (b) 가운데 0 쪽 이동, 또는 종결 뒤 정지 중 명령이 0 쪽으로 움직임. 보고만(A17 입력, 판정은 P4): 종결 뒤 참 바퀴 쪽 등가각의 표류(`true_drift_deg`)와 허용 초과 여부 |
| WHEEL_LIMIT(A21) | 참 바퀴각이 기계 한계를 넘은 행이 있으면 |
| 클래스 한계 | `class_limits`를 주면 LATERAL_RMSE/P95/MAX, HEADING_RMSE/MAX를 낸다. 값은 PREREG_STEER에서 정한다(기본 없음) |

- `STEER_AFTER_TERMINAL` 2026-11-09 고침(리뷰 R1, 치명): 처음 규칙은 "종결 직전 한 표본의 가변저항 해석값"(`steer_measured_rad`)도 기준으로 썼고 `auto_centre`도 그 값을 기준으로 했다. 기본 프로필 L1에서는 비비례 ADC의 급전 처짐(구동 duty에 비례)·잡음·양자화·서보 데드밴드 때문에 명령이 그대로여도 그 값이 서보 목표와 1° 넘게 달라서, 명령 조향이 전혀 움직이지 않은 정지에서 사건과 `auto_centre`가 거짓으로 떴다. 같은 행렬(L1·L0 × hold_target·hold_measured·center × +6°/−8° × duty 100/30 % × seed 100~107, 창 192개)을 두 규칙으로 평가했다(`runs/wf2b/fix_wf2b/terminal_rule_before_after.json`, 네이티브 MinGW 빌드):

| 창 | 첫 규칙 사건 | 고친 규칙 사건 |
|---|---|---|
| L1 hold_target·hold_measured(명령 그대로) 64개 | 8 (hold_target +6° 100 % 2, 30 % 2; hold_measured +6° 100 % 4) | 0 |
| L0 hold 64개 | 0 | 0 |
| center(정지가 목표를 0으로 옮김) L1 32개, L0 32개 | 64(모두 `auto_centre`) | 64(모두 `auto_centre`) |

  시험: 합성 trace(처짐 있는 해석값, hold_measured 지연, center, 정지 뒤 새 명령, 참 표류)와 L1 brake 정지 회귀 시험(`test_l1_brake_stops_without_a_steering_move_are_not_events`, seed 100~107)을 더했다.
- 정지 분류 제안 표(`PROPOSED_STEERING_STOP_TABLE`): 설계 6.6의 새 이유를 다음처럼 둔다. **제안이다. PREREG_STEER가 첫 평가 전에 정하고, 나중에 더 유리한 분류로 바꾸지 않는다.** 등록된 표에 있는 이유(`steering_authority_insufficient` → CONTROL_AUTHORITY)는 등록된 표가 이긴다.

| 이유 | 제안 분류 | 이유 |
|---|---|---|
| `steering_feedback_invalid`, `steering_feedback_stale`, `steering_not_following`, `steering_stalled`, `steering_ineffective`, `steering_uncalibrated`, `steering_polarity_fault`, `steering_driver_fault`, `steering_authority_insufficient` | CONTROL_AUTHORITY | 조향 구동 사슬 결함. 등록된 DRIVE 부표지(구동 무응답 → CONTROL_AUTHORITY)와 같은 쪽 |
| `stop_unconfirmed` | SYSTEM_FAULT | 확인되지 않은 STOP은 안전 장치 결함 |

## 3. A21 평가기 부분 — 발자국 검사가 조향각별 스윕을 반영하는지

방법(`evaluate_steer.a21_evaluator_case`):

1. W0, L0 서보, 정지 상태 조향 대본: 0.5 s에 목표 0 → 30°(플랜트가 등가 소프트 한계 23.46°로 자른다). "바퀴를 꺾은 채 줄기 옆에서 출발"하는 경우다(설계 7.1).
2. 그 trace에서 바퀴각이 가장 큰 순간, 바퀴(브래킷 여유 없는 맨 바퀴)의 가장 바깥 모서리에서 0.004 m 안쪽에 반경 0.01 m 기둥을 놓는다. 기둥은 차체 사각형 밖이다. 바퀴가 실제로 기둥에 닿는다.
3. 그 기둥이 있는 세계로 같은 trace를 평가한다.

결과(Windows 3.13.2, `runs/wf2b/p2_eval/a21_evaluator/A21_EVALUATOR.json`):

| 쪽 | 최대 바퀴각(왼/오른, °) | 차체만 최소 여유 | 스윕 최소 여유 | 브래킷 여유 없는 스윕 | 바퀴 한계 위반 | 판정 |
|---|---|---|---|---|---|---|
| 왼쪽 조향 | 25.81 / 22.87 | +0.026 m | −0.025 m | −0.012 m | 0 | 통과 |
| 오른쪽 조향 | 21.39 / 23.97 | +0.023 m | −0.024 m | −0.012 m | 0 | 통과 |

- 차체만 보는 검사는 통과하고, 스윕을 넣은 검사는 접촉을 낸다. 바퀴는 기계 한계(28°) 안이다(바퀴 각에 0점 오차 표본이 더해져 25.8°).
- 바퀴가 차체 반폭 밖으로 나온 최대 거리는 0.054~0.060 m(맨 바퀴 약 0.034~0.040 m + 브래킷 여유 0.02 m)다.
- 같은 내용을 합성 trace로도 시험한다(래스터 셀, W0 기둥, 양쪽).
- 주의: 일정한 조향으로 계속 돌면 바퀴 모서리가 지나는 곳은 차체가 쓸고 지나가는 고리 안에 있다. 스윕 발자국이 결과를 바꾸는 것은 정지 중 조향, 출발 보정, 조향을 바꾸는 순간 같은 과도 상태다.

## 4. A24 — 출발 조건 가능성

### 4.1 방법(`entry_feasibility.py`)

- 이상 제어("최대 조향 속도로 즉시 최대 조향", A24)를 뱅뱅 두 단계로 정했다.
  - 돌기: 출발 바퀴각 δ0에서 주행 중 조향 속도 한계(`steer_rate_max_deg_s`, 30°/s)로 σ × 등가 소프트 한계(23.46°)까지 가고 유지한다.
  - 펴기: 지금 바로 펴면(δ → 0, 같은 속도) 회랑과 나란히 끝나는 순간에 펴기 시작한다. 예측 최종 방향 오차 = ψ + sign(δ) · v/(L r) · (−ln cos δ). 0이 되는 순간이다. δ가 0이 되면 창이 끝난다.
  - σ(돌 방향): 출발 때 예측값 P0의 반대쪽이 교정 방향이다(σ·P0 < 0). \|P0\| ≤ 0.5°이면 두 방향 모두 교정 방향이다. 교정 방향 가운데 이탈 여유가 큰 쪽을 이상 제어로 본다.
  - 방향 오차의 기준은 중심선의 현(s − 1 m에서 s + 1 m로의 방향)이다. 카탈로그 중심선(0.25 m 선분)에 선분 사이 1.5°까지의 꺾임이 있어 한 선분과 나란한 것은 회랑과 나란한 것이 아니기 때문이다.
- 차량: 기구학 자전거(뒤차축 기준, 미끄럼 0, 효율 1, 언더스티어 0, 떠돎 0, 구동기 지연 0, 0점 오차 0). 속도 0.4 m/s(설계 6.3 목표)로 t = 0부터 달린다. 정지 중 미리 조향은 하지 않는다. 기구학 모델이라 결과는 "조향 속도 / 차속"(30°/s, 0.4 m/s에서 75°/m)에만 달려 있다. 0.01 s마다 정확한 원호로 적분한다.
- 출발 자세: 단일열 사전등록 start_pose(base s = 1.0 m, d = d0, 방향 = 접선 + ψ0), 바퀴 δ0.
- 이탈 판정: 2.3절의 조향 형태(스윕 발자국이 열 선에 닿음). 창의 모든 0.01 s 표본과 출발 자세에서 본다. 여유 < 0이면 INFEASIBLE.
- 보고만(판정 아님): 사전등록 몸체 형태의 여유, 이상 궤적의 스윕 발자국과 hard 셀의 여유(20 Hz, 10/10 지도 필요), 조향 속도 10°/s(`steer_rate_slow_10`, 프로필 범위 하한)에서의 판정과 여유.
- 출발 조건(설계 5.3 + 기준): `nominal`, `entry_lat_p04/m04`(±0.40 m), `entry_hdg_p08/m08/p12/m12`(±8°, ±12°), `entry_div_pos/neg`(0.40 m + 8° 벌어짐), `start_wheels_turned_p10/m10`(바퀴 ±10°).

### 4.2 회랑

- 개발 회랑만 돌렸다(설계 7.3): 동결 8방향(단일열 `CORRIDOR_FREEZE.json`) + 카탈로그(`external_paths` 'corridor_catalogue', SHA-256 `28d25df4…`)의 나머지 포함 방향 = **32방향**(동결 8, 포함 24).
- **돌리지 않은 방향(5개)**: 봉인 3방향(042301 c01·c04·c07 fwd, 단일열 개정 A1)과 봉인 열 쌍의 역방향 2개(042301 c01·c04 rev; 설계 7.3 "같은 열 쌍 역방향은 개발 근거로 쓰지 않는다"; c07 rev는 카탈로그에서 제외 방향이다). **이 5방향의 A24 목록은 조향 사전등록 단계로 미룬다.** 코드가 이 방향을 시뮬레이션하지 않고, `--only`로 줘도 거부한다. 규칙은 `scene_spec.sealed_status()` 한 곳이고, 장면 명세도 같은 방향을 `allow_sealed` 없이 거부한다(2026-11-09, 리뷰 R10).

### 4.3 결과

실행: Windows 3.13.2, numpy 2.2.6, 작업자 6개, 59.6 s. 출력 `runs/wf2b/p2_eval/a24_dev/A24_ENTRY_FEASIBILITY.json`, `A24_TABLE.txt`.

- **32방향 × 11조건 = 352개 모두 FEASIBLE.** 조향 속도 10°/s에서도 모두 FEASIBLE.
- 모든 행에서 교정 방향이 있었다. 창 끝의 방향 오차(현 기준)는 0.9° 이하다. 남는 값은 0.5~0.75 m 창 동안 중심선이 휘어서 생긴다(주로 051127 회랑).
- 조건별 최소 이탈 여유(모두 040927 c04 rev, 출발점 간격 2.36 m에서):

| 조건 | 최소 여유(30°/s) | 최소 여유(10°/s) | 사전등록 몸체 형태 최소 여유 |
|---|---|---|---|
| nominal | 0.83 m | 0.83 m | 0.83 m |
| entry_lat_p04 / m04 | 0.43 / 0.43 m | 0.43 / 0.43 m | 0.43 / 0.43 m |
| entry_hdg_p08 / m08 | 0.74 / 0.74 m | 0.72 / 0.72 m | 0.79 / 0.79 m |
| entry_hdg_p12 / m12 | 0.69 / 0.69 m | 0.66 / 0.66 m | 0.79 / 0.79 m |
| entry_div_pos / neg | 0.34 / 0.34 m | 0.32 / 0.32 m | 0.41 / 0.41 m |
| start_wheels_turned_p10 / m10 | 0.80 / 0.80 m | 0.75 / 0.75 m | 0.82 / 0.82 m |

- 이상 궤적이 hard 셀(수관·줄기 래스터)에 닿은 경우(보고, 판정 아님):

| 회랑 | 조건 | 30°/s | 10°/s | 출발 자세부터 닿음 |
|---|---|---|---|---|
| 040927:c04:rev | entry_div_pos | −0.121 m | −0.170 m | 아니오 |
| 051127:c06:fwd | entry_lat_p04 | −0.097 m | −0.097 m | 예 |
| 051127:c06:fwd | entry_div_pos | −0.100 m | −0.119 m | 예 |
| 051127:c03:fwd | entry_div_neg | 닿지 않음 | −0.063 m | 아니오 |
| 051127:c11:fwd | entry_div_pos | 닿지 않음 | −0.001 m | 아니오 |

  - 이 회랑들은 동결 8방향이 아니다. 동결 8방향은 출발 자세 검사 V2(|d| ≤ 0.95 m 안에 점유·미관측 셀 없음)를 거쳤다.
  - 열 중심선(이탈 판정 기준)보다 수관이 안쪽에 있어서, 이탈 판정은 통과하지만 이상 제어로도 수관에 닿는 출발이 있다. 이것을 INFEASIBLE에 넣을지는 PREREG_STEER에서 정한다(아래 6절 제안).

표: 이상 궤적의 이탈 여유(m). **INF** 표시가 있으면 INFEASIBLE이다(이번에는 없다). `hdg`는 방향, `lat`은 옆 위치, `div`는 벌어짐, `wheels`는 바퀴 꺾임 출발이다.

| corridor | role | spacing at start (m) | nominal | lat_p04 | lat_m04 | hdg_p08 | hdg_m08 | hdg_p12 | hdg_m12 | div_pos | div_neg | wheels_p10 | wheels_m10 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 035952:c02:fwd | included | 3.80 | 1.55 | 1.15 | 1.15 | 1.47 | 1.47 | 1.42 | 1.42 | 1.07 | 1.07 | 1.52 | 1.52 |
| 035952:c03:fwd | included | 3.73 | 1.52 | 1.12 | 1.12 | 1.42 | 1.42 | 1.38 | 1.38 | 1.02 | 1.02 | 1.48 | 1.49 |
| 035952:c03:rev | included | 3.63 | 1.46 | 1.07 | 1.06 | 1.38 | 1.38 | 1.33 | 1.33 | 0.98 | 0.98 | 1.44 | 1.44 |
| 035952:c04:fwd | included | 3.75 | 1.53 | 1.13 | 1.13 | 1.44 | 1.44 | 1.40 | 1.40 | 1.04 | 1.04 | 1.50 | 1.50 |
| 035952:c04:rev | frozen | 4.07 | 1.68 | 1.28 | 1.28 | 1.59 | 1.59 | 1.54 | 1.54 | 1.19 | 1.19 | 1.65 | 1.65 |
| 035952:c05:fwd | included | 3.85 | 1.57 | 1.17 | 1.17 | 1.49 | 1.49 | 1.45 | 1.45 | 1.09 | 1.09 | 1.55 | 1.55 |
| 035952:c05:rev | frozen | 3.90 | 1.60 | 1.20 | 1.20 | 1.49 | 1.49 | 1.45 | 1.45 | 1.09 | 1.09 | 1.55 | 1.55 |
| 040927:c01:rev | included | 3.68 | 1.49 | 1.09 | 1.09 | 1.40 | 1.40 | 1.36 | 1.36 | 1.00 | 1.00 | 1.46 | 1.46 |
| 040927:c02:fwd | frozen | 3.71 | 1.50 | 1.11 | 1.10 | 1.42 | 1.41 | 1.38 | 1.37 | 1.02 | 1.01 | 1.48 | 1.48 |
| 040927:c02:rev | included | 4.12 | 1.70 | 1.31 | 1.30 | 1.63 | 1.62 | 1.59 | 1.58 | 1.24 | 1.22 | 1.68 | 1.68 |
| 040927:c03:rev | included | 3.38 | 1.34 | 0.94 | 0.94 | 1.19 | 1.20 | 1.15 | 1.16 | 0.79 | 0.80 | 1.25 | 1.27 |
| 040927:c04:rev | included | 2.36 | 0.83 | 0.43 | 0.43 | 0.74 | 0.74 | 0.69 | 0.69 | 0.34 | 0.34 | 0.80 | 0.80 |
| 040927:c05:rev | included | 4.05 | 1.67 | 1.27 | 1.27 | 1.59 | 1.59 | 1.55 | 1.55 | 1.19 | 1.19 | 1.65 | 1.65 |
| 040927:c06:fwd | included | 3.93 | 1.62 | 1.22 | 1.22 | 1.51 | 1.51 | 1.47 | 1.47 | 1.11 | 1.11 | 1.57 | 1.57 |
| 040927:c06:rev | frozen | 4.05 | 1.68 | 1.28 | 1.28 | 1.59 | 1.59 | 1.54 | 1.54 | 1.19 | 1.19 | 1.65 | 1.65 |
| 040927:c08:fwd | frozen | 3.65 | 1.48 | 1.08 | 1.08 | 1.39 | 1.39 | 1.35 | 1.35 | 0.99 | 0.99 | 1.45 | 1.45 |
| 042301:c02:fwd | frozen | 3.93 | 1.62 | 1.22 | 1.22 | 1.52 | 1.52 | 1.47 | 1.48 | 1.12 | 1.12 | 1.58 | 1.58 |
| 042301:c02:rev | included | 3.99 | 1.64 | 1.24 | 1.24 | 1.56 | 1.56 | 1.52 | 1.51 | 1.16 | 1.16 | 1.62 | 1.62 |
| 042301:c03:fwd | included | 3.80 | 1.55 | 1.15 | 1.15 | 1.46 | 1.46 | 1.41 | 1.41 | 1.06 | 1.06 | 1.52 | 1.51 |
| 042301:c03:rev | included | 3.94 | 1.62 | 1.22 | 1.22 | 1.54 | 1.53 | 1.49 | 1.49 | 1.14 | 1.13 | 1.60 | 1.59 |
| 042301:c05:fwd | frozen | 3.72 | 1.51 | 1.11 | 1.11 | 1.42 | 1.42 | 1.38 | 1.38 | 1.02 | 1.02 | 1.48 | 1.48 |
| 042301:c05:rev | included | 3.76 | 1.53 | 1.13 | 1.13 | 1.45 | 1.45 | 1.41 | 1.40 | 1.05 | 1.05 | 1.51 | 1.51 |
| 045553:c02:rev | included | 3.47 | 1.38 | 0.98 | 0.98 | 1.30 | 1.30 | 1.26 | 1.26 | 0.90 | 0.90 | 1.36 | 1.36 |
| 045553:c03:fwd | included | 3.41 | 1.35 | 0.96 | 0.95 | 1.27 | 1.26 | 1.23 | 1.21 | 0.88 | 0.86 | 1.33 | 1.32 |
| 045553:c04:fwd | frozen | 3.71 | 1.50 | 1.10 | 1.11 | 1.42 | 1.42 | 1.37 | 1.37 | 1.02 | 1.02 | 1.48 | 1.48 |
| 051127:c03:fwd | included | 3.93 | 1.58 | 1.19 | 1.18 | 1.50 | 1.48 | 1.46 | 1.44 | 1.10 | 1.08 | 1.58 | 1.53 |
| 051127:c03:rev | included | 3.93 | 1.60 | 1.20 | 1.22 | 1.53 | 1.52 | 1.50 | 1.48 | 1.14 | 1.14 | 1.59 | 1.59 |
| 051127:c05:fwd | included | 3.52 | 1.32 | 0.93 | 0.92 | 1.18 | 1.17 | 1.12 | 1.17 | 0.79 | 0.81 | 1.22 | 1.35 |
| 051127:c05:rev | included | 3.82 | 1.53 | 1.13 | 1.14 | 1.44 | 1.47 | 1.39 | 1.44 | 1.04 | 1.07 | 1.49 | 1.53 |
| 051127:c06:fwd | included | 2.82 | 1.05 | 0.66 | 0.65 | 0.89 | 0.89 | 0.86 | 0.84 | 0.51 | 0.49 | 1.01 | 0.94 |
| 051127:c07:fwd | included | 4.50 | 1.90 | 1.50 | 1.50 | 1.82 | 1.82 | 1.78 | 1.78 | 1.42 | 1.42 | 1.88 | 1.88 |
| 051127:c11:fwd | included | 3.36 | 1.33 | 0.93 | 0.93 | 1.25 | 1.24 | 1.21 | 1.20 | 0.85 | 0.84 | 1.30 | 1.30 |

### 4.4 한계

- 이탈 판정의 기준은 열 중심선(줄기 선)이다. 수관은 그보다 안쪽에 있으므로 이탈 판정은 너그럽다. 그래서 hard 셀 접촉을 따로 보고했다.
- 이상 제어는 정지 중 미리 조향을 하지 않는다. 미리 조향(설계 6.5 출발 전 정렬)을 허용하면 여유가 더 커질 수 있다. 이번 결과는 모두 FEASIBLE이라 판정에는 영향이 없다.
- 기구학 모델이다. 미끄럼, 효율, 언더스티어, 구동기 지연, 0점 오차, 백래시가 없다. 실제 제어기가 이 여유를 다 쓰지는 못한다.
- 10/10 지도는 SLAM 재구성이다(설계 5.1). "SLAM이 본 과수원"의 결과다.
- 봉인 방향과 그 역방향 5개는 계산하지 않았다.

## 5. A13 — 참값 격리, F 정적 검사 (`truth_isolation.py`)

### 5.1 범위와 역할

검사 범위(저장소 기준 경로): `sim/ubts/steering/**`, `sim/ubts/closed_loop/**`, `sim/ubts/orchard_perception/scan_corridor.py`, `sim/ubts/field_response.py`, `sim/ubts/cpu_lidar.py`, 조향·폐루프 시험, `tools/row_executor/**`(P4), `tools/field_calib/**`(P4 식별 분석: 기록만 읽는 라이브러리, 역할 shared, 시험은 test; 검토 P4R-13), `dashboard/scripts/steer_*.py`, `dashboard/scripts/esp_motor_v3.py`(P3). 나머지(차동 SIM 스택)는 범위 밖이다. ROS 그래프 검사(A13 G)는 P6a다.

- 면제(`SINK_EXEMPTIONS`, 검토 P4R-01): 플랜트 쪽 싱크 발견은 정확한 (경로, 함수, detail) 키로만 면제한다. (경로, 함수) 두 칸 키는 없앴다(그 함수 안의 새 누출을 숨겼다). 키 하나는 발견 하나만 면제한다. `check_tree()` 결과에 면제된 발견 목록(`exempted`, 이유 포함)과 수(`exempted_count`), 아무것도 맞지 않는 키(`unused_exemptions`)가 있다. 지금 면제 12건: `sim_in_loop.py` 8건(차동 이력 하네스: 플랜트 이름이 고르는 SIM 보정값 3건, exec_field의 조작자 측정 열 길이 `true_row_length`를 받는 `ExecutorInTheLoop(...)` 1건, 결과 기록 `result["executor"]` 4건), `single_row_sim.py` 2건, `gz_probe.py` 1건(WF-3b), `core.single_row_module`의 동적 import 1건.

| 역할 | 뜻 | 파일(현재) |
|---|---|---|
| decision | 제어기·추정기·실행기·호스트 브리지. 참값 0 | `closed_loop/estimator.py`, `controllers.py`, `single_row.py`, `orchard_perception/scan_corridor.py`, (P8) `closed_loop/steer_control.py`, (P4) `tools/row_executor/*`, (P3) `dashboard/scripts/steer_*.py`, `esp_motor_v3.py` |
| shared | 양쪽이 쓰는 라이브러리. 참값 0 | `steering/__init__.py`, `vehicle_profile.py`, `geometry.py`, `drive.py`, `host_profile_export.py`(P3), `closed_loop/__init__.py`, `external_paths.py` |
| plant (허용) | 플랜트, 구동기 백엔드, 세계, 센서 합성, 실행기(runner), 하네스 고리, 장면 명세 | `steered_plant.py`, `l0_servo.py`, `actuator_link.py`, `native/*`, `fast_sim.py`, `flat_world.py`, `scene_spec.py`, `gz_*.py`, `closed_loop/harness.py`, `world.py`, `raster_corridor.py`, `raster_catalogue.py`, `field_response.py`, `cpu_lidar.py` (+ P4·P6a·P7의 `sim_in_loop*`, `scan_unobserved_clip.py`, `procedural_orchard.py`) |
| evaluator (허용) | 참값으로 지표·합격 검사 | `evaluate_steer.py`, `entry_feasibility.py`, `closed_loop/evaluate.py`, `single_row_eval.py`, `matrix.py` |
| truth_fed_driver (허용) | 참값을 설계상 읽는 SIM 시험 운전자 | `maneuvers.py`(국면 시간을 참 속도·방향 변화로 잰다) |
| contract / tool / test (허용) | 참값 계약 정의(`backend_api.py`), 이 검사기, 시험·시험 자료 | |

목록에 없는 범위 안 파일이 참값을 읽으면 오류(`unclassified_truth_read`)다. 새 파일은 역할을 정해 목록에 넣어야 한다.

### 5.2 규칙

decision·shared 코드에서 다음은 모두 오류다. 주석과 docstring은 코드가 아니어서 보지 않는다.

| 규칙 | 잡는 것 |
|---|---|
| truth_field | `backend_api.TRUTH_OUTPUT_FIELDS`(참 바퀴각, 모터 쪽 각, `applied_steer_pwm`, 모델 표지) 속성이나 문자열 |
| truth_key | `truth_*` 문자열(fast_sim trace 참값 칸), `lane_centers_y` 같은 월드 정답 |
| truth_contract | `TRUTH_OUTPUT_FIELDS`, `ModelFlag`, `MODEL_FLAGS`, `MODEL_FLAG_NAMES` |
| truth_method | `.truth()`, `.rear_axle_pose()`, `.run_info()`, `.servo_view_rad()`, `.lidar_pose()`, `.corridor_truth()`, `.pose_in_field()`, `.to_field()`, `.clearance()`, `.clearances()`, `.class_at()` 호출 |
| plant_reference | 이름·속성에 `plant`가 든 것(플랜트 객체와 그 x·y·yaw·v·w·distance_m 등 모든 상태), `['plant']` 첨자. 대문자 상수 이름(`PLANT_PROFILE_NAMES`)은 제외 |
| world_reference | 정확히 `world`인 이름·속성(SIM 세계: 지형, 지도, 라벨) |
| truth_topic | `/odom`, `/tf`(동적; `/tf_static`은 허용), `/joint_states`, `/ground_truth/odom`, `/sim/truth/*`, `/gps/*` |
| truth_import | 플랜트·세계·실행기(runner)·평가기·참값 운전자·장면 명세(`steering.scene_spec`: 장면의 참 출발 자세와 세계) 모듈, Gazebo 바인딩 `gz.*`, 혼합 모듈의 플랜트 이름(`field_response.FieldPlant`; 같은 모듈의 공칭 구동 곡선 `forward_curve`는 프로필 값이라 허용). `importlib.import_module('<참값 모듈>')`, `__import__('<참값 모듈>')`도 |
| dynamic_import | `importlib.import_module`·`__import__`의 이름이 문자열 상수가 아님(검사할 수 없음) |
| rig_reference | T1 장치를 가리키는 이름·속성: `rig`, `_rig`, `rig_*`, `*_rig`(대문자 상수 제외; `origin`·`right`는 아님)와 펌웨어 프로세스 손잡이 `.fw`(검토 RS-05) |
| rig_channel | T1 장치의 평가 채널 호출(LOCKSTEP 11.6): `.mcu_channel()`, `.mcu_channel_at()`, `.mcu_lines()`, `.serial_log()`, `.serial_log_bytes()`, `.serial_log_sha256()`, `.trace_arrays()`, `.trace_sha256()`, `.host_writes()` |
| scene_truth | 장면·장치의 참 출발: `.start_pose`, `.start_steer_rad` |

- 장치 세 규칙(rig_reference, rig_channel, scene_truth)은 decision·shared에만 건다. 플랜트 쪽 연결 코드(`sim_in_loop_steer.py`)는 설계대로 장치 시계와 `RigClient`를 실행기에 준다. `RigClient`는 장치를 비공개(`_rig`)로 가진다.

허용 목록 코드(plant, evaluator, truth_fed_driver)에는 "싱크 규칙"을 건다. 참값과 결정 쪽이 만나는 곳(예: 하네스 고리)을 보는 규칙이다.

- 함수마다 참값에서 나온 값을 오염으로 표시한다. 함수 안 대입 전체를 흐름 무관하게 반복해 고정점까지 퍼뜨린다. 오염된 값을 넣은 컨테이너(`append`, `heappush` …)도 오염된다. 안쪽 함수(클로저)는 바깥 함수의 오염 이름을 물려받는다.
- 오염된 값이 결정 쪽 객체에 닿으면 `truth_to_decision` 오류다. 닿는 길은 셋이다. (1) 받는 쪽 이름에 `estimator`, `controller`, `executor`, `bridge_client`가 든 메서드 호출(`estimator.on_scan(x)`). (2) 이 단어가 든 이름의 호출: 생성자·팩토리(`SensorEstimator(x)`, `controller_factory(x)`). `world_factory`, `plant_factory` 같은 다른 팩토리는 플랜트 쪽이라 싱크가 아니다. (3) 그런 객체의 속성·첨자에 대입(`estimator.pose = x`, `controller.hints['end'] = x`).
- 상태 호출은 결과를 깨끗하게 한다: `.imu_sample()`, `.steer_status()`, `.status()`, `.snapshot()`. 센서 합성 호출 `.sample()`, `.scan()`은 결과를 깨끗하게 하지 않는다. ScanFrame과 `scan()` 결과에는 참 센서 위치(`origin`)와 월드 방향(`directions`)도 들어 있기 때문이다. LaserScan 칸(`.ranges`, `.angle_min`, `.angle_increment` …)으로 읽을 때만 센서 자료다. `ranges, origin, directions = caster.scan(...)`처럼 풀면 이 칸 이름(`ranges`)만 깨끗하다. 그래서 하네스가 참 자세로 만든 `frame.ranges`를 추정기에 주는 것은 센서 합성이라는 설계이고, 프레임 전체·`frame.origin`·`origin`을 주는 것은 참값 누출이다.
- 2026-11-09 고침(리뷰 R2): 처음 버전은 (2)·(3), `sample()`·`scan()` 결과의 origin·프레임 전체, 장면 명세 import, 문자열 동적 import를 놓쳤다. 심어 둔 13가지(검증 probe)와 리뷰 probe 11가지를 모두 잡고(`runs/wf2b/fix_wf2b/check_a13_after_fix.json`, `a13_holes_after_fix.json`), 깨끗한 사례(`frame.ranges`, 풀어낸 `ranges`, `world_factory(config.world)`)는 잡지 않는다.

### 5.3 결과

- 검토 처리 뒤(2026-11-09 fix_wf3a, `checks_final_w6`의 A13): 178개 파일, 역할 decision 30, shared 17, plant 37, evaluator 13, truth_fed_driver 2, contract 1, tool 2, test 76, 미분류 0. **오류 0, 통과.** 면제 12개가 모두 쓰였고 쓰지 않은 면제 키는 0이다. 실행기 구독은 허용 목록 안이다(/scan, /imu/data_raw, /rover/control/requested).
- 리뷰 고친 뒤(2026-11-09 fix_wf2b): 81개 파일, 역할 decision 8, shared 7, plant 20, evaluator 5, truth_fed_driver 1, contract 1, tool 1, test 38, 미분류 0. **오류 0, 통과.** 새 싱크 규칙이 지금 트리(하네스 고리, `gz_probe`·`gz_direct_drive`의 플랜트 쪽 구동기 모델 `ModelSteerController`, `_HarnessController`, `ProcessPoolExecutor`)에서 거짓 양성을 내지 않는다.
- 처음 실행(2026-11-09, WF-2a의 호스트 파일 4개와 P5의 `gz_direct_drive.py`가 생긴 뒤): 79개 파일, 역할 decision 8, shared 7, plant 20, evaluator 5, truth_fed_driver 1, contract 1, tool 1, test 36, 미분류 0. **오류 0, 통과.** 필수 decision 4개(추정기, 두 제어기, 스캔 회랑)가 모두 검사됐고 참값 읽기 0이다. WF-2a의 `steer_bridge_core.py`, `steer_control_core.py`, `steer_rover_bridge.py`, `esp_motor_v3.py`도 참값 읽기 0이다.
- 심어 둔 위반은 모두 잡힌다(시험): decision 파일의 8개 규칙, 플랜트 고리에서 추정기·제어기로 가는 참값 2건, 클로저를 거친 참값, 분류 안 된 파일의 참값 읽기. docstring·주석·센서 합성·공칭 구동 곡선·`/tf_static`·대문자 상수는 잡지 않는다.
- 작업 중 고친 것: 처음 버전은 안쪽 함수 본문을 바깥 함수에 넣어 P5의 `gz_direct_drive.py`에서 거짓 양성을 냈다(그 `controller`는 플랜트 쪽 구동기 모델 `gz_probe.ModelSteerController`다). 안쪽 함수를 따로 분석하고 클로저 오염을 물려받게 고쳤다.

### 5.4 한계

- 정적 과대 근사다. 함수 안 흐름만 본다. 다른 객체의 속성, 전역, 다른 함수를 거친 값은 따라가지 않는다. 그래서 모듈 단위 역할표가 기본 방어선이다. 통과가 실행 중 참값이 절대 새지 않는다는 증명은 아니다.
- 플랜트 쪽 Gazebo ECM 접근(`world_pose()` 등)은 싱크 규칙의 참값 원천에 넣지 않았다. G-SIM의 실행기는 ROS 노드라서 A13(G) 그래프 검사(P6a)가 맡는다. decision 코드가 `gz.*`를 import하면 오류다.
- 받는 쪽을 이름으로 판단한다. 결정 쪽 객체를 다른 이름으로 부르면 싱크 규칙이 놓친다.
- `steer_status()`는 상태 호출로 깨끗하게 본다. 그러나 hold_measured 정지 뒤 그 `target_rad`는 플랜트의 서보 관점 각(참 모터 쪽 각에서 만든 값)이다(리뷰 R9). P1·P3에서 hold_measured를 MCU 쪽 측정으로 바꾸는 것을 결정 제안으로 넘겼다. 그전까지 이 값은 trace·문서에 참값에서 만든 값으로 표시한다.

## 6. PREREG_STEER에서 정할 것(제안)

1. 안전 띠 판정 형태: 사전등록 형태(\|d_base\| ≤ 0.60)와 조향 형태(스윕 발자국 ≤ 0.95 m) 가운데 무엇으로 판정할지. 둘 다 계산한다.
2. 이탈 판정의 방향 오차 포함: 설계 7.1 문구대로 "발자국 + 앞바퀴의 회랑 법선 방향 최대 거리"를 쓰면 방향 오차가 있을 때 사전등록보다 엄격하다(12°에서 반폭 0.35 → 약 0.47 m). 이 평가기는 그렇게 계산하고, 사전등록 몸체 형태도 함께 보고한다.
3. A24 판정에 hard 셀 접촉을 넣을지: 지금 판정은 이탈 판정만 쓴다(합격 기준 문구). 이상 궤적이 수관에 닿는 출발 3건(30°/s), 5건(10°/s)이 있다. 출발 자세부터 닿는 051127 c06 fwd의 `entry_lat_p04`, `entry_div_pos`는 따로 다루는 것이 좋다.
4. 정지 분류 제안 표(2.3절)의 확정.
5. 봉인 3방향과 그 역방향 2개의 A24는 사전등록 단계에서 같은 코드로 계산한다(`--only`가 지금은 거부하므로 그때 허용 플래그를 더한다).

## 7. 실행 방법과 시험

```
cd sim/ubts
python -B -m steering.evaluate_steer run --run <fast_sim run 폴더> [--out <저장소 밖 파일>] [--only 장면 ...]
python -B -m steering.evaluate_steer a21 --out <저장소 밖 폴더>
python -B -m steering.entry_feasibility --out <저장소 밖 폴더> [--workers N] [--only record:cid:dir ...] [--no-hard-cells]
python -B -m steering.truth_isolation [--json]
```

- 외부 자료(카탈로그, 10/10 지도)는 `UBTS_STEER_PATHS`로 찾는다(`PATHS_KO.md`). 출력 폴더가 저장소 안이면 거부한다. 이 작업 트리뿐 아니라 같은 저장소의 다른 작업 트리(main checkout 포함) 안도 거부한다(`scene_spec.inside_repository`, 2026-11-09 리뷰 R12 고침. 읽는 규칙은 FAST_SIM_KO.md 4.2절).
- 시험(2026-11-09, 새 시험 42개):
  - Windows 3.13.2(numpy 2.2.6, pytest 9.1): 42 통과.
  - 기반 이미지 `6209168a3507`(Python 3.12.3, numpy 1.26.4, pytest 7.4.4): 42 통과, `truth_isolation` CLI 통과.
  - WSL 3.10.12(numpy 2.2.6, pytest 없음, `python3 -m unittest`): 42개 중 41 통과, 1 건너뜀(`single_row_eval` import에 scipy 필요, R10).
- 리뷰 고친 뒤 시험(2026-11-09, fix_wf2b). 3개 파일 51개: `test_steering_evaluate.py` 31, `test_steering_entry_feasibility.py` 11, `test_steering_truth_isolation.py` 9.
  - Windows 3.13.2(numpy 2.2.6, pytest 9.1): 51 통과. 기록 `runs/wf2b/fix_wf2b/pytest_win313_final.log`(WF-2b·1단계 16개 파일 650 통과, 1 건너뜀).
  - 기반 이미지 `6209168a3507`(Python 3.12.3, numpy 1.26.4, scipy 1.11.4, pytest 7.4.4): 50 통과, 1 건너뜀(저장소 가드 시험: 컨테이너에는 붙인 작업 트리 말고 같은 저장소의 다른 작업 트리가 없다). `truth_isolation` CLI 통과(81개 파일, 오류 0). 기록 `fix_wf2b/pytest_docker_py312.log`.
  - WSL 3.10.12(`python3 -m unittest`): 51개 중 50 통과, 1 건너뜀(`single_row_eval` import에 scipy 필요, R10). 저장소 가드 시험은 WSL에서 main checkout(`/mnt/d/...`)까지 보고 통과했다(드라이브 경로 읽기 고침 뒤, FAST_SIM_KO.md 4.2절). `truth_isolation` CLI 통과. 기록 `fix_wf2b/tests_wsl_py310.log`.
- 실제 trace 평가(보고): p2_core probe run 768개(162 s, 에피소드당 약 0.2 s), a8 W0 run 16개, W2 동결 회랑 040927 c08 fwd의 직진(L0, L1)·열로 꺾기(이탈 검출)·정지(hold_measured: 사건 없음, center: `STEER_AFTER_TERMINAL`과 자동 중앙 복귀 검출). 원자료는 `runs/wf2b/p2_eval/`에 있다.

## 8. 실행기 trace 평가 (2026-11-09, 3단계 WF-3a p4_exec)

- 무엇: `tools/row_executor/sim_in_loop_steer.py`(T1 실행기 루프)의 trace를 평가한다. 설명은 `EXECUTOR_STEER_KO.md`.
- `evaluate_executor_trace(arrays, spec, summary, …)`: 에피소드 명세(세계, 제어기)와 결과 요약(`ended_at_s` = 실행기의 종결 시각, `end_reason`)으로 `evaluate_trace`를 부른다. 종결 시각은 요약에서 온다(`terminal_t_s`). 사전등록 종료와 회랑 지표는 W2 세계에만 쓴다. W0의 열린 기동(대본)은 그것 없이 평가한다(fast_sim 기동과 같다).
- `evaluate_executor_run(run 폴더)`: `RESULT.json`과 `TRACE.npz`가 있는 하위 폴더마다 평가하고 `STEER_EVALUATION_EXECUTOR.json`(스키마 `ubts.steer_sim.steer_evaluation_executor_run.v1`)을 쓴다. `field_validated`는 항상 false다.
- CLI(sim/ubts에서): `python -B -m steering.evaluate_steer executor --run <run 폴더> [--out <저장소 밖 파일>]`.
- 붙잡기 기준(`hold_reference()`, R13-2·R13-3): `STEER_AFTER_TERMINAL` 규칙 (b)의 위치. 순서: `truth_hold_position_rad`(실행기·T1 trace, 장치가 쓴다: MCU가 거른 전위차계 위치 = hold_measured STOP이 붙잡는 목표, 출처 `truth_hold_position`) → fast_sim trace가 플랜트 스텝마다 한 행(`trace_every` 1)이면 `SteeredPlant.measured_hold_rad()`를 `pot_raw` 행으로 다시 계산한 값(출처 `measured_hold_recomputed`: 행 j의 앞 `pot_filter.median_of`개 `pot_raw` 중 정렬 `median_of // 2`번째 값의 명목 해석, `steer_measured_rad`의 그 행) → `truth_servo_view_rad`(fast_sim, R13-2 전 trace나 솎은 행) → `steer_measured_rad`(옛 trace). 창마다 `hold_position_source`에 적는다. `evaluate_trace(..., hold_reference_source=...)`로 출처를 정할 수 있다(예: R13-2 전 fast_sim trace는 `truth_servo_view`).
- 검토 P4R-03·RS-06(fix_wf3a): R13-2로 플랜트의 hold_measured 목표가 측정값이 되었는데 fast_sim trace의 기준은 참 서보 보기였다. 그래서 측정과 참값의 차이(100 % duty에서 최대 약 1.02°)를 정지 뒤 목표 이동으로 보고 `STEER_AFTER_TERMINAL`·`auto_centre`를 거짓으로 셌다. fast_sim은 다른 작업 흐름의 파일이라 평가기 쪽에서 같은 기준을 다시 계산하게 고쳤다. 다시 계산한 값은 플랜트 값과 같다(`test_recomputed_hold_reference_is_the_plant_hold_measured_target`, 첫 `median_of`행 뒤 비트까지 같음). fast_sim이 나중에 `truth_hold_position_rad`를 쓰면 그 칸이 먼저 쓰인다.
- W2 판별: trace의 `meta_json`에서 `world`가 없으면 `rig.world.kind`(T1 장치 trace)를 본다. W2 trace를 세계 없이 넣으면 전처럼 거부한다.
- `world_for_spec(world)`: 장면 명세의 세계(사전 형태도 됨)에서 평가 세계를 만든다. W0, W2만. 봉인 회랑은 `allow_sealed`가 없으면 거부한다(`sealed_corridor_refused`). 장면 명세 자체도 먼저 거부한다.
- 정지 분류 제안 표에 실행기 연결 사유를 더했다(모두 SYSTEM_FAULT, 제안): `mcu_reset`, `link_watchdog`, `link_bad_lines`, `link_ttl_expired`, `link_command_rejected`, `link_hardware_fault`, `mcu_state_mismatch`, `arm_unconfirmed`, `arm_rejected`, `link_identity_mismatch`, `bridge_internal_error`, `mcu_disarmed`.
- 시험(`test_steering_evaluate.py` `ExecutorTraceTest` 4개): 붙잡기 칸 순서, run 폴더 평가, 봉인 회랑 거부, 연결 사유 분류.
- 예전 알려진 실패(풀림): `test_l1_brake_stops_without_a_steering_move_are_not_events`의 hold_measured 창 16개 중 4개가 사건으로 나왔다. 위 다시 계산한 기준으로 0/16이 되어 통과한다. 시험은 기준 출처가 `measured_hold_recomputed`인지도 본다.
- 남은 모형 차이(검토 RS-12, P8 전 결정): F-SIM 시험용 서보는 참 서보 보기로 되먹이므로 가변저항 이득·직선성·떠돎 오차가 있으면 hold_measured 정지 뒤 바퀴가 그만큼 움직인다(펌웨어는 목표와 되먹임이 둘 다 P[n]이라 그 오차로는 움직이지 않는다). 평가기의 (b) 판정에는 영향이 없다(목표 칸만 본다). 참 바퀴 표류(`true_drift_deg`)는 보고값이다.
