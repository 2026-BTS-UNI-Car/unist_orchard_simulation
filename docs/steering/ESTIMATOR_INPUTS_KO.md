# 추정기 입력·출력 확장 (PC-05)

- 작성: 2026-11-12. 작업 번호 PC-05. 설계 `DESIGN_v2_KO.md` V2-1(2·4항), V2-3. 결정 R31, R33.
- 대상: `sim/ubts/closed_loop/estimator.py`, 새 파일 `sim/ubts/closed_loop/actuator_prior.py`, vendor 사본(`tools/row_executor/vendor/closed_loop_copy/`), `tools/row_executor/core.py`의 vendor 검사.
- 세 기능 모두 기본 꺼짐이다. 기본 설정의 추정기는 바뀌지 않았다(3절). 모든 값은 자리값이고, 수치는 자리값 기반 SIM 결과다.

## 1. 더한 것

| 기능 | 켜는 설정(`EstimatorConfig`) | 입력 | 출력 |
|---|---|---|---|
| 구동기 사전값(L1) | `actuator_prior`(`ActuatorPriorConfig`, 기본 None) | `on_actuator(stamp, duty_left_pct, duty_right_pct, steer_rad)`: MCU 상태 줄의 **적용** duty(%, 부호 있음)와 보정한 조향각(rad, 모르면 None) | 스냅샷 `actuator_prior` 블록 |
| 통로 품질 | `corridor_quality`(기본 False) | 없음 | 스냅샷 `corridor_quality` 블록 |
| 외부 통로 관측 자리 | `external_corridor_gain`(기본 0), `external_corridor_sources` | `on_corridor_observation(stamp, lateral_offset_m, heading_error_rad, width_m, quality, source)` | 반환값(쓴 이유·안 쓴 이유), 스냅샷 `external_corridor` 블록 |
| 기준점 자세 | 늘 있음(읽기 전용) | 없음 | `base_pose(now)`: 스냅샷의 통로 값이 가리키는 자세(표시·평가용) |

### 1.1 구동기 사전값

- 입력은 상태 줄의 적용 duty와 측정 조향각뿐이다. 요청값이나 SIM 값은 받지 않는다. PWM은 엔코더가 아니다.
- 속도 사전값 v_p = 표(|duty|) × 배율. 표는 (duty %, m/s) 점의 꺾은선이고 duty에 따라 줄어드는 표는 거부한다. 요레이트 사전값 r_p = v_p·tan δ / L(휠베이스가 없으면 없음).
- 상태 줄 하나는 다음 줄이 올 때까지, 길어야 `status_max_age_s`(0.3 s) 동안만 쓴다. 그보다 늦으면 사전값을 끄고 LiDAR만 쓴다.
- 가중 `weight`가 0이면 기록과 감시만 한다. 추정값은 그대로다(시험으로 확인).
- 가중이 0보다 크면:
  - (가) 스캔 정합의 이동 시드 = (1 − w)·LiDAR 속도·Δt + w·사전값 이동량. 회전 시드는 지금처럼 gyro다. 스캔 간격이 0.5 s 이상이면 원래는 정합하지 않는데, 사전값 시드가 있으면 정합을 시도한다.
  - (나) 정합이 실패한 스캔과 마지막 스캔 뒤의 시간은 w·사전값 이동량으로 예측한다. 마지막 정합 자세부터 `predict_max_s`(1 s) 또는 `predict_max_m`(0.5 m) 가운데 먼저 오는 것까지다. 그 뒤에는 더하지 않고 `limit`을 낸다.
- (다) 감시: 최근 약 20 s 주행의 LiDAR 이동 ÷ 사전값 이동(`online_scale`). 표 오차 |사전값 ÷ LiDAR − 1|이 `mismatch_fraction`(0.2)을 넘고 표본이 10 s 이상이면 `table_mismatch`. 표시일 뿐이고 아무것도 세우지 않는다.
- (마) `distance_m`: 사전값 이동량의 누적. LiDAR 거리(`distance_m` 스냅샷 값)는 바뀌지 않는다.
- 기본 표 `DEFAULT_SPEED_TABLE`은 자리값이다: `docs/drive_response/README_KO.md`의 직선 v = 0.00855·duty + 0.025(80 %까지)와 10/10의 90 % 0.715 m/s, 100 % 0.899 m/s. 조향 장치를 달기 전 차의 값이다.

### 1.2 통로 품질

`corridor_quality` 블록: `sides`(both / left / right / none), `source`(init / candidate / guided), `support_left`·`support_right`(선 가까이의 반환 수), `age_s`·`full_age_s`·`left_age_s`·`right_age_s`, `one_sided_run`, `rejected_run`, `lateral_sd_m`·`heading_sd_rad`·`width_sd_m`(받아들인 갱신의 혁신 제곱 EMA의 제곱근). 추정기는 이 값을 다시 읽지 않는다.

### 1.3 외부 통로 관측 자리

- 카메라나 실행기의 국소 격자(PC-07)가 통로 관측을 넣는 자리다. 관측의 뜻은 스냅샷과 같다(횡오차 = 통로 중심에서 왼쪽으로 +, 방향 오차 = 통로 축 − 차 방향).
- 쓰는 조건: 허용한 `source`, `quality` ≥ 문턱, 나이 ≤ 0.5 s, 띠가 있음, 추정기의 방향·횡·폭 문턱 안. 안 쓰면 이유를 돌려준다(`EXTERNAL_REASONS`).
- 쓰면 띠를 이득만큼 당긴다(방향은 차를 중심으로, 그다음 중심, 폭).
- 하지 않는 것: 띠를 새로 만들기, 띠의 나이 갱신(LiDAR 지지가 없으면 2 s 뒤 띠는 그대로 사라진다), 갱신 횟수 세기, 열 끝 근거 바꾸기.

## 2. 시험

| 파일 | 수 | 내용 |
|---|---|---|
| `sim/ubts/test_closed_loop_actuator_prior.py` | 43 | 표·설정 검사, 상태 줄 유지 시간, 시드, 시간·거리 한도, 감시와 표 어긋남(±30 %에서 켜짐, ±15 %에서 꺼짐), 추정기에서 가중 0이면 스냅샷이 같음, 스캔 끊김·정합 실패에서의 한정된 예측 |
| `sim/ubts/test_closed_loop_estimator_inputs.py` | 20 | 통로 품질(양쪽·한쪽·잃음·거부 수), 외부 관측(결과 이유 12종: 쓴 경우 1, 안 쓴 경우 11. 이득만큼 당김, 띠를 살려 두지 않음), 켜도 추정값이 같음, `base_pose` |
| `sim/ubts/test_closed_loop_estimator_exact.py` | 60 | 새 설정 6개와 상태 3개(`prior`, `quality`, `external`)를 선택 기능 목록에 넣었다. 꺼진 상태에서 스캔·IMU마다 추정기 상태 전체가 동결 기준(999a863)과 같고 새 상태는 None이다 |

모두 합성 스캔이다(플랜트·참값 없음).

## 3. 끄면 비트 동일

- 기준 에피소드 10개(E1~E6, R1~R4, `EXECUTOR_IMPORT_KO.md` 2.2절)를 이 변경을 담은 커밋 `d1f1d74`에서 돌려 `main` 3289d16의 결과와 비교했다.

| 산출물 | 같은 수 |
|---|---|
| 행 | 10 / 10 |
| trace | 10 / 10 |
| 하네스 trace(R) | 4 / 4 |
| 실행기 기록(`vendor_commit` 칸 뺌) | 10 / 10 |

- 다른 칸은 실행기 기록의 `vendor_commit`뿐이다(vendor를 새 커밋 `8cafba3`에서 다시 만들었다). `main`의 결과는 가져오기 기준선(`imported_run2`)과도 같다.
- `check_vendor_sync.py`, `check_pins.py` OK. 고정 파일 `estimator.py`가 바뀌어 `PIN_ALLOWLIST.json`에 이유와 함께 넣었다. 검토 뒤 `PIN_MANIFEST.json`을 갱신해야 한다.
- 참값 격리(A13 F): 통과. `actuator_prior.py`는 decision 역할이다.
- 빠른 SIM의 run 식별 파일 목록(`fast_sim.IDENTITY_FILES`)에 `closed_loop/actuator_prior.py`를 넣었다(추정기가 import한다).

## 4. 남은 것

- 실행기 연결(PC-06): 브리지 응답의 상태 줄을 `on_actuator`에 넣기, 설정 키, 예측 한도에서 일시정지(`prediction_limit`), 기록 칸. 만들다 멈췄다. 이 저장소에는 없다.
- 등록 한도로 재는 빠른 SIM 확인(`lidar_blackout_1p0`, 표 ±30 %)은 하지 않았다.
- 국소 격자 층(PC-07)은 시작하지 않았다. 외부 관측 자리가 그 연결점이다.
  - 2026-11-13 갱신: 국소 격자 층이 이 자리를 쓴다(source `local_grid`, 실행기 설정 `local_grid_fusion_gain`). `LOCAL_GRID_KO.md`. 추정기는 바꾸지 않았다.

## 5. 굽은 열의 열 끝 (2026-11-13, 묶음 S1c, 결정 R51)

- 설정: `EstimatorConfig.row_end_curve_fit`(기본 False), `row_end_curve_reach_m`(4, 6, 8, 10 m), `row_end_curve_behind_m`(2 m), `row_end_curve_max_1_m`(1/15 1/m), `row_end_curve_min_points`(6), `row_end_curve_edge_m`(1.5 m), `row_end_curve_max_gap_m`(3 m). 실행기 front_steer는 `steer_row_end_curve_fit`(기본 참)으로 켠다. 차동 실행기는 그대로(꺼짐)다.
- 하는 일: 두 선이 다 보이는 갱신마다 열 끝 규칙(`_side_extents`) 앞에서 두 열 선에 공통 포물선 옆 = 열별 거리 + c × 앞^2을 맞춘다. 가까운 곳(LiDAR 뒤 2 m ~ 앞 4 m)의 선 지지(옆 허용 0.35 m)로 시작해 6, 8, 10 m로 넓히며 그때의 곡선 둘레에서 다시 고른다. 열마다 6점이 안 되면 거기서 멈추고, 한쪽 선이 없으면 c = 0(곧은 규칙). |2c| ≤ 1/15 1/m. 한 단계는 두 선의 곡선 둘레 반환이 그 거리의 `row_end_curve_edge_m`(1.5 m) 안까지 이어지고 앞뒤 빈칸이 `row_end_curve_max_gap_m`(3 m) 이하일 때만 쓴다. 첫 단계가 안 되면 c = 0이다. 열 끝 가까이에서는 선이 첫 단계에 못 미쳐 곧은 규칙이 끝을 정한다(반복 2에서 이 조건 없이 W2 `042301_c03`의 열 끝 너머 구조물까지 곡선을 키워 끝이 0.7~1.7 m 늦었다).
- 열 끝 규칙(선 지지, 빈 거리, 가로 구조물, 지지 자르기, 엇갈림)은 이 곡선에서 잰 옆 거리를 쓴다. band(횡오차·방향·폭)와 다른 규칙은 바뀌지 않는다.
- 까닭: 곧은 규칙은 반지름 30 m 열(8 m 앞에서 곧은 선과 1.07 m 어긋남)의 두 선이 4~8 m 앞에서 '끝나고' 그 뒤가 비었다고 보아, band와 함께 물러나는 열 끝을 냈다. 넓은 확인 반복 1에서 `curve_l30` 개발 seed 2개가 24 m 열의 9, 13 m에서 섰다(`STEER_FOLLOW_WIDE_RESULT_KO.md`).
- 시험: `sim/ubts/test_closed_loop_estimator_curve.py`(9: 열 끝 가까이의 구조물·빈칸에서 곡선을 키우지 않음, 반지름 ±30·60 m의 이어지는 열은 열 끝이 아님, 곧은 규칙은 ±30 m에서 거짓 끝, 진짜 열 끝은 곧은 열·굽은 열 모두 그대로, 한쪽 선만 있으면 c = 0, 범위·설정 검사). 꺼진 상태의 비트 동일은 `test_closed_loop_estimator_exact.py`(새 설정 5개를 `CURVE_FIELDS`로 등록, 80 통과)가 본다.
