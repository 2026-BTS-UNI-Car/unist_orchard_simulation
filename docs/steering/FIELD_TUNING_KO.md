# 현장 조정 파일 (PW-07, PW-08, PC-13)

- 작성: 2026-11-13 (묶음 S1, 브랜치 `autonomy-exec-20261113`)
- 설계: `DESIGN_v2_KO.md` V2-5, 결정 `DECISIONS_KO.md` R35. 화면 규약: `OPERATOR_API_KO.md` 6절.
- 모든 값은 자리값이다. 조향 장치를 단 차로 잰 값이 아니다. 이 파일로 낸 수치는 자리값 기반 SIM 결과이며 현장 성능이 아니다.

## 1. 무엇인가

현장에서 사람이 바꿀 값(속도 표 배율, 목표 속도, 주시거리, 이득 등)을 JSON 파일 하나에 모은다. 실차 실행기, 빠른 SIM(T1),
Gazebo 실행기 컨테이너가 같은 파일을 읽는다. 셋 다 실행기 루프(`executor_main.ExecutorLoop`)를 만들고, 루프가 시작할 때
설정 키 `tuning_file`이 가리키는 파일을 읽는다. `tuning_file`이 비어 있으면 패키지 기본 파일
`tools/row_executor/field_tuning.default.json`을 읽는다(이 파일은 `SHA256SUMS`에 들어 있어 실차 실행기가 시작할 때 검사한다).

- 코드: `tools/row_executor/field_tuning.py`(표준 라이브러리만, Python 3.10·3.13에서 같은 결과를 확인, 3.12는 이 PC에 없어 못 돌림).
- 스키마 이름: `ubts.field_tuning.v1`.
- 루프는 값을 실행기 설정의 `tuning_values`(조향 제어기 값)와 `prior_speed_table`·`prior_speed_scale`(구동기 사전값)에 넣는다.
  그래서 설정 digest가 조정 값을 따라 바뀌고, 조향 제어기(`closed_loop/steer_control.py`)는 그 값을 기본값 위에 쓴다.
  SIM 실험의 제어기 spec에 값이 있으면 그것이 파일 값보다 앞선다. 실행기 설정에 사전값 표·배율을 따로 적은 경우(SIM 실험,
  예: `prior_checks`의 표 ×1.3)도 설정 값이 앞선다.
- `vehicle_profile`이 `front_steer`일 때만 쓴다. 차동 설정은 예전 현장 키 3개(`base_duty_pct` 등)를 그대로 쓴다.

## 2. 묶음

| 묶음 | 키(기본값) | 바꾸는 곳 |
|---|---|---|
| 기본 | `speed_scale`(k_field 1.0, 0.5~1.3), `target_speed_m_s`(0.40, 0.20~0.50), `lookahead_base_m`(1.5), `lookahead_per_speed_s`(1.25), `lookahead_max_m`(3.0), `k_d`(1.0), `k_psi`(1.0), `steer_rate_fraction`(0.75: 실행기 가드 변화 속도 한계의 비율) | 화면(`POST /api/params`), 파일 |
| 고급 | `speed_table`(duty→속도 표), `surface_scale`, `battery_scale`, `min_speed_m_s`, `approach_speed_m_s`, `approach_window_m`, `end_stop_offset_m`, `duty_max_pct`, 속도 PI 이득·한도, duty 올림·내림 속도, `k_r`, trim 이득·한도, `steer_follow_limit_deg`, `steer_filter_s`, `d_scan_m`, `d_blind_m`, `loop_fixed_s`, `corridor_weak_age_s`, 횡오차 감속 구간, `lateral_accel_max_m_s2`, `steer_slow_start`, `fresh_slow_age_s` | 파일 |
| 잠금 | `max_speed_m_s`(0.50), `front_margin_m`(0.35), 과속 한도(0.05 m/s, 1 s), 그리고 실행기 설정에 있는 `duty_cap_pct`, 신선도 문턱(스캔 0.45, IMU 0.40, 조향 상태 0.3, 루프 틈 0.35 s), `operator_timeout_s`(0.5 s), `guard_heading_deg`(25), 사전값 예측 한도(1.0 s, 0.5 m) | 파일만, 다시 시작해야 바뀜 |

- 잠금 묶음 중 실행기 설정에 있는 값은 파일과 설정이 같아야 한다. 다르면 파일을 거부하고 실행기가 시작하지 않는다(안전 한계의
  출처는 실행기 설정 하나).
- 전체 표는 `field_tuning.describe()`(이름, 묶음, 단위, 범위, 기본값, 설명)이고 상태의 `tuning.basic`에 기본 묶음의 범위가 있다.
- 실제 속도 배율 = `speed_scale` × `surface_scale` × `battery_scale`, 0.5~1.3 안이어야 한다.

## 3. 검사(PW-07)

다음을 거부하고 이유를 키 이름과 함께 돌려준다: 중복 키, NaN·Infinity, 모르는 키, 다른 묶음에 놓인 키, 범위 밖, 정수 자리의
소수, duty가 늘지 않거나 속도가 줄어드는 표, 값 사이 규칙 위반(최저 < 목표 ≤ 최대, 최저 ≤ 접근 ≤ 최대, 주시거리 바닥 ≤
상한, 횡오차 감속 시작 < 끝, duty 상한 ≤ 잠금 duty 상한, `fresh_slow_age_s` < 스캔 신선도 문턱, 배율 곱 범위). 조향 제어기의
규칙(추종 조향 한도 ≤ 0.9 × 소프트 한계 등)은 값을 적용할 때 제어기가 다시 본다.

## 4. 화면에서 바꾸기(PC-13)

`POST /api/params`(실행기 운영자 HTTP, 대시보드가 대신 부름):

- `{"target_speed_m_s": 0.3, "k_d": 1.2}` 기본 묶음 키만. 고급·잠금 키는 400.
- `{"preset": "slow_030"}` 프리셋 불러오기(혼자만). `default`는 기본값이다. 기본 프리셋으로 되돌리면 digest가 기본값의 digest와 같다.
- `{"save": true}`, `{"save_preset": "이름", "note": "메모"}` 지금 값을 파일에 쓴다(임시 파일 뒤 이름 바꾸기). 패키지 기본 파일에는
  쓰지 않는다(400): 실차에서는 `tuning_file`을 패키지 밖 경로로 둔다.
- 정지 상태에서만: run이 열려 있으면 409 `tuning_locked_while_running`, lease를 쥐고 있으면 `tuning_locked_while_armed`,
  LiDAR 속도가 0.03 m/s 이상이면 `tuning_locked_vehicle_moving`. 값은 run을 시작할 때 고정된다(run 동안 바꿀 수 없다).
- 범위 밖·규칙 위반은 400 `invalid_params`와 `problems` 목록.
- 바꾸면 프리셋 이름 끝에 `*`가 붙고 `revision`이 1 오른다. 실행 기록(JSONL)에 키마다 `tuning_change` 사건(이전·새 값, 바꾸기
  전후 digest, 새 설정 digest)이 남는다. 저장하면 `tuning_save`(파일 SHA-256).

## 5. 기록에 남는 것(PW-08)

- 실행기 기록의 `config` 사건과 run마다의 `start` 사건: `tuning`(프리셋, revision, digest, 파일 경로, 파일 SHA-256).
- 빠른 SIM `RESULT.json`의 `summary.tuning`, Gazebo 실행기 컨테이너 `config.json`의 `tuning`.
- 셋이 같은 패키지 파일을 읽는 것은 시험 `tools/row_executor/tests/test_field_tuning.py`(ThreePlacesTest)로 본다. 빠른 SIM에서
  기본 파일을 읽은 실행(nominal seed 0)의 직렬 로그와 trace가 파일 없이 돌린 실행과 같았다(기본값 = 제어기 기본값).

## 6. 하지 않은 것

- 표의 한 칸을 바꿨을 때 세 곳의 정상 속도가 같이 바뀌는지 재는 시험과 차 모델만 표와 어긋나게 바꾼 5단계 어긋남 시험(PW-08
  완료 기준)은 측정이라 이 묶음에서 돌리지 않았다. 사전등록이 먼저다(묶음 S1c).
- 화면의 고급 묶음 편집, 온라인 배율 저장(PW-10)은 없다.
