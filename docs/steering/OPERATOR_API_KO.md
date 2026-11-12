# 자율 주행 시작·정지 규약 (실행기 운영자 HTTP, 최종판)

- 작성: 2026-11-13 (묶음 S1, 브랜치 `autonomy-exec-20261113`). 조정 쪽 초안 `runs/_locks/AUTONOMY_API_CONTRACT_20261113.txt`(v0)과
  그 끝의 개정 1·2를 합친 최종판이다. 대시보드 서버(묶음 S4b)와 SIM 대시보드(묶음 S5)는 이 문서를 기준으로 만든다.
- 설계: `DESIGN_v2_KO.md` V2-2(한 번 눌러 시작), V2-4(속도 조절기), V2-5(현장 조정 값). 결정 R32, R34, R35.
  합격 기준: `SIM_ACCEPTANCE_KO.md` 개정 31(A17 자율 시작·정지), 개정 32(A11 보고).
- 코드: `tools/row_executor/operator_http.py`(경로), `executor_main.py`(`OperatorLink`, `ExecutorLoop`), `field_tuning.py`.
- 수치는 자리값이다. SIM에서 잰 값은 자리값 기반 SIM 결과이며 현장 성능이 아니다.

## 1. 접근 규칙(바뀌지 않음)

- 실행기가 설정한 주소에만 묶인다(현장 기본 10.77.0.1:8097, SIM 컨테이너는 127.0.0.1:8097). 와일드카드 바인드는 설정이 거부한다.
- 토큰: 실행기가 시작할 때마다 새로 만든다(`<log-dir>/operator_token.txt`, 0600). 모든 API는 헤더 `X-Row-Token`.
- `Host`는 정확히 `<주소>:<포트>`. 모든 POST에 `Origin: http://<주소>:<포트>`. JSON 본문만, 4 KiB 이하, 엄격한 해석(NaN 거부).
- 대시보드 서버가 실행기를 대신 부른다. 토큰은 서버의 설정 파일 경로에서 읽고 화면으로 내보내지 않는다.

## 2. 시작 방식

`start_mode`는 실행기 설정 키다(잠금: 설정 파일만).

| 값 | 뜻 |
|---|---|
| `one_press` | 앞바퀴 조향(`vehicle_profile` `front_steer`)의 기본. Start → Arm → `/api/go` 한 번. 화면이 보이는 동안 `/api/heartbeat`를 보낸다 |
| `hold_to_run` | 차동 설정의 기본(이력). 누르는 동안만 간다(`/api/hold`, `/api/release`, 100 ms 심장 박동, 0.5 s 기한) |
| 비움(null) | 프로필 기본(위 두 줄) |

- 수동 조종(대시보드 W/S/A/D)은 이 규약과 상관없이 누르는 동안만 간다.

## 3. 경로

모두 JSON. 거절은 `{"ok": false, "error": "<이유>"}`.

| 경로 | 본문 | 뜻 | 응답 |
|---|---|---|---|
| `GET /api/status` | - | 상태(4절). 화면은 2 Hz로 읽는다 | 200 |
| `POST /api/start` | `{"row_len_m": L 또는 null}` | 새 run. 단일열 모드에서 열 길이는 비워도 된다 | 200 / 409 |
| `POST /api/arm` | `{}` | 브리지 lease(MOTOR만) | 200 / 409 |
| `POST /api/go` | `{"session": "<id>"}` | one_press: 열린 run을 한 번 출발시키거나 일시정지 뒤 다시 출발 | 200 / 409(+`blockers`) |
| `POST /api/heartbeat` | `{"session": "<id>"}` | one_press: 운영자 연결 확인 | 200 / 409 |
| `POST /api/stop` | `{}` | 즉시 정지(op stop), 래치. 누구나, 어느 세션이든 | 200 / 502(브리지 미확인) |
| `POST /api/ack` | `{}` | 래치 확인(다시 Arm하지 않음) | 200 |
| `POST /api/end` | `{}` | run 정상 종료 | 200 / 409 |
| `POST /api/params` | 6절 | 현장 조정 값(front_steer) / 현장 키 3개(차동) | 200 / 400 / 409 |
| `POST /api/hold`, `/api/release` | `{"press": "<id>"}` | hold_to_run에서만. one_press에서는 409 `start_mode_one_press` | |

- `session`: 화면이 페이지를 열 때 만드는 무작위 id(4~64 글자, 예: 12바이트 hex). 한 화면은 한 id를 계속 쓴다.
- hold_to_run 에서 `/api/go`·`/api/heartbeat`는 409 `start_mode_hold_to_run`.
- `/api/go`·`/api/heartbeat`에 `session` 말고 다른 키가 있으면 400 `unknown_parameter`.

### 3.1 운영자 연결

- 화면은 자율 화면이 보이는 동안 `/api/heartbeat`를 100 ms마다 보낸다(늦어도 250 ms). 탭을 숨기거나(visibility hidden) 떠나면
  (pagehide) 멈춘다. 실제 기한은 상태의 `operator_link.timeout_s`다.
- `operator_timeout_s`(실행기 설정, 기본 0.5 s, 범위 0.5~3.0, 잠금): 마지막 heartbeat 뒤 이 시간이 지나면 실행기가 스스로 선다.
  구동 0을 그 틱에 보내고 조향은 그 자리에 둔다. `run_state` `paused`, `stop_reason` `OPERATOR_LINK_LOST`.
  초안의 1.0 s가 아니라 0.5 s인 까닭: 설계 V2-2 3항과 R32, 측정 전에 등록된 A17 개정 31("운영자 연결 끊김 → 구동 0 송신
  ≤ 0.5 s + 1틱")이다. 1.0 s로 바꾸려면 새 결정과 기준 개정이 먼저다.
- 자동 재출발은 없다. heartbeat가 다시 와도 run은 일시정지 그대로다. 다시 가려면 새 `/api/go`가 필요하다(입력이 1 s 신선해진 뒤).
- 한 화면만 운영자다. 다른 세션이 묶인 세션이 살아 있는 동안 heartbeat나 go를 보내면 409 `session_conflict`이고 묶인 화면에는
  아무 일도 없다. 묶인 화면의 연결이 끊긴 뒤에는 다른 세션이 묶일 수 있지만 그 화면의 go를 물려받지 않는다(새 go 필요).
- STOP은 세션과 상관없이 누구나 된다. STOP은 go를 끝낸다(다시 가려면 확인 → Start → Arm → go).
- 한 run의 일시정지(신선도, 운영자 연결, 속도 상한 아래, 과속, 사전값 예측 한도)를 합쳐 `max_stale_events`(3)을 넘으면 run을
  끝낸다(`stale_repeat_limit`).

### 3.2 `/api/go`의 시작 조건

실행기가 main loop에서 본다. 하나라도 걸리면 409 `{"ok": false, "error": <첫 이유>, "blockers": [...]}`.

| 이유 | 뜻 |
|---|---|
| `session_conflict` | 다른 화면의 세션이 묶여 있고 살아 있다 |
| `operator_link_not_fresh` | (상태 표시용) 운영자 heartbeat가 기한 안에 없다 |
| `no_run` / `already_running` | run이 없다(Start 먼저) / 이미 주행 중 |
| `latched` / `arm_first` | MOTOR: 래치(확인 먼저) / Arm 전 |
| `inputs_not_fresh` | 스캔·IMU가 `fresh_recover_s`(1 s) 동안 신선하지 않았다 |
| `steering_feedback` | 조향 상태가 신선하지 않거나 확인되지 않았다 |
| `vehicle_moving` | LiDAR 속도 ≥ 0.03 m/s(정지 확인 안 됨) |
| `corridor_not_acquired` | 통로를 아직 못 잡았다 |

- 화면의 '자율 주행 시작' 버튼은 상태의 `go_ready`가 참일 때만 켜고, 아니면 `go_blockers`를 보인다.
- 실행기 자신의 검사(입력 신선, 조향 상태, 정지 확인)는 go를 받은 뒤에도 그대로다.

### 3.3 첫 Arm

브리지는 lease가 없을 때도 MCU 상태 줄이 늦으면 `steering_status_stale` 래치를 건다(G-SIM: Gazebo 시계가 멈칫할 때). 예전에는 그
래치 때문에 첫 Arm이 한 번 거절됐다. 이제 실행기는 결함이 그 래치뿐이고(MCU 결함 비트·다른 래치 없음), 아무도 lease를 쥐지
않았고, 지금 상태가 신선하면 그 Arm 누름을 운영자 재무장으로 본다. 브리지는 MCU의 ARM 확인 때 래치를 지운다. 다른 결함은
그대로 거절한다. 그래서 화면은 Arm을 재시도할 필요가 없다.
- 호스트 도구 `steering.exec_run`(G-SIM)은 이 고침 전에 만든 실행기 이미지도 몰기 때문에 그 래치 거절과 브리지 준비 전 거절
  (`ARM_RETRY_ERRORS`)에만 확인 → 새 run → 다시 Arm을 한다. 다른 거절은 한 번에 끝낸다(자동 확인 없음).

## 4. 상태(`GET /api/status`)

기존 칸에 더해:

| 칸 | 값 |
|---|---|
| `start_mode` | `one_press` / `hold_to_run` |
| `run_state` | `idle`(run 없음) / `ready`(Start 뒤 Arm 전, MOTOR만) / `armed`(go를 기다림; SHADOW는 Arm이 없어 곧바로 armed) / `running` / `paused` / `ended` / `latched` |
| `operator_link` | `{fresh, age_s, timeout_s, session(묶인 세션 id의 SHA-256 앞 8자), go_active, go_serial, last_cancel, 횟수들}` |
| `go_ready`, `go_blockers` | 3.2절 |
| `stop_reason` | 일시정지·끝의 분류: `ROW_END`, `OBSTACLE`, `END_STRUCTURE`, `PERCEPTION_OR_LOCALISATION`, `CONTROL_AUTHORITY`, `OPERATOR_OR_BACKSTOP`, `SYSTEM_FAULT`, `OPERATOR_LINK_LOST`, `UNCLASSIFIED`; 주행 중·대기 중은 null |
| `stop_detail` | 원래 이유 문자열(예: `operator_link_lost`, `row_end`, `stale_lidar`, `speed_cap_below_min`) |
| `speed` | `{target_mps, limit_mps, limit_reason, caps}`(5절) |
| `tuning` | `{preset, revision, editable, ...}`(6절) |

- `stop_reason` 분류표: 새 이유의 분류는 `operator_link_lost` → `OPERATOR_LINK_LOST`, `speed_cap_below_min`·`prediction_limit` →
  `PERCEPTION_OR_LOCALISATION`(센서 쪽), `overspeed` → `CONTROL_AUTHORITY`(구동 쪽). 제안이며 조향 사전등록(PC-14)에서 정한다.

## 5. 속도 조절기(PC-10)

`speed`:

- `target_mps`: 현장 조정 파일의 목표 속도(기본 0.40 m/s, 0.20~0.50).
- `limit_mps`: 이 틱의 속도 기준 v_ref = 아래 상한의 최솟값(주행 중만, 아니면 null).
- `limit_reason`: 묶인 상한 이름. 주행 중이 아니면 `not_running`.
- `caps`: 상한마다 값.

| 상한 | 식 | 뜻 |
|---|---|---|
| `cruise` | 목표 속도 | 순항 |
| `scan` | d_scan / T_scan | 스캔 사이 간격(최근 2 s p90) |
| `blind` | d_blind / τ_loop | 스캔 나이(첫 틱, p90) + 고정 지연 + 늦은 틱(틱 간격 p90 − 50 ms): 연산 지연 |
| `map` | d_map / A_map | 국소 격자 나이(PC-07, 지금은 없음) |
| `stop` | 정지 거리 + 앞 여유 ≤ 앞 빈 거리 | 앞 막힘 |
| `curvature` | √(a_lat / \|κ\|) | 곡률 |
| `lateral` | 횡오차가 클수록 | 횡오차 |
| `steer` | 조향 요구량(목표 조향 / 추종 한도)이 0.8을 넘으면 최저 속도까지 | 조향 요구량 |
| `quality` | 최저 속도 | 한쪽 통로, 오래된 통로, 사전값 예측 중 |
| `fresh` | 최저 속도 | 최신 스캔이 0.30 s보다 오래됨(신선도 정지 0.45 s 전) |
| `end` | 접근 속도 | 열 끝 접근 |

- v_ref가 최저 속도(0.20 m/s) 아래면 `speed_cap_below_min`으로 서서 일시정지한다(새 go 필요). 앞 막힘은 `obstacle_ahead`(끝).
- 과속: LiDAR 속도가 최대 속도 + 0.05 m/s를 넘으면 그 틱에 최저 속도의 duty로 내리고, 1 s 넘게 이어지면 `overspeed`로 일시정지.
- 실행 기록 틱의 `sr` 블록에 `cap`(묶인 상한)과 `tau`(τ_loop)가 있다(A11 개정 32 보고용).

## 6. 현장 조정 값(`POST /api/params`, front_steer)

자세한 것은 `FIELD_TUNING_KO.md`.

- 본문: 기본 묶음 키만 `{"target_speed_m_s": 0.3, ...}`, 또는 `{"preset": "<이름>"}`, 또는 `{"save": true}` /
  `{"save_preset": "<이름>", "note": "..."}`.
- 정지 상태에서만: 아니면 409(`tuning_locked_while_running`, `tuning_locked_while_armed`, `tuning_locked_vehicle_moving`).
- 범위 밖·규칙 위반·고급·잠금 키·모르는 키: 400 `invalid_params`와 `problems` 목록.
- 성공: `{"ok": true, "action", "changes": [{key, old, new}], "tuning": {preset, revision, digest, file, file_sha256},
  "config_digest"}`.
- 상태 `tuning`: `preset`(바꾸면 끝에 `*`), `revision`, `editable`, `not_editable_because`, `digest`, `file`, `file_sha256`,
  `modified`, `presets`, `writable`, `unsaved`, `basic`(이름·값·범위·단위), `values`(묶음별 모든 값).

## 7. 화면이 할 일(요약)

1. 페이지를 열 때 세션 id 하나를 만든다. 보이는 동안 100 ms마다 heartbeat.
2. Start → Arm(MOTOR). `go_ready`가 참이면 '자율 주행 시작'을 켠다. 누르면 `/api/go` 한 번.
3. 주행 중에는 STOP을 크게 보인다. 일시정지면 `stop_reason`·`stop_detail`과 '다시 가려면 다시 누르기'를 보인다.
4. 409 `session_conflict`를 받으면 '다른 화면이 운영자'를 보이고 STOP만 둔다.

## 8. 시험과 SIM 확인

- 단위·HTTP: `tools/row_executor/tests/test_one_press.py`(링크, 세션, 끊김, 재출발 없음, STOP 우선, MOTOR Arm 순서, HTTP 경로),
  `test_steer_executor.py`(FirstArmTest), `test_field_tuning.py`, `test_steer_follow_runner.py`(속도 조절기 일시정지).
- 빠른 SIM(T1, 실행기 포함): 안전 대본 `operator_link_lost`, `operator_link_back_no_restart`, `other_screen_refused`,
  `one_press_stop`. 2026-11-13 결과: 7개(이 넷과 deadman_release, hold_lost, mid_row_stop_w0) 모두 기대대로.
  `operator_link_lost`: 마지막 heartbeat 4.95 s, 일시정지 5.50 s, 구동 명령 0 첫 trace 5.51 s(한도 5.55 s). 자리값 기반 SIM 결과.
- 호스트 도구: `python3 -B -m steering.exec_run start`(G-SIM 실행기 컨테이너 안)가 one_press 실행기에 세션 heartbeat와 go를 보낸다.
- 기존 G-SIM 점검(`steering.exec_checks`)은 아직 hold로 누른다. 그 점검은 실행기 컨테이너를
  `--config-overrides '{"start_mode": "hold_to_run"}'`로 띄우거나 one_press 로 고쳐야 한다(묶음 S2).
