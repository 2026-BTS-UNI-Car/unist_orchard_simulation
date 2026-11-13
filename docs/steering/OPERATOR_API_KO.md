# 자율 주행 시작·정지 규약 (실행기 운영자 HTTP, 최종판)

- 작성: 2026-11-13 (묶음 S1, 브랜치 `autonomy-exec-20261113`). 조정 쪽 초안 `runs/_locks/AUTONOMY_API_CONTRACT_20261113.txt`(v0)과
  그 끝의 개정 1·2를 합친 최종판이다. 대시보드 서버(묶음 S4b)와 SIM 대시보드(묶음 S5)는 이 문서를 기준으로 만든다.
- 설계: `DESIGN_v2_KO.md` V2-2(한 번 눌러 시작), V2-4(속도 조절기), V2-5(현장 조정 값). 결정 R32, R34, R35.
  합격 기준: `SIM_ACCEPTANCE_KO.md` 개정 31(A17 자율 시작·정지), 개정 32(A11 보고).
- 코드: `tools/row_executor/operator_http.py`(경로), `executor_main.py`(`OperatorLink`, `ExecutorLoop`), `field_tuning.py`.
- 개정 5(2026-11-14, G-SIM 5단계): 세션 묶임·풀림 규칙(3.1절). 같은 글을 `runs/_locks/AUTONOMY_API_CONTRACT_20261113.txt` 끝에 적었다.
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
- `operator_timeout_s`(실행기 설정, 기본 **1.0 s**, 범위 0.5~3.0, 잠금): 마지막 heartbeat 뒤 이 시간이 지나면 실행기가 스스로
  선다. 구동 0을 그 틱에 보내고 조향은 그 자리에 둔다. `run_state` `paused`, `stop_reason` `OPERATOR_LINK_LOST`.
  2026-11-13 결정 R60 으로 0.5 s 에서 1.0 s 로 바꿨다(합격 기준 A17 개정 60: "운영자 연결 끊김 → 구동 0 송신 ≤ operator_timeout_s
  + 1틱"). 까닭: 느린 연결 시험대(대시보드 docs/slow_link.md 12절, 나쁨 등급 명령 왕복 p50 230 ms, 꼬리 1 s 안팎)와 G-SIM 의
  대시보드 중계 지연(p95 124 ms, 최대 356 ms)에서 0.5 s 는 heartbeat 틈 하나로 자주 섰다. 화면 heartbeat 100 ms 는 그대로다.
  처음(개정 31, R32)에는 0.5 s 였다.
- 자동 재출발은 없다. heartbeat가 다시 와도 run은 일시정지 그대로다. 다시 가려면 새 `/api/go`가 필요하다(입력이 1 s 신선해진 뒤).
- 한 화면만 운영자다(세션 규칙, 개정 5, 2026-11-14):
  - 묶임: 묶인 세션이 없을 때 heartbeat나 go를 보낸 세션이 묶인다. 묶인 세션이 살아 있는(마지막 heartbeat 뒤
    `operator_timeout_s` 안) 동안 다른 세션의 heartbeat는 409 `session_conflict`이고 묶인 화면에는 아무 일도 없다.
  - go가 묶음을 확정한다: 그 run에서 go를 한 세션이 살아 있는 동안 다른 세션의 go는 409 `session_conflict`다(주행 중, 일시정지
    중 모두). heartbeat만 보낸 세션(그 run에서 go 없음)은 다른 세션의 go를 막지 않는다. go가 묶음을 넘겨받고(`takeover`), 앞
    세션의 다음 heartbeat는 409다.
  - 풀림: (1) run이 끝나면(열 끝, STOP, 결함, `/api/end`; `run_state` `ended`·`latched`) 실행기가 그 틱에 세션을 푼다.
    (2) heartbeat가 `operator_timeout_s` 넘게 오지 않으면 세션을 푼다(달리던 run은 전처럼 `OPERATOR_LINK_LOST`로 일시정지).
    풀린 뒤 처음 heartbeat나 go를 보낸 세션이 묶인다.
  - 새로 묶인 세션은 앞 화면의 go를 물려받지 않는다(새 go 필요). 자동 재출발은 없다. STOP은 누구나 된다.
  - 기록: 실행기 JSONL에 `operator_link` 사건(`action` bind / takeover / release / conflict, 세션은 SHA-256 앞 8자만, conflict는
    묶인 세션과 거절된 세션 쌍마다 처음 한 번). 상태 `operator_link`에 `committed`, `last_release`, `releases`, `takeovers`.
  - 까닭: G-SIM 4단계 화면 확인(2026-11-13, `STEER_FOLLOW_GSIM_FINAL_KO.md` 13.3)에서 다른 세션이 armed 동안 heartbeat만
    보내며 묶고 있어 운영자 화면의 heartbeat 270개와 go가 약 32 s 동안 409였다. 예전 규칙에서는 run이 끝나도 세션을 풀지
    않았고, heartbeat만 보낸 세션도 go를 막았다.
- STOP은 세션과 상관없이 누구나 된다. STOP은 go를 끝낸다(다시 가려면 확인 → Start → Arm → go).
- STOP으로 끝난 run의 끝 이유는 언제나 `operator_stop`(`OPERATOR_OR_BACKSTOP`)이다. 실행기가 STOP을 받으면 브리지에 op stop을
  보내기 전에 표시를 세운다. 그 STOP이 같은 틱의 구동 요청을 막아(`stop_barrier`) 거절이 와도 run은 `operator_stop`으로 끝난다
  (2026-11-13 SIM 대시보드 세션 s2: 일시정지 중 STOP이 `bridge_error` / `SYSTEM_FAULT`로 남았다). 대시보드는 STOP 때 브리지에도
  직접 STOP을 보낸다. 그쪽이 먼저 닿아 실행기의 lease가 사라지면(`control_not_owned` / `stop_barrier`, 브리지 제어 이유
  `operator_stop`, 조향 결함 없음) 그것도 `operator_stop`이고 `stop_detail`은 그대로, 끝 메모가 `bridge_operator_stop:...`이다.
  CLI나 두 번째 화면의 STOP도 같다. 브리지의 제어 이유만으로는 누가 눌렀는지 모른다.
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
| `lease_lost` | MOTOR: run이 첫 go를 기다리는 동안 브리지가 lease를 거뒀다(3.4절). `arm_first`와 함께 나온다 |
| `inputs_not_fresh` | 스캔·IMU가 `fresh_recover_s`(1 s) 동안 신선하지 않았다 |
| `steering_feedback` | 조향 상태가 신선하지 않거나 확인되지 않았다 |
| `vehicle_moving` | LiDAR 속도 ≥ 0.08 m/s(정지 확인 안 됨). 자리값, 결정 R62(0.03 에서 바꿈) |
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

### 3.4 기다리는 동안의 lease

- Arm 뒤 go를 기다리는 동안(`armed`) 실행기는 send_hz(10 Hz)마다 구동 0·조향 유지 요청(조향 목표가 아직 없으면
  keepalive)을 보낸다. 브리지의 티켓(1.5 s)과 소유자(30 s) 기한은 이것으로 늘 새로워진다. 오래 기다려도 lease는 남는다
  (단위 시험 45 s).
- 그래도 브리지가 lease를 거둘 수 있다. G-SIM 세션 s2·s4(2026-11-13)에서는 호스트 부하로 MCU 상태 줄이 150 ms 넘게 늦어 브리지가
  `steering_status_stale` 래치를 걸고 lease를 거뒀다(s4 브리지 사건 기록으로 확인). 이것은 실행기가 막을 일이 아니고 막아서도 안
  된다(안전 래치).
- 고른 쪽: run을 끝내지 않고 다시 Arm하게 한다. 조건은 run이 한 번도 움직이지 않았고(첫 go 전, 0만 보냄) 거절이
  `control_not_owned` / `stop_barrier`이며, 새 브리지 상태에 조향 결함이 없거나 결함이 상태 줄 틈 래치뿐이고, 브리지 제어 이유가
  STOP(`operator_stop`)이 아닐 때다. 그때 상태는 `run_state` `ready`, `go_blockers` `lease_lost`·`arm_first`, `stop_reason`
  `CONTROL_AUTHORITY`, `stop_detail` `lease_lost`, 새 칸 `lease_lost`(거절, 브리지 이유, 래치, 시각)다. 화면은 'Arm'을 다시
  보인다. 다음 Arm 누름이 그 래치의 운영자 확인이다. 실행기는 그 확인으로 브리지 재무장을 청하고, 브리지는 조건이 아직 남아
  있으면 거절한다(거절이면 run은 `bridge_arm_failed`로 끝난다). 다시 Arm한 뒤 go 한 번으로 출발한다. Arm 응답에
  `rearm_after_lease_lost: true`가 붙는다.
- 이 쪽이 안전한 까닭: 차는 움직이지 않았고 브리지는 이미 STOP했다. 실행기는 스스로 다시 Arm하지 않고, 운영자의 Arm과 go라는
  두 번의 명시적 동작이 있어야 움직인다. go의 다른 시작 조건(입력 신선, 조향 상태, 정지, 통로)도 그대로다. lease를 지키려고
  기한을 늘리거나 재시도하지 않는다.
- 그 밖은 전과 같다: 주행 중이거나 한 번이라도 움직인 run에서 lease를 잃음, 기다리는 중의 STOP(`operator_stop`)이나 다른 조향
  결함(예: `steering_not_following`), 실행기 자신이 티켓 기한(1.5 s)보다 오래 멈춤(`stale_input_rearm_required`, `bridge_error`)은
  run을 끝낸다(`latched`, 확인 → Start → Arm → go).

## 4. 상태(`GET /api/status`)

기존 칸에 더해:

| 칸 | 값 |
|---|---|
| `start_mode` | `one_press` / `hold_to_run` |
| `run_state` | `idle`(run 없음) / `ready`(Start 뒤 Arm 전, MOTOR만) / `armed`(go를 기다림; SHADOW는 Arm이 없어 곧바로 armed) / `running` / `paused` / `ended` / `latched` |
| `operator_link` | `{fresh, age_s, timeout_s, session(묶인 세션 id의 SHA-256 앞 8자), committed(그 세션이 이 run에서 go를 했다), go_active, go_serial, last_cancel, last_release, 횟수들(beats, gos, conflicts, invalid, link_lost, cancels, bindings, releases, takeovers)}` |
| `go_ready`, `go_blockers` | 3.2절 |
| `stop_reason` | 일시정지·끝의 분류: `ROW_END`, `OBSTACLE`, `END_STRUCTURE`, `PERCEPTION_OR_LOCALISATION`, `CONTROL_AUTHORITY`, `OPERATOR_OR_BACKSTOP`, `SYSTEM_FAULT`, `OPERATOR_LINK_LOST`, `UNCLASSIFIED`; 주행 중·대기 중은 null |
| `stop_detail` | 원래 이유 문자열(예: `operator_link_lost`, `row_end`, `stale_lidar`, `speed_cap_below_min`) |
| `speed` | `{target_mps, limit_mps, limit_reason, caps}`(5절) |
| `tuning` | `{preset, revision, editable, ...}`(6절) |
| `lease_lost` | 3.4절 상태일 때 `{t, where, error, bridge_reason, bridge_latches, steering_fault, run_id, client_latch}`, 아니면 null |

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
3a. `run_state`가 `ready`이고 `go_blockers`에 `lease_lost`가 있으면 '브리지가 권한을 거뒀다: Arm을 다시 누른다'를 보인다.
4. 409 `session_conflict`를 받으면 '다른 화면이 운영자'를 보이고 STOP만 둔다. heartbeat만 409인 동안(다른 화면이 go 전)에는
   '자율 주행 시작'을 눌러 go를 보내도 된다(개정 5: go가 묶음을 넘겨받는다).

## 8. 시험과 SIM 확인

- 단위·HTTP: `tools/row_executor/tests/test_one_press.py`(링크, 세션, 끊김, 재출발 없음, STOP 우선, MOTOR Arm 순서, HTTP 경로,
  개정 5의 run 끝·기한 풀림, heartbeat만 보낸 세션의 넘겨줌, conflict 기록),
  `test_steer_executor.py`(FirstArmTest), `test_field_tuning.py`, `test_steer_follow_runner.py`(속도 조절기 일시정지),
  `test_stop_lease.py`(일시정지 중 STOP의 분류, 브리지에 먼저 닿은 STOP, 기다리는 동안의 lease와 다시 Arm; 고치기 전에는 10개 중
  6개가 실패했다).
- 빠른 SIM(T1, 실행기 포함): 안전 대본 `operator_link_lost`, `operator_link_back_no_restart`, `other_screen_refused`,
  `one_press_stop`. 2026-11-13 결과: 7개(이 넷과 deadman_release, hold_lost, mid_row_stop_w0) 모두 기대대로.
  `operator_link_lost`: 마지막 heartbeat 4.95 s, 일시정지 5.50 s, 구동 명령 0 첫 trace 5.51 s(한도 5.55 s). 자리값 기반 SIM 결과.
- 호스트 도구: `python3 -B -m steering.exec_run start`(G-SIM 실행기 컨테이너 안)가 one_press 실행기에 세션 heartbeat와 go를 보낸다.
- 빠른 SIM 대본 `armed_wait_lease_lost`(A17 개정 34): 0.85 s부터 상태 줄 0.4 s 끊김 → 브리지 래치 → run 열린 채 →
  3 s에 다시 Arm → go → 3 m 주행. `two_screens_and_cli`, `late_drive_after_stop`의 기대 끝 이유는 `operator_stop`이다.
  2026-11-13 등록 실행: T1 안전 대본 71/71 기대대로(자리값 기반 SIM 결과, `EXECUTOR_STEER_KO.md` 13절 8항).
- G-SIM 점검(`steering.exec_checks`)은 실행기의 `start_mode`를 따른다. one_press(front_steer 기본)면 검사가 이 화면처럼
  세션 heartbeat 100 ms와 go 한 번으로 출발한다(`GoBeater`). `deadman_release`는 놓을 버튼이 없어 돌리지 않고 `not_run`에
  적는다. `hold_lost`는 화면 신호 멈춤이 되어 기대 일시정지가 `operator_link_lost`다. hold_to_run 실행기(설정
  `start_mode: hold_to_run`)에는 예전처럼 hold로 누른다. `--start-mode`는 확인만 한다(다르면 검사가 시작 전에 멈춘다).
