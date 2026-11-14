# 구동기 사전값 층(L1, PC-06)

- 작성: 2026-11-13 (묶음 S1, 브랜치 `autonomy-exec-20261113`). 추정기 쪽 입력(PC-05)은 `ESTIMATOR_INPUTS_KO.md`.
- 설계: `DESIGN_v2_KO.md` V2-1 2항, 결정 R31. 확인 기준: `SIM_ACCEPTANCE_KO.md` 개정 33(커밋 13d5c1f, 첫 등록 실행보다 앞섬).
- PWM은 엔코더가 아니다. 가변저항은 조향 링크 각의 측정값이다. 둘 다 바퀴 속도를 재지 않는다. 사전값은 짧은 구간과 감시에만 쓴다.
- 모든 수치는 자리값 기반 SIM 결과이며 현장 성능이 아니다.

## 1. 실행기에 넣은 것

| 설정 키 | 기본 | 뜻 |
|---|---|---|
| `prior_enabled` | null(프로필 기본) | 조향 상태마다 MCU가 보고한 적용 duty(상태의 핀 값 D8−D5, D3−D0, −255..255 → %)와 보정 조향각을 추정기에 넣는다. null 이면 front_steer 에서 켜지고(가중 0 = 기록·온라인 배율 감시만, PW-10, 4절) 차동에서 꺼진다. true/false 로 정할 수 있다. 2026-11-14 전까지 기본은 false 였다 |
| `prior_weight` | 0 | 0 = 기록·감시만. (0, 1] = 스캔 정합 시드와 정합 실패·스캔 틈의 한정된 예측 |
| `prior_speed_table`, `prior_speed_scale` | () = vendor 표, 1.0 | duty→속도 표와 k_field. 현장 조정 파일이 넣는다(실행기 설정에 따로 적으면 설정이 앞섬) |
| `prior_wheelbase_m` | null | 요레이트 사전값 v·tan δ / L |
| `prior_predict_max_s`, `prior_predict_max_m` | 1.0 s, 0.5 m | 예측 한도(잠금). 넘으면 `prediction_limit` 일시정지(신선도 일시정지와 합쳐 셈) |
| `prior_mismatch_fraction` | 0.2 | 표 속도와 LiDAR 속도 비가 이만큼 어긋나면 '표 어긋남' 표시(그것만으로 서지 않음) |
| `corridor_quality` | false | 추정기의 통로 품질 블록 |

- 요청값은 쓰지 않는다. 조향 상태가 0.3 s보다 늦으면 사전값을 끄고 LiDAR만 쓴다.
- 손대지 않은 묶음은 설정 digest에서 빠진다(기존 설정 digest 그대로, 시험으로 고정).
- 틱 기록 `prior`(fresh, duty, v, w, k(온라인 배율), mon, mismatch, pred, ps, pm, lim, d), 상태 `actuator_prior`.
- 원래 작업: 추정기 묶음의 `pc06_wip` 패치(`runs/pc_estimator_20261112/pc06_wip/`). one_press 변경과 겹친 hunk(config 5, core 2, T1 1)는 손으로 넣었다(내용 같음).

## 2. 확인(개정 33)

`python -B -m steering.safety_scripts.prior_checks --out <밖> --workers 3`(sim/ubts에서). T1 빠른 SIM, W0 기둥 두 줄, 대본 제어기
straight_hold 40 %, 10 m, 60 s, seed 0~4, seed마다 run 7종.

등록 실행(2026-11-13, export 88d3ceb, 작업자 1, 1038 s, 다른 작업이 같은 PC에서 돌던 때): **55 검사 모두 합격.**

- 첫 실행(export 8ac107b)은 seed 0의 off·monitor 뒤 table_ok에서 설정 오류로 멈췄다(SIM 설정 함수가 list로 온 표를 받지
  못함). 판정 전이다. 고친 커밋 88d3ceb 뒤 처음부터 다시 돌렸다. 기준과 판정 코드는 바꾸지 않았다.

| seed | 보정 배율(monitor 온라인) | 동일성(가중 0) | 표 ×1.0 / ×1.3 / ×0.7 어긋남 | blackout 예측 오차 [m] | 참 / 예측 이동 [m] | 가중 0 오차(비교) [m] | 사전값만 주행 틱 |
|---|---|---|---|---|---|---|---|
| 0 | 1.011 | 같음 | 꺼짐 / 켜짐 / 켜짐 | 0.041 | 0.224 / 0.183 | 0.116 | 0 |
| 1 | 0.912 | 같음 | 꺼짐 / 켜짐 / 켜짐 | 0.034 | 0.198 / 0.165 | 0.078 | 0 |
| 2 | 1.074 | 같음 | 꺼짐 / 켜짐 / 켜짐 | 0.074 | 0.268 / 0.195 | 0.068 | 0 |
| 3 | 0.901 | 같음 | 꺼짐 / 켜짐 / 켜짐 | 0.059 | 0.222 / 0.163 | 0.054 | 0 |
| 4 | 1.027 | 같음 | 꺼짐 / 켜짐 / 켜짐 | 0.038 | 0.222 / 0.186 | 0.060 | 0 |

- 한도: 예측 오차 ≤ 0.15 m(최대 0.074 m), 사전값만 주행 틱 0(blackout의 가장 긴 예측 시간 0.43 s).
- 표 ×1.3이면 온라인 배율 약 0.77, ×0.7이면 약 1.43으로 '표 어긋남'이 켜졌다.
- 1.0 s LiDAR 끊김에서 차는 16.3 s에 `stale_lidar`로 선다(스캔 0.45 s 문턱). 예측은 그 정지까지의 이동을 다룬다.
- 가중 0(기존 추정)과 비교하면 seed 0·1·4는 오차가 줄고 seed 2·3은 비슷하거나 조금 늘었다. 사전값 예측이 늘 낫다고 할 수 없다.
- 결과 원본은 기록 폴더 `runs/autonomy_exec_20261113/measure/prior_checks_v2/PRIOR_CHECKS.json`(레포 밖).

## 3. 한계

- 표는 조향 장치를 달기 전 차의 값이다. T1의 '실측' 표는 프로필 공칭값(SIM_ASSUMED)이다.
- 예측 확인은 W0 평지의 직진 40 %에서 1.0 s 끊김 하나다. 굽은 열, 경사, 미끄러짐은 묶음 S1c의 조건 행렬에서 본다.
- 가중 > 0은 기본이 아니다. 켜는 결정은 개정 33 통과와 넓은 행렬 뒤에 한다.

## 4. 온라인 배율 표시와 기록(PW-10)

- 작성: 2026-11-14 (브랜치 `pw10-online-scale-20261114`). todo PW-10: 달리는 동안 약 20 초 평균 배율(LiDAR 속도 ÷ 표 속도)을
  화면에 보이고 기록한다. 주행 뒤 전체 배율로 저장할지 묻는다. LiDAR 속도가 품질 기준을 못 넘으면 갱신하지 않는다.
- 모든 수치는 자리값 기반 SIM 결과이며 현장 성능이 아니다. 배율은 배터리와 지면이 합쳐진 값이라 원인을 나누지 못한다.

### 4.1 무엇을 바꿨나

- 감시는 원래 있었다(1절의 `(c) monitor`): 스캔 정합이 된 구간만, 사전값 속도 0.1 m/s 이상인 주행 구간만 창에 넣고,
  온라인 배율 = 창(주행 시간 20 s) 안의 LiDAR 이동 ÷ 사전값 이동, 주행 10 s 뒤부터 값이 있다. 사전값 속도 = 표(적용 duty) ×
  `prior_speed_scale`(= `speed_scale` × `surface_scale` × `battery_scale`). 그래서 배율이 0.85 면 차가 표가 말하는 속도의 0.85 로 간다.
- 기본 켜짐(결정 필요, 조정 쪽 확인): 2026-11-14 전에는 `prior_enabled` 기본이 false 라 현장 실행기에서 배율이 나오지 않았다.
  이제 기본 null = 프로필 기본(`config.prior_active`)이고 front_steer 에서 가중 0 으로 켜진다. 가중 0 은 추정 결과를 바꾸지 않는다
  (1절, 개정 33의 동일성 판정과 같은 성질). 출력이 같은지는 회귀로 다시 본다(4.4). 설정 digest: 손대지 않은 묶음은 그대로
  빠진다(`ExecutorConfig()` digest 그대로). front_steer 설정은 조정 파일이 표를 넣어 묶음이 이미 들어가므로 digest 가 바뀐다
  (`prior_enabled` false → null). 끄려면 실행기 설정에 `prior_enabled: false`. `prior_checks`의 off run 은 false 를 적었다.
- 품질 기준을 못 넘으면 멈춤: 창은 스캔 정합(맞춤 비율 0.5 이상, 스캔 틈 0.5 s 미만)이 된 구간에서만 바뀐다. 새 칸
  (추정기 스냅샷 `actuator_prior`): `online_scale_frozen`(값이 있고 마지막 갱신이 `FROZEN_AFTER_S` 0.5 s 보다 오래됨),
  `online_scale_hold`(`lidar_stale` 새 스캔 없음, `lidar_unmatched` 가장 새 스캔이 정합 실패, `not_driving` 사전값이 감시 속도 아래,
  `no_status` 상태 줄 없음), `online_scale_age_s`, `monitor_window_s`, `monitor_min_s`, `unmatched_scans`. 0.5 s 는 느린 호스트가
  스캔을 건너뛸 때 깜박이지 않게 고른 값이다(자리값).
- 기록: 실행기 틱 기록 `prior` 에 `k`(배율), `mon`(창 길이), `fz`(멈춤), `hold`(이유)가 매 틱(20 Hz) 남는다 = 시간열.
  run 이 끝나면 사건 `online_scale_offer`(run, 배율, 창 길이, 끝 이유, 지금 k_field, 제안 k_field, 받을 수 있는지와 까닭).
- 상태 `online_scale`(운영자 HTTP, `OPERATOR_API_KO.md` 6.1): `enabled, run_id, value, window_s, target_s, min_s, frozen, hold,
  age_s, mismatch, speed_scale, prior_scale, offer`.
- 저장 묻기: 제안 k_field = `prior_speed_scale` × 배율 ÷ (`surface_scale` × `battery_scale`), 소수 3 자리(지형·배터리 배율은 그대로
  두고 k_field 만 고친다). 받지 못하는 까닭: `no_scale`(주행 10 s 미만), `no_forward_motion`, `scale_set_in_config`(실행기
  설정이 `prior_speed_scale`을 따로 정함), `out_of_range`(k_field 0.5~1.3, 곱 0.5~1.3), `no_change`. 답은
  `POST /api/params {"online_scale": "accept"|"dismiss", "run_id": N}`. accept 는 정지 상태에서만(다른 조정 값과 같은 잠금),
  그 run 의 제안일 때만, run 뒤 배율 값이 바뀌지 않았을 때만 받고 `speed_scale` 을 화면 변경과 같은 길로 바꾼다(`tuning_change`
  사건 action `online_scale`, `source_run_id`, `online_scale`, `window_s`; revision +1; 파일 저장은 따로 `{"save": true}`).
  dismiss 는 바꾸지 않고 `online_scale_answer` 사건만. 새 run 이 시작되면 답하지 않은 제안은 `superseded` 로 기록하고 지운다.
- 코드: `sim/ubts/closed_loop/actuator_prior.py`(vendor 사본 같음), `tools/row_executor/online_scale.py`, `executor_main.py`
  (`_track_online_scale`, `_req_online_scale`), `config.py`(`prior_active`). 대시보드 서버 중계: 대시보드 레포
  `dashboard_server/executor_proxy.py`(상태 칸), `steer_tuning.py`(답 형식과 기록). 화면 표시와 저장 묻기 화면은 대시보드 메뉴를
  되돌린 뒤 자리를 정해 따로 한다(2026-11-14 조정 쪽 지시).

### 4.2 사전등록(측정 전, 이 절과 `PREREG_PW10_ONLINE_SCALE.json`이 등록 run 보다 앞선 커밋)

- 장치: T1 빠른 SIM, 실행기 포함(`steering.safety_scripts.online_scale_checks`). 실행기는 기본 설정(위의 기본 감시), 조정 파일
  패키지 기본, 넓은 평가의 열 추종 제어기와 한 번 눌러 시작(`follow_wide.spec`의 nominal 열). 플랜트 구동 표 = 조정 파일 표 × 0.85
  (넓은 평가의 `table_x085`와 같은 장치). seed 100~104.
- 화면 값: 실행기 상태 `online_scale`(대시보드가 그대로 넘기는 칸)을 SIM 시간 0.5 s 마다 읽는다.
- run 3 종:
  - exact: 프로필 v0.1.2 에서 속도 흔들림(`speed_fluctuation_sd`)과 조향 scrub 손실을 0 으로(플랜트 정상 속도가 표 × 0.85 와 같다). 열 끝까지.
  - cut: exact + LiDAR 끊김 1.0 s(30.0 s, 리그 결함 sensor_blackout). 실행기는 stale_lidar 로 일시정지하고 새 go 가 없어 다시
    가지 않는다. 60 s 까지 본다.
  - wide: 등록된 `table_x085` 그대로(기본 플랜트, 흔들림 5 %, 2 s).
- 판정(모두, 모든 seed):
  1. exact.converged: 창이 찬 뒤(창 ≥ 20 − 0.2 s)부터 run 끝까지 읽은 화면 값 모두와 run 뒤 제안의 배율이 0.85 ± 0.03, 읽은 값 10 개 이상.
  2. exact.series: 첫 값 뒤 run 끝까지 모든 실행기 틱에 `prior.k` 가 있다(시간열).
  3. cut.frozen: 30.3 s(끊기기 전 마지막 스캔이 처리된 뒤)부터 끝까지 모든 틱의 `k` 가 30.3 s 의 값과 같고(None 아님), 30.6 s 부터
     읽은 화면 값이 모두 frozen 이며 값이 같고, 끊긴 동안의 틱 하나 이상이 hold `lidar_stale` 이다.
  4. wide.tracking: 창이 찬 뒤 읽은 화면 값마다 |값 − SIM 참값 비| ≤ 0.03. 참값 비 = 같은 길이의 마지막 주행 창에서 Σ(참 속도·dt) ÷
     Σ(MCU duty 의 표 속도 × 사전값 배율 · dt)(표 속도 0.1 m/s 이상인 trace 표본). 평가만(A13 역할 evaluator).
- 보고만(판정 아님): wide 의 마지막 값·제안과 0.85 의 차이(플랜트 자체가 20 s 창마다 흔들린다), 제안 값, 끝 이유.
- 까닭(측정 전 개발 확인, 등록 밖 seed 7 과 고치기 전 코드의 seed 100~103): 온라인 배율은 SIM 참값 비를 0.01 안에서 따라가지만,
  기본 플랜트의 참값 비가 20 s 창마다 0.80~0.91 로 흔들렸다(seed 100 마지막 값 0.895). 그래서 '0.85 로 수렴'은 흔들림을 끈
  플랜트로 판정하고, 기본 플랜트는 참값 추종으로 판정한다. 기준 0.85 ± 0.03 은 todo 그대로다.
- 시험: `sim/ubts/test_steering_online_scale.py`(개발 seed 7, 짧은 run), `tools/row_executor/tests/test_online_scale.py`.
