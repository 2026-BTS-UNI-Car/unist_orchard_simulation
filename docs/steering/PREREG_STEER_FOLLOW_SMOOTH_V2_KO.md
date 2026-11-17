# SMOOTH r5 열 추종 재판정 사전등록 v2 (steer_follow_smooth_r5_v2)

- 작성: 2026-11-17 (작업 sim-smooth-wide). 등록 번호 `steer_follow_smooth_r5_v2`. 기계가 읽는 사본: `PREREG_STEER_FOLLOW_SMOOTH_V2.json`
  (`follow_smooth2.py --prereg` 의 출력과 같아야 한다. 시험 `sim/ubts/test_steering_follow_smooth2.py` 가 본다).
- 이 문서는 판정에 쓰는 run 을 돌리기 전에 커밋한다. 앞 등록 [`PREREG_STEER_FOLLOW_SMOOTH_KO.md`](PREREG_STEER_FOLLOW_SMOOTH_KO.md)(v1)과 그 결과
  (`matrix` 15/15, `wide` 52/60)는 바꾸지 않고 그대로 둔다.
- 이 커밋까지 새 프로필로 돌린 것(판정 아님): 개발 seed 100~104 의 run([`SMOOTH_R5_DISTURBANCE_KO.md`](SMOOTH_R5_DISTURBANCE_KO.md) 5절), T1 단위 시험.
  판정 seed(matrix 0~4, wide 4000~4002)는 새 프로필로 돌린 적이 없다.
- 결과는 **자리값 기반 SIM 결과**다. 현장 성능이 아니다. 제안 부분은 **SIM 제안 r6p 의 효과**이며 차의 펌웨어 결과가 아니다(보드에 쓴 적 없음).

## 1. 왜 다시 재나

[`SMOOTH_R5_DISTURBANCE_KO.md`](SMOOTH_R5_DISTURBANCE_KO.md) 의 분석:

1. v1 의 두 외란 조건은 차의 보드(r5)에 없는 V3 MCU 감시(POT_NOISE, TRACK)로 멈췄고, SIM 호스트의 `steering_not_following` 은 V3 브리지 규칙(램프 조건,
   3 s)이었다. 차의 감시 묶음(보드에 결함 감시 없음, 실행기 ESP32 어댑터의 3 deg / 2 s, 램프 조건 없음)을 흉내 내는 프로필
   `front_steer_v2_1_field_20261117` 로 다시 잰다.
2. 그 밑의 원인은 SMOOTH 유지 자체(PWM_MAX 80 의 힘, 취소 뒤 풀림)라서, SIM 제안 r6p 프로필 `front_steer_v2_2_proposal_20261117` 도 같은 조건으로 잰다.
3. `operator_link_lost` 의 기준 0.55 s 는 결정 R60(2026-11-13, 운영자 연결 기한 0.5 → 1.0 s) 전 값이다. 합격 기준 A17 개정 60("마지막 운영자 신호 뒤 구동 0
   요청 ≤ `operator_timeout_s` + 1틱")을 따라 **1.05 s**(기한 1.0 s + 실행기 한 틱 0.05 s)로 맞춘다. 이 값은 v1 측정 전(2026-11-13)에 정해진 규칙에서
   나온다. v1 과 펄스 v1 의 측정값(1.05 s)을 보고 새로 만든 문턱이 아니다. 0.55 s 로 본 결과도 행마다 보고 칸(`v1_link_check`)으로 남긴다.

## 2. 무엇으로 재나

| 항목 | 값 |
|---|---|
| 시뮬레이터 | F-SIM T1 리그, 실행기 포함(`tools/row_executor/sim_in_loop_steer.py`). v1 과 같다 |
| 차 프로필(부분 `matrix`, `wide`) | `front_steer_v2_1_field_20261117`(스키마 v6): v2 + `steer_monitor_stack esp32_r5`, `host_track_mismatch` 3 deg / 2 s. 조향 법칙 `smooth_r5` 그대로 |
| 제안 프로필(부분 `proposal_matrix`, `proposal_wide`) | `front_steer_v2_2_proposal_20261117`: v2.1 + `steer_fw_mode smooth_r6p`, `smooth_fw_proposal` hold_pwm_max 200, push_counts 24, rest_brake 1, hold_no_cancel 1 |
| 실행기 설정 | 리그가 만드는 설정 그대로(`steer_executor_config`, SMOOTH 측정값 12.39 °/s, 문턱 바꿈 없음). v1 과 같다 |
| 제어기 | `{"kind": "row_follow", "wheelbase_m": 0.657}` |
| 발자국 | 1.22 × 0.70 m(자리값) |

## 3. 부분과 기준

| 부분 | 프로필 | run | 판정 |
|---|---|---|---|
| `matrix` | v2.1 | `steer_follow_v1` 의 11조건 15회(v1 과 같은 조건·seed) | 15회 모두 통과면 통과 |
| `wide` | v2.1 | `steer_follow_wide_v2` 의 20조건 × seed 4000, 4001, 4002 = 60회(v1 과 같다) | 조건마다 통과 수, 60회 모두 통과면 통과 |
| `proposal_matrix` | v2.2(제안) | `matrix` 와 같은 15회 | 15회 모두 통과면 통과 |
| `proposal_wide` | v2.2(제안) | `wide` 와 같은 60회 | 60회 모두 통과면 통과 |

- 기준: v1 과 같다(`follow_matrix.CRITERIA`, `follow_wide.CRITERIA`, 측정·판정 함수 그대로). 하나만 다르다: `operator_link_lost` 의 '마지막 heartbeat 뒤
  구동 0' ≤ **1.05 s**(v1 0.55 s). 외란 조건의 '조향 MCU 결함 줄 없음'과 '호스트 브리지 래치 없음'도 그대로다(v2.1 에서는 호스트의 어댑터 규칙 래치가 이에 걸린다).
- 순서: 한 잠금 세션에서 `matrix`, `wide`, `proposal_matrix`, `proposal_wide`. 일꾼 4.
- 이 등록의 판정은 두 개다: 차 모형(v2.1)의 판정(`matrix` + `wide`)과 제안(v2.2)의 판정(`proposal_*`). 제안의 통과는 차의 통과가 아니다.

## 4. 고칠 수 있는 것

- 반복 없음. 결과는 그대로 적는다. 실패는 원인 조사(등록 밖)와 다음 등록의 제안으로 남긴다.

## 5. 함께 적는 것 (판정 아님)

- 동일성: 같은 잠금 세션에서 main 54f9c34(분리 작업 트리)와 이 브랜치의 T1 안전 대본 71(직렬 기록 머리 줄 뺀 digest, trace), 빠른 SIM nominal 두 프로필
  (v0.1, v1)의 digest, 그리고 v2 프로필의 v1 등록 `matrix` 15 가 v1 판정 run 과 같은지.
- v1 결과와의 비교 표(조건별).

## 6. 이 등록이 보이지 않는 것

현장 성능, 땅에 닿은 조향, 실제 조향 모터 정지 토크(5 N·m 자리값)와 외란 크기, PWM 200 의 전류·열, 실기 REBOUND(링크 되튐)와 제안의 상호작용,
브리지와 어댑터의 두 겹 정책(SIM 은 하나로 합침), Gazebo.
