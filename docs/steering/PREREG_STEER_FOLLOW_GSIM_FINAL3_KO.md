# 최종 이미지 G-SIM 확인 3단계 사전등록 (steer_follow_gsim_final3_v1)

- 작성: 2026-11-13 (묶음 S2 3단계, 브랜치 `gsim-final3-20261113`). 등록 번호 `steer_follow_gsim_final3_v1`.
- 기계가 읽는 사본: `PREREG_STEER_FOLLOW_GSIM_FINAL3.json`(`steering.gsim_final3.prereg()`의 출력과 같아야 한다. 시험
  `sim/ubts/test_steering_gsim_final3.py`가 본다).
- 이 문서는 등록 run 을 하나도 돌리기 전에 커밋한다. 그 전에 SIM 시계 모드가 도는지 개발 확인(`s3_smoke`, 판정 아님)만 했다:
  A13(G) 그래프 검사 통과(SIM 시계, 실행기 노드가 `/clock`을 구독), IMU 자세 나옴, 대시보드 실시간 지도 scan_count 182·cell_count 90730,
  A17(G) 세 대본 중 `hold_lost`가 '차 정지'에서 실패(W1 경사 밀림 → 개정 62), 열 추종 한 run 은 9.7 s 에 `stale_lidar` 일시정지
  (5 s 창 RTF 0.77~0.89, 실행기 루프 최대 188 ms, 그 순간 호스트 감시가 2.5 s 끊김).
- 앞 등록 `PREREG_STEER_FOLLOW_GSIM_FINAL_KO.md`(2단계, 이미지 wf5)와 그 결과는 그대로 둔다.
- 바뀐 것(결정 R60~R63, 합격 기준 개정 60·61): 이미지 `ubts-steer-sim:wf6`(운영자 연결 기한 1.0 s, 출발 정지 문턱 0.08 m/s,
  IMU 게임 회전 벡터, 대시보드 사본 `a92cfec`), 판정은 **SIM 시계 모드**, 벽시계는 부하 기록, A17(G) one_press 는 W1 에서 대본마다
  출발점으로 되돌림, 사용자 로컬 명령(sim_stack.py, sim_auto.py) 3 run 이 열 끝까지.
- 모든 수치는 **자리값 기반 SIM 결과**다. 실차 성능이 아니다.

## 1. 공통

| 항목 | 값 |
|---|---|
| 이미지 | `ubts-steer-sim:wf6`(이 등록을 넣은 커밋의 깨끗한 작업 트리에서 등록 run 전에 빌드, 매니페스트 파일이 커밋 blob 과 같음, compose 기본 태그) |
| 시계 | SIM 시계 모드(`UBTS_STEER_CLOCK=sim`: 브리지 wrapper 와 실행기가 신선도·기한을 SIM 시간으로 잰다). 개정 61 |
| 시작 | 세션마다 새 스택, sim 컨테이너 healthy 뒤 **120 s** 기다린 뒤 첫 단계 |
| 유효성 | SIM 시계라 RTF 가 1 보다 작아도 run 은 유효하다. 판정 단계의 5 s 창 RTF 를 모두 적는다. 5 s 창 RTF 가 **0.5 미만**이면 호스트 고장으로 보고 판정하지 않고 새 스택에서 한 번 더 돈다. 두 번이면 '판정 못 함' |
| 호스트 | 세션 전 `STACK_OPERATIONS_KO.md` 2절 부하 확인을 기록한다(최대 60 s 기다림). 멈춤 탐침은 켜지 않는다 |
| 기준 | 아래 그대로. 측정 뒤에 바꾸지 않는다. 바꿀 일이 생기면 8절에 개정을 먼저 적고 커밋한다 |

## 2. A: 열 추종 4 run (SIM 시계)

- 검사 `steering.exec_checks follow --run <이름> --out /runs/follow`(Start, Arm, heartbeat 100 ms, go 한 번). 장면 w1_force, 제어기
  `row_follow` 기본값, 실행기 설정·현장 조정 파일 기본값. run 네 개 `nominal_1`, `nominal_2`, `nominal_3`, `lateral_p040`(+0.40 m),
  run 마다 새 스택.
- 기준: `steer_follow_gsim_v1` 그대로(종결 `row_end`, 주행 뒤 일시정지 0, 횡 RMSE ≤ 0.20 m, p95 ≤ 0.35 m, 최대 ≤ 0.50 m, 방향 RMSE ≤ 5°,
  최대 ≤ 15°, 참 속도 최대 ≤ 0.50 m/s, e ∈ [−1, +1] m, 최소 여유 > 0, 끝 속도 ≤ 0.02 m/s, guard 위반 없음, 종결 뒤 3 s 명령 조향
  이동 없음).
- 판정 `steering.gsim_final3.judge_follow`. 통과 = 판정된 run 이 모두 통과, 네 자리 중 셋 이상 판정.
- `nominal_3`에서 차량 카메라 프레임을 받는다(영상).

## 3. W: 벽시계 부하 기록 (판정 아님)

- 같은 검사를 벽시계 모드로 한 스택에서 `nominal_1`, `lateral_p040` 차례로 돈다. 판정하지 않고 RTF·실행기 루프 시간과 함께 적는다.

## 4. B: SIM 대시보드 화면 (SIM 시계)

스택 sim, steer, recorder, dashboard, executor, 장면 w1_force. Chrome(playwright)으로 운영자처럼 누른다(저장소 밖 대본
`runs/gsim_final_20261113/scripts/sim_e2e_browser.py`, 단계 view, auto, stop, restart, map, `LINK_CHECK_S=2.0`, `HEARTBEAT_GAP_MAX_S=0.5`).

| 항목 | 기준 |
|---|---|
| 준비 | 자율 칸이 '준비'를 보이고, 누르면(Arm 을 내밀면 그것도) 16 s 안에 '자율 주행 시작' |
| 한 번 눌러 주행 | 한 번 누르면 10 s 안에 '주행 중', 차가 움직인다(에뮬레이터 속도 > 0.05 m/s) |
| 하트비트 | 탭이 보이는 동안 마지막 20 간격 중 가장 긴 것 < 0.5 s(기한 1.0 s 의 절반) |
| 탭 숨김 | 숨기고 2.0 s 에 run_state paused, stop_reason OPERATOR_LINK_LOST, 숨긴 뒤 heartbeat 0, 4 s 에 차 < 0.05 m/s, 다시 보이면 '다시 출발'이고 여전히 paused |
| 주행 중 화면 STOP | 새 run, 한 번 눌러 약 6 s 주행(> 0.05 m/s), 상태 막대 STOP: stop_detail operator_stop(OPERATOR_OR_BACKSTOP), 3 s·6 s 뒤 < 0.05 m/s |
| STOP 뒤 다시 시작 | 그 STOP 뒤 래치 확인, 새 run 준비, 한 번 누르면 6 s 안에 다시 달린다(> 0.05 m/s). STOP 이 다시 세운다(operator_stop, 3 s 뒤 < 0.05 m/s) |
| 실시간 지도 | `/map`에서 '그리기 시작' 뒤 20 s 에 `/api/live/map` scan_count > 0, cell_count > 0 |

판정 `steering.gsim_final3.judge_browser`. 통과 = 모든 항목 통과.

## 5. C: A17(G) 실행기 대본 (one_press, W1, SIM 시계)

- 검사 `steering.exec_checks safety --out /runs/safety --reset-pose --still-speed 0.05`(대본마다 끝난 run 을 정리하고 차를 출발점에
  놓은 뒤 시작). 기준은 `SIM_ACCEPTANCE_KO.md` A17 을 `exec_checks`에 적힌 그대로, 개정 60(운영자 연결 끊김 → 구동 0 ≤
  operator_timeout_s + 1틱), 개정 62(W1 에서 '차 정지'는 참 속도 < 0.05 m/s).
  `deadman_release`는 one_press 에 없다.
- 통과 = 모든 대본 통과. 오류로 멈춘 대본(예: go 거절)은 실패로 센다.

## 6. E: PW-08 (W1, SIM 시계)

- 검사 `steering.exec_checks pw08 --out /runs/pw08`. 기준은 2단계 등록 E 의 실행기 쪽 그대로(되돌림은 상태가 바뀐 뒤 읽는다: wf6 의
  검사 코드). 플랜트 정상 속도 비는 보고.
- 통과 = 실행기 쪽 기준 모두 통과.

## 7. F: 사용자 로컬 명령으로 열 끝까지 3 run

- 사용자가 쓰는 명령 그대로(저장소 밖 로컬 도구 폴더 `tools/local_env/`): `python -B sim_stack.py start --no-browser`(이미지 wf6, 기본 SIM 시계), 150 s 뒤
  `python -B sim_auto.py start`를 돌리고 끝나면 `python -B sim_auto.py reset`, 20 s. 이것을 세 번.
- 기준: 세 run 모두 열 끝에서 끝난다(실행기 기록 run_end 의 end_reason `row_end`). 거리·시간은 보고.
- 판정 `steering.gsim_final3.judge_sim_auto`. 통과 = 3/3.

## 8. 개정

### 개정 1 (2026-11-13, A·W·B·C·E 세션(09:29~10:31 UTC) 뒤, B 를 다시 돌리기 전과 F 전에 커밋)

1. **B 의 평가 결함 두 가지.** ① 대본의 속도 검사가 `abs(v or 9) < 0.05`라 에뮬레이터 속도가 정확히 0.0 이면 움직이는 것으로
   셌다(시도 1 의 'screen STOP: no restart (3 s later)', 'restart: the STOP stops it again', 값 v 0.0). ② heartbeat 시각을 대본 자신의
   사건 처리기가 돈 때로 쟀는데, 대본이 그 처리기를 동기 `docker exec`로 막았다(시도 1 최대 틈 1.926 s, 그 동안 실행기는 연결 끊김을
   보지 않았다). ①은 같은 기록의 값으로 다시 판정한다. ②는 기록으로 판정할 수 없다. 그래서 대본을 고쳐(속도는 None 일 때만 '모름',
   heartbeat 시각은 브라우저의 요청 시작 시각) 새 스택에서 B 를 한 번 더 돈다. B 의 판정은 시도 2 이고, 시도  1 은 그대로 적는다.
2. **C 의 평가 결함.** `second_screen_and_cli`의 끝 이유 기대값이 아직 `bridge_error` 또는 `bridge_latched`였다. 이 등록보다 먼저
   등록된 합격 기준 개정 34 는 다른 클라이언트의 STOP 이 먼저 브리지에 닿아도 run 끝 이유를 `operator_stop`으로 정했다(T1 대본은 그때
   바꿨고 이 G-SIM 검사는 놓쳤다). 같은 기록을 `operator_stop` 기대로 다시 판정하고 `exec_checks`를 고친다. 다른 대본은 기록대로.
