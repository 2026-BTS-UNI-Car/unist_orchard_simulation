# 최종 이미지 G-SIM 확인 사전등록 (steer_follow_gsim_final_v1)

- 작성: 2026-11-13 (묶음 S2 2단계, 브랜치 `gsim-final2-20261113`, main `0b859c2`에서). 등록 번호 `steer_follow_gsim_final_v1`.
- 기계가 읽는 사본: `PREREG_STEER_FOLLOW_GSIM_FINAL.json`(`steering.gsim_final.prereg()`의 출력과 같아야 한다. 시험
  `sim/ubts/test_steering_gsim_final.py`가 본다).
- 이 문서는 최종 이미지 `ubts-steer-sim:wf5`로 스택을 돌리기 전에 썼다. 이 커밋까지 wf5 이미지는 없고, 이 등록의 어느 항목도
  돌리지 않았다. 앞 등록은 `PREREG_STEER_FOLLOW_GSIM_KO.md`(`steer_follow_gsim_v1`, 이미지 wf4c, 누르는 동안만 방식)다.
- 바탕: 1단계 결과 `GSIM_GAPS_RESULT_KO.md`(호스트가 바쁘면 SIM 이 실시간보다 느려지고 실행기가 `stale_lidar`로 멈춘다),
  실행기 규약 `OPERATOR_API_KO.md`(한 번 눌러 시작, 운영자 신호 끊김 0.5 s), 합격 기준 `SIM_ACCEPTANCE_KO.md` A17(개정 31), A25.
- 모든 수치는 **자리값 기반 SIM 결과**다. 실차 성능이 아니다.

## 1. 공통

| 항목 | 값 |
|---|---|
| 이미지 | `ubts-steer-sim:wf5`(깨끗한 작업 트리에서 빌드, 매니페스트의 파일이 커밋 blob 과 같음, compose 기본 태그) |
| 시계 | 실차 시계 모드(`UBTS_STEER_CLOCK=wall`) |
| 시작 | 세션마다 스택을 새로 올린다. sim 컨테이너가 healthy 가 된 뒤 **120 s** 기다렸다가 첫 단계를 시작한다(1단계에서 healthy 뒤 1~2분 사이 RTF 가 0.5 까지 내려간 구간을 봄) |
| 호스트 | 세션 전 `STACK_OPERATIONS_KO.md` 2절의 부하 확인을 기록한다. 멈춤 탐침은 켜지 않는다 |
| 유효성 | A10 규칙(RTF, 에뮬레이터 `/clock` 벽시계 틈 100 ms, SIM 이 만든 호스트 래치)과 이 등록의 RTF 규칙: 검사 노드가 잰 5 s 창 RTF 가 하나라도 **0.9 미만이면 무효**. 무효는 기록하고 판정하지 않는다. 무효 run 은 새 스택에서 한 번 더 돈다. 두 번 무효면 '판정 못 함'으로 적는다 |
| 기준 | 아래 표 그대로. 측정 뒤에 바꾸지 않는다. 바꿀 일이 생기면 이 문서에 개정을 먼저 적고 커밋한다 |

## 2. A: 한 번 눌러 시작으로 열 끝까지

- 검사: `steering.exec_checks follow --run <이름> --out /runs/follow`. 검사가 운영자다: Start, Arm, 하트비트(100 ms)와 go 한 번.
  다시 go 하지 않는다. 실행기는 `start_mode` one_press(기본).
- 장면 w1_force(원본 과수원, seed 7), 제어기 `{"kind": "row_follow"}` 기본값, 실행기 설정과 현장 조정 파일 기본값.
- run 네 개: `nominal_1`, `nominal_2`, `nominal_3`(출발점 그대로), `lateral_p040`(set_pose 로 왼쪽 +0.40 m). run 마다 새 스택.
- 기준: `steer_follow_gsim_v1` 그대로(`steering.gsim_follow.CRITERIA`). 종결 이유 `row_end`, 주행 시작 뒤 일시정지 0회, 횡 RMSE ≤ 0.20 m,
  p95 ≤ 0.35 m, 최대 ≤ 0.50 m, 방향 RMSE ≤ 5°, 최대 ≤ 15°, 참 속도 최대 ≤ 0.50 m/s, 열 끝 정지 위치 e ∈ [−1, +1] m(마지막 줄기 선
  기준, R37), 최소 여유 > 0 m, 끝 속도 ≤ 0.02 m/s, guard 위반 종결 없음, 종결 뒤 3 s 명령 조향 이동 없음. `lateral_p040`은 RMSE·p95
  를 6.0 m 뒤부터.
- 판정: `steering.gsim_final.judge_follow`(기록 `GSIM_FOLLOW_RUN.json`). 통과 = 판정된 run 이 모두 모든 기준 통과, 그리고 네 자리 중
  셋 이상 판정됨.
- `nominal_3`에서만 차량 카메라 프레임을 받는다(영상, DOC-15). 결과에 적는다.
- 참고(판정 아님): 결정 R50(열 끝 = 마지막 구조물)과 넓은 조건 평가 2차의 `row_end_agree_m` 1.0 m. e 는 R37 정의(마지막 줄기 선)로 잰다.
  1단계에서 e 가 +0.45 m 로 끝 기둥 쪽인 것을 보았다.

## 3. B: SIM 대시보드 화면에서

- 스택: sim, steer, recorder, dashboard, executor(사용자 보기 스택과 같은 다섯 서비스), 장면 w1_force. 화면은 Chrome(playwright)으로
  운영자처럼 누른다(저장소 밖 대본 `runs/gsim_final_20261113/scripts/sim_e2e_browser.py`, 묶음 S5 대본을 고친 것).

| 항목 | 기준 |
|---|---|
| 준비 | 자율 칸이 '준비'를 보이고, 누르면(화면이 Arm 을 내밀면 그것도 누름) 16 s 안에 '자율 주행 시작'. 두 번 눌러야 했으면 적는다 |
| 한 번 눌러 주행 | go_ready 뒤 한 번 누르면 10 s 안에 '주행 중', 차가 움직인다(에뮬레이터 속도 > 0.05 m/s) |
| 하트비트 | 탭이 보이는 동안 마지막 20 간격 중 가장 긴 것 < 0.35 s |
| 탭 숨김 | 숨기고 1.5 s 에 run_state paused, stop_reason OPERATOR_LINK_LOST, 숨긴 뒤 하트비트 0, 4 s 에 차 정지(< 0.05 m/s). 다시 보이면 '다시 출발'이고 여전히 paused(자동 재출발 없음) |
| 주행 중 화면 STOP | 새 run, 한 번 눌러 6 s 주행, 상태 막대 STOP: stop_detail operator_stop(stop_reason OPERATOR_OR_BACKSTOP), 3 s 안에 차 정지, 다시 가지 않음 |
| 실시간 지도 | 화면에서 '그리기 시작' 뒤 20 s 에 `/api/live/map` scan_count > 0(SIM LiDAR 브리지의 원본 스캔, `af7a1c3`) |

- 유효성: 1절. 판정 = 모든 항목 통과.

## 4. C: A17(G) 실행기 대본 (one_press)

- 검사: `steering.exec_checks safety --out /runs/safety`, 장면 w0_force, 실행기 one_press. 대본별 기준은 `SIM_ACCEPTANCE_KO.md` A17
  (개정 31: 운영자 연결 끊김 -> 구동 0 송신 ≤ 0.5 s + 1틱)을 `exec_checks`에 적힌 그대로. `deadman_release`는 one_press 에 없다.
- 유효성: 대본마다 A10 규칙(`exec_checks`). 무효 대본은 `--only`로 한 번 더. 두 번 무효면 적는다.
- 판정: 유효한 대본이 모두 통과.

## 5. D: A25 (에뮬레이터 경로, 새 브리지 판정)

- 검사: `steering.p6a_checks a25 --out /runs/a25`, 장면 w0_force. 최종 이미지의 호스트 브리지(개정 43: 서보 램프가 목표에 닿은 줄만
  `steering_not_following`에 셈). 기준은 `p6a_checks`에 적힌 그대로(가장 좋은 지연에서 관절 − 공유 모델 p99 ≤ 0.2°, 관절 속도 튐 없음,
  조향이 움직임, drive 거절 없음). 유효성 A10. 판정 = 모든 기준 통과.

## 6. E: PW-08 Gazebo 쪽

- 검사: `steering.exec_checks pw08 --out /runs/pw08`, 장면 w0_force. 정지 상태에서 현장 조정 값의 표 배율 `speed_scale`을
  1.00 → 1.10(`POST /api/params`). 앞뒤로 대본 제어기 50 % 직진 8 m 를 한 번씩 돌린다.
- 실행기 쪽(판정): 요청 200, 상태의 값 1.10, revision +1 과 새 digest, 다음 run 의 start 사건이 새 digest, 50 % 에서 실행기 표 속도의
  비 1.10 ± 1 %, 끝에 기본 프리셋으로 되돌려 처음 digest.
- 플랜트 쪽(보고): 50 % 에서 Gazebo 플랜트 정상 속도(참값, 움직이기 시작한 뒤 3~7 s 평균)의 앞뒤 비. Gazebo 의 구동은 프로필의 물리
  모형이고 조정 파일을 읽지 않으므로 바뀌지 않을 것으로 본다. 판정하지 않고 값을 적는다.
- 유효성: 두 주행 모두 A10. 판정 = 실행기 쪽 기준 모두 통과.

## 7. 결과를 적는 곳

- `STEER_FOLLOW_GSIM_FINAL_KO.md`와 `STEER_FOLLOW_GSIM_FINAL.json`, 그림·영상(DOC-15)은 `docs/assets`의 매니페스트 규칙대로.
- `docs/RESULTS_KO.md` 1절의 'Gazebo 최종 확인' 줄과 README 결과 요약 마지막 줄을 실제 결과로 고친다.

## 8. 개정

### 개정 1 (2026-11-13, 첫 세션들 06:50~08:10 UTC 뒤, 어느 세션도 다시 돌리기 전에 커밋)

1. **C(A17(G) one_press)는 다시 돌리지 않고 '판정 못 함'으로 적는다.** 세션 `s2cde_w0`(장면 w0_force)에서 대본 25개가 모두 시작
   단계의 go 거절(`corridor_not_acquired`)로 끝났다. 한 번 눌러 시작 실행기는 row 종류 run 의 go 를 통로를 잡기 전에는
   거절하고(`executor_main._go_blockers`, 설계 v2 V2-2 2항), Gazebo W0 는 나무 열이 없는 평지라 통로가 생기지 않는다. 등록 때
   놓친 조건이다(등록 오류). W1 에서 돌리려면 대본마다 출발점으로 되돌리는 검사 코드 변경이 필요하고, 대본의 조향(3~5°, 수십 초)이
   나무 열 쪽으로 간다. 다음 등록 몫으로 둔다.
2. **E(PW-08)는 장면을 w1_force 로 바꿔 새 스택에서 한 번 돈다.** 기준·판정은 6절 그대로. W0 시도(08:09 UTC)는 첫 주행의 go 가
   거절됐다(`lease_lost`, `arm_first`, `corridor_not_acquired`). W1 출발점에서 50 % 직진 8 m 를 두 번(두 번째는 첫 주행이 선 곳에서)
   달리면 열(30 m) 안이다.
3. **B 시도 1(`s2b_ui`)은 등록 규칙으로 무효다**: 검사 노드 RTF 5 s 창 최소 0.863(< 0.9), 그 창이 화면 단계 안에 있다. 등록대로 새
   스택에서 한 번 더 돈다. 시도 1 의 대본은 실시간 지도 단계에서 멈췄다(대시보드 90094fd 는 실시간 지도를 `/map` 화면에 두는데
   대본은 주행 화면에서 지도 칸을 찾았다). 다시 돌 때 대본은 그 단계 전에 `/map` 으로 간다. 판정하지 않는 참고 단계 하나를 더한다:
   주행 화면 LiDAR 보기와 `/sensors`의 IMU 칸 갱신(90094fd 는 그 두 곳에서만 LiDAR·IMU 소켓을 연다). 항목·기준·순서(view, auto,
   stop, map)는 그대로다.
4. 관찰(판정에 쓰지 않음): B 시도 1 의 STOP 단계에서 새 run 의 go 가 `vehicle_moving`으로 거절됐다. 실행기 틱에서 Arm 된 동안
   (구동 0) 차가 뒤로 0.02~0.07 m/s 로 움직였다(Disarm·래치 중에는 정지).
