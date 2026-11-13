# 최종 G-SIM 확인 4단계 사전등록 (steer_follow_gsim_final4_v1)

- 작성: 2026-11-13 (묶음 S2 4단계, 브랜치 `gsim-final4-20261113`, main `b00b666`에서). 등록 run 을 하나도 돌리기 전에 커밋한다.
- 기계가 읽는 사본: `PREREG_STEER_FOLLOW_GSIM_FINAL4.json`(`steering.gsim_final4.prereg()`와 같아야 한다. 시험
  `sim/ubts/test_steering_gsim_final4.py`).
- 2단계(`PREREG_STEER_FOLLOW_GSIM_FINAL_KO.md`)·3단계(`PREREG_STEER_FOLLOW_GSIM_FINAL3_KO.md`) 등록과 결과는 그대로 둔다.
- 바뀐 것: 이미지 `ubts-steer-sim:wf7`(main `b00b666`의 실행기 처리 시간 작업: 걸음의 입력 처리가 조향 상태를 0.15 s 넘게 낡게 하면
  틱 전에 브리지 상태를 한 번 새로 읽음(0.3 s 한도 그대로), 루프 전 `gc.freeze()`, compose 의 steer·executor CPU 3. 근거
  `EXECUTOR_PERF_KO.md`). 대시보드 사본은 `a92cfec` 그대로.
- 모든 수치는 **자리값 기반 SIM 결과**다.

## 1. 공통

3단계 등록 1절 그대로(SIM 시계, 세션마다 새 스택, healthy 뒤 120 s, 판정 단계 5 s 창 RTF 0.5 미만은 호스트 고장으로 한 번 더,
부하 확인 기록). 이미지는 `ubts-steer-sim:wf7`(이 등록을 넣은 커밋의 깨끗한 작업 트리에서 등록 run 전에 빌드, 매니페스트 =
커밋 blob, compose 기본 태그). 조용한 호스트(다른 무거운 작업 없음, 무거운 잠금).

## 2. A: 열 추종 4 run (W1, SIM 시계)

3단계 A 와 같다(`exec_checks follow`, `nominal_1`, `nominal_2`, `nominal_3`, `lateral_p040`, run 마다 새 스택, `steer_follow_gsim_v1`
한도). 통과 = 판정된 run 이 모두 통과, 네 자리 중 셋 이상 판정.

## 3. F: 사용자 명령 그대로 3 run

- 저장소 밖 로컬 도구 폴더 `tools/local_env/`에서 `python -B sim_stack.py start --no-browser`(이미지 wf7, 기본 SIM 시계, steer·
  executor CPU 3 은 compose 에서), 150 s 뒤 `python -B sim_auto.py start`, 끝나면 `python -B sim_auto.py reset`, 20 s. 세 번.
- 세 번째 run 에서 차량 카메라 프레임을 받는다(`capture_media.py camera`). F 가 통과하면 이 run 으로 README 첫 화면 영상을 만든다.
- 통과 = 세 run 모두 실행기 run_end 의 end_reason `row_end`.

## 4. B: SIM 대시보드 화면 (짧게 한 번)

- 3단계 대본(개정 1 고침 포함)의 단계 auto, stop, restart. `LINK_CHECK_S=2.0`, `HEARTBEAT_GAP_MAX_S=0.5`.
- 항목: 준비, 한 번 눌러 주행, heartbeat 최대 틈 < 0.5 s(브라우저 요청 시각), 탭 숨김(2.0 s 에 OPERATOR_LINK_LOST, 4 s 에 정지,
  다시 보이면 '다시 출발'), 주행 중 화면 STOP, STOP 뒤 다시 시작. 기준 문구는 3단계 등록 4절 그대로.
- 통과 = 여섯 항목 모두 통과.

## 5. C: A17(G) `bridge_stall`만 (3단계 실패 1건 가르기)

- 한 새 스택(W1, SIM 시계)에서 C0: `exec_checks safety --only bridge_stall --still-speed 0.05`(자세 되돌림 없음: 장면 출발점, 스택의 첫
  대본), 이어서 C1·C2·C3: 같은 명령에 `--reset-pose`(3단계 방법).
- 기준: A17 `bridge_stall`을 `exec_checks`에 적힌 그대로(개정 60·62). 통과 = C0~C3 모두 통과.
- 원인 가르기(판정과 별개로 미리 정함): run 마다 ACTIVE 첫 1 s 의 실행기 yaw rate(추정기 gyro, 틱 칸 `yaw`)를 적는다. 실패한 run 이
  모두 이 값의 크기가 2 °/s 를 넘고(3단계: 25 run 중 3 run 이 4.9, 5.5, −2.1 °/s, 그 중 하나가 `bridge_stall`), 그 값이 작은 run 은
  모두 통과하면 3단계 실패는 출발 때의 yaw rate 어긋남(센서·추정 쪽, 자세 되돌림 여부와 함께 적음)으로 본다. 그 값이 작은데 실패한
  run 이 있으면 시험 대상 동작 쪽으로 본다.

## 6. 반복

실패한 부분은 SIM 쪽 고침을 8절 개정으로 먼저 적고 커밋한 뒤 한 번 더 돌 수 있다. 두 시도를 다 적는다.

## 7. 결과를 적는 곳

`STEER_FOLLOW_GSIM_FINAL_KO.md` 13절, `STEER_FOLLOW_GSIM_FINAL.json`의 `stage4`, `docs/RESULTS_KO.md` 1절, README 결과 요약·한계.

## 8. 개정

(없음)
