# 최종 G-SIM 확인 5단계 사전등록 (steer_follow_gsim_final5_v1)

- 작성: 2026-11-14 (묶음 S2 5단계, 브랜치 `gsim-stage5-20261114`, main `abf6d0b`에서). 등록 run 을 하나도 돌리기 전에 커밋한다.
- 기계가 읽는 사본: `PREREG_STEER_FOLLOW_GSIM_FINAL5.json`(`steering.gsim_final5.prereg()`와 같아야 한다. 시험
  `sim/ubts/test_steering_gsim_final5.py`).
- 2~4단계 등록과 결과는 그대로 둔다.
- 모든 수치는 **자리값 기반 SIM 결과**다.

## 0. 바뀐 것과 까닭

4단계(wf7)에서 A 열 추종 2/4, B 대시보드 화면 3/6 이었다.

- A 의 두 실패(출발 0.4~0.6 s 뒤 실행기 입력 약 0.35 s 끊김): 5단계 개발 확인(판정 아님) d1·d2 에서 실행기 컨테이너에 입력 진단을
  넣고 쟀다(`ebc9128`). 끊김 동안 실행기 입력 스레드의 타이머는 돌았고 GIL 탐침 늦음 0.3~2 ms, 주 루프 폴링 틈 2.5~68 ms 였다.
  실행기의 /clock 틈은 ESP 에뮬레이터(steer 컨테이너)의 /clock 정지 기록과 같은 단조 시각이었다. SIM 시간은 가는데 /clock·IMU 가
  안 온 틈(약 0.31~0.37 s)은 새 DDS 참가자가 스택에 들어온 뒤 몇 초 안에만 나왔다(검사 컨테이너 시작 4~7 s 뒤, 짧은 참가자
  4 번 중 3 번, 스택 시작). 4단계 A 는 검사 컨테이너가 뜬 뒤 5~6 s 에 go 를 눌렀다. 사용자 명령(sim_auto)은 DDS 참가자를 만들지
  않는다(4단계 F 3/3). 그래서 실행기 쪽 고침이 아니라 SIM 쪽 고침이다: 열 추종 검사가 자기 /clock 이 10 s 동안 0.1 s 넘는 틈
  없이 올 때까지(최대 60 s) 기다린 뒤 Start 한다(`d58649d`). 실행기의 안전 한도(조향 상태 0.3 s, MCU 명령 TTL)는 그대로다.
  ros_gz_bridge 출력이 멈추는 FastDDS 안쪽 까닭은 가르지 못했다.
- B 의 `session_conflict`: 운영자 규약 개정 5(`c29fc35`, `OPERATOR_API_KO.md` 3.1). run 이 끝나면 세션을 풀고, heartbeat 가
  기한을 넘기면 풀고, heartbeat 만 보낸 세션은 다른 세션의 go 를 막지 않는다(go 가 묶음을 넘겨받음). 주행 중 다른 세션의 go 거부,
  자동 재출발 없음은 그대로. 4단계 B 의 묶인 세션은 같은 스택의 다른 클라이언트였다(그 스택은 새로 뜬 것이라 앞 대본의 세션일 수
  없다). 호스트 Chrome 에 18090 대시보드 탭이 열려 있는 것으로 보인다(대시보드 기록의 카메라 WS 세션). 확정하지 못했고 사용자
  브라우저는 건드리지 않는다.
- 이미지 `ubts-steer-sim:wf8`: 위 셋과 README 문구(`d5ed282`), 기본 태그(`7fad51a`). 대시보드 사본은 `a92cfec` 그대로.

## 1. 공통

4단계 등록 1절 그대로(SIM 시계, 세션마다 새 스택, healthy 뒤 120 s, 판정 단계 5 s 창 RTF 0.5 미만은 호스트 고장으로 한 번 더, 부하
확인 기록). 이미지는 `ubts-steer-sim:wf8`(이 등록을 넣은 커밋의 깨끗한 작업 트리에서 등록 run 전에 빌드, 매니페스트 = 커밋 blob,
compose 기본 태그). 모든 세션 동안 사용자 스택은 내려 두고 무거운 잠금을 잡는다.

## 2. A: 열 추종 4 run (W1, SIM 시계)

- 4단계 A 와 같다(`exec_checks follow`, `nominal_1`, `nominal_2`, `nominal_3`, `lateral_p040`, run 마다 새 스택,
  `steer_follow_gsim_v1` 한도). 검사는 Start 전에 DDS 가 조용해질 때까지 기다린다(위 0절, 기다린 시간과 틈을
  `discovery_settle`로 남긴다).
- 통과 = 판정된 run 이 모두 통과, 네 자리 중 셋 이상 판정.
- run 마다 적는 것: `discovery_settle`, 그 run 동안 실행기 `input_gap` 사건과 틱 `diag`의 /clock 틈 0.25 s 초과, 에뮬레이터
  `clock_stalls`, 실행기 루프 p50/p99/최대와 건너뛴 틱.
- 원인 규칙(판정과 별개, 미리 정함, 판정은 기록대로): 실패한 run 의 끝 1 s 안 앞에 실행기 /clock 틈이 0.25 s 를 넘고 에뮬레이터도
  같은 단조 시각(0.1 s 안)에 정지를 기록했으면 SIM 쪽(스택의 DDS 전달), 아니면 실행기·제어기 쪽으로 본다.

## 3. B: SIM 대시보드 화면 (6 항목 전부)

- 4단계 대본의 단계 auto, stop, restart. `LINK_CHECK_S=2.0`, `HEARTBEAT_GAP_MAX_S=0.5`. 대본은 화면 자신의 세션 digest 를 요청
  본문에서 읽어 `page_sessions`로 남긴다(판정 아님).
- 항목: 준비, 한 번 눌러 주행, heartbeat 최대 틈 < 0.5 s(브라우저 요청 시각), 탭 숨김(2.0 s 에 OPERATOR_LINK_LOST, 4 s 에 정지,
  다시 보이면 '다시 출발'), 주행 중 화면 STOP, STOP 뒤 다시 시작. 기준 문구는 3단계 등록 4절 그대로.
- 통과 = 여섯 항목 모두 통과.
- 적는 것: 실행기 기록의 `operator_link` 사건(bind, takeover, release, conflict)과 화면 세션 digest. 화면 것이 아닌 세션이 묶였거나
  넘겨졌으면 적는다.

## 4. F: 사용자 명령 그대로 3 run (회귀)

- 저장소 밖 로컬 도구 폴더 `tools/local_env/`에서 `python -B sim_stack.py start --no-browser`(기본 이미지 wf8, 기본 SIM 시계), 150 s
  뒤 `python -B sim_auto.py start`, 끝나면 `python -B sim_auto.py reset`, 20 s. 세 번. 카메라는 받지 않는다.
- `sim_auto.py`는 heartbeat 가 거절되어도 go 를 보낸다(개정 5: heartbeat 만 보낸 세션은 넘겨받는다. go 를 한 세션이 있으면 그대로
  거절).
- 통과 = 세 run 모두 실행기 run_end 의 end_reason `row_end`.

## 5. 반복

실패한 부분은 SIM 쪽 고침을 7절 개정으로 먼저 적고 커밋한 뒤 한 번 더 돌 수 있다. 두 시도를 다 적는다.

## 6. 결과를 적는 곳

`STEER_FOLLOW_GSIM_FINAL_KO.md` 14절, `STEER_FOLLOW_GSIM_FINAL.json`의 `stage5`, `docs/RESULTS_KO.md` 1절, README 결과 요약·한계.

## 7. 개정

(없음)
