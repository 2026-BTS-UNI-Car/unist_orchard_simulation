# 최종 G-SIM 확인 8b단계 사전등록 (steer_follow_gsim_final8b_v1)

- 작성: 2026-11-14 (브랜치 `gsim-stage8b-20261114`, main `0f07467`에서). 등록 run 을 하나도 돌리기 전에 커밋한다.
- 기계가 읽는 사본: `PREREG_STEER_FOLLOW_GSIM_FINAL8B.json`(`steering.gsim_final8b.prereg()`와 같아야 한다. 시험
  `sim/ubts/test_steering_gsim_final8b.py`).
- 2~7단계 등록·결과와 8단계(하지 않음, `STEER_FOLLOW_GSIM_FINAL_KO.md` 18절) 기록은 그대로 둔다.
- 모든 수치는 **자리값 기반 SIM 결과**다.

## 0. 무엇을 바꾸나

7단계 등록(`steer_follow_gsim_final7_v1`, `PREREG_STEER_FOLLOW_GSIM_FINAL7_KO.md`)을 그대로 다시 잰다. 바꾸는 것은 둘뿐이다.

1. **호스트 CPU 배치(측정 동안만, 2026-11-14 17:40 KST 사용자 결정).** 7단계의 실패 여섯은 모두 바로 앞 0~2 s 에 Windows 효율 등급 0(E 코어)이
   79~80 % 로 차고 등급 1(P 코어)은 15~29 % 였다(`STEER_FOLLOW_GSIM_FINAL_KO.md` 16.7). 그래서 WSL 가상 머신을 P 코어에 둔다.
   - 논리 CPU 지도(측정 전 `GetLogicalProcessorInformationEx` 로 읽음, Intel Core Ultra 7 155H, 논리 22개): 효율 등급 1(P) = 0, 1,
     10~19(6 코어 × 2 스레드), 등급 0(E) = 2~9, 20, 21(E 8 + 저전력 E 2). P 마스크 `0xFFC03`.
   - 첫째 방법: WSL VM 프로세스(`vmmemWSL`, 이름이 `vmmem`이면 그것)의 프로세서 affinity 를 P 마스크로. 원래 값을 먼저 적고, 바꾼 뒤
     다시 읽는다.
   - 대신 방법(첫째가 거절되거나(보호 프로세스, 권한 없음) 효과가 없을 때): 활성 전원 구성의 AC 값 `SUB_PROCESSOR SCHEDPOLICY`와
     `SHORTSCHEDPOLICY`(다른 유형의 스레드 예약 정책, 짧은 실행 스레드 예약 정책)를 2 '성능 기준에 맞는 프로세서 선호'로 하고
     `powercfg /setactive SCHEME_CURRENT`. 측정 전 값은 둘 다 AC 5 '자동'이었다(DC 도 5, DC 는 바꾸지 않는다). 전원 구성을 만들거나
     가져오거나 지우지 않는다.
   - 기간: 작업 표본기를 켜기 전부터 F 가 끝날 때까지. 끝나면(실패해도, finally) 원래 값으로 되돌리고 다시 읽어 확인한다.
     `.wslconfig` 편집, WSL 다시 시작, 재부팅, 그 밖의 호스트 설정은 하지 않는다.
   - 효과 확인(판정 기준 아님, 기록): 판정 run 마다 표본기의 E·P 등급 평균, E 등급 78 % 이상 표본 수를 7단계 값 옆에 적는다. 배치가
     E·P 부하를 실제로 옮겼는지 그렇지 않은지 그대로 적는다.
   - 두 방법 모두 쓸 수 없으면 8b 세션은 돌리지 않는다(7단계의 반복이 되므로). 그렇게 적는다.
2. **이미지 `ubts-steer-sim:wf11`**(sim main `0f07467`의 깨끗한 작업 트리에서 빌드, 매니페스트 680 파일 = 커밋 blob, 펌웨어 빌드
   `04353B49`, 이 단계에서 다시 빌드하지 않음). wf10 과의 차이: 대시보드 사본 `e2a9785`(메뉴 8개 원래대로, PW-10 화면), 실행기의 PW-10
   온라인 배율(LiDAR 속도 ÷ 표 속도, 기록·제안만). PW-10 작업의 출력 같음 회귀(base `acd1a08` 대 `d8e2349`, 기록 폴더
   `runs/pw10_20261114/` 9절): 차동 기준 에피소드 같음, 빠른 SIM follow 15/15·T1 71/71·wide 3/3 직렬·trace 같음. 그 밖의 차이는 ST-03 의
   식별 절차·시험 대본(스택 경로 밖)이다. B 대본이 쓰는 화면 요소(`#autoPanel`, `#autoPrimary`, `#autoState`, `#autoNotice`, `#autoEnd`,
   `#frameStopButton` 등)와 자율 칸 문구는 wf11 사본에도 그대로 있다(자율 칸에는 배율 줄과 주행 뒤 질문이 더해졌다).

## 1. 그대로인 것

7단계 등록 1~5절 전부: SIM 시계, 장면 `w1_force`, A·B 세션마다 새 스택(측정 프로젝트, 대시보드 포트 18091, 그동안 사용자 스택은 내리고
무거운 잠금), healthy 뒤 120 s, Windows 프로세스별 표본기, 판정 run 바로 전 호스트 조용함 확인(마지막 30 s 전체 CPU 평균 45 % 미만 **그리고**
E 등급 평균 50 % 미만, 5 s 마다 최대 900 s), 호스트 고장 규칙(실패 사건 전 5 s 에 전체 70 % 이상 표본, 1.0 s 이상 늦은 표본, 또는 E 등급
평균 75 % 이상), 무효 자리는 run 당 최대 두 번 더(세 번 다 무효면 판정 못 함), RTF 규칙(5 s 창 0.5 미만), 실패 사건 시각(B 는 실행기의
실제 멈춤), 판정 함수(`steering.gsim_final7`과 같은 함수), 안전 한도(조향 상태 틈 150 ms, 실행기 조향 상태 0.3 s, MCU 명령 TTL, fresh·
blind 상한, 속도 조절기 최저 속도, 운영자 기한 1.0 s)와 자율주행 센서의 충실도·주기·지연.

- A: `nominal_1`, `nominal_2`, `nominal_3`, `lateral_p040`, 한도 `steer_follow_gsim_v1`. 통과 = 판정된 run 모두 통과, 네 자리 중 셋 이상
  판정.
- B: 대시보드 화면 여섯 항목(단계 auto, stop, restart, `LINK_CHECK_S=2.0`, `HEARTBEAT_GAP_MAX_S=0.5`). 통과 = 여섯 모두.
- F: 저장소 밖 로컬 도구로 `sim_stack.py start --no-browser`(기본 이미지 wf11, SIM 시계), 150 s 뒤 자리 세 개에서 `sim_auto.py start`·
  `reset`. 통과 = 판정된 세 run 모두 `row_end`. F 가 끝나면 사용자 스택은 wf11 로 떠 있다.

## 2. 결과를 적는 곳

`STEER_FOLLOW_GSIM_FINAL_KO.md` 19절(8b단계), `STEER_FOLLOW_GSIM_FINAL.json`의 `stage8b`, `docs/RESULTS_KO.md` 1절 한 줄. 레포 파일에는
표본기가 본 호스트 프로세스 이름을 적지 않고 'WSL VM', '다른 프로세스 합' 같은 묶음으로만 적는다(배치 대상인 WSL VM 프로세스 이름은
예외, 원 자료는 기록 폴더 `runs/gsim_stage8_20261114/stage8b/`).

## 3. 개정

(없음)
