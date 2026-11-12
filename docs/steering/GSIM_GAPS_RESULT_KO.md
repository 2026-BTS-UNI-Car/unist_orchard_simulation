# G-SIM 빈틈 정리 결과 (묶음 S2 1단계, 2026-11-13)

- 대상: ST-05 나머지(A17(G) 실패 3개 재판정), ST-04(A19(G) 다시, 약 0.34 s 멈춤, 자르기 노드 빠른 경로), ST-06(조향 기록 A23),
  ST-03 나머지(A18), SR-12(이미지 안 시험, 기반 이미지), G-SIM 열린 것 세 가지(열 끝 e +0.45 m, 빠뜨린 틱·`follow_blind`, 스택 시작 직후
  LiDAR 늦음), 화면 자료(DOC-13 일부).
- 브랜치 `gsim-final-20261113`(main `506eff3`에서). 이미지 `ubts-steer-sim:wf4c`(소스 `becf605`). 고친 검사 코드는 검사 컨테이너에만
  읽기 전용으로 붙인 스냅숏으로 썼다(`STACK_OPERATIONS_KO.md` 7절 override). 스택 프로세스는 이미지 소스로 돌았다. 예외: ST-06 세션은
  recorder 컨테이너에도 같은 스냅숏(사이드카 연결 패치 포함)을 붙였다.
- 모든 수치는 **자리값 기반 SIM 결과**다. 실차 성능이 아니다.
- 기록 폴더는 저장소 밖 `runs/gsim_final_20261113/`(이 PC)이다. 세션마다 `sessions/<이름>/`에 검사 결과 JSON, 호스트 부하 확인, 내릴 때의
  영수증(남은 컨테이너·네트워크 0)이 있다.

## 1. 요약

| 항목 | 원인 구분 | 근거(요약) | 고친 것 | 시험 |
|---|---|---|---|---|
| A17(G) `executor_freeze`·`executor_forced_kill`: 값 null | 검사 쪽 | 브리지 기록을 mark 0.05 s 앞부터만 읽음. 실행기 drive 는 0.1 s 간격 | 마지막 drive 를 mark 1.0 s 앞부터 찾음(`d4b7176`) | 단위 시험 1, 스택에서 값을 얻음. 다시 판정 통과(3.2절) |
| A17(G) `stop_unconfirmed`: 재전송 6번 | 검사 쪽 | 실행기 페이지 STOP 은 op stop 을 두 번 보냄(설계). 두 번째가 STOP 한 줄을 더 씀 | op stop 요청 뒤 60 ms 안 첫 STOP 을 그 요청 것으로 셈(`d4b7176`, `4ecda74`) | 단위 시험 1, 다시 판정 통과(3.2절) |
| A17(G) `executor_loop_stall`·`executor_freeze`: 구동 0 못 찾음(11/13) | 검사 쪽(전제) | 주입 1.9 s·3.0 s 전에 이미 `stale_lidar`로 멈춰 있었음(호스트 부하) | 주입 직전 runner 가 ACTIVE 아니면 무효(`4ecda74`) | 단위 시험 1, 다시 판정 통과(3.2절) |
| A17(G) `output_guard` | 검사 쪽 | settle 구간이 앞 대본의 조향에서 0 으로 내려가는 램프(S 112, 94, 57, 5, 0)인데 검사가 S 0 만 기대 | 0 쪽으로만 가고 0 에서 끝나는지로 봄(`4ecda74`) | 단위 시험 1, 다시 판정 통과(3.2절) |
| A19(G) `host_not_following` 래치 시각 | 검사 쪽 | 68 s 앞 다른 사례(track_model_noise)의 오차 구간을 기준으로 잡음 | 사례 mark 뒤 줄만 봄(`4ecda74`) | 단위 시험 1, 다시 판정 통과(3.3절) |
| A17(G) 입력 기한 0.772~0.776 s(한도 0.77) | 검사 쪽(기준 시각) | 브리지는 요청을 다음 루프 틱에 처리하고 기한을 그때 정함. 도착부터 재면 한 틱이 더 듦 | 브리지가 처리한 때(reply)부터 잼(`8c581c2`) | 단위 시험 1, 같은 기록으로 0.7598·0.7595 s |
| `exec_run start` 첫 Arm 거절 뒤 시작 못 함 | 동작 쪽(로컬 도구) | 거절된 Arm 이 실행기 클라이언트를 래치하고 run 을 닫는데 도구는 Arm 만 다시 보냄 | ack + 새 run 뒤 다시 Arm(`92e407f`) | 단위 시험 1 |
| A25(에뮬레이터 경로) | 모름 | 실행 끝 무렵 호스트 `steering_not_following` 래치 1건 뒤 drive 거부 27, p99 0.62° | 없음 | - |
| 시작 직후 약 0.34 s 멈춤 | 동작 쪽(SIM 환경) | 에뮬레이터는 futex 대기(= /clock 을 기다림), 그때 sim 컨테이너가 CPU 한도(cpus 4)에 초당 5~9번 걸림 | 없음 | - |
| 열 끝 e +0.45 m | 동작 쪽(추정기 열 끝 규칙) | 빠른 SIM 에 끝 기둥을 넣으면 e +0.08 → +0.64 m(0.6 m 뒤 기둥), +1.23 m(1.2 m), +2.02 m(2.0 m) | 없음(인지 층 몫, 4.1절) | 빠른 SIM 4 run |
| 빠뜨린 틱·`follow_blind` | 동작 쪽(SIM 실시간성) | 스캔 처리 스텝이 이미 30 ms(50 ms 예산). 호스트·SIM 이 바쁜 구간에 스텝 40~250 ms, 실행기 cgroup 제한도 걸림 | 없음 | - |
| 시작 직후 LiDAR 늦음(`stale_lidar`) | 동작 쪽(SIM 실시간성) | RTF 0.51~0.98 구간 약 40 s: 스캔 초당 2~9개, (수신−헤더) 지연 +6.1 s, 스캔 시각 추정 나이 0.47~1.5 s | 없음(측정 규칙과 시작 대기로 다룸, 4.3절) | - |
| ST-06 A23 | 통과 | 녹화 1회, `check_recording` 14항목 참, MCU 손실 0 | 사이드카를 recorder 역할에 연결(`1b57f43`) | 스택 녹화 1회 |
| ST-03 A18 | 불합격 그대로(0점 오차) | 한 사례(B·12) 값이 3단계와 같음. F3 손 정지 4건이 사이드카 시각으로 깨끗해짐 | 식별 편집 적용(`1c4263f`) | 단위 시험, A18 한 사례 |

## 2. 세션과 호스트 상태

| 세션 | 장면·시계 | 서비스 | 검사 코드 | 시작 전 부하 확인 | 세션 동안 Windows CPU(평균 / p90 / 최대) | 한 것 |
|---|---|---|---|---|---|---|
| `w0_a17` | w0_force, wall | sim, steer, recorder, executor | 스냅숏 s1(`d4b7176`) | 조용함(170 s 기다림) | 64 / 78 / 90 % | A17(G) 22대본, A19 faults, bridge_kill, a3noise, a25, 멈춤 탐침(steer·sim) |
| `w1_follow` | w1_force, wall | 같음 | s1, recorder 에도 붙임(사이드카 연결) | 바쁨(306 s 기다린 뒤 시작) | 65 / 85 / 90 % | 시작 직후 run, LiDAR 시작 탐침 150 s, 조사용 follow run, A23 녹화, 실행기 탐침 |
| `w0_a17b` | w0_force, wall | 같음 | s2(`4ecda74`) | 바쁨(187 s) | 67 / 86 / 89 % | A17(G) 고친 5대본, A19 faults |
| `media_w1`, `media_w1b` | w1_force, wall | sim, steer, recorder, dashboard, executor | 이미지 그대로 | 바쁨 | - | 그림(8절). 첫 세션은 내 대본 실수로 화면·카메라를 못 얻어 다시 돌림 |

- 세션마다 스택을 새로 올리고 내렸다(`stack.py`, 프로젝트 `ubts-steer-gsimfinal`, 내린 뒤 컨테이너 0·네트워크 0).
- 호스트는 거의 내내 바빴다. 시작 전 확인을 통과한 `w0_a17`도 세션 동안 다른 작업의 Python 계산(약 2코어)과 WSL VM(약 3.7코어)이
  돌았다. 그래서 이 문서의 시간 값은 개발 값이다. A10 규칙(RTF < 0.9, 에뮬레이터 `/clock` 틈 > 100 ms, SIM 이 만든 호스트 래치)으로
  무효가 된 단계는 무효로 적었다.
- 멈춤 탐침(`steering.p6a_stall_probe`, 10 ms)은 steer 컨테이너 CPU 를 19~27 %에서 44~62 %로 올렸다. 판정 run 에서는 켜지 않는다.

## 3. A17(G)·A19(G)·A25·A3(G) (ST-05 나머지, ST-04)

### 3.1 A17(G) 첫 실행 (`w0_a17`, 검사 `d4b7176`)

26항목 중 통과 20, 실패 3, 무효 3. 239.7 s, RTF 중앙 1.000·최소 0.883.

| 대본 | 결과 | 값 |
|---|---|---|
| 운영자 STOP 5번, deadman_release, hold_lost, c_ttl_link_returns, link_watchdog, fault_stop, second_screen_and_cli, mcu_reset, usb_unplug, half_frame, crc_one_line, crc_burst, range_error, system_time_change, bridge_stall, bridge_forced_kill | 통과 | |
| executor_loop_stall, executor_freeze | 실패 | 주입 1.9 s·3.0 s 전에 이미 `stale_lidar`로 멈춤(실행기 기록: lidar_age 0.32~0.45 s). 구동 0 전이를 찾을 수 없었다 |
| output_guard | 실패 | C 줄 S 112, 94, 57, 5, 0(settle 램프) 뒤 guard 종결. 검사가 S 0 만 기대 |
| steering_status_loss, stop_unconfirmed, executor_forced_kill | 무효 | SIM 이 만든 호스트 래치. forced_kill 은 고친 검사로 값을 얻었다: 입력 기한 0.7731 s(도착 기준) |

### 3.2 A17(G) 고친 검사로 다시 (`w0_a17b`, 검사 `4ecda74`, 입력 기한 판정은 `8c581c2` 기준)

5대본 모두 유효(단계 RTF 0.946~1.09).

| 대본 | 결과 | 값 | 기준 |
|---|---|---|---|
| executor_loop_stall | 통과 | 실행기 감시 0.383 s 에 구동 0 보냄, 정지, 재출발 0 | 구동 0, 정지, 재출발 0 |
| executor_freeze | 통과 | 구동 0·정지·재출발 0. 입력 기한: 브리지가 마지막 drive 를 받은 때부터 0.7598 s(도착부터 0.7761 s), 종결 이유 bridge_error | ≤ 0.75 + 0.02 s |
| stop_unconfirmed | 통과 | 재전송 5번 100.1, 160.2, 219.8, 259.7, 320.1 ms(격자 뒤 첫 루프 틱), 래치 408.2 ms. op stop 요청 −7.9·+3.1 ms 두 번(요청이 쓴 STOP 2줄: 0, 20.1 ms) | 재전송 5번, 격자, 래치 400 ms 뒤 첫 틱 |
| output_guard | 통과 | S 21, 2, 0 뒤 guard_violation:steer_rate_above_limit, LATCHED | 원 걸음이 MCU 에 가지 않음 |
| executor_forced_kill | 통과 | 구동 0·정지·재출발 0·프로세스 없음. 입력 기한 0.7595 s(도착부터 0.7720 s) | ≤ 0.77 s |

- `w0_a17b`의 실행 기록은 검사 `4ecda74`로 돌았다. 그 판에서 executor_freeze·executor_forced_kill 은 도착 기준 값으로 실패였고,
  `8c581c2`의 기준(브리지 처리 때)으로 같은 기록의 브리지 로그에서 다시 계산한 값이 위 표다.
- 실행기 페이지의 STOP 은 op stop 을 두 번 보낸다(설계): `emergency_stop()`이 HTTP 스레드에서 바로, 그다음 주 루프의 `_handle_stop()`이
  클라이언트를 래치하면서 한 번 더. 브리지 코어의 op stop 은 비상 사건만 켜고 바로 답하며 STOP 줄은 루프가 다음 틱에 쓴다. 그래서
  두 번째 요청이 루프 틱 하나 뒤에 STOP 한 줄을 더 쓴다. 재전송 격자(첫 STOP 기준)는 그대로다(래치 408.2 ms).
- 입력 기한: 브리지는 대기열의 요청을 루프 틱마다 하나씩 처리하고, 기한(`input_until = clock() + 0.75`)을 그 처리 때 정한다.
  그래서 요청이 도착한 때부터 재면 루프 틱 하나가 더 들어간다. 한도(0.75 s + 브리지 틱 하나)는 그대로 두고 기준 시각을 처리 때로
  바꿨다. 두 값을 다 적는다.
- steering_status_loss 는 이번에 유효한 실행을 얻지 못했다(첫 실행 무효). 이전 실행(ST-05 2차)은 통과였다.

### 3.3 A19(G) faults

| 실행 | 결과 | 비고 |
|---|---|---|
| `w0_a17`(검사 `d4b7176`) | 23 사례 중 통과 18, 보고 4, 실패 1 | 실패: host_not_following 래치 − 기준 65.9 s. 68 s 앞 다른 사례(track_model_noise, 역시 steering_not_following 래치)의 오차 구간을 기준으로 잡았다 |
| `w0_a17b`(검사 `4ecda74`) | **통과**(판정 항목 모두), 146.5 s, RTF 0.981~1.000 | host_not_following 래치 − 기준 0.883 ms(한도 0~20 ms). track·thermal·watchdog·ttl 통과(ST-05 의 `afb8bfc` 고침 확인) |

- 이 두 실행은 이미지 `wf4c`의 브리지(옛 not_following 규칙)로 돌았다. main `09cc932`(이 브랜치에 병합)는 브리지와 판정에 개정 43(정착한 줄만 셈)을 넣었다. 사례 창 고침(`4ecda74`)은 그 판정에 그대로 얹혔고 단위 시험을 맞췄다(`ad12bd0`). 새 규칙의 브리지로 A19 를 다시 돌리지는 않았다.

### 3.4 bridge_kill, A3(G) 잡음, A25 (`w0_a17`)

| 검사 | 결과 | 값 |
|---|---|---|
| bridge_kill | 통과 | 브리지 SIGKILL(-9) 2번, 감독기 재시작 3번, 포트 다시 열고 MCU 리셋, 자동 재출발 0, 재시작 뒤 무장 안 됨 |
| a3noise(A3(G), 개정 15) | 통과 | DISARMED 정지 11,630틱(≥ 10,000), raw sd 비 1.0345(프로필 1.000 ± 10 %), 재생 응답 같음 |
| a25(에뮬레이터 경로) | 실패 | 관절 − 공유 모델 p99 0.62°(한도 0.2°). 실행 끝 무렵 호스트 `steering_not_following` 래치 1건, 그 뒤 drive 거부 27. 원인 모름(호스트 바쁨, 브리지 상태별 목표·각 기록이 남지 않음). 측정 뒤 main `09cc932`가 브리지 규칙을 바꿨다(개정 43: 서보 램프가 목표에 닿은 줄만 셈). 사인 구간처럼 램프가 계속 움직일 때의 오검출이었다면 그 고침이 다루지만 확인하지 않았다 |

- 브리지 강제 종료는 실차 시계에서 1번 했다. SIM 시계 실행은 하지 않았다(ST-04 할 일의 나머지).

## 4. G-SIM 열린 것

### 4.1 열 끝 정지 위치 e = +0.45 m (빠른 SIM +0.04~+0.09 m)

- 원인(동작 쪽, 추정기의 열 끝 규칙): 과수원 생성기는 각 나무 열 끝에 콘크리트 끝 기둥을 둔다(`gsim_follow.post_positions`: 간격
  1.25 m 이면 마지막 줄기 선 30.02 m 뒤 약 0.6 m 인 30.625 m, 0.10 m 각). 추정기는 선 위 지지(97번째 백분위)로 열 끝을 정하므로
  끝 기둥까지 열로 본다.
- 근거 1(G-SIM 기록, 11/12 nominal_1): 추정기의 열 끝(주행 거리 + 앞쪽 끝 거리)은 8 m 앞에서 처음 잡힌 뒤 정지까지 34.0~34.16 m
  (출발 기준)로 일정했다. 참값 좌표로 30.35~30.49 m 다. 마지막 줄기 선보다 0.33~0.47 m 뒤, 끝 기둥 앞면(약 30.55 m) 근처다.
- 근거 2(빠른 SIM, 같은 실행기·제어기, 기록 `runs/gsim_final_20261113/offline/end_post_fsim/`): 기둥 두 줄 끝(24.0 m) 뒤에 같은 크기의
  끝 기둥을 두면 e 가 기둥 거리만큼 밀린다.

| 끝 기둥 | e [m] | 횡 RMSE [m] |
|---|---|---|
| 없음(등록 nominal, seed 0) | +0.080 | 0.041 |
| 마지막 기둥 +0.6 m | +0.643 | 0.038 |
| +1.2 m | +1.225(한도 ±1.0 넘음) | 0.024 |
| +2.0 m | +2.020(넘음) | 0.033 |

- G-SIM +0.45 와 빠른 SIM(+0.6 m 기둥) +0.64 의 차 약 0.19 m 는 모름(줄기 반지름, 잎, 스캔 잡음 가운데 무엇인지 가르지 않음).
- 고치지 않았다. 추정기(인지 층)는 다른 묶음이 바꾸는 중이다. 넘기는 것: 열 선 위 1.2 m 이상 뒤에 구조물(끝 기둥, 울타리, 다음 블록)이
  있으면 e 가 한도를 넘는다. 끝 기둥을 열 끝으로 볼지(멈추는 곳을 기둥 선으로) 마지막 줄기로 볼지는 설계 결정이 필요하다
  (R37: e 의 기준은 마지막 줄기 선).

### 4.2 빠뜨린 틱과 `follow_blind`

- 11/12 실행기 기록(gsimfollow control 볼륨에서 읽기 전용 복사): 빠뜨린 틱이 없던 run(nominal_1, lateral_p040)은 한 스텝 p50 3.6~4.1 ms,
  p99 30~31 ms, 최대 40 ms. 빠뜨린 run(nominal_2, 3)은 p50 6.3~6.9 ms, p99 47~51 ms, 최대 177~208 ms 이고 늦은 틱(> 75 ms)이 58, 53개다.
  늦은 틱은 그 스텝 자체가 40~70 ms(가끔 100~250 ms)였다. `follow_blind` 구간과 늦은 틱 구간이 겹친다: 제어기가 잰 루프 지연
  (스캔 나이 p90 + loop_fixed)으로 속도를 묶은 것(설계대로).
- 스캔을 처리하는 스텝이 바쁘지 않을 때도 30 ms 쯤(50 ms 예산의 60 %)이라, 실행기 프로세스가 1.5~2배 느려지면 틱을 빠뜨린다.
- 11/13 조사용 run(`w1_follow`): 실행기 cgroup 이 follow 동안 6번 제한에 걸렸다(제한 시간 합 264 ms, cpus 1.5). 실행기 스레드 20개, BLAS 는
  참조 BLAS(단일 스레드)였다. 같은 run 에서 SIM RTF 가 0.856 까지 내려갔다.
- 원인: SIM 실시간성(호스트와 SIM 컨테이너가 바쁜 구간)과 실행기 컨테이너 CPU 제한이 겹친 것. 어느 쪽이 얼마인지는 가르지 못했다.
  코드로 고치지 않았다. 측정은 조용한 호스트에서 하고 RTF 를 run 마다 적는다(2단계 사전등록 초안).

### 4.3 스택을 켠 직후 LiDAR 늦음(`stale_lidar`)

- 11/12 첫 시도(사용자 스택, `local_env_check/auto_run_1.log`): run 시작 9~21 s 에 lidar_age 0.21 → 0.47 s, 11 s 에 `stale_lidar`
  (한도 0.45 s), 2.22 m 에서 멈춤. 사용자 스택 control 볼륨의 실행기 기록(읽기 전용 복사)으로 보면 그 구간에 스캔은 초당 9~11개 왔고
  실행기 한 스텝이 74~254 ms 였다(그 전 20 ms).
- 11/13 LiDAR 시작 탐침(`w1_follow`, healthy 뒤 약 10 s 부터 150 s, `/scan`·`/sim/scan`·`/clock` 수신과 실행기 스캔 시각 규칙 흉내):

| 구간(탐침 시작 기준) | RTF | 스캔/s | (수신 − 헤더) 지연의 변화 | 스캔 시각 추정 나이 최대 |
|---|---|---|---|---|
| 0~48 s | 0.99~1.01 | 10 | 0 | 0.20~0.25 s |
| 49~87 s | 0.51~0.98 | 2~9(가장 긴 틈 1.24 s) | +6.1 s | 0.47~1.50 s(27초 동안 0.45 넘음) |
| 88~150 s | 0.95~1.01 | 10 | 0 | 0.20~0.28 s |

- 그 구간에 sim 컨테이너 CPU 가 160 %에서 300~345 %(한도 4코어)로, 실행기 28 → 45~87 %, steer 40 → 97~118 % 로 같이 올랐다.
  호스트 CPU 는 그 구간에 더 오르지 않았다. `w0_a17`의 sim 탐침에서도 비슷한 구간(gz 80~100 %, sensor_contract 47~71 %, cpu_lidar 23 %,
  cgroup 사용 초당 3.1~3.4 s, 제한 걸림)이 있었다.
- 실행기는 실차 시계 모드에서 스캔 시각을 헤더(SIM 시각) + 최근 50개 (수신 − 헤더)의 중앙값 − 0.08 s 로 정한다(`ScanStamper`
  header_offset). SIM 이 실시간보다 느리면 그 차가 계속 커지고 중앙값이 뒤처져 나이가 커 보인다. 스캔 틈과 실행기 루프 지연이 더해진다.
- 원인: SIM 이 실시간보다 느려지는 구간(동작 쪽, SIM 환경). 무엇이 그 구간에 sim 컨테이너 CPU 를 올리는지는 모름(모든 ROS 프로세스가
  같이 오름. DDS 쪽 일일 수 있으나 확인 안 함). 실행기는 설계대로 멈췄다(안전 정지, 다시 누르지 않으면 안 감).
- 조사용 follow run 도 같은 이유로 21.3 m 에서 멈췄다(RTF 최소 0.856, `follow_blind` 761틱, 한 스텝 최대 244 ms).
- 고치지 않았다. 실차 시계 모드의 시간 민감 결과는 RTF < 0.9 구간에서 무효(A10)다. 로컬 실행 도구는 이미 시작 명령을 90 s 까지 기다린다.
  2단계 초안은 healthy 뒤 120 s 기다림과 run 동안 RTF 최소 0.9 를 유효 조건으로 넣었다.
- 시작 직후 run 은 이번에 시작조차 못 했다: 첫 Arm 이 `steering_status_stale`로 거절되자 실행기가 run 을 닫았고(`start_a_run_first`),
  `exec_run`은 Arm 만 다시 보내 5번 더 거절됐다(종료 코드 4). 첫 Arm 거절은 한 번 눌러 시작을 만드는 묶음(S1)에 넘긴다.

### 4.4 스택 시작 직후 약 0.34 s 멈춤 (ST-04)

- 멈춤 탐침(`w0_a17`, steer·sim 컨테이너, 10 ms): 에뮬레이터가 16.6 s 에 332 ms 멈췄다. 그동안 스레드는 대부분 futex 대기(잠든 상태)였고
  탐침 자신은 16 ms 이하 간격으로 깼다(VM 전체 멈춤이 아님). 같은 때(10~19 s) sim 컨테이너가 cgroup CPU 한도(cpus 4)에 초당 5~9번
  걸렸다(제한 시간 합 초당 0.7~4.1 s). 브리지는 20~23 s 에 138~340 ms 멈췄고 그때 steer 컨테이너도 한도에 걸렸다(탐침 부하 포함).
  cpu_lidar 는 6~18 s 에 200~430 ms 멈췄다.
- 원인(근거 위): 시작 때 sim 컨테이너가 CPU 한도에 걸려 `/clock`이 끊기고, `/clock`을 기다리는 에뮬레이터와 그 뒤의 브리지가 멈춘 것으로
  본다. SIM 안 문제이고 실차와 무관하다. A10 규칙이 이 구간을 무효로 처리한다. 고치지 않았다.
- 덧붙여 본 것: VM 실시간 시계(CLOCK_REALTIME)가 단조 시계 기준으로 30 s 마다 +0.55 s, +0.68 s 두 번씩 뛰었다(탐침 `realtime_jump`).
  실행기·브리지 기한은 단조 시계라 영향이 없다. 벽시계 값을 쓰는 기록(`/clock` 벽시계 틈 등)을 볼 때 알아 둘 것.

### 4.5 자르기 노드 빠른 경로 (ST-04)

- 판단: 넣지 않았다. 조건(`test_field_world` 통과와 A7 재측정)을 이번에 하지 못했다. A7 은 field1010 장면(W2) 스택이 필요하다.
- 패치(`runs/steer_phase4_20261112/held/g_gaps_fast_clip.patch`)는 이 브랜치에 깨끗이 붙는다(`git apply --check`).
- 기본 장면(w1_force)과 G-SIM 열 추종은 자르기 노드를 쓰지 않는다.

## 5. 조향 기록 A23 (ST-06)

- 연결: recorder 역할의 감독기가 고정 기록기 옆에서 `steering.steering_sidecar`를 돌린다(`1b57f43`, 보류 패치
  `st06_sidecar_recorder_role.patch` 그대로). `UBTS_STEER_SIDECAR=none`이면 기록기만 돈다. 사이드카 상태 파일은 recorder 역할의 파일이다.
- 녹화 1회(`w1_follow`, 손실 주입 없음): 기록기 종류 both, IMU·GPS 포함, 15.95 s. 그동안 브리지에 30 % 구동, 조향 +60 → −60 → 0 mrad.
  대본은 저장소 밖 `runs/gsim_final_20261113/scripts/a23_once.py`(사이드카를 직접 켜지 않고 recorder 역할이 켠 것을 쓴다).

| A23 항목 | 값 | 판정 |
|---|---|---|
| `steering.jsonl` 스키마(`orchard.steering.timeline.v1`) | 274줄, 형식 문제 0, 설명자 frame_count 와 같음, t_s 단조 | 통과 |
| 20 Hz, 누락률(보고) | MCU 상태 줄 유일 274·손실 0·반복 0(누락률 0.0), MCU 주기 20.0 Hz. 벽시계로는 18.12 Hz(SIM 이 실시간보다 느려 SIM 13.65 s 동안 274줄, 가장 긴 틈 0.79 s) | 통과(보고) |
| stale 표시 | stale 0(주입 없음). 모든 줄에 bool, 상태 줄 없는 줄은 stale | 통과 |
| 설명자 SHA | 내용 SHA·파일 SHA(`steering_descriptor.sha256`)·정준 바이트 일치 | 통과 |
| 보정표 사본 | `host_profile.json` 사본, cal_id 1, 표시 각 계산식, 메시지와 일치 | 통과 |
| 별도 bag | `/rover/steering/state` 하나, 138개, 마감(metadata finalized), mcap | 통과 |

- `steering_sidecar.check_recording`의 14항목이 모두 참이다. 녹화 폴더 목록에 `steering` 폴더가 녹화로 나오지 않는다. 고정 기록기의
  `motor.jsonl`(27줄)도 그대로 있다.
- 11/09 개발 녹화의 누락 11.5 %(15.4 Hz)는 개발 대본이 일부러 넣은 1.5 s 상태 줄 손실이었다. 주입이 없으면 손실은 0 이다.
- 본 것(판정 밖): 별도 bag 은 녹화의 약 절반(SIM 6.86 s, 138개, 사이드카 274줄)만 담았다. bag 프로세스는 녹화 시작과 같이 켜졌으므로
  `ros2 bag record`의 시작·구독 지연으로 보인다(확인 안 함). A23 의 bag 기준(토픽·개수 > 0)은 통과한다. 사이드카 `steering.jsonl`이
  정본이다.
- 결정 제안(`STEER_RECORDING_KO.md` 2절, 녹화 폴더 안 `steering/`)은 이번 녹화 1회로 확인했다: 목록에 섞이지 않았고 기록기의
  마감이 끝났다(기록기 상태 archived, complete). 미디어 작업은 보지 않았다.

## 6. 식별 A18 (ST-03 나머지)

- 보류했던 식별 편집(`runs/wf4/p4_gaps/edits/edit_ident_1.py`)을 그대로 붙였다(`1c4263f`): B0 CAL POL 이 옳은 방향으로 움직였지만
  펌웨어 최소(4 카운트)에 못 미치면 조향 바퀴를 든 상태(SIM 대역)로 한 번 다시, B7 부하의 조향 쓸기(기본 꺼짐), field_calib C7 의
  손 정지(hard stop) 명령 시각을 조향 사이드카의 MCU 상태 줄에서 찾기(사이드카가 없으면 그 사건은 깨끗하지 않음, 감사 AUD3A-03).
- field_calib 끝-끝 시험이 사이드카 없이 깨끗한 손 정지 1건을 기대하고 있어서 편집 뒤 실패했다(HEAD 에서는 통과). 편집의 뜻대로
  0건과 '시각 미해결' 표시를 기대하게 고치고 `sidecar_brake`·`steering_stop_lines` 시험을 더했다.
- A18 실행: 메모리 규칙(1 GB 넘는 프로세스 3개 이하, 3단계에서 9사례 동시 실행이 커밋 메모리 61 GB)을 지키려고 한 사례만 돌렸다:
  변형 B·시드 12, 14분 10초(Windows, Python 3.13, 네이티브 빌드는 저장소 밖). 기록 `runs/gsim_final_20261113/a18/b12/`.

| 값 | 결과 | 이전(fix_wf3a, 같은 사례) |
|---|---|---|
| 주행·정지 조향 속도, 최소 PWM 둘, τ, 백래시 | 6/6 한계 안 | 같은 값 |
| 0점 오차(F1) | 0/1, +0.537°(한계 ±0.2°) | 같은 값(곡률 떠돎 때문, `IDENT_KO.md` 6절) |
| κ(δ) 표(F2) | 6/6, 가장 큰 −4.47 % | 같은 값 |
| duty→속도(F3) | 4/4, 가장 큰 +4.67 % | 같은 값 |

- 이 사례에서 CAL POL 은 처음에 통과했다(d1 +5, d2 −5). 다시 하기 경로는 돌지 않았다.
- 새 F3 손 정지(조향 사이드카의 MCU 상태 줄로 명령 시각): 4건 모두 깨끗함, 정지 거리 0.035, 0.052, 0.081, 0.102 m(명령 때 속도
  0.235~0.481 m/s), p50 0.067 m. 결과 안의 `distance_over_model` p50 0.32 는 분석 쪽 제동 모형보다 짧다는 뜻인데 가르지 않았다.
- **남은 것**: A18 표의 '정지 거리 대 참값' 행은 `ident/a18.py`에 없다(편집에 들어 있지 않음). 9사례 전체 A18 은 돌리지 않았다
  (한 사례 14분, 규칙상 동시 2~3개). Python 3.10(numpy < 2) 실행도 하지 않았다. A18(F) 판정은 3단계 그대로 불합격이다.

## 7. 이미지 안 시험과 기반 이미지 (SR-12)

- 이미지 안 전체 시험: 이미지 `wf4c` 의 Python 3.12.3(numpy 1.26.4, pytest 7.4.4), 작업 트리(커밋 `92e407f`과 아직 커밋 전이던
  `gz_views` 두 파일)를 읽기 전용으로 붙임, 2 CPU·2 GiB·네트워크 없음, 8분 43초. 기록 `runs/gsim_final_20261113/sr12/img312/`.

| 묶음 | 결과 |
|---|---|
| 조향(`test_steering_*` 등) | 1132 통과, 10 건너뜀 |
| field_world | 20 통과 |
| 실행기(`tools/row_executor/tests`) | 459 통과, 13 건너뜀 |
| 호스트 브리지 | 127 통과 |
| field_calib | 22 통과 |
| 적합성 대본 | 92/92 네이티브, 92/92 백엔드 |
| 참값 격리, `verify_sources`, vendor 동기, 트리 안 캐시 | 통과, 통과, OK, 0 |

- 이미지 소스와 트리의 차이는 이 브랜치에서 바꾼 파일과 새 파일뿐이다(15줄).
- 기반 이미지 정의(`docker/Dockerfile.base`)는 이번에도 빌드하지 않았다. `ros:jazzy-ros-base` 내려받기와 apt 미러·PyPI 접속(새 이미지
  약 5 GB)이 필요한데 이 작업은 외부 접속을 하지 않는다. 완료 기준의 '이 파일만으로 빌드'와 '그 위에서 조향 이미지 빌드·import 검사'는
  미확인 그대로다(`docs/provenance/BASE_IMAGE_KO.md`).

## 8. 화면 자료 (DOC-13 일부)

모두 실제 실행에서 만들었다. 자리값 기반 SIM 결과다. 사용자 이름, 주소, 좌표가 없는 것을 눈으로 확인했다(대시보드 화면에 보이는
경로는 컨테이너 안 경로 `/run/ubts-steer/esp-tty`뿐). 파일은 저장소 밖 `runs/gsim_final_20261113/media/`에 있다. 저장소에는 아직
넣지 않았다(넣을 것은 문서·미디어 묶음이 고른다).

| 파일 | 크기 | 무엇 | 만든 곳 |
|---|---|---|---|
| `rover_model_front.png`, `rover_model_side.png`, `rover_model_top.png`, `rover_model_oblique.png` | 1280×720 | 조향 로버 모델(프로필 v0.1, 구 바퀴 충돌), 앞·옆·위·비스듬히, W0 평지(1 m 격자는 표시용) | `steering.gz_views w0` |
| `rover_model_front_steer_left15.png` | 1280×720 | 같은 앞 시점, 조향 목표 +15°(바퀴 +15.6°/+14.5°) | 같음 |
| `orchard_overview_top.png`, `orchard_overview_oblique.png` | 1280×720 | 원본 과수원 장면 w1_force 전경(위 36 m, 비스듬히). 출발점의 차가 보인다 | `steering.gz_views w1` |
| `orchard_lane_start.png` | 1280×720 | 출발점에서 열 사이 통로를 뒤에서 본 것 | 같음 |
| `rover_camera_row_follow.png` | 640×360 | 열 추종 run 중 차량 카메라 한 장(`media_w1b`, 1300번째 프레임) | `capture_media.py camera`, `still` |
| `lidar_scan_w1_start.png`, `lidar_scan_w1_in_row.png` | 864×864 | `/scan`(실차 S2 형식) 한 장을 차 기준 좌표로 그린 것(출발점, 통로 안) | `scripts/dump_scan.py`, `plot_scan.py` |
| `sim_dashboard_rover_control_drive.png` | 1440×1500 | 열 추종 run 중 SIM 대시보드 rover-control 화면(LiDAR 패널, 카메라) | `capture_media.py page` |
| `video/gsim_row_follow_camera_w1.gif` | 360 폭, 7.5 MB | 열 추종 run 의 차량 카메라 42 s 를 2배 빠르게(240장) | ffmpeg |
| `video/gsim_row_follow_camera_w1_low.mp4` | 480×270, 1.3 MB | 같은 run 의 카메라 약 63 s(14.3 fps, crf 34) | ffmpeg |

- RViz 는 조향 스택에 없다. LiDAR 화면은 대시보드 LiDAR 패널(대시보드 화면 안)과 `/scan` 그림으로 대신했다.
- 그림의 run(`media_w1b`)은 호스트가 바빠 19.8 m 에서 `steering_feedback_stale`로 끝났다(SIM 상태 줄 틈, 4.3절과 같은 원인). 영상은 그 앞
  구간이다. 열 끝까지 간 영상은 11/12 의 `docs/steering/figures/steer_follow_gsim/gsim_follow_camera_w1.mp4`(nominal_3)다.
- 과수원 그림 세 장은 PNG 1.2~1.7 MB 라 저장소에 넣으려면 줄여야 한다(규칙: 그림 1 MB 미만).

다시 만드는 명령(이 PC):

```
# 모델·과수원 그림: 이미지 wf4c, GPU 렌더(D3D12/Ogre 1), 작업 트리 sim/ubts 스냅숏을 읽기 전용으로 붙임, 4 GiB 예약
bash runs/gsim_final_20261113/scripts/gz_views_run.sh
#   컨테이너 안: python3 -B -m steering.gz_views w0 --out /runs/views_w0 ; python3 -B -m steering.gz_views w1 --out /runs/views_w1
# 대시보드·카메라·/scan: 5서비스 스택에서 run 하나와 갈무리
bash runs/gsim_final_20261113/scripts/session_media.sh <세션 이름>
python -I runs/gsim_final_20261113/scripts/plot_scan.py <scan.json> <out.png> --title "<제목>"
# GIF·mp4: 프레임 목록(concat)으로 ffmpeg (runs/gsim_final_20261113/NOTES_KO.txt 의 명령)
```

## 9. 다시 하는 법

```
# 스택 세션: 부하 확인, up, 검사, 상태 복사, down(영수증). runs/gsim_final_20261113/scripts/session.sh
#   session.sh <이름> <장면> <시계> <서비스> <프로필> "<검사 이름>|<python3 -B -m 뒤>" "host:<이름>|<셸>" "hostbg:..." "pre:..."
SESSION_OVERRIDES=<검사 스냅숏 override> bash session.sh w0_a17b w0_force wall "sim steer recorder executor" executor \
  "safety|steering.exec_checks safety --out /runs/safety --only executor_loop_stall executor_freeze stop_unconfirmed output_guard executor_forced_kill" \
  "a19|steering.p6a_checks faults --out /runs/a19"
# 멈춤 탐침(판정 run 에서는 켜지 않음): 컨테이너 안에서
python3 -B -m steering.p6a_stall_probe --seconds 600 --out /runtime/steer/stall_probe_steer.jsonl
# 빠른 SIM 끝 기둥 확인(저장소 밖 대본, sim/ubts 에서)
UBTS_STEER_BUILD_DIR=<저장소 밖 폴더> PYTHONPATH=. python -B runs/gsim_final_20261113/offline/end_post_fsim.py <out>
```

- 대본과 결과는 저장소 밖 `runs/gsim_final_20261113/`(scripts/, sessions/, offline/, media/, NOTES_KO.txt)에 있다.

## 10. 넘기는 것

| 무엇 | 누구에게 | 내용 |
|---|---|---|
| 첫 Arm 거절 | 실행기 묶음(S1, 한 번 눌러 시작) | 브리지 호스트 래치 `steering_status_stale`(SIM 상태 줄 틈)가 남아 첫 Arm 이 거절되고 실행기가 run 을 닫는다. `exec_run`은 고쳤다(`92e407f`). 한 번 눌러 시작(`/api/go`)에서도 같은 처리가 필요하다 |
| 끝 기둥과 e | 인지 층 묶음(S1b), 설계 결정 | 4.1절. 열 선 위 1.2 m 이상 뒤 구조물이면 e 가 한도를 넘는다 |
| SIM 실시간성 | 2단계(G-SIM 최종 확인) | 4.2·4.3절. 조용한 호스트, healthy 뒤 120 s 기다림, run 동안 RTF 최소 0.9 를 유효 조건으로(사전등록 초안) |
| bag 시작 지연 | 기록 쪽 | 5절. 별도 bag 이 녹화의 약 절반만 담음 |
| A25(에뮬레이터 경로), 브리지 강제 종료 SIM 시계, A17 steering_status_loss 유효 실행 | ST-04/ST-05 나머지 | 조용한 호스트에서 |
| 자르기 노드 빠른 경로 | W2 작업 | 4.5절 조건(A7 재측정) |

## 11. 2단계(최종 이미지로 G-SIM 확인) 준비

- 빌드 인자: `S2_NODE_EXEC_KO.md` 9.3절 그대로(`BASE_IMAGE_ID` = 로컬 기반 `ubts-sim-clean-tenth:…` 의 ID, `SOURCE_COMMIT`,
  `SOURCE_STATUS_SHA256`, `IMAGE_TAG_HINT`). 깨끗한 트리에서 만들고 매니페스트를 커밋 blob 과 견준다. compose 기본 태그와 Dockerfile
  `IMAGE_TAG_HINT`는 같은 커밋에서 바꾼다.
- 세션 명령과 사전등록 초안: 저장소 밖 `runs/gsim_final_20261113/stage2/STAGE2_PREP_KO.txt`, `stage2/PREREG_STAGE2_DRAFT_KO.md`
  (첫 run 전에 저장소에 따로 커밋할 것). 초안이 v1 등록에 더한 것: 한 번 눌러 시작(`/api/go` + `/api/heartbeat`), 운영자 신호 끊김 run,
  healthy 뒤 120 s 기다림, run 동안 RTF 최소 0.9(못 넘으면 무효, 같은 자리에서 한 번 다시).
- `exec_checks.start_drive`와 `exec_run.start_run`의 시작 경로는 실행기 묶음의 새 운영자 규약(`/api/go`)에 맞춰 바꿔야 한다.
- 측정은 조용한 호스트에서 한다(이번 세션들은 다른 작업 때문에 Windows CPU 평균 60~67 %였고, 그때 열 추종 run 이 `stale_lidar`·
  `steering_feedback_stale`로 21.3 m, 19.8 m 에서 멈췄다). 판정 run 에서 멈춤 탐침은 켜지 않는다.
