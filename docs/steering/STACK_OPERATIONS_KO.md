# 조향 G-SIM 스택 측정 운영 규칙 (ST-07)

- 작성: 2026-11-12. 근거: 4단계 작업 흐름(2026-11-09)에서 여러 단계가 같은 compose 프로젝트 이름(`ubts-steer`)으로 스택을 올리고 내려 서로의 측정을 끊은 일, 예산 대기 2시간(7,200 s), `HOLD_SIM_MATRIX`·`HOLD_PROTOCOL.txt` 표시 파일이 끝난 뒤에도 남아 큰 조건 묶음을 막은 일(계획 4절 나9). `DECISIONS_KO.md` R22.
- 대상: `compose.ubts-steer.yaml`의 스택(SIM, ESP 에뮬레이터·브리지, 기록기, 대시보드, 실행기, 검사)과, 시간에 민감한 측정이나 큰 조건 묶음(T1 점검 전체, 오탐률 실행, A18 식별, 위상 훑기, Gazebo 측정).
- 도구: `tools/steer_sim/stack.py`(규칙 1·3·4·5와 2의 표시 파일·기록 확인을 강제한다), 시험 `sim/ubts/test_steering_stack.py`(Docker 없이, 가짜 docker와 가짜 예산 도구).
- 이 문서는 규칙이다. 측정 결과는 없다. 모든 SIM 수치는 자리값 기반 SIM 결과이고 현장 검증이 아니다.

## 1. 한 번에 스택 하나

- 조향 스택은 한 번에 하나만 올린다. `stack.py up`은 상태 폴더의 잠금 파일(`stack.lock.json`)이 다른 프로젝트를 가리키거나, 이름이 `ubts-steer`로 시작하는 다른 compose 프로젝트의 컨테이너가 돌고 있으면 거절한다(이 도구 없이 올린 스택도 막는다).
- 옛 SIM 스택(`ubts-sim`, 18080)이나 다른 측정 스택과도 함께 올리지 않는다(compose 머리말과 같다). 이것은 2절의 호스트 부하 확인으로 본다.
- 상태 폴더는 저장소 밖이다: `--state-dir` 또는 `UBTS_STEER_STACK_STATE`. 없으면 예산 도구 폴더의 `../locks/steer_stack`.

## 2. 큰 조건 묶음이나 시간 민감 실행 전의 호스트 부하 확인

옛 표시 파일 `HOLD_SIM_MATRIX`, `HOLD_PROTOCOL.txt`는 2026-11-12에 `<보관 폴더>/root_files/`로 옮겼다(W-15). 다시 만들지 않는다. 대신 시작 전에 아래를 확인하고, 그 결과를 실행 기록에 남긴다.

| 볼 것 | 방법(예) | 기준 |
|---|---|---|
| Windows 전체 CPU | PowerShell `Get-Counter '\Processor(_Total)\% Processor Time' -SampleInterval 5 -MaxSamples 6` | 30 s 평균 30 % 미만, 표본 6개 모두 50 % 미만 |
| WSL CPU | `wsl -d Ubuntu-22.04 -- cat /proc/loadavg` (1분 평균) | 1분 부하 1.0 미만 |
| 가용 메모리 | PowerShell `(Get-Counter '\Memory\Available MBytes').CounterSamples[0].CookedValue` | 6,000 MB 넘음(R20 호스트 메모리 규칙) |
| 1 GB 넘는 호스트 프로세스 | PowerShell `Get-Process | Where-Object WorkingSet64 -gt 1GB` | 3개 이하(R20). 실행할 작업이 더해도 3개 이하 |
| 다른 측정·SIM 프로세스 | 명령줄에 `pytest`, `sim_in_loop`, `ttl_window`, `steering.ident`, `run_conformance`, `gz sim`, `closed_loop`, `slow_link`가 든 python·gz 프로세스(PowerShell `Get-CimInstance Win32_Process`, WSL `ps -eo args`) | 내 작업 말고 0개 |
| Docker | `docker ps --format '{{.Label "com.docker.compose.project"}} {{.Names}}'`, `python -I <tools>/docker_budget.py status` | `ubts-steer*`·`ubts-sim*` 컨테이너 0개(내가 올린 것 제외), 예산 예약이 내 것 말고 없음. Docker를 쓰지 않는 작업은 Docker Desktop이 꺼져 있어도 된다 |
| 다른 작업의 측정 표시 | 계획(todo.md 0절)과 체크포인트에 '측정 중'으로 적힌 일, 사용자가 말한 조용한 시간 | 없음 |

- 기다리기: 기준을 못 넘으면 30 s마다 다시 보고, 최대 10분(600 s) 기다린다. 그래도 바쁘면 시작하지 않고 '호스트 바쁨, 시작 안 함'과 마지막 측정값을 기록한 뒤 다른 일을 한다. 남의 프로세스를 끄거나 남의 컨테이너를 내리지 않는다.
- 기록: 확인 결과를 JSON 하나로 그 실행의 기록 폴더(`runs/<주제>/<단계>/host_load_check.json`)에 쓴다. 칸: 시각(UTC), 각 표본값, 기준, 판정, 찾은 다른 프로세스·컨테이너 이름(명령줄은 앞 120자만), 예산 상태, 기다린 시간. 스택을 올리면 `stack.py up --load-check <그 파일>`로 넘겨 잠금 파일에 경로와 SHA-256이 남게 한다(파일이 없으면 거절).
- 실행 중 감시: 시간 민감 단계는 실행 중에도 호스트 표본을 남긴다(옛 규칙: 다른 작업이 표본의 5 %를 넘게 겹치면 그 실행은 무효이고 같은 자리에서 한 번 다시 돈다). A10의 무효 규칙(RTF < 0.9, 에뮬레이터 /clock 틈 > 100 ms, SIM이 만든 호스트 래치; R19 결정 2)은 그대로다.
- 표시 파일: 다른 작업이 잠깐 호스트를 쓰겠다고 알릴 때는 파일 하나를 `UBTS_STEER_HOLD_FILE`로 정해 둔다. 그 파일이 있으면 `stack.py up`이 거절한다. 만든 쪽이 끝날 때 지운다. 끝난 뒤 남은 파일은 계획에 적고 사용자 확인 없이 남의 파일을 지우지 않는다.

### 2.1 스택 없이 도는 큰 조건 묶음의 시작 절차 (2026-11-17, 계획 W-15)

빠른 SIM(T1 리그)의 열 추종 행렬(`follow_matrix`, `follow_wide`, `follow_realdims`, `follow_smooth`), T1 안전 대본 전체(`sim_in_loop_steer.py
checks`), 식별 실행처럼 스택을 올리지 않는 큰 묶음도 같은 부하 확인을 거친다(단위 시험과 적합성 대본은 가벼운 시험이라 잠금 없이 돈다). 순서:

1. 무거운 측정 잠금을 잡는다: `mkdir <작업 폴더>/runs/_locks/heavy_measure`(이미 있으면 그 안 `OWNER.txt` 를 보고 1분마다 다시; 1시간 넘게
   그대로면 OWNER 의 기록 폴더 NOTES 를 보고 판단), 안에 `OWNER.txt`(누가, 시각, 무엇, 기록 폴더), 큐 파일(`runs/_locks/QUEUE_*.txt`)에 `order` 줄.
   잠금이 다른 무거운 측정을 막으므로 부하 확인은 잠금을 잡은 **뒤**에 한다.
2. 호스트 부하 확인: 위 표의 항목(전체 CPU, WSL, 가용 메모리, 1 GB 넘는 프로세스, 다른 측정·SIM 프로세스, Docker, 다른 작업의 측정 표시)에 더해
   Windows 효율 등급 0(E 코어; 이 PC 는 논리 CPU 2~9, 20, 21) 30 s 평균 50 % 미만(최종 확인 7단계 규칙, `PREREG_STEER_FOLLOW_GSIM_FINAL7_KO.md`).
   예: PowerShell `Get-Counter '\Processor(2)\% Processor Time', ... , '\Processor(21)\% Processor Time' -SampleInterval 5 -MaxSamples 6`
   의 평균. 기준을 못 넘으면 30 s 마다 다시 보고 최대 10분 기다린다. 그래도 바쁘면 잠금을 풀고(`rmdir`) 큐에 '호스트 바쁨, 시작 안 함'을
   적은 뒤 시작하지 않는다.
3. 결과를 기록 폴더의 `host_load_check.json` 에 쓴다(칸은 위와 같다). 묶음의 실행 기록(로그 머리)에 그 파일 이름을 적는다.
4. 묶음을 돈다(일꾼 4 이하: 행렬 도구의 상한). 끝나면(실패해도) 잠금 폴더를 지우고 큐에 `done` 줄.

- 판정에 쓰는 것과 부하의 관계: 빠른 SIM 은 SIM 시간으로 돌아 직렬 로그·trace·판정이 부하에 따라 바뀌지 않는다(같은 코드·seed 면 같은 digest).
  부하는 걸린 시간과 실행기 기록의 시간 칸(`ms`, `loop_ms`, `scan.proc_ms`)만 바꾼다. 그래서 부하 확인을 못 넘은 묶음은 '무효'가 아니라 시간
  칸을 쓰지 않는다고 적는다. 시간 칸이나 걸린 시간을 판정에 쓰는 실행(실행기 처리 시간, Gazebo)은 위 2절의 무효 규칙을 그대로 따른다.

## 3. 작업마다 다른 프로젝트 이름

- 이름은 `ubts-steer-<작업>`이다(소문자, 숫자, `-`, `_`; 예 `ubts-steer-st05`, `ubts-steer-sr11`). compose 파일의 기본 이름 `ubts-steer`는 쓰지 않는다(`stack.py`가 거절한다).
- 모든 docker 명령에 `-p <이름>`을 붙인다. `run`, `ps`, `down`도 `up`과 같은 이름, 같은 compose 파일, 같은 프로필, 같은 `UBTS_STEER_*` 값이어야 한다(다르면 compose가 sim 서비스를 다시 만들어 ROS 그래프가 끊긴다). `stack.py`는 `up` 때의 값을 잠금 파일에 적고 뒤 명령에 그대로 쓴다.
- 이름 공간이 갈려도 대시보드 포트(127.0.0.1:18090), ROS 도메인 43, GZ 파티션은 같다. 그래서 1절(한 번에 하나)을 지킨다.

## 4. 짧은 예산 대기

- `stack.py up`은 `docker_budget.py acquire --label <프로젝트> --gib 6 --wait-s 600`으로 메모리를 예약한다. 10분 안에 못 잡으면 컨테이너를 하나도 올리지 않고 끝낸다(종료 코드 3). 다시 시도는 2절부터.
- 예약 유효 시간(`--ttl-s`, 기본 4시간)이 측정보다 길게 잡혀야 한다. 측정이 길면 나눠서 한다.

## 5. 올린 쪽이 내린다, 남은 것을 기록한다

- `stack.py down --project <이름>`은 잠금 파일이 그 이름일 때만 내린다. 다른 이름, 또는 잠금 없이 돌고 있는 스택은 내리지 않는다(시험 `test_a_job_cannot_take_down_or_use_another_projects_stack`). 남의 스택이 걸리적거리면 그 작업에 알리거나 계획에 적는다.
- 내리기는 `docker compose -p <이름> ... down -t 20`이다. `-v`를 쓰지 않는다(이름 붙은 볼륨은 남긴다). `docker prune`, 볼륨 삭제, `rm -v`를 쓰지 않는다.
- 끝나면 그 프로젝트의 남은 컨테이너·네트워크 수를 확인해 영수증(`--receipt`)에 적는다. 0이 아니면 잠금을 풀지 않는다(다음 `up`이 막힌다). 원인을 보고 고친 뒤 다시 `down`.
- 작업이 중간에 끊기면 다음 사람은 `stack.py status`로 잠금과 돌고 있는 프로젝트를 보고, 잠금의 프로젝트 이름으로 `down`한다(그 작업을 이어받은 경우만).

## 6. 시험

`python -B -m pytest -p no:cacheprovider sim/ubts/test_steering_stack.py`(또는 unittest). 9개: 프로젝트 이름 규칙, 자기 프로젝트의 up·run·down(예산 라벨, 10분 대기, `-p`, `down -t 20`, `-v` 없음, 영수증), 한 번에 하나, 남의 스택을 내리거나 쓰지 못함(잠금이 없을 때도), 다른 조향 프로젝트 컨테이너가 돌면 거절, 예산 시간 초과면 아무것도 올리지 않음, 표시 파일과 부하 확인 기록, 내린 뒤 남은 것이 있으면 잠금 유지, override 파일(`--override`)이 `up`·`run`·`down`에 같이 쓰임. Docker를 쓰지 않는다.

## 7. 기본 이미지, override, 그림·영상 갈무리 (2026-11-12 ST-05)

- compose 기본 이미지는 `ubts-steer-sim:wf7`이다(2026-11-13 4단계: 실행기 처리 시간 작업 main `b00b666` — 늦은 틱 전 조향 상태 새로 읽기, gc.freeze. wf6 은 최종 확인 3단계: 운영자 연결 기한 1.0 s, 출발 정지 문턱 0.08 m/s, IMU 게임 회전 벡터, 대시보드 사본 a92cfec. 시계는 `UBTS_STEER_CLOCK`, 로컬 도구는 SIM 시계로 띄운다. wf5 는 2026-11-13 최종 이미지: 한 번 눌러 시작 실행기, 대시보드 사본과 SIM 확장, recorder 역할의 사이드카, LiDAR 브리지의 profile=mapping 원본 스캔. 그 전 wf4c 는 실행기 패키지, S2 형식 노드, 열 추종 제어기). 그냥 `up`이면 `/scan`(실차 형식)이 나온다. 실행기는 `--profile executor`로 켠다. 측정 결과는 `S2_NODE_EXEC_KO.md` 7절.
- `stack.py up --override <파일>`(여러 번 가능)은 compose override 파일을 스택 파일 뒤에 더한다. 잠금 파일에 남아 `run`·`ps`·`down`이 같은 파일을 쓴다. 쓰는 곳: 검사 컨테이너에만 고친 검사 코드 스냅숏을 붙일 때, field1010 장면의 지도·목록을 sim에 읽기 전용으로 붙일 때.
- 그림과 영상은 돌고 있는 스택에서 `tools/steer_sim/capture_media.py`로 갈무리한다(스택을 올리거나 내리지 않는다). 문서에 넣는 파일은 `docs/steering/figures/`에 두고 캡션에 '자리값 기반 SIM 결과'를 적는다.

```
# 차량 카메라 프레임(sim 컨테이너의 카메라 WebSocket), 영상, 한 장
python -I tools/steer_sim/capture_media.py camera --project ubts-steer-<작업> --seconds 70 --out <기록>/media/cam
python -I tools/steer_sim/capture_media.py clip --frames <기록>/media/cam --out <파일>.mp4 --every 2 --max-mb 5
python -I tools/steer_sim/capture_media.py still --frames <기록>/media/cam --index 40 --out <파일>.png
# 대시보드 rover-control 화면(대시보드 서비스가 켜진 스택, 이 PC의 playwright와 Chrome)
python -I tools/steer_sim/capture_media.py page --out <파일>.png --wait-s 12
```

- 대시보드(2026-11-13, D-13): 대시보드 레포의 새 화면 사본(`dashboard/DASHBOARD_SOURCE.json`)과 SIM 확장(`dashboard_sim/`)이며 sim 네트워크 이름공간에서 돈다(18090 은 sim 서비스가 낸다). 실행기 운영자 페이지를 127.0.0.1:8097 로 부른다. `docs/steering/SIM_DASHBOARD_KO.md`.
- 개요(Overview) 화면에는 PC 주소가 보인다. rover-control 화면만 갈무리하고, 넣기 전에 사용자 이름·주소·좌표가 없는지 눈으로 본다.
- 실행기와 검사를 같이 쓰는 스택(대시보드 없음)에서는 화면 갈무리를 할 수 없다. 화면은 기본 스택에서, 주행 영상은 실행기 스택에서 따로 갈무리한다.

## 8. CPU 한도: steer·실행기 3 (2026-11-13, 실행기 처리 시간 작업)

- `compose.ubts-steer.yaml`의 CPU: sim 4, steer(에뮬레이터·브리지) 3, recorder 1, dashboard 1, executor 3. 합 12, 이 PC(Intel Core Ultra 7
  155H, 논리 CPU 22)의 절반 남짓이다. 그 전에는 steer·executor 가 1.5 였다.
- 까닭(잰 것, 자리값 기반 SIM 결과): 최종 확인 3단계 F(SIM 시계, `STEER_FOLLOW_GSIM_FINAL_KO.md` 12.6)에서 steer 컨테이너가 CFS 주기
  5,091 개 중 858 개에서 스로틀됐고, 실행기 루프 p99 가 209 ms 였다. SIM 시계 모드에서 브리지·실행기가 2 ms 마다 시계를 보며 CPU 를
  더 쓴다(실행기 루프 p50 8.5 -> 17~18 ms, steer CPU p90 74 -> 107~153 %). 개발 확인으로 두 컨테이너를 3 으로 올리자 스로틀이 거의
  없어지고 루프 p99 가 32 ms 가 됐다. 실행기의 빽빽한 스캔 처리(통로 갱신 한 번 P 코어 약 40 ms, `EXECUTOR_PERF_KO.md`)도 한 코어를
  넘게 쓸 때가 있다.
- 메모리 한도(6 GiB 예산)는 그대로다. CPU 는 예산 도구가 묶지 않으므로 다른 측정 스택과 함께 올리지 않는 1절이 그대로 중요하다.
- 이 PC 는 P·E·저전력 E 코어가 섞여 있어, 같은 일이 놓인 코어에 따라 2~3 배 느리다(`EXECUTOR_PERF_KO.md` 표). 시간을 재는 실행 전에는
  2절 부하 확인을 한다.

## 9. 새 ROS 참가자가 들어오면 스택 출력이 약 0.35 s 멈춘다 (2026-11-14, 최종 확인 5단계)

- 잰 것(개발 확인 d1·d2, 자리값 기반 SIM 결과, 기록 폴더 `runs/gsim_stage5_20261114/`): 스택에 ROS 참가자가 새로 들어오면(검사
  컨테이너 시작, 짧게 떠는 ROS 프로세스) 그 뒤 몇 초 안에 ros_gz_bridge 의 /clock·IMU 와 /clock 에 맞춰 나오는 스캔이 벽시계로 약
  0.31~0.37 s 멈춘다. 그동안 SIM 시간은 간다(Gazebo 는 돈다). 실행기와 ESP 에뮬레이터가 같은 단조 시각에 같은 틈을 봤고, 실행기의
  입력 스레드·GIL·주 루프는 제때였다. 짧은 참가자 4 번 중 3 번, 검사 컨테이너 시작 4~7 s 뒤, 스택 시작 때 나왔다. 정확한 FastDDS
  안쪽 까닭은 가르지 못했다.
- 따로: Gazebo set_pose, 카메라 구독을 새로 만들 때는 SIM 시간까지 같이 0.1~0.2 s 멈췄다(SIM 시계 소비자에게 해 없음).
  hook 호출(에뮬레이터 status·mark, 브리지 status, 실행기 hook status)은 틈을 만들지 않았다.
- SIM 시계 모드에서 이 틈은 실행기 시계가 멈췄다가 뛰는 것으로 보인다. 그래서 출발 직후에 걸리면 조향 상태 나이(0.3 s)나 MCU 명령
  TTL 로 run 이 끝난다(4단계 A 두 run).
- 할 일: 운전하는 검사는 Start 전에 DDS 가 조용해질 때까지 기다린다(`exec_checks.settle_discovery`: 자기 /clock 이 10 s 동안 0.1 s 넘는
  틈 없이, 최대 60 s; 지금은 `follow`). 측정 중인 스택에 `stack run`으로 ROS 프로세스를 새로 띄우지 않는다. 사용자 명령(대시보드,
  `sim_auto.py`)은 DDS 참가자를 만들지 않는다.
- 진단: SIM 시계 모드의 실행기 틱마다 `diag`(구독 콜백 수와 가장 긴 틈, 입력 스레드 타이머, GIL 탐침, 주 루프 폴링 틈), 150 ms 넘는
  구독 틈은 `input_gap` 사건(`exec_container_main.SpinMonitor`).

## 10. 실행기 CPU 가중치, 입력 진단 켜기, 바쁜 호스트 (2026-11-14, 최종 확인 6단계)

- `compose.ubts-steer.yaml`의 executor 서비스에 `cpu_shares: 8192`가 있다. 컨테이너 안에서 읽은 `cpu.weight`는 532 다(Docker 29.9 의
  runc 환산, 1024 = 100). 다른 서비스는 `cpu_shares`가 없어 100 이다. VM 의 CPU 가 모자랄 때만 실행기 몫이 커진다. CPU 한도(`cpus`)는
  그대로다(8절). 이 PC 의 WSL VM 은 vCPU 22 개이고 스택이 쉴 때 load 가 4 안팎이라 평소에는 차이가 없을 수 있다(재지 않았다).
- 사용자 명령 `sim_stack.py`(저장소 밖 로컬 도구)는 `main` 가지의 compose 파일을 내보내 쓴다. 이 가지가 `main`에 병합되기 전에는
  그 스택에 이 가중치와 아래 환경 변수가 없다(이미지 기본값 = 진단 끔).
- 입력 진단(5단계 `SpinMonitor`, 틱 `diag`, `input_gap` 사건)은 기본 끔이다. 켜기: 스택을 올리기 전에 셸에서
  `UBTS_STEER_EXEC_INPUT_DIAG=1`(compose 가 실행기 컨테이너에 넘긴다), 또는 `exec_container_main --input-diag on`. 끄면 SIM 시계
  모드의 틱 `diag`에는 그 틱의 단조 시각 `wm`과 벽시계 `ww`만 있다(호스트 표본기와 맞추는 데 쓴다).
- 실행기의 SIM 시계 루프는 2 ms 마다 시계를 보지 않고, /clock 표본이 다음 50 ms 격자점에 닿으면 깨어난다(가장 길게 50 ms 기다린 뒤
  멈춤·기한을 본다). 실행기 ROS 노드는 파라미터 서비스, 타입 설명 서비스, /rosout 을 만들지 않는다(rclpy 실행기가 콜백마다 노드의
  모든 서비스를 대기 집합에 넣는다). 구독·QoS·콜백은 그대로다. 브리지 래퍼(steer 컨테이너)의 2 ms 폴링은 그대로다.
- 잰 것(개발 확인, 자리값 기반 SIM 결과, 기록 폴더 `runs/gsim_stage6_20261114/`): 쉬는 실행기 CPU(run 전) 22.0 % -> 15.9 %
  (ROS 입력 스레드 13.9 -> 11.1, 주 스레드 5.2 -> 2.6, GIL 탐침 0.4 -> 없음), run 이 끝난 뒤 58.1 % -> 35.7 %(ENDED 에서도 추정기가
  스캔을 처리한다: 정지 관측, 실차 코드의 뜻). 콜백은 약 164 Hz(/clock 120, IMU 32, 스캔 10, 사람 명령 2).
- 바쁜 호스트(1단계 Windows 프로세스별 표본기, 1 s): 스택 없음 16 %, 우리 스택이 쉴 때 37~47 %, 열 추종 run 평균 40 %(최대 71 %), 실패한
  사용자 명령의 마지막 5 s 평균 54 %(최대 71.5 %). 우리 것이 아닌 몫: 화면 합성(dwm) 약 0.9 코어, 원격 화면 프로그램 약 0.8 코어,
  svchost, 보안 프로그램 0.2~1.5 코어, 작업 창(VS Code) 0.2~2.9 코어. 전원 구성표 '균형 조정'. 실패한 run 에서는 실행기의 모든 단계
  (스캔 처리, 조향 상태 갱신, 브리지 왕복)가 함께 3~10 배 느려졌다(VM 전체가 느려진 꼴). 70 % 남짓에서도 코어가 남는데 왜 그만큼
  느려지는지(P/E 코어 배치, 코어 파킹, 주파수)는 가르지 못했다.
- 할 일: 시간 민감 run 바로 전에 2절 확인과 함께 Windows 전체 CPU 의 30 s 평균을 본다(6단계 등록: 45 % 미만, 최대 15 분 기다림).
  측정 중에는 작업 창에 출력을 쏟지 않는다.
- 측정 스택의 대시보드 포트: 호스트 브라우저에 18090 탭이 열려 있으면 같은 포트의 측정 스택에 붙는다(5단계 B 의 다른 세션). 측정
  스택은 override 파일(`ports: !override`)로 다른 포트를 쓴다(6단계 A·B: 18091). 사용자 명령은 18090 그대로다.

## 11. steer·sim 컨테이너 가볍게, VM 안 CPU 를 재는 법 (2026-11-14, 최종 확인 7단계)

- 브리지 래퍼(`dashboard/scripts/steer_rover_bridge.py`)의 SIM 시계 루프는 2 ms 마다 ROS 를 돌지 않고 ROS 대기에서 잠든다. /clock 표본이나
  제어 요청(요청을 넣을 때마다 guard condition 을 깨운다)이 오면 깨고, 가장 길게 50 ms 기다린 뒤 멈춤을 본다. STOP 이 걸려 있는데 아직
  걸음이 그것을 받지 않았으면 전처럼 2 ms 폴링이다. 격자 걸음(20 ms SIM)과 STOP 의 즉시 걸음은 SIM 시간과 걸린 STOP 에만 달려 있어
  정하는 것은 같다. 쌓인 /clock 드레인은 '첫 spin 전의 시계'와 비교해 빈 대기열이면 대기 집합을 한 번만 만든다. 실차의 벽시계 모드
  (20 ms 격자)는 바뀌지 않는다.
- SIM 시계 모드의 브리지 노드, ESP 에뮬레이터 노드, 센서 계약 노드는 파라미터 서비스·타입 설명 서비스·/rosout 이 없다(실행기와 같다,
  10절). 토픽·QoS·콜백·발행 주기는 그대로다.
- VM 안의 CPU 를 잴 때(컨테이너별 /proc 표본): 같은 일도 VM 의 vCPU 가 Windows 의 E 코어에 놓이면 CPU 시간이 더 든다. 같은 판의 두
  세션에서 쉬는 스택 전체가 186.8 % 와 273.5 % 로 달랐고, 그때 Windows E 등급 평균이 26.4 % 와 46.7 % 였다. 세션 사이 전후 비교는 바뀌지
  않은 프로세스(gz sim, ros_gz_bridge, 카메라·LiDAR 브리지, cpu_lidar, 녹화기)의 합으로 나눈 몫으로 한다. `docker stats --no-stream`
  한 번 읽기는 크게 틀리므로(첫 표본) 흐르는 docker stats 나 /proc 두 번 읽기를 쓴다.
- 잰 것(개발 확인, 자리값 기반 SIM 결과, 기록 폴더 `runs/gsim_stage7_20261114/`): 쉬는 측정 스택(대시보드 클라이언트 없음)에서 브리지
  래퍼의 몫(바뀌지 않은 프로세스 합에 대한 비)이 0.256 -> 0.144(약 44 % 줄음, wf9 에서 22.6 % 이던 것이 같은 조건 환산 약 12.7 %).
  에뮬레이터(0.242 -> 0.256)와 센서 계약(0.248 -> 0.232)은 노드 옵션으로 가를 만큼 줄지 않았다(콜백 자체가 일의 대부분: 에뮬레이터는
  /clock 마다 펌웨어 걸음과 /sim/act·truth 발행, 이것은 SIM 충실도라 그대로). 쉬는 스택 전체로는 약 10 % 코어(약 5 %)를 줄였다.
- 남은 큰 몫(쉼, wf9, 한 코어 = 100 %): Gazebo 서버 약 43, 센서 계약 약 22, ros_gz_bridge 약 16(카메라 원 영상 포함), 카메라 WebSocket
  브리지 약 13, 에뮬레이터 약 21, 실행기 약 17, 녹화기 약 15. 카메라 경로와 녹화기는 고정 파일이라 이 단계에서 바꾸지 않았다.
