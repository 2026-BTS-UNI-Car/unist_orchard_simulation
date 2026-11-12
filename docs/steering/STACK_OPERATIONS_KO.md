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

옛 표시 파일 `HOLD_SIM_MATRIX`, `HOLD_PROTOCOL.txt`는 2026-11-12에 `D:/Develop/UBTS/_archive/root_files/`로 옮겼다(W-15). 다시 만들지 않는다. 대신 시작 전에 아래를 확인하고, 그 결과를 실행 기록에 남긴다.

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

- compose 기본 이미지는 `ubts-steer-sim:wf4b`다(실행기 패키지와 S2 형식 노드 포함). 그냥 `up`이면 `/scan`(실차 형식)이 나온다. 실행기는 `--profile executor`로 켠다. 측정 결과는 `S2_NODE_EXEC_KO.md` 7절.
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

- 개요(Overview) 화면에는 PC 주소가 보인다. rover-control 화면만 갈무리하고, 넣기 전에 사용자 이름·주소·좌표가 없는지 눈으로 본다.
- 실행기와 검사를 같이 쓰는 스택(대시보드 없음)에서는 화면 갈무리를 할 수 없다. 화면은 기본 스택에서, 주행 영상은 실행기 스택에서 따로 갈무리한다.
