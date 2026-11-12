# 조향 기록: 사이드카와 별도 bag (P6b, 작업 표지 record)

- 작성: 2026-11-12. 코드는 2026-11-09 4단계 작업 흐름(WF-4)의 record 단계가 썼고, 그 단계는 스택에서 개발 녹화 1회를 돌리던 중 사용 한도로 멈췄다. 변경을 새 저장소로 옮겼다(`DECISIONS_KO.md` R22).
- 설계: `DESIGN_v1_KO.md` 4.5, 8.3 P6b. 합격 기준: `SIM_ACCEPTANCE_KO.md` A23(바꾸지 않았다).
- **Docker 검증 전.** Docker 없이 도는 시험만 확인했다(`test_steering_sidecar.py`, `test_steering_sidecar_bag.py`(POSIX 신호·프로세스 그룹: Linux에서만), `test_steering_sidecar_host.py`). A23 판정과 이 문서의 결과 절은 ST-06이 쓴다. 모든 값은 자리값 기반 SIM 결과이고 현장 검증이 아니다.

## 1. 무엇을 하나

고정된 기록기(`dashboard/scripts/data_recorder_service.py`, `dashboard_server/motor_pwm.py`)는 조향 상태를 기록하지 못한다(토픽 목록이 고정이고 `motor_pwm.py`는 차동 칸만 남긴다, 설계 4.5). 그래서 따로 도는 프로세스가 기록기의 녹화마다 조향 시간열을 옆에 남긴다.

| 모듈 | 하는 일 |
|---|---|
| `sim/ubts/steering/steering_sidecar.py` | 기록기 HTTP `GET /status`(5 Hz)로 녹화 시작·끝을 알아채고(기록기를 고치지 않는 녹화 ID 맞추기), `steering/steering.jsonl`(스키마 `orchard.steering.timeline.v1`, 20 Hz, 설계 4.5의 칸 그대로, `motor_pwm.sanitize_status`처럼 허용 목록과 형·범위 검사), 설명자(`steering_descriptor.json`, SHA), 호스트 프로필 사본(보정표)을 쓴다. 오래된 상태는 stale로 표시하고 단조 시계를 시간 기준으로 쓴다 |
| `sim/ubts/steering/steering_bag_record.py` | `/rover/steering/state`만 따로 받는 `ros2 bag record` 프로세스를 녹화 시작·끝에 맞춰 켜고 끈다(프로세스 그룹, 고아 회수용 pid 파일) |

## 2. 사이드카를 두는 곳 (결정 제안)

- 녹화 폴더 안의 `steering/`(다시 시작하면 `steering-2`, …)에 둔다. 설계 기본안 `<archive_dir>/_steering/<id>/`는 쓰지 않는다.
- 근거: 기록기와 대시보드가 기록 폴더의 모든 하위 폴더를 녹화로 나열한다. `data_recorder_service.py` `list_recordings`(229~241행: `archive_dir.iterdir()`에서 폴더마다 행을 만든다), `dashboard_server/telemetry.py` `recordings`(1051행 이후, 1055행과 1086행의 `iterdir()`). `_steering` 폴더를 만들면 녹화 하나처럼 보인다. 마감 절차와 미디어 작업은 이름이 정해진 파일만 다룬다(코드의 설명, ST-06에서 녹화 1회로 다시 확인).
- 사이드카는 녹화 폴더를 만들지 않는다(기록기가 `exist_ok=False`로 만든다, `data_recorder_service.py` 333행). `rosbag2/`와 `mapping_source/` 안에는 쓰지 않는다(미디어 작업이 그 해시를 잰다).
- 이 결정은 ST-06의 녹화 1회 확인 뒤 결정 기록에 확정한다.

## 3. 남은 것 (ST-06)

- 11/09 개발 녹화의 누락 11.5 %(기록 15.4 Hz)의 원인(상태 전달 경로, 사이드카 시작 지연, 스택 부하).
- 녹화 1회로 A23 항목(스키마, 20 Hz와 누락률, stale 표시, 설명자 SHA, 보정표 사본, 별도 bag 토픽·개수)을 판정하고 이 문서에 결과를 쓴다.
- compose·감독기 연결(기록기 역할에 사이드카와 bag 프로세스 넣기)은 아직 없다. 11/09 개발 녹화는 저장소 밖 덮어쓰기 compose 파일로 돌렸다(`runs/wf4/record/compose.record.yaml`).
