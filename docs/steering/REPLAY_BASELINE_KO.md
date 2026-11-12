# 10/10 기록 재생 도구 재이식과 인지 기준선 재현 (PC-04)

- 작성: 2026-11-12. 작업 번호 PC-04.
- 대상: `tools/row_executor/replay_adapters.py`, `row_shadow_replay.py`, `replay_executor_shadow.py`, `row_gate_report.py`, `tests/test_replay_gate.py`(단일열 작업 커밋 `546fa8c`에서 가져옴), `sim_paths.py`의 `measurement_hold_file()`.
- 이 문서의 수치는 **2026-10-10 수동 주행 기록을 개방 루프로 재생한 결과**다. 차는 사람이 몰았고 제어기 명령은 적용되지 않았다. 차량은 조향 장치를 달기 전의 차동 구동 차다. 닫힌 고리 조향 성능이나 현장 성능의 근거가 아니다.
- 이 문서에는 GNSS 좌표, 지도 원점, 접속 정보가 없다. 입력 파일은 자료 폴더 이름 대신 `sim_paths.py`의 키 이름으로 적는다.

## 1. 요약

1. 재생 도구 네 개와 그 시험을 `546fa8c`에서 바이트 그대로 가져온 뒤, 이 PC의 절대 경로와 보조 도구 이름이 든 문자열만 고쳤다(3절). 계산 코드는 바꾸지 않았다.
2. 10/22 보고서의 '엄격 조기 정지' 집계 스크립트는 보관돼 있지 않았다. 보고서의 정의대로 `row_gate_report.premature()`를 다시 만들었고, 보관된 10/22 run 요약 30묶음(변형 15개 × 출발 종류 2)에 돌려 보관된 집계 파일과 모든 칸이 같음을 확인했다(4절).
3. 같은 설정(`c_proposed`)으로 10/10 기록 6개를 다시 재생했다. **추종 거리 1,081.7 m, 정지 36회, 엄격 조기 정지 8회**로 10/22 결과와 같다. 허용 ±1 % 안이 아니라 run 162개의 모든 판정과 틱 기록 28,824행이 바이트까지 같다(5절). SIM 그대로 설정(`c_base`)도 같다(700.3 m, 89회, 33회, 틱 20,626행).
4. 입력 기록 18개 파일의 SHA-256과 수정 시각이 재생 전후에 같다(6절). 기록은 읽기만 했다.
5. 10/22의 추정기 사본(`1eb9c07`)과 지금 사본(`546fa8c`에서 온 것, 1,236 → 1,499줄)은 다르다. 그 사이 더해진 것은 모두 기본 꺼짐 선택 기능이라 기본 설정의 결과가 같다는 것을 실기록으로 확인한 셈이다.

## 2. 무엇을 재현했나

10/22 C1 gate 보고서(2026-10-22 현장 자율주행 작업 폴더의 `gate/C1_GATE_REPORT.txt`)의 표 5 'P1+P2+P3 (c_proposed)' 행이다.

- 재생: `row_shadow_replay.py`. 10/10 기록의 스캔과 IMU를 실행기의 `core.ScanAdapter`, `core.ImuYawRate`로 바꿔 vendor 추정기에 넣고, 20 Hz로 vendor 제어기 네 개(continuous, continuous_side × SIM 기본, 주말 듀티)를 돌린다. 명령은 적용되지 않는다. run마다 제어기의 첫 정지 결정까지만 채점한다.
- 사람 동작의 참조는 오프라인 ICP(`lidar_motion.npz`)다. 추정기에 넣지 않는다.
- run: 사람이 3 m 이상 전진한 구간 153개(1,765.2 m). flying은 153개 전부, standing은 1 s 이상 정지 뒤 출발한 9개다.
- 설정 `c_proposed`: 근거리 반환 2-scan 확인(`near_m` 2.0, `tol_m` 0.35, `confirm_scans` 2), `imu_max_age_s` 0.4, 기하 가드(방향 25°, 차체 반폭 0.35 m, 여유 0). `c_base`: 추가 설정 없음.
- 표의 기준 제어기는 `continuous_side/weekend`다.
- 엄격 조기 정지: 직선 중간 정지(정지 뒤 사람이 3 m 이상 직진했고 구간 끝까지 3 m 이상 남음) 가운데 FOLLOW 뒤 1 m 이상 간 것이고, 사유가 `front_blocked`면 0.45 m/s의 정지 요구 거리로도 걸리는 것만 센다.

## 3. 가져온 파일과 바꾼 곳

`git show 546fa8c:tools/row_executor/<파일>`의 바이트를 복사했다(줄바꿈 LF).

| 파일 | 546fa8c SHA-256 | 바꾼 것 |
|---|---|---|
| `replay_adapters.py` | `e7b2713702db428aed924a9a551324b506d5c9abc01c070a0725e0d1a68473e6` | 없음(바이트 같음) |
| `row_shadow_replay.py` | `854aa9ac2a1522f20c0fa4e857f3f56818b021dc46b97e02e9439b186af151e7` | 경로 5곳(아래) |
| `replay_executor_shadow.py` | `3bd5cc8c1c76a85ec7abfabdd6772cf0af84c1491ffe4229cf9fbae511861fdb` | HOLD 파일 경로 |
| `row_gate_report.py` | `a842c56a1d08c2947d39f3ae54ba6184e81ad75d66fbdccc9cc180f21c6803f0` | 사용 예 문구, 엄격 조기 정지 집계 추가(4절) |
| `tests/test_replay_gate.py` | `2bc3ca390a9ad0cb858cfb704bb7545ce4cca5b782897fecefb83bdea36b6700` | 집계 시험 3개 추가(기존 시험 12개는 그대로) |

바꾼 곳:

| 파일 | 옛 코드 | 새 코드 |
|---|---|---|
| `row_shadow_replay.py`, `replay_executor_shadow.py` | `HOLD_FILE` = 이 PC의 HOLD_MEASUREMENT 절대 경로 | `sim_paths.measurement_hold_file()`. SIM 트리의 `closed_loop/external_paths.py`(키 `hold_dir`)를 파일 경로로 읽는다. SIM 트리 이름을 `sys.path`에 올리지 않는다. SIM 트리가 없는 사본(예: ARM 오프라인 시간 측정)에서는 HOLD 파일이 없다(None). SIM 트리가 있는데 `hold_dir`가 설정되지 않았으면 오류로 멈춘다(`sim_in_loop`과 같은 규칙) |
| `row_shadow_replay.py` | `--mask-dir` 기본값 = 10/22 작업 폴더의 절대 경로(보조 도구 이름 포함) | 기본값 `<out-dir>/masks`. 없으면 재생이 leave-one-out 마스크를 그 폴더에 만든다 |
| `row_shadow_replay.py` | `--base`(10/03 변환 자료용) 설명에 자료 폴더 이름 | 중립 문구. drive 추출 파일 폴더를 따로 주는 `--drive-dir`(환경 변수 `ROW_REPLAY_DRIVE`)를 더했다. 이 저장소의 `field_records.load_record`가 두 위치를 따로 받기 때문이다 |
| `row_shadow_replay.py`, `row_gate_report.py` | 사용 예에 적힌 이 PC의 드라이브 경로 | `<runs folder outside the repository>` 등 |

- 가져오지 않은 것은 그대로다: `convert_1003.py`(10/03 변환, 자료 경로가 박혀 있음), 회전 도구(`EXECUTOR_IMPORT_KO.md` 5절).
- `tests/test_executor_loop.py`의 고정 사본 `ReplayNearTransientFilter`는 그대로 두었다. `replay_adapters.NearTransientFilter`가 다시 들어왔으므로 뒤에 합칠 수 있다.
- 자료 위치: `sim_paths.py`의 `field_manual_1010`(그 아래 `lidar/`), `field_manual_1010_drive`. 저장소는 자료 폴더 이름을 적지 않는다.

## 4. 엄격 조기 정지 집계 다시 만들기

- 10/22 집계 파일(`gate/premature_breakdown.json`)을 만든 스크립트는 작업 폴더와 저장소 어디에도 없다. 보고서 1절의 정의로 `row_gate_report.is_strict_premature()`와 `premature()`를 썼다.
- 낸 값: 추종 거리, 정지 수, 출발 거부 수(FOLLOW 뒤 1 m 안)와 사유, 엄격 조기 정지 수와 사유, 1회당 거리, 율 중앙값(× ln 2), 30/45/60 m를 조기 정지 없이 지날 확률(exp(−L / 1회당 거리)), 엄격 조기 정지를 사건으로 한 Kaplan-Meier 중앙값·p10.
- 확인: 보관된 10/22 run 요약(`gate/runs/<변형>/summary_*.json`) 15개 변형 × 출발 종류 2 = 30묶음에 새 함수를 돌렸다. 보관된 집계 파일의 30개 키 모두, 모든 칸이 같다(다른 키 0). 10/03 변환 자료의 6묶음도 들어 있다.
- 쓰는 법: `python -I -B tools/row_executor/row_gate_report.py --runs-dir <runs> --variants c_base c_proposed --out <metrics.json> --premature-out <premature.json>`.

## 5. 재현 결과

환경: Windows 11, Python 3.13.2, numpy 2.2.6, scipy 1.18.0(10/22와 같다). 작업 트리는 `main` 3289d16에 이 작업의 변경을 더한 것이고 vendor 추정기는 `1e51e6b` 기준 사본(`546fa8c`와 바이트 같음)이다. 재생은 작업자 3개로 돌렸다. 판정은 작업자 수와 무관하다(처리 시간만 달라진다).

flying 153 run, `continuous_side/weekend`:

| 설정 | 출발 실패 | 추종 거리 | 정지 | 직선 중간 | 엄격 조기 정지 | 1회당 거리 | P(30/45/60 m) | 10/22와 차이 |
|---|---|---|---|---|---|---|---|---|
| `c_base` | 19 | 700.3 m | 89 | 60 | 33 | 21.2 m | 0.24 / 0.12 / 0.06 | 없음 |
| `c_proposed` | 17 | 1,081.7 m | 36 | 16 | 8 | 135.2 m | 0.80 / 0.72 / 0.64 | 없음 |

- `c_proposed` 정지 사유: row_end_candidate 14, guard_heading 12, guard_lateral 5, front_blocked 4, corridor_lost 1. 엄격 조기 정지 8: row_end_candidate 5, front_blocked 1, guard_lateral 1, corridor_lost 1. 출발 거부 16.
- standing 9 run: `c_base` 5개 추종 97.0 m 정지 1회, `c_proposed` 5개 추종 102.5 m 정지 0회. 둘 다 출발 실패 4(pair_not_on_dominant_axis 3, ambiguous_corridor_axis 1).
- 사람 전진 시간 중 FOLLOW 비율(`c_proposed`): run 안 0.586, 기록 전체 0.516.

10/22 결과와의 비교(보관된 `gate/runs/c_base`, `gate/runs/c_proposed`):

| 비교 | 결과 |
|---|---|
| run별 요약(구간, 추종 여부, 출발 실패, 첫 정지의 사유·시각·거리, 틱 수, 횡오차 통계) | 162 run × 제어기 4개, 두 설정 모두 다른 run 0 |
| 틱 배열(`ticks_*.npz`: 상태·사유·명령·통로·횡오차·방향·열 끝 거리, 실수 포함) | 12개 파일 모두 같음(`c_proposed` 28,824행, `c_base` 20,626행) |
| 집계(`metrics`, 엄격 조기 정지) | 모든 칸 같음 |
| 다른 것 | 처리 시간(`scan_ms_*.npy`, 요약의 시간 칸), 요약의 `vendor_commit` 칸(`1eb9c07…` → `1e51e6b…`) |

- 판정: 완료 기준 '추종 거리 1,082 m·정지 36회·엄격 조기 정지 8회 재현(±1 %)'을 차이 0으로 충족한다.
- 추정기 사본은 10/22(`1eb9c07`, 1,236줄)와 지금(1,499줄)이 다르다. 그 사이의 변경(열 끝 분절, 차량 기준 한쪽 갱신, 엇갈린 끝, 마지막 줄기, 첫 획득 창, 경로 길이 방식)은 모두 기본 꺼짐이다. 재생 도구는 추정기 기본 설정을 쓰므로 수치가 같다. 제어기와 통로 감지기는 10/22와 바이트가 같다.
- 실행기 고리 재생(`replay_executor_shadow.py`)은 가장 짧은 기록 하나(57 s)로 도는지만 확인했다(예외 0, 1,129틱). 그 도구의 10/23 결과 재현은 이 작업의 범위가 아니다.

## 6. 입력 파일

재생이 읽는 파일은 기록마다 셋이다(스캔, ICP 참조, drive 추출). 재생 전과 후에 SHA-256, 크기, 수정 시각을 적어 비교했다. 18개 모두 같다.

| 입력(`<sim_paths 키>/<파일>`) | 바이트 | SHA-256 |
|---|---|---|
| `field_manual_1010/lidar/20261010T035952Z_both.lidar_motion.npz` | 252788 | `febb8fcfe78e6917bd74daf53a3fc219cded5c9ba78acb88743e40925f992e25` |
| `field_manual_1010/lidar/20261010T035952Z_both.scans.npz` | 21063990 | `70fbbc9605da52a7c7858f6d68163497dd016bb402b03b9f1c18418addd09674` |
| `field_manual_1010/lidar/20261010T040927Z_both.lidar_motion.npz` | 282795 | `67e744521ae68e0926e3f5c3247046e2bbab6e72961e0d708f44f1e4cdfaf43a` |
| `field_manual_1010/lidar/20261010T040927Z_both.scans.npz` | 22621551 | `f3a8e61b87804434f54a58b15b39d6ffeaadf691168c7b20f741192eac47e505` |
| `field_manual_1010/lidar/20261010T042200Z_both.lidar_motion.npz` | 31458 | `cc90257716969cc0d25565634d8d26665b023283a0071e06cc84def21d1ecf99` |
| `field_manual_1010/lidar/20261010T042200Z_both.scans.npz` | 2358556 | `3f138e28fac66709910301f219b9f9ef05dde456ebadf00066c5b7b5f26da61d` |
| `field_manual_1010/lidar/20261010T042301Z_both.lidar_motion.npz` | 370680 | `2b115002fe7874fda940e4c9dbeabee4f3ef523e7d04709787e45a8837df72de` |
| `field_manual_1010/lidar/20261010T042301Z_both.scans.npz` | 25215298 | `bb058f3e6029fdb33b4237dca02cf8e61bc069f7965fcffb8169b2a1c9d9d949` |
| `field_manual_1010/lidar/20261010T045553Z_both.lidar_motion.npz` | 398716 | `b2bbbcb86e93a95657f355299585dc360c1a4f61a874d0265c05e307dc9b99e9` |
| `field_manual_1010/lidar/20261010T045553Z_both.scans.npz` | 32684110 | `6a2e63e674a11ce74a5c8ae86ff2e5b6cb14bc37fe1afa1b719b4d259ac37ee6` |
| `field_manual_1010/lidar/20261010T051127Z_both.lidar_motion.npz` | 482677 | `ab414c93335183bee3cf481aa59d12b8dcfffad8afc58a4445707c05196255ab` |
| `field_manual_1010/lidar/20261010T051127Z_both.scans.npz` | 38638707 | `47a2127a3be7d80133bb7020ab47809346d12328702dc08818f13504d66123c0` |
| `field_manual_1010_drive/20261010T035952Z_both.drive.json.gz` | 686617 | `1e623910cc051afcfa56a49dec305a115a8893be3d968e5651e879022bd22d59` |
| `field_manual_1010_drive/20261010T040927Z_both.drive.json.gz` | 793497 | `32b645d005462a9886af3d2d9e0c6465ee49809ec137a6bd46a41ff425303c0a` |
| `field_manual_1010_drive/20261010T042200Z_both.drive.json.gz` | 82260 | `e8bf189b29a3c64a3c0370e7c5e1a898dd94ca3f91ccc4bacd23f6a6eebfc95c` |
| `field_manual_1010_drive/20261010T042301Z_both.drive.json.gz` | 1038847 | `bebe323f1f7b74b9f5ff897573cc14e1ad61bcf46165f695a524ab3d4aaf33dd` |
| `field_manual_1010_drive/20261010T045553Z_both.drive.json.gz` | 1114470 | `20de1bbea73e61f366c2082ae44c367b4ef7b94fb5da89dabbf8df4688c3e31f` |
| `field_manual_1010_drive/20261010T051127Z_both.drive.json.gz` | 1332152 | `75a7a5acd76179c2b58f1824da467aa475e3dfada69f4d9ba7515acc7a671efb` |

## 7. 돌리는 방법

PowerShell, 작업 트리 루트에서. 출력 폴더는 저장소 밖에 둔다.

```powershell
$env:UBTS_STEER_PATHS = '<저장소 밖 설정 폴더>/paths.json'      # hold_dir, 옆의 row_executor_paths.json(10/10 기록 위치)
cd tools/row_executor
python -I -B row_shadow_replay.py --variant base --name c_base --out-dir <runs> --workers 3
python -I -B row_shadow_replay.py --variant base --name c_proposed --out-dir <runs> --workers 3 `
    --ctl imu_max_age_s=0.4 --spec '{"transient": {"near_m": 2.0, "tol_m": 0.35, "confirm_scans": 2}, "guard": {"heading_deg": 25.0, "body_half_width_m": 0.35, "lateral_margin_m": 0.0}}'
python -I -B row_gate_report.py --runs-dir <runs> --variants c_base c_proposed --out <metrics.json> --premature-out <premature.json>
```

- 걸린 시간(작업자 3, 다른 작업이 같은 PC에서 도는 중): 설정 하나에 약 4~5분. 처리 시간 값은 다른 부하의 영향을 받으므로 기준값으로 쓰지 않는다.
- 시험: `cd tools/row_executor/tests; python -B -m pytest -q -p no:cacheprovider test_replay_gate.py`(15개).

## 8. 한계와 남은 일

- 개방 루프다. 사람이 100 % 듀티(약 0.9 m/s)로 몰았다. 횡오차·방향 오차·포화는 사람 경로의 값이다(10/22 보고서 6절의 한계가 그대로다).
- 10/10 IMU는 20 Hz다. 차는 조향 장치를 달기 전의 차다.
- 열 끝 정답이 없다. 조기 정지 판정은 '사람이 계속 직진했는가'라는 대리값이다.
- 이 기준선은 뒤 작업의 비교 기준이다: 국소 격자 층과 빠른 층의 일치(PC-07), 인지 평가와 ARM 오프라인 비용(PC-17). 추정기 선택 기능을 켠 재생은 하지 않았다.
- 10/03 과수원 기록 재생(`convert_1003.py`)은 가져오지 않았다.
