# 외부 데이터 경로 설정 (조향 SIM 브랜치)

- 작성: 2026-11-08 (설계 8.3 P2 "절대 경로 정리")
- 코드: `sim/ubts/closed_loop/external_paths.py`
- 목적: 저장소 코드에 이 PC의 절대 경로를 두지 않는다. 외부 데이터의 위치는 환경 변수나 저장소 밖 JSON 설정 파일에서 읽는다.

## 1. 찾는 순서

1. 항목마다 정한 환경 변수. 값이 비어 있지 않을 때만 쓴다.
2. 환경 변수 `UBTS_STEER_PATHS`가 가리키는 JSON 파일의 같은 이름 키.
3. 둘 다 없으면 `external_paths.PathNotConfigured`를 낸다. 이 오류는 `FileNotFoundError`의 하위 클래스다. 메시지에 환경 변수 이름, 키 이름, 이 문서 이름이 들어 있다.

- 내장 기본 경로는 없다. 설정이 없을 때 다른 경로로 넘어가지 않는다.
- 값은 있는데 그 경로가 없으면 `FileNotFoundError`를 낸다. 메시지에 값이 어디서 왔는지(환경 변수 또는 설정 파일)가 들어 있다.
- 옛 코드는 환경 변수 값이 틀려도 내장 경로로 넘어갔다. 새 코드는 넘어가지 않는다.

## 2. 항목

| 키 | 환경 변수 | 위치 | 쓰는 곳 |
|---|---|---|---|
| `maps_1010_root` | `UBTS_MAPS_1010` | 10/10 지도 폴더. 안에 `offline/maps/<record>/`가 있어야 한다 | `raster_corridor.maps_root`, `raster_catalogue.trajectory_path` |
| `corridor_catalogue` | `UBTS_SINGLE_ROW_CATALOGUE` | 단일열 회랑 카탈로그 `CATALOGUE.json` | `raster_corridor.catalogue_path` (`load_catalogue`, `corridor_world_config`) |
| `rows_root` | `UBTS_ROWS_1010` | 10/10 열 분석 폴더(`rows_offline_<record>.json`) | `raster_catalogue.rows_path` |
| `hold_dir` | `UBTS_HOLD_DIR` | `HOLD_MEASUREMENT` 파일을 찾는 폴더 | `single_row_eval.main` (`external_paths.hold_file`) |
| `baseline_546fa8c_traces` | `UBTS_BASELINE_546FA8C_TRACES` | 546fa8c 기준 trace 폴더(`MANIFEST.json`) | `test_closed_loop_harness_hooks.py`(시험 전용) |

- 환경 변수 `UBTS_MAPS_1010`, `UBTS_SINGLE_ROW_CATALOGUE`는 옛 코드에도 있던 이름이다. 나머지 셋은 새로 정했다.

## 3. 설정 파일 형식

```json
{
 "schema": "ubts.steer_sim.paths.v1",
 "note": "자유 설명",
 "maps_1010_root": {"nt": "D:/Develop/UBTS/data/maps/maps-1010-20261022", "posix": "/mnt/d/Develop/UBTS/data/maps/maps-1010-20261022"},
 "corridor_catalogue": "D:/Develop/UBTS/data/prereg/single-row-20261102/corridors/CATALOGUE.json",
 "rows_root": "data/rows"
}
```

- 값은 경로 문자열이다. 또는 `{"nt": ..., "posix": ...}` 객체다. 객체면 `os.name`에 맞는 값을 쓴다. 그래서 한 파일을 Windows와 WSL에서 같이 쓸 수 있다.
- 상대 경로는 설정 파일이 있는 폴더 기준이다.
- 허용하는 최상위 키는 위 다섯 개와 `schema`, `note`뿐이다. 모르는 키, 잘못된 값 형식, 다른 `schema`는 오류다. 오타가 조용히 지나가지 않게 했다.
- 파일을 읽을 수 없거나 JSON이 아니면 오류다.

## 4. 이 PC의 설정

- 파일: `D:/Develop/UBTS/steering-sim-20261108/config/paths.json` (저장소 밖).
- 값: 2026-11-14에 현장 자료를 작업 폴더의 `data/` 아래(`recordings/`, `maps/`, `prereg/`)로 옮기고 이 파일과 실행기·구동 응답 설정의 경로만 고쳤다(폴더 이름과 안의 파일은 그대로). `nt`는 Windows 경로(`D:/...`), `posix`는 WSL 경로(`/mnt/d/...`)다. 코드와 문서의 주석에 남은 옛 폴더 이름(`maps-1010-20261022/...`, `single-row-20261102/...`, `orchard-realdata-audit/...`)은 출처를 가리키는 이름이고 위치는 이 설정이 정한다.
  - `maps_1010_root`: `D:/Develop/UBTS/data/maps/maps-1010-20261022`
  - `corridor_catalogue`: `D:/Develop/UBTS/data/prereg/single-row-20261102/corridors/CATALOGUE.json`
  - `rows_root`: `D:/Develop/UBTS/data/maps/maps-1010-20261022/quality/rows`
  - `hold_dir`: `D:/Develop/UBTS`
  - `baseline_546fa8c_traces`: `D:/Develop/UBTS/steering-sim-20261108/baseline_546fa8c/traces`
- 켜는 방법:
  - PowerShell: `$env:UBTS_STEER_PATHS = 'D:/Develop/UBTS/steering-sim-20261108/config/paths.json'`
  - Git Bash: `export UBTS_STEER_PATHS=D:/Develop/UBTS/steering-sim-20261108/config/paths.json`
  - WSL: `export UBTS_STEER_PATHS=/mnt/d/Develop/UBTS/steering-sim-20261108/config/paths.json`
  - Docker: 데이터와 설정 파일을 컨테이너 안 같은 `/mnt/d/...` 경로에 읽기 전용으로 붙이고 `-e UBTS_STEER_PATHS=...`를 준다. 또는 항목별 환경 변수를 준다(환경 변수가 설정 파일보다 먼저다).

## 5. 설정이 없을 때

- 라이브러리 함수는 위 오류를 낸다. 예: `raster_corridor.maps_root()` → "the 10/10 maps folder (offline/maps/<record>/) is not configured: set UBTS_MAPS_1010, or 'maps_1010_root' in the JSON file named by UBTS_STEER_PATHS ...".
- `single_row_eval.main`은 hold 폴더가 설정되지 않으면 에피소드를 하나도 돌리지 않고 오류로 멈춘다. 측정 보류 표시를 놓치지 않게 하려는 것이다.
- 실제 데이터가 필요한 시험은 이유를 적고 건너뛴다.
  - `test_closed_loop_raster_corridor.py`의 실제 10/10 시험 2개.
  - `test_closed_loop_harness_hooks.py`의 546fa8c 기준 비교 7개. 기준을 기록한 환경(Python, numpy, 플랫폼)과 지금 환경이 다를 때도 건너뛴다. 환경이 바뀌면 비트 비교가 무효이기 때문이다.

## 6. 옛 코드와 바뀐 것

| 파일 | 옛 코드 | 새 코드 |
|---|---|---|
| `raster_corridor.py` | `MAP_ROOTS`, `CATALOGUE_PATHS`(Windows·WSL 절대 경로) | 지움. `maps_root()`, `catalogue_path()`가 `external_paths`를 부른다 |
| `raster_catalogue.py` | `ROWS_ROOTS`, 사용법 문구의 절대 경로 | 지움. `rows_path()`가 `external_paths`를 부른다. 사용법은 `--out <corridors output folder>` |
| `single_row_eval.py` | `HOLD_FILE = Path(__file__).resolve().parents[4] / 'HOLD_MEASUREMENT'` | `main()` 안에서 `external_paths.hold_file()` |

- `parents[4]`가 실제로 가리킨 곳(2026-11-08 확인):
  - 이 worktree(Windows): `D:\Develop\UBTS`. 단일열 worktree와 깊이가 같아서 같은 폴더였다.
  - WSL에서 본 이 worktree: `/mnt/d/Develop/UBTS`.
  - 546fa8c 추출본: `D:\Develop\UBTS\steering-sim-20261108\baseline_546fa8c`. 다른 폴더다.
  - Docker에서 `/work/tree`로 붙인 경우: `/work`. 다른 폴더다.
  - 소스 위치에 따라 결과가 달라진다. 그래서 설정 항목으로 바꿨다.
