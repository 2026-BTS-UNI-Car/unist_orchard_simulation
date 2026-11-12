# unist_orchard_simulation

UNIST BTS 유니주행차 팀의 과수원 로버 시뮬레이션 저장소입니다. 앞바퀴 조향 로버가 과수원 한 열을 행간 중심선을 따라
달리고 열 끝에서 서는 동작을, 실차에 올리기 전에 시뮬레이션으로 만들고 확인하는 데 씁니다.

> 이 문서는 뼈대입니다. 구성 그림, 실행 예, 결과 그림은 뒤에 채웁니다.

## 지금 상태

- 조향 장치의 값(조향 모터, 가변저항, 치수)은 모두 자리값입니다. 이 저장소의 수치는 자리값 기반 시뮬레이션 결과이며
  현장 성능이 아닙니다.
- 설계와 결정 기록은 `docs/steering/` 에 있습니다. `docs/steering/DESIGN_v1_KO.md` 부터 읽고, 바뀐 곳은 개정 문서
  `docs/steering/DESIGN_v2_KO.md` 를 봅니다. 지연 값의 근거는 `docs/steering/TIMING_BASELINE_KO.md` 입니다.
- 이 저장소는 옛 시뮬레이션 저장소에서 조향 시뮬레이션이 실제로 쓰는 파일만 골라 옮겨 만들었습니다. 출처와 범위는
  `docs/provenance/IMPORT_NOTES_KO.md`, 파일별 출처표는 `docs/provenance/IMPORT_MANIFEST.json` 입니다.

## 폴더

| 폴더 | 내용 |
|---|---|
| `sim/ubts/steering/` | 조향 시뮬레이션: 차량 모형, 빠른 SIM, Gazebo 모델과 장면, ESP 에뮬레이터, 적합성 대본, 검사 |
| `sim/ubts/field_world/` | 현장 지도에서 만드는 월드와 좌표 검사(`coord_lint`) |
| `sim/ubts/closed_loop/`, `sim/ubts/orchard_perception/`, `sim/ubts/encoderless/` | 한 열 폐루프 SIM, 스캔 통로 검출, 센서만 쓰는 운동 추정 |
| `sim/ubts/` (바로 아래) | 다시 쓰는 기반 모듈(어댑터, CPU LiDAR, 월드 조립 등)과 시험 |
| `firmware/ubts_steer_v3/` | 조향 펌웨어(ESP) 소스. 네이티브 lockstep 프로그램에 함께 컴파일됩니다 |
| `dashboard/` | 조향 스택이 실행 때 쓰는 대시보드 사본(서버, 브리지, 정적 파일)과 조향 V3 호스트 브리지 |
| `ros2_ws/src/orchard_gazebo/`, `ros2_ws/src/orchard_navigation/` | 원본 프로젝트의 과수원 월드 생성기와 제어 참조 |
| `tools/row_executor/`, `tools/field_calib/` | 한 열 실행기와 현장 보정 도구 |
| `tools/steer_sim/` | 저장소 검사 도구(고정 파일, vendor 동기, 출처표) |
| `docker/`, `compose.ubts-steer.yaml` | 조향 SIM 이미지와 스택 정의 |
| `docs/steering/`, `docs/provenance/` | 조향 문서, 반입 출처 기록 |

## 확인 명령

저장소 맨 위에서 실행합니다.

```bash
python -I tools/steer_sim/check_pins.py              # 다시 쓰는 기반 파일이 고정된 내용 그대로인지
python -I tools/steer_sim/check_vendor_sync.py       # 실행기의 vendor 사본, manifest, SHA256SUMS
python -I tools/steer_sim/check_import_manifest.py   # 출처표 (반입 직후 상태 확인은 --strict)
(cd sim/ubts && python -B -m field_world.coord_lint ../.. --all-text)   # 좌표 검사, 트리 전체
```

시험은 파일마다 따로 돌립니다. 예:

```bash
cd sim/ubts && python -B -m pytest -q -p no:cacheprovider test_steering_geometry.py
cd tools/row_executor/tests && python -B -m pytest -q -p no:cacheprovider test_config.py
cd sim/ubts && python -B -m steering.conformance.run_conformance --out <저장소 밖 폴더>/conformance.json
```

저장소 밖 자료(지도, 통로 목록 등)의 위치는 환경 변수 `UBTS_STEER_PATHS` 가 가리키는 설정 파일로 줍니다
(`docs/steering/PATHS_KO.md`). 결과와 측정 파일은 저장소 밖에 둡니다.

## 이미지

`docker/Dockerfile.ubts-steer` 가 조향 SIM 이미지를 만듭니다. 기반 이미지는 지금 개발 PC 의 로컬 이미지를 가리키므로
이 저장소만으로는 아직 이미지를 만들 수 없습니다. 스스로 서는 기반 이미지 정의는 뒤에 더합니다.
`docker/Dockerfile.ubts` 는 그 기반 이미지를 만든 옛 정의이며 패키지 목록을 확인하는 용도로 둡니다.

## 라이선스

Apache-2.0 (`LICENSE`). 원본 프로젝트와 제3자 구성요소 고지는 `NOTICE`, `third_party/README.md` 에 있습니다.
