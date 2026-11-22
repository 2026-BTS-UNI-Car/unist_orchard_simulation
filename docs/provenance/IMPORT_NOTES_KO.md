# 반입 기록 (2026-11-12)

이 저장소의 첫 내용이 어디에서 왔고, 무엇을 가져오고 무엇을 뺐으며, 가져오면서 무엇을 바꿨는지 적는다.
파일별 출처표는 `IMPORT_MANIFEST.json` (같은 폴더) 이다.

## 1. 출처

| 항목 | 값 |
|---|---|
| 옛 저장소 | `2026-BTS-UNI-Car/orchard-rover-clone` (비공개. 2026-11-14 보관(읽기 전용)으로 바꿨고 지우지 않는다) |
| 브랜치와 커밋 | `steering-sim-20261108`, 커밋 `50fc854ee2462e761e7ce281dca937f1700e0f2b` |
| 원본 프로젝트 | `pathcmd95/orchard-rover` (Apache-2.0), 커밋 `c3c1f8657ab0526f8ff2221064ebc145e0a0831a` |
| 옛 커밋의 추적 파일 | 2216개, 102.85 MB |
| 가져온 파일 | 598개, 12.62 MB |
| 새로 쓴 파일 | 5개 (`README.md`, `docs/provenance/IMPORT_MANIFEST.json`, `docs/provenance/IMPORT_NOTES_KO.md`, `tools/steer_sim/PIN_MANIFEST.json`, `tools/steer_sim/check_import_manifest.py`) |

옛 저장소의 이력은 가져오지 않았다. 이 저장소의 이력은 새로 시작하고, 옛 커밋과는 출처표로 잇는다.
가져온 파일은 작업 트리를 거치지 않고 옛 커밋의 blob 을 그대로 옮겼다. 그래서 아래 4절에 적은 파일을 뺀 모든 파일의
blob ID 가 옛 커밋 `50fc854` 의 blob ID 와 같다. 실행 권한 표시(100755)도 그대로다.

## 2. 가져온 범위

기준은 '조향 시뮬레이션이 실제로 쓰는가'다. Python import 를 정적으로 추적한 결과와, 실행 구성
(`sim/ubts/steering/supervisor_steer.py`, `compose.ubts-steer.yaml`, `docker/Dockerfile.ubts-steer`)이 가리키는
파일·폴더를 합쳤다. 경로는 옛 저장소와 같다(이미지의 `/opt/ubts` 경로, 실행기의 SHA256SUMS, vendor manifest 가 경로에
묶여 있다).

| 커밋 묶음 | 분류 | 파일 수 | 크기 |
|---|---|---|---|
| 1 | 저장소 규칙 파일 | 2 | 1.5 kB |
| 1 | 라이선스와 고지 | 3 | 16.7 kB |
| 2 | 원본 과수원 월드 생성기 (orchard_gazebo) | 15 | 120.5 kB |
| 2 | 원본 주행 패키지의 일부 (orchard_navigation) | 4 | 21.2 kB |
| 3 | 다시 쓰는 SIM 기반 모듈 | 23 | 254.4 kB |
| 3 | 실행이나 시험 때 읽는 파일 (rover.yaml, fastdds.xml, health.py, bridge.yaml) | 4 | 7.0 kB |
| 3 | 한 열 폐루프 SIM 패키지 | 15 | 512.6 kB |
| 3 | 센서만 쓰는 운동 추정 패키지 | 12 | 158.3 kB |
| 3 | 스캔 통로 검출 패키지 | 7 | 77.2 kB |
| 3 | 기반 모듈 시험 | 13 | 103.0 kB |
| 3 | 폐루프 SIM 시험 | 15 | 261.5 kB |
| 4 | 대시보드 사본의 줄 끝 규칙 | 1 | 0.1 kB |
| 4 | 대시보드 사본의 출처 메모 | 1 | 8.4 kB |
| 4 | 대시보드 설정 | 2 | 1.7 kB |
| 4 | 대시보드 서버 패키지 | 53 | 635.7 kB |
| 4 | 대시보드 브리지·기록기 모듈 | 24 | 291.3 kB |
| 4 | 조향 V3 호스트 브리지와 시험 | 11 | 279.6 kB |
| 4 | 대시보드 정적 파일 | 41 | 1186.2 kB |
| 4 | 대시보드의 제3자 웹 라이브러리 | 15 | 2536.9 kB |
| 5 | 이미지·스택 정의 | 4 | 16.6 kB |
| 5 | 조향 펌웨어 소스 | 24 | 88.9 kB |
| 5 | 현장 월드 생성기와 좌표 검사 | 10 | 155.2 kB |
| 5 | 조향 시뮬레이션 패키지 | 174 | 2321.6 kB |
| 5 | 조향·현장 월드 시험 | 29 | 841.2 kB |
| 6 | 현장 보정 도구 | 9 | 130.9 kB |
| 6 | 한 열 실행기 | 62 | 1504.9 kB |
| 6 | 저장소 검사 도구 | 4 | 27.8 kB |
| 7 | 조향 문서 | 21 | 1054.2 kB |

출처표의 `category` 값이 이 분류다(영문 이름).

처음 후보 목록(600개)에서 달라진 것:

- 더함: `ros2_ws/src/orchard_navigation/orchard_navigation/__init__.py`
- 더함: `ros2_ws/src/orchard_navigation/orchard_navigation/headland.py`
  (`sim/ubts/autonomy.py` 가 `orchard_navigation.headland` 를 import 한다. 없으면 `adapter` 를 import 할 수 없다.)
- 뺌: `sim/ubts/test_adapter_clock.py`
- 뺌: `sim/ubts/test_adapter_fault_transition.py`
- 뺌: `sim/ubts/test_adapter_qos.py`
- 뺌: `sim/ubts/test_mesh_raycast.py`
  (rclpy 나 trimesh 가 있어야 도는 옛 시험이다. Docker 없이 확인할 수 없어 뺐고, 이미지 안에서 통과를 확인한 뒤 다시 본다.)

## 3. 가져오지 않은 것

| 옛 위치 | 파일 수 | 크기 | 내용 |
|---|---|---|---|
| `artifacts/validation` | 309 | 29.7 MB | 차동 구동 시기의 검증 JSON·화면 |
| `dashboard/docs` | 75 | 17.8 MB | 대시보드 문서와 그림 |
| `models` | 4 | 14.8 MB | 비전 모델 (하나는 AGPL-3.0) |
| `docs/media` | 5 | 14.2 MB | 원본 설명 영상·그림 |
| `docs` | 142 | 4.2 MB | 날짜 붙은 상태 문서와 원본 설명서 (docs/steering 은 가져옴) |
| `tools/native_mapping_pc` | 196 | 2.7 MB | PC 네이티브 매핑 도구 |
| `sim/ubts` | 181 | 1.6 MB | 옛 실험 모듈, 옛 supervisor·verify, 가져온 모듈과 무관한 옛 시험 |
| `dashboard/tests` | 121 | 1.0 MB | 대시보드 시험 (실제 대시보드 저장소에 있음) |
| `ros2_ws/src` | 110 | 0.6 MB | 3D LiDAR·PX4·인식·계획 패키지와 orchard_navigation 의 나머지 |
| `dashboard/deploy` | 20 | 0.2 MB | 실차 배포 설정 |
| `dashboard/scripts` | 24 | 0.2 MB | 조향 스택이 쓰지 않는 스크립트 |
| 그 밖 | 431 | 3.31 MB | 지도 연동 도구(cartographer 계열, sim_native 계열), 옛 Compose 파일, CI 작업 정의, PX4 설정, 루트 README 등 |

합계 1618개, 90.24 MB. 필요해지면 보존해 둔 옛 저장소에서 같은 방식(blob 그대로, 출처표에 추가)으로 가져온다.
CI 작업 정의는 넣지 않았다. 이 저장소로 올리기 전의 검사는 저장소 밖의 push 검사 도구가 맡는다.

## 4. 의도적으로 바꾼 파일

아래 파일만 옛 커밋과 내용이 다르다. 출처표에서 `modified: true` 인 항목과 같다.

다시 쓴 파일(이 저장소에 실제로 든 내용에 맞춤):

- `NOTICE`, `third_party/README.md`: 원본 프로젝트와 커밋, 그대로 가져온 파일과 고친 파일, 실제로 든 제3자 구성요소만 적었다.
- `.gitattributes`: 줄 끝 규칙을 줄인 트리에 맞췄다. CRLF 나 섞인 줄 끝으로 저장돼 있던 파일 4개와 제3자 웹
  라이브러리는 바이트를 그대로 둔다.
- `.gitignore`: 결과 폴더, 빌드 산출물, 기록 파일(bag, mcap, db3), 모델 파일(onnx, pt), 비밀 정보 파일.

고친 파일:

- `dashboard/.gitattributes`: 위 4개 파일의 `-text` 줄을 더했다(이 폴더의 `*.py` 규칙이 맨 위 규칙보다 먼저 적용되기 때문).
- `sim/ubts/field_response.py`: 모듈 설명문 9행의 자료 폴더 이름 표기를 바꿨다. 코드는 그대로다(설명문을 뺀 구문 트리가 같다).
- `sim/ubts/field_world/coord_lint.py`: 확장자와 상관없이 모든 텍스트 파일을 읽는 `--all-text` 모드를 더했다. 기본 모드의
  결과는 그대로다.
- `tools/steer_sim/check_pins.py`: 고정 목록을 `tools/steer_sim/PIN_MANIFEST.json` 에서 읽는다. 비교할 원격 브랜치가
  없을 때(첫 push 전)는 반입 때의 Python 파일 목록을 기준으로 새 모듈 이름을 본다.
- `tools/steer_sim/regen_vendor.py`: 저장소 이름과 브랜치 상수를 이 저장소의 값으로 바꿨다.
- `tools/steer_sim/PIN_ALLOWLIST.json`: 비웠다. 옛 저장소에서 예외로 두던 7개 파일은 지금 내용 그대로 고정 목록에 들어 있다.

이 저장소의 커밋을 기준으로 다시 만든 파일:

- `tools/row_executor/vendor/SOURCE_MANIFEST.json`: `source_commit` 이 이 저장소의 커밋을 가리킨다(옛 값은 `derivation` 에 남는다).
  vendor 사본 6개의 내용은 그대로다.
- `tools/row_executor/SHA256SUMS`: manifest 가 바뀌어 그 한 줄이 달라졌다. 그래서 실행기가 기록하는 SHA256SUMS digest 가
  옛 기록과 다르다.

새로 쓴 파일: `README.md`, `docs/provenance/IMPORT_MANIFEST.json`, `docs/provenance/IMPORT_NOTES_KO.md`, `tools/steer_sim/PIN_MANIFEST.json`, `tools/steer_sim/check_import_manifest.py`.

## 5. 알려 둘 것

- **좌표 검사.** `field_world.coord_lint` 의 판정 항목(latlon_pair, yaml_origin, gnss_path)은 트리 전체에서 0건이다.
  엄격 쌍(소수 4자리 이상) 1건은 `ros2_ws/src/orchard_gazebo/orchard_gazebo/world_gen.py` 에 있는 원본 프로젝트의 월드
  원점 값이다. 공개된 원본 파일을 바꾸지 않고 가져온다는 결정에 따라 그대로 두었다.
- **문서의 경로 표기와 옛 설명.** `docs/steering/` 문서에는 개발 PC 의 결과 폴더 경로 표기가 남아 있었다. 바이트 동일성 확인
  범위를 줄이지 않으려고 반입 때는 고치지 않았고, 2026-11-22 자리표시로 바꿨다(다음 항목). 같은 문서의 일부 설명(옛 저장소와 브랜치 이름,
  옛 고정 파일 목록, push 규칙)도 옛 저장소 기준이므로 뒤에 개정한다.
- **공개 전에 거른 표기.** 반입 때는 비공개 저장소였고, 가져온 코드와 문서에 팀 내부 도메인 이름 2건, 장비의 홈
  경로 2건, 사설망 주소 13건, 개발 PC 와 NAS 의 경로 표기 73건(`docs/steering/`)이 들어 있었다. 토큰, 비밀번호, 인증서는
  없고, GNSS 좌표 꼴의 값은 위 원본 월드 원점 1건뿐이다. 2026-11-22 공개 준비 점검에서 문서와 주석의 개발 PC·NAS·장비
  경로를 자리표시(`<작업 폴더>`, `<runs>` 등, [`../steering/PATHS_KO.md`](../steering/PATHS_KO.md) 0절)로 바꿨다. 남은 것:
  대시보드 사본(`dashboard/`)의 팀 도메인 이름과 장비 홈 경로는 대시보드 저장소에서 고친 뒤 사본을 다시 받아야 한다.
  사설망 주소(Jetson AP 와 실행기 운영자 화면의 기본 바인드 주소)는 밖에서 닿지 않는 설정 기본값이라 그대로 둔다.
  경로 처리 시험(`sim/ubts/test_steering_scene_spec.py`)의 드라이브 경로는 시험 값이다. 이 저장소의 이력(지난 커밋)에는
  고치기 전 표기가 그대로 있다.
- **기반 이미지.** `docker/Dockerfile.ubts-steer` 의 기반은 개발 PC 의 로컬 이미지다. 그 기반을 만든 옛 정의
  `docker/Dockerfile.ubts` 는 가져오지 않은 옛 Compose 파일을 복사해 이 저장소에서는 빌드되지 않으므로 2026-11-22 트리에서
  뺐다(반입 커밋과 `IMPORT_MANIFEST.json` 에 남는다. 고정 목록의 예외는 `tools/steer_sim/PIN_ALLOWLIST.json`). 스스로 서는
  기반 정의는 `docker/Dockerfile.base`([`BASE_IMAGE_KO.md`](BASE_IMAGE_KO.md))이고 아직 빌드·검증하지 않았다. 이미지 안에서
  도는 시험과 Gazebo 스택 검사는 그때 다시 확인한다.
- **원본 생성기 시험 1개.** `ros2_ws/src/orchard_gazebo/test/test_generators.py` 의
  `test_model_adds_sensors_and_keeps_px4_sensors` 는 가져오지 않은 `orchard_description/config/rover.yaml`
  (원본의 3D LiDAR·카메라 장착 설정)을 읽어서 실패한다. 조향 시뮬레이션은 그 모델 생성 경로를 쓰지 않는다. 같은 파일의
  나머지 시험 5개는 통과한다.
- **작업 트리가 하나일 때 건너뛰는 시험.** `sim/ubts/test_steering_evaluate.py` 의
  `test_repository_guard_covers_every_work_tree` 는 같은 저장소의 다른 작업 트리가 있어야 돈다.
- **대시보드 사본.** 서버·브리지·정적 파일만 가져왔다. 대시보드의 문서 보기 화면은 문서 폴더가 없어 목록만 나온다.
  조향 V3 호스트 브리지는 지금 이 사본에만 있다.

## 6. 확인 방법

```bash
python -I tools/steer_sim/check_import_manifest.py --strict              # 출처표가 트리와 정확히 맞는지 (반입 직후 상태)
python -I tools/steer_sim/check_import_manifest.py --old-repo <옛 저장소>  # 옛 커밋의 blob ID 와 대조
python -I tools/steer_sim/check_import_manifest.py                       # 그 뒤: 반입 뒤 바뀐 파일 수를 알려 준다
```
