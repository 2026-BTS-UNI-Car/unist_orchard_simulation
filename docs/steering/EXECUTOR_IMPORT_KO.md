# 실행기 가져오기 (설계 8.3 P4: import 폐포, 문자열·경로 정리, s2_format, vendor)

- 작성: 2026-11-09 (3단계 WF-3a, 작업 p4_import)
- 대상: 새 폴더 `tools/row_executor/`, 새 파일 `sim/ubts/steering/s2_format.py`, `tools/steer_sim/regen_vendor.py`, `tools/steer_sim/check_vendor_sync.py`
- 원본: 단일열 작업 커밋 `546fa8c1addd9d3ba15eead1081620afb41a1c5d`. 파일은 `git show 546fa8c:<경로>`로 바이트 그대로 복사한 뒤 고쳤다.
- 이 단계는 동작을 바꾸지 않는다. 차동 경로는 기준선과 비트까지 같다(8절). 조향 출력(`Decision` → throttle·steer_mrad, `SteerOutputGuard`), 안전 대본, 식별 도구는 다음 작업(p4_exec, p4_ident)이 한다.
- 모든 수치는 SIM 결과이고 현장 검증이 아니다.

## 1. 요약

1. 기준선: `546fa8c`의 `tools/row_executor`와 `sim`을 저장소 밖에 풀었다. 실행기 시험 20개 파일(366 통과, 13 건너뜀)과 기준 에피소드 10개를 Windows Python 3.13.2에서 돌렸다. 두 번 돌려 모든 산출물 SHA-256이 같았다.
2. import 폐포를 AST로 셌다(함수 안 import, 지연 import, `importlib`, `spec_from_file_location`, 하위 프로세스 포함). 설계 목록 밖에 더 필요한 것은 `field_records.py` 하나다(`body_mask_fit.py`가 import한다).
3. 55개 파일을 가져왔다(코드 18, 설정 예시 2, vendor 7, 문서·실행 스크립트 5, 시험 23). 20개는 가져오지 않았다(회전·재생·변환·보고 도구와 그 시험).
4. `executor_main.py`에서 `turn_observation` import와 사용처를 지웠다. `turn_observe` 요청은 모든 모드에서 거부된다. 세 진입점 어느 것을 import해도 `turn_*` 모듈이 올라오지 않는다(새 시험).
5. 보조 도구 이름 문자열을 중립 문구로 바꿨다. 이 PC의 절대 경로를 코드·시험·문서에서 지웠다. 경로는 환경 변수, 저장소 밖 설정 파일, 저장소 기준 기본값으로 찾는다(`tools/row_executor/sim_paths.py`, SIM 트리 `closed_loop/external_paths.py`).
6. `sim_in_loop.RealFormat`을 `sim/ubts/steering/s2_format.py`로 옮겼다. 코드는 한 글자도 바꾸지 않았다(AST 같음). `sim_in_loop`은 같은 이름으로 다시 내보낸다.
7. vendor 재생성 도구와 동기 검사 도구를 만들었다. 작업 트리 기준(`WORKTREE-UNCOMMITTED`)으로 다시 만들었고 `executor_main.verify_sources()`, `core.verify_vendor()`, `check_vendor_sync.py`가 통과했다.
8. 비트 동일: 에피소드 결과 행, 하네스 trace, 단일열 trace, 실행기 JSONL 기록이 기준선과 같다. 다른 것은 JSONL의 `vendor_commit` 한 칸뿐이다(8.3절).
9. 시험: Windows 3.13.2 23개 파일 391 통과·13 건너뜀·실패 0, s2_format 15 통과. 기반 이미지 3.12.3 391 통과·13 건너뜀·실패 0, s2_format 15 통과. WSL 3.10.12는 import 점검 통과, scipy가 필요 없는 13개 파일과 s2_format 통과, 나머지 10개 파일은 그 환경의 scipy 문제로 실패(DECISIONS R10).
10. 남은 문제였던 것: `check_pins.py`가 vendor 꾸러미의 `__init__.py` 두 개를 이름 가림으로 봤다(11절). 검토 P4R-02 처리로 vendor 꾸러미 이름을 바꾸고 `sys.path`에 올리지 않게 했다(13절). 이제 `check_pins`가 OK다.

## 2. 기준선 (저장소 밖)

- 추출: `git archive 546fa8c tools/row_executor sim` → `D:/Develop/UBTS/steering-sim-20261108/baseline_546fa8c_exec/src/` (343개 파일). 단일열 worktree에서는 아무것도 돌리지 않았다. `python -B`로만 돌렸다.
- 환경: Windows 11, Python 3.13.2, numpy 2.2.6, scipy 1.18.0, pytest 9.1.1.
- 대시보드 `rover_control_core.py`: 기준선은 실차 대시보드 사본(`dashboard-live-raw`)을 읽었다. SHA-256 `5cc86b226e880406a849ad958ba92812e6c4f2c03e2b427592209f0b1680b92e`. 이 저장소의 `dashboard/scripts/rover_control_core.py`도 같은 값이다.

### 2.1 실행기 시험

- 실행기 시험은 파일마다 pytest 프로세스 하나로 돌린다. vendor를 쓰는 시험과 SIM 트리 `closed_loop`을 import하는 시험(`test_field1010_closed_loop`, `test_navigation_observation`, `test_single_row_sim`)을 한 프로세스에 모으면 수집 단계에서 SIM 트리가 먼저 올라와 `core.load_vendor()`가 `VendorError`를 낸다. 단일열 작업도 파일마다 돌렸다(`executor_tests_per_file` 기록). 처음에 한 프로세스로 돌린 실행(146 실패)은 이 이유로 무효이고 기록만 남겼다.
- (fix_wf3a 뒤) vendor가 더는 SIM 트리 이름을 쓰지 않는다(13절). 실행기 시험 전체를 한 pytest 프로세스로 돌려도 된다: 438 통과, 13 건너뜀(Windows 3.13.2).
- 결과: 20개 파일 모두 종료 코드 0, 366 통과, 13 건너뜀(Linux 전용 브리지 시험 12, `/dev/full` 1).
- 기록: `runs/wf3a/p4_import/baseline_tests/per_file_win313/RESULT.json`.

### 2.2 기준 에피소드

`runs/wf3a/p4_import/tools/episodes_driver.py`가 한 프로세스에서 순서대로 돌린다(작업자 풀 없음). 산출물마다 정규 JSON(키 정렬)을 만들고 SHA-256을 남긴다. 벽시계 값은 해시 전에 뺐다: 행의 `wall_s`·`step_ms`·`step_ms_hist`·`scan_ms`·`scan_proc_ms`·`raster_world.lidar.ms_*`, tick의 `loop_ms`·`scan.proc_ms`, 종료 사건의 `runner.stats.*_proc_s`, trace의 `step_ms`. 단일열 행의 `run_id`도 뺐다.

| id | 내용 | 끝 | 행 SHA | trace SHA | 기록(vendor_commit 뺌) SHA |
|---|---|---|---|---|---|
| E1 | sim_in_loop exec_field, nominal_open_end, seed 0, p_fit1003 | row_distance_backstop | `3cd35dc09c1f4f79` | `d733db91f1a51046` | `1fa2ee379167bfd7` |
| E2 | sim_in_loop exec_cone, heading_40deg_offset, seed 1, p_fit1003 | guard_heading | `44e243d7119a6979` | `73cf4e089e216992` | `60126979232b359b` |
| E3 | sim_in_loop exec_cone, lidar_blackout_dropouts, seed 2, p_fit1003 | row_time_backstop | `88b671dfff61f9c6` | `8b70b7aafde33e63` | `79d52f5a43fefeb2` |
| E4 | sim_in_loop exec_field, midrow_yaw_disturbance, seed 0, field1010 | time_limit | `af0645389b14a11b` | `a609f68e0167eb89` | `e67893530b704d46` |
| E5 | sim_in_loop exec_field, nominal_open_end, seed 1, stick40r30 | row_time_backstop | `566a0eb2a7873590` | `60b0e1c3374d02c7` | `df2fc8ec973c67b6` |
| E6 | sim_in_loop plain, nominal_open_end, seed 0 (실행기 없음, trace 없음) | row_end_candidate | `9e99cf5707d5a845` | (빈 목록) | (빈 목록) |
| R1 | single_row_sim P_fit1003, nominal, 042301 c03 fwd, seed 0, 상한 없음 | TERMINAL_STOP / PASS | `53742403e307d203` | `e21d2722cc7e79df` | `e49d84606df6f1b1` |
| R2 | single_row_sim field1010, gain_left085, 042301 c03 fwd, page_ready, 30 s | OPEN (상한) | `ddf967a7cb0462bb` | `ae488956a816cf3c` | `e5663357ec4d6171` |
| R3 | single_row_sim P_fit1003, yaw_bursts, 040927 c06 fwd, 60 s | OPEN (상한) | `d4ca7d7060aa511b` | `2b8fa38f974009dc` | `a12ce125f8fc0e13` |
| R4 | single_row_sim P_fit1003, lidar_latency, 042301 c03 fwd, page_ready, 20 s | OPEN (상한) | `2fd0a6bd2081dd40` | `b057dc528cf136ae` | `e62f703b2945829d` |

- R 에피소드의 하네스 trace도 따로 해시했다(`R1 6afec7c88fbc49c7`, `R2 de48014b4564b989`, `R3 0b8f5a0114476a13`, `R4 4a6c4dc5c5b9af03`).
- 표의 SHA는 앞 16자리다. 전체 값: `runs/wf3a/p4_import/reference/baseline_run1/MANIFEST.json`.
- 회랑은 봉인 회랑(042301 c01/c04/c07, 양방향)을 쓰지 않는다. 드라이버가 봉인 회랑이면 멈춘다. `exec_cone_simlike` 변형은 546fa8c에서 이미 `validate()`가 거부하므로(geometry guard 없이 모터 출력) 쓰지 않았다.
- 결정성: 같은 트리를 두 번 돌려(`baseline_run1`, `baseline_run2`) 10개 에피소드의 모든 산출물이 같았다.

## 3. import 폐포

- 방법: `runs/wf3a/p4_import/tools/import_closure.py`. 파일을 import하지 않고 AST만 읽는다. 모듈 수준·함수 안 import, 상대 import, `importlib.import_module`, `spec_from_file_location`, `subprocess` 호출을 모은다. 결과: `runs/wf3a/p4_import/closure/closure_3entries_546fa8c.json`, `closure_tools_546fa8c.json`, `closure_tests_546fa8c.json`.

| 진입점(546fa8c) | `tools/row_executor` 안의 폐포 |
|---|---|
| `executor_main.py` | `calib_runs`, `config`, `bridge_client`, `core`, `guard`, `navigation_observation`, `operator_http`(지연: `main`), `turn_observation` → `turn_assist`, vendor `closed_loop/{controllers,estimator}`, `orchard_perception/scan_corridor`(`core.load_vendor`) |
| `sim_in_loop.py` | 위 전부 + `__init__`, vendor `closed_loop/__init__` |
| `single_row_sim.py` | 위 전부 + `sim_in_loop`, `single_row_video`(지연: `_render`), vendor `closed_loop/single_row`(동적: `core.single_row_module`의 `import_module(package + '.single_row')`) |
| 도구 `calib_analysis.py` | `calib_runs`, `config`, `guard` |
| 도구 `body_mask_static.py` | `config` |
| 도구 `body_mask_fit.py` | **`field_records.py`**(자기 폴더를 `sys.path`에 넣고 `import field_records`) |
| 도구 `write_sha256sums.py` | `executor_main`(위 폐포) |

- 동적·외부 경로:
  - `sim_in_loop.rover_control_class()`가 `spec_from_file_location`으로 대시보드의 `scripts/rover_control_core.py`를 읽는다. 저장소 밖 대시보드 사본 또는 이 저장소의 `dashboard/`(6.3절).
  - 하위 프로세스: `single_row_sim._git`(git `rev-parse`·`status`, 기록용), `single_row_video`(ffmpeg). 파이썬 진입점을 하위 프로세스로 부르는 코드는 시험 `live_ros_smoke.py`뿐이다(`python3 -m row_executor.executor_main`).
- SIM 트리 의존(복사하지 않음, 이 worktree에 이미 있다): `sim/ubts/closed_loop/{__init__,controllers,estimator,harness,matrix,world,raster_corridor,single_row,external_paths}.py`, `sim/ubts/field_response.py`, `sim/ubts/orchard_perception/{scan_corridor,scenarios}.py`, `sim/ubts/cpu_lidar.py`(`world.py`가 모듈 수준에서 import), `sim/ubts/mesh_raycast.py`(`cpu_lidar.main` 안의 지연 import, trimesh 필요, F-SIM은 쓰지 않음). 이제 `steering/s2_format.py`도 더해진다.
- 시험 폐포: `test_executor_loop.py`가 `replay_adapters.py`를 import한다(근접 반사 거르기 한 시험의 비교 기준). `replay_*`는 가져오지 않으므로 그 클래스를 시험 안에 고정 사본으로 넣었다(6.5절). `test_replay_gate.py`는 `replay_adapters`, `row_shadow_replay`를 쓰므로 가져오지 않았다.
- `row_gate_report.py`, `convert_1003.py`, `replay_*`, `row_shadow_replay.py`는 어느 진입점의 폐포에도 없다.

## 4. 가져온 파일

`git show 546fa8c:<경로>`로 복사한 바이트의 git blob id를 `git ls-tree`와 대조했다. CRLF는 없다. 기록: `runs/wf3a/p4_import/closure/copy_record.json`.

| 경로(tools/row_executor/ 아래) | 546fa8c SHA-256 | 지금 |
|---|---|---|
| `__init__.py` | `597ab949148693398560eb87e11d6fb4535a7de72b112ed42ebbaa76b566b75b` | 바꿈(문서열) |
| `bridge_client.py` | `f13592f52ca71a44bc6fdb4fad4cfe620fc63e1e615ce36b40c27c4266343eb6` | 바꿈(문자열) |
| `calib_runs.py` | `ea8e37124186e8b6578e40e8ab3fd9ad8724af786e69bfdd89d2503ec158301f` | 같음 |
| `calib_analysis.py` | `88c98d46ead978ee79c6051c0aad58cb7ccb36c12bf8b3e63655726a96d00d50` | 바꿈(문서열) |
| `config.py` | `ac7daa86789d4102c5a72b21fea3ad96db37faddc9ae0c678cab6bf692f9cc3e` | 바꿈(문서열·주석) |
| `core.py` | `a3680df945db4088a41ecb74ba4088b90d770ff1f248b59b2e640c6c1a58e07e` | 같음 |
| `executor_main.py` | `581ac9faeb83c9f4d0a9487cff7ce807975cfe33528d931a8aec9d8e141372c9` | 바꿈(turn 제거) |
| `guard.py` | `c092a10de12c8872ecaac80561edf509a81b9021a5ad349cac48b88840eb5d65` | 같음 |
| `navigation_observation.py` | `15ea3ac5cf5db92cab0ede3eb4af0400ece761ed83586f9f2ffd05eae46a56cf` | 같음 |
| `operator_http.py` | `8c6ef70a28001c5d01b940fee775ef4a8548cf78628209a1c79083b208faf6c8` | 같음 |
| `sim_in_loop.py` | `5d96e6ce2de3b01f509723ccb697ba32a46612db87f47d40c4db4de8ad8faf72` | 바꿈(경로·RealFormat 이동) |
| `single_row_sim.py` | `809b906244bd1fe71613c8fab040cc15ea54d93427a806c8b24b43fc7258f5d6` | 바꿈(경로) |
| `single_row_video.py` | `2037ad7d4dc9474a1833b7198b0375ebc55924a66d6252cd6826c6a4aeaee570` | 바꿈(경로) |
| `body_mask_static.py` | `f999ec79462b34972e802f5bc23fb7903549afbd1e64c4cf2754324e5f88d277` | 같음 |
| `body_mask_fit.py` | `9eae2c11ffcfb1bc4d5dcaf1d9ecbc949268f5725738d7d35b3dc854c6c7d2de` | 같음 |
| `field_records.py` | `d6599b99f5aa686bce4cf4ffe69b6f75c3034ac58a44f9d4edabc9bb17fba061` | 바꿈(경로) |
| `write_sha256sums.py` | `51f94d02f51284f68fd033f52264d8fd03182291408332ba975a185fa01011fa` | 같음 |
| `SHA256SUMS` | `520b03ee4ef8d631f8ed7aafaddae946f99cf10fe8a04b781ef990129c3dbe2d` | 다시 만듦(58개 파일) |
| `config.field_shadow.example.json` | `aec25cb2d6377e0d3450ebf9e58675c4304af7ab04558b931d2cedfbc51bc7fe` | 같음 |
| `config.single_row_shadow.example.json` | `753775e85f0c0f9be76c448b83f1e22b0c7263959bc03fd30df72f91b0c2df33` | 같음 |
| `vendor/SOURCE_MANIFEST.json` | `e63d3aafc49d6227bfb6dfbf19dd38ec8de655bf3cc8baebeb57aa3c75947631` | 다시 만듦 `8f8f9d5f2925f3bd…` |
| `vendor/closed_loop/__init__.py` | `a7a68f3ebb74e2d40c91e6a9c8b83e86a701cf84fc65a72cdc809cf6dfc9a5d0` | 같음 |
| `vendor/closed_loop/controllers.py` | `34570abd7a900edef39ba05dc50325e2e1e3f83f997a9e3b500b51fd217552a6` | 같음 |
| `vendor/closed_loop/estimator.py` | `39a088681f30c82dc07b8c4cd4cf667aa0a5f0468e7ec62e720de96712aaee97` | 같음 |
| `vendor/closed_loop/single_row.py` | `dfb8cf252c12b42c647f5f6484eca814232a63bcdbe5c2ee42f7cc0b620e9278` | 같음 |
| `vendor/orchard_perception/__init__.py` | `d0603fe76e558b1ade6da73b02abca703f31eaa33c63bb48c8e968f772be9def` | 같음 |
| `vendor/orchard_perception/scan_corridor.py` | `198ef42a4def368dadf72874cdc929b177f5f805b1d67c76a0d4769185a1fe5b` | 같음 |
| `README_FIELD.txt` | `df5b11223b6c149aa2d3cb370d4832e9b29e3609af2a590350099d42881cfb8b` | 바꿈(문구·머리말) |
| `OPERATIONS.txt` | `879853652d4e036a5f42362e47f1a041cd364eb6ac19bb6a31814189c592ccfe` | 바꿈(문구·머리말) |
| `SINGLE_ROW_MODE.txt` | `58e8a5acf0e1a07ecea06e7ab15b75ff1d62ea912ae1b08f4b35a0ea3908f003` | 바꿈(경로·머리말) |
| `NATIVE_NAVIGATION_KO.md` | `2c7164e8f509452b416ed587860c85fb5fc126723939d5277353c1b251ee6b22` | 같음 |
| `run_executor.sh` | `5d9a5fb8709cca1dc94da2b27a5ade0fbe0992a8e7d170cdbe76209d1c2ad21b` | 같음(546fa8c 모드 100755) |
| `tests/rowexec_testlib.py` | `49eabbd7fe5b4373929872cd3625cfc693138fa7fead9ee6db7fb6fdd99fd5f9` | 바꿈(경로) |
| `tests/page_js_harness.js` | `a6d90a9a18dc34b6adfcf7085c8d0631697238374b8b1a93d069c8ae1ba5ffff` | 같음 |
| `tests/live_ros_smoke.py` | `77b2a38d0e69f44d44c7c03f6e33fbff3138b378ba8d995cd144cd680fef0b3a` | 바꿈(문서열) |
| `tests/test_body_mask_static.py` | `f66b5f0572c55bc1406e79ff65fab77a1e9a104683df2383f048fb029454462b` | 바꿈(경로) |
| `tests/test_bridge_client.py` | `378152f42ef3dd61afc8309c17d26c567e586026788e92fcc25242e65016cca3` | 같음 |
| `tests/test_bridge_protocol_linux.py` | `f087dde949568226d23f748b3e2c3d440d0984a06ec84560746eb11544619045` | 바꿈(임시 폴더 접두사) |
| `tests/test_calib_runs.py` | `f25082f6b146653cc5270ce23fdbf6dd24cf00d921c9735bde832acbf23738e1` | 같음 |
| `tests/test_config.py` | `7bfe9d97b481f2409209572d647f3e65ca294f49b3702e6426c2fdd8e16f4c07` | 같음 |
| `tests/test_executor_loop.py` | `048d6eca7281fdc5aaac7c9492ea6ebb8378cb773e0542c6c0766b6ad01bf0f0` | 바꿈(재생 기준 사본·문서열) |
| `tests/test_field1010_closed_loop.py` | `b074c52dbc1b2b0ee76b2e88a376c07b985e761e8597e4789a02899e0b7fd575` | 같음 |
| `tests/test_guard.py` | `4bf636892fbd4432cdb611bb3c2328a78bb27a2ae884ed50623375b78e5ea987` | 같음 |
| `tests/test_imu_yaw.py` | `e46e8f71b632779ea8344b50a7f13ce524e1407082223b413b7523d736de8e1c` | 같음 |
| `tests/test_navigation_observation.py` | `5ea171727ec2f50a286794199fc475974589a8c698cbe0cfa8bf7a55908946ff` | 같음 |
| `tests/test_operator_http.py` | `76df2258a09db743d749fe929d1331125e7415993a439b6335a887aa1f1d8223` | 같음 |
| `tests/test_page_js.py` | `9af27d4db01a155bfbe19ba2456cd0011f9853577708409760c8e032e05174a6` | 같음 |
| `tests/test_review_fixes_core.py` | `b8d08c4a11a20b6f4eff292b5b7fa8672f5a89a4a2fb10d0c29f539680d32780` | 같음 |
| `tests/test_review_fixes_day2.py` | `c796654f2040529fa77eb43913ba453abe84967ed467eefaa627e2c3072654c4` | 바꿈(문서열) |
| `tests/test_run_script.py` | `575318df7490206f7915306f036bc15e42037437eb0c151e5d49f5e573ccd020` | 바꿈(셸 경로) |
| `tests/test_runner.py` | `148cf87d22f8617d293a201daa25289d4cf31c0e1502347154c104c29108e113` | 바꿈(vendor 출처 검사) |
| `tests/test_scan_adapter.py` | `f08deb2105d9220a62a8f49965671e5ffdb69958e397b9ad3e54d88d79beb093` | 같음 |
| `tests/test_shadow_startup_script.py` | `84d046f77f1a60c0128dee39efb555952ee7aa6bb8205c09790ace0c186abadd` | 같음 |
| `tests/test_single_row_mode.py` | `5f50fa0fb96eb0e564ec5d84d74cf7058a3e285c20b64ad138ec9706339c53f3` | 같음 |
| `tests/test_single_row_sim.py` | `5f91a17aefa5a6ef53abb4072d7b7f6b1992c56b75f2d9a8126b0681ba2cc4b8` | 같음 |

새 파일:

| 파일 | 내용 |
|---|---|
| `tools/row_executor/sim_paths.py` | SIM 쪽 도구의 외부 위치 조회(6.3절) |
| `tools/row_executor/tests/test_no_turn_import.py` | turn 모듈이 올라오지 않음, turn 파일 없음, `turn_observe` 거부 |
| `tools/row_executor/tests/test_sim_paths.py` | 조회 순서, 설정 파일 검사, 오류 문구 |
| `tools/row_executor/tests/test_vendor_tools.py` | 임시 사본에서 두 vendor 도구의 통과·실패·재생성 |
| `sim/ubts/steering/s2_format.py` | 실제 S2 형식과 `RealFormat`(6.4절) |
| `sim/ubts/test_steering_s2_format.py` | s2_format 시험(unittest 형식, 3.10에서도 돈다) |
| `tools/steer_sim/regen_vendor.py`, `tools/steer_sim/check_vendor_sync.py` | vendor 도구(7절) |

- 설정 예시 두 개의 설정 digest는 기준선과 같다(`ExecutorConfig()`와 `config.field_shadow.example.json` `700ee41c0188dffa…`, `config.single_row_shadow.example.json` `594143cab2ba608b…`).

## 5. 가져오지 않은 파일과 이유

| 파일 | 이유 |
|---|---|
| `turn_observation.py`, `turn_assist.py` | 머리 회전(유턴) 관찰·보조. 범위 밖. `executor_main`에서 import를 지웠다(6.1절) |
| `turn_bridge.py` | 회전용 서명 브리지 클라이언트. 회전 전용 |
| `turn_real_replay.py`, `turn_sim_in_loop.py` | 회전 재생·SIM 도구 |
| `TURN_ASSIST.txt` | 회전 문서 |
| `replay_adapters.py`, `replay_executor_shadow.py`, `row_shadow_replay.py` | 10/10 기록 재생 도구. 폐포에 없다. `test_executor_loop`의 비교 기준 하나는 고정 사본으로 바꿨다 |
| `convert_1003.py` | 10/03 기록 변환(이 PC의 자료 경로가 박혀 있다). 폐포에 없다 |
| `row_gate_report.py` | 재생 관문 보고. 폐포에 없다 |
| `tests/test_replay_gate.py` | 재생 도구 시험 |
| `tests/test_turn_assist.py`, `test_turn_bridge.py`, `test_turn_bridge_protocol_linux.py`, `test_turn_config.py`, `test_turn_guard.py`, `test_turn_observation_integration.py`, `tests/turn_testlib.py` | 회전 시험 |
| `deploy/field10_c0_shadow_v3` | `tools/row_executor` 밖, 옛 실행기의 Jetson 배치 |
| `tools/field_calib` | p4_ident가 가져온다 |

- (2026-11-12, PC-04) 재생 도구 `replay_adapters.py`, `replay_executor_shadow.py`, `row_shadow_replay.py`, `row_gate_report.py`와 `tests/test_replay_gate.py`는 뒤에 가져왔다. 바꾼 곳과 10/22 기준선 재현 결과는 `REPLAY_BASELINE_KO.md`에 있다. `convert_1003.py`와 회전 도구는 여전히 없다.
- `config.py`의 회전 설정 키(`turn_assist_enabled`, `turn_*`, `turn_feasibility`)와 `guard.TurnGuard`는 그대로 두었다. 지우면 설정 스키마와 digest가 바뀐다. `turn_assist_enabled`는 기본 false이고 단일열 모드는 이 키를 거부한다.

## 6. 바꾼 것

### 6.1 turn 의존 제거 (`executor_main.py`)

| 곳 | 옛 동작 | 새 동작 |
|---|---|---|
| import | `from .turn_observation import TurnObservationRunner` | 지움 |
| `__init__` | `self._turn_end_logged` | 지움 |
| `reset_runner`, `_start`, `_publish`(tick 기록과 상태), `_run_end_event` | `isinstance(self.runner, TurnObservationRunner)` 분기 | 지움. 회전 runner가 없으니 늘 거짓이던 분기다 |
| `_observation_end_event`와 호출 3곳(`_req_end`, `_req_params`, `shutdown`) | 회전 관찰 종료 사건 | 지움. 회전 runner가 아니면 아무것도 하지 않던 함수다 |
| `_req_turn_observe` | 수동 SHADOW·field 프로필에서 회전 관찰 시작 | 늘 거부. 단일열 모드는 전과 같이 `turn_observe_not_in_single_row_mode`, 그 밖은 `turn_observe_not_available` |
| 상태 `turn_observe_available` | 수동 SHADOW·field·단일열 아님이면 참 | 늘 거짓. 운용자 화면의 회전 카드는 숨겨진다 |

- `operator_http.py`의 `/api/turn_observe` 경로와 화면 카드는 그대로 두었다(바꾸면 화면 시험이 바뀐다). 요청은 위 거부로 끝난다.
- JSONL 기록은 바뀌지 않는다. 회전 칸은 회전 runner일 때만 썼다.
- 시험: `test_no_turn_import.py`(새 인터프리터에서 세 진입점 import 뒤 `turn_*` 모듈 0개, 패키지 소스에 `turn_*` import 0개, 회전 파일 없음, 세 모드 거부). `test_single_row_mode.py`의 기존 거부 시험은 바꾸지 않고 통과한다.

### 6.2 보조 도구 이름 문자열 → 중립 문구

옛 문구의 도구 이름은 이 문서에도 쓰지 않는다. 아래에서는 `<도구>`로 적는다.

| 파일 | 옛 문구 | 새 문구 | 종류 |
|---|---|---|---|
| `bridge_client.py` 62행 | `(<도구>.md 2026-10-13 table)` | `(workspace guide, 2026-10-13 table)` | 실행 중 안내 문자열(`RECOVERY_PROCEDURE`) |
| `calib_analysis.py` 39행 | `(branch <도구>/field-calib-20261022)` | `(field calibration branch of 2026-10-22)` | 문서열 |
| `config.py` 67행 | `(<도구>-field-autonomy-20261022/gate/C1_GATE_REPORT.txt section 5;` | `(field autonomy work folder of 2026-10-22, gate/C1_GATE_REPORT.txt section 5;` | 문서열 |
| `config.py` 1291–1292행 | `steering GO rule <도구>-field-autonomy-20261022/` `steer_rule/RULE_KO.txt (continuous_side,` | `steering GO rule (field autonomy work folder of` `2026-10-22, steer_rule/RULE_KO.txt; continuous_side,` | 주석 |
| `tests/test_bridge_protocol_linux.py` 95행 | `mkdtemp(prefix='ubts-<도구>-field-')` | `mkdtemp(prefix='ubts-rowexec-')` | 임시 폴더 이름 |
| `tests/live_ros_smoke.py` 3행 | `--name ubts-<도구>-field-rossmoke` | `--name ubts-rowexec-rossmoke` | 문서열 |
| `tests/test_executor_loop.py` 6행 | `(<도구>-field-autonomy-20261022/review_*_day1)` | `(field autonomy work folder of 2026-10-22, review_*_day1)` | 문서열 |
| `tests/test_review_fixes_day2.py` 1행 | `(<도구>-field-autonomy-20261022/review_safety_day2 N1-N9,` | `(field autonomy work folder of 2026-10-22, review_safety_day2 N1-N9,` | 문서열 |
| `OPERATIONS.txt` | 브랜치 `<도구>/sim-cross-alley-20261019`, `<도구>.md 10/13 표`, 컨테이너 `ubts-<도구>-field-<x>`, `/tmp/ubts-<도구>-field-robind`(2곳), `<도구>-field-autonomy-20261022/fixb_day1/…`, 브랜치 `<도구>/field-calib-20261022`, `<도구>/field-integrated-20261022` | `2026-10-19 cross-alley SIM 브랜치 커밋 …`, `작업 루트 지침 문서 10/13 표`, `ubts-rowexec-<x>`, `/tmp/ubts-rowexec-robind`, `2026-10-22 현장 자율주행 작업 폴더의 fixb_day1/…`, `2026-10-22 현장 보정 브랜치`, `2026-10-22 통합 브랜치` | 문서 |
| `README_FIELD.txt` | `<도구 대문자>_DAY3_STATUS_20261023.md`, `<도구>-field-autonomy-20261022/{field_sheets,review_safety_day1,gate,steer_rule}/…`, 브랜치 `<도구>/field-integrated-20261022` | `DAY3 상태 문서(2026-10-23)`, `2026-10-22 현장 자율주행 작업 폴더의 …`, `통합 브랜치` | 문서 |

- 실행 중 문자열은 `RECOVERY_PROCEDURE`(브리지 고장 안내)와 임시 폴더 접두사 둘이다. 둘 다 결과 trace·기록에 들어가지 않는다(8절에서 확인).
- 문서 교체는 저장소 밖 `runs/wf3a/p4_import/tools/clean_docs.py`가 정확한 문자열로 했다(각 문구가 정해진 횟수만 맞아야 한다). 기록: `runs/wf3a/p4_import/closure/doc_clean_record.json`(도구 이름은 가려 적음).

### 6.3 절대 경로 → 설정값

| 파일 | 옛 코드 | 새 코드 |
|---|---|---|
| `sim_in_loop.py` | `DASH_CANDIDATES`(대시보드 사본의 Windows·WSL 절대 경로, `/dash`) | `rover_control_class()`가 `sim_paths` `dashboard_dir`를 쓴다. `scripts/rover_control_core.py`가 없으면 `RuntimeError` |
| `sim_in_loop.py` | `HOLD_FILE = <작업 트리의 부모 폴더>/HOLD_MEASUREMENT` | `hold_file()` = `closed_loop.external_paths.hold_file()`(키 `hold_dir`). 설정이 없으면 `main()`이 아무것도 돌리지 않고 오류로 멈춘다(P2의 `single_row_eval`과 같은 규칙) |
| `sim_in_loop.py` 문서열 | 이 PC의 HOLD 파일 경로 | "설정된 hold 폴더의 HOLD_MEASUREMENT" |
| `single_row_sim.py` | `PREREG_CANDIDATES`(PREREG.json의 Windows·WSL 절대 경로) | `prereg_path()`가 `sim_paths` `single_row_prereg`를 쓴다. 내장 경로 없음 |
| `single_row_sim.py` | `hold_present()`가 `SIL.HOLD_FILE`을 봤다 | `SIL.hold_present()`. `main()` 첫머리에서 `SIL.hold_file()`로 설정을 먼저 확인한다 |
| `single_row_sim.py` 문서열 | HOLD 경로, 사용 예 `field1010:nominal:042301:c04:rev:0` | hold 폴더 설명, 예를 `042301:c03:rev`로 바꿨다(c04 rev는 봉인 열 쌍의 역방향이라 이 브랜치에서 돌리면 안 된다). 봉인 회랑 경고를 더했다 |
| `single_row_video.py` | `FFMPEG_CANDIDATES`(이 PC의 ffmpeg 절대 경로) 다음 PATH | 명시 경로가 파일이면 그것, 아니면 `sim_paths` `ffmpeg`(PATH의 ffmpeg가 기본값). `shutil` import는 쓰지 않게 되어 지웠다 |
| `single_row_video.py` 문서열 | `--out` 예의 절대 경로 | `<runs folder outside the repository>` |
| `field_records.py` | `BASE_1010 = UBTS_RD_1010 또는 이 PC의 10/10 자료 폴더` | `BASE_1010 = sim_paths.optional_path('field_manual_1010')`(설정이 없으면 None). drive 추출 파일 폴더는 따로 `DRIVE_1010 = sim_paths.optional_path('field_manual_1010_drive')`다(저장소는 그 자료 폴더 이름을 적지 않는다: A16). `load_record`가 None이면 분명한 `FileNotFoundError`를 낸다 |
| `config.py` 문서열 | 목표 문서의 절대 경로 | `goal document SINGLE_ROW_GOAL_20261102.md` |
| `tests/rowexec_testlib.py` | `DASH_CANDIDATES` | `dash_dir()`가 `sim_paths` `dashboard_dir`를 쓴다 |
| `tests/test_body_mask_static.py` | `RD_CANDIDATES`(환경 변수, 절대 경로 2개, `/rd`) | `UBTS_RD_1010_LIDAR` 또는 `<field_manual_1010>/lidar`. 없으면 이유를 적고 건너뛴다 |
| `tests/test_run_script.py` | Windows Git bash의 절대 경로 | `sim_paths` `posix_shell`(PATH의 sh·bash가 기본값, Windows System32의 WSL 실행기는 뺀다). 없으면 건너뛴다 |
| `OPERATIONS.txt`, `SINGLE_ROW_MODE.txt` | 대시보드 사본·목표 문서의 절대 경로 | "대시보드 폴더", "작업 루트의 SINGLE_ROW_GOAL_20261102.md" |

`sim_paths.py` 조회 순서:

1. 항목의 환경 변수(비어 있지 않을 때).
2. `UBTS_ROW_EXECUTOR_PATHS`가 가리키는 JSON 파일. 그 변수가 없으면 `UBTS_STEER_PATHS` 파일과 같은 폴더의 `row_executor_paths.json`(있을 때).
3. 기본값이 있는 항목만 기본값. 이 PC 경로는 기본값이 아니다.
4. 없으면 `sim_paths.PathNotConfigured`(`FileNotFoundError` 하위 클래스). 메시지에 환경 변수, 키, 이 문서 이름이 들어 있다. 값이 있는데 그 경로가 없으면 값의 출처를 적은 `FileNotFoundError`다.

| 키 | 환경 변수 | 기본값 | 쓰는 곳 |
|---|---|---|---|
| `dashboard_dir` | `UBTS_DASH_DIR`(옛 코드에도 있던 이름) | 이 저장소의 `dashboard/` | `sim_in_loop.rover_control_class`, `tests/rowexec_testlib.dash_dir` |
| `single_row_prereg` | `UBTS_SINGLE_ROW_PREREG`(옛 이름) | 없음 | `single_row_sim.prereg_path` |
| `field_manual_1010` | `UBTS_RD_1010`(옛 이름) | 없음 | `field_records.BASE_1010`, `body_mask_fit`, `tests/test_body_mask_static` |
| `field_manual_1010_drive` | `UBTS_RD_1010_DRIVE` | 없음 | `field_records.DRIVE_1010`(drive 추출 파일 `<rid>.drive.json.gz`). `tools/field_calib/records.py`의 `open_compact`는 같은 환경 변수나 `drive_dir` 인자를 쓴다 |
| `ffmpeg` | `UBTS_FFMPEG`(새) | PATH의 ffmpeg | `single_row_video.ffmpeg_path` |
| `posix_shell` | `UBTS_POSIX_SHELL`(새) | PATH의 sh·bash(System32 실행기 제외) | `tests/test_run_script.preferred_shell` |

- 설정 파일 형식은 `external_paths`와 같다(`schema` `ubts.row_executor.sim_paths.v1`, 값은 경로 문자열 또는 `{"nt": …, "posix": …}`, 상대 경로는 파일 폴더 기준, 모르는 키·다른 schema·틀린 형식은 오류).
- 새 키를 `closed_loop/external_paths.py`에 넣지 않은 이유: 그 파일은 이 작업의 소유가 아니고, 모르는 키를 거부하므로 `paths.json`에 키를 더하면 그 파일을 쓰는 모든 코드가 멈춘다. 그래서 옆 파일을 따로 둔다.
- 이 PC의 설정: `D:/Develop/UBTS/steering-sim-20261108/config/row_executor_paths.json`(저장소 밖). `single_row_prereg`, `field_manual_1010`(nt·posix), `field_manual_1010_drive`(nt·posix), `ffmpeg`·`posix_shell`(nt)을 적었다. `dashboard_dir`는 적지 않아 저장소의 `dashboard/`를 쓴다. `UBTS_STEER_PATHS=D:/Develop/UBTS/steering-sim-20261108/config/paths.json`만 주면 옆 파일이 함께 읽힌다.
- 대시보드 기본값을 실차 사본에서 이 저장소의 `dashboard/`로 바꿨다. 두 `rover_control_core.py`의 SHA-256이 같다(2절). 설계 2.4대로 SIM 저장소 사본이 원본이고 실차 사본과는 A26으로 비교한다. 실차 사본을 쓰려면 `UBTS_DASH_DIR`를 준다. `single_row_sim`의 `RUN_MANIFEST.json`에는 읽은 파일의 SHA가 전과 같이 남는다.
- 남긴 절대 경로: Jetson·컨테이너 실행 경로(`/run/ubts-rover/control.sock`, `/opt/row_executor`, `/etc/row_executor`, `/var/log/row_executor`, Jetson 홈 아래 녹화 폴더)는 실차 규약이라 그대로다. vendor `closed_loop/single_row.py` 3행 문서열의 목표 문서 절대 경로는 SIM 트리 `sim/ubts/closed_loop/single_row.py`와 바이트가 같아야 해서 고치지 않았다(11절).

### 6.4 `s2_format.py`

- 위치: `sim/ubts/steering/s2_format.py`(DECISIONS R16. 설계 2.3 표의 `tools/row_executor/s2_format.py`를 대신한다).
- 옮긴 것: `RealFormat` 클래스와 그 상수 `RAW_AMIN`, `RAW_INC`, `BEAMS`, `SIM_BEAMS`. 546fa8c `sim_in_loop.py`와 AST가 같다(문서열 포함). `sim_in_loop`은 `from steering.s2_format import BEAMS, RAW_AMIN, RAW_INC, SIM_BEAMS, RealFormat`으로 같은 이름을 다시 내보낸다. 그래서 `S.RealFormat`, `SIL.RealFormat`을 쓰는 코드와 시험은 바뀌지 않는다.
- 더한 상수(실제 형식의 사실, 변환에는 쓰지 않음): `FRAME_ID = 'laser'`, `SCAN_HZ = 10.`, `RANGE_MIN_M = .05`, `RANGE_MAX_M = 30.`, `DEFAULT_YAW_OFFSET_DEG = 179.08`. G-SIM S2 노드가 쓸 값이다.
- import: `math`, `numpy`뿐이다. 하네스·추정기·scipy를 import하지 않는다(시험이 새 인터프리터에서 확인한다). Python 3.10에서 돈다(WSL에서 확인).
- 약속(시험으로 고정):
  - 3240빔, `angle_min` = float32(−π), 증분 약 0.1111°(3240빔이 약 360.11°를 덮는다. `ScanAdapter`는 자르지 않는다).
  - 원 순서: 배열을 다시 늘어놓지 않는다. 원 빔 j의 차체 각 = wrap(`angle_min` + j·증분 + `lidar_yaw_offset_deg`). 실행기 `ScanAdapter`가 `angle_min`에 같은 오프셋을 더해 되돌린다. 오프셋 179.08°면 차체 정면은 원 빔 8, 0°면 1619, +90°면 810, −90°면 2429다.
  - 각 원 빔은 차체 각에 가장 가까운 SIM 빔(3200빔, −π부터)을 받는다. SIM 빔마다 원 빔 1~2개가 쓴다.
  - 0 = 반사 없음: NaN, +inf, −inf, 30 m 이상은 0.0.
  - 반올림: 유한하고 30 m 미만이면 `np.round(r·1000)/1000`(짝수 쪽 반올림, 예 1.0625 → 1.062, 1.1875 → 1.188). 30 m 검사가 반올림보다 먼저라 29.9996 m는 30.0이 된다. 음수와 0은 그대로 나간다(range_min으로 자르지 않는다). 출력은 float32.
  - SIM 빔이 3200개보다 적으면 `IndexError`, 많으면 앞 3200개만 쓴다.
- 확인: 546fa8c `RealFormat`과 새 모듈에 오프셋 7개 × 무작위·경계 스캔 280개를 넣어 출력 바이트가 모두 같았다(`runs/wf3a/p4_import/reference/realformat_compare.json`).

### 6.5 시험 조정

| 파일 | 바꾼 것과 이유 |
|---|---|
| `tests/test_executor_loop.py` | `replay_adapters` import를 지우고 그 `NearTransientFilter`를 시험 안 고정 사본 `ReplayNearTransientFilter`로 넣었다. 메서드 AST가 546fa8c 원본과 같다(문서열만 다름). 근접 반사 거르기 비교 시험은 그대로 돈다 |
| `tests/test_runner.py` | vendor 시험이 `source_commit == c35126025d9e…`와 이전 커밋을 요구했다. 이 브랜치는 vendor를 SIM 트리에서 다시 만든다(7절). 그래서 `source_commit`이 `WORKTREE-UNCOMMITTED` 또는 40자리 커밋 id인지, 원래 6개 파일이 있는지, vendor 파일마다 SIM 트리 원본과 바이트가 같은지를 본다 |
| `tests/rowexec_testlib.py`, `tests/test_body_mask_static.py`, `tests/test_run_script.py` | 경로(6.3절) |
| `tests/test_bridge_protocol_linux.py`, `tests/live_ros_smoke.py`, `tests/test_review_fixes_day2.py` | 문자열(6.2절) |

- 회전만 시험하던 파일은 가져오지 않았다(5절). 지운 시험 함수는 없다.

### 6.6 문서

- `OPERATIONS.txt`, `README_FIELD.txt`, `SINGLE_ROW_MODE.txt` 맨 위(제목 줄 다음)에 머리말 한 줄을 넣었다: 546fa8c에서 가져온 사본이고 회전·재생 도구는 없다는 것과 이 문서 이름.
- `README_FIELD.txt`의 회전 관찰 카드 줄 아래에 "조향 브랜치 사본에서는 이 요청을 모든 모드에서 거부한다"를 더했다. 나머지 회전 설명은 이력으로 남겼다.
- `NATIVE_NAVIGATION_KO.md`, `run_executor.sh`는 고칠 것이 없어 바이트 그대로다.
- `__init__.py` 문서열에 `sim_paths.py`와 조향 브랜치 사본이라는 줄을 더했다.

## 7. vendor 절차와 도구

### 7.1 다시 만들기 전 비교

- 이 브랜치의 SIM 트리 파일은 546fa8c에서 왔고, vendor 사본은 c35126025d9e…(546fa8c manifest의 `source_commit`)에서 왔다.
- 6개 vendor 파일을 다섯 곳에서 비교했다: 546fa8c vendor, 546fa8c SIM 트리, c3512602 SIM 트리, 이 worktree, HEAD. 6개 모두 다섯 곳에서 SHA-256이 같다. 차이 없음.
- 같은 시점 `check_vendor_sync.py`는 vendor 쪽 문제 0(manifest, SIM 트리, `git show c3512602` 모두 같음)이고 `SHA256SUMS` 쪽 문제만 냈다(가져오지 않은 파일과 고친 파일 때문). 기록: `runs/wf3a/p4_import/vendor/check_before_regen.json`.

### 7.2 작업 트리 기준 재생성

- `python -I tools/steer_sim/regen_vendor.py --worktree`: 쓴 파일 0개(바이트가 이미 같다), manifest 다시 씀, `SHA256SUMS` 58개 파일, `core.verify_vendor()`와 `executor_main.verify_sources()` 통과. 두 번째 실행은 manifest를 바꾸지 않는다.
- 새 manifest: `source_commit` `WORKTREE-UNCOMMITTED`, `source_branch` `steering-sim-20261108`, 파일은 `vendored_path` 순, 키 정렬, LF. `git_blob`는 그 바이트의 blob id다(예 `closed_loop/__init__.py` `3ec6b943…`, 546fa8c와 같다). `derivation`은 바꾼 manifest를 가리킨다: `previous_source_commit` c35126025d9e…, `previous_manifest_SHA256` `e63d3aafc49d6227…`, `previous_source_branch` single-row-20261102. 작업 트리 기준 manifest를 다시 만들 때는 이 derivation을 그대로 둔다(늘 마지막 커밋 기준 vendor를 가리킨다).
- `--commit` 확인: `--commit HEAD`(0c1311e2…)로 한 번 만들어 `check_vendor_sync.py`의 `git show` 비교까지 통과한 뒤, 작업 트리 기준 manifest로 되돌렸다. 되돌린 뒤 `SHA256SUMS`가 바이트까지 같았다. 기록: `runs/wf3a/p4_import/vendor/regen_commit_HEAD*.json`, `check_after_commit_HEAD.json`.

### 7.3 `tools/steer_sim/regen_vendor.py`

```
python -I tools/steer_sim/regen_vendor.py --worktree              # 작업 트리 바이트, source_commit WORKTREE-UNCOMMITTED
python -I tools/steer_sim/regen_vendor.py --commit <sha>          # git show <sha>:<source_path>, git_blob 기록
  --add <source_path> <vendored_path>   (여러 번, 예 sim/ubts/closed_loop/steer_control.py closed_loop_copy/steer_control.py)
  --remove <vendored_path>              (여러 번)
  --dry-run, --json
```

1. 지금 manifest의 목록에 `--add`/`--remove`를 적용한다. 원본은 `sim/ubts/` 아래 경로만 받는다.
2. vendor 폴더에 목록에 없는 파일이 있으면 아무것도 쓰기 전에 멈춘다.
3. 모든 원본을 먼저 읽고(커밋이면 blob id까지 검사), 바뀐 파일만 바이트 그대로 쓴다(줄바꿈 변환 없음).
4. `vendor/SOURCE_MANIFEST.json`을 다시 쓴다(`modified: false`는 `core.verify_vendor`가 요구한다).
5. `write_sha256sums.py`로 `SHA256SUMS`를 다시 쓴다.
6. 새 인터프리터에서 `core.verify_vendor()`와 `executor_main.verify_sources()`를 실제로 돌린다. 모두 통과해야 종료 코드 0.

### 7.4 `tools/steer_sim/check_vendor_sync.py`

```
python -I tools/steer_sim/check_vendor_sync.py [--json] [--no-git]
```

읽기만 한다. 다음 중 하나라도 있으면 종료 코드 1과 목록을 낸다.

- manifest를 읽을 수 없거나 `modified: false`가 아니거나 파일 목록이 비었다.
- vendor 파일이 없거나 바이트 수·sha256·git_blob가 manifest와 다르다.
- vendor 파일이 SIM 트리 원본(`source_path`)과 다르다.
- vendor 폴더에 목록에 없는 파일이 있다.
- manifest가 커밋을 가리키면 `git show <commit>:<source_path>`와 다르다(`--no-git`이나 git이 없으면 건너뛰고 적는다).
- `SHA256SUMS`에 없는 파일, 남는 파일, 다른 파일이 있다. 즉 `executor_main.verify_sources()`가 실패할 상태다. 마지막으로 두 함수를 새 인터프리터에서 실제로 돌린다.

시험 `tests/test_vendor_tools.py`(7개)가 시스템 임시 폴더의 사본에서 통과·실패 경우와 재생성을 확인한다(작업 트리에는 쓰지 않는다).

### 7.5 통합 담당 절차

1. SIM 트리 변경(예 `closed_loop/*.py`, P8의 `steer_control.py`)을 먼저 커밋한다.
2. `python -I tools/steer_sim/regen_vendor.py --commit <그 커밋>`(새 파일은 `--add`).
3. `python -I tools/steer_sim/check_vendor_sync.py`가 OK인지 본다.
4. vendor와 `SHA256SUMS` 변경을 따로 커밋한다. push 전 검사(`safe_push.py`)에 `check_vendor_sync.py`를 넣는다(설계 8.5 5항).
- 작업 흐름 안에서는 `--worktree`로 만든다(DECISIONS R16). `tools/row_executor` 아래 파일을 하나라도 고치면 `write_sha256sums.py`나 `regen_vendor.py --worktree`를 다시 돌려야 `verify_sources()`가 통과한다.

## 8. 비트 동일 결과

### 8.1 비교한 것

- 같은 PC, 같은 Python 3.13.2, numpy 2.2.6, scipy 1.18.0. 기준선(`baseline_546fa8c_exec/src`)과 가져온 트리(이 worktree의 `tools/row_executor` + `sim/ubts`, `UBTS_STEER_PATHS` 설정)에서 2.2절 에피소드 10개를 같은 드라이버로 돌렸다.
- 에피소드마다 행, trace(sim_in_loop은 하네스 trace, single_row_sim은 trace 파일), 하네스 trace(R), 실행기 JSONL 기록 전체, `vendor_commit`을 뺀 JSONL 기록의 SHA-256을 비교했다.
- 가져온 트리를 두 번 돌렸다(`imported_run1`: 시험 조정 전, `imported_run2`: 모든 변경 뒤). 두 번의 산출물이 모두 같다.

### 8.2 결과

| 산출물 | 같은 수 |
|---|---|
| 행 | 10 / 10 |
| trace | 10 / 10 |
| 하네스 trace(R) | 4 / 4 |
| JSONL 기록(`vendor_commit` 뺌) | 10 / 10 |
| JSONL 기록 전체 | 1 / 10 (E6는 실행기가 없어 기록이 빈 목록) |

기록: `runs/wf3a/p4_import/reference/BIT_IDENTITY.json`, `log_diff_baseline_vs_imported.json`.

### 8.3 다른 칸

| 사건 | 칸 | 기준선 | 가져온 트리 | 에피소드 |
|---|---|---|---|---|
| `config` | `vendor_commit` | `c35126025d9ea1d4c939312b64e09c57e90a70b6` | `WORKTREE-UNCOMMITTED` | E1–E5, R1–R4 (각 1줄) |
| `shutdown` | `runner.vendor_commit` | 위와 같음 | 위와 같음 | R1–R4 (각 1줄, single_row_sim이 에피소드 끝에 `shutdown`을 부른다) |

- 이유: `vendor_commit`은 vendor manifest의 `source_commit`을 그대로 적는 칸이다(`core.verify_vendor` → `estimator.VENDOR_COMMIT`). vendor 파일 6개의 바이트는 같다. 출처 표기만 바뀌었다.
- 통합 담당이 `--commit`으로 다시 만들면 그 커밋 id가 된다. 기준선 값과는 앞으로도 다르다. 고칠 대상이 아니라 출처 기록이다.
- 그 밖에 다른 칸은 없다. 대시보드 경로(`rover_control_core.py`)는 바뀌었지만 SHA가 같고 해시한 산출물에 들어가지 않는다.
- 이 비교는 같은 플랫폼 안의 비교다(A8과 같은 범위).

## 9. Python 버전별 시험

| 환경 | 실행 | 결과 |
|---|---|---|
| Windows 11, Python 3.13.2, numpy 2.2.6, pytest 9.1.1 | `tools/row_executor/tests`의 23개 파일, 파일마다 pytest 하나. `sim/ubts/test_steering_s2_format.py` | 23개 모두 종료 코드 0: 391 통과, 13 건너뜀, 실패 0. 기존 20개 파일의 통과·건너뜀 수는 기준선과 파일마다 같다. 새 3개 파일 25 통과. s2_format 15 통과 |
| 기반 이미지 `6209168a3507`, Python 3.12.3, numpy 1.26.4, pytest 7.4.4 | 같은 파일. 컨테이너 `ubts-steer-p4_import-1`(`--rm`, 2 CPU, 2 GiB, `--network none`, 예산 도구로 2 GiB 확보·반납), 작업 트리와 자료는 읽기 전용, 출력은 runs 폴더 | 23개 모두 0: 391 통과, 13 건너뜀(브리지 RO bind 1, node 없음 12). Linux 전용 실제 브리지 시험 11개가 돌았다. s2_format 15 통과 |
| WSL Ubuntu-22.04, Python 3.10.12, numpy 2.2.6, pytest 없음 | import·함수 점검(`runs/wf3a/p4_import/tools/wsl_smoke.py`), 23개 파일 unittest(파일마다 하나), s2_format unittest | 아래 |

WSL 3.10 결과:

- import 통과: `executor_main`, `core`, `config`, `bridge_client`, `guard`, `sim_paths`, `navigation_observation`, `calib_runs`, `operator_http`, `calib_analysis`, `body_mask_static`, `steering.s2_format`.
- 함수 점검 통과: `RealFormat.convert`(3240빔, float32), `core.verify_vendor()`와 `executor_main.verify_sources()`(Windows와 같은 sums digest), `ExecutorConfig().digest()`(기준선과 같은 `700ee41c…`), `ScanAdapter.convert`, `turn_observe` 거부, `turn_*` 모듈 0개.
- unittest 통과(13개 파일과 s2_format): body_mask_static 11(실제 10/10 정지 창 포함), bridge_client 17, bridge_protocol_linux 12(1 건너뜀, 실제 Linux 브리지), config 23, guard 9, imu_yaw 4, no_turn_import 6(2 건너뜀: scipy), page_js 12(모두 건너뜀: node 없음), run_script 11, scan_adapter 10, shadow_startup_script 8, sim_paths 12, vendor_tools 7, s2_format 15.
- 실패(10개 파일): calib_runs, executor_loop, field1010_closed_loop, navigation_observation, operator_http, review_fixes_core, review_fixes_day2, runner, single_row_mode, single_row_sim. 오류마다 scipy 이진 호환 메시지(`_ARRAY_API not found`, `numpy.dtype size changed`)가 있고 단언 실패(FAIL)는 0이다. 이 환경의 scipy는 numpy 1.x용으로 빌드되어 numpy 2.2.6에서 import되지 않는다(DECISIONS R10). vendor 추정기, `NearConfirmFilter`, SIM 트리 추정기가 scipy `cKDTree`를 쓴다. 같은 파일은 3.12·3.13에서 통과했다.
- rclpy는 이 환경에 없다. `RosInput`과 `live_ros_smoke.py`는 3.10에서 돌릴 수 없다.

## 10. 이 트리에서 돌리는 방법

PowerShell, 작업 트리 루트에서:

```powershell
$env:UBTS_STEER_PATHS = 'D:/Develop/UBTS/steering-sim-20261108/config/paths.json'   # 지도·카탈로그·hold + 옆의 row_executor_paths.json
python -B tools/row_executor/sim_in_loop.py --out <runs>/sil.json --seeds 0 --variants exec_field `
    --scenarios nominal_open_end --workers 2
python -B tools/row_executor/single_row_sim.py --out-dir <runs>/sr_smoke --run-kind smoke `
    --episode P_fit1003:nominal:042301:c03:fwd:0 --workers 1
```

- `sim_in_loop`은 시작할 때 hold 폴더를 본다(`hold_dir`가 없으면 오류로 멈춘다). `HOLD_MEASUREMENT`가 있으면 기다린다.
- `single_row_sim`은 PREREG.json(`single_row_prereg`), 카탈로그와 10/10 지도(`external_paths`)가 필요하다. 출력 폴더는 저장소 밖에 둔다. 봉인 회랑(042301 c01/c04/c07, 양방향)은 쓰지 않는다.
- WSL: `export UBTS_STEER_PATHS=/mnt/d/Develop/UBTS/steering-sim-20261108/config/paths.json`. 단 3.10은 scipy 때문에 이 두 도구를 돌릴 수 없다.
- Docker: 자료와 설정을 컨테이너 안 같은 `/mnt/d/...` 경로에 읽기 전용으로 붙이고 `-e UBTS_STEER_PATHS=...`를 준다.
- 실행기 시험은 파일마다 따로 돈다(2.1절): `cd tools/row_executor/tests; python -B -m pytest -q -p no:cacheprovider --basetemp=<runs>/pt <파일 하나>`.

## 11. 시험하지 않은 것과 남은 일

- (p4_import 때 기록, 13절에서 풀림) `python -I tools/steer_sim/check_pins.py`는 FAIL이었다. 고정 파일 쪽은 통과한다(423개 중 바뀐 것 7, 모두 허용 목록, 예상 밖 0). 실패 이유는 이름 검사 두 줄뿐이었다: `tools/row_executor/vendor/closed_loop/__init__.py`가 `closed_loop`, `vendor/orchard_perception/__init__.py`가 `orchard_perception`을 가린다고 본다. 이 둘은 SIM 꾸러미의 바이트 사본이고 `core.load_vendor()`가 vendor 폴더를 `sys.path` 맨 앞에 넣을 때만 import된다. 지울 수 없다: manifest 목록에 있고, `__init__.py`가 없으면 이름 공간 꾸러미가 되어 `load_vendor`의 두 번째 호출이 `__file__`이 없는 모듈로 거부한다. `check_pins.py`는 이 작업의 소유가 아니다. 통합 담당이 vendor 폴더를 이름 검사에서 빼는 규칙을 정해야 한다.
- `run_executor.sh`는 546fa8c에서 모드 100755였다. Windows 작업 트리에서는 새 파일이 100644로 올라갈 수 있다. 커밋할 때 실행 비트를 지정해야 한다(`git add --chmod=+x`).
- SIM 트리 `sim/ubts/closed_loop/single_row.py` 3행 문서열에 목표 문서의 절대 경로가 있다(vendor 사본도 같다). P2에서 가져온 파일이고 이 작업의 소유가 아니다. 고치면 vendor를 다시 만들어야 한다.
- 시험하지 않은 것: `live_ros_smoke.py`(ROS Iron 컨테이너 필요), 운용자 화면 JS 시험의 3.12·3.10 실행(node 없음), `ReadOnlyBindTest`(같은 폴더를 RW·RO로 두 번 붙인 컨테이너 필요), `single_row_video`의 실제 영상 생성(ffmpeg 실행), `sim_in_loop`·`single_row_sim`의 `main()`과 작업자 풀 경로(에피소드는 `run_job`을 직접 불렀다), `body_mask_fit.py`의 실제 실행, Jetson 위 실행.
- `sim/ubts/__pycache__`, `sim/ubts/steering/__pycache__`가 작업 트리에 있다. 11:37:52에 생긴 `fast_sim`, `steered_plant`, `field_response` 등의 바이트코드다. 이 작업은 그 모듈들을 import하지 않았고 늘 `-B`로 돌렸다. 다른 작업 흐름이 만든 것으로 보이며 지우지 않았다.
- 다음 작업(p4_exec, p4_ident)에 넘기는 것: `Decision`을 throttle·steer_mrad로 바꾸기, `SteerOutputGuard`, GO 규칙·회전 중심, BridgeClient 조향 필드, `sim_in_loop_steer.py`, 식별 도구. SIM 트리 `closed_loop`을 바꾸거나 `steer_control.py`를 vendor에 넣으면 7.5절 절차를 따른다.

## 12. p4_exec 뒤 (2026-11-09)

- p4_exec가 실행기를 앞바퀴 조향으로 넓혔다(`EXECUTOR_STEER_KO.md`). 4절 표의 "지금" 칸은 p4_import 때의 값이다. p4_exec가 고친 파일: `config.py`, `core.py`, `guard.py`, `bridge_client.py`, `executor_main.py`. 새 파일: `sim_in_loop_steer.py`, 시험 `tests/test_steer_executor.py`, `tests/test_sim_in_loop_steer.py`.
- 차동 경로는 그대로다. 2.2절의 기준 에피소드 10개(E1–E6, R1–R4)를 고친 트리로 다시 돌렸다. 행, trace, 실행기 기록(vendor_commit 포함·제외)의 SHA-256이 p4_import의 `imported_run1`과 모두 같다. 다른 것은 벽시계 값 `wall_s`뿐이다. 기록: `runs/wf3a/p4_exec/reference_diff/run1/MANIFEST.json`.
- 손대지 않은 조향 블록은 설정 dump와 digest에서 빠진다. 기본 설정 digest `700ee41c…`, 단일열 SHADOW digest `594143ca…`가 546fa8c와 같다(시험이 고정값으로 비교).
- `turn_*` import 금지, 경로 규칙, vendor 절차(7절)는 그대로다. 실행기 파일이 바뀌었으므로 `regen_vendor.py --worktree`로 `SHA256SUMS`와 `vendor/SOURCE_MANIFEST.json`을 다시 만들었다. `check_vendor_sync.py`와 `executor_main.verify_sources()`가 통과한다.
- 11절의 넘길 일 가운데 Decision(throttle, steer_mrad), SteerOutputGuard, GO 규칙·회전 중심, BridgeClient 조향 필드, `sim_in_loop_steer.py`는 끝났다. 식별 도구는 p4_ident의 일이다.

## 13. 검토 처리 (fix_wf3a, 2026-11-09): vendor 이름과 import

- 검토 P4R-02·RS-07: `check_pins`가 `vendor/closed_loop/__init__.py`, `vendor/orchard_perception/__init__.py`를 SIM 트리 꾸러미 이름 가림으로 봤다(A15 'import 가림 0' 불합격, R3상 커밋 불가). 실제로도 `load_vendor()`가 vendor 폴더를 `sys.path` 맨 앞에 넣어 `closed_loop`·`orchard_perception` 이름을 차지했다. 그래서 SIM 트리와 vendor를 한 프로세스에서 쓸 수 없었다(2.1절의 146 실패).
- 고친 것:
  - vendor 꾸러미 이름: `vendor/closed_loop_copy/`, `vendor/orchard_perception_copy/`(파일 6개는 SIM 트리와 바이트가 같다). `regen_vendor.py --worktree --remove … --add …`로 옮기고 빈 옛 폴더를 지웠다(`prune_empty_dirs`).
  - `core.load_vendor()`: vendor 폴더를 `sys.path`에 넣지 않는다. 사본은 이 꾸러미의 하위 꾸러미로 import된다(`row_executor.vendor.closed_loop_copy.estimator` 등. `vendor/`는 `__init__.py` 없는 이름 공간 꾸러미). vendor 추정기는 SIM 이름 `orchard_perception.scan_corridor`를 절대 import한다(바이트를 바꿀 수 없다). 그 import 동안만 `sys.modules`의 그 이름이 vendor 사본을 가리키고(`_SimTreeNames`), 끝나면 원래 항목(SIM 트리 것 또는 없음)을 되돌린다. import한 파일마다 manifest sha256을 다시 본다. 추정기가 vendor 감지기를 쓰는지도 본다.
  - `verified_modules`, `single_row_module`: manifest를 `source_path`(SIM 트리 경로)로 찾는다. vendor 이름이 바뀌어도 그대로다.
  - `vendor/SOURCE_MANIFEST.json`의 `import_root`, `verification` 문구를 새 방식으로 바꿨다(`regen_vendor.py`가 늘 새 문구를 쓴다).
  - `SINGLE_ROW_MODE.txt`의 경로 한 줄.
- 확인: `python -I tools/steer_sim/check_pins.py` → `pinned 423: unchanged 416, changed 7, missing 0, allowed 7, unexpected 0`, `OK`. `check_vendor_sync.py` OK. 실행기 시험 전체를 한 pytest 프로세스로 돌려 438 통과, 13 건너뜀(Windows 3.13.2). 새 시험: `test_runner.py::test_vendor_and_sim_tree_live_in_one_process`(SIM 이름이 새로 생기지 않음, `sys.path` 그대로, vendor 추정기가 vendor 감지기를 씀), `test_vendor_tools.py::test_regen_moves_a_vendored_file_and_prunes_the_old_folder`.
- 바뀐 시험: `test_runner.py`(vendor 경로, 감지기는 `load_vendor()[0].observe_scan_corridor`로 얻는다), `test_vendor_tools.py`(vendor 경로).
- 실차 배치(`/opt/row_executor/row_executor/vendor/`)도 같은 구조다. 실행 방식(`python3 -m row_executor.executor_main`)은 그대로다. 이 작업은 실차에 설치하지 않았다.
- 검토 P4R-12: `check_vendor_sync.py`가 git은 있는데 manifest 커밋이나 경로를 git이 모르는 경우를 PROBLEM으로 센다. git이 없거나 git 작업 트리가 아닐 때만 note로 둔다. 시험 `test_vendor_tools.py::CheckGitTest`(가짜 git 응답으로 커밋 없음, 경로 없음, git 없음, 작업 트리 아님).
