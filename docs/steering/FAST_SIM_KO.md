# 조향 빠른 SIM: 장면 명세·기동·실행기 (P2 뒷부분)

- 작성: 2026-11-09. 설계 v1 8.3 P2 뒷부분(장면 명세, 기동 대본, W0, 실행기), 7.2 A1(F)·A4(F)·A8·A11, P1 셋째 씨앗 시험, probe 재실행.
- 2026-11-09 리뷰 고침(작업 표지 fix_wf2b): 봉인 열 쌍 역방향 거부(R10), A4(F) release 개정 1 표본과 판정(R4), 건너뛰기 식별값(R5), trace 메타의 장면 SHA(R6), 하네스 첫 시간 표본(R7), 하네스 발자국(R8), `steer_target_rad` 표시(R9), 저장소 안 run·빌드 폴더 거부(R12). 고친 뒤 기록은 `runs/wf2b/fix_wf2b/`에 있다.
- 코드: `sim/ubts/steering/{scene_spec,maneuvers,flat_world,fast_sim}.py`. 시험: `sim/ubts/test_steering_{scene_spec,maneuvers,fast_sim,seed_regressions}.py`.
- 모든 수치는 자리값 기반 SIM 결과다. 현장 검증이 아니다. 실행 기록은 저장소 밖 `D:/Develop/UBTS/steering-sim-20261108/runs/wf2b/p2_core/`에 있다.
- 평가기(`evaluate_steer.py`)는 이 문서 범위 밖이다. 아래 trace 형식 위에 만든다.

## 1. 장면 명세 (`scene_spec.py`)

한 장면은 에피소드에 필요한 것을 모두 적는다. seed 하나가 에피소드 하나다.

| 칸 | 뜻 | 기본값 |
|---|---|---|
| `name` | 소문자·숫자·`_ . -`, 96자 이하 | 필수 |
| `stream_key` | 난수 흐름 키. (seed, CRC-32(키), 용도)로 흐름을 만든다(하네스와 같은 규칙) | `name` |
| `world` | `W0`(평지, 기둥 `markers` [[x, y, r]]) 또는 `W2`(카탈로그 회랑: `record`, `corridor_id` c01…, `direction`, `modifiers`) | W0, 기둥 없음 |
| `start` | base 자세 `x_m`, `y_m`, `yaw_deg`(W0는 바닥 좌표, W2는 회랑 s·d·방향 오차), 참 바퀴쪽 등가각 `steer_deg`, 조건 이름 `condition` | 0, 0, 0, 0, `nominal` |
| `profile` | `profile_id`, 선택 `sha256` 고정, `overrides`(VehicleProfile.with_overrides), `override_label`, `allow_out_of_range`, `curve`(main/bracket/곡선 이름), `geometry_check`, `steer_mode` | 자리값 프로필, main, test_servo |
| `seeds` | 겹치지 않는 0 이상 정수 목록 | [0] |
| `backend` | `profile`(프로필의 `steer_actuator_level`), `L0`(Python 이상 서보), `L1`(C++ 구동기). L0·L1은 프로필 보기의 수준도 같이 바꾼다. 그래서 결과 표지의 구동기 수준이 늘 프로필과 맞다 | `profile` |
| `runner` | `direct`(fast_sim 루프) 또는 `harness`(closed_loop 하네스 hook) | `direct` |
| `estimator` | 하네스에서만 `sensor`(SensorEstimator) 또는 `null`. direct에서는 비워 둔다 | 하네스면 `sensor` |
| `maneuver` / `controller` | 정확히 하나. 기동은 2절. 제어기는 `probe_pure_pursuit` 하나(7절) | 필수 |
| `duration_s` | SIM 시간 상한 | 30 |
| `command_latency_s` | 명령 지연(하네스 EpisodeConfig 기본과 같다) | 0.05 |
| `gyro_noise_scale` | gyro 잡음 배율 | 1 |
| `stop_rules` | direct 전용. `travel_m`, `field_x_m`, `turned_deg`, `rest_s`, `abs_lateral_m` | 없음 |
| `trace_every` | trace를 몇 스텝마다 남길지(시간 기록은 매 스텝) | 1 |

- 정규 JSON: 모든 칸을 기본값까지 적고, 키를 정렬하고, 간결 구분자·ASCII·NaN 금지로 쓴다. 기동 매개변수도 기본값을 채우고 수를 float로 바꾼다. 그래서 같은 장면은 쓰는 방식이 달라도 SHA-256이 같다. 예외: 프로필 `overrides` 안의 수는 적은 그대로 둔다(30과 30.0은 SHA가 다르다).
- `sha256()`: 장면 전체. `episode_sha256(seed)`: seeds를 [seed]로 바꾼 장면. seed를 더해도 다른 에피소드의 식별값은 그대로다. 결과마다 둘 다 남긴다(설계 7.3).
  - trace 안(`meta_json`, 내용 SHA에 들어감)에는 `episode_scene_sha256`만 둔다. 장면 전체 SHA는 `EPISODES.jsonl`에만 있다. 2026-11-09 고침(리뷰 R6): 처음에는 meta에 장면 전체 SHA가 들어가서, seed를 더한 장면에서는 같은 에피소드의 trace 내용 SHA가 달라졌다(A8 비교 거짓 실패). 이제 seed를 더해도 같은 에피소드의 trace 내용 SHA가 같다.
- 거부하는 것: 모르는 키, 잘못된 이름·seed·범위, 기동과 제어기 동시 지정, direct의 estimator, 하네스의 stop_rules, `geometry_check`와 L1 동시 지정(geometry_check는 L0), probe 제어기의 하네스·W2 사용, W0의 회랑 칸, W2의 기둥.
- 봉인 회랑(단일열 개정 A1, 설계 7.3): 042301 c01·c04·c07 fwd와 그 열 쌍의 역방향(c01·c04·c07 rev, 설계 7.3 "같은 열 쌍 역방향은 개발 근거로 쓰지 않는다")은 `allow_sealed: true`가 없으면 거부한다. 규칙은 `scene_spec.sealed_status()` 한 곳에 있고 A24 목록(`entry_feasibility.split_directions`)도 그것을 쓴다. 2026-11-09 고침(리뷰 R10): 처음에는 장면 명세가 c01·c04 rev를 받았다.
- 목록 파일: `load_scenes()`가 장면 목록(JSON 배열 또는 `{"scenes": [...]}`)을 읽는다. 이름은 겹치면 안 된다. `dump_scenes()`가 정규 형태로 쓴다.

## 2. 기동 (`maneuvers.py`)

- 기동은 SIM 시험 입력이다. 실행기가 아니다. 20 Hz마다 `command(t, state)`가 `Command(throttle_pct, steer_target_mrad, stop_mode, steer_policy, phase)`를 낸다. 앞 두 칸이 하네스 `SteeringActuation`이 읽는 값이다.
- `state`는 스텝 뒤 플랜트 참값(시간, 속도, 출발 뒤 방향 변화, 거리)이다. 상태를 읽는 기동은 둘뿐이다: 방향 전환 지그재그와 정지 기동(속도가 0이 되면 휴지 시작). 그 밖의 기동은 시간만 쓴다.
- `stop_mode`가 있으면 실행기가 그 정지 모드로 바뀔 때 한 번만 `plant.stop(mode, steer=policy)`를 부른다. 같은 정지 명령이 반복되면 플랜트를 건드리지 않는다(반복하면 정지 사건이 다시 시작되고, 구동 명령은 coast의 EN을 다시 켜기 때문이다).
- 속도 목표(`speed_m_s`)는 만들 때 한 번 duty로 바꾼다(`duty_for_speed`: 프로필 구동 곡선의 명목 정상 속도, 이분법). 플랜트의 run별 속도 배율은 그대로 적용된다.

| 기동 | 매개변수(시간 s, 각 deg, duty %) | 동작 |
|---|---|---|
| `step` | `at_s`, `throttle_pct`·`steer_deg`(값 하나 또는 [전, 후]) | at_s에서 계단 |
| `ramp` | `start_s`, `end_s`, `throttle_pct`·`steer_deg`(값 또는 [시작, 끝]) | 두 채널 선형 보간 |
| `circle` | `throttle_pct` 또는 `speed_m_s`, `steer_deg` | 고정 원 |
| `sine_sweep` | 구동, `amplitude_deg`, `f0_hz`, `f1_hz`, `sweep_s`, `start_s`, `offset_deg` | 선형 chirp: 2π(f0·τ + (f1−f0)·τ²/(2·sweep_s)) |
| `zigzag` | 구동, `amplitude_deg`, `switch`(time/heading), `half_period_s`, `heading_deg`, `start_s`, `switches` | 시간 전환 또는 방향 ±heading_deg에서 전환(고전 지그재그). switches번 뒤 0 |
| `stop_test` | 구동, `steer_deg`, `drive_s`, `hold_s`, `repeat` | drive_s 주행 → throttle 0(키 놓기, MCU 페이드) → 정지 뒤 hold_s 휴지 → 반복 → 끝 |
| `stop_mode` | `mode`(release/brake/coast), `steer_policy`, 구동, `steer_deg`, `drive_s`, `hold_s`, `repeat` | 주행 → `plant.stop(mode)` → 휴지 → 반복 → 끝 |

## 3. W0 평지 세계 (`flat_world.py`)

- 하네스 세계와 같은 이름을 준다: `to_field`·`pose_in_field`(항등, 바닥 좌표가 곧 field 좌표), `corridor_truth(0)`(열 끝 1000 m, 벽 없음, 다음 열 UNKNOWN), `clearance()`(기둥까지 발자국 거리, 가지 없음).
- 하네스 hook: `make_flat_lidar(world, seed)`(정확한 광선·원 교차, numpy만 사용, 3200빔·30 m·잡음 0.015 m, 하네스 SIM 값), `flat_clearance`, `FlatHooks.kwargs()`.
- `registration_posts()`: x축 양옆에 번갈아 선 기둥 몇 개(LiDAR 정합용, 설계 5.1).

## 4. 실행기 (`fast_sim.py`)

### 4.1 두 실행 방식

| 방식 | 내용 |
|---|---|
| `direct` | 플랜트 100 Hz, 구동 명령 20 Hz, 지연 큐. 하네스와 같은 시각 규칙이다. stop_rules를 쓴다 |
| `harness` | `closed_loop.harness.run_episode` + `plant_factory`(trace를 남기는 SteeredPlant 하위 클래스) + `actuation`(`ScriptActuation`, SteeringActuation 하위 클래스, 정지 모드 처리) + 세계 hook(W0 flat_world, W2 raster_corridor). 기동이 끝나면 terminal HOLD로 들어가고 하네스가 정지 1 s 뒤 끝낸다. 하네스 자체의 충돌 종료와 여유 지표는 `EpisodeConfig.plant_overrides`로 받은 장면 프로필의 발자국 사각형(`footprint_length_m`, `footprint_width_m`)을 쓴다. plant_factory가 받는 덮어쓰기는 이 두 값뿐이다 |

- 같은 장면·seed·stream_key면 두 방식의 플랜트 궤적이 비트까지 같다. 센서 합성과 추정기는 플랜트를 건드리지 않기 때문이다. 실제로 A1·A4 835쌍에서 공통 구간이 모두 같았다(5절).
- 하네스 방식은 scipy가 필요하다. WSL Python 3.10에서는 쓰지 못한다(결정 R10). direct 방식은 3.10에서도 돈다.
- 2026-11-09 고침(리뷰 R8): 처음에는 하네스가 차동 `FieldResponseParams` 기본 발자국 1.22 × 0.70 m로 충돌을 판정했다. 프로필 발자국을 바꾸면(예: 너비 0.60 m) 하네스가 몸체 접촉 없이 'harness:collision'으로 일찍 끝나 direct와 결과가 갈렸다. 이제 같은 장면에서 두 방식의 충돌 판정이 같다(시험 `test_harness_collision_uses_the_profile_footprint`). 자리값 프로필은 1.22 × 0.70 m라 기본 결과는 바뀌지 않는다.

### 4.2 사용법

```
cd sim/ubts
python -B -m steering.fast_sim run --set a1 --out <run 폴더> [--runner harness] [--workers N]
python -B -m steering.fast_sim run --scenes <장면 목록.json> --out <run 폴더> --workers 6
python -B -m steering.fast_sim check --kind a1|a4|probe --run <run 폴더>
python -B -m steering.fast_sim compare --runs <run 폴더1> <run 폴더2> ...
```

- 내장 장면 묶음: `a1`(원 8개), `a4`(정상 속도·후진·release 개발 seed 0~399 × 2곡선·brake/coast 18개), `a4_judgement`(A4(F) 판정 묶음: `a4`와 같고 release만 개정 1의 seed 10000~13999 × 2곡선, `--seeds` 거부), `a8`(L1·L0, direct·하네스, probe 섞음 8장면 × 2 seed), `probe`(probe 재실행, 7절).
- `check`의 종료 코드: 통과 0, 실패 1, 판정 아님 2(예: 개발 seed의 release 중앙값만 있는 A4 run).
- 라이브러리: `run_scenes(scenes, out_dir, workers=1, force=False, hold_check=True, hold_poll_s=60, native_executable=None, native_build_dir=None, progress=None)` → SUMMARY dict. `run_episode(scene, seed, native_executable=None)` → (trace 배열, 시간 배열, 기록). 분석: `load_trace`, `analyse_circle`, `analyse_stops`, `analyse_probe`, `check_a1`, `check_a4`, `compare_runs`, `probe_table`.
- 환경: `UBTS_STEER_PATHS`(외부 경로 설정, `PATHS_KO.md`), `UBTS_STEER_BUILD_DIR`(네이티브 빌드 위치, 작업 트리 밖으로). HOLD 폴더는 `UBTS_HOLD_DIR` 또는 설정 파일의 `hold_dir`. 설정이 없으면 실행을 시작하지 않는다(`PathNotConfigured`). `hold_check=False`는 시험용이다.
- L1 장면이 빌드가 필요하면 `native_build_dir` 인자나 `UBTS_STEER_BUILD_DIR`이 있어야 하고 저장소 밖이어야 한다(`checked_native_build_dir`). 없으면 실행 전에 거부한다. 2026-11-09 고침(리뷰 R12): 처음에는 둘 다 없으면 `native/build.py` 기본값(작업 트리 안 `steering/native/_build/`, git 무시)에 빌드했다.
- run 폴더는 저장소 밖이어야 한다. 이 작업 트리뿐 아니라 같은 저장소의 다른 작업 트리(main checkout 포함) 안도 거부한다(`scene_spec.inside_repository`: git 파일 `.git`, `commondir`, `worktrees/*/gitdir`를 읽는다. git 명령은 쓰지 않는다). 2026-11-09 고침(리뷰 R12).
  - Windows git이 만든 작업 트리의 git 파일에는 드라이브 경로(`D:/...`)가 있다. WSL에서는 이것을 WSL 기본 붙임 위치 `/mnt/d/...`로 읽는다(`scene_spec._git_file_path`). 상대 경로(git `worktree.useRelativePaths`)는 그 파일이 있는 폴더 기준으로 읽는다. 처음 고친 판은 WSL에서 드라이브 경로를 읽지 못해 이 작업 트리만 보았다(WSL 시험 기록에서 저장소 가드 시험이 "다른 작업 트리 없음"으로 건너뜀, `fix_wf2b/tests_wsl_py310_before_gitpath_fix.log`).
  - 경로가 이 환경에 없으면(예: 컨테이너 안의 호스트 경로) 그 작업 트리는 빠진다. 컨테이너 안에서는 붙인 작업 트리만 막는다.
- 작업자: 1이면 같은 프로세스, N > 1이면 spawn 프로세스 N개(Windows 안전, 12개 이하). 작업자는 에피소드마다 `HOLD_MEASUREMENT`가 있는 동안 기다린다. BLAS 스레드는 1로 둔다.
- 네이티브(L1): `run_scenes`가 처음에 한 번 빌드(또는 재사용)하고 그 파일을 모든 에피소드·작업자에 넘긴다. 실행 중 소스가 바뀌어도 한 run은 한 바이너리로 돈다. 에피소드마다 하위 프로세스를 만들고, 끝나면(실패해도) `plant.close()`로 멈춘다.

### 4.3 출력 구조

```
<run 폴더>/
  RUN.json              장면(정규 명세와 SHA), 코드 식별값(파일별 SHA), Python·numpy·플랫폼, 네이티브 바이너리 SHA, 작업자 수
  EPISODES.jsonl        에피소드 기록. 장면 순서 → seed 순서(작업자 수와 무관)
  SUMMARY.json          개수, 장면별 종료 이유, trace SHA 목록과 묶음 SHA, A11 시간(이번 실행분만)
  trace/<id>.npz        trace(결정적 zip: 고정 시각, 정렬된 멤버)
  timing/<id>.npz       스텝별 벽시계(plant_step_us, backend_step_us, loop_step_us, float32). 해시에 넣지 않는다
  episodes/<id>.json    에피소드 기록(다음 실행의 건너뛰기 판단에 쓴다)
```

- `<id>`는 `<장면 이름>__seed<n>`이다.
- 건너뛰기: 기록이 있고, status가 ok이고, 에피소드 장면 SHA·식별 SHA(코드 파일, 프로필, 네이티브 바이너리, Python, numpy, 플랫폼, W2면 카탈로그·지도 PGM·YAML SHA)가 같고, trace 파일 SHA가 기록과 같을 때만 건너뛴다. `force=True`면 모두 다시 돈다. 실패한 에피소드는 `status: failed`와 오류를 남기고 run은 계속한다.
- 2026-11-09 고침(리뷰 R5):
  - 코드 파일 목록(`IDENTITY_FILES`)에 에피소드가 실제로 싣는 sim/ubts 모듈을 모두 넣었다(빠졌던 것: `closed_loop/__init__.py`, `orchard_perception/__init__.py`, `scan_corridor.py`, `scenarios.py`, `steering/native/__init__.py`, `lockstep_v2.py`). 시험(`test_identity_covers_every_loaded_module`)이 새 프로세스에서 direct L0·하네스 sensor·L1 에피소드를 돌리고 실린 모듈이 목록 안인지 본다.
  - W2 장면은 카탈로그 SHA와 회랑 지도 PGM·YAML SHA(`world_data_identity`)가 식별값에 들어간다. 지도가 바뀌면 다시 돈다. `build_raster_world`가 지도 파일을 이 SHA와 대조한다.
  - 작업자는 에피소드가 끝난 뒤(쓰는 모듈이 모두 실린 뒤) 코드 파일 SHA를 다시 계산한다. 계획 때와 다르면 trace를 저장하지 않고 `status: failed`, `error.type: code_identity_changed`로 남긴다(다른 작업이 소스를 고치는 중일 때 기록이 돌지 않은 코드를 가리키지 않게). 같은 프로세스 안에서 이미 실린 모듈이 그 뒤 디스크에서 바뀐 경우는 이 검사로 잡지 못한다(spawn 작업자는 새로 싣는다).
- `EPISODES.jsonl`의 각 줄에 `run_scene_sha256`(이번 run의 장면 SHA)을 적는다. 재사용한 기록의 `scene_sha256`은 그 기록을 만든 run의 값이다(리뷰 R6).

### 4.4 trace 형식 (평가기용)

한 행은 기록한 플랜트 스텝 하나다(`trace_every`). 단위는 이름 끝에 있다.

| 키 | 단위·형 | 참값? | 뜻 |
|---|---|---|---|
| `t_s` | s, f8 | 아님 | 스텝 끝 SIM 시각 |
| `cmd_throttle_pct` | %, f8 | 아님 | 효력 중인 구동 명령(지연 큐 뒤). 첫 명령 전 0 |
| `cmd_steer_target_mrad` | mrad, f8 | 아님 | 효력 중인 조향 목표 명령. 첫 명령 전 NaN(플랜트는 출발 각을 유지) |
| `cmd_stop_mode` | i1 | 아님 | 0 없음, 1 release, 2 brake, 3 coast |
| `phase`, `phase_names` | i2, 문자열 | 아님 | 효력 중인 명령의 단계 이름 번호. 첫 명령 전 `start` |
| `steer_target_rad` | rad, f8 | 아님(아래 주의) | 등가 소프트 한계로 자른 서보 목표. 주의(리뷰 R9): hold_measured 정지 뒤에는 플랜트의 서보 관점 각(`servo_view_rad()`, 참 모터 쪽 각에서 만든 값)이다. 실제 MCU는 거른 가변저항 측정을 잡을 것이다(P1·P3). trace 메타 `steer_target_note`에도 적는다 |
| `drive_request_pct`, `drive_applied_pct` | %, f8 | 아님 | 요청 duty와 MCU 램프 출력. 속도가 아니다 |
| `drive_en`, `steer_en` | i1 | 아님 | 명령한 EN |
| `pot_raw`, `steer_measured_rad`, `fault_bits` | count i2, rad f8, i4 | 아님 | 가변저항 ADC, 그 명목 해석, MCU 결함 마스크 |
| `truth_x_m`, `truth_y_m`, `truth_yaw_rad`, `truth_yaw_unwrapped_rad` | m, rad | 참값 | base 자세(W0 바닥, W2 래스터 국소 m) |
| `truth_field_x_m`, `truth_field_y_m`, `truth_field_heading_rad` | m, rad | 참값 | field 좌표(W0는 위와 같다, W2는 회랑 s, d, 방향 오차) |
| `truth_rear_x_m`, `truth_rear_y_m` | m | 참값 | 뒤차축 위치 |
| `truth_v_m_s`, `truth_w_rad_s`, `truth_external_yaw_rate_rad_s` | m/s, rad/s | 참값 | 속도, 조향 요레이트(외부 yaw 제외), 외부 yaw |
| `truth_distance_m`, `truth_curvature_1_m`, `truth_steer_curvature_1_m`, `truth_wander_deg_per_m`, `truth_lateral_slip_m_s` | m, 1/m, deg/m, m/s | 참값 | 거리, 곡률(떠돎 포함·조향만), 곡률 떠돎, 옆 미끄럼 |
| `truth_delta_eq_rad`, `truth_delta_left_rad`, `truth_delta_right_rad`, `truth_motor_side_rad`, `truth_applied_steer_pwm`, `truth_model_flags` | rad, 분수, 마스크 | 참값 | 구동기 참값 출력(backend_api.TRUTH_OUTPUT_FIELDS) |
| `truth_servo_view_rad` | rad, f8 | 참값 | 시험용 서보가 보는 서보 쪽 각(`SteeredPlant.servo_view_rad()`). hold_measured가 잡는 위치이며 평가기 `STEER_AFTER_TERMINAL`의 유지 기준이다. 2026-11-09 더함 |
| `truth_decelerating`, `truth_regime` | i1 | 참값 | 감속 중, 0 정지·1 주행·2 감속·3 막힘 |
| `ctrl_*` | 이름 끝 단위 | 참값에서 만듦 | probe 제어기의 관측(참 뒤차축 자세 + 잡음)과 곡률 명령 |
| `meta_json` | u1 바이트(JSON) | 아님 | schema, id, 에피소드 장면 SHA, dt, 단위, 참값 키 목록, `steer_target_note`. 장면 전체 SHA는 넣지 않는다(리뷰 R6) |

- `content_sha256`: 멤버마다 (이름, dtype, shape, C 순서 바이트)를 이름 순으로 이은 SHA-256. A8은 이 값을 비교한다. 파일 자체도 결정적이라 같은 플랫폼에서는 파일 SHA도 같다.
- 에피소드 기록(`EPISODES.jsonl` 한 줄): id, 장면 이름, seed, 장면·에피소드 SHA, 식별 SHA, 실행 방식, 세계, 출발, 구동기(요청 수준, 실제 수준, 클래스, 네이티브면 빌드 id와 바이너리 SHA), 구동원 설명(`truth_fed_driver`), 프로필 표지(`profile_id`, SHA, 출처 등급, `자리값 기반 SIM 결과`, 덮어쓰기), run별 뽑기값(`run_info`), 종료(이유, 시각, 스텝), 간단 통계, trace 파일·SHA, 시간 요약, 하네스 결과(하네스 방식).
- 종료 이유: `duration`, `maneuver_done`, `stop_rule:<규칙>`, 하네스는 `maneuver_done`(terminal_hold) 또는 `harness:<끝>`.

## 5. 합격 검사 결과 (Windows Python 3.13.2, numpy 2.2.6)

기록: `runs/wf2b/p2_core/acceptance_win313/`(run1), `acceptance_win313_run2/`(run2), `a4_release_diag/`, 판정 run `runs/wf2b/fix_wf2b/a4_judgement/run/`(`ACCEPTANCE_A4.json`).

### 5.1 A4(F) release 판정 (합격 기준 개정 1)

- `SIM_ACCEPTANCE_KO.md` 개정 1(측정 전 개정): 곡선마다 release 장면을 아직 쓰지 않은 seed 10000~13999(4000개)로 돌리고, stream key는 `a4_release_<곡선>`(실행 방식 이름 없음), 정지 직전 속도 0.90~0.95 m/s 창 안의 정지 전부로 중앙값을 낸다. 창(0.56~0.60 m)은 그대로다.
- 코드: 내장 묶음 `a4_judgement`(`fast_sim.a4_judgement_scenes()`), `release_sample_kind()`가 표본을 `amendment_1`/`development`로 가른다. `check_a4`는 개정 1 표본(장면 하나, 에피소드 4000개 모두 ok)일 때만 release를 판정하고, 다른 표본은 보고만 한다(`passed` None). 그전 기준의 `n >= 30` 조건은 등록 문구에 없어서 뺐다. 시험은 이 seed를 돌리지 않는다.
- 판정 실행은 한 번 했다(2026-11-09, direct, 작업자 6, 8027 에피소드 모두 ok, 198 s). 결과:

| 곡선(장면) | 창 안 정지 수 | 중앙값 | p10 / p90 | 판정 |
|---|---|---|---|---|
| P_fit1003 (`a4_direct_release_main`) | 970 | 0.5830 m | 0.5235 / 0.6534 m | 통과 |
| field1010 (`a4_direct_release_bracket`) | 900 | 0.5652 m | 0.4788 / 0.6417 m | 통과 |

- 정상 속도 9개, 램프 9개, brake·coast 18개도 같은 run에서 모두 통과했다. 그래서 A4(F) 전체가 이 판정 run에서 통과다(`ACCEPTANCE_A4.json` `passed` true).
- field1010 중앙값은 창 아래 끝에서 5.2 mm 위다. 진단 실행(seed 0~3999)의 모델 중앙값 0.5648 m와 맞는다. 자리값 기반 SIM 결과다.
- 아래 표의 run1·run2 release 행은 개정 1 전 기록이다. run1 하네스 실패(field1010 0.5575 m)는 그대로 남긴다. run2 표본은 실패 뒤 stream key를 바꾼 재표본이라 판정으로 쓰지 않는다(리뷰 R4).

### 5.2 첫 기록(run1, run2)

| 검사 | 결과 | 판정 |
|---|---|---|
| A1(F) direct | 원 8개(δ 5/10/20/−10°, v 0.3/0.5). 반경 오차 최대 2.4e-7. 요레이트 부호 모두 일치 | 통과 |
| A1(F) 하네스 | 같은 8개. 최대 2.4e-7, 부호 일치 | 통과 |
| A4(F) 정상 속도 | main 40/55/70/88 %, bracket 90/95/100 %, 후진 2개: 오차 0(이상값 보기) | 통과 |
| A4(F) 램프 | 100 %/s(255 PWM/s), 9개 모두 | 통과 |
| A4(F) brake·coast | 18개 모두 닫힌 식 v·onset + v²/(2a) 안(최대 차 4.9 mm < v·dt), 시간도 안 | 통과 |
| A4(F) release 중앙값, run2 | P_fit1003 0.5784 m(n 108), field1010 0.5751 m(n 91). direct·하네스 같음 | 판정 아님(실패 뒤 바꾼 재표본, 5.1절이 판정) |
| A4(F) release 중앙값, run1 | direct: 0.5826 / 0.5644 m 창 안. 하네스(다른 난수 흐름): P_fit1003 0.5877 m 창 안, field1010 0.5575 m | 하네스 표본은 **실패**(기록 보존) |
| A8 | 같은 seed 두 번(작업자 1)과 작업자 6: 16개 에피소드 trace 내용 SHA·파일 SHA 모두 같음 | 통과(같은 플랫폼) |
| 실행 방식 교차 | 같은 stream_key·seed에서 direct와 하네스 trace 공통 구간이 모두 같음: A1 8쌍, A4 827쌍(474,780행) | 같음 |

- release 실패의 원인(진단, 판정 아님): 4000 seed 표본에서 모델 중앙값은 field1010 0.5648 m(창 안 887개), P_fit1003 0.5812 m다. 둘 다 창 안이다. 그러나 정지 약 100개 표본을 다시 뽑으면 field1010 중앙값이 창 밖으로 나가는 비율이 24.7 %다(P_fit1003 0.05 %). 그래서 그 크기의 표본으로는 통과·실패가 seed에 갈렸다. 기준은 바꾸지 않았고, 통합 담당이 측정 전에 표본을 정하는 개정 1을 적었다(5.1절).
- run1과 run2의 차이: run2에서 A1·A4 장면의 stream_key에서 실행 방식 이름을 뺐다. 그래서 두 방식이 같은 난수를 쓰고 비트 비교가 된다. run1 하네스 실패 기록은 그대로 남겼다.

## 6. 처리 시간 (A11, 보고 전용)

같은 PC에서 다른 작업(Gazebo 등)이 함께 돌 때 잰 값이다. 실행기 `ExecutorLoop.step` 시간(dev_r3 p99 56.2 ms)과는 다른 양이다. fast SIM 자체의 비용이다.

| 묶음 | 작업자 | plant step p50 / p99 / p99.9 | backend step p50 / p99 | loop step p99 | 에피소드 벽시계 |
|---|---|---|---|---|---|
| A8 direct L1(네이티브) | 1 | 103 / 339 / 679 µs | 79 / 278 µs | 377 µs | p50 0.32 s(SIM 약 12~30 s) |
| A8 direct L1 | 6 | 166 / 314 / 486 µs | 127 / 236 µs | 346 µs | p50 0.56 s |
| A8 direct L0 | 1 / 6 | 35 / 128 / 248 µs, 40 / 132 / 230 µs | 18 / 69 µs | 150 µs | 0.05 s(SIM 10 s) |
| A8 하네스 L0·L1(센서 추정기, 기둥 8개) | 1 | 40~135 / 140~430 µs | | 8.3~9.1 ms(10 Hz LiDAR·추정기 스텝) | 1.8~1.9 s(SIM 25 s) |
| probe L1 | 6 | 128 / 515 / 1136 µs, 최대 147 ms(OS 지연) | 97 / 395 µs | 570 µs | p50 1.20 s, p99 3.56 s(SIM 약 76 s) |
| probe 이상 L0 | 6 | 30 / 82 / 263 µs | 18 / 50 µs | 118 µs | p50 0.36 s |

- A8 묶음 전체: 작업자 1이 11.8 s, 작업자 6이 4.8 s. probe 768개는 작업자 6으로 145 s.
- 2026-11-09 고침(리뷰 R7): 하네스 방식은 시간 기록 시계를 `H.run_episode` 호출 전에 맞춰서 첫 loop 표본에 에피소드 준비 시간(세계·플랜트, L1이면 하위 프로세스 시작, 추정기, LiDAR)이 들어갔다. 그래서 위 표의 하네스 최대값과 SUMMARY의 max가 준비 시간이었다(첫 표본 L1 19.1~25.0 ms, L0 1.8~4.8 ms). 이제 첫 loop 표본은 첫 플랜트 스텝 시작부터 잰다(두 방식 모두, 첫 표본 = 첫 플랜트 스텝).
- 고친 뒤 다시 잰 A8 묶음(`runs/wf2b/fix_wf2b/a8/`, Windows 3.13.2, 이때 이 PC의 다른 부하는 적었다). A8 판정: 작업자 1 두 번과 작업자 6의 16개 trace 내용·파일 SHA가 모두 같다(통과, 같은 플랫폼). trace 형식이 바뀌어(`truth_servo_view_rad` 칸, meta) 옛 run과는 SHA가 다르다.

| 묶음(작업자 1) | plant step p50 / p99 / p99.9 | loop step p50 / p99 / 최대 | 에피소드 벽시계 p50 |
|---|---|---|---|
| direct L0 | 16 / 42 / 104 µs | 19 / 57 / 158 µs | 0.023 s |
| direct L1 | 49 / 122 / 222 µs | 54 / 136 / 1845 µs | 0.17 s |
| 하네스 L0(센서 추정기, 기둥 8개) | 21 / 73 / 141 µs | 27 / 4141 / 6937 µs | 0.91 s |
| 하네스 L1 | 73 / 181 / 312 µs | 86 / 3988 / 6502 µs | 0.83 s |

  - 하네스 첫 loop 표본은 78~190 µs(첫 플랜트 스텝과 같음)이고, 하네스 최대값 6.3~6.9 ms는 10 Hz LiDAR·추정기 스텝이다. 묶음 전체 벽시계: 작업자 1 6.0 s, 작업자 6 3.2 s.
- L1 한 스텝의 대부분은 하위 프로세스 왕복(backend step)이다. MCU 로직(P3)이 붙으면 다시 잰다.

## 7. probe 재실행 (설계 6.2 갱신, 보고)

- 제어기 `probe_pure_pursuit`: upstream pure pursuit 곡률(포팅, 8절 시험으로 같음 확인) → κ를 1/R_min으로 자름(R_min = L̂ / tan(등가 소프트 한계 작은 쪽)) → 명목 속도에서 요레이트 0.8 rad/s로 자름 → δ = atan(L̂ κ). 관측은 참 뒤차축 자세에 잡음(0.03 m, 1°)을 더하고 10 Hz, 지연 0.1 s다. 원래 probe와 같다. 참값을 쓰는 측정용 대역이므로 실행기가 아니다. 기록에 `truth_fed_driver: true`를 남긴다.
- 조건: 0.4 m/s(main 곡선 duty 약 47 %), 30 m 직선, 출발(뒤차축) 0.30 m·3°와 0.60 m·10°, seed 0~11, 조향 속도 10/15/30/60°/s(정지 중은 절반, 5~30), 주시거리 1.0/1.5/2.0/2.5 m. 통과 = 완주 + 5 m 뒤 |y_뒤차축| 최대 < 0.30 m(원래 probe 정의, 발산 판정 아님).
- `profile`: 자리값 프로필 그대로(L1 구동기: 백래시 2°, 서보 데드밴드 0.5°, run별 0점 오차 N(0, 0.7°), 곡률 떠돎 평균 −0.15·sd 0.95 °/m, 옆 미끄럼, η·언더스티어·끌림 뽑기, 가변저항 잡음). `ideal`: 같은 격자에 geometry_check(L0, 오차항 없음).
- 768개 모두 완주했다. 기록: `runs/wf2b/p2_core/probe_win313/PROBE_TABLE.json`.

통과 수(24회 중, 두 출발 합)와 5 m 뒤 최대 횡오차 중앙값(m), 명령 sd 중앙값(°):

| 주시거리 | 10°/s | 15°/s | 30°/s(기본) | 60°/s |
|---|---|---|---|---|
| profile 2.5 m | 22/24, 0.225, 1.74° | 23/24, 0.206, 1.79° | 24/24, 0.184, 1.57° | 24/24, 0.143, 1.23° |
| profile 2.0 m | 24/24, 0.160, 2.03° | 24/24, 0.143, 1.69° | 24/24, 0.135, 1.57° | 24/24, 0.138, 1.48° |
| profile 1.5 m | 24/24, 0.101, 2.62° | 24/24, 0.089, 2.25° | 24/24, 0.080, 1.90° | 24/24, 0.083, 1.82° |
| profile 1.0 m | 13/24, 0.206, 7.05° | 24/24, 0.068, 3.96° | 24/24, 0.055, 3.46° | 24/24, 0.054, 3.38° |
| ideal 2.5 m | 24/24, 0.060, 0.83° | 24/24, 0.060, 0.81° | 24/24, 0.053, 0.81° | 24/24, 0.055, 0.83° |
| ideal 2.0 m | 24/24, 0.022, 1.05° | 24/24, 0.020, 1.05° | 24/24, 0.019, 1.04° | 24/24, 0.020, 1.04° |
| ideal 1.5 m | 24/24, 0.022, 1.58° | 24/24, 0.020, 1.59° | 24/24, 0.020, 1.60° | 24/24, 0.019, 1.55° |
| ideal 1.0 m | 21/24, 0.099, 4.40° | 24/24, 0.015, 3.12° | 24/24, 0.014, 3.13° | 24/24, 0.014, 3.14° |

- 0.4 m/s, 주시거리 2.0 m는 자리값 프로필에서 조향 속도 10~60°/s 모두 96/96 통과다. 5 m 뒤 최대 횡오차는 중앙값 0.135~0.160 m, 최댓값 0.188~0.276 m다. 10°/s에서 최댓값이 0.30 m 기준에 가깝다.
- 실패는 1.0 m·10°/s(진동, profile 13/24, ideal 21/24)와 2.5 m·10~15°/s(profile 3회)에서만 나왔다.
- ideal의 명령 sd(2.5 m 0.8°, 1.5 m 1.6°, 1.0 m 3.1°)는 원래 probe 기록(0.8 / 1.6 / 3.2°)과 맞는다.
- 오차항이 횡오차를 키운다: 2.0 m에서 ideal 약 0.02 m → profile 약 0.14 m. 분해(진단, `probe_decompose/DECOMPOSE.json`, 2.0 m·30°/s, 같은 seed 24회): 모두 켬 0.119 m(최대 0.259), 떠돎 끔 0.084(0.176), 0점 오차 끔 0.117(0.203), 백래시 끔 0.101(0.224), 셋 다 끔 0.071(0.123). 단일 항으로는 곡률 떠돎의 영향이 가장 크다. 셋을 꺼도 남는 0.07 m는 옆 미끄럼·η 뽑기·서보 데드밴드 몫으로 본다[추정].
- 0점 오차의 정상 오차(설계 3.2 표 κ_b·L_d²/2): 10 m~끝 평균 y를 예측값에 맞추면 기울기 0.91(1.5 m), 0.99(2.0 m), 0.83(2.5 m), 1.50(1.0 m), 상관 0.52~0.69, 잔차 sd 1.3~6.4 cm다. 1.5~2.0 m에서는 설계 식이 대체로 맞는다. 나머지는 떠돎·백래시가 섞인 결과다.
- 6.2절의 "실제 조향 속도를 재기 전까지 주시거리 2.0 m 이상"은 이 자리값 결과와 어긋나지 않는다. 다만 횡오차는 주시거리가 길수록 오차항 때문에 커진다(2.5 m가 2.0 m보다 나쁨). 주시거리 선택은 trim 적분(6.2절 b_δ) 뒤 P8에서 다시 본다.

## 8. 씨앗 회귀 시험 (`test_steering_seed_regressions.py`)

- upstream 순수 함수(`ros2_ws/src/orchard_navigation/orchard_navigation/controller.py`, 시험 `test_controller.py`)는 읽기만 한다. 파일 텍스트를 별도 모듈 객체로 실행한다(바이트코드·sys.path 변경 없음).
- 포팅한 곡률 시험: fast_sim 포팅 함수 = upstream 함수(210점 격자, 값이 정확히 같음), 중심선 쪽으로 도는 부호(기하 → 플랜트 요레이트), 중심선 위 직진(이상 플랜트가 정확히 직선), 최소 회전반경 제한(R_min = L / tan(등가 소프트 한계)에서 upstream 자르기 곡률 = 플랜트가 한계로 자른 곡률, 좌·우), w = v·κ = 플랜트 요레이트. 장애물·신뢰도 속도 시험은 곡률 함수가 아니어서 옮기지 않았다.
- probe 자전거: probe `run()`을 그대로 옮겼다. 환경 변수 `UBTS_STEER_PROBE_RESULT`가 probe `result.json`(저장소 밖)을 가리키면 기록된 행 6개를 정확히 재현한다(없으면 이 시험만 건너뜀). 그다음 같은 폐루프를 이상 플랜트(L0, 30°/s, τ 0.3 s)에 돌리고, 같은 관측 잡음으로 플랜트 속도를 따른 probe 자전거와 비교한다. 4경우에서 뒤차축 횡위치 차 최대 0.5~7.2 mm(기준 20 mm), 두 쪽 모두 probe 통과.

## 9. 시험 실행

| 환경 | 결과 |
|---|---|
| Windows Python 3.13.2, pytest 9.1 | 4개 파일 118 통과 |
| 기반 이미지 `6209168a3507`, Python 3.12.3, numpy 1.26.4, pytest 7.4.4 | 118 통과(건너뜀 0). 1단계 `test_steering_plant.py` 69 통과(네이티브 확인) |
| WSL Python 3.10.12, numpy 2.2.6, 대체 실행기(pytest 아님) | 115 통과, 3 건너뜀(하네스 시험: scipy가 numpy 2와 맞지 않음, R10) |

리뷰 고친 뒤(2026-11-09, fix_wf2b). 4개 파일 129개: `test_steering_scene_spec.py` 50, `test_steering_maneuvers.py` 28, `test_steering_fast_sim.py` 40, `test_steering_seed_regressions.py` 11. probe 결과 파일은 `UBTS_STEER_PROBE_RESULT`로 주었다.

| 환경 | 결과 |
|---|---|
| Windows Python 3.13.2, numpy 2.2.6, pytest 9.1 | 129 통과. 같은 실행에서 WF-2b 나머지와 1단계 조향 시험까지 16개 파일 650 통과, 1 건너뜀(W1 조립은 SIM 컨테이너 필요). 기록 `fix_wf2b/pytest_win313_final.log` |
| 기반 이미지 `6209168a3507`, Python 3.12.3, numpy 1.26.4, scipy 1.11.4, pytest 7.4.4 | 129 통과(하네스 시험 포함). 1단계 조향 시험 8개 파일 407 통과, 7 건너뜀(546fa8c 비트 동일성 기준은 같은 환경에서만 비교). 기록 `fix_wf2b/pytest_docker_py312.log` |
| WSL Python 3.10.12, numpy 2.2.6, 대체 실행기(pytest 아님) | 125 통과, 4 건너뜀(하네스 시험 4개: scipy가 numpy 2와 맞지 않음, R10). 기록 `fix_wf2b/tests_wsl_py310.log` |

- 시험은 작업 트리에 파일을 만들지 않는다(run 폴더·네이티브 빌드·HOLD 폴더 모두 임시 폴더). Docker에서는 작업 트리를 읽기 전용으로 붙였다.
- A4 시험은 개발 seed(0~399)로 파이프라인을 돌리고 release 중앙값을 판정하지 않는다(`sample` development, `passed` None). 판정 표본(seed 10000~13999)은 시험에서 돌리지 않는다.

## 10. 한계와 남은 일

- 모든 값이 자리값이다. 결과는 자리값 범위에 대한 SIM 결과일 뿐이다.
- probe 제어기는 참값 관측을 쓴다. 실행기·추정기 경로(A13)는 P4에서 본다.
- 하네스 방식의 `sensor` 추정기는 W0 기둥만으로는 ICP가 자주 실패한다(odom_failures). 기동에는 영향이 없다(추정기는 플랜트에 닿지 않는다).
- W2는 direct에서 수정자 없이 돌았다(Windows·Docker 시험). 수정자(gap 등)와 W2 하네스는 scipy가 필요하다.
- 평가기(조향 지표, 스윕 포함 발자국, REVERSE, STEER_AFTER_TERMINAL, 정지 때 벽·장애물 여유), A13·A21(평가기)·A24는 `EVALUATE_STEER_KO.md`에 있다. 위 trace 형식과 `run_scenes` API를 그대로 쓴다.
- 하네스 방식은 몸체 발자국(프로필 너비·길이, 리뷰 R8)이 닿으면 `harness:collision`으로 에피소드를 끝낸다. direct에는 충돌로 끝내는 규칙이 없다(멈춤 규칙은 이동 거리 등, 1절). 충돌 판정은 두 방식 모두 평가기(몸체와 바퀴 스윕)로 한다.
- 리뷰 R9의 원인(hold_measured 목표가 참 쪽 서보 값)은 P1·P3 범위라 고치지 않았다. 지금은 trace 메타의 `steer_target_note`와 4.4절에 표시만 했다.
- 네이티브 백엔드는 WF-2a가 firmware 모드를 더하는 중이다. 이번 측정은 L0·L1(test_servo)만 썼고, 각 run의 바이너리 SHA를 RUN.json에 남겼다.

## 11. 3단계 P7에서 더한 것 (2026-11-09)

- 장면 세계 W3(`world.kind: W3`): 통계 맞춤 절차 과수원(`steering/procedural_orchard.py`, `WORLDS_KO.md` 6절). 키는 `procedural`(seed와 생성 매개변수, 기본값을 채워 정규화)과 `modifiers`다. 에피소드 회랑은 `corridor_index` 통로(행 k와 k+1 사이, +x 진행)다. 출발 자세는 W2처럼 회랑 좌표(s, d, 방향 오차)다.
- W3 세계는 W2와 같은 `RasterCorridorWorld` 계약을 따른다(래스터 LiDAR, 접촉 평가, 회랑 좌표). 지도 파일이 없으므로 하네스 방식에서는 에피소드마다 생성기로 새 세계를 만든다(접촉 계수기가 섞이지 않게). gap·remove_trunks 수정자는 생성한 단위를 셀 단위로 지운다. 나머지 수정자는 `raster_corridor.apply_modifiers`를 그대로 쓴다.
- 세계 식별값: W3는 정규화한 생성 매개변수의 SHA-256(`procedural_params_sha256`)이다. 생성기 파일은 코드 식별 목록(`IDENTITY_FILES`)에 넣었다(`steering/procedural_orchard.py`, `field_world/{__init__,raster_ops,build_field_world,scene_spec}.py`). 그래서 이 판의 코드 식별 SHA는 이전 판과 다르다. trace 내용은 바뀌지 않는다.
- 프로필 `plant_tables`(A12 전용, 합격 기준 개정 3): 바퀴 쪽 등가각 |δ|별 조향 효율 η(|δ|)(선형 보간, 0°는 1)와 회전 중 속도 손실 loss(|δ|, v)(쌍선형, 속도는 표 범위로 자름) 표다. 주면 플랜트가 `TableSteeredPlant`가 된다. 이 플랜트는 회마다 뽑는 η·언더스티어·끌림 손실 대신 표를 쓰고 언더스티어는 0이다. `geometry_check`와 함께 쓸 수 없다.
- 기동 `schedule`: `knots [[t, 구동 duty %, 조향 deg], ...]`, `interpolation` linear 또는 hold, `end_s`. G-SIM 탐침(`gz_probe.ScriptSteerController`)이 같은 `maneuvers.schedule_value`로 같은 값을 쓴다.
- 기존 장면의 정규 JSON은 그대로다: W0·W2의 `world`에는 `procedural` 키가 없고, `plant_tables`가 없는 장면의 `profile`에는 그 키가 없다(시험으로 확인). 그래서 기존 장면 SHA와 trace가 바뀌지 않는다.
- 시험: `test_steering_worlds.py`(W3 직접·하네스 실행, 표 플랜트의 곡률·속도 손실, schedule). 결과는 `WORLDS_KO.md` 9절.
- 2026-11-09 리뷰 수정(`WORLDS_KO.md` 12절): W3 생성기가 단위를 서로 떼어 놓고(래스터에서 따로 덩어리가 되게) 셀 크기를 데이터 측정 규칙에 맞춘다. 그래서 같은 seed라도 W3 래스터와 trace는 이전 판과 다르다(W0·W2는 그대로). `row_length_m`는 10~120 m, crop 상한은 4096² 셀이다. holdout seed(100000~100999)는 `world.allow_holdout: true` 없이 거절한다(`world_w3_holdout_seed`). `allow_holdout`은 참일 때만 정규 JSON에 쓰므로 개발 W3 장면의 SHA는 그대로다.
