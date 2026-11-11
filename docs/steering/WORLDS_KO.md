# 조향 SIM 월드: field_world · field1010 · 자르기 노드 · 절차 과수원 (3단계 P7)

- 작성: 2026-11-09 (WF-3b p7). 설계 5.1~5.3, 8.3 P7, 합격 기준 A6·A7·A10(W2)·A12·A16(개정 3·6).
- 모든 수치는 자리값 기반 SIM 결과다. 현장 검증이 아니다. 10/10 지도로 만든 월드는 "SLAM이 본 과수원"이다.
- 봉인 3방향과 봉인 열 쌍의 역방향 2개는 만들지도, 돌리지도, 열지도 않았다(`scene_spec.sealed_status()`). 모든 검사는 개발 32방향(동결 8 + 봉인 밖 포함 방향)에서 했다.
- 이 문서의 결과 파일은 저장소 밖 `D:/Develop/UBTS/steering-sim-20261108/runs/wf3b/p7/`에 있다.
- 2026-11-09 리뷰(review_world RW-01~09)를 반영해 고쳤다. 고친 내용과 다시 잰 결과는 12절이다. 12절의 결과 파일은 `runs/wf3b/fix_wf3b/`에 있다. 앞 절의 처음 결과는 지우지 않는다.

## 0. 요약

1. 한 장면 명세(`field_world/scene_spec.py`)에서 래스터 사본(F-SIM 감지·접촉 모델, RASTER_V1)과 Gazebo SDF를 함께 만든다(`field_world/build_field_world.py`). 모드 R(래스터 동등)과 모드 T(나무 형상)를 만들었다. 모드 V(방문 일관)는 하지 않았다.
2. `gz_assemble.py`에 장면 `field1010`을 더했다. 고정 파일 `assemble_world.py`는 고치지 않았다. 새 메타데이터 스키마를 쓰고 정답은 `eval_truth.json`에만 둔다.
3. 자르기 노드 `steering/scan_unobserved_clip.py`(노드 `ubts_steer_scan_clip`): `/sim/scan`과 `/ground_truth/odom`을 받아 첫 UNOBSERVED 셀 뒤의 반사를 "반사 없음"으로 바꾸고, 자른 뒤 거리 잡음을 더해 `/sim/scan_clipped`로 낸다. field1010 장면에서 P6a 감독기가 켠다(12절).
4. 절차 과수원 W3 `steering/procedural_orchard.py`: 10/10 개발 회랑 통계(단위 간격·크기·옆 어긋남, 열 간격)에 맞춘 과수원이다. 래스터에서 데이터와 같은 규칙으로 잰 덩어리 통계가 데이터와 맞도록 단위를 떼어 놓고 셀 크기를 정한다(12절). 같은 명세에서 래스터와 SDF가 나온다. F-SIM 장면 세계 `W3`로 돈다.
5. 검사 결과는 9절 표에 모았다.

## 1. 월드 종류와 모드

| 기호 | 원천 | 래스터 사본 | SDF | 정답 |
|---|---|---|---|---|
| W2 | 단일열 카탈로그의 10/10 회랑 방향(`source: catalogue`) | F-SIM과 같은 crop(회랑 폴리라인 상자 + 35 m, 가장자리 UNOBSERVED)에 수정자 적용 | 모드 R / T | 카탈로그 라벨(SLAM 라벨) |
| W3 | 절차 과수원(`source: procedural`, seed) | 생성기가 직접 만든 클래스 격자(같은 crop 규칙) | 모드 R / T | 생성기 기하(정확) |

모드:

- **R(래스터 동등, A7 기준 월드)**: 래스터 사본의 반사 셀(OCCUPIED, ADDED_HARD, ADDED_BRANCH)을 모두 상자로 세운다. z는 0.3~0.9 m(`box_z_m`). 클래스마다 셀을 겹치지 않는 직사각형으로 합친다(한 행의 연속 구간을 다음 행의 같은 구간과 합침). 모든 상자는 래스터 축을 따른다(W에서 같은 yaw). 충돌은 기본 없음이다(접촉의 정본은 래스터 평가기, 설계 5.2). `collision: hard`면 ADDED_HARD만 충돌을 가진다.
- **T(나무 형상)**: OCCUPIED 덩어리(1셀 팽창 뒤 8연결 = 체비셰프 거리 3 이하 사슬)를 크기로 나눈다. 대각 ≤ 0.30 m는 줄기 후보 원기둥(무게중심, 반경 clamp((범위 − 0.05)/2, 0.03, 0.12) m, 0~0.9 m, `hard`면 충돌), 0.30~1.2 m는 수관 단면 셀 상자(0.3~0.9 m, 충돌 없음), 1.2 m 초과는 구조물 셀 상자(0~1.5 m, `hard`면 충돌). ADDED_HARD는 0~1.5 m(충돌), ADDED_BRANCH는 0.4~0.9 m(시각만). 선택 위쪽 수관(`upper_canopy`, SIM_ASSUMED)은 회랑 두 행을 따라 1 m마다 z 1.0~2.8 m, 폭 1.2 m 상자(시각만)다. `trunk_cells`(기본 켬)는 줄기 덩어리의 셀 상자도 남겨 0.6 m 평면이 모드 R과 같은 셀을 덮게 한다. 높이와 반경 규칙은 모두 SIM_ASSUMED다(지도에 높이가 없다).
- **V(방문 일관)**: 하지 않았다(선택 항목, 시간 부족).

## 2. 명세 형식 (`field_world/scene_spec.py`)

```json
{"schema": "ubts.steer_field_world.v1", "name": "f040927_c02_fwd_r", "source": "catalogue",
 "record": "040927", "corridor_id": "c02", "direction": "fwd", "procedural": null, "modifiers": [],
 "mode": "R", "crop": {"kind": "fsim"}, "box_z_m": [0.3, 0.9], "collision": "none",
 "trunk_cells": null, "upper_canopy": null, "ground": {"mu": null, "margin_m": 10.0, "thickness_m": 0.2},
 "start": {"s_m": 1.0, "d_m": 0.0, "heading_deg": 0.0}, "allow_sealed": false}
```

- 정규 JSON(기본값을 모두 채움, 키 정렬, ASCII, NaN 없음)의 SHA-256이 명세 식별값이다.
- `crop`: `fsim`(기본, F-SIM과 같은 래스터) 또는 `band`(`half_width_m` 12, `before_m` 8, `after_m` 15 기본). `band`는 띠 밖 셀을 래스터 사본에서 UNOBSERVED로 바꾼다(두 월드 모두 띠 끝에서 광선이 끝난다). 이 사본은 F-SIM W2 세계와 다르다(manifest `fsim_equivalent: false`).
- `modifiers`: `raster_corridor` 수정자 사전(gap, remove_trunks, branch_rect, branch_blob, obstacle_box, wall). F-SIM과 같은 코드로 래스터 사본에 적용한다(W2의 gap·remove_trunks는 scipy가 필요하다. W3는 생성 단위를 직접 지운다).
- `ground.mu` null이면 프로필 `ground_mu`다. `start`는 field1010 장면의 로버 출발 자세(회랑 좌표)다.
- 봉인 방향과 봉인 쌍 역방향은 `allow_sealed` 없이 거절한다(`field_sealed_corridor`).
- 절차 명세의 holdout seed(100000~100999)는 `allow_holdout` 없이 거절한다(`field_procedural_holdout_seed`, RW-06). `allow_holdout`은 참일 때만 정규 JSON에 쓴다(개발 명세의 SHA는 그대로다). F-SIM 장면 세계 W3(`scene_spec.WorldSpec`)와 `procedural_orchard.build_raster_world`도 같은 규칙이다.

## 3. 생성물과 좌표

`build_field_world.write(fw, out)`이 쓰는 것:

| 파일 | 내용 |
|---|---|
| `raster.npz` | 래스터 사본: 클래스(uint8, 0행 = 가장 낮은 y), 해상도 0.05 m, crop 오프셋, 래스터 국소 → W 변환, 명세 SHA. 자르기 노드와 F-SIM이 쓰는 SIM 참값(플랜트 쪽) |
| `field_world.sdf` | 단독 Gazebo 월드(물리·기본 시스템·조명·바닥·월드 모델, 로버 없음). Embree(`mesh_raycast`)가 그대로 읽는다 |
| `manifest.json` | 명세와 SHA, 입력 SHA(카탈로그, 지도 PGM·YAML), 생성기 파일 SHA, 변환(래스터 국소 미터만), 개수(셀·상자·원기둥·삼각형 추정·SDF 바이트), 출력 SHA, A16 lint 결과 |
| `eval_truth.json` | 평가 전용(실행기·추정기·제어기 입력 아님): 중심선(W), 두 행 선, 열 끝 s_E와 끝점, 행별 마지막 단위, 지지 단위(s, d), 벽 |

- 월드 좌표 W: 원점 = 회랑 시작점(s 0, d 0), +x = 시작 접선, +y 왼쪽, z 위. 래스터 국소 L(기록 래스터 왼쪽 아래 모서리 기준 미터)은 manifest 변환과 `raster.npz`에만 있다(`p_L = p0 + R(θ) p_W`).
- SDF의 구면 좌표 요소, GNSS 좌표, 지도 YAML 원점, 실제 기록 궤적은 어떤 생성물에도 없다. SDF 위치는 소수 5자리(10 µm)로 쓴다(A16 lint의 위경도 꼴 규칙에 걸리지 않게).
- 같은 명세는 같은 바이트를 낸다(시험 `test_mode_r_boxes_are_exactly_the_reflector_cells_and_outputs_are_deterministic`, A7 월드 재생성에서 SDF·래스터·자세 SHA 같음 확인).

## 4. 장면 `field1010` (`gz_assemble.py`)

```
python -B -m steering.gz_assemble --scene field1010 --out <folder> --record 040927 --corridor c02 [--direction fwd] [--mode T] [--no-camera]
python -B -m steering.gz_assemble --scene field1010 --out <folder> --field-spec <spec.json>
```

- 고정 `assemble_world.assemble()`를 preset `sensor_test`로 부른 뒤 나무·시험 장애물·끝 벽·생성 바닥을 지우고, field world의 바닥과 모델을 넣고, 로버를 명세 출발 자세(회랑 좌표 → W)에 둔다. 조향 모델 변환(`gz_model`)은 W0·W1과 같다(구 바퀴 충돌 기본).
- `world_metadata.json`은 새 스키마 `ubts.steer_field_scene_metadata.v1`이다(렌더러 값, 출발 자세, field world 폴더, 명세 SHA). 옛 `lane_centers_y` 같은 월드 정답은 없다. 정답은 `field_world/eval_truth.json`에만 있다.
- 월드 표지(RW-09): 장면 manifest와 `world_metadata.json`의 `world`는 명세 원천을 따른다. 카탈로그는 W2, 절차는 W3다(`gz_assemble.world_label`). 처음 판은 둘 다 W2로 적었다.
- field world 파일은 `<out>/field_world/`에 쓴다. 자르기 노드는 `<out>/field_world/raster.npz`를 읽는다.
- W3도 같은 장면으로 만든다(`source: procedural` 명세).
- `gz_probe`: `rtf --scene field1010 --field-spec <spec.json>`(A10 W2), 고정 그룹 `a12_w0`·`a12_w2`(A12).

## 5. 자르기 노드 (`steering/scan_unobserved_clip.py`)

- 규칙: RASTER_V1에서는 광선이 UNOBSERVED 셀(또는 래스터 밖)에 들어가면 반사 없이 끝난다. Gazebo에는 거기 물체가 없어 Embree가 지나가 뒤를 본다. 노드는 광선마다 첫 UNOBSERVED 셀까지의 거리를 래스터 사본에서 구하고, Embree 반사가 그 거리 이상이면 +inf(반사 없음)로 바꾼다. 꼭짓점 동점은 래스터 규칙대로 종료 셀이 이긴다(허용 1e-4 m). 센서 자신이 UNOBSERVED 셀에 있으면 모두 반사 없음이다.
- 광선 기하는 `cpu_lidar.py`와 같다: 원점 = 몸체 자세 + R × `rover.yaml` LiDAR 장착, 방향 = R × (cos a, sin a, 0). 래스터는 2D라 광선을 수평 투영으로 따라가고 거리는 r × |d_h|로 비교한다(평평한 스캔 평면에서 정확).
- 계산: 래스터 경계 셀(관측 셀과 붙은 UNOBSERVED 셀)까지의 최단 거리보다 짧은 반사는 DDA를 하지 않는다. 나머지 광선만 반사 거리까지 supercover DDA(`field_world.raster_ops.first_stop`, 참조 DDA와 셀·거리까지 같음, 시험)를 돈다.
- ROS 인터페이스(노드 `ubts_scan_unobserved_clip`):
  - 구독: `/sim/scan`(sensor_msgs/LaserScan, cpu_lidar 출력), `/ground_truth/odom`(nav_msgs/Odometry, 참값, 플랜트 쪽). 스캔의 자세는 같은 stamp의 odometry다(cpu_lidar가 그 자세의 stamp를 스캔에 찍는다).
  - 발행: `/sim/scan_clipped`(LaserScan: 입력의 header·각도·한계, 잘린 광선은 +inf). 다음 단계 S2 형식 노드가 이것을 받아 `/scan`을 낸다.
  - 상태: `--status-file`(기본 `/runtime/scan_clip_status.json`): 스캔 수, 발행 수, 자세 없는 스캔 수, 잘린 광선 수, 마지막 처리 ms. 자세를 못 찾은 스캔은 내보내지 않는다(잘리지 않은 스캔은 미관측 공간을 꿰뚫어 본다).
- 실행(SIM 컨테이너, 감독기 명령): `python3 -B -m steering.scan_unobserved_clip --input /sim/scan --output /sim/scan_clipped --raster <scene>/field_world/raster.npz --status-file /runtime/scan_clip_status.json --noise-sd SD --noise-seed N`. 처음 판은 노드가 `--raster`만 받고 감독기는 `--input/--output`만 넘겨 시작하지 못했다(RW-01). 순수 처리부는 `ClipProcessor`(자세 버퍼, stamp 짝 맞추기, 자르기, 잡음)이고 ROS 없이 시험한다.
- 거리 잡음(RW-04): F-SIM은 정확한 거리로 UNOBSERVED 종료를 정한 뒤 잡음을 더한다. cpu_lidar(고정 파일)는 잡음을 먼저 더한다. 그대로 두면 UNOBSERVED 셀 바로 앞 반사가 잘리거나 바로 뒤 반사가 남는다(A7 표본에서 σ 0.01일 때 약 0.03 %). 그래서 자르기 노드가 있는 장면에서 감독기는 cpu_lidar를 잡음 0인 rover.yaml 사본(`/runtime/steer/rover_lidar_no_noise.yaml`)으로 켜고, 자르기 노드가 자른 뒤 rover.yaml의 잡음(σ 0.01)을 ScanSynthesizer 규칙(유한 반사만, 최소 거리 고정, 최대 거리 넘으면 +inf)으로 더한다. F-SIM 래스터 LiDAR의 σ는 0.015로 값이 다르다(바꾸지 않음, 차이로 적음).
- 참값 격리(A13): 플랜트 쪽 역할이다(`truth_isolation` 패턴 표에 이미 있음). 실행기는 `/sim/*`, `/ground_truth/*`를 구독하지 않는다.

## 6. 절차 과수원 W3 (`steering/procedural_orchard.py`)

- 통계 원천: `field_world/orchard_stats.py`로 개발 회랑 22쌍(동결 8 + 봉인 밖 포함 방향에서 열 쌍마다 한 번)의 카탈로그 단위와 래스터를 쟀다(단위 3,853개 중 3,849개를 덩어리와 맞춤). 결과 `ORCHARD_STATS_DEV.json`(분위수만, 위치 없음).

| 통계 | 10/10 개발 회랑 | 생성기에서 |
|---|---|---|
| 단위 간격(m) | 중앙 0.60, q25 0.36, q75 0.95, q90 1.45, q99 4.05 | 역누적분포 매듭(꼬리 6 m 0.3 %, 최대 12 m) |
| 단위 크기(열 방향 / 가로, m) | 중앙 0.152 / 0.142, 상관 0.67 | 같은 매듭 + 가우스 코퓰라 상관 0.67 |
| 단위 옆 어긋남(m) | 중앙 0, q10 −0.26, q90 +0.23 | 같은 매듭 |
| 열 간격(m, 쌍 중앙) | 3.19~3.95, 중앙 3.75 | 통로마다 같은 분포에서 뽑음 |
| 2.5 m 초과 틈(100 m당) | 중앙 3.2, 평균 5.8 | 간격 꼬리에서 자연히 생김 |
| 중심선 곡률(1/m) | p95 0.017, 최대 0.039 | 행마다 저주파 사인 2개(파장 30~90 m, 진폭 0~0.25 m, SIM_ASSUMED) |

- 단위는 데이터처럼 스캔 평면의 점유 덩어리다(나무가 아니다). 작은 단위(대각 ≤ 0.30 m, 분위수로 보면 약 3분의 2: 대각 중앙 0.21 m, q75 0.34 m)가 모드 T의 줄기 후보가 된다. 규칙적 줄기 층(`tree_spacing_m`, `trunk_diameter_m`)은 선택이다(기본 꺼짐, SIM_ASSUMED).
- 매개변수: `seed`, `rows`(기본 4), `corridor_index`(기본 가운데), `row_length_m`(60), `row_spacing_m`(null = 분포), `field_yaw_deg`(null = seed로 0~180°), `bend_wavelength_m`, `bend_amplitude_m`, `end_stagger_m`(1.5), `observed_margin_m`(6), `headland_m`(15), `end_wall_m`(null), `tree_spacing_m`, `trunk_diameter_m`, `unit_scale`.
- 정답(`eval_truth`): 생성기 행 선, 중심선(두 행 선의 가운데), 시작(두 행이 모두 시작하는 곳), 끝(두 행의 마지막 단위 중 늦은 것: 카탈로그 fwd 규칙), 단위 목록(크기 포함).
- 래스터: 관측 영역(행 상자 + `observed_margin_m`) 안은 FREE, 단위 셀은 OCCUPIED, 밖은 UNOBSERVED, crop은 F-SIM 규칙(중심선 상자 + 35 m, 가장자리 UNOBSERVED).
- seed: 개발 0~999, holdout 100000~100999(`HOLDOUT_SEEDS`). holdout seed는 조향 사전등록 전에는 쓰지 않는다(제안, 10절).
- F-SIM: 장면 세계 `{"kind": "W3", "procedural": {...}, "modifiers": [...]}`. 직접 방식과 하네스 방식 모두 돈다(하네스는 에피소드마다 새 세계). 자세한 것은 `FAST_SIM_KO.md` 11절.
- 처음 판의 문장 "생성한 래스터의 덩어리 통계를 다시 재면 데이터와 비슷하다"는 틀렸다(RW-02). 시험은 생성 단위 목록의 간격만 봤고, 래스터에서는 단위 18 %가 합쳐지고 셀 상자가 작았다. 고친 생성기와 래스터 수준 비교는 12.2절이다.

## 7. 판정 정의 (첫 측정 전에 정한 것)

- **A7**(등록 기준, 개정 6): 개발 32방향 × 자세 6 × 3,200광선 = 614,400광선. 자세 규칙은 단일열 raycast_check와 같다(s ~ U(1, s_E − 1), d ~ U(−1, 1), 방향 오차 ~ U(−0.6, 0.6) rad, 새 seed 20261109). LiDAR 높이 0.6 m 수평. 참조 = `closed_loop/testdata/raster_world_ref_e71e27d4.py`의 supercover DDA(반사 셀 반사, UNOBSERVED·밖은 종료). G 쪽 = 모드 R SDF의 Embree 첫 반사(최소 거리 처리 없음) + 자르기 함수. 분모: 반사 여부 불일치는 전체 광선, 거리 차(> 0.071 m)는 둘 다 반사한 광선(더 엄격). 거리 차 비율은 전체 광선 기준도 적는다. 불일치마다 광선을 ±1e-6 rad 또는 원점을 ±1e-5 m 움직였을 때 참조가 G 결과와 같아지면 '스침'으로 분류한다.
- **A12**(등록 기준, 개정 3): 7.2절 ①②③대로다. 세부:
  - ① 맞춤: g_update의 A1(G) 힘 모드 결과(`A12_INPUT_G.json`). 판정 형식은 δ별 η(5/10/20°, |δ| 선형 보간, η(0) = 1, K = 0)다. 같은 A1(G) 실행의 회전 중 속도 손실 표(5/10/20° × 0.3/0.5 m/s)를 F-SIM 끌림 항으로 같게 둔다(② "같은 값으로 고정"). 등록 형식(상수 η 0.968, K_us −0.01346, 끌림 손실 최소제곱 상수 0.00371/°)은 보고한다.
  - ② 무작위 항 끔: 0점 오차, 보정 잔차, 백래시 떠돎, 좌우 비대칭, 곡률 떠돎, 옆 미끄럼, 속도 배율·요동. 두 SIM 모두 같은 프로필 view와 공유 L0 서보(백래시·잡음·데드밴드 없음)를 쓴다.
  - ③ 대본(`maneuvers` schedule, F-SIM 직접 방식의 명령 시각: 20 Hz, 지연 50 ms, MCU 램프 100 %/s, 소프트 한계 자름): W0 4개(사인 5° 0.4 m/s 36 s, 계단 ±10° 0.3 m/s 30 s, 원호 20° 0.3 m/s 20 s, 열 추종 비슷한 작은 조향 0.4 m/s 34 s), W2 모드 R 2개(040927 c02 fwd, s 1 m와 40 m에서 출발: 열 추종 비슷한 대본, 사인 대본).
  - 위치 차 = 대본 시간 동안 기준점(발자국 중심) 위치 차의 최대(시간 맞춤, G 표본을 F 시각에 보간; W0는 출발 기준, W2는 W). 방향 차 = |wrap(ψ_F − ψ_G)|의 최대.
  - 예측 차이 = 같은 대본의 F-SIM 두 실행 차이: 맞춤 그대로와, A1 잔차만큼 흔든 것(η × (1 ± 형식의 최대 곡률 잔차), 직진 속도 × (1 ± A4(G) 최대 정상 속도 오차 0.22 %), 큰 쪽). 합격: 위치 차 ≤ max(0.10 m, 1.5 × 예측), 방향 차 ≤ max(2°, 1.5 × 예측), 대본 전부.
  - 같은 거리에서 맞춘 위치 차(속도 차이를 뺀 경로 모양 차이)와 끝 자세 차도 적는다.
- **A10(W2 모드 R)**: 개정 2와 같다(240 s, 앞 20 s 제외, 5 s 창 Δsim/Δreal 중앙값, GUI 끔, 카메라 끔·켬 640×360, 렌더링 D3D12/Ogre1). 월드 040927 c02 fwd 모드 R(fsim crop), 직진 0.4 m/s. 등록 초안 기준 ≥ 0.8, 첫 측정 뒤 고정(개정 제안).
- **A16**: `field_world/coord_lint.py`의 세 규칙(위경도 꼴 숫자 쌍, 10/10 지도 YAML 원점 값, GNSS 자료 폴더 경로)이 0. GNSS 필드·토픽 낱말(제거용 코드가 이름을 부르는 경우)은 별도 보고 규칙이다.
- **폐합**: 모드 R SDF를 0.6 m 평면에서 래스터 사본 격자로 다시 래스터화해 반사 셀과 셀 단위로 비교(누락·초과 0), 카탈로그 지지 단위를 다시 찾음(사본과 같은 수), 열 끝 s_E(두 행 마지막 맞춘 단위 중 늦은 것)가 카탈로그 값과 0.011 m 안(단위 좌표 0.01 m 반올림), eval_truth 중심선이 카탈로그 폴리라인과 1 mm 안. 행 선 편차는 보고(회랑 좌표의 꼭짓점 투영 왕복 오차 때문).

## 8. 크기 (설계 5.2와 비교)

개발 32방향, `SIZE_TABLE.json`(최소 / 중앙 / 최대):

| 변형 | 상자 | 원기둥 | 삼각형(추정) | SDF 바이트 | 충돌 형상 | 생성 시간 s |
|---|---|---|---|---|---|---|
| R, fsim crop | 2,219 / 3,085 / 4,472 | 0 | 26.6k / 37.0k / 53.7k | 0.80 / 1.12 / 1.63 MB | 0 | 0.21 / 0.35 / 1.27 |
| R, ±12 m 띠 | 413 / 1,563 / 2,678 | 0 | 5.0k / 18.8k / 32.1k | 0.15 / 0.57 / 0.97 MB | 0 | 0.14 / 0.24 / 0.83 |
| T, fsim crop | 2,219 / 3,085 / 4,472 | 329 / 612 / 813 | 153k / 272k / 366k | 1.18 / 1.63 / 2.39 MB | 984 / 1,161 / 1,775 | 0.34 / 0.60 / 1.58 |
| T, ±12 m 띠 | 413 / 1,563 / 2,678 | 91 / 354 / 572 | 45k / 155k / 252k | 0.25 / 0.86 / 1.40 MB | 181 / 537 / 1,261 | 0.20 / 0.38 / 1.88 |

- 설계 5.2: ±12 m 띠 상자 495~2,761(삼각형 약 5.9k~33k), 지도 전체 상자 ≤ 10,359(삼각형 ≤ 124k). 모드 R은 띠와 fsim crop 모두 이 범위 안이다(띠 최소 413은 회랑이 다르기 때문). 모드 T의 삼각형은 원기둥(96각, 384개) 때문에 124k를 넘는다(fsim crop 최대 366k). 원본 상세 과수원 lite 768k보다는 작다.
- 모드 T 상자 수가 모드 R과 같은 것은 덩어리끼리 셀이 붙지 않아 직사각형이 덩어리를 넘지 않기 때문이다.
- 로드 시간·첫 스캔까지 시간: 9절 A7(Embree)·A10(Gazebo) 행.

## 9. 검사 결과

컨테이너 측정은 이미지 `ubts-steer-sim:wf3`(`01125b6287d8`)에 p7 소스 스냅숏을 읽기 전용으로 붙여 돌렸다. 스냅숏 = HEAD `0c1311e` + g_update 파일(wf3 이미지 상태) + p7 파일(`runs/wf3b/p7/snap_p7/SNAPSHOT.json`, 919파일, 결과의 provenance와 같은 파일 SHA). Embree 검사와 Python 3.12 시험은 2 CPU·2 GiB 컨테이너, Gazebo 검사(A12 G-SIM, A10)는 4 CPU·4 GiB·GPU 컨테이너에서 했다. A10은 앞 컨테이너가 끝난 뒤 시작했다. 실행 뒤에 바꾼 것은 `a12_twin.py` 모듈 설명의 CLI 줄과 이 문서·`FAST_SIM_KO.md`뿐이다(코드는 같음, AST 비교).

### 9.1 판정 검사

| 검사 | 결과 | 판정 |
|---|---|---|
| A7 (모드 R, 등록 기준) | 614,400광선(32방향 × 6자세 × 3,200). 반사 여부 불일치 5개(8.1e-6, 기준 ≤ 1e-3). 거리 차 > 0.071 m 6개(둘 다 반사한 379,153광선의 1.6e-5, 기준 ≤ 5e-3. 전체 광선 기준 9.8e-6). 불일치 11개 중 10개는 스침(작은 흔들림에 참조가 G 결과와 같아짐). 나머지 1개는 반사 셀과 UNOBSERVED 셀의 입구가 5 µm 떨어진 광선이다. 자르기의 동점 허용 1e-4 m가 UNOBSERVED를 골랐다. 잘린 광선 21,495개. | 합격 |
| A12 (등록 기준, 개정 3: δ별 η 형식) | W0 4대본 + W2 모드 R 2대본, 6개 모두 합격. 위치 차 최대 0.0054~0.0080 m(한계 0.10 m). 방향 차 최대 0.035~0.155°(한계 2°). 같은 거리에서 맞춘 위치 차 ≤ 0.0098 m. 예측 차이는 위치 0.012~0.035 m, 방향 0.02~0.32°라 한계는 모두 바닥값이다. W2 두 대본은 W0의 같은 대본과 같은 값이다(평평한 바닥, 같은 마찰, 충돌 없는 모드 R이라 같은 운동). | 합격 |
| A16 | 월드 생성물(개발 32방향 × 모드 R·T = 64폴더), 실행 결과(JSON·로그·G-SIM 출력), p7가 만들거나 고친 저장소 파일 22개: 세 규칙 모두 0(지도 YAML 원점 검사 켬). 실행 스냅숏 안의 HEAD 사본에는 1건이 있다(10절) | 합격 |

### 9.2 보고 검사

- **A12 등록 형식**(상수 η 0.968, K_us −0.01346, 끌림 0.00371/°): 6대본 중 4개가 한계 안이다. 사인 5°는 0.1045 m(한계 0.10 m), 원호 20°는 0.206 m·5.40°(한계 0.187 m·4.42°)로 넘는다. 상수 η로는 20° 근처 곡률을 못 맞춘다(맞춤 잔차 0.0226 대 δ별 3.3e-5). 개정 3에서 δ별 η를 허용한 이유와 같다.
- **A7 모드 T**(보고): 같은 614,400광선. 반사 여부 불일치 3,035개(0.49 %). 거리 차 > 0.071 m 6,378개(둘 다 반사한 광선의 1.68 %, 전체의 1.04 %). 스침은 20개, 나머지 9,393개는 스침이 아니다. 방향마다 처음 12개씩 남긴 예시 384개를 보면, 거리 차 270개 중 269개에서 G가 더 짧고(중앙 0.105 m), 반사 여부 불일치 114개는 모두 G만 반사했다. 줄기 원기둥(덩어리 무게중심, 반경 ≥ 0.03 m)이 셀 밖으로 나와 먼저 맞기 때문이다. 그래서 모드 T는 래스터와 같은 스캔을 내지 않는다. A7 기준 월드는 모드 R이다(설계대로). Embree 비용: 삼각형 153k~366k(중앙 272k), 로드 1.37~7.71 s(중앙 2.10 s), 첫 스캔 137~575 ms(중앙 300 ms), 로드 + 첫 스캔 중앙 2.36 s, 스캔 중앙 2.8 ms, 자르기 중앙 61 ms. 불일치 분류 때문에 전체 1,144 s가 걸렸다.
- **A6**(실제 스캔 비교, 기준선): 개발 22쌍 중 궤적 노드가 있는 11쌍, 660노드 × 3,240빔 = 2,138,400빔.
  - 래스터(F-SIM DDA): 반사 여부 일치 0.661, 실제 반사율 0.933, 합성 반사율 0.651, 실제만 반사 0.311, 합성만 반사 0.029. 둘 다 반사한 빔의 거리 차(실제 − 합성) 중앙 −0.032 m, |차| ≤ 0.05 m 12.0 %, ≤ 0.3 m 30.9 %, ≤ 1 m 42.4 %. 합성 반사가 두 행 띠 안인 빔: 실제 반사율 0.962, 거리 차 중앙 +0.063 m, |차| ≤ 0.1 m 26.0 %, ≤ 0.3 m 39.4 %. 쌍별 일치 0.567~0.768.
  - Embree(G-SIM 모드 R 월드 + 자르기 함수, SIM 이미지): 래스터와 사실상 같다. 일치 0.660632(래스터 0.660631), 합성 반사율 0.651153(0.651154), 거리 차 중앙 −0.032304 m(같음), 두 행 띠 실제 반사율 0.962482(0.962480). 쌍별 일치 차 최대 1.5e-5. 기준선과의 차(일치 1e-6, 중앙 0 m)는 회귀 허용(0.005, 0.005 m) 안이다. 85.8 s.
  - 해석: 지도는 이 스캔들로 만든 SLAM 지도라 자기 일관성 검사다. 합성 스캔은 지도 밖(UNOBSERVED)에서 끝나고 실제 스캔은 더 멀리 본다(실제만 반사 31 %). 거리 차 꼬리가 큰 것은 2D 평면 지도가 한 순간의 스캔과 다른 물체를 먼저 맞기 때문이다(잎, 여러 번 지나간 흔적의 합). 판정이 아니라 회귀 비교용 기준선이다(`A6_BASELINE.json`).
- **A10(W2 모드 R, 첫 측정)**: 040927 c02 fwd 모드 R(fsim crop, 상자 3,085개, SDF 1.14 MB), 직진 0.4 m/s, 240 s, GPU 렌더링(D3D12/Ogre1), 컨테이너 4 CPU·4 GiB. 카메라 끔: RTF 중앙 1.000, p10 0.986, 최소 0.628, 경로 93.4 m, `gz sim` 시작에서 첫 월드 통계까지 2.3 s. 카메라 켬(640×360): 중앙 0.998, p10 0.814, 최소 0.146, 카메라 28.1 fps(벽시계)·30.3 fps(SIM 시간), 경로 81.5 m(RTF가 떨어진 창이 있어 같은 240 s에 SIM 시간이 짧다), 첫 통계까지 4.4 s. 등록 초안(중앙 ≥ 0.8)은 두 경우 모두 넘는다. 고정 기준 제안은 중앙 ≥ 0.8 그대로다(측정 1회, LiDAR·자르기 노드 부하 없음). 같은 PC의 다른 작업 부하는 통제하지 않았다.
- **폐합**(32방향): 모드 R은 32개 모두 통과. 반사 셀 284,901개 중 누락 0, 초과 0. 지지 단위는 64개 행 모두 사본과 같은 수로 다시 찾았다. 열 끝 차 최대 0.005 m. 중심선 차 최대 0.07 mm. 행 선 차는 최대 0.0146 m(2방향이 1 mm 초과: 회랑 좌표의 꼭짓점 투영 왕복 오차, 보고). 모드 T(보고): 누락 0, 초과 18,846셀(6.6 %, 원기둥이 셀보다 큼), 64개 행 중 17개에서 단위 맞춤 수가 달라진다. 열 끝과 중심선은 모드 R과 같다.
- **Embree 비용**(A7 모드 R, 컨테이너 2 CPU, 3,200광선, 방향별 중앙값의 범위): 월드 로드 0.88~12.1 s(중앙 1.46 s), 첫 스캔 29~188 ms(중앙 47 ms), 로드 + 첫 스캔 중앙 1.54 s, 스캔 1.5~9.1 ms(중앙 2.4 ms). 자르기 함수 38~204 ms(중앙 69 ms). 040927의 4방향은 자르기가 104~204 ms로 느렸다(그중 3개는 로드도 5~12 s). 그 시간에 다른 작업의 이미지 빌드 예약이 있었으므로 부하가 겹쳤을 수 있다. 부하 없는 비용은 따로 재지 않았다. 자르기 중앙 69 ms는 10 Hz 스캔 주기 100 ms에 가깝다(10절).
- **크기**: 8절.

### 9.3 시험과 정적 검사

| 항목 | 결과 |
|---|---|
| Windows Python 3.13, pytest(p7 시험 2개 + 소유 시험 5개) | 171 통과, 1 건너뜀(113 s) |
| Windows, `test_steering_gz_model.py`(p6a 소유, gz_assemble·gz_probe를 씀) | 61 통과, 1 건너뜀 |
| WSL Python 3.10(pytest 없음, 시험 shim + unittest) | shim 156 통과, 5 건너뜀(하네스 방식 시험: WSL의 scipy·numpy로 하네스를 import할 수 없음), unittest(entry_feasibility, gz_model) 73 통과(1 건너뜀) |
| 이미지 Python 3.12, pytest(같은 7개 파일, 실행 스냅숏) | 171 통과, 1 건너뜀(259 s, 2 CPU) |
| A13 정적 검사(work tree) | 통과, 161파일, 오류 0 |
| `check_pins.py` | 고정 423개 중 바뀐 것 7개(모두 허용), 예상 밖 0. 결과는 FAIL: WF-3a의 `tools/row_executor/vendor/closed_loop`·`orchard_perception`이 모듈 이름을 다시 쓴다(p7 파일 아님) |
| 파일 위생(p7가 만든·고친 파일) | BOM 0, CR 0, 금지 낱말 0 |

## 10. 시험하지 않은 것과 한계

- 모드 V(방문 일관 서브맵 월드)는 만들지 않았다.
- 자르기 노드를 ROS 그래프 안에서(`cpu_lidar` → 노드 → S2 형식 노드) 돌린 것은 없다. 다음 단계(P6a 통합)에서 한다. 이번에는 같은 순수 함수를 A7·A6에서 Embree 결과에 썼다.
- G-SIM의 W2 장면에서 LiDAR를 켠 실시간 실행(cpu_lidar + 자르기 노드 부하 포함 RTF)은 재지 않았다. A10(W2)은 Gazebo 서버(카메라 끔·켬)만 쟀다.
- 모드 T의 줄기 원기둥은 작은 덩어리(1~2셀)보다 커서 0.6 m 평면에서 셀 밖으로 나온다(폐합 초과 셀). 모드 T의 스캔 동등성은 보고 전용이다.
- 지형(요철·두둑), 기울어진 스캔, 위쪽 수관이 카메라·기울어진 스캔에 주는 영향은 재지 않았다.
- W3 통계 맞춤은 분위수 매듭 표본이다. 생성 월드의 분포를 데이터와 통계 검정으로 비교하지는 않았다(시험은 넓은 범위 확인뿐).
- A6은 SLAM 지도 자기 일관성이다. 22쌍 중 11쌍에는 궤적 노드가 없다(차가 그 통로를 달리지 않았다: 카탈로그 `trajectory.nodes_inside` 0과 같음).
- 자르기 함수는 순수 Python + numpy DDA다. A7에서 스캔당 중앙 69 ms(최대 204 ms, 2 CPU 컨테이너)라 10 Hz에서 여유가 작다. ROS 노드로 쓰기 전에 더 빠르게 해야 할 수 있다(예: 셀마다 가장 가까운 종료 셀 거리를 미리 계산, 광선 묶음 DDA).
- A12는 개방 루프 대본만 비교했다. 폐루프 제어기, 잡음·무작위 항을 켠 비교, 경사·요철 지면은 하지 않았다.
- A16: 실행 스냅숏(`snap_p7`)은 HEAD 파일의 바이트 사본이다. 그 안의 `sim/ubts/field_response.py`(HEAD, p7 이전 커밋) 9행에 GNSS 자료 폴더 이름이 1건 있다(gnss_path 규칙). p7 파일이 아니라 고치지 않았다. HEAD 사본에는 GNSS 필드 낱말(보고 규칙)도 316건 있다. 같은 스냅숏의 HEAD 파일 12개에는 BOM이나 CRLF가 있다(저장소에 원래 있음).
- 실차, 현장 지면·타이어는 없다.

## 11. 파일과 다시 돌리기

- 저장소: `sim/ubts/field_world/`(`scene_spec.py`, `raster_ops.py`, `build_field_world.py`, `coord_lint.py`, `orchard_stats.py`, `real_scan_compare.py`, `a7_equivalence.py`, `a12_twin.py`, `closure.py`), `sim/ubts/steering/procedural_orchard.py`, `sim/ubts/steering/scan_unobserved_clip.py`, 고친 `gz_assemble.py`·`gz_probe.py`·`scene_spec.py`·`fast_sim.py`·`maneuvers.py`, 시험 `test_field_world.py`·`test_steering_worlds.py`(그리고 `test_steering_scene_spec.py` 한 줄).
- 다시 돌리기(sim/ubts에서, `UBTS_STEER_PATHS` 설정, A6는 `UBTS_SCANS_1010` 또는 `--scans-dir`):

```
python -B -m field_world.build_field_world --record 040927 --corridor c02 --mode R --out <folder>
python -B -m field_world.orchard_stats --out <stats.json>
python -B -m field_world.a7_equivalence prepare --out <worlds> --mode R
python3 -B -m field_world.a7_equivalence run --worlds <worlds> --out <a7.json>                 (SIM 이미지)
python -B -m field_world.closure <worlds>/f* --out <closure.json>
python -B -m field_world.real_scan_compare --out <a6.json> [--engine embree --worlds <worlds>]
python -B -m field_world.a12_twin fsim --a1-input <A12_INPUT_G.json> --out <fsim run>
python3 -B -m steering.gz_probe run a12_w0 a12_w2 --out <gsim>                                  (SIM 이미지)
python -B -m field_world.a12_twin compare --a1-input ... --fsim <fsim run> --gsim-w0 <gsim>/a12_w0 --gsim-w2 <gsim>/a12_w2 --out <a12.json>
python3 -B -m steering.gz_probe rtf --scene field1010 --field-spec <spec.json> --out <a10> --duration 240 --camera both   (SIM 이미지, GPU)
python -B -m field_world.coord_lint <folders and files>
```

## 12. 2026-11-09 리뷰 수정 (review_world RW-01~09, 작업 표지 fix_wf3b)

결과 파일은 저장소 밖 `D:/Develop/UBTS/steering-sim-20261108/runs/wf3b/fix_wf3b/`에 있다. 등록 기준은 바꾸지 않았다.

### 12.1 고친 것

| 지적 | 고친 것 | 파일 |
|---|---|---|
| RW-01 자르기 노드와 감독기 | 노드가 `--input`, `--output`, `--raster`, `--status-file`, `--noise-sd`, `--noise-seed`를 받는다. 같은 stamp의 자세보다 먼저 온 스캔은 자세를 기다리고, 잃은 자세는 앞뒤 자세로 보간한다(12.4절). 감독기가 field1010 장면(명세 `--field-spec`/`UBTS_STEER_FIELD_SPEC`)에서 노드를 이 인자로 켠다. 노드 처리부를 ROS 없는 `ClipProcessor`로 떼어 시험한다(인자 해석, stamp 짝, 자세 없는 스캔은 내지 않음). 노드 이름을 그래프 규칙과 같은 `ubts_steer_scan_clip`으로 바꾸고 참값 구독 허용 목록에 넣었다. `/sim/scan_clipped`는 S2 노드만 구독할 수 있다(R3) | `scan_unobserved_clip.py`, `supervisor_steer.py`, `p6a_graph.py` |
| RW-02 W3 래스터 통계 | 단위를 따로 덩어리가 되게 떼어 놓는다(먼저 놓인 단위 셀과 체비셰프 4셀 이상). 닿는 단위는 열 방향 위치(간격 통계)는 그대로 두고 옆으로만 옮긴다: 이전 단위와 0.25 m 안이면 데이터의 가까운 쌍 옆 차이(|Δd|) 분포 × 0.8로 옆에 놓고, 아니면 가장 작은 옆 이동. 첫 옆 어긋남은 이웃이 0.5 m 넘게 떨어진 데이터 단위의 분포에서 뽑는다. 셀은 열 방향 사각형 안 셀 중심이고, 측정 규칙(셀 중심 투영의 최대 − 최소 + 0.05 m)으로 잰 크기가 뽑은 크기에 가장 가깝게 반 크기를 단위마다 고른다(5 × 5 후보). 래스터 수준 통계를 데이터와 같은 규칙으로 재는 `orchard_stats.raster_summary`와 비교 시험을 더했다 | `procedural_orchard.py`, `field_world/orchard_stats.py`, `test_steering_worlds.py` |
| RW-03 상한 | `build_field_world.LIMITS`(모드별 상자·원기둥·삼각형·SDF 바이트·충돌 형상·crop 셀), `TIME_LIMITS`(Embree 로드, 첫 스캔, 합, Gazebo 첫 통계). `write()`가 넘는 월드를 거절하고(일부러 쓸 때 `allow_over_limits`) manifest에 보고를 넣는다. A7 실행이 월드마다 시간 상한을 비교한다. 값은 첫 P7 측정 뒤에 정했으므로 그 측정은 근거이고 판정이 아니다 | `build_field_world.py`, `a7_equivalence.py`, `test_field_world.py` |
| RW-04 잡음 순서 | 스택에서 cpu_lidar 잡음을 0으로 하고 자르기 노드가 자른 뒤 잡음을 더한다(5절). A7 결과에 잡음 변형 보고를 더했다 | `scan_unobserved_clip.py`, `supervisor_steer.py`, `a7_equivalence.py` |
| RW-05 긴 열 | `row_length_m` 10~120 m(데이터 23.8~106.9 m), crop 상한 4096² = 16,777,216셀(16 MiB). 120 m 열은 모든 yaw에서 상한 안이다(시험: 15° 간격) | `procedural_orchard.py` |
| RW-06 holdout seed | `allow_holdout` 없이 holdout seed를 거절한다(생성기, F-SIM 장면 W3, field world 명세) | `procedural_orchard.py`, `steering/scene_spec.py`, `field_world/scene_spec.py`, `fast_sim.py`, `build_field_world.py` |
| RW-07 lint | 실행 출력용 엄격 규칙 `latlon_pair_strict`: 소수 4자리 이상 숫자 중 두 띠에 하나씩 있는 모든 쌍(이웃 숫자만이 아님), 200자 창. `--strict`(또는 `strict=True`)일 때 판정한다. 월드 생성물(SDF, manifest)은 W·L 좌표가 소수 5자리라 띠 쌍이 될 수 있어 기존 판정 규칙을 쓴다 | `coord_lint.py` |
| RW-08 A10(W2) 부하 | P6a 스택(cpu_lidar + 자르기 노드 + 센서 계약 + 카메라)에서 field1010을 쟀다(12.4절) | 실행 |
| RW-09 W3 표지 | 절차 명세의 field1010 장면은 W3로 적는다 | `gz_assemble.py` |

### 12.2 W3 래스터 수준 통계 (RW-02)

같은 측정 규칙(`orchard_stats.raster_summary`: 행 띠 덩어리 = 1셀 팽창 뒤 8연결, 단위 맞춤 0.06 m, 덩어리 무게중심이 단위 s 범위 안)으로 잰 값이다. 데이터는 개발 22쌍, W3는 개발 seed 0~11 기본 매개변수다(`logs/ORCHARD_STATS_RASTER.json`). 고치기 전 W3 값은 리뷰 측정(`runs/wf3b/review_world/orchard_compare.json`, `verify_repro/w3_clump_stats.json`)이다.

| 통계 | 데이터 | W3 고치기 전 | W3 고친 뒤 |
|---|---|---|---|
| 단위가 따로 덩어리인 비율 | 0.999 | 0.821 | 0.987 |
| 덩어리 100 m당 | 131.5 | 106.5 | 126.7 |
| 덩어리 간격 q10 / q25 / q50 / q75 / q90 (m) | 0.168 / 0.334 / 0.571 / 0.918 / 1.394 | 0.319 / 0.477 / 0.728 / 1.090 / 1.648 | 0.193 / 0.351 / 0.595 / 0.926 / 1.434 |
| 1셀 비율(맞춘 단위) | 0.164 | 0.27~0.28 | 0.146 |
| 덩어리 대각 q10 / q25 / q50 / q75 / q90 (m) | 0.071 / 0.116 / 0.214 / 0.335 / 0.551 | 0.071 / 0.071 / 0.193 / 0.365 / 0.643 | 0.071 / 0.137 / 0.219 / 0.351 / 0.555 |
| 옆 어긋남 q05 / q10 / q25 / q50 / q75 / q90 / q95 (m) | −0.427 / −0.256 / −0.051 / 0 / 0.045 / 0.226 / 0.391 | (잼 안 함) | −0.431 / −0.286 / −0.044 / 0.001 / 0.040 / 0.267 / 0.458 |
| 가까운 쌍(간격 0.3 m 미만) 옆 차이 q10 / q50 / q90 (m) | 0.181 / 0.415 / 0.712 | (잼 안 함) | 0.176 / 0.416 / 0.687 |

- 덩어리 간격·밀도·1셀 비율·대각이 데이터에 가까워졌다. 옆 어긋남의 꼬리(q10, q90, q95)는 아직 데이터보다 0.03~0.07 m 넓다. 가까운 단위를 옆으로 떼어 놓는 규칙(SIM_ASSUMED, 0.25 m와 0.8배)은 이 표를 보고 골랐다. 통계 검정은 하지 않았다.
- 시험 `test_procedural_raster_matches_the_data_raster_statistics`(seed 0~5): 따로 덩어리 비율 ≥ 0.95, 덩어리 밀도 ±10 %, 간격 q10 ±0.06 m, q50·q90 ±15 %, 1셀 비율 0.10~0.22, 대각 q25~q90 ±25 %, 옆 어긋남 q10·q90 ±0.08 m, q25·q75 ±0.03 m, 가까운 쌍 옆 차이 중앙 ±0.08 m. 고치기 전 생성기는 따로 덩어리 비율(0.82), 밀도(−19 %), 간격 q50(+27 %), 1셀 비율(0.27)에서 실패한다.
- 같은 seed의 W3 세계(래스터, 단위, eval_truth)는 이전 판과 다르다. 이전 W3 결과와 섞지 않는다.

### 12.3 A7 다시 돌림 (잡음 변형, 시간 상한)

같은 p7 월드(개발 32방향, 모드 R)를 고친 소스(스냅숏)로 다시 돌렸다(이미지 `ubts-steer-sim:p6a`, 컨테이너 2 CPU·2 GiB, 93.2 s, `a7/a7_result_R.json`).

- 판정 수치는 p7과 같다: 614,400광선, 반사 여부 불일치 5개(8.1e-6), 거리 차 6개(둘 다 반사한 광선의 1.6e-5), 스침 10, 아님 1, 잘린 광선 21,495개. 합격.
- 잡음 변형(보고, RW-04): 반사 여부 불일치가 σ 0.01에서 잡음 뒤 자르기 178개(2.9e-4), 자른 뒤 잡음 5개(잡음 없는 비교와 같음). σ 0.015에서 293개(4.8e-4)와 5개. 스택은 이제 자른 뒤 잡음 순서다.
- 시간 상한(RW-03): 32월드 모두 상한 안. 로드 최대 2.1 s(중앙 1.17 s), 첫 스캔 최대 138 ms, 합 최대 2.2 s. 자르기 함수 스캔당 중앙값 32~84 ms(월드별 중앙값, 그 중앙 48 ms).

### 12.4 A10(W2) P6a 스택 (RW-08)

P6a 스택(실차 시계, GUI 없음, 카메라 640×360, cpu_lidar + 자르기 노드 + 센서 계약 + 에뮬레이터·브리지 + 녹화기·대시보드)에서 field1010 장면(040927 c02 정방향, 모드 R)을 켰다. 묶음 D(스냅숏 v2)와 G(v3)다. 실행 조건과 다른 검사는 `P6A_HEADLESS_KO.md` 9.9절이다.

| 묶음 | 정의 | 중앙 | 하위 10 % | 최소 | 전체 | 0.9 아래 창 |
|---|---|---|---|---|---|---|
| D | 240 s, 앞 20 s 제외, 5 s 창 43개 | 0.981 | 0.606 | 0.410 | 0.919 | 8 |
| G | 90 s, 앞 10 s 제외, 15개 | 0.343 | 0.280 | 0.262 | 0.419 | 15 |

- D: 0.9 아래 창 8개는 한 덩어리(약 40 s)였다. 원인을 찾지 못했다(같은 시간 이 작업은 짧은 단위 시험만 돌렸다. 다른 작업 흐름의 부하일 수 있다). 다시 재지 않았다. 토픽 율(SIM 초당): 카메라 30.28, IMU 33.34, GPS 4.80, 조향 상태 20.00, cpu_lidar 10.02.
- G는 호스트 CPU 99 %(다른 작업 흐름)에서 돌아 RTF를 비교하지 않는다. 자르기 노드의 자세 기다림 확인용이다.
- W1(같은 스택, 묶음 C)은 중앙 0.99997, 최소 0.965였다. W2는 cpu_lidar(상자 약 1만 개 Embree)와 자르기 노드가 더해진다.
- 판정: A10(W2)는 "P7 첫 측정, 보고"다. D의 중앙값 0.98은 0.9를 넘지만 창 8개가 0.9 아래다. 실차 시계 시간 민감 검사를 W2에서 돌리면 그 창은 무효다(A10).

자르기 노드(같은 실행):

| 묶음 | 받은 스캔 | 낸 스캔 | 늦은 자세로 낸 것 | 보간 자세로 낸 것 | 버림 | 스캔당 평균 처리 |
|---|---|---|---|---|---|---|
| D(v2) | 2,211 | 2,136 (96.6 %) | (기능 없음) | (기능 없음) | 75 (3.4 %) | 65.7 ms |
| G(v3) | 383 | 370 (96.6 %) | 133 | (기능 없음) | 13 (3.4 %) | 208 ms(호스트 99 %) |

- D의 버림은 실행기가 같은 stamp의 자세보다 스캔을 먼저 꺼낸 경우다. v3은 스캔이 자세를 기다린다: G에서 133개를 늦게 온 자세로 냈다(v2라면 버렸을 것).
- G에 남은 버림 13개는 더 새 자세가 먼저 온 경우(자세 토픽은 best effort, 호스트 부하에서 자세를 잃음)로 본다. v3 계수는 이 경우와 8개 초과를 가르지 않는다(v5는 보간 수를 따로 센다). 고침(스택 미시험): 잃은 자세를 앞뒤 자세(50 ms 이내) 사이로 보간한다(위치 선형, 쿼터니언 정규화 선형, `interpolated`로 셈). 단위 시험만 했다.
- 스캔당 처리는 넘파이 DDA(래스터 1327 × 2699)다. 호스트가 바쁘면 스캔 주기(SIM 0.1 s)를 넘을 수 있다(G 평균 208 ms, RTF 0.34라 벽시계 주기 약 0.29 s 안).

### 12.5 상한 값 (RW-03)

| 항목 | 모드 R | 모드 T | 근거 |
|---|---|---|---|
| 상자 | ≤ 10,359 | ≤ 10,359 | 설계 5.2 지도 전체 상자 |
| 원기둥 | 0 | ≤ 2,000 | 첫 측정 최대 813 |
| 삼각형(추정) | ≤ 124,320 | ≤ 768,000 | R: 설계 5.2(약 124k). T: G-SIM이 이미 돌리는 원본 과수원 lite |
| SDF 바이트 | ≤ 4,000,000 | ≤ 8,000,000 | 상자당 약 365 B(측정) × 상자 상한 |
| 충돌 형상 | ≤ 10,359 | ≤ 12,359 | 상자 + 원기둥 상한 |
| crop 셀 | ≤ 16,777,216 | 같음 | 4096²(16 MiB) |
| Embree 로드 | ≤ 30 s | 같음 | 첫 측정 최대 12.1 s(부하 겹침) |
| 첫 스캔 | ≤ 1.0 s | 같음 | 첫 측정 최대 0.58 s(모드 T) |
| 로드 + 첫 스캔 | ≤ 30 s | 같음 | |
| Gazebo 시작 → 첫 통계 | ≤ 60 s | 같음 | 첫 측정 4.4 s. 감독기 준비 한도(gz 180 s)보다 작게 |

- 첫 측정(p7) 뒤에 정한 값이다. 그래서 p7 측정은 상한의 근거이고 판정이 아니다. 등록은 통합 담당의 일이다(제안).

### 12.6 시험하지 않은 것 (12절 수정 뒤)

- W3의 통계 일치는 분위수 비교이고 통계 검정은 아니다. 옆 어긋남 꼬리는 여전히 넓다.
- 모드 T의 상한은 첫 측정과 G-SIM W1 크기에서 정했다. 모드 T 월드의 Gazebo 실시간 비율은 재지 않았다.
- 자르기 노드의 자세 보간(스냅숏 v5)은 단위 시험만 했다. 스택에서 다시 재지 않았다. W2 RTF는 조용한 호스트에서 다시 재야 한다(D의 40 s 내림, G는 호스트 포화).
