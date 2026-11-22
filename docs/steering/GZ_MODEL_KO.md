# Gazebo 조향 모델 (P5)

- 작성: 2026-11-09. 설계 8.3 P5, 3.2·3.5·3.6·3.7·4.3·5.1 구현 기록이다.
- 코드: `sim/ubts/steering/gz_model.py`(SDF 변환), `gz_assemble.py`(장면), `gz_probe.py`(헤드리스 실행·합격 측정), 시험 `sim/ubts/test_steering_gz_model.py`, 이미지 `docker/Dockerfile.ubts-steer`.
- 모든 값은 자리값 프로필(`front_steer_v0_placeholder`, SHA-256 `561681518c2d…`)에서 온다. 결과는 "자리값 기반 SIM 결과"다. 현장 검증이 아니다.
- 고정 파일(`assemble_world.py`, `generate_world.py`, `rover.yaml` 등)은 고치지 않았다. 생성 결과를 받아 변환한다(`verify_wheel_friction.build_fixture`와 같은 방식).
- 2026-11-09 리뷰 고침(작업 표지 fix_wf2b, 11절): 차 전체 질량을 프로필 30 kg에 맞춤(F6), 정지 거리 시작 시각(F4), 출발 방향 하나인 묶음의 검사(F5), fdir1 진단 묶음(F3), 결과 출처 기록(F2), 센서 장착 차이 거부(F7), 이미지에 펌웨어 소스(F8), 원 로그 BOM·CRLF(F9). 바퀴 충돌 모양(F1)은 선택지(`wheel_collision`)를 넣고 측정했으며 기본값은 그대로다(결정 제안).
- 2026-11-09 3단계 갱신(작업 표지 g_update, 12절): 바퀴 충돌 기본을 구로 바꿨다(결정 R13-1). 원통은 선택지(`wheel_collision=cylinder`)로 남는다. 새 이미지 `ubts-steer-sim:wf3`를 만들고, 프로필 v0.1(기본, R14)과 구 충돌로 A1(G)·A4(G)·A21(G)·A25·A10(W0)·W1 주행을 다시 쟀다. 1~11절의 수치는 프로필 v0와 원통 충돌의 이력이다.
- 2026-11-09 P6a(작업 표지 p6a): 이 모델과 장면(w0_force, w1_force)을 헤드리스 전체 스택(ESP 에뮬레이터, 센서 계약, 브리지 래퍼, 대시보드)으로 돌렸다. `docker/Dockerfile.ubts-steer`의 `IMAGE_TAG_HINT` 기본값을 `ubts-steer-sim:p6a`로 바꾸고, P6a 모듈의 import 확인을 빌드 단계에 더했다. 새 이미지는 `ubts-steer-sim:p6a`다. 모델 SDF·토픽은 바꾸지 않았다. 기록은 `P6A_HEADLESS_KO.md`.

## 1. 모델 구조

`generate_world.generate()`가 만든 4륜 스키드 모델 `ubts_rover`를 앞바퀴 조향 모델로 바꾼다.

```
base_link
 ├─ front_axle_roll_joint (revolute, 축 x, ±8°, 감쇠만, 스프링 없음) → front_axle_pivot_link   [기본 켬, 2026-11-09 p5_demo]
 │    ├─ left_front_steering_joint  (revolute, 축 z, 바퀴별 기계 한계) → left_front_steering_link
 │    │    └─ left_front_wheel_joint  (revolute, 축 y)                 → left_front_wheel
 │    └─ right_front_steering_joint …                                   → right_front_steering_link
 │         └─ right_front_wheel_joint                                    → right_front_wheel
 ├─ left_rear_wheel_joint   → left_rear_wheel
 ├─ right_rear_wheel_joint  → right_rear_wheel
 └─ laser_fixed, imu_link_fixed, camera_link_fixed (그대로)
```

- 2026-11-09 관절 직접 조종 시연(p5_demo)에서 찾은 결함 두 가지를 모델에서 고쳤다(자세한 수치는 `GZ_DEMO_KO.md` 6절).
  - 차체 상자와 바퀴가 겹쳤다(뒷바퀴 위쪽 5 cm, 조향한 앞바퀴 최대 7 cm). 차체를 바퀴 홈이 있는 상자들로 나눴다(아래 2절 표 "차체").
  - 앞차축이 단단히 붙어 있어 원본 과수원(W1) 요철에서 바퀴 하나가 자주 떴다(주행 시간의 63~86 %). duty 0.48·0.8에서 멈췄다. 앞차축 롤 피벗(`front_axle_pivot`, 기본 켬)을 넣었다. 끄면(`front_axle_pivot=False`) 옛 단단한 모델이다.

- 그대로 둔 것: 모델 이름 `ubts_rover`, 모델 원점(`base_footprint`, 지면 높이), 링크 `base_link`·`laser`·`imu_link`·`camera_link`의 이름과 자세, 바퀴 관절 이름 `*_wheel_joint` 4개, `OdometryPublisher`(참값), `JointStatePublisher`(참값).
- 지운 것: `DiffDrive`(바퀴 오도메트리는 이상적 엔코더다). 그래서 `/sim/cmd_vel`, `/sim/wheel_odom`, `/sim/wheel_tf`가 없다.
- 조향 관절 이름은 `_wheel_joint`로 끝나지 않는다.
- `axle_pivot` 형식이면 앞차축 링크 `front_axle_steering_link` 하나와 관절 `front_axle_steering_joint` 하나를 둔다. 두 앞바퀴가 그 링크에 붙는다. 한계는 방향마다 두 바퀴 한계 중 작은 값이다.
- 조향 링크는 킹핀 위치(앞차축 x, ±킹핀 폭/2, 앞바퀴 반경)에 있다. 바퀴 중심은 (앞차축 x, ±윤거/2)에 있어서 바퀴는 킹핀을 중심으로 돈다(스크럽 반경 = (윤거 − 킹핀 폭)/2, 자리값 0.05 m).
- 캐스터·킹핀 경사가 0이 아니면 조향 축을 기울인다(위쪽이 뒤로, 바퀴 쪽은 차 안쪽으로). 자리값은 둘 다 0이라 축은 (0, 0, 1)이다.
- 바퀴 위치·반경·폭·질량·관성, 차체 질량·무게중심·상자는 프로필 값으로 다시 쓴다. 자리값 프로필은 `rover.yaml`과 같은 값이라 위치는 바뀌지 않았다(변환 manifest의 `moved_m` = 0).
- LiDAR·카메라 링크 위치와 프로필 값이 다르면 변환을 거부한다(`SteerModelError: sensor_mount_not_applied`). 링크는 옮기지 않는다(Embree LiDAR가 고정 `rover.yaml`의 장착값을 쓰기 때문이다). `allow_sensor_mount_mismatch=True`면 만들되 manifest `result_flags`에 `sensor_mount_not_applied:<링크>`를 적고, LiDAR가 다르면 `lidar_imu_relative_pose_not_the_profile`도 적는다(IMU 센서는 프로필 위치로 옮기므로 LiDAR–IMU 관계가 프로필과 달라진다). probe `result.json`·시연 `run.json`이 이 표시를 `model_result_flags`로 옮긴다. 2026-11-09 고침(리뷰 F7): 처음에는 차이를 manifest에만 적고 조용히 rover.yaml 값을 썼다. 자리값에서는 차이가 0이다.
- 차 질량(2026-11-09 고침, 리뷰 F6): `base_link` 질량 = 프로필 `mass_kg`(바퀴 제외 차체 질량, 축별 체중계로 잴 값) − 다른 바퀴 아닌 링크(조향 링크 2 × 0.5, 앞차축 피벗 0.5, 현가 링크(켤 때), 생성 센서 링크 3 × 0.19 kg). 그래서 링크 질량 합이 `drive.vehicle_mass_kg` = 30 kg(F-SIM과 같음)이다. `base_link` 무게중심은 바퀴 아닌 링크 전체가 프로필 무게중심(`cog_x_in_base_m`, 0, `cog_z_m`)에 오도록 둔다. 관성은 envelope 상자를 `base_link` 질량으로 계산한다. 자리값에서 `base_link` 21.93 kg. 변환 끝에 링크 질량 합과 무게중심을 검사한다(manifest `mass_check`, 다르면 오류). 처음에는 `base_link`에 24 kg을 그대로 두어 전체가 32.07 kg(7 % 무거움)이었다.
- 바퀴 충돌 모양(옵션 `wheel_collision`, 2026-11-09 더함, 리뷰 F1): `sphere`(바퀴 반경의 구, 시각 요소는 원통 그대로, 충돌 자세·마찰·fdir1 그대로) 또는 `cylinder`(생성 모양). 처음에는 원통이 기본이었다. 3단계 첫 작업에서 결정 R13-1대로 구를 기본으로 바꿨다(`gz_model.DEFAULT_WHEEL_COLLISION`, 12절). 옵션을 주지 않으면 구, `wheel_collision=cylinder`면 원통이다. manifest의 `wheel_collision`(모양)과 `wheel_collision_choice`(모양, 기본·옵션 출처, 시각 요소 원통, 결정 번호)에 남는다. 바퀴 홈 검사(`wheel_body_clearance`)는 충돌이 구면 시각 원통을 쓴다(한 모델 안 링크는 부딪히지 않는다). 구는 실차 타이어를 잰 값이 아니라 SIM 모델 선택이다.

## 2. 플러그인과 매개변수

| 항목 | 값(자리값) | 출처 |
|---|---|---|
| 조향 관절 한계 | 바퀴별·방향별 ±28° | `steer_mech_limit_deg`(SPEC_PENDING). 공유 구동기 모델과 같은 값 |
| 조향 관절 effort 한계 | 1e6 N·m | 자리값 `steer_joint_effort_limit_nm`(SIM_NUMERIC). gz-physics(DART)는 effort 한계가 없는 관절의 속도 명령이 위치 한계를 지키지 않는다고 경고한다(실측: `JointFeatures.cc:175`) |
| 조향 제어 | `JointPositionController`, `use_velocity_commands` true, `cmd_max` ±5 rad/s | `gz_steer_joint_cmd_max_rad_s`(SIM_NUMERIC). gz-sim 8.15는 매 스텝 −오차/dt를 `cmd_max`로 잘라 속도로 낸다. PID 이득(p 1, i 0, d 0)은 이 모드에서 쓰이지 않는다(자리값, SIM_NUMERIC) |
| 조향 링크 | 0.5 kg, 원통 r 0.03 m × 0.10 m, 관성 하한 1e-3 kg·m², 충돌 없음 | `steer_link_mass_kg`(SIM_NUMERIC), 모양·하한은 자리값(SIM_NUMERIC) |
| 구동 제어(기본 `force_dc`) | 구동 바퀴마다 `JointController` 힘 모드. p 0.2390 N·m/(rad/s), i·d 0, `cmd_max` 1.6 N·m, `cmd_min` −1.6 N·m(프로필 v0 값. v0.1은 τ_stall 4.4 N·m라 p 0.6572, `cmd_max` ±4.4 N·m, 12절) | 설계 3.6 DC 직선. `drive.dc_motor_line()`: p = τ_stall/ω0, τ_stall = `drive_tau_stall_nm`(SPEC_PENDING), ω0 = 6.6946 rad/s(DERIVED, `P_fit1003` 직선, r 0.14 m) |
| 구동 속도 목표 | duty × ω0 (rad/s) | 설계 3.6 |
| 데드밴드 | 구동 관절 쿨롱 마찰 0.0773 N·m = d0 × τ_stall, d0 = 0.0483(v0.1: 0.2127 N·m) | `drive_deadband_frac`(DERIVED) |
| 안정 수 | p·dt/J = 0.1355 (< 1)(v0.1: 0.3726) | J = 0.0147 kg·m²(`wheel_inertia_kgm2`, DERIVED), dt 1/120 s |
| 구동 바퀴 | 4륜(좌·우 같은 duty) | `drive_wheels` 4wd_lr_pairs(SPEC_PENDING), `drive_lr_mode` equal_duty |
| 기구학 모드(`kinematic_velocity`) | 바퀴 4개 `JointController` 속도 모드. 바퀴별 기구학 속도를 보낸다 | `gz_model.kinematic_wheel_speeds()` |
| 교차 확인(`ackermann`) | 표준 `AckermannSteering`(뒤 2륜 구동, 조향 관절 2개, `steer_p_gain` 1.0, `steering_limit` 28°) | 비교 전용(설계 위험 R5). 오도메트리는 바퀴 오도메트리라 `/sim/truth/ackermann_wheel_odom`에만 낸다 |
| 바퀴 마찰 | mu(옆) 0.9, mu2(앞뒤) 0.9, `fdir1` 0 0 1 | `wheel_mu_lat`, `wheel_mu_long`(SIM_ASSUMED). DART는 두 면의 최솟값을 쓴다. 진단용 모델 옵션 `wheel_mu_lat`/`wheel_mu_long`이 이 차만 덮어쓴다(manifest `wheel_friction.source` model_option) |
| 바퀴 충돌 | r 0.14 m 구(기본, 2026-11-09 결정 R13-1). 옵션 `wheel_collision=cylinder`면 원통 r 0.14 m × 0.12 m(생성 모양). 시각 요소는 둘 다 원통 | SIM 모델 선택. 8절, 12절 |
| 바퀴 | r 0.14 m, 폭 0.12 m, 1.5 kg, 굴림 축 관성 0.0147 kg·m² | 프로필 A·B표 |
| 차체 | 프로필 24 kg = `base_link` 21.93 kg + 조향 링크 1.0 + 피벗 0.5 + 센서 링크 0.57 kg. 바퀴 아닌 링크 전체의 무게중심 (0, 0, 0.32) m. 상자 1.10 × 0.54 × 0.24 m(관성은 이 상자, `base_link` 질량). 차 전체 30 kg | `mass_kg`, `cog_*`, `gz_body_box_m`(SIM_ASSUMED). `drive.vehicle_mass_kg` |
| 차체 바퀴 홈 | 같은 외형을 상자 5개로 나눈다: 위 판(바퀴 꼭대기 + 피벗 들림 + 틈 위, 1.10 × 0.54 × 0.103 m)과 아래 조각 4개(앞 조각 폭 0.28 m, 뒤 바퀴 사이 0.42 m, 가운데 0.54 m). 모든 바퀴에서 조향 범위·피벗 범위 전체에 틈 약 1 cm(측정 최소 0.0093 m) | `body_wheel_clearance_m` 0.01 m(SIM_ASSUMED). `gz_model.body_boxes()`, 검사 `wheel_body_clearance()` |
| 앞차축 롤 피벗(기본 켬) | `front_axle_roll_joint` 축 x, ±8°, 감쇠 2 N·m·s/rad, 스프링 없음. 링크 0.5 kg, 위치 (앞차축 x, 0, 바퀴 반경) | `front_pivot_roll_limit_deg`(SIM_ASSUMED, 차체 유연성 대신), `front_pivot_damping_nms_rad`, `front_pivot_link_mass_kg`(SIM_NUMERIC) |
| 바퀴 현가(실험, 기본 끔) | 바퀴마다 수직 스프링·감쇠(정적 처짐 1.5 cm, 감쇠비 0.5, ±4 cm) | `suspension_*`(SIM_ASSUMED). DART 관절 스프링이 120 Hz에서 지면 접촉 때 정적 하중을 틀리게 낸다(1.2~2.9배, `GZ_DEMO_KO.md` 6.3). 평가에 쓰지 않는다 |
| IMU | 33 Hz, 장착 자세 단위 회전 | `imu_rate_hz`(CONFIG), `imu_mount_rpy_deg`(SPEC_PENDING). `yaw_sign` −1이면 roll + π |
| 물리 | 120 Hz(gz는 8,333,333 ns로 자른다) | `gz_physics_hz` = `rover.yaml` `physics_hz` 120 |
| 참값 발행 | 자세 30 Hz(실제 약 24 Hz), 관절 30 Hz(약 24 Hz) | 생성 값 그대로 |

토픽(설계 4.3):

| 토픽 | 형식 | 뜻 |
|---|---|---|
| `/sim/act/steer_left_cmd`, `/sim/act/steer_right_cmd` | `gz.msgs.Double` | 바퀴 조향각 목표(rad, 왼쪽 +). `axle_pivot`은 `/sim/act/steer_axle_cmd` |
| `/sim/act/<바퀴>_cmd` (`left_front_wheel` 등) | `gz.msgs.Double` | 구동 관절 속도 목표(rad/s). 힘 모드는 duty × ω0, 기구학 모드는 바퀴별 속도 |
| `/sim/act/ackermann_cmd_vel` | `gz.msgs.Twist` | 교차 확인 장면 전용 |
| gz `/sim/ground_truth` → ROS `/ground_truth/odom` | Odometry | 참값. `cpu_lidar.py`가 쓴다. 실행기 금지 |
| gz `/sim/truth/joint_states` | Model | 참값 관절(조향 관절 포함). 실행기 금지 |

## 3. 장면

| 장면 | 월드 | 내용 |
|---|---|---|
| `w0_force` | W0 | `generate_world`의 `sensor_test` 월드에서 나무·시험 장애물을 지우고 바닥 200 × 200 m(마찰 1.0 = `ground_mu`), LiDAR 정합용 기둥 6개(r 0.08 m, 높이 1.5 m, 원점에서 22 m 이상)를 둔다. 주 구성(힘 모드) |
| `w0_kinematic` | W0 | A1(G) 기하 검사. Ackermann 비율 1, 바퀴 마찰 1.0, 바퀴별 기구학 속도 모드 |
| `w0_ackermann_check` | W0 | 표준 `AckermannSteering` 교차 확인(비율 1, 마찰 1.0) |
| `w1_force` | W1 | 원본 과수원(`orchard_gazebo.world_gen`, lite, seed 7)에 조향 차. 주 구성 |

- 모든 장면은 `assemble_world.assemble()`을 부른 뒤 변환한다. 월드 이름은 `ubts_orchard` 그대로다(기존 도구가 `/world/ubts_orchard/control`을 쓴다).
- LiDAR는 Embree가 기본이다(`assemble_world`가 gpu_lidar를 지운다. `cpu_lidar.py`가 `/ground_truth/odom`으로 `/sim/scan`을 만든다. 이번 단계에서는 돌리지 않았다).
- 원본 과수원 생성기는 실제 지명 근처 GPS 원점을 `<spherical_coordinates>`와 metadata의 `config.latitude/longitude/elevation`에 쓴다. 조향 장면은 둘 다 지운다. 쓴 파일에 `gnss_lint()`가 무엇이든 찾으면 오류로 멈춘다.
- W1 출발 높이(2026-11-09 고침): 생성기는 자기 차(PX4) 자세를 지형보다 약 0.15 m 위에 둔다. 조향 차 원점은 바퀴 바닥이라 출발 때 0.15 m 떨어졌다. `fix_spawn_height()`가 metadata `spawn`의 지형 높이 + 0.03 m(`W1_SPAWN_DROP_M`, SIM_ASSUMED)로 고친다. 장면 manifest `world_summary.spawn_height`에 남는다.
- W1 조립은 메시를 `/root/.gz`에 쓰므로 SIM 컨테이너 안에서 한다.

```bash
# 컨테이너 안(이미지 ubts-steer-sim:p5), 출력은 저장소 밖 폴더
python3 -B -m steering.gz_assemble --scene w0_force --out /runs/scenes/w0_force [--no-camera]
python3 -B -m steering.gz_assemble --scene w1_force --out /runs/scenes/w1_force
```

## 4. 이미지

- `docker/Dockerfile.ubts-steer`. 기반은 `ubts-sim-clean-tenth:20261008-1791399576735742400-14657b29@sha256:6209168a3507…`(이 PC에서 RepoDigest = 이미지 ID). 빌드 전후 기반 ID가 같은지 확인했다.
- apt·pip는 쓰지 않는다. 기반 이미지의 옛 소스(`/opt/ubts/dashboard`, `sim/ubts`, `ros2_ws/src/orchard_gazebo`, `orchard_navigation`, `compose.ubts*.yaml`)를 지우고 브랜치 소스를 복사한다. 그래서 브랜치에서 지운 파일이 남지 않는다.
- 이미지 g++로 대시보드 raster 커널(옛 기반 정의 `Dockerfile.ubts`와 같은 단계)과 조향 네이티브 lockstep 실행 파일(결정 R1 (d))을 만든다. 네이티브 빌드가 실패해도 이미지 빌드는 이어지고 `/opt/ubts/steer_native_build/BUILD_RESULT.json`에 남는다.
- `/opt/ubts/STEER_IMAGE_MANIFEST.json`: 기반 이미지 참조·ID, 소스 커밋, `git status --porcelain` SHA-256, 파일별 SHA-256, 네이티브 빌드 결과, `firmware_build_id`.
- 펌웨어 소스(2026-11-09 고침, 리뷰 F8): `firmware/ubts_steer_v3`(P3)를 dockerignore 허용 목록(`!firmware/`, `!firmware/**`)과 `COPY firmware /opt/ubts/firmware`에 넣었다. manifest 파일 목록에도 들어간다. 네이티브 빌드 뒤 `BUILD_RESULT.json`에 `firmware_build_id`가 없으면 이미지 빌드가 멈춘다(`UBTS_STEER_REQUIRE_FIRMWARE=1` 기본, 0이면 펌웨어 모드 없는 이미지를 허용하고 manifest에 적는다). 처음에는 펌웨어 없이 빌드되어도 이미지 빌드가 이어졌고, firmware 모드는 실행 때 `firmware mode needs the firmware sources`로 멈췄을 것이다.
  - 이번에는 새 이미지를 만들지 않았다(이 브랜치의 새 이미지 ID 두 개를 이미 썼다). 그래서 Dockerfile 고침은 빌드로 확인하지 않았다(미시험). 확인한 것: 같은 dockerignore로 BuildKit 문맥만 꺼낸 시험(이미지를 만들지 않는 `--output type=local`)에서 `firmware/ubts_steer_v3`가 문맥에 들어간다(11절). 그리고 이미지의 g++로 `/work/firmware`가 있는 작업 트리에서 네이티브 빌드를 하면 `firmware_build_id` 73575045가 들어간다(fix_wf2b `acc/a25_native` 결과의 빌드 기록).
- 주의: `.gitignore` 23행 `firmware/`(옛 PX4 빌드 결과 규칙)가 P3 펌웨어 소스를 git에서 숨긴다. 보통 `git add`로는 커밋에 들어가지 않는다(통합 담당 확인 필요, WF-2a·통합 범위).

빌드(작업 트리 뿌리에서, PowerShell):

```powershell
$base = 'ubts-sim-clean-tenth:20261008-1791399576735742400-14657b29'
$baseId = docker image inspect --format '{{.Id}}' $base
$status = git status --porcelain | Out-String
$sha = [BitConverter]::ToString([Security.Cryptography.SHA256]::Create().ComputeHash([Text.Encoding]::UTF8.GetBytes($status))).Replace('-','').ToLower()
python -I <작업 폴더>/tools/docker_budget.py acquire --label <label> --gib 4
docker build -f docker/Dockerfile.ubts-steer --build-arg "BASE_IMAGE_ID=$baseId" `
  --build-arg "SOURCE_COMMIT=$(git rev-parse HEAD)" --build-arg "SOURCE_STATUS_SHA256=$sha" -t ubts-steer-sim:p5 .
python -I <작업 폴더>/tools/docker_budget.py release --label <label>
docker image inspect --format '{{.Id}}' $base      # 빌드 전과 같은지 본다
```

- 3단계 이미지 `ubts-steer-sim:wf3`는 작업 트리를 그대로 쓰지 않고 HEAD + 이 작업이 바꾼 파일의 스냅숏 문맥으로 만들었다(다른 작업 흐름의 커밋 전 수정이 섞이지 않게). 방법과 결과는 12.2절이다.

## 5. probe 사용법

`gz_probe.py`는 GUI 없이 돈다. 엔진은 둘이다.

- `fixture`(합격 측정): gz-sim 서버를 같은 프로세스에서 돌린다(`gz.sim8.TestFixture`). 스텝마다 참값을 ECM에서 읽고 다음 스텝 명령을 gz-transport로 보낸다. 같은 프로세스 안 전달은 동기라서 스텝 k 뒤에 보낸 명령이 스텝 k+1 동안 쓰인다(실측: 목표와 관절 차이 약 1e-9°). 물리는 장면 그대로(120 Hz, DART)이고 실시간 배율만 0(최대 속도)으로 둔다. 기록은 물리 주기(120 Hz)다. 한 월드에 차를 여러 대 두고 차마다 토픽 접두사 `/probe/<이름>/act`를 쓴다. 센서와 Sensors 시스템은 뺀다.
- `server`(A10, 시연): `gz sim -s -r`를 하위 프로세스로 실시간 배율 1로 돌린다. 장면의 센서(카메라 포함)를 그대로 쓴다. `/world/ubts_orchard/stats`로 RTF를 잰다.

명령(컨테이너 안):

```bash
python3 -B -m steering.gz_probe run all --out /runs/acc            # A1, A4, A21, A25 묶음 + 진단 묶음
python3 -B -m steering.gz_probe run a1_kinematic a25_native --out /runs/acc
python3 -B -m steering.gz_probe rtf --out /runs/acc/a10 --duration 240 --camera both     # A10(W0)
python3 -B -m steering.gz_probe drive --scene-dir /runs/scenes/w1_force --out /runs/w1 --duration 90 --steer-deg 0 --duty 0.48
python3 -B -m steering.gz_probe track --out /runs/acc/track --duration 60                # 비동기 추종(보고)
python3 -B -m steering.gz_probe report --out /runs/acc                                  # ACCEPTANCE_P5.json
```

- 묶음: `a1_kinematic`, `a1_force`, `a1_force_ideal`(진단), `a1_ackermann`, `a4_drive`, `a4_tau_sweep`(진단), `a21_limits`, `a25_l0`, `a25_native`, `fdir_aniso`(진단, 6절).
- 모델 옵션을 모든 차에 줄 수 있다: `--model-option front_axle_pivot=false`(단단한 옛 모델), `--model-option suspension=true`(실험), `--model-option wheel_collision=sphere`(8절). 값은 true/false, 수, 그 밖에는 글자다. 결과 `result.json`의 `model_options`에 남는다.
- `run_server`(A10, drive)는 `extra_camera_topics`로 다른 카메라(예: 시연 카메라)의 프레임 수와 fps도 센다(`extra_cameras`).
- A25 목표는 공유 구동기 모델에서 온다. `a25_l0`은 Python L0 서보, `a25_native`는 C++ L1 `test_servo`(컨테이너 g++로 빌드)다. 모델을 물리 dt로 한 스텝씩 돌리고, 스텝 k+1 끝의 바퀴각을 스텝 k+1 동안의 목표로 보낸다. 그래서 같은 SIM 시각의 관절 각과 모델 각을 비교한다.
- 컨테이너 실행 예(D3D12 카메라 경로, `compose.ubts.gpu.yaml`과 같은 설정):

```powershell
python -I <작업 폴더>/tools/docker_budget.py acquire --label <label> --gib 4
docker run --rm --cpus 4 --memory 4g --network none --name ubts-steer-<label> `
  --device /dev/dxg -v /usr/lib/wsl/lib:/usr/lib/wsl/lib:ro -v /usr/lib/wsl/drivers:/usr/lib/wsl/drivers:ro `
  -v /mnt/host/wslg/.X11-unix:/tmp/.X11-unix:ro `
  -e UBTS_RENDER_ENGINE=ogre -e UBTS_HEADLESS_RENDERING=0 -e LIBGL_ALWAYS_SOFTWARE=0 -e GALLIUM_DRIVER=d3d12 `
  -e MESA_D3D12_DEFAULT_ADAPTER_NAME=Intel -e DISPLAY=:0 -e EGL_PLATFORM=x11 -e LD_LIBRARY_PATH=/usr/lib/wsl/lib `
  -e GZ_PARTITION=ubts-steer-p5 -v "<runs>/<folder>:/runs" `
  ubts-steer-sim:p5 python3 -B -m steering.gz_probe rtf --out /runs/a10 --duration 240 --camera both
python -I <작업 폴더>/tools/docker_budget.py release --label <label>
```

- GPU 없이 소프트웨어 렌더링이면 `-e LIBGL_ALWAYS_SOFTWARE=1 -e EGL_PLATFORM=surfaceless -e GALLIUM_DRIVER=llvmpipe`(기본 `ogre2`, headless)를 준다. 카메라가 없는 장면이나 fixture 묶음은 GPU 설정이 필요 없다.
- 3단계부터 합격 측정은 `ubts-steer-sim:wf3`의 자기 소스로 돈다(소스를 붙이지 않음, 12.2절 명령).
- 개발 중에는 이미지 대신 기반 이미지에 작업 트리를 읽기 전용으로 붙여도 된다: `-v <work tree>:/work:ro -e PYTHONPATH=/opt/ubts/raydeps:/work/dashboard:/work/sim/ubts -w /work/sim/ubts`.
- 결과 출처(2026-11-09 고침, 리뷰 F2, 설계 7.3): probe `result.json`, `drive_result.json`, A10 요약, 시연 `run.json`, `circles.json`에 `provenance`가 있다. (a) 소스: 커밋과 `git status` 요약(호스트 git, 컨테이너 안에서는 git이 작업 트리를 못 읽으므로 호스트가 넘긴 `UBTS_STEER_SOURCE_COMMIT`, `UBTS_STEER_SOURCE_STATUS_SHA256`, `source_commit_origin`에 출처), 실제로 돈 파일의 SHA 묶음(`source_files`: 이미지 manifest와 같은 규칙, 뿌리별 값도), 이미지 manifest 요약. (b) 월드 SDF(probe_world.sdf 또는 장면) SHA. (c) 구동기 모델: L0는 `info()`, 네이티브는 `result_label()`(빌드 id, 실행 파일 SHA)과 빌드 기록(`build_key`, `binary_sha256`, `source_sha256`, `firmware_build_id`). 장면 manifest도 같은 커밋·파일 SHA 묶음을 적는다. 처음에는 컨테이너 결과의 커밋이 모두 null이었고 네이티브 구동기는 클래스 이름만 남았다.
- 그래서 컨테이너 실행에 두 환경 변수를 넘긴다: `-e UBTS_STEER_SOURCE_COMMIT=$(git log -1 --format=%H) -e UBTS_STEER_SOURCE_STATUS_SHA256=$(git status --porcelain | sha256sum | cut -d' ' -f1)`.
- p5_demo/acc(7.4절)는 이 기록 전의 실행이다. 그 시각의 소스는 결과에 없다. 리눅스 네이티브 빌드가 둘(f3c6fe450ee85c89 05:14, 9474fda8ae1e773f 05:51) 있었고 a25_native는 05:29~05:30에 돌아 시각으로만 f3c6…로 짐작할 수 있다. 판정 근거로는 11절의 다시 돈 결과를 쓴다.

## 6. 측정 정의

등록된 기준(`SIM_ACCEPTANCE_KO.md`)에 숫자가 없는 곳은 아래처럼 정하고 측정했다. 기준을 느슨하게 하지 않았다. 아래 정의는 합격 실행 전에 코드에 넣었다. 하나만 첫 짧은 시험(12 s) 뒤에 바꿨다: 조향 관절 속도 튐 한계를 모델 속도의 1.25배에서 1.5배로 넓혔다(이유는 아래). 1.25배 한계로도 모든 측정이 통과했다(최대 0.612 rad/s ≤ 0.654 rad/s).

- 회전반경 R: 뒤차축 중심 궤적에 원을 맞춘다(대수 맞춤 뒤 기하 거리 Gauss-Newton). 창은 출발 뒤 8~48 s(교차 확인은 12~48 s). 기하값은 L/tan δ.
- 요레이트: 창 안 yaw를 펴서 시간에 직선 맞춤. 기구학 모드의 기준은 명령 v로 계산한 v·tan δ/L이다(측정 v로 계산한 값도 남긴다).
- 출발 방향 무관: δ 10°, v 0.3 m/s에서 출발 방향 0°, 90°, 113.33°의 R이 평균의 ±1 % 안. 등방 마찰(mu = mu2)에서의 방향 무관이다. 이 조건에서는 마찰 기저가 바퀴 좌표든 월드 좌표든 결과가 거의 같아서 fdir1 확인이 아니다(리뷰 F3). 출발 방향이 하나뿐인 묶음(`a1_ackermann`)은 이 검사를 `passed` None(해당 없음)으로 둔다(리뷰 F5, 처음에는 편차 0으로 통과를 냈다).
- fdir1 진단(`fdir_aniso`, 2026-11-09 더함, 리뷰 F3): 설계 3.6의 [추정] "fdir1(0 0 1)은 바퀴 충돌 좌표에서 조향과 함께 돈다"를 본다. 바퀴 마찰을 크게 이방성으로 둔다(옆 mu 0.005, 앞뒤 mu2 1.0, 모델 옵션, 프로필 범위 밖은 일부러). 힘 모드, δ 20°, 0.5 m/s 목표, 출발 방향 0/45/90/135/200°, 24 s. 통과: 출발 좌표로 돌린 궤적끼리의 차가 1e-6 m 안이고, 프로필 마찰 기준 차와의 궤적 차가 0.05 m 이상(이방성이 실제로 작용함). 마찰 기저가 월드에 고정이면 출발 방향마다 궤적이 달라진다.
- A4(G) "duty 3점": 40, 70, 88 %. 기준 속도는 주 곡선(`forward_curve` P_fit1003, F-SIM과 같은 값)이다. 55 %도 남긴다. 정상 속도는 10~15 s 평균.
- 정지 거리: 사건 시각부터 차체 속도가 0.005 m/s 아래로 처음 내려갈 때까지의 경로 길이. release는 MCU 램프(100 %/s)로 duty를 0까지 줄인다(EN 켬). brake는 duty를 바로 0으로 한다(EN 켬). release 목표는 0.56~0.60 m ±15 % = 0.476~0.690 m.
  - 2026-11-09 고침(리뷰 F4): 표본은 스텝 끝 시각이고, 사건 명령은 끝 시각이 사건 시각 이후인 첫 스텝부터 걸린다. 그래서 "사건 시각부터"는 그 스텝 직전 상태(`searchsorted(t, t_event) − 1`)다. 처음 버전은 한 스텝 늦게 시작해 첫 사건 스텝의 이동(약 7 mm)을 빼고, `speed_at_event_m_s`로 한 스텝 제동 뒤 속도를 적었다. 결과에 `event_start_t_s`를 남긴다. release 실패 판정은 바뀌지 않았다.
- A21(G): 관절 각이 바퀴별 기계 한계를 1e-6 rad 넘게 벗어나면 위반이다(수치 허용만 둔다).
- A25 관절 속도 튐: 조향 관절 \|속도\| ≤ 1.5 × 공유 모델의 가장 큰 조향 속도(30°/s → 45°/s), 구동 관절 \|속도\| ≤ 1.25 × ω0(기구학 모드는 1.25 × 가장 큰 명령 속도). 값이 유한해야 한다. a = 1, 20°에서 바퀴 하나의 각속도는 등가각 속도의 약 1.25배까지 될 수 있어서 조향은 1.5배로 잡았다.
- A25 정지 표류: 60 s 창에서 차체 위치 변화 최대 < 1 cm, yaw 변화 최대 < 0.1°.
- A10: 실시간 배율 1로 240 s(앞 20 s 제외)를 돌리고 5 s 창의 Δsim/Δreal 중앙값을 쓴다. 차는 δ 10°, 0.4 m/s 목표로 원을 돈다. GUI 끔.

## 7. 합격 검사 결과

- 최종 실행: 이미지 `ubts-steer-sim:p5`(ID `sha256:e440445947ff…`, 기반 `6209168a3507…`, 소스 커밋 `d6f0eac` + 작업 트리 변경, manifest의 파일 SHA 합 `72f9c762…`), 2026-11-09. 출력은 저장소 밖 `<runs>/wf2b/p5_model/acc3/`(`ACCEPTANCE_P5.json`, 묶음별 `result.json`, `logs.npz`, 그림).
- 같은 소스를 기반 이미지에 붙여 먼저 돌린 결과(`acc1/`)와 비교했다. `a1_kinematic`, `a1_force`, `a4_drive`, `a21_limits`, `a25_l0`, `a25_native`의 기록(`logs.npz`)은 비트까지 같았다. `a1_ackermann`(표준 `AckermannSteering`)만 조금 달랐다: R 차이는 1e-9 m 수준, 과도 구간 관절 속도 차이는 최대 0.18 rad/s. 그 플러그인 안 어딘가가 실행마다 다르다고 본다(원인은 찾지 않았다). 교차 확인 전용이라 판정에는 영향이 없다.
- 자리값 기반 SIM 결과다. 현장 검증이 아니다.

| 검사 | 기준 | 결과 | 판정 |
|---|---|---|---|
| A1(G) 기구학 | a = 1, 마찰 1.0, δ 5/10/20°, v 0.3/0.5 m/s: R 오차 ≤ 3 %, ω가 v·tan δ / L의 ±5 %, 요레이트 부호 | R 오차 최대 0.019 %. ω 오차(명령 v 기준) +0.67 / +1.30 / +2.47 %(5/10/20°). 측정 v 기준은 0.02 % 이내. δ −10°도 부호 일치 | 통과 |
| A1(G) 힘 모드 | 보고 + 적합(유효 L, η, 언더스티어)과 잔차, 요레이트 부호 | a = 0.5, 마찰 0.9: R/R_geom = 1.013 / 1.026 / 1.052(5/10/20°). 0.3과 0.5 m/s의 R 차이 < 1e-4(속도에 무관). κ = tan δ / (L_eff + K v²) 맞춤: L_eff 0.790 m, η 0.962, K −0.077 s²/m(이 형식은 δ 의존을 담지 못해 K가 속도와 섞인다), 곡률 잔차 최대 2.2 %. K = 0으로 δ별 η: 0.987 / 0.975 / 0.950. 부호 일치 | 보고, 부호 통과 |
| (진단) 힘 모드 a = 1 | 보고 | R 오차 최대 0.013 %. 그래서 위의 R 차이는 a = 0.5 링크(두 앞바퀴 축선이 뒤차축 선 위 한 점에서 만나지 않음)에서 온다 | 보고 |
| A1 교차 확인 | 표준 `AckermannSteering`과 비교 | R 오차 최대 0.018 %. 뒤차축 속도는 명령보다 0.66 / 1.29 / 2.47 % 느리다(기구학 모드는 같은 크기로 빠르다, 8절) | 보고 |
| 출발 방향 무관 | 0°, 90°, 113.33° 출발에서 R이 평균 ±1 % 안 | 차이 1e-11(기구학), 5e-12(힘 모드) | 통과 |
| A4(G) 정상 속도 | DC 직선 적합 뒤 duty 3점(40/70/88 %) ±5 % | 측정 직선 r·ω0 0.93724 m/s, d0 0.04834(설계값과 같다). 40/70/88 %: +0.05 / −0.16 / −0.22 %. 20~100 % 전부 ±1 % 안(100 %에서 −0.90 %, 주 곡선은 90 % 위에서 꺾인다) | 통과 |
| A4(G) release 정지 | 보고, release 목표 0.56~0.60 m ±15 %(0.476~0.690 m) | 0.89 m/s에서 0.960 m(2.61 s). 목표 밖 | 실패(목표 밖) |
| A4(G) brake 정지 | 보고 | 0.88 m/s에서 0.521 m(2.06 s) | 보고 |
| (진단) τ_stall 스윕 | 보고 | release: 0.5 → 2.13, 0.8 → 1.50, 1.0 → 1.28, 2.5 → 0.77, 4.4 → 0.62, 5.0 → 0.59 m. brake: 1.69 / 1.06 / 0.84 / 0.33 / 0.18 / 0.16 m. 목표 안은 4.4와 5.0 N·m | 보고 |
| A21(G) | 바퀴별 기계 한계를 넘는 관절 각 0 | ±40°, ±60° 직접 명령(서 있음, 반대 방향, 주행 중)과 공유 L0 모델 ±35°, ±80° 명령에서 넘음 최대 2e-9°(수치). 한계 ±28.000000° | 통과 |
| A25 | 관절 − 공유 모델 목표 p99 ≤ 0.2°, 정지 60 s 표류 < 1 cm·< 0.1°, 관절 속도 튐 없음 | L0: p99 1.08e-9°. 네이티브 L1 `test_servo`: p99 1.11e-9°. 0°로 서 있음 60 s: 위치 1e-13 m, yaw 2e-14°. 20° 꺾고 서 있음 60 s: L0 3e-11 m·8e-9°, L1 2.5e-5 m·6.0e-4°. 조향 관절 속도 최대 0.612 rad/s(한계 0.785), 구동 관절 2.89 rad/s(한계 8.37) | 통과 |
| A10(W0) | GUI 끔, 카메라 640×360, RTF 중앙값 ≥ 0.9 | D3D12 카메라 경로 240 s: 중앙값 1.000, 최소 1.000, 카메라 30.30 fps. 카메라 없음: 1.000(최소 0.998). 소프트웨어 렌더링 150 s(진단 실행): 1.000, 30.30 fps(7.1) | 통과 |
| 120 Hz 안정 | 관절 속도 튐 없음, 정지 표류, 발산 없음 | 모든 묶음에서 값이 유한하고 위 한계 안. 조향 링크 관성 하한 1e-3, effort 1e6, 속도 명령 모드에서 발산 없음 | 통과 |

- A1 기구학 모드의 ω 오차(+2.47 %)는 기준(±5 %) 안이다. 원인은 8절의 바퀴 모서리 접촉으로 본다[추정].
- A25가 1e-9° 수준인 것은 lockstep으로 목표를 같은 SIM 시각에 맞췄기 때문이다. 다른 프로세스에서 비동기로 보낼 때의 값은 7.2(보고)에 있다.

### 7.1 A10(W0)

| 조건 | RTF 중앙값 | 10번째 백분위 | 최소 | 카메라 | 판정 |
|---|---|---|---|---|---|
| 카메라 640×360, D3D12(GPU) 경로, 240 s | 1.000 | 1.000 | 1.000 | 30.30 fps(벽시계), 7,252장 | 통과 |
| 카메라 없음(서버만), 240 s | 1.000 | 1.000 | 0.998 | — | 통과 |
| 카메라 640×360, 소프트웨어(llvmpipe, `ogre2` headless) 경로, 150 s | 1.000 | 1.000 | 1.000 | 30.30 fps(벽시계), 4,524장 | 보고(기준 이상) |

- 카메라가 30 Hz 설정인데 30.30장/s인 까닭: 프레임 시각을 보면 첫 장 뒤로 25장마다 한 번 간격이 0.025 s(물리 3스텝)이고 나머지는 0.0333 s다. 빠진 프레임은 없다. 2026-11-09 p5_demo가 W1 실시간 실행 기록에서 확인했다(`GZ_DEMO_KO.md` 4절). 원인은 조사하지 않았다.
- 차는 δ 10°, 목표 0.4 m/s로 원을 돌았다(240 s에 76 m). GUI는 껐다. Embree LiDAR 프로세스와 ROS 브리지는 이번 측정에 없다(P6a에서 전체 스택으로 잰다).
- D3D12 경로에는 `/dev/dxg`와 `/usr/lib/wsl/lib`, `/usr/lib/wsl/drivers`, WSLg X11 소켓 마운트가 필요하다. `drivers` 마운트가 없으면 GLX 문맥을 만들지 못하고 gz sim이 종료 1로 끝났다(실측).
- 소프트웨어 렌더링: 150 s 실행(창 25개)은 위 표처럼 기준 이상이었다. 그보다 먼저 한 40 s 시험(창 3개)은 0.96 / 0.54 / 0.76, 카메라 21.6 fps였다. 그때 이 PC에 다른 부하가 있었는지는 확인하지 않았다. 짧은 시험 값은 판정에 쓰지 않는다. 컨테이너 CPU는 4개로 제한했다.

### 7.2 보고: 비동기 추종, W1 주행

- 비동기 추종(`gz_probe track`, 등록 기준 아님): 실시간 서버(`gz sim -s -r`)와 다른 프로세스에서 공유 L0 모델이 조향 관절을 몰았다. 관절 상태를 받을 때마다 모델을 다음 스텝 끝까지 돌리고 목표를 보냈다. 60 s, 표본 14,416개 모두 제시간(한 스텝 늦음 0), 오차 p99 1.09e-9°, 빠진 관절 상태 스텝 0. 서버가 스텝마다 남는 시간(약 8 ms)이 있어서 왕복이 늦지 않았다. P6a 에뮬레이터도 같은 방식(다음 스텝 끝 각을 보냄)이면 한 스텝 지연을 피할 수 있다. 실시간 배율이 1보다 크게 빨라지거나 부하가 크면 다시 재야 한다.
- W1 주행(보고): 원본 과수원에서 직진 duty 0.48(평지 목표 0.4 m/s), 90 s. RTF 1.000, 카메라 30.31 fps. 그러나 차는 0.43 m 가고 섰다. 진단(같은 이미지, 카메라 없음, 60 s):

| τ_stall | duty | 60 s 이동 거리 |
|---|---|---|
| 1.6 N·m(자리값) | 0.48 | 0.63 m(섬) |
| 1.6 N·m | 0.80 | 28.3 m |
| 4.4 N·m(진단 프로필) | 0.48 | 18.9 m |

  - τ_stall 1.6에서 duty 0.48의 정지 토크는 바퀴당 0.77 N·m다. 관절 마찰 0.077 N·m를 빼면 네 바퀴 밀힘은 약 20 N이고, 30 kg 차가 약 4° 비탈까지만 오른다. 원본 지형의 기복·요철에서 걸린 것으로 본다(토크 부족)[추정]. 원인 구간의 지형 기울기는 따로 재지 않았다.
  - 시연 단계에서 W1을 τ_stall 1.6 자리값으로 돌리면 duty를 0.8 근처로 올려야 움직인다.
  - 2026-11-09 추가(p5_demo): 원인은 토크만이 아니었다. 단단한 차체가 W1 요철에서 바퀴 하나(때로 대각 두 개)를 띄웠다. 뜬 바퀴는 무부하 속도로 헛돌았다(duty 0.48에서 2.89 rad/s). 앞차축 롤 피벗을 넣은 뒤 duty 0.48로도 간다(7.4절, `GZ_DEMO_KO.md` 6.3).

### 7.3 진단: τ_stall 4.4 N·m

- 저장소 밖 진단 프로필(자리값 프로필에서 `drive_tau_stall_nm`만 4.4, id `front_steer_v0_tau44_diag`, SHA-256 `f71ddfb135be…`)로 `a1_force`, `a4_drive`를 같은 이미지에서 다시 돌렸다. 범위(0.5~5) 안의 값이다.
- 4.4는 F-SIM 출발 지연 `drive_accel_tau_s` 0.25 s와 같은 G-SIM 시정수가 되는 값이다(T = r²·ω0·m_eff / (4 τ_stall)).
- A4(G): 정상 속도는 그대로(ω0가 같다). 90 % 도달 시간 1.63 s → 0.59 s. release 0.616 m(목표 안, 통과), brake 0.180 m, 속도 추종 coast 2.25 m(5.0 s에 섬).
- 회전 중 속도 손실(0.3 / 0.5 m/s): 5° 4.5 / 2.8 %, 10° 10.1 / 6.4 %, 20° 24.1 / 15.9 %. τ_stall 1.6보다 작지만 F-SIM 끌림 손실 범위(20°에서 최대 12 %)보다 크다. R/R_geom은 같다(1.013 / 1.026 / 1.052).
- 이 결과는 진단이다. 프로필 값 변경은 통합 담당이 새 프로필 파일로 결정한다.

### 7.4 고친 모델로 다시 돈 결과 (2026-11-09, p5_demo)

- 모델: 바퀴 홈 차체 + 앞차축 롤 피벗(기본) + W1 출발 높이(1절, 2절, 3절). 새 이미지를 만들지 않고 `ubts-steer-sim:p5`(e440445947ff)에 작업 트리를 읽기 전용으로 붙여 돌렸다. 출력 `runs/wf2b/p5_demo/acc/`, 이전 결과와 비교 `acc/COMPARE_acc3_vs_acc.json`.
- 판정은 모두 같다. 자리값 기반 SIM 결과다.

| 검사 | 이전(acc3, 단단한 모델) | 다시 돈 결과 | 판정 |
|---|---|---|---|
| A1(G) 기구학 | R 오차 최대 0.019 %, ω 최대 +2.47 % | R 차이 ≤ 2e-5 m, 같은 값 | 통과 |
| A1(G) 힘 모드 | R/R_geom 1.013/1.026/1.052, L_eff 0.790, η 0.962, K −0.077, 잔차 2.2 % | R/R_geom 같음, L_eff 0.790, η 0.962, K −0.074, 잔차 2.2 %. 회전 중 속도 0.3 m/s에서 5/10/20° 손실 11.0/24.3/56.6 %(이전 11.9/26.1/59.6 %) | 보고, 부호 통과 |
| 교차 확인 | R 오차 ≤ 0.018 % | 차이 ≤ 2e-6 m | 보고 |
| 출발 방향 무관 | 1e-11 | 같음 | 통과 |
| A4(G) 정상 속도 | 최대 0.22 % | 0.22 % | 통과 |
| A4(G) release | 0.960 m | 0.968 m | 실패(목표 밖), 그대로 |
| A4(G) brake | 0.521 m | 0.528 m | 보고 |
| τ_stall 스윕 | 4.4·5.0만 목표 안 | 같음(0.619, 0.594 m) | 보고 |
| A21(G) | 넘음 최대 2e-9° | 최대 1e-9° | 통과 |
| A25 L0 | p99 1.08e-9° | p99 9.2e-10° | 통과 |
| A25 네이티브 L1 | p99 1.11e-9°, 꺾고 정지 2.5e-5 m·6e-4° | p99 9.5e-10°, 2.1e-5 m·5.1e-4° | 통과 |
| A10(W0) 카메라 없음 | 1.000 | 중앙값 1.000, 최소 0.99997(240 s) | 통과 |
| A10(W0) 카메라 640×360 | 1.000, 30.30 fps | 중앙값 1.000, 최소 0.997, 카메라 30.30 fps(7,240장) | 통과 |
| W1 주행 보고(duty 0.48, 90 s) | 0.59 m(그 뒤 멈춤), RTF 1.000, 카메라 30.31 fps | 22.3 m, RTF 1.000, 카메라 30.31 fps | 보고 |
| 단위 시험 `test_steering_gz_model.py`(3.12, pytest, W1 포함) | 33 통과 | 47 통과 | |

## 8. F-SIM과 다른 점

- 회전 곡률: F-SIM은 κ = η tan δ / (L + K_us v²)에서 η·K_us를 run마다 뽑는다(U(0.85, 1.0), U(0, 0.03)). G-SIM 힘 모드(a = 0.5)는 η가 δ에 따라 0.987 → 0.950으로 줄고 속도에 무관하다(K ≈ 0). a = 1이면 η ≈ 1이다. A12 전에 F-SIM 쪽을 이 결과에 맞출지 정해야 한다.
- 회전 중 속도 손실: 같은 duty 4륜이라 회전하면 바퀴가 다툰다. 첫 측정(7절, 단단한 모델, 32.07 kg)에서 G-SIM은 직진 목표 0.3 m/s에서 δ 5/10/20°에 0.264 / 0.222 / 0.121 m/s(손실 12 / 26 / 60 %), 0.5 m/s에서 7 / 16 / 37 % 느렸다. 고친 모델(11절, 30 kg, 원통 충돌)에서는 0.3 m/s에 10.3 / 22.7 / 52.5 %, 0.5 m/s에 6.3 / 14.0 / 33.0 %다. F-SIM의 끌림 손실 U(0, 0.006)/°는 20°에서 최대 12 %다. 차이의 대부분은 아래 원통 모서리 접촉이다. 낮은 duty에서 DC 직선 토크 여유가 작은 것(τ_stall 1.6 자리값)도 함께 작용한다. τ_stall 4.4 진단은 7.3에 있다.
- 바퀴 충돌 원통의 양 끝 두 점 접촉(2026-11-09 측정, 리뷰 F1; 처음에는 [추정]): 평면 위 원통은 양 모서리 두 점으로 닿는다. DART 쿨롱 마찰에는 미끄럼 강성이 없어서 회전하면 두 접점이 반대로 미끄러지며 끌림을 만들고, 굴림 기준점이 바퀴 가운데가 아닌 모서리로 간다. 같은 probe 묶음을 `wheel_collision=sphere`(바퀴 반경 구, 시각·관성·마찰·fdir1 그대로)로 다시 돌렸다(fix_wf2b `sphere/`, 고친 모델):

| 측정 | 원통(기본) | 구 | 비고 |
|---|---|---|---|
| 힘 모드 회전 중 속도비, 0.3 m/s, δ 5/10/20° | 0.897 / 0.773 / 0.475 | 0.987 / 0.950 / 0.814 | 구도 20°에서 18.7 % 손실(F-SIM 최대 12 %보다 큼) |
| 같은 값, 0.5 m/s | 0.937 / 0.860 / 0.670 | 0.991 / 0.965 / 0.871 | |
| 힘 모드 R/R_geom, δ 5/10/20° | 1.013 / 1.026 / 1.052 | 1.013 / 1.026 / 1.052 | 회전반경(곡률, η)은 모양에 무관 |
| 힘 모드 맞춤 L_eff, η, K, 곡률 잔차 최대 | 0.790, 0.962, −0.070, 2.2 % | 0.787, 0.966, −0.029, 2.3 % | K가 속도와 섞이는 것(개정 3)은 같다 |
| 기구학 모드 ω 오차(명령 v 기준), δ 5/10/20° | +0.67 / +1.30 / +2.47 % | −0.001 % (모두) | 원통의 편향은 모서리 접촉에서 왔다 |
| 기구학 모드 R 오차 최대 | 0.019 % | 0.018 % | |
| A4(G) 정상 속도 40/70/88 % 오차 | +0.05 / −0.16 / −0.22 % | +0.05 / −0.16 / −0.22 % | 직진은 같다(리뷰의 미시험 항목) |
| A4(G) release / brake 정지 거리 | 0.943 / 0.504 m | 0.943 / 0.504 m | 직진 정지는 같다 |
| A25 L0 / 네이티브 p99, 정지 60 s 표류 최대 | 8.2e-10° / 8.6e-10°, 1.7e-5 m | 2.8e-11° / 7.1e-11°, 6.2e-7 m | 둘 다 통과 |

  - 결론: 회전 중 속도 손실과 기구학 모드 속도 편향의 큰 몫은 원통 모서리 접촉이라는 단순화에서 온다. 회전반경은 영향이 없다. A12(P7)에서 F-SIM 끌림을 맞출 때와 P6a 폐루프의 회전 중 속도를 볼 때 이 인공물을 분리해야 한다. 구로 바꿀지는 결정 제안이다(통합 담당). 바꾸면 A1(G)·A4(G)·A25는 위 `sphere` 열 값으로 다시 판정하고(모두 통과), 시연 영상도 다시 만든다. 구는 옆으로 바퀴 반경만큼 퍼져서 W1 요철에서 옆면 접촉이 원통과 다를 수 있다(미시험).
  - 리뷰 probe(원통 폭 스윕 0.12/0.06/0.02 m)는 폭이 줄수록 구 값으로 다가가고 기구학 편향이 (w/2)/(d − w/2) 가설처럼 폭에 비례함을 보였다(`runs/wf2b/review_gz/`, `verify_findings/`).
- 가감속: G-SIM DC 직선(τ_stall 1.6)은 출발 시정수가 약 0.65 s(90 % 도달 1.55 s, 고친 30 kg 모델)다. 첫 측정(32.07 kg)은 1.63 s였다. F-SIM은 `drive_accel_tau_s` 0.25 s다. τ_stall 4.4 N·m면 G-SIM 시정수가 약 0.24 s가 된다(T = r²·ω0·m_eff / (4 τ_stall), m_eff = 30 kg + 4J/r² ≈ 33 kg; 첫 모델은 32.07 kg + 3 ≈ 35 kg이었다).
- 정지: F-SIM release는 0.56~0.60 m(옛 구동계 측정)를 재현한다(개정 1 판정 통과). G-SIM release는 MCU 램프 100 %/s를 따라 duty를 줄여도 DC 직선 토크가 작아 0.94 m를 간다(고친 모델, 첫 측정 0.96 m). F-SIM brake는 시작 0.10 s + 2.0 m/s²(약 0.28 m)이고 G-SIM brake(duty 0, 단락 제동 꼴)는 0.50 m다. coast는 G-SIM이 표현하지 못한다(9절).
- 조향: G-SIM 관절은 공유 모델 각을 그대로 따른다. 그래서 조향 동역학(속도 한계, 지연, 데드밴드, 백래시, 끝 멈춤)은 F-SIM과 같은 모델이다. 차이는 차체·타이어 쪽에서만 생긴다.
- 정지 중 조향: 서 있는 차에서 앞바퀴를 ±20° 계단으로 돌리는 64 s 동안 차체는 최대 0.04 mm(L0)·0.12 mm(L1) 움직이고 yaw는 0.002° 안이었다. 20°로 꺾은 채 60 s 서 있으면 2.5e-5 m 안이다. 조향 관절은 effort 1e6으로 붙잡혀 지면 마찰이 관절을 밀지 못한다(자기 잠금 가정). 실제 차는 정지 중 조향 부하로 바퀴가 덜 돌 수 있다. 그 효과는 공유 구동기 모델(정지 중 속도·최소 PWM)이 맡는다.

## 9. 표현하지 못하는 것과 플러그인 검토

- 조향 free 모드(비자기잠금, 출력 0): `JointPositionController` 속도 모드는 관절을 늘 붙잡는다. 표현하지 못한다. G-SIM 결과는 자기 잠금 가정이다.
- 구동 coast(EN 끔): `JointController`는 실행 중 이득을 바꿀 수 없다. 이번에 lockstep에서만 되는 우회를 실험했다: 매 스텝 속도 목표를 측정 바퀴 속도로 두면 P 출력이 0이 된다. 0.89 m/s에서 6 s 동안 4.19 m를 가고 서지 않았다(감속은 관절 쿨롱 마찰뿐, 약 0.07 m/s²). F-SIM coast(0.6 m/s², SIM_ASSUMED)와 다르다. 비동기 전송에서는 정확하지 않으므로 제품 경로로 쓰지 않는다.
- 사용자 정의 gz 시스템 플러그인 검토(보고만, 저장소에 넣지 않음):
  - 이미지에 헤더(`gz/sim8/gz/sim/System.hh`)와 CMake 설정(`gz-sim8`, `gz-plugin2`, `gz-transport13`, `gz-msgs10`, `gz-cmake3`)이 있다. g++ 13.3, cmake 3.28.
  - 시험 플러그인(DC 직선 + EN, 약 90줄 C++)을 저장소 밖에서 컴파일했다: configure + build 8.5 s, `GzPluginHook` 내보냄.
  - W0 차 한 대의 구동 `JointController` 4개를 이 플러그인으로 바꿔 돌렸다: EN 켬 duty 1.0에서 0.8919 m/s(DC 직선 값과 같음), 10 s에 EN 끔 → 10 s 동안 5.7 m coast(감속은 관절 마찰뿐).
  - 비용 추정: 구동 EN·coast 플러그인을 제품으로 넣는 데 반나절~1일(코드, Dockerfile 빌드 단계, 시험, coast 감속 모델 정하기). 조향 free 모드는 속도 서보와 힘 모드를 전환하고 복원 토크를 주는 플러그인이 필요하고, 가벼운 조향 링크의 힘 제어 안정성(설계 3.6, PX4 발산 이력)을 따로 확인해야 해서 1~2일로 본다. 결정은 통합 담당에게 넘긴다.
- 자기 정렬 토크, 타이어 미끄럼 강성: 단순 마찰 모델이라 없다.

## 10. 시험하지 않은 것

- Embree LiDAR(`cpu_lidar.py`), ROS 브리지, IMU·카메라 출력 형식(A5)과 가상 GPS. P6a 범위다. 카메라는 RTF 측정에서 프레임 수만 셌다.
- `axle_pivot` 형식, 캐스터·킹핀 경사, 비대칭 한계, `rear2`/`front2` 구동, 배터리 배율의 Gazebo 실행. 변환 단위 시험만 했다. 앞차축 롤 피벗과 `axle_pivot`의 조합도 단위 시험만 했다.
- 앞차축 롤 피벗의 한계(±8°)·감쇠 스윕. 기본값과 W0·W1 몇 실행만 돌렸다.
- 프로필 스윕(마찰, 질량, 링크 질량, 한계)에서의 120 Hz 안정성. 기본 프로필과 τ_stall 스윕만 돌렸다.
- 지형(`terrain_ruts` 등). W0는 평지, W1은 원본 지형(주행 보고만).
- A12(두 SIM 일치), W2 월드, P3 `firmware` 모드 백엔드로 Gazebo 관절 구동(firmware 모드 구동은 12절에서 보고로 했다).
- 비동기 전송 A25는 보고로만 쟀다(7.2). 등록 기준은 lockstep으로 판정했다.
- 구 바퀴 충돌의 W1(요철 지형) 실행, 구로 만든 시연 영상, 구에서 A10(12절에서 했다).
- 고친 Dockerfile로 이미지 빌드(새 이미지 ID를 더 만들지 않았다. 12절에서 `ubts-steer-sim:wf3`로 했다).
- 실차. 모든 값은 자리값이다.

## 11. 리뷰 고친 뒤 다시 돈 결과 (2026-11-09, fix_wf2b)

- 실행: 이미지 `ubts-steer-sim:p5`(`e440445947ff`)에 작업 트리를 읽기 전용으로 붙였다(새 이미지 없음, `--network none`, 4 CPU, 4 GiB, 예산 잡고 풂). 출처(결과 `provenance`): 커밋 `d6f0eac` + 작업 트리 변경(호스트가 넘김), 돈 파일 SHA 묶음 `fbe53908d73a5d7d…`, 네이티브 리눅스 빌드 `3c00c17756ace866`(바이너리 `f09ab7cd…`, `firmware_build_id` 73575045). 출력 `runs/wf2b/fix_wf2b/acc/`(`ACCEPTANCE_P5.json`), `sphere/`(8절), `media_runs/`, `media/`.
- 모델: 차 30 kg(F6), 바퀴 충돌 원통(기본). 정지 거리는 고친 정의(F4). 자리값 기반 SIM 결과다. 현장 검증이 아니다.

| 검사 | p5_demo/acc(32.07 kg, 첫 정지 거리 정의) | 고친 뒤 | 판정 |
|---|---|---|---|
| A1(G) 기구학 | R 오차 최대 0.019 %, ω +0.67 / +1.30 / +2.47 % | R 오차 최대 0.019 %, ω +0.67 / +1.31 / +2.47 % | 통과 |
| A1(G) 힘 모드 | R/R_geom 1.013 / 1.026 / 1.052, L_eff 0.790, η 0.962, K −0.074, 잔차 2.2 %; 0.3 m/s 회전 중 손실 11.0 / 24.3 / 56.6 % | R/R_geom 같음, L_eff 0.790, η 0.962, K −0.070, 잔차 2.2 %; 손실 10.3 / 22.7 / 52.5 % | 보고, 부호 통과 |
| 교차 확인 | R 오차 ≤ 0.018 % | R 오차 최대 0.018 %. 출발 방향 검사는 해당 없음(방향 하나, F5) | 보고 |
| 출발 방향 무관(등방 마찰) | 1e-11 | 1.1e-11(기구학), 5.4e-12(힘 모드) | 통과 |
| fdir1 진단(새 묶음, F3) | — | 출발 방향 5개의 출발 좌표 궤적 차 최대 8.1e-11 m, yaw 차 4.4e-10°. 프로필 마찰 기준 차와 궤적이 11.7 m 다름(이방성이 작용) | 통과(진단) |
| A4(G) 정상 속도 | 최대 0.22 % | 40/70/88 %: +0.05 / −0.16 / −0.22 %. 직선 r·ω0 0.93724 m/s, d0 0.04834. 90 % 도달 1.55 s(전 1.65 s) | 통과 |
| A4(G) release 정지 | 0.968 m | 0.943 m(2.52 s, 0.892 m/s에서) | 실패(목표 0.476~0.690 m 밖), 그대로 |
| A4(G) brake 정지 | 0.528 m | 0.504 m(1.97 s) | 보고 |
| τ_stall 스윕 | 4.4·5.0만 목표 안 | release 0.5→2.06, 0.8→1.45, 1.0→1.25, 2.5→0.76, 4.4→0.61, 5.0→0.59 m; brake 1.62 / 1.01 / 0.81 / 0.32 / 0.18 / 0.16 m. 4.4·5.0만 목표 안 | 보고 |
| A21(G) | 넘음 최대 1e-9° | 1.1e-9° | 통과 |
| A25 L0 | p99 9.2e-10° | p99 8.2e-10°. 정지 60 s 표류 1.7e-9 m | 통과 |
| A25 네이티브 L1 | p99 9.5e-10°, 꺾고 정지 2.1e-5 m | p99 8.6e-10°, 꺾고 정지 1.7e-5 m·4.2e-4°. 조향 관절 속도 최대 0.59 rad/s, 구동 관절 2.89 rad/s | 통과 |
| 차 질량(F6) | 32.07 kg | 30.000 kg, 바퀴 아닌 링크 무게중심 오차 6e-17 m | |
| W1 주행 보고(duty 0.48, 90 s, GPU 카메라 640×360) | 22.3 m, RTF 1.000, 카메라 30.31 fps | 23.7 m, RTF 1.000, 카메라 30.30 fps | 보고 |
| 단위 시험 3.12(pytest, W1 포함) | 47 통과 | 57 통과 | |
| 단위 시험 Windows 3.13.2(pytest) / WSL 3.10.12(unittest) | 46 통과·1 건너뜀 / 47 실행·1 건너뜀 | 56 통과·1 건너뜀 / 56 통과·1 건너뜀(둘 다 W1 조립은 SIM 컨테이너 필요) | |
| A10(W0), 240 s씩, GUI 끔, GPU 경로 | 카메라 없음 RTF 중앙값 1.000 / 640×360 1.000, 30.30 fps | 카메라 없음 RTF 중앙값 1.000(p10 0.99999, 최소 0.99993) / 640×360 1.000(p10 0.999998, 최소 0.99997), 카메라 7,248장 30.30 fps | 통과(기준 0.9) |

- A10(W0)은 나머지 묶음 뒤 따로 돌렸다(`fix_wf2b/a10/a10_summary.json`, 기록 `a10.log`, 4 CPU, 4 GiB, 예산 잡고 풂). 그 사이 `scene_spec.py`(저장소 가드의 경로 읽기, Gazebo 모듈은 쓰지 않음)와 문서가 바뀌어 출처의 파일 SHA 묶음은 `7e3a4c55…`(904개)다. 커밋은 같은 `d6f0eac` + 작업 트리 변경이다.
- W1 주행 보고를 처음 다시 돌릴 때 GPU 장치 없이 돌려 카메라를 소프트웨어로 그렸다(RTF 0.19, 90 s에 3.7 m). 비교할 수 없는 값이라 판정·보고에 쓰지 않는다(`acc/w1_drive/`). GPU 경로로 다시 돌린 값(`acc/w1_drive_gpu/`)을 위에 적었다.
- F8 확인(이미지를 만들지 않는 BuildKit `--output type=local`, 기록 `fix_wf2b/ctxcheck/ctxcheck.log`): 새 dockerignore로 `COPY firmware` 성공(24파일). 같은 ignore로 `COPY docs`는 거부(ignore 파일이 적용됨을 확인). 고치기 전 dockerignore로 `COPY firmware`는 실패.
- F9: `runs/wf2b/p5_model`, `p5_demo`의 텍스트 1,714개 가운데 BOM이나 CRLF가 있던 44개(BOM 25개)를 BOM 없는 UTF-8, LF로 바꿨다. 파일마다 바꾸기 전후 SHA는 `fix_wf2b/CONVERT_TEXT_FILES.json`에 있다. 다시 검사하니 0개다. dry/·scenes_win/의 `world.sdf`·`world_metadata.json`은 옛 gz_assemble이 Windows에서 고정 생성기로 쓴 것이고, 다른 기록에 그 SHA가 없다. 이번 로그는 bash 리디렉션과 `write_bytes`로 썼다.

## 12. 3단계 갱신: 구 충돌 기본, 이미지 wf3, 프로필 v0.1 재측정 (2026-11-09, g_update)

- 작업 표지 g_update(WF-3b). 결정 R13-1(바퀴 충돌 기본을 구로), R14(프로필 v0.1), SIM_ACCEPTANCE 개정 2·3·5를 따른다.
- 모든 수치는 자리값 기반 SIM 결과다. 현장 검증이 아니다. v0.1의 A4(G) release 통과는 독립 검증이 아니라 맞춤 확인이다(개정 5). v0의 A4(G) release 실패(0.943 m)는 그대로 남는다(7·11절).
- 원 기록은 저장소 밖 `<runs>/wf3b/g_update/`에 있다. 묶음별 `result.json`, `logs.npz`, 세 변형 비교 `SUMMARY_G.json`, A12 입력 `A12_INPUT_G.json`, 빌드 기록 `build_*.log`.

### 12.1 바뀐 코드

- `gz_model.py`: `DEFAULT_WHEEL_COLLISION = 'sphere'`. `steer_model_element(wheel_collision=None)`은 구다. `cylinder`는 옵션으로 남는다. 시각 요소는 둘 다 원통이고 충돌 자세(축 = 충돌 좌표 z)·마찰·fdir1은 같다. manifest에 `wheel_collision`(모양)과 `wheel_collision_choice`(모양, 출처 default/model_option, 기본값, 시각 요소, 결정 번호)를 적는다. CLI `--wheel-collision`을 더했다.
- `gz_assemble.py`: CLI `--wheel-collision cylinder|sphere`(장면 하나를 원통으로 만들 때).
- `gz_probe.py`:
  - 힘 모드 A1 묶음이 δ별 η(δ)(K = 0, 개정 3)의 곡률·반경 잔차와 선형 맞춤 η(δ) = 1 − c·|δ|를 함께 낸다(`per_delta_eta_summary`).
  - 회전 중 속도 손실 검사(`turning_speed_loss`, 보고): δ·목표 속도마다 1 − 원 위 속도 / 목표 속도와, 같은 δ의 F-SIM 끌림 손실 범위(`scrub_speed_loss_per_deg`: 뽑기 U(0, 0.006)/°, 프로필 범위 최대 0.01/°)를 나란히 둔다.
  - `a25_firmware` 묶음(보고, `REPORT_GROUPS`): firmware 모드 백엔드(`FirmwareSteeringBackend`, 펌웨어 소스 + 구동기·가변저항 모델)가 조향 관절을 몬다. 최소 호스트다: 보정된 EEPROM 이미지(11.11절 방식), 10틱에 `ARM <boot_id>`, 20틱부터 50틱마다 새 q의 `C q 300 L R S`(S = 목표각 × 보정 눈금 ‰/°, L = R = 255 × duty). 바퀴는 MCU 구동 duty × ω0를 받는다. MCU 줄 요약(결함 F, 거절 N, 마지막 상태 줄)을 `run.controller_summaries`에 남긴다. 등록 A25 판정은 그대로 `a25_l0`, `a25_native`다.
  - `rtf`에 `--model-option`(예: `wheel_collision=cylinder`)을 더했다. 결과에 `wheel_collision`, 달린 거리(`truth_path_m`)를 적는다.
- `gz_direct_drive.py`: 정지 그림 둘째 줄, 그래프 제목, `run.json`·`circles.json`에 바퀴 충돌 모양을 적는다(전후 그림을 구별하려고).
- `docker/Dockerfile.ubts-steer`: manifest의 `image_tag_hint`를 빌드 인자 `IMAGE_TAG_HINT`(기본 `ubts-steer-sim:wf3`)로 바꿨다. 예전에는 `ubts-steer-sim:p5`로 고정이었다.
- 단위 시험 `sim/ubts/test_steering_gz_model.py`: 기본이 구인지, 원통 옵션, manifest의 선택 기록, δ별 η 잔차와 선형 맞춤, 회전 손실 표, `a25_firmware` 정의, firmware 모드 호스트를 Gazebo 없이 3 s 돌리는 시험(ARM 받음, 결함·거절 줄 0, 10° 목표에 등가각 10 ± 2.5°, C 줄의 duty가 바퀴 목표가 됨), 그림 글자의 충돌 표시, Dockerfile의 태그 인자. 이미지 안에서는 `docker/` 폴더가 없으므로 Dockerfile 검사는 이유를 남기고 건너뛴다(빌드 뒤에 고침, 12.3).

### 12.2 이미지 `ubts-steer-sim:wf3`

- 이미지 ID `sha256:01125b6287d8d4589787579e556c93f2859567ba21413dae50bef6a3eefb3488`(2026-11-09T02:25:30Z). 이 브랜치의 새 이미지 ID 상한(WF-3b 2개) 가운데 하나다. `ubts-steer-sim:p5`(`e440445947ff`)는 그대로 둔다.
- 기반 이미지 ID: 빌드 전후 모두 `sha256:6209168a3507…`(같음). 이미지 ID 수 122 → 123, 새 ID는 wf3 하나다.
- 빌드 문맥: 작업 트리를 그대로 쓰지 않았다. 같은 작업 트리에서 WF-3a가 P4 수정을 하고 있었고 커밋 전이다. 그래서 HEAD `0c1311e`의 파일(git cat-file 원 바이트)에 이 작업이 맡은 파일 7개(`gz_model.py`, `gz_assemble.py`, `gz_probe.py`, `gz_direct_drive.py`, `test_steering_gz_model.py`, Dockerfile, dockerignore. dockerignore는 바뀌지 않음)만 작업 트리에서 덮었다(`runs/wf3b/g_update/make_snapshot.py`, `snap_wf3/SNAPSHOT.json`). 문맥 사본 폴더는 뒤에 지웠다. HEAD와 SNAPSHOT.json·이미지 manifest의 파일 SHA로 다시 만들 수 있다. WF-3a가 고치던 6개(`native/fw_session.cc`, `fw_session.h`, `lockstep_v2.py`, `steer_config.cc`, `steer_config.h`, `conformance/run_conformance.py`)는 HEAD 그대로다. 나머지 파일은 작업 트리와 바이트가 같았다.
- status SHA: 위에서 바뀐 6개 파일(dockerignore 제외)만 바뀐 트리가 낼 `git status --porcelain` 줄(6줄)의 SHA-256 `ce8f7c6bd230…`. 컨테이너에도 같은 값을 넘겼다.
- 이미지 안 manifest(`/opt/ubts/STEER_IMAGE_MANIFEST.json`, 사본 `runs/wf3b/g_update/image_wf3/`): `image_tag_hint` ubts-steer-sim:wf3, `source_commit` 0c1311e2d9aa…, `source_status_sha256` ce8f7c6bd230…, 파일 905개, 파일 SHA 묶음 `0ecd2b2fd4ee2516…`(스냅숏과 905개 모두 같음), 네이티브 빌드 키 `7c158dbbb7b3df17`(g++ 13.3.0, 바이너리 SHA `0033e5710b02…`, 네이티브 소스 SHA `b2be888b9a21…` = R15 P4 기준선), `firmware_build_id` 04353B49(= R15 기준선), `firmware_required` true.
- firmware_build_id 검사가 빌드를 멈추는지(이미지를 만들지 않는 `--output type=cacheonly`, 기록 `build_check_*.log`):

| 임시 문맥 | 결과 |
|---|---|
| `firmware/` 없음 | `COPY firmware`에서 멈춤(`"/firmware": not found`). 검사보다 앞에서 멈춘다 |
| `firmware/`는 있고 `ubts_steer_v3/` 없음 | 네이티브는 firmware 모드 없이 빌드되고(`firmware_build_id None`) 검사가 `firmware_build_id missing in BUILD_RESULT.json …`으로 빌드를 멈춤(종료 1) |
| 같은 문맥 + `UBTS_STEER_REQUIRE_FIRMWARE=0` | 빌드 통과. manifest `firmware_required` false(대조 실험) |

  세 빌드 모두 이미지를 만들지 않았다(이미지 ID 수 122 그대로). 둘째 줄은 이 검사의 메시지로 멈췄고, 같은 문맥에서 검사를 끈 셋째 줄은 통과했다. 그래서 둘째 줄을 멈춘 것은 이 검사다. 임시 문맥 사본은 확인 뒤 지웠다.
- 실행(컨테이너, 소스를 붙이지 않음):

```powershell
python -I <작업 폴더>/tools/docker_budget.py acquire --label <label> --gib 4
docker run --rm --cpus 4 --memory 4g --network none --name ubts-steer-<label> -w /opt/ubts/sim/ubts `
  -e UBTS_STEER_SOURCE_COMMIT=<commit> -e UBTS_STEER_SOURCE_STATUS_SHA256=<status sha> -v <runs 폴더>:/runs `
  ubts-steer-sim:wf3 python3 -B -m steering.gz_probe run a1_kinematic a1_force a4_drive a21_limits a25_l0 a25_native `
  --out /runs/acc
python -I <작업 폴더>/tools/docker_budget.py release --label <label>
```

  - 원통 비교: `... run <묶음> --out /runs/acc_cyl --model-option wheel_collision=cylinder`.
  - 장면: `python3 -B -m steering.gz_assemble --scene w1_force --out /runs/scene [--wheel-collision cylinder]`.
  - A10(W0)·W1 주행은 5절의 GPU 설정(`/dev/dxg` 등)을 더한다: `... gz_probe rtf --out /runs/a10 --duration 240 --camera both`, `... gz_probe drive --scene-dir /runs/scene --out /runs/w1 --duration 90 --steer-deg 0 --duty 0.48`.

### 12.3 시험

| 환경 | 결과 |
|---|---|
| Windows Python 3.13.2, pytest(작업 트리) | 61 통과, 1 건너뜀(W1 조립은 SIM 컨테이너 필요) |
| WSL Ubuntu-22.04 Python 3.10.12, unittest(작업 트리) | 62 실행, OK, 건너뜀 1(같은 이유) |
| 이미지 wf3 Python 3.12.3, pytest(이미지 소스, W1 포함) | 61 통과, 1 실패: Dockerfile 검사가 이미지 안에 없는 `docker/` 파일을 읽었다(환경 문제). 시험에 건너뜀 조건을 더하고 같은 이미지 소스에 고친 시험 파일로 다시 돌려 61 통과, 1 건너뜀 |

- 이미지 안 시험 파일은 고치기 전 판이다(새 이미지 ID를 아끼려고 다시 빌드하지 않았다). 다음 이미지 빌드 때 들어간다.

### 12.4 다시 잰 결과: 세 변형 비교

- v0 + 원통: WF-2b 자격 실행(`runs/wf2b/qualify_wf2b/gz_acc`, 이미지 p5 + 작업 트리, 커밋 d6f0eac).
- v0.1 + 원통: 이번 실행, `--model-option wheel_collision=cylinder`(`acc_cyl`).
- v0.1 + 구: 이번 실행, 새 기본(`acc_sphere`). 판정은 이 열로 한다.
- 정의는 6절(개정 2) 그대로다. 둘 다 이미지 wf3의 자기 소스로 돌았다(결과 `provenance.source.image_manifest.running_the_image_sources` true).

| 검사 | v0 + 원통 | v0.1 + 원통 | v0.1 + 구 | 판정(v0.1 + 구) |
|---|---|---|---|---|
| A1(G) 기구학: R 오차 최대 | 0.0186 % | 0.0186 % | 0.0182 % | 통과(≤ 3 %) |
| A1(G) 기구학: ω 오차(명령 v 기준) 5/10/20° | +0.67 / +1.31 / +2.47 % | +0.67 / +1.31 / +2.47 % | −0.001 % (모두) | 통과(±5 %) |
| 출발 방향 무관(기구학 / 힘 모드) | 1.1e-11 / 5.4e-12 | 1.1e-11 / 8.4e-12 | 1.1e-11 / 9.8e-12 | 통과(±1 %) |
| 요레이트 부호, 관절 속도 튐 | 통과 | 통과 | 통과 | 통과 |
| A1(G) 힘: R/R_geom 5/10/20° | 1.013 / 1.026 / 1.052 | 1.013 / 1.026 / 1.052 | 1.013 / 1.026 / 1.052 | 보고 |
| 등록 형식 맞춤 L_eff, η, K, 곡률 잔차 최대 | 0.790 m, 0.962, −0.070, 2.23 % | 0.787 m, 0.966, −0.030, 2.33 % | 0.785 m, 0.968, −0.014, 2.26 % | 보고 |
| δ별 η(K = 0) 5/10/20°, 곡률 잔차 최대 | 0.9871 / 0.9747 / 0.9502, 0.0033 % | 0.9871 / 0.9747 / 0.9502, 0.0032 % | 0.9871 / 0.9747 / 0.9502, 0.0033 % | 보고 |
| 회전 중 속도 손실 0.3 m/s, 5/10/20° | 10.3 / 22.7 / 52.5 % | 3.9 / 8.9 / 21.5 % | 0.65 / 2.5 / 9.5 % | 보고 |
| 같은 값 0.5 m/s | 6.3 / 14.0 / 33.0 % | 2.4 / 5.7 / 14.4 % | 0.51 / 2.0 / 7.4 % | 보고 |
| A4(G) 정상 속도 40/70/88 % | +0.05 / −0.16 / −0.22 % | +0.05 / −0.16 / −0.22 % | +0.05 / −0.16 / −0.22 % | 통과(±5 %) |
| 맞춘 DC 직선 r·ω0, d0 | 0.93724, 0.04834 | 0.93724, 0.04834 | 0.93723, 0.04834 | |
| 90 % 도달 시간 | 1.55 s | 0.56 s | 0.57 s | 보고 |
| A4(G) release 정지(0.89 m/s에서) | 0.943 m(2.53 s) | 0.614 m(1.36 s) | 0.614 m(1.36 s) | 통과(0.476~0.690 m). 맞춤 확인(개정 5) |
| A4(G) brake 정지 | 0.504 m(1.97 s) | 0.179 m(0.71 s) | 0.179 m(0.71 s) | 보고 |
| (실험) coast 우회 | 4.14 m(6 s에 서지 않음) | 2.16 m(4.82 s) | 2.16 m(4.82 s) | 보고 |
| A21(G) 넘음 최대 | 1.1e-9° | 1.1e-9° | 2.0e-9° | 통과(1e-6 rad) |
| A25 L0: p99, 정지 60 s 표류(0° / 20°) | 8.2e-10°; 1e-10 m / 1.7e-9 m | 8.2e-10°; 1e-10 m / 4.2e-9 m | 4.5e-11°; 1.9e-10 m / 1.3e-9 m | 통과 |
| A25 네이티브 L1: p99, 정지 표류(0° / 20°) | 8.6e-10°; 2.8e-14 m / 1.7e-5 m·4.2e-4° | 8.6e-10°; 2.8e-14 m / 1.1e-7 m·7.9e-6° | 7.2e-11°(최대 9.1e-6°); 9.6e-10 m / 3.8e-8 m·2.8e-6° | 통과 |
| A25 firmware 모드(보고) | — | — | 7.6e-11°(최대 9.1e-6°); 9.6e-10 m / 3.3e-8 m·2.6e-6°. MCU 줄은 아래 | 보고(기준 이하) |
| 관절 속도 최대(조향 / 구동, L0) | 0.612 / 2.89 rad/s | 0.612 / 2.91 rad/s | 0.612 / 3.26 rad/s | 한계 0.785 / 8.37 안 |
| (진단) fdir1 이방성 | 8.1e-11 m, 효과 11.7 m, 통과(fix_wf2b) | — | 출발 방향 궤적 차 1.37e-6 m, 효과 11.9 m. 1e-6 m 허용을 넘어 `passed` false | 진단 |
| A10(W0) 카메라 없음 / 640×360 | 1.000 / 1.000(30.30 fps) | 재지 않음 | 1.000 / 1.000(30.30 fps) | 통과(≥ 0.9) |
| W1 주행(duty 0.48, 90 s, 카메라 640×360) | 23.7 m, RTF 1.000, 30.30 fps(fix_wf2b) | 32.0 m, RTF 1.000, 30.30 fps | 31.9 m, RTF 1.000, 30.29 fps | 보고 |

- 두 변화의 몫:
  - 프로필 v0.1(τ_stall 1.6 → 4.4 N·m)은 구동 쪽만 바꾼다: 출발 시정수(90 % 도달 1.55 → 0.56 s), release·brake 정지 거리, 회전 중 속도 손실의 대부분(원통에서 52.5 → 21.5 %, 0.3 m/s·20°). 기구학 모드(속도 명령)와 곡률(η)은 바뀌지 않는다.
  - 구 충돌은 회전 쪽만 바꾼다: 기구학 모드의 속도 편향(+2.47 → −0.001 %), 회전 중 속도 손실의 나머지(21.5 → 9.5 %). 곡률(R/R_geom, δ별 η)과 직진(정상 속도, 정지 거리)은 바뀌지 않는다.
- 회전 중 속도 손실과 F-SIM: F-SIM 끌림 손실은 run마다 U(0, 0.006)/° × |δ|라 5/10/20°에서 0~3 / 0~6 / 0~12 %다(프로필 범위 최대 5 / 10 / 20 %). v0.1 + 구의 G-SIM 손실(0.3 m/s 0.65 / 2.5 / 9.5 %, 0.5 m/s 0.51 / 2.0 / 7.4 %)은 여섯 칸 모두 그 뽑기 범위 안이다. v0.1 + 원통은 0.3 m/s에서 세 칸 모두, 0.5 m/s에서 20°가 범위 밖이었다. G-SIM 손실은 δ에 대해 직선이 아니다(0.3 m/s에서 °당 0.13 / 0.25 / 0.48 %). F-SIM은 °당 일정하다.
- 원통의 모서리 접촉 편향(8절)은 v0.1에서도 그대로다(기구학 ω +2.47 %). 그래서 구 기본은 회전 결과를 바꾼다. 실차 타이어를 잰 값이 아니다.

### 12.5 A12에 넘기는 A1(G) 힘 모드 맞춤 (v0.1 + 구)

- 기록: `runs/wf3b/g_update/A12_INPUT_G.json`(맞춤, 행, 출처). W0, a = 0.5 링크, 바퀴 마찰 0.9, 120 Hz, 뒤차축 중심 원 맞춤 8~48 s.
- 등록 형식 κ = tan δ / (L_eff + K v²): L_eff 0.7847 m, η = L/L_eff 0.9685, K −0.0139 s²/m(K_us = K·η −0.0135). 곡률 상대 잔차(5°/0.3, 5°/0.5, 10°/0.3, 10°/0.5, 20°/0.3, 20°/0.5, −10°/0.3): +1.77, +1.48, +0.49, +0.21, −2.01, −2.26, +0.49 %. 반경 잔차 −0.155 ~ +0.050 m. 이 형식은 δ 의존을 담지 못한다(개정 3).
- δ별 η(δ), K = 0(개정 3이 허용): 5° 0.98712, 10° 0.97467, 20° 0.95021. 곡률 상대 잔차 최대 0.0008 / 0.0014 / 0.0033 %, 반경 잔차 최대 7e-5 m. 속도 의존(0.5 m/s 반경 / 0.3 m/s 반경 − 1)은 1.6e-5 / 2.1e-5 / 6.5e-5.
- 선형 맞춤 η(δ) = 1 − c·|δ|(°), c = 0.0025021/°. η 잔차 −3.7e-4, −3.1e-4, +2.5e-4(5/10/20°). 5° 아래는 재지 않았다. 사이 값은 |δ|에 대해 직선으로 잇는다(η(0) = 1, A12를 위한 선택).
- 회전 중 속도 손실(1 − 원 위 속도/목표 속도): 0.3 m/s 0.65 / 2.54 / 9.52 %, 0.5 m/s 0.51 / 1.97 / 7.43 %(5/10/20°).
- 원통으로 다시 맞춰야 하면 같은 폴더 `acc_cyl/a1_force/result.json`이다(δ별 η는 같고 손실이 다르다).

### 12.6 firmware 모드로 Gazebo 관절 몰기 (보고)

- 처음으로 firmware 모드(펌웨어 04353B49)가 Gazebo 조향 관절을 몰았다. 관절은 모델 각을 그대로 따른다(p99 7.6e-11°, 최대 9.1e-6°). 표류와 속도 튐 검사도 기준 안이다.
- MCU 줄(차 5대, 64 s, C 줄 1280개씩, CRC 모두 맞음):
  - `sine_moving`, `steps_moving`, `hold_standstill_0`, `hold_standstill_turned`: 끝까지 ARMED, 결함 줄 0, 거절 줄 0.
  - `steps_standstill`(정지, 0 → +20°(2 s) → −20°(12 s) → +5°(22 s) → … 목표): 23.755 s에 `F THERMAL 5001`(30 s 창 안에서 |PWM| ≥ 80 %가 5001 ms). 그 뒤 MCU는 DISARMED였고 남은 C 줄 805개를 모두 `N q state`로 거절했다. 모델의 실제 조향 출력(참값 `applied_steer_pwm`, 물리 주기)으로 세면 |PWM| ≥ 0.8인 시간이 첫 계단 1.18 s, 둘째(40°) 2.68 s, 셋째(25°, 결함까지) 1.14 s, 합 5.00 s다. 정지 중 큰 계단 세 번이 자리값 THERMAL 프록시(80 % / 30 s / 5 s, 규격 P10)를 넘긴 것이다. 결함 순간 조향은 −5.7°(목표 +5°)였고 그 뒤 출력이 꺼진 채 약 −4.8°에 머물렀다. R15에 적힌 남은 경로(정지 중 큰 조향이 THERMAL로 끊김)와 같은 종류다. A19(오검출률) 보고 대상으로 넘긴다. 관절 추종 판정에는 영향이 없다(관절은 멈춘 모델 각을 따른다).
- 2026-11-09 리뷰(RG-01): 이 THERMAL은 정상 대본(결함을 노리지 않는 대본)의 기대하지 않은 결함 줄이다. R15-3 정의로 A19(G) 오검출 1건이다. `p6a_checks a19g_normal` 요약이 이 결과 파일을 입력으로 받아 센다(`P6A_HEADLESS_KO.md` 9.9절).
- 호스트는 최소 대본이다(브리지, STOP·TTL 처리, 재무장 없음). A17·A19 판정이 아니다.

### 12.7 빠른 SIM: P3 커밋과 프로필 바꿈 뒤

- 이미지와 같은 스냅숏 트리(HEAD 0c1311e + 이 작업 파일, F-SIM 코드는 HEAD)에서 Windows Python 3.13으로 돌렸다(`runs/wf3b/g_update/fsim_snap/`, 기록 `run_fsim.sh`).
- A1(F) direct·harness 8 에피소드씩 통과. A4(F) 개정 1 판정 표본: P_fit1003 중앙값 0.5830 m(n 970), field1010 0.5652 m(n 900), 통과. A8: 작업자 1로 두 번, 6으로 한 번, 16 trace 같음, 통과.
- WF-2b 자격 실행(`runs/wf2b/qualify_wf2b/fsim`)과 trace 비교(`compare_fsim_snap_vs_qualify_wf2b.json`): a1_direct 8, a1_harness 8, a4_judgement 8027, a8 16 × 3의 trace에서 숫자 열(meta_json 말고 모든 열)이 비트까지 같다. 다른 것은 meta의 `episode_scene_sha256` 하나다. 장면 명세가 프로필 id·SHA를 담기 때문이다. `drive_tau_stall_nm`는 F-SIM에서 쓰이지 않는다.
- 공유 작업 트리(그 시각의 WF-3a 수정 포함)에서는 돌리지 않았다. WF-3a의 P4 수정이 커밋 전이었고 그동안에도 바뀌고 있었다(F-SIM은 돌리는 중 코드가 바뀌면 그 에피소드를 `code_identity_changed`로 실패시킨다). 그 수정의 영향은 WF-3a의 자격 실행에서 본다.

### 12.8 W1 주행과 A10 (보고·판정)

- A10(W0), 이미지 wf3, D3D12/Ogre1(GPU) 경로, GUI 끔, 240 s씩, 앞 20 s 빼고 5 s 창(6절). 차는 δ 10°, 목표 0.4 m/s로 원을 돈다.
  - 카메라 없음: RTF 중앙값 1.0000, p10 0.999997, 최소 0.99998. 통과.
  - 카메라 640×360: 중앙값 1.0000, p10 0.999994, 최소 0.99998, 카메라 7,239장, 30.30장/s(벽시계·SIM). 통과.
  - 240 s에 93.0 m를 돌았다(v0 + 원통은 76 m). 회전 중 속도 손실이 줄어서다. Embree LiDAR, ROS 브리지는 이 측정에 없다(P6a).
  - v0.1 + 원통의 A10은 재지 않았다. 충돌 모양이 RTF에 주는 영향은 이번 구 결과(1.000)로만 본다.
- W1 주행(보고): 원본 과수원, 조향 0, duty 0.48(평지 목표 0.4 m/s), 벽시계 90 s(SIM 약 87 s), 차량 카메라 640×360, GPU 경로. 이 명령은 앞부분을 빼지 않아 최소값에 시작 구간이 들어간다.

| 변형 | 간 거리 | RTF 중앙값 / p10 / 최소 | 카메라 |
|---|---|---|---|
| v0 + 원통(fix_wf2b `acc/w1_drive_gpu`) | 23.7 m | 1.000 / 0.996 / 0.51 | 30.30장/s |
| v0.1 + 원통(`w1/drive_cylinder`) | 32.0 m | 1.000 / 0.99997 / 0.49 | 30.30장/s |
| v0.1 + 구(`w1/drive_sphere`) | 31.9 m | 1.000 / 0.996 / 0.38 | 30.29장/s |

  - 거리 차이는 τ_stall에서 온다(요철에서 속도를 지킨다). 직진에서 충돌 모양의 차이는 0.1 m다. 장면의 GNSS 원점은 지웠고 `gnss_lint`는 비었다.
  - W1에서 구가 열 사이 요철·기둥 옆면에 닿는 방식이 원통과 다를 수 있다. 바퀴 뜸·옆면 접촉은 따로 재지 않았다.

- 같은 모델(v0.1, 구)로 만든 시연 그림·영상은 `GZ_DEMO_KO.md` 10절에 있다.

### 12.9 남은 것과 시험하지 않은 것

- fdir1 진단은 구에서 허용(1e-6 m)을 1.37e-6 m로 넘었다. 이방성 효과(11.9 m)에 비하면 1e-7 수준의 수치 차이로 본다. 허용은 원통 측정 뒤 정한 값이고 결과를 본 뒤 바꾸지 않았다(제안: 효과에 대한 상대 허용).
- firmware 모드 A25는 최소 호스트로만 쟀다. 브리지 코어·T1 장치(WF-3a)로 모는 것은 P6a·P4 범위다.
- A10(W0) v0.1 + 원통, W1에서 구의 옆면 접촉(열 사이 기둥·요철)과 원통의 차이, 이동 중 W1 RTF 장시간 측정은 하지 않았다.
- 구 충돌은 SIM 모델 선택이다. 실제 타이어 모양·접지 면적·미끄럼 강성은 모른다.
- 실차. 모든 값은 자리값이다.
