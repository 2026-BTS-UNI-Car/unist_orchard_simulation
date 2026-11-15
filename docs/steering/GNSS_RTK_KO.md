# GNSS RTK 약한 제약 (`weak_constraint`)

- 작성: 2026-11-16. 근거: 사용자 결정(2026-11-14, RTK 보정을 쓸 계획), 설계 `DESIGN_v2_KO.md` V2-1 3항(L3 모드 `weak_constraint`), `GNSS_LAYER_KO.md` 2절(그때는 만들지 않음), 결정 R100, 합격 기준 개정 100.
- 코드: `tools/row_executor/gnss_rtk.py`(제약), `perception_settings.py`(`RtkSettings`, 설정 검사), `perception_layers.py`(연결), `config.py`의 `gnss_rtk_*`, `core.py`(조향 제어기 입력), `gnss_layer.py`(`end_reason`). SIM: `sim/ubts/steering/gnss_sim.py`의 RTK 수신기 모형, `tools/row_executor/sim_in_loop_steer.py`(fix 종류 전달), 행렬 `steering/safety_scripts/rtk_matrix.py`.
- **기본은 꺼짐이다.** `gnss_mode`의 기본은 `'off'`이고, `weak_constraint`는 현장 설정 파일에서 일부러 켜야 한다. 끄면(그리고 `log_only`, `geofence`에서도) 이 문서의 코드는 하나도 돌지 않는다.
- 모든 값은 자리값이다. SIM 수치는 **자리값 기반 SIM 결과**다. 과수원 나무 아래에서 실제 RTK 수신기가 FIXED를 얼마나 오래 유지하는지, FLOAT·잘못된 FIXED가 얼마나 자주 나오는지는 잰 적이 없다. 이 문서와 코드·기록에 GNSS 좌표는 없다.

## 1. 무엇을 하나

RTK FIXED 해는 몇 cm 오차다. 일반 fix(10/10 기록 수평 정확도 p50 5.3~10.0 m)와 달리 열 간격(3.6~4.1 m) 안의 횡 위치를 말할 수 있다. 그래서 이 모드에서만 GNSS가 **조향 제어기의 입력 하나**, 열 중심선에서의 횡오차(`lateral_offset`)를 조금 당길 수 있다.

손대지 않는 것:

- 추정기(통로 band의 중심·방향·폭, 열 끝 증거). GNSS는 추정기에 아무것도 쓰지 않는다.
- 방향 오차 입력, 구동 명령, 속도 상한, 열 끝 판정.
- 잃은 통로를 살려 두기. LiDAR 통로가 없으면 보정도 쓰이지 않는다(제어기는 원래대로 통로 잃음 정지).
- 기하 가드(`guard_lateral`·`guard_heading`), 실행기 기록의 스냅숏, 모든 정지 규칙. 이것들은 LiDAR 원값을 본다.

| 모드 | 하는 일 |
|---|---|
| `off`(기본) | 없음 |
| `log_only` | 점검·기록만(`GNSS_LAYER_KO.md` 3절) |
| `geofence` | `log_only` + 열 범위 펜스로 run 끝 |
| `weak_constraint` | `geofence` + 이 문서의 RTK 약한 제약. `vehicle_profile` `front_steer`에서만 받는다 |

## 2. 무엇을 묶나: run 국소 좌표의 열 중심선

- 원점: run에서 처음 쓴 RTK fix. 원점은 메모리에만 있고, `rtk` 상태 블록에는 원점에서 잰 위치도 남기지 않는다(보정량, 불일치, 선의 점 수·길이·잔차만). 일반 층의 `rel_m`(run 첫 fix 기준 국소 미터, 0.1 m 반올림)은 전과 같다.
- 방향: 실행기 odometry 궤적과 RTK 궤적을 2D Kabsch로 맞춘 회전 α(odometry → RTK 좌표). 경로 `min_span_m`(8 m) 뒤부터 fix마다 창 전체로 다시 맞춘다.
- 차체 기준점: 안테나 위치에서 레버암(`gnss_rtk_antenna_x_m`, `_y_m`, 차체 좌표)을 뺀다. 방향은 odometry yaw + α. 레버암이 틀려도 일정한 부분은 학습한 선에 같이 들어가 지워진다(단위 시험: 0.5 m·0.2 m 틀려도 보정 차이 0.05 m 미만. 남는 것은 방향 흔들림(약 2°) × 레버암과 그것이 바꾼 학습).
- 열 중심선 학습: LiDAR 통로가 유효하고 **두 쪽**으로 잡힌 때의 fix마다, 기준점에서 LiDAR 횡오차만큼 법선 방향으로 옮긴 점이 중심선의 한 점이다. 지난 `window_m`(20 m) 경로에서 배운 점들에 직선을 맞춘다. 방향은 앞 1/3과 뒤 1/3의 중앙값 점을 잇는 방향, 위치는 전체 중앙값 점이다(한쪽 끝의 몇 점이 선을 기울이지 못하게). 점 `min_points`(6) 이상, 길이 `min_span_m`(8 m) 이상, 잔차 RMS `max_rms_m`(0.05 m) 이하일 때만 선으로 쓴다. 굽은 열이나 나쁜 자료면 선이 없다. 선이 생긴 뒤에는 경로 `refresh_m`(5 m)마다만 다시 맞추고, 그 사이에는 고정한다(새 점이 자기를 판정하는 선을 움직이지 못하게).
- 즉 선은 "LiDAR가 잘 보던 동안의 통로 중심을 RTK 좌표에 적어 둔 것"이다. 측량한 열 지도나 위도·경도 선은 쓰지 않는다.
- 학습 문: 점은 찍힌 뒤 경로 `learn_delay_m`(2 m)이 지나서야 배울지 정한다. 찍힐 때의 거른 불일치(δ_f, 3절)가 `learn_gate_m`(0.03 m) 안이고, 그 점이 지금 선에서 그 두 배 안이면 배운다. 하나가 거절되면 아직 기다리는 점들도 같이 거절한다(치우침의 시작). 나무가 한쪽으로 몰린 곳이나 선 건너뜀(R90의 W2 `045553_c03`)처럼 LiDAR 중심이 치우친 구간을, 치우치기 전의 선과 비교해 걸러 내기 위해서다. 거른 불일치가 문 밖인 채로 경로 `disagree_reset_m`(8 m)이 지나거나, 거른 불일치가 `max_disagree_m`(0.20 m)를 넘으면 선을 버리고 그 뒤의 LiDAR로 다시 배운다(열이 실제로 옮겨 가거나 휘었을 때, 선이 틀렸을 때 RTK가 오래 붙잡지 않게). 이 규칙들은 개발 run에서 바꿨다(사전등록 문서 5절).
- 방향 문: LiDAR 통로 축(odometry yaw + 방향 오차 + α)이 선과 `heading_gate_deg`(3°)보다 벌어지면 선을 바로 버린다. 굽은 열에서 직선이 통로를 벗어나기 시작하면 이 문이 먼저 걸린다.
- 좌표계 확인: 창의 odometry 궤적과 RTK 궤적이 회전(+평행 이동)으로 `frame_rms_m`(0.05 m) 안에 맞고, 거울(반사)로는 그보다 `frame_ratio`(2)배 넘게 나쁘게 맞아야 한다. 동·북이 바뀐 좌표계는 반대로 반사가 맞고, 축 배율이 틀린 좌표계는 둘 다 안 맞는다. 곧은 궤적은 거울과 구별되지 않으므로 odometry 궤적의 옆 퍼짐이 `min_spread_m`(0.01 m) 이상일 때만, 경로 1 m마다 한 번 본다. 회전이 맞으면 확인(그 run 동안 유지), 반사 판정이 `frame_reject_after`(3)번 나오면 그 run에서는 제약을 거부한다. 확인 전에는 아무것도 적용하지 않는다. (처음에는 RTK 횡위치와 LiDAR 횡오차의 기울기로 보았으나, 창 안에 LiDAR 치우침의 계단이 있으면 거울 좌표계에서도 기울기가 양수가 되어 확인되는 것을 단위 시험에서 보고 바꿨다. 기울기는 보고만 한다.)

## 3. 얼마나 약하게

- fix마다 불일치 δ = RTK 횡오차 − LiDAR 횡오차를 저역 통과(`innovation_beta` 0.3)한다.
- 목표 보정 = clamp(이득 `gnss_rtk_gain` 0.5 × δ_f, ±상한 `gnss_rtk_cap_m` 0.10 m). 선과 좌표계 확인이 있고 마지막으로 쓴 fix가 `gnss_rtk_hold_s`(1.5 s)보다 새것일 때만, 아니면 0.
- 적용 보정 u는 틱마다 목표로 `gnss_rtk_rate_m_s`(0.05 m/s) × 틱 간격까지만 움직인다. 0.10 m까지 2 s 걸린다.
- 제어기는 `lateral_offset + u`를 받는다. 그래서 RTK가 무엇을 말하든 제어기의 횡 목표는 LiDAR 열 선에서 0.10 m보다 멀어지지 않는다. u가 0이면 제어기는 원래 스냅숏 객체를 그대로 받는다(명령이 비트까지 같다).

## 4. 문(gate)과 전이

fix가 쓰이려면 아래가 모두 맞아야 한다. 안 맞으면 그 까닭을 센다(`counts`).

| 순서 | 조건 | 까닭 이름 |
|---|---|---|
| 1 | 해 종류가 `gnss_rtk_fix_types`(기본 `fixed`; `fixed,float` 가능) 안 | `fix_type` |
| 2 | 보고 수평 정확도 ≤ `gnss_rtk_max_hacc_m`(0.05 m) | `hacc` |
| 3 | 일반 층(`gnss_layer`)이 받아들였다(품질, 순서, 3 m 튐 문). 끊김 뒤 다시 시작·계단 뒤 다시 잡음이면 RTK 궤적도 새로 | `layer` |
| 4 | 일반 층이 그 fix로 표시(진행 방향·위치 불일치, 펜스 밖)를 올리지 않았다 | `layer_flag` |
| 5 | odometry 자세가 있다 | `no_pose` |
| 6 | odometry로 예측한 위치와의 차이 ≤ `jump_gate_m`(0.25 m). α 전에는 크기만 비교. 서로 맞는 기각이 `reanchor_after`(5)번 이어지면 수신기 해의 계단으로 보고 궤적·선을 새로 시작 | `rtk_jump`, `track_restart` |
| 7 | LiDAR 통로가 유효하다 | `no_corridor` |

전이:

- FIXED → FLOAT → 없음, 끊김: FLOAT는 기본 설정에서 문 1에 걸린다. 쓰는 fix가 `hold_s` 동안 없으면 목표가 0이 되고 u는 속도 한도로 줄어든다. 계단이 없다(단위 시험과 행렬의 '계단 없음' 확인).
- 잘못된 FIXED(정수 모호성 오판)로 해가 몇 dm 옮겨 가면 문 6이 기각한다. 오래가면 새 해로 다시 배운다(그동안은 선이 없어 u가 0으로 줄어든다).
- 좌표계 거부, 선 버림, 궤적 새로 시작도 모두 u를 속도 한도로 줄인다.
- run이 새로 시작하면(`start_run`) 모든 상태가 지워진다.

## 5. 실제 입력

### 5.1 ZED-F9P가 내는 것

- NMEA GGA 6번 칸 quality: 4 = RTK FIXED, 5 = RTK FLOAT(1 단독, 2 DGNSS). 정확도는 GST(위도·경도 표준편차).
- UBX NAV-PVT: `flags` 6~7비트 `carrSoln`(1 FLOAT, 2 FIXED), `hAcc`(mm). NAV-HPPOSLLH `hAcc`(0.1 mm).

### 5.2 지금 Jetson 드라이버가 내는 것 (대시보드 레포, 읽기만 함)

- `scripts/gps_ros2_driver.py`와 디코더 `dashboard_server/gps.py`. UBX NAV-PVT를 먼저 쓰고(`carrSoln` 2 → quality 4, 1 → 5, 그 밖에 DGNSS 비트 → 2, 아니면 1), 2 s 안에 UBX가 없으면 GGA quality, 그다음 RMC 모드 문자(R → 4, F → 5)를 쓴다.
- `/gps/fix`(`sensor_msgs/NavSatFix`): `status.status` = quality 4 **또는** 5이면 2(GBAS_FIX), 그 밖의 유효 fix는 0, 무효는 −1. `position_covariance` = [hAcc², 0, 0, 0, hAcc², 0, 0, 0, vAcc²], 형식 1(APPROXIMATED). 정확도는 NAV-PVT `hAcc`, NAV-HPPOSLLH가 같은 epoch이면 그 값, NMEA만이면 GST.
- `/gps/data`(`std_msgs/String`, JSON): 디코더 프레임 전체. `quality`, `fix_type`(`rtk_fixed`/`rtk_float` 등), `h_acc_m`가 있다. **위도·경도도 같이 있다.**
- 드라이버는 수신기에 설정을 쓰지 않는다(읽기 poll만). RTCM 보정을 수신기에 넣는 경로는 드라이버에 없다.

### 5.3 실행기 쪽

- `gnss_rtk.fix_type_from_message`: SIM 메시지의 `fix_type`, UBX식 `carr_soln`, NMEA식 `quality`, 마지막으로 NavSatFix `status`(2 → `'rtk'`)를 읽는다.
- 지금 실행기 노드가 구독하는 것은 `/gps/fix`뿐이다(A13 개정 30의 정적 검사가 `executor_main.py`의 `/gps/fix`만 허용). 그래서 실차에서는 FIXED와 FLOAT를 가를 수 없다. `'rtk'`는 FLOAT를 허용한 설정(`gnss_rtk_fix_types` `fixed,float`)에서만 쓰인다. **기본 설정(FIXED만)이면 지금 드라이버로는 제약이 한 번도 걸리지 않는다**(안전 쪽). 정확도 문(0.05 m)은 그대로 걸린다.
- 실차에서 FIXED만 쓰려면 둘 중 하나가 필요하다(사용자·조정 쪽 결정, 이 작업에서는 하지 않음):
  1. 드라이버가 FIXED와 FLOAT를 가르는 값을 위도·경도 없이 따로 낸다(예: 작은 상태 토픽). 실행기는 그 토픽을 구독하고 A13 허용 목록에 더한다.
  2. 실행기가 `/gps/data`를 구독해 `quality`·`h_acc_m`만 읽고 위도·경도는 바로 국소 미터로 바꾼다. 개정 30은 `/gps/data`를 허용하지만 정적 검사의 허용 목록(`EXECUTOR_GNSS_TOPICS`)과 그 시험을 바꿔야 한다.
- 수신기 지연(USB, 5~10 Hz)은 재지 않았다. 지연은 주로 진행 방향으로 나타나고 횡 위치에는 작다(0.4 m/s, 방향 오차 2°, 0.3 s → 약 0.004 m).

## 6. 현장에서 해야 할 일 (사용자 몫)

- 보정원 하나:
  - 기지국: 두 번째 F9P(+ 측량용 안테나)를 열 근처에 두고 survey-in이나 알고 있는 좌표로 고정, RTCM3을 무선(LoRa 등)이나 네트워크로 로버 수신기에 보낸다.
  - NTRIP: 인터넷이 되는 곳에서 보정 서비스 계정으로 RTCM3을 받아 로버 수신기에 넣는다. 계정·비밀번호는 레포·기록·문서에 넣지 않는다.
  - 어느 쪽이든 RTCM을 수신기 UART2나 USB로 넣는 프로그램이 따로 필요하다(지금 드라이버는 하지 않음).
- 안테나 위치를 차체 기준점(base: 프로필 `gnss_antenna_pose_in_base`, 실행기 `lidar_x_in_base_m`와 같은 기준)에서 재서 `gnss_rtk_antenna_x_m`, `_y_m`에 넣는다.
- 열 하나에서 FIXED 비율, FLOAT로 떨어지는 곳, 보고 `hAcc`를 먼저 `log_only`로 기록해 본다(좌표는 기록에 남지 않는다: 실행기 기록은 run 국소 값뿐).
- 5.3절의 FIXED 구분 경로가 정해지기 전에는 `weak_constraint`를 켜도 기본 설정에서는 걸리지 않는다.

## 7. SIM 수신기 모형 (`gnss_sim.py`, 키 `rtk`)

| 키 | 뜻 | 자리값 |
|---|---|---|
| `rtk.default_type` | 창 밖의 해 종류 | `fixed` |
| `rtk.windows` | `[[t0, t1, 종류], ...]` 종류 `fixed`·`float`·`single`·`none` | 없음 |
| `rtk.fixed` | 보고 hAcc, 흔들림, 흰 잡음, Gauss-Markov 떠돎 σ·τ | 0.014 m, 0.002, 0.008 m, 0.006 m·30 s |
| `rtk.float` | 같은 항목 | 0.25 m, 0.05, 0.06 m, 0.20 m·20 s |
| `single` | 일반 수신기 모형(편향 σ 3 m, 떠돎 1.5 m, 흰 0.4 m, hAcc 7 m) | |
| `frame_mirror` | 동·북을 바꾼다(틀린 좌표계) | false |
| `frame_scale` | 축 배율 | [1, 1] |

- `rtk`가 있으면 모든 패킷에 `fix_type`이 붙고, RTK 난수는 일반 수신기 난수 뒤에 뽑아 일반 흐름이 그대로다. `rtk`가 없으면 패킷·난수·보고가 전과 같다(시험 `test_general_receiver_unchanged`가 main의 표본 값으로 확인).
- 행렬의 수신기: 5 Hz(200 ms 주기, 위상 30 ms), 지연 0.1 s. 튐(`jumps`)과 끊김(`dropouts`)은 일반 키를 그대로 쓴다.
- 값의 크기는 F9P급 자료표의 차수에 맞춘 자리값이다. 나무 아래 다중경로, 잘못된 FIXED의 빈도는 모형에 없다(행렬에서는 계단·튐으로 일부러 넣는다).

## 8. 상태와 기록

`gnss` 블록 안의 `rtk`(실행기 틱 기록마다):

- `state`: `learning`(선 또는 좌표계 확인 전), `active`, `fading`, `refused`(좌표계 거부).
- `frame`, `correction_m`(적용 u), `target_m`, `innovation_m`(저역 통과한 δ), `max_abs_correction_m`, `max_step_m`.
- `fix_type`, `fix_reason`(마지막 fix의 까닭), `fix_hacc_m`, `fix_age_s`, `used_age_s`, `line`(점 수, 길이, 잔차 RMS), `slope`, `spread_m`, `counts`.
- 위치·원점은 없다. 대시보드는 이 블록을 그대로 보여 주면 된다(메뉴는 바꾸지 않는다).

## 9. 참값 격리 (A13 개정 100)

- `gnss_rtk.py`는 결정 코드(역할 decision)다. 참값·SIM 모듈을 읽지 않는다. SIM의 RTK fix는 참 안테나 위치에서 합성한 측정이다(`gnss_sim.py`, 역할 plant).
- GNSS 값이 제어기 입력에 닿는 길은 `PerceptionLayers.controller_snapshot` → `RtkConstraint.apply` 하나다. 추정기로 가는 길은 없다.
- 모드 `off`·`log_only`·`geofence`의 기준(개정 30)은 그대로다.

## 10. 시험과 측정

- 단위 시험 `tools/row_executor/tests/test_gnss_rtk.py`: fix 종류 읽기(SIM, NavSatFix, GGA, NAV-PVT), 설정 검사, 일관된 RTK에서 확인·작은 보정, LiDAR 치우침을 상한·속도 한도 안에서 당김, 오래 어긋나면 선 버림, FLOAT·정확도 문, FIXED → FLOAT → 없음에서 계단 없는 줄어듦, 튐 기각과 계단 뒤 새 궤적, 일반 층 결과·표시 문, 거울 좌표계 거부, 레버암 오차, 굽은 열에서 상한 안, run 초기화, front_steer Runner에서 fix 없으면 결정·제어기 입력이 같음, 보정이 제어기 입력에만 들어감.
- SIM 수신기 시험 `sim/ubts/test_steering_gnss_sim.py`의 `RtkModelTest`.
- 사전등록 행렬: [`PREREG_GNSS_RTK_KO.md`](PREREG_GNSS_RTK_KO.md). 결과는 판정 run 뒤 `GNSS_RTK_RESULT_KO.md`에 쓴다.

## 11. 한계와 남은 일

- 직선 모형이다. 굽은 열에서는 방향 문(3°), 잔차 문, 거른 불일치 0.20 m 문이 선을 버리지만, 그 전에는 호와 현의 차이만큼 당길 수 있다(상한 0.10 m 안). 행렬의 `curve_fixed`가 그 크기를 잰다.
- LiDAR가 처음부터 치우친 구간(출발 직후 선 건너뜀 등)은 고치지 못한다. 선은 LiDAR가 잘 본 구간에서만 배운다.
- 실차의 FIXED 구분(5.3절), 수신기 지연, 안테나 레버암, 나무 아래 FIXED 비율은 모두 남은 일이다.
- G-SIM(Gazebo) 가상 GPS에는 RTK 모형이 없다. 빠른 SIM에서만 확인했다.
