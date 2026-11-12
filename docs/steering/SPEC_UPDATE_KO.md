# 조향 장치 사양이 오면 할 일 (ST-17 준비)

- 작성: 2026-11-13 (묶음 S3)
- 고칠 파일은 하나다: `sim/ubts/steering/profiles/steer_spec_input.yaml`
- 돌릴 명령도 하나다: `tools/steer_sim/apply_spec.py`
- 바탕 프로필: `front_steer_v0_2_placeholder`(스키마 v3, `STEER_PROFILE_V02_KO.md`). 질문 번호는 `VEHICLE_SPEC_REQUEST_KO.md`와 같다.
- 사양 값이 들어가도 결과는 "일부 실측 기반 SIM 결과"다. 현장 검증이 아니다. 펌웨어 쓰기, 벤치 시험, 실차 주행, Jetson 설치는 사용자가 현장에서 한다.

## 1. 흐름

```mermaid
flowchart LR
  A["답·사진·실측<br/>(사양 요청서 Q1~Q29, B·F 시험)"] --> B["steer_spec_input.yaml<br/>value·source·date·note"]
  B --> C["apply_spec.py<br/>입력 검사 → 환산"]
  C --> D["새 프로필 후보<br/>(front_steer_v1_spec_날짜)"]
  D -->|"프로필 읽기 검사<br/>범위·의미·가변저항 창"| E{"받음?"}
  E -->|"거부: 코드"| B
  E -->|"받음"| F["계산값·펌웨어 CFG·<br/>호스트 프로필·Gazebo 관절"]
  F --> G["바뀐 값 요약 +<br/>다시 잴 것 목록"]
  G --> H["짧은 확인<br/>native · T1 직진 · T1 외란 · 빠른 SIM 원"]
  H --> I["--install → profiles/에 넣고 커밋<br/>사전등록 다시 · 민감도 다시"]
```

## 2. 무엇을 재서 어느 칸에 넣나

`value`에 값, `source`에 `사용자`(설명·어림값) / `측정`(실측) / `제품표기`(라벨·판매 페이지), `date`(측정이면 필수, YYYYMMDD), `note`(어떻게 얻었는지; 측정·제품표기는 필수이며 프로필의 source가 된다). 모르는 칸은 `null`로 둔다. 바탕 프로필 값이 그대로 남는다.

| 칸 | 질문 | 무엇을, 어떻게 재나 | 단위 |
|---|---|---|---|
| `wheelbase_m` | Q2 | 뒤차축 중심 ~ 앞 조향 회전 중심(B1, 줄자) | m |
| `track_front_m`, `track_rear_m` | Q2 | 좌우 바퀴 접지 중심 사이(B1) | m |
| `kingpin_width_m` | Q2 | 두 킹핀 축 사이, 바퀴 중심 높이(B1) | m |
| `wheel_radius_front_m`, `wheel_radius_rear_m` | Q2 | 바퀴 반경(지름 / 2) | m |
| `steer_type` | Q10 | `linkage`(링크) 또는 `axle_pivot`(앞차축 전체 회전) | — |
| `steer_mech_limit_deg` | Q2 | 바퀴마다 기계 끝까지의 각(B2·B3, 각도기). 숫자 하나면 네 방향 같음, 다르면 `{left_wheel_toward_left: .., left_wheel_toward_right: .., right_wheel_toward_left: .., right_wheel_toward_right: ..}` | deg |
| `steer_ackermann_ratio` | Q2 | B3 정적 각도표에서 안쪽·바깥쪽 바퀴각으로 맞춤(0 평행, 1 Ackermann) | — |
| `steer_backlash_deg` | Q10 | 링크 유격(B6 왕복 히스테리시스) | deg |
| `footprint_length_m`, `footprint_width_m`, `mass_kg` | Q2 | 조향 장치 돌출 포함 발자국, 질량(B1) | m, kg |
| `steer_self_locking` | Q11 | 감속기가 손으로 돌려지지 않으면(웜기어 등) `true` | — |
| `steer_motor_stall_torque_nm` | Q11 | 조향축 기준 정지 토크(라벨 정지 토크 × 감속비 × 효율, 또는 벤치) | N·m |
| `steer_motor_stall_current_a`, `steer_gear_ratio` | Q11 | 라벨·판매 페이지 | A, — |
| `steer_rate_max_deg_s` | Q11 | B5 계단 응답(주행 중 또는 띄움)의 최대 조향 속도 | deg/s |
| `steer_rate_standstill_deg_s` | Q11 | F5 서 있을 때 조향 속도 | deg/s |
| `steer_min_pwm_pct`, `steer_min_pwm_standstill_pct` | Q11 | B4 느린 PWM 램프에서 움직이기 시작하는 PWM(띄움 / 지면) | % |
| `steer_supply_shared_with_drive` | Q26 | 조향 모터가 구동 배터리를 같이 쓰면 `true` | — |
| `belt_type` | Q22 | `timing`(이 있는 벨트) / `flat` / `round` | — |
| `belt_pulley_teeth_pot` | Q22 | 가변저항 쪽 풀리 잇수 | 개 |
| `belt_pulley_teeth_steer` | Q22 | 입력 전용: 조향 쪽 잇수. 벨트 비 = 조향 쪽 / 가변저항 쪽 | 개 |
| `belt_pot_play_deg` | Q23 | 측정 쪽 유격(B6를 가변저항 raw와 바퀴 각으로 같이) | deg(조향) |
| `pot_location` | Q23 | 벨트가 걸린 축이 링크 유격 앞(모터·감속기)이면 `motor`, 뒤(조향 기둥·바퀴 쪽)면 `wheel` | — |
| `pot_range_deg` | Q23 | 가변저항 전기 회전 범위(한 바퀴형은 보통 270~300) | deg |
| `pot_to_steer_ratio` | Q22 | 벨트 비를 바로 알 때 | — |
| `pot_adc_end_raws` | Q24 | 입력 전용: 왼쪽 끝·가운데·오른쪽 끝 ADC `[L, C, R]`(B2). 벨트 비와 `pot_center_raw`로 바뀐다 | count |
| `pot_turn_deg_end_to_end` | Q24 | 입력 전용: 끝에서 끝까지 가변저항이 도는 각. 벨트 비로 바뀐다 | deg |
| `steer_limit_switch` | Q25 | 양 끝 스위치가 있으면 `both`(지금은 모형이 없어 거부된다, 5절) | — |

벨트 비를 주는 칸(`belt_pulley_teeth_steer`, `pot_adc_end_raws`, `pot_turn_deg_end_to_end`, `pot_to_steer_ratio`)이 여럿이면 서로 5 % 안이어야 한다. 아니면 입력을 거부한다.

## 3. 명령

저장소 루트에서(네이티브 빌드는 저장소 밖 `UBTS_STEER_BUILD_DIR`):

```
python -B tools/steer_sim/apply_spec.py --spec sim/ubts/steering/profiles/steer_spec_input.yaml --out <저장소 밖 폴더>
python -B tools/steer_sim/apply_spec.py --spec ... --out ... --install     # 결과가 괜찮으면 profiles/에 넣는다
```

- 출력 폴더: 새 프로필 후보 `<profile_id>.yaml`, `REPORT.json`, `SUMMARY_KO.txt`, `host_profile.json`(브리지용), `firmware_cfg.txt`(CFG 줄).
- `--no-checks`: 짧은 확인을 건너뛴다. `--force`: 같은 이름 프로필을 덮어쓴다(쓰지 않는 것이 원칙: 새 판은 새 이름).
- 종료 코드: 0 모두 됨, 1 짧은 확인 실패, 2 입력·프로필 거부.
- 걸리는 시간: 이 PC에서 짧은 확인까지 약 10초(T1 2회, 빠른 SIM 1회).

## 4. 결과 읽기

| 요약 줄 | 뜻 |
|---|---|
| 바뀐 값 | 입력이 바꾼 칸과 그 칸에서 계산되는 값(예: 소프트 한계, 가변저항 LSB, 이 각, 보정 반쪽 폭). provenance가 `SPEC_PENDING`에서 `MEASURED_<날짜>`·`USER_APPROX`·`PRODUCT_LABEL`로 바뀐다 |
| 범위 넓힘 | 값이 바탕 프로필의 스윕 범위 밖이었다. 범위를 그 값까지 넓혔다. 그 칸으로 잰 스윕 결과는 다시 돌린다 |
| 펌웨어 CFG·보정 표지 | `fw_eeprom.params_from_profile`의 41개 CFG 값과 계산 보정 표지(raw L/C/R)가 어떻게 바뀌는지. 눈금 s(‰/°)가 바뀌면 ‰ 단위 문턱(db, polp, stm, trk, dsm)이 같이 바뀐다 |
| 호스트 프로필 | `host_profile_export.build`의 소프트 한계, 보정표, 문턱 |
| Gazebo 조향 관절 한계 | `gz_model`이 만든 SDF의 조향 관절 한계(rad) |
| 다시 재거나 돌릴 것 | 바뀐 칸 묶음별로 다시 할 검사(아래 표) |
| 짧은 확인 | 네이티브 시작·스텝, T1 직진(개정 40 N1, seed 0), T1 +2.5 N·m 외란(개정 41 D1, seed 0), 빠른 SIM 10° 원(반경 / L·tan δ). 판정이 아니라 연기 시험이다 |

### 4.1 거부될 때

| 코드 | 까닭 | 할 일 |
|---|---|---|
| `out_of_range`(입력 칸) | 선택지 밖(예: `steer_type`이 둘 밖) | 값 확인 |
| `pot_range_exceeded` | 끝 멈춤의 raw가 보정 창(60~963) 밖. 조향 전 범위 × 벨트 비가 가변저항 유효 범위를 넘는다 | 벨트 비를 줄이거나 회전 범위가 넓은 가변저항. 자리값 한계 28°에서 받는 비는 약 0.53~4.2 |
| `pot_cal_half_width_small` | 보정 표지가 가운데에서 50 count 안쪽(펌웨어 `marks_valid`가 보정을 거부) | 벨트 비를 키운다 |
| `not_modelled` | 리밋 스위치 `both` | 배선·핀(ESP8266 남는 핀 D4 하나)과 펌웨어 설계부터. 그다음 모형을 더한다 |
| `standstill_order` | 정지 중 조향 속도 > 주행 중, 또는 정지 중 시작 PWM < 주행 중 | 두 값을 같이 확인 |

거부된 후보는 `<profile_id>.refused.yaml`로 남는다.

## 5. 다시 재야 하는 것

| 바뀐 칸 묶음 | 다시 할 것 |
|---|---|
| 기하(휠베이스, 윤거, 킹핀 폭, 반경, 형식, 기계 한계, Ackermann, 발자국, 질량) | A1(F, G), A12, A21, A24, Gazebo 모델 다시 만들기, 열 추종 사전등록 행렬(`PREREG_STEER_FOLLOW`) |
| 구동기(조향 속도, 시작 PWM, 링크 유격, 자기 잠금, 정지 토크, 급전 공유) | A2(`UBTS_STEER_TEST_PROFILE=<새 파일>`로 `test_steering_native.py`), 서보 최소 출력 간격(`servo_min_pwm_pct` ≥ 시작 PWM + 5 %p), 개정 40 ② 정상 대본 오검출, 개정 41 외란 판정, A18 식별 복원, THERMAL·DRV_STUCK 문턱 |
| 가변저항·벨트(종류, 잇수, 유격, 위치, 회전 범위, 벨트 비, 가운데 raw) | A3(F), 보정 표지 B2·B3, POT_JUMP·POT_NOISE 문턱(count), POLARITY 문턱(polp), 벨트 이 건너뜀 대본(개정 41 R3) |
| 리밋 스위치 | 배선·펌웨어 설계 |

판정에 쓰는 측정은 새 사전등록(기준을 먼저 커밋)을 하고 잰다. 새 프로필로 바꿔도 예전 판의 결과 파일은 그대로 둔다.

## 6. 실차 저장소로 옮길 것

| 무엇 | 어디서 나오나 | 실차에서 | 비고 |
|---|---|---|---|
| 펌웨어 CFG 41개 | `firmware_cfg.txt`(`CFG <key> <value>` 줄) | 벤치에서 DISARMED로 `CFG` 줄을 보내고 B2 보정 뒤 `CAL SAVE`로 EEPROM에 저장(ESP 규격 4.10) | 펌웨어 소스(`firmware/ubts_steer_v3/`)는 그대로. 컴파일 기본값은 자리값이며 EEPROM 레코드가 이긴다 |
| 보정 표지(raw L/C/R)·소프트 한계 | 계산값은 참고. 실기는 B2·B3 실측 | `CAL MARK`·`CAL SOFT`·`CAL SAVE`(벤치 절차 `IDENT_KO.md`) | 계산값과 실측이 많이 다르면 가변저항 모형(벨트 비·가운데 raw·직선성)을 고친다 |
| 호스트 프로필 | `host_profile.json` | 브리지 `steer_rover_bridge.py --host-profile <파일>`(또는 `UBTS_STEER_HOST_PROFILE`) | 브리지는 SIM 코드를 import하지 않는다(`HOST_BRIDGE_V3_KO.md`) |
| 실행기 조향 한계·휠베이스 | 호스트 프로필의 `steer_limit`, `vehicle.wheelbase_m` | 실행기 설정(실행기 담당이 반영. 이 도구는 `tools/row_executor`를 고치지 않는다) | |
| 차량 프로필 | `profiles/<id>.yaml` | 실차 저장소에는 복사하지 않는다(SIM 전용). 바뀐 값은 위 셋으로 간다 | |

## 7. 시험

`sim/ubts/test_steering_apply_spec.py`: 빈 입력 파일이 바탕과 같은 프로필을 만든다, 잇수 → 벨트 비 환산, ADC 끝값 → 벨트 비·가운데 raw, 벨트 비 입력이 어긋나면 거부, 범위 밖 값은 범위를 넓힌다, 가변저항 창 밖 비는 거부(후보 남김), 측정에는 날짜·note 필수, 모르는 칸 거부, 펌웨어 CFG·호스트·Gazebo 차이 계산.

## 8. 한계

- 계산 보정 표지는 자리값 가변저항 모형에서 나온다. 실기는 B2·B3로 다시 잡는다.
- 외란 토크 모형(정지 토크, 되밀림 배율)은 사양 값이 와도 가정이 남는다(`STEER_EXT_MODEL_KO.md` 6절).
- 이 도구는 SIM 쪽 값만 만든다. 실기 펌웨어 쓰기와 벤치·현장 시험은 사용자가 한다.
