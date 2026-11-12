# 조향 프로필 v0.2 (ST-10): 자리값 일관성, 벨트 가변저항, 외란 모형

- 작성: 2026-11-13 (묶음 S3)
- 파일: `sim/ubts/steering/profiles/front_steer_v0_2_placeholder.yaml` (스키마 `ubts.vehicle_profile.v3`)
- 근거: 결정 R18 결정 2·3(자리값끼리 맞지 않는 곳), 설계 v2 V2-6(벨트로 이은 가변저항, 일반 DC 모터, 리밋 스위치 없음, 직진 유지), 결정 R36
- 사전등록: `SIM_ACCEPTANCE_KO.md` 개정 40(v0.2 자격), 41(외란 아래 직진 유지)
- 모든 값은 자리값이다. 이 프로필로 낸 결과는 자리값 기반 SIM 결과이며 현장 검증이 아니다. v0, v0.1 파일과 그 결과는 그대로 둔다.

## 1. 바꾼 값 (v0.1 → v0.2)

v2 매개변수 가운데 아래 다섯 칸만 바꿨다(시험 `test_v02_consistency_changes_against_v01`이 지킨다). 펌웨어 소스는 그대로이고 값은 프로필 → CFG(`fw_eeprom.params_from_profile`)로 들어간다.

| 칸 | v0.1 | v0.2 | 이유 (R18) | 펌웨어 CFG |
|---|---|---|---|---|
| `servo_min_pwm_pct` | 40 % | 45 % | 정지 중 조향 시작 PWM 35 %와의 간격 5 %p → 10 %p(결정 2: B4 시작 + 5 %p 이상, 여유). 정지 중 서보 최소 출력의 조향 속도 1.15 → 2.3 °/s | `minp` 102 → 115 |
| `polarity_fault` | 1° / 0.1 s | 2° / 0.2 s | 정지 중 기어감과 거른 값 ±2 count 흔들림으로 오탐(결정 2, A19(F) 불합격 1/40) | `polp` 40 → 80‰, `polt` 100 → 200 ms |
| `driver_stuck_fault` | 2° / 0.3 s | 3° / 0.3 s | 주행 중 비자기잠금 바퀴가 가운데로 돌아올 때 오판(결정 3) | `dsm` 80 → 120‰ |
| `thermal_proxy` 누적 | 5 s / 30 s | 10 s / 30 s | 정지 중 전폭 쓸기를 두 번만 허용(결정 3) | `tha` 5000 → 10000 |
| `host_track_mismatch` | 3° / 1.5 s | 3° / 3.0 s | 정지 중 느린 조향과 반전 때 백래시 통과에 걸림(결정 3) | 호스트 프로필 |

범위(스윕 폭)도 위 칸은 넓혔다. `pot_to_steer_ratio`(벨트 비)의 범위는 1~5 → 0.5~5다(값 1 그대로).

## 2. 새 칸 (스키마 v3)

I 묶음(벨트·리밋·급전)과 J 묶음(외란)은 `STEER_EXT_MODEL_KO.md`에 모형과 식이 있다. 사양 요청서의 질문 번호를 `identify`에 적었다(Q22~Q26). 계산값은 `belt_tooth_step_deg`(18°), `pot_travel_deg`(54.2°), `pot_cal_half_width_counts`(95.2)다.

## 3. 프로필 읽기 검사 (스키마 v3)

- 벨트 비가 범위 안이어도 끝 멈춤이 보정 창(60~963) 밖이면 `pot_range_exceeded`, 보정 표지가 가운데에서 50 count 안쪽이면 `pot_cal_half_width_small`로 거부한다. 자리값에서 받는 비는 약 0.53~4.2다(0.5는 47.6 count로 거부).
- 리밋 스위치 `both`는 `not_modelled`로 거부한다.
- 이 건너뜀은 `belt_type` timing에서만.
- 스윕용 덮어쓰기는 `allow_out_of_range=True`면 위 가변저항 검사를 경고로 낮춘다.

## 4. 결과

측정 결과는 같은 문서 5절 이후에 사전등록 순서대로 적는다(개정 40 ①~④). 잰 결과만 적는다.
