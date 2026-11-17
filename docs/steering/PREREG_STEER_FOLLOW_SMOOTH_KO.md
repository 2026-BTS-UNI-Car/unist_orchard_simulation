# SMOOTH r5 조향 펌웨어로 열 추종 재기 사전등록 (steer_follow_smooth_r5_v1)

- 작성: 2026-11-17. 등록 번호 `steer_follow_smooth_r5_v1`. 기계가 읽는 사본: `PREREG_STEER_FOLLOW_SMOOTH.json`(`follow_smooth.py --prereg`의 출력과 같아야
  한다. 시험 `sim/ubts/test_steering_follow_smooth.py`가 본다).
- 이 문서는 판정에 쓰는 run 을 돌리기 전에 커밋한다. 이 커밋까지 새 프로필로 돌린 것(개발 확인, 판정 아님): T1 정지 계단(모터 속도 맞춤, rate 60~100),
  단위 시험과 적합성 대본, 등록 밖 seed 99 의 `nominal` 1회(열 끝 정지, 기능 확인: 처음에는 실행기 설정이 명령 변화율 반올림으로 거부되어
  `smooth_hold.executor_values` 의 속도를 0.01 °/s 아래로 내리게 고친 뒤 다시 돌림).
- 결과는 **자리값 기반 SIM 결과**다. 실차 성능이 아니다. 새 프로필도 SPEC_PENDING 자리값 34칸과 SIM 가정값이 남는다.
- 근거: [`SMOOTH_R5_SIM_KO.md`](SMOOTH_R5_SIM_KO.md)(펌웨어 흉내와 프로필), 앞 등록 [`PREREG_STEER_FOLLOW_REALDIMS_KO.md`](PREREG_STEER_FOLLOW_REALDIMS_KO.md)
  (같은 조건·기준, 펄스 유지 v1 프로필).

## 1. 무엇으로 재나

| 항목 | 값 |
|---|---|
| 시뮬레이터 | F-SIM T1 리그, 실행기 포함(`tools/row_executor/sim_in_loop_steer.py`). 앞 등록과 같다 |
| 프로필 | `front_steer_v2_field_20261116`(스키마 v5). v1 의 치수 그대로, 보드 보정 1475/2017/2513, `steer_fw_mode smooth_r5`(ESP 에뮬레이터가 r5 의 SMOOTH 유지를 보드에 저장된 SM 값으로 돈다), 조향 모터 속도 80 °/s(r5 계단에 맞춤), `servo_hz` 1000 |
| 실행기 설정 | 리그가 만드는 설정 그대로(`steer_executor_config`). 조향 '측정' 값은 r5 계단 응답: 측정 조향 속도 12.39 °/s, 명령 변화율 한도 9.912 °/s, 서보 데드밴드 0.216°(`smooth_hold.executor_values`). GO 문턱(8 °/s)을 바꾸지 않는다 |
| 제어기 | `{"kind": "row_follow", "wheelbase_m": 0.657}`(앞 등록과 같다) |
| 발자국 | 1.22 × 0.70 m(프로필 자리값 그대로) |

## 2. 부분

| 부분 | run | 판정 |
|---|---|---|
| `matrix` | `steer_follow_v1` 의 11조건 15회(앞 등록 표 A 그대로) | 15회 모두 통과면 통과 |
| `wide` | `steer_follow_wide_v2` 의 20조건 × seed 4000, 4001, 4002 = 60회(앞 등록 표 B 그대로, 조향 외란 두 조건도 이 프로필의 외력 모형으로) | 조건마다 통과 수를 적는다. 60회 모두 통과면 통과 |

- 순서: `matrix` 먼저. `wide` 는 시간이 되면 돌리고, 못 돌리면 '돌리지 않음'으로 적는다.
- 기준(앞 두 등록과 같은 수): 접촉 없음(여유 > 0 m), 끝에 정지(≤ 0.02 m/s), guard 위반 종결 없음, A22 규칙. 열 끝 run 은 `row_end`, 주행 뒤 일시정지 0,
  횡 RMSE ≤ 0.20 m, p95 ≤ 0.35 m, 최대 ≤ 0.50 m, 방향 RMSE ≤ 5°, 최대 ≤ 15°, 참 속도 최대 ≤ 0.50 m/s(`speed_0p5` 는 0.55), e −1.0 … +1.0 m.
  출발 어긋남 조건과 W2 는 RMSE·p95 를 6.0 m 뒤부터 잰다. 일시정지 조건은 앞 등록의 규칙.

## 3. 고칠 수 있는 것

- 이 등록에는 반복이 없다. 결과는 그대로 적는다. 실패가 나오면 원인 조사(등록 밖)와 다음 등록의 제안으로 남긴다.

## 4. 비교로 적는 것 (판정 아님)

- v1(펄스 유지, GO 문턱 4.0) 결과: `matrix` 15/15, `wide` 51/60(`STEER_FOLLOW_REALDIMS_RESULT_KO.md`).
- V3 동일성: 이 작업의 바꿈 뒤 T1 안전 대본 71개와 빠른 SIM `nominal` 두 프로필(v0.1, v1)의 digest 가 main aa7f2f4 와 같은지.

## 5. 이 등록이 보이지 않는 것

현장 성능, 땅에 닿은 조향 속도(바퀴 든 계단으로 맞춤), 전류 따라 바뀌는 가변저항 어긋남(기본 0), 실기 보드에 없는 V3 감시의 차이, 실기 브리지와
어댑터의 두 겹 정책(SIM 은 하나로 합침), Gazebo.
