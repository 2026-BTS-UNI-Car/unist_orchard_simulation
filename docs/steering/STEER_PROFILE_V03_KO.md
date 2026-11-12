# 조향 프로필 v0.3: v0.2 값 + 고친 호스트 추종 규칙 아래 다시 판정

- 작성: 2026-11-13 (묶음 S3)
- 파일: `sim/ubts/steering/profiles/front_steer_v0_3_placeholder.yaml` (스키마 v3). 값은 v0.2와 모두 같다(시험 `test_v03_has_the_values_of_v02`). 바뀐 것은 note·설명과 이 판을 판정한 조건이다.
- 사전등록: `SIM_ACCEPTANCE_KO.md` 개정 43(새 판정 seed). v0.2의 개정 40~42 결과(불합격)는 `STEER_PROFILE_V02_KO.md` 4절, `STEER_HOLD_RESULT_KO.md`에 그대로 있다.
- 모든 값은 자리값이다. 결과는 자리값 기반 SIM 결과이며 현장 검증이 아니다.

## 1. v0.2 불합격 두 가지를 어떻게 다뤘나

| v0.2 불합격 | 원인 | 처리 |
|---|---|---|
| 개정 40 ② N5(정지 중 ±20° 쓸기)에서 호스트 `steering_not_following` 10/10 | 호스트가 MCU 서보 램프(정지 12 °/s)가 40°를 끝내기 전(3.3 s)부터 요청각과 측정각 차를 셌다 | 호스트 브리지 `steer_bridge_core._track_monitor`가 MCU 램프가 목표에 닿은 상태 줄(`cmd` = 소프트 한계로 자른 `tgt`)만 센다. 램프 중 뒤처짐은 MCU TRACK(진행 확인)이 본다. 평가 쪽 재계산(`safety_scripts/states.py`, `p6a_checks.host_timing`)도 같은 규칙. `HOST_BRIDGE_V3_KO.md` |
| 개정 40 ① A3 양자화 시험 | 시험 기준식에 벨트 유격이 없었다 | 시험의 깨끗한 변형이 벨트 유격도 0으로 둔다(이미 7aff7e2) |
| 개정 41·42 D6 seed 1004 첫 결함 POLARITY | 외력이 서보를 이겨 바퀴가 명령에서 멀어진다. MCU는 극성 반대와 구별할 수 없다. 개시 위상을 바꿔도 seed가 정한다(100회, 틱 차 0) | 오검출이 아니라 원인을 못 가리는 결함 이름이다. 크기 밖 외란의 정해진 결함에 POLARITY를 넣는다(개정 43). 펌웨어 규칙 구별은 P9 후보(`STEER_HOLD_RESULT_KO.md` 5절) |

## 2. 결과

개정 43의 ①~⑤ 결과는 아래 3절에 적는다(판정 뒤 같은 커밋에서 채운다).
