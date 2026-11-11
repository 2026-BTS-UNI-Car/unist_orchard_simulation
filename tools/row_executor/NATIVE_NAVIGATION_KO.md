# 10/10 응답 모델 기반 폐루프와 native navigation 계약

현재 패키지는 최신 설정에서 native navigation provider를 필수로 요구한다.
provider가 없거나 실제 native adapter 자격이 확인되지 않으면 `WAIT`이고
모터 요청은 0이다. 현재 제공되는 것은 이 계약과 생성 fake-provider 시험이며,
실제 Cartographer의 동일 snapshot pose·센서 시각·지도 cost 공급자는 아직 없다.
GUI 표시용 tracked pose를 제어용 pose로 승격하지 않는다.

실차 기본값은 계속 OFF다. `motor_output_enabled`, `calib_motor_output_enabled`,
`turn_assist_enabled`는 별도의 기존 설정·CLI·Arm 조건 없이 켜지지 않는다.
이 패키지 변경만으로 현장 측정값이나 구동 권한을 만들지 않는다.

## 실제로 연결해 시험한 소프트웨어 경로

`FieldPlant → 생성 LiDAR/IMU → ExecutorLoop → BridgeClient → 실제 대시보드
RoverControl(FakeMotor) → 명령 지연 → MCU ramp/방향 전환 대기 → FieldPlant`
폐루프를 실행했다. 재생한 고정 PWM을 따르는 방식이 아니다.

`sim_in_loop --plant field1010`은 10/10 `FieldResponseParams` 기본값을 보존한다.
90/100% 직진 속도와 고출력 회전·진동 자료를 반영한 모델이지만, 90% 미만
속도·deadzone·저속 C7·일부 응답은 미측정 SIM 가정이다. 예를 들어 개발 SIL의
55% 속도 0.09 m/s, C7 0.05 m, breakaway 20%를 실차 보정값으로 복사하면 안 된다.
기존 C7·조향·속도·STOP·신선도·ownership guard는 그대로 남아 있다.

이 개발 경로의 localization은 간이 keyframe ICP와 IMU이며 native SDK가 아니다.
GNSS와 occupancy map의 실제 공급자도 연결되지 않았다. `require_native_navigation`
가 false인 경로는 명시적인 `LEGACY_SENSOR_ONLY_DEVELOPMENT`로 표시한다.
최신 production 설정의 provider 누락을 이 모드로 자동 대체하지 않는다.

## 센서 경로거리의 변경

기존 scan별 `hypot(dx,dy)` 누적은 정지·저속 ICP 지터까지 양수 거리로 더했다.
`distance_mode=resolved_polyline`은 기존 nominal laser→base 기하를 이용한
관측 위치의 anchor 상대 최대 변위를 유지하고, 기존 keyframe 0.4 m 해상도에서
polyline을 확정한다. ICP cell 0.12 m 이상 되돌아가는 관측은 극점을 확정해
왕복 이동으로 구분한다. 정합 공백에서는 선분을 끊고 미관측 이동을 발명하지 않는다.
정지 pose cloud의 매 표본 길이를 더하지 않으며 truth·PWM·엔코더를 사용하지 않는다.

이는 공간 해상도로 분해한 센서 경로거리다. 0.12 m 미만 왕복이나 강한 곡률의
실제 이동거리를 정확히 복원한다는 뜻은 아니다. 측정 보정이나 native SLAM 대체도 아니다.
`scan_norm_legacy`는 기존 재현을 위해 별도로 보존했다.

resolved 프로필에서는 느린 이동의 정지 오인을 줄이기 위해 기존 12개 pose history 중
1초 창·최소 0.8초 유효 관측을 요구한다. 기존 2.5 cm/0.6° 공간 조건은 그대로다.
0.45초 창에서는 약 0.06 m/s 이동도 2.5 cm 안에 들어 정지로 오인했다.
drive 최소 속도, no-response 시간, STOP 조건을 낮추거나 완화하지 않았다.

## native provider가 가져야 할 관측

`navigation_observation.py`는 동일 session/epoch/transform/frame, 단조 sequence,
별도로 확인된 pose age, tracking, 동일 pose sequence의 complete occupancy cost를
검사한다. 원문 좌표와 capability는 status에 내보내지 않는다.
반복 sequence는 age를 갱신하지 않는다. context 변경·동일 sequence 내용 변경·시계 역전은
해당 gate를 fence한다. unknown·불완전 map·오래된 pose·장애 cost는 자유 공간이 아니다.

WAIT 후 provider가 회복돼도 이미 누르던 hold를 되살리지 않는다. 사용자의 새로운
press가 있어야 기존 재개 규칙을 다시 거친다. fake provider의 READY는 명시적인
in-process 생성 시험 의존성을 주입한 경우에만 허용되며 production CLI 옵션은 없다.
현재 실제 adapter는 자격화되지 않았으므로 non-test gate는 `ADAPTER_UNQUALIFIED`다.

## 관측한 결과와 한계

`field10-autonomy-integration-20261024/`에 사전등록·실패·수정·시계열을 보존했다.
동일 개발 7종을 수정 전후에 비교했다. 거리 수정 후 절대 거리오차는 명목
0.641→0.008 m, 약한 조향 0.473→0.081 m, 가지/불규칙 0.465→0.013 m였다.
이는 개발 생성 장면의 결과이며 독립 holdout이나 실차 정밀도는 아니다.

정지 오인 수정 후 60초 고정 평가에서는 6개 관문이 통과했고 편향 장면은
계속 FOLLOW 중 60초 제한에 도달해 FAIL로 보존했다. 별도 사전등록한 70초
개발 연장에서는 같은 장면이 63.7초에 열끝을 검출하고 정지했다.
기존 60초 FAIL을 전체 통과로 소급하지 않는다. 벽·stall의 안전 정지는 열 완주가 아니다.
가지 장면의 정지는 마지막 지지점보다 약 1.28 m 늦어 추가 검증·개선이 필요하다.

생성 trace의 truth는 평가기와 센서 합성에서만 사용된다. trace에는 요청 PWM,
지연된 MCU target, 모델 적용 PWM을 따로 기록한다. 적용 PWM은 측정 바퀴 속도나
측정 odometry가 아니다. GNSS 원시 좌표를 내보내지 않는다.

실제 native map/pose/cost 공급자, 품질별 weak GNSS anchor, 불확실 PWM 응답 예측,
실측 과수원 지형에서의 폐루프, 독립 holdout과 실차 주행은 남아 있다.
이 문서의 개발 통과는 실차 자율주행 GO 또는 설치·배포 완료 선언이 아니다.

## 소스 계보

SIM estimator의 수정은 독립 branch의 정식 source commit에 기록했다.
vendor는 `SOURCE_MANIFEST.json`의 새 commit에서 복사한 byte-exact 사본이다.
manifest의 `modified=false`는 그 선언된 새 소스와 vendor가 같다는 의미이며,
과거 estimator가 수정되지 않았다는 주장이 아니다. 이전 manifest hash와 변경 이유를
`derivation`에 보존했다. 원 main의 고정 423 source와 원 fieldint worktree는 수정하지 않았다.

## 단일열 모드(mode 'single_row', 2026-11-02)

단일열 모드의 제어는 실행기 자신의 2D LiDAR + raw IMU 추정기를 쓴다(통로 기준 횡오차·heading 오차,
scan matching 속도, `resolved_polyline` 거리). 상태와 기록에는 `SENSOR_ONLY_SINGLE_ROW`로 표시한다.
이 모드에서는 native provider를 필수로 요구하지 않으며, 설정은 `require_native_navigation=false`를
명시해야 한다(true면 설정을 거부한다: provider 요구를 조용히 무시하거나 대체하지 않는다).

provider가 있으면 매 tick 평가해 `native_cross_check`로 보고만 한다. WAIT·ADAPTER_UNQUALIFIED·STALE 등은
출발 허가도 정지 사유도 아니다. 완전히 유효한 같은 snapshot의 `COST_OCCUPIED`만 일시정지를 만든다
(`native_cross_check_cost_occupied`, OBSTACLE; 안전 쪽으로만 작동하며 새 누름이 있어야 재개한다).
생성 시험 provider를 시험 모드 밖에서 쓰면 `GENERATED_PROVIDER_FORBIDDEN`으로 보고만 하고 거부권도 없다.
입력 신선도·소유권·hold·전방/측면 장애물·기하 guard·OutputGuard·STOP 규칙은 그대로다.
다른 모드(`continuous`, `continuous_side`)의 `require_native_navigation` 동작은 바꾸지 않았다.
자세한 설명은 `SINGLE_ROW_MODE.txt`에 있다. 이것은 실제 native adapter의 자격화가 아니다.
