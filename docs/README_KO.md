# 문서 색인

이 저장소의 문서가 어디에 있는지와, 지금 기준인 문서와 지난 단계의 기록을 나눠 적었다. 처음이면 1절 순서대로 읽는다.
모든 SIM 수치는 자리값 기반 SIM 결과이며 현장 성능이 아니다. (작성 2026-11-13, DOC-09)

## 1. 처음 읽을 것

| 순서 | 문서 | 무엇 |
|---|---|---|
| 1 | [`../README.md`](../README.md) | 저장소 첫 화면: 목표, 그림, 구성, 빠른 시작, 결과 요약, 한계 |
| 2 | [`ARCHITECTURE_KO.md`](ARCHITECTURE_KO.md) | 도식 6개: 시스템 구성, SIM 스택, 자료 흐름, 제어 루프, 추정기 층, 실행기 상태 |
| 3 | [`HARDWARE_KO.md`](HARDWARE_KO.md) | 차량·센서·보드의 알려진 사양과 SIM 자리값, 모르는 것 |
| 4 | [`RESULTS_KO.md`](RESULTS_KO.md) | 합격 검사 A1~A26과 사전등록 판정 결과(불합격 포함) |
| 5 | [`steering/DESIGN_v1_KO.md`](steering/DESIGN_v1_KO.md) + [`steering/DESIGN_v2_KO.md`](steering/DESIGN_v2_KO.md) | 설계. 둘이 다르면 v2가, v2와 결정 기록이 다르면 결정 기록이 앞선다 |
| 6 | [`steering/DECISIONS_KO.md`](steering/DECISIONS_KO.md) | 결정 기록 R1~R22, R30~R38(단계별 결과와 결정) |
| 7 | [`DOCS_POLICY_KO.md`](DOCS_POLICY_KO.md) | 문서·그림 방침, 자산 규칙과 검사 |

## 2. 지금 기준인 문서 (`steering/`)

설계와 기준

| 문서 | 무엇 |
|---|---|
| [`DESIGN_v1_KO.md`](steering/DESIGN_v1_KO.md) | 앞바퀴 조향 SIM 재구축 설계(두 단계 SIM, 차량 모델, 인터페이스, 월드, 제어기 방향, 평가, 단계 계획) |
| [`DESIGN_v2_KO.md`](steering/DESIGN_v2_KO.md) | v1 개정: 입력 규칙, 한 번 눌러 시작, 네 층, 속도 조절기, 현장 조정 값, 조향 하드웨어 가정 |
| [`DECISIONS_KO.md`](steering/DECISIONS_KO.md) | 결정 기록 |
| [`SIM_ACCEPTANCE_KO.md`](steering/SIM_ACCEPTANCE_KO.md) | SIM 자체 합격 검사 A1~A26과 개정(측정 전 등록) |
| [`PREREG_STEER_FOLLOW_KO.md`](steering/PREREG_STEER_FOLLOW_KO.md), [`PREREG_STEER_FOLLOW_GSIM_KO.md`](steering/PREREG_STEER_FOLLOW_GSIM_KO.md) | 열 추종 판정의 사전등록(빠른 SIM, Gazebo) |

규약과 실행

| 문서 | 무엇 |
|---|---|
| [`OPERATOR_API_KO.md`](steering/OPERATOR_API_KO.md) | 자율 주행 시작·정지 규약(실행기 운영자 HTTP, `run_state`) |
| [`ESP_PROTOCOL_V3_DRAFT_KO.md`](steering/ESP_PROTOCOL_V3_DRAFT_KO.md) | ESP 규약 V3 규격(줄 형식, CRC, TTL, 정지 모드, 결함 코드, 핀) |
| [`HOST_BRIDGE_V3_KO.md`](steering/HOST_BRIDGE_V3_KO.md) | 호스트 조향 브리지 V3(lease, STOP 확인, 래치) |
| [`FIRMWARE_V3_KO.md`](steering/FIRMWARE_V3_KO.md) | 조향 펌웨어 V3 구현 메모 |
| [`EXECUTOR_STEER_KO.md`](steering/EXECUTOR_STEER_KO.md) | 조향 실행기와 T1 실행기 루프(13절이 최신 마무리) |
| [`EXECUTOR_ESP32_KO.md`](steering/EXECUTOR_ESP32_KO.md) | 실행기와 실차 ESP32-S3 브리지(2026-11-16): 어댑터(눈금, ANGLE 정책, 추종·미끄럼 감시, Arm 전 점검), 차량 SHADOW 설정, 안전 규칙과 남는 위험, Jetson 실행 환경, 가짜 보드 끝단 시험 |
| [`STACK_OPERATIONS_KO.md`](steering/STACK_OPERATIONS_KO.md) | Gazebo 스택 운영 규칙과 그림·영상 갈무리 |
| [`SIM_DASHBOARD_KO.md`](steering/SIM_DASHBOARD_KO.md) | SIM 대시보드(대시보드 저장소 사본 + SIM 확장) |
| [`FIELD_TUNING_KO.md`](steering/FIELD_TUNING_KO.md) | 현장 조정 파일(기본·고급·잠금 묶음) |
| [`PATHS_KO.md`](steering/PATHS_KO.md) | 저장소 밖 자료의 경로 설정 |

추정기 층과 모형

| 문서 | 무엇 |
|---|---|
| [`ESTIMATOR_INPUTS_KO.md`](steering/ESTIMATOR_INPUTS_KO.md) | 추정기 입력·출력 확장 |
| [`ACTUATOR_PRIOR_KO.md`](steering/ACTUATOR_PRIOR_KO.md) | L1 구동기 사전값 층과 확인 결과 |
| [`LOCAL_GRID_KO.md`](steering/LOCAL_GRID_KO.md) | L2 국소 격자 층과 재생·SIM 비교 |
| [`GNSS_LAYER_KO.md`](steering/GNSS_LAYER_KO.md) | L3 GNSS 약한 층 |
| [`STEER_EXT_MODEL_KO.md`](steering/STEER_EXT_MODEL_KO.md) | 벨트 가변저항·조향 급전·외란 토크 모형 |
| [`STEER_PROFILE_V02_KO.md`](steering/STEER_PROFILE_V02_KO.md), [`STEER_PROFILE_V03_KO.md`](steering/STEER_PROFILE_V03_KO.md) | 프로필 v0.2·v0.3과 자격 판정 |
| [`SPEC_UPDATE_KO.md`](steering/SPEC_UPDATE_KO.md), [`VEHICLE_SPEC_REQUEST_KO.md`](steering/VEHICLE_SPEC_REQUEST_KO.md) | 조향 사양이 오면 할 일, 사양·실측 질문 Q1~Q29 |
| [`ABS_HOLD8_SIM_KO.md`](steering/ABS_HOLD8_SIM_KO.md), [`PREREG_STEER_FOLLOW_REALDIMS_KO.md`](steering/PREREG_STEER_FOLLOW_REALDIMS_KO.md), [`STEER_FOLLOW_REALDIMS_RESULT_KO.md`](steering/STEER_FOLLOW_REALDIMS_RESULT_KO.md) | 2026-11-16 실제 치수 프로필과 새 조향 보드 펌웨어(ESP32-S3, ABS-HOLD-8) 흉내, 그 프로필로 열 추종 다시 재기의 사전등록과 결과 |

결과

| 문서 | 무엇 |
|---|---|
| [`STEER_FOLLOW_RESULT_KO.md`](steering/STEER_FOLLOW_RESULT_KO.md) | 열 추종, 빠른 SIM 15회 |
| [`STEER_FOLLOW_GSIM_RESULT_KO.md`](steering/STEER_FOLLOW_GSIM_RESULT_KO.md) | 열 추종, Gazebo 4회 |
| [`GSIM_GAPS_RESULT_KO.md`](steering/GSIM_GAPS_RESULT_KO.md) | Gazebo 빈틈 정리(A17·A19·A25·A23·A18, 열 끝 e, 빠뜨린 틱) |
| [`STEER_FOLLOW_GSIM_FINAL_KO.md`](steering/STEER_FOLLOW_GSIM_FINAL_KO.md) | 최종 이미지 wf5 의 Gazebo 확인(열 추종 4 run, 화면, A17(G), A25, PW-08) |
| [`STEER_HOLD_RESULT_KO.md`](steering/STEER_HOLD_RESULT_KO.md) | 외란 아래 직진 유지 |
| [`TIMING_BASELINE_KO.md`](steering/TIMING_BASELINE_KO.md), [`REPLAY_BASELINE_KO.md`](steering/REPLAY_BASELINE_KO.md) | 지연·비용 기준선, 10/10 기록 재생 기준선 |

## 3. 지난 단계의 기록 (만든 때의 상태를 적은 문서)

아래 문서는 해당 단계에서 무엇을 만들고 쟀는지 적은 기록이다. 코드 설명으로 아직 쓸모가 있지만 "지금 상태"는 2절 문서와
[`RESULTS_KO.md`](RESULTS_KO.md)가 앞선다.

| 문서 | 단계 |
|---|---|
| [`LOCKSTEP_PROTOCOL_KO.md`](steering/LOCKSTEP_PROTOCOL_KO.md), [`NATIVE_ACTUATOR_MODEL_KO.md`](steering/NATIVE_ACTUATOR_MODEL_KO.md) | 1단계(P1): 네이티브 구동기·가변저항 모델과 lockstep 규약 |
| [`FAST_SIM_KO.md`](steering/FAST_SIM_KO.md), [`EVALUATE_STEER_KO.md`](steering/EVALUATE_STEER_KO.md) | 2단계(P2): 빠른 SIM, 평가기, A24, A13(F) |
| [`GZ_MODEL_KO.md`](steering/GZ_MODEL_KO.md), [`GZ_DEMO_KO.md`](steering/GZ_DEMO_KO.md) | 2단계(P5): Gazebo 조향 모델과 관절 직접 조종 시연 |
| [`CONFORMANCE_KO.md`](steering/CONFORMANCE_KO.md) | 2·4단계(P3): MCU 적합성 대본 |
| [`EXECUTOR_IMPORT_KO.md`](steering/EXECUTOR_IMPORT_KO.md), [`T1_RIG_KO.md`](steering/T1_RIG_KO.md), [`IDENT_KO.md`](steering/IDENT_KO.md) | 3단계(P4): 실행기 가져오기, T1 장치, 식별 도구 |
| [`P6A_HEADLESS_KO.md`](steering/P6A_HEADLESS_KO.md), [`WORLDS_KO.md`](steering/WORLDS_KO.md) | 3단계(P6a, P7): 헤드리스 Gazebo 스택, 월드 |
| [`S2_NODE_EXEC_KO.md`](steering/S2_NODE_EXEC_KO.md), [`STEER_RECORDING_KO.md`](steering/STEER_RECORDING_KO.md) | 4단계: S2 형식 노드·실행기 컨테이너, 조향 기록 |

## 4. 다른 폴더

| 폴더·문서 | 무엇 |
|---|---|
| [`drive_response/README_KO.md`](drive_response/README_KO.md) | 조향 장착 전 차의 PWM duty와 속도·출발·정지·이동거리 측정(그림 `figures/`, 표 `tables/`) |
| [`provenance/IMPORT_NOTES_KO.md`](provenance/IMPORT_NOTES_KO.md), [`provenance/BASE_IMAGE_KO.md`](provenance/BASE_IMAGE_KO.md) | 옛 저장소에서 옮겨 온 범위와 출처, 기반 이미지 |
| [`assets/ASSETS.json`](assets/ASSETS.json) | README 그림·GIF의 매니페스트(출처, 크기, 해시, 눈으로 본 기록) |
| `steering/figures/` | 결과 문서의 그래프·영상 |
| [`../third_party/README.md`](../third_party/README.md), [`../NOTICE`](../NOTICE) | 원본 프로젝트와 제3자 구성요소 고지 |
