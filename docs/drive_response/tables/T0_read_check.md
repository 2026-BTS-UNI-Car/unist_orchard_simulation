# 표 0. 읽기 확인(PW-02): 모터 줄 수와 길이를 11/11 목록과 대조

조향 장치 장착 전 차(좌우 차동 구동)의 2026-10-03·10-10 수동 주행 값

목록 = steering-sim-20261108/runs/replan_20261111/pwm_data/probe_pwm_inventory.out.txt. 10/3은 원래 motor.jsonl(전체 추출본)을 읽고, 같은 기록의 압축 추출본 모터 줄과 줄 수·수신 시각·요청·보고값을 대조했다.

| 기록 | 모터 읽기(field_calib.records) | 모터 줄 | 목록의 모터 줄 | 모터 시간 폭 s | 목록의 폭 s | 목록과 | 10/3 압축본 모터 = motor.jsonl | LiDAR 움직임 | GNSS 줄 | Cartographer 궤적 | 경고 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 20261003T072238Z_both | full (records.motor_from_jsonl) | 1522 | 1522 | 780.8 | 780.8 | 같음 | 같음 | O | 772 | - |  |
| 20261003T072329Z_both | full (records.motor_from_jsonl) | 179 | 179 | 91.4 | 91.4 | 같음 | 같음 | O | 89 | - |  |
| 20261003T072530Z_both | full (records.motor_from_jsonl) | 1021 | 1021 | 524.0 | 524.0 | 같음 | 같음 | O | 518 | O |  |
| 20261003T073551Z_both | full (records.motor_from_jsonl) | 1584 | 1584 | 812.7 | 812.7 | 같음 | 같음 | O | 801 | - |  |
| 20261003T080351Z_both | full (records.motor_from_jsonl) | 480 | 480 | 246.1 | 246.1 | 같음 | 같음 | O | 243 | - |  |
| 20261003T080909Z_both | full (records.motor_from_jsonl) | 408 | 408 | 209.1 | 209.1 | 같음 | 같음 | O | 207 | - |  |
| 20261003T081934Z_both | full (records.motor_from_jsonl) | 1061 | 1061 | 544.2 | 544.2 | 같음 | 같음 | O | 539 | - |  |
| 20261010T035952Z_both | compact (records.motor_from_compact) | 924 | 924 | 474.9 | 474.9 | 같음 |  | O | 470 | O |  |
| 20261010T040927Z_both | compact (records.motor_from_compact) | 1024 | 1024 | 527.0 | 527.0 | 같음 |  | O | 522 | O |  |
| 20261010T042151Z_both | compact (records.motor_from_compact) | 6 | 6 | 2.4 | 2.4 | 같음 |  | - | 3 | - | no lidar_motion.npz |
| 20261010T042200Z_both | compact (records.motor_from_compact) | 110 | 110 | 56.1 | 56.1 | 같음 |  | O | 56 | - |  |
| 20261010T042301Z_both | compact (records.motor_from_compact) | 1338 | 1338 | 688.8 | 688.8 | 같음 |  | O | 681 | O |  |
| 20261010T045553Z_both | compact (records.motor_from_compact) | 1452 | 1452 | 747.2 | 747.2 | 같음 |  | O | 739 | O |  |
| 20261010T051127Z_both | compact (records.motor_from_compact) | 1759 | 1759 | 905.0 | 905.0 | 같음 |  | O | 895 | O |  |
