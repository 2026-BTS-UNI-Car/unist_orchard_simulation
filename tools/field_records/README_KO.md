# 현장 녹화 확인 도구 (`tools/field_records/`)

현장 녹화(rosbag2 mcap)를 처음부터 끝까지 읽어 실행기가 스스로 잰 값(거리, 횡 위치, 속도, 조향)을 다른 센서와 맞춰 보는 도구다.
2026-11-20 과수원 주행 기록을 확인하면서 만들었다([`docs/steering/ORCHARD_1120_RESULT_KO.md`](../../docs/steering/ORCHARD_1120_RESULT_KO.md) 5절).
녹화 원본과 결과는 레포에 넣지 않는다.

| 파일 | 하는 일 |
|---|---|
| `bag_extract.py` | 녹화 하나의 bag 을 순서대로 읽는다(끝의 색인을 쓰지 않아 닫히지 않은 bag 도 잘린 곳까지 읽힌다). 주제별 수·빈도·가장 긴 틈과 LiDAR·IMU·조향·브리지 상태·실행기 상태·GNSS 품질·카메라 프레임의 시계열을 `out/<녹화>.npz`, `out/<녹화>.json` 으로 쓴다 |
| `checks.py` | 뽑은 시계열로 녹화마다 실행기 거리 대 GNSS 직선 거리, 속도, 횡 위치 대 스캔의 좌우 거리, 열 폭, 조향 각(실행기 대 브리지), IMU 방향 변화를 견줘 `out/CHECKS.json` 과 표를 낸다 |

## 쓰는 법

```bash
python -m pip install numpy opencv-python mcap mcap-ros2-support
export UBTS_FIELD_RECORDINGS=<녹화 폴더>     # 그 아래 <녹화 id>/metadata.json, <녹화 id>/rosbag2/rosbag2_0.mcap
export UBTS_FIELD_OUT=<결과 폴더>            # 레포 밖. out/ 과 stills/ 가 생긴다
python -B tools/field_records/bag_extract.py <녹화 id> [<녹화 id> ...]
python -B tools/field_records/checks.py
```

## 알아 둘 것

- **GNSS 좌표를 쓰지 않는다.** 위치는 읽는 즉시 그 녹화의 첫 유효 표본에서의 동·북 거리(m)로 바꾼다. 위도·경도는 어느 출력에도 없다.
- LiDAR 를 차에 놓는 숫자(빔 각 + 179.08°가 진행 방향에서 잰 각, 차 기준점에서 앞으로 0.1185 m, 0.05~12 m)는 2026-11-20 현장 실행기 설정값이다.
  설정이 바뀌면 `bag_extract.py` 맨 위의 상수를 같이 고친다.
- 카메라 프레임(`stills/`)에는 사람이 찍힐 수 있다. 문서나 레포에 쓰기 전에 한 장씩 확인한다(문서 방침: 현장 카메라 장면은 레포에 싣지 않는다).
- bag 의 받은 시각은 뭉칠 수 있다(2026-11-20 기록에서 최대 1.1 s). 견줄 때는 센서 시각(header stamp)을 쓴다.
- 실행기의 거리·횡 위치는 실행기의 추정값이다. 이 도구는 다른 센서와 얼마나 맞는지만 보여 주고, 줄자나 측량 값을 대신하지 않는다.
