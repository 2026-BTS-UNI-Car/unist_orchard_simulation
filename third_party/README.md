# third_party — 제3자 라이선스 고지

이 저장소의 코드는 Apache-2.0 (`LICENSE`) 입니다. 아래 파일은 예외로, 원본의 라이선스를 그대로 따릅니다.
원본 프로젝트(orchard-rover, 커밋 c3c1f86)에서 가져온 범위와 고친 파일은 `NOTICE` 와
`docs/provenance/IMPORT_NOTES_KO.md` 에 있습니다.

## 저장소에 들어 있는 제3자 파일

모두 대시보드 사본의 웹 화면이 쓰는 파일입니다. 라이선스 전문은 각 폴더에 함께 있습니다.

| 파일 | 원본 | 라이선스 | 전문 |
|---|---|---|---|
| `dashboard/static/vendor/hls/hls.min.js` | hls.js 1.7.3 | Apache-2.0 (Copyright (c) 2017 Dailymotion) | `dashboard/static/vendor/hls/LICENSE` |
| `dashboard/static/vendor/maplibre-gl/*` | MapLibre GL JS 6.11.2 | BSD-3-Clause (Copyright (c) 2023, MapLibre contributors) | `dashboard/static/vendor/maplibre-gl/LICENSE.txt` |
| `dashboard/static/vendor/pmtiles/pmtiles.js` | PMTiles 4.5.0 | BSD-3-Clause | `dashboard/static/vendor/pmtiles/LICENSE.txt` |
| `dashboard/static/vendor/protomaps_basemaps/basemaps.js` | @protomaps/basemaps 5.7.2 | BSD-3-Clause 외 (Copyright 2019-2023 Protomaps LLC, Kelso Cartography) | `dashboard/static/vendor/protomaps_basemaps/LICENSE.txt` |
| `dashboard/static/vendor/korean-font/korean.woff2` | Noto Sans KR (weight 400) 부분 글꼴 | SIL Open Font License 1.1 | `dashboard/static/vendor/korean-font/KOREAN-OFL.txt` |
| `dashboard/static/vendor/ubts-sans/*.woff2` | Pretendard 1.3.9 (Pretendard Variable) 부분 글꼴, 굵기 400~700. 예약 글꼴 이름 조건에 따라 글꼴 안 이름을 `UBTS Sans` 로 바꾼 사본 | SIL Open Font License 1.1 (Reserved Font Name Pretendard) | `dashboard/static/vendor/ubts-sans/PRETENDARD-OFL.txt` |

출처 주소와 sha256 은 `dashboard/static/vendor/gps-map-vendors.json`, `dashboard/static/vendor/hls/provenance.json` 에 있습니다.
UBTS Sans 의 원본 파일·sha256·만든 방법은 `dashboard/static/vendor/ubts-sans/ubts-sans-var.json`, `ubts-sans-split.json` 에 있습니다.

## 이 저장소에 없는 것

원본 프로젝트에 있던 다음 파일은 가져오지 않았으므로 해당 고지도 싣지 않았습니다: Ultralytics YOLO 로 학습한
검출 모델(AGPL-3.0), torchvision 구조의 분할 모델, PX4 airframe(BSD-3-Clause), livox_ros_driver2 설정(MIT).
이 가운데 어느 것이든 다시 들여오면 그 라이선스 고지와 전문을 함께 들여와야 합니다. AGPL-3.0 모델은 함께
배포되는 전체에 AGPL 조건이 걸릴 수 있으므로 따로 검토한 뒤에만 들여옵니다.

## 실행하거나 이미지를 만들 때 받는 것 (저장소에는 없음)

ROS 2 Jazzy · Gazebo Harmonic (Apache-2.0), 그리고 `docker/Dockerfile.base`(`docker/Dockerfile.ubts-steer` 의 기반 이미지 정의)가 설치하는 배포판 패키지와 Python
패키지(numpy, scipy, PyYAML, OpenCV, websockets, trimesh, embreex 등). 각 라이선스 전문은 원본 배포처에 있고,
Docker 이미지 안에서는 `/usr/share/doc` 등에 있습니다.
