# 기반 이미지: 정의와 패키지 목록 (SR-12)

- 작성: 2026-11-12. 관련: `docker/Dockerfile.base`, `docker/base-packages/`, `docker/Dockerfile.ubts-steer`, `docs/provenance/IMPORT_NOTES_KO.md`.

## 1. 지금 쓰는 기반

- 조향 이미지(`docker/Dockerfile.ubts-steer`)는 apt·pip 없이 기반 이미지 위에 소스만 얹는다. 그 기반은 이 PC에만 있는 로컬 이미지다: `ubts-sim-clean-tenth:20261008-1791399576735742400-14657b29` (`sha256:6209168a3507…`, 2026-10-07 빌드). 옛 저장소의 `docker/Dockerfile.ubts`로 `ros:jazzy-ros-base` 위에 만들었고, 옛 소스가 들어 있다(조향 이미지 정의가 그 소스를 지우고 새로 복사한다).
- **지금까지의 모든 조향 결과(이미지 안 시험, verify_steer, P6a·4단계 검사, 비트 동일성)는 이 로컬 기반에서 검증했다.** 새로 만든 기반에서 검증한 결과는 아직 없다.
- 로컬 기반의 패키지 목록은 `docker/base-packages/`에 있다(2026-11-12에 그 이미지의 컨테이너에서 뽑음): `dpkg.txt`(1,373개), `apt_manual.txt`(119개), `pip_freeze.txt`(93개), `pip_raydeps.txt`(2개), `README.txt`(주요 버전: Ubuntu 24.04.5, Python 3.12.3, numpy 1.26.4, scipy 1.11.4, gz sim 8.15.0(ROS vendor 패키지), g++ 13.3.0, cmake 3.28.3, pytest 7.4.4).

## 2. 스스로 서는 정의 (`docker/Dockerfile.base`)

- 내용: `ros:jazzy-ros-base`를 다이제스트로 고정(로컬 기반을 만들 때 쓴 사본의 다이제스트 `sha256:066420e0…`), 옛 `Dockerfile.ubts`와 같은 apt 패키지 이름(그 옛 정의는 이 저장소에서 빌드되지 않아 2026-11-22 트리에서 뺐다. 반입 커밋과 `IMPORT_MANIFEST.json`에 남아 있다), pip 두 개(`trimesh==5.1.0`, `embreex==4.4.0`)를 `/opt/ubts/raydeps`에, 옛 소스 복사 없음. 마지막에 numpy·scipy·cv2·rclpy·gz 바인딩 import를 확인한다.
- Gazebo(gz sim 8, DART)와 gz Python 바인딩은 `ros-jazzy-ros-gz`가 끌어오는 ROS vendor 패키지(`ros-jazzy-gz-*-vendor`)에서 온다. matplotlib, PIL, numpy, cmake도 의존성으로 들어온 것이다(로컬 기반에서 자동 설치 표시 확인).
- 조향 이미지는 빌드 인자 `UBTS_STEER_BASE`로 기반을 고른다(기본은 로컬 기반, 그대로 둔다).
- **아직 빌드하지 않았다.** 빌드에는 apt 미러와 PyPI 접속이 필요한데, 이 파일을 쓴 작업에서는 외부 접속을 하지 않았다. 그래서 완료 기준 가운데 '이 파일만으로 기반이 빌드된다'와 '그 위에서 조향 이미지 빌드와 import 검사 통과'는 **확인하지 않았다**.

## 3. 새 기반을 만들 때 할 일

1. `docker build -f docker/Dockerfile.base -t ubts-steer-base:<날짜> docker/base-packages`(네트워크 필요, 새 이미지 약 5 GB).
2. 새 기반의 `dpkg-query -W` 목록을 `docker/base-packages/dpkg.txt`와 견준다. apt는 빌드하는 날 미러에 있는 버전을 주므로 Gazebo·DART·numpy 버전이 달라질 수 있다. 다른 패키지와 버전을 기록한다.
3. 그 기반으로 조향 이미지를 만들고(`--build-arg UBTS_STEER_BASE=...`) 이미지 안 시험, verify_steer, 비트 동일성 검사(A8, A25, 기준 trace)를 다시 돈다. 통과하기 전에는 새 기반의 결과를 기준 결과로 쓰지 않는다.
4. 통과하면 조향 이미지 정의의 기본 기반을 바꾸고 결정 기록에 적는다.
