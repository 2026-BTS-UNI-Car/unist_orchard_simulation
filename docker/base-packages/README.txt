Package lists of the LOCAL base image of the steering SIM (docker/Dockerfile.base, docs/provenance/BASE_IMAGE_KO.md).

Image: ubts-sim-clean-tenth:20261008-1791399576735742400-14657b29
       sha256:6209168a3507cf7a657464af99711e0cdf45329617e67bdafca87007367688b0 (built 2026-10-07, amd64)
       on ros:jazzy-ros-base@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca
Taken on 2026-11-12 from a container of that image (no network): Ubuntu 24.04.5 LTS.

  dpkg.txt          dpkg-query -W: package, version, architecture (1373 packages)
  apt_manual.txt    apt-mark showmanual (119: the base image's own packages and the ones of Dockerfile.base)
  pip_freeze.txt    python3 -m pip list --format=freeze (system site, 93 distributions)
  pip_raydeps.txt   python3 -m pip list --path /opt/ubts/raydeps (the two pip installs)

Main versions:
  python 3.12.3 numpy 1.26.4 scipy 1.11.4 cv2 4.6.0 yaml 6.0.1 matplotlib 3.6.3 PIL 10.2.0 pytest 7.4.4 websockets 10.4
  gz.sim8 (gz Python bindings of ros-jazzy-gz-sim-vendor)
  gz sim 8.15.0 (ROS vendor packages ros-jazzy-gz-*-vendor; DART from ros-jazzy-gz-dartsim-vendor)
  g++ (Ubuntu 13.3.0-6ubuntu2~24.04.1) 13.3.0
  cmake version 3.28.3
Every recorded steering result was measured on this base. A base built again from docker/Dockerfile.base gets the
versions the mirrors have on that day: compare its lists with these before using it.
