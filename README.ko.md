# GNU 캠퍼스 고카트 시뮬레이터

[English](README.md) | [한국어](README.ko.md)

경상국립대학교 가좌캠퍼스를 NVIDIA Isaac Sim과 ROS 2 Humble에서 주행하기
위한 KAR 고카트 시뮬레이터입니다. 실측 주행 궤적·LiDAR 자료와 OSM 도로 및
건물을 결합했고, 기존 고카트·센서·MOZA R5 제어 구조를 유지했습니다.


캠퍼스 환경 제작에 사용한 실측 주행 궤적·LiDAR 맵 자료와 OSM 요소의 정합 결과입니다.

![GNU 캠퍼스 주행 궤적 및 OSM 오버레이](docs/gnu_campus_trajectory_osm_overlay.png)

## 핵심 기능

- 실제 주행 영역 전체를 포함한 지형, 도로, 보도, 건물
- 도로·지형·건물 PhysX 충돌과 지면에 맞춘 고카트 시작 위치
- RGB/Depth 카메라, 3D LiDAR, GNSS, IMU, Odometry
- RoboRacer `AckermannDriveStamped` 및 Autoware `Control` 입력
- MOZA R5 수동/공유 제어 파이프라인
- GNU 캠퍼스 ROS 2 실제 차량 속도를 **60 km/h**로 제한
- 경사진 시작 도로에서 중력에 의한 밀림을 줄이는 선택형 정지 브레이크
- 명령값과 실제 차량 상태를 표시하는 Viewport HUD

## 시스템 요구 사항

| 구성 요소 | 요구 사항 |
|---|---|
| 운영체제 | X11을 사용하는 Ubuntu 22.04 이상 |
| GPU | NVIDIA RTX, RTX 4070 이상 권장 |
| 드라이버 | NVIDIA 580 이상 |
| 컨테이너 | Docker 24 이상, CDI를 지원하는 NVIDIA Container Toolkit |
| Git | Git LFS 3 이상 |
| 저장공간 | Isaac Sim 이미지와 프로젝트용 45 GB 이상 |
| ROS | 이미지 내부에 ROS 2 Humble 설치 |

대용량 GNU 캠퍼스 USD 파일은 Git LFS로 관리합니다.

## 빠른 시작

```bash
git lfs install
git clone https://github.com/JunYoung270/gnu-campus-gokart-isaacsim.git
cd roboracer_isaacsim
git lfs pull
./docker/build.sh
```

컨테이너를 실행하고 종료하지 않은 채 유지합니다.

```bash
./docker/run.sh
```

컨테이너 안에서 GNU 캠퍼스 시뮬레이터를 실행합니다.

```bash
unset ROS_DISTRO
source /root/isaacsim/_build/linux-x86_64/release/setup_ros_env.sh
/root/isaacsim/_build/linux-x86_64/release/python.sh \
  scripts/launch_gnu_campus.py
```

첫 실행은 셰이더 컴파일로 몇 분 걸릴 수 있습니다. 사용자가 Isaac Sim을
닫거나 컨테이너를 중지할 때까지 GUI가 계속 실행됩니다.

## 조작 방법

고카트는 `ROS2_CONTROL` 모드로 시작합니다.

| 입력 | 동작 |
|---|---|
| `1` 길게 누르기 | Ego 차량의 ROS 2/키보드 제어 전환 |
| `W` / `↑` | 키보드 모드에서 가속 |
| `S` / `↓` | 후진/감속 |
| `A`, `D` / 방향키 | 조향 |
| `Space` | 키보드 속도 명령 정지 |
| `Backspace` | 시뮬레이션 재시작 |
| `/` | HUD 표시 전환 |
| `` ` `` | Perspective 카메라 |

## ROS 2 인터페이스

| 토픽 | 형식 | 용도 |
|---|---|---|
| `/ego/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | RoboRacer 목표 속도/조향 |
| `/ego/control` | `autoware_control_msgs/msg/Control` | Autoware 목표 속도/조향 |
| `/ego/odom` | `nav_msgs/msg/Odometry` | 실제 차량 운동 상태 |
| `/ego/imu` | `sensor_msgs/msg/Imu` | IMU 데이터 |
| `/ego/point_cloud` | `sensor_msgs/msg/PointCloud2` | 3D LiDAR |
| `/ego/gnss` | `sensor_msgs/msg/NavSatFix` | GNSS 위치 |
| `/ego/camera/*` | ROS 이미지/카메라 정보 | 스테레오 및 Depth 카메라 |

저속 직진 명령 예시:

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
ros2 topic pub --rate 10 /ego/drive \
  ackermann_msgs/msg/AckermannDriveStamped \
  '{drive: {speed: 0.5, steering_angle: 0.0}}'
```

GNU 캠퍼스 설정은 두 ROS 입력 토픽의 목표 속도 크기를
`16.6666666667 m/s`(60 km/h)로 제한하며 후진에도 동일하게 적용합니다.
NaN/Inf 명령은 거부합니다. 이 목표 속도 제한은 직접 throttle 명령을 사용하는
actuator 모드를 폐루프 속도 제어로 바꾸지는 않습니다.

## MOZA R5

레이싱휠 조작 시 Isaac Sim을 직접 actuator 차량/설정으로 실행합니다.

```bash
/root/isaacsim/_build/linux-x86_64/release/python.sh \
  scripts/launch_gnu_campus.py \
  --config scripts/configs/gnu_campus_kar_actuator.yaml
```

컨테이너 안에서 워크스페이스를 빌드합니다.

```bash
cd /workspace/autoware_off-road_sim/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select moza_gokart_interfaces moza_gokart_control
source install/setup.bash
ros2 launch moza_gokart_control manual_drive.launch.py \
  config:=/workspace/autoware_off-road_sim/ros2_ws/src/moza_gokart_control/config/moza_r5_actuator.yaml
```

HUD에는 정규화한 실제 페달 입력이 먼저 표시되고, 괄호 안에는 안전 기능을
적용해 PhysX로 전달한 출력값이 표시됩니다. GNU actuator 설정은 최초 저속
검증을 위해 액셀 출력을 15%로 제한하며, 명령이 끊기면 브레이크를 적용하고,
정지 직전에는 최대 브레이크를 적용하며 실제 속도 60 km/h 제한을 사용합니다.
페달 방향과 제동을 저속에서 확인하기 전에는 액셀 출력 제한을 높이지 마십시오.

목표 속도 모드, 직접 actuator 모드, fail-safe 및 계측 방법은
[MOZA 제어 문서](ros2_ws/src/moza_gokart_control/README.md)를 참고하세요.

## 설정

목표 속도 설정은
[`scripts/configs/gnu_campus_kar_physx.yaml`](scripts/configs/gnu_campus_kar_physx.yaml),
MOZA 직접 actuator 설정은
[`scripts/configs/gnu_campus_kar_actuator.yaml`](scripts/configs/gnu_campus_kar_actuator.yaml)입니다.

```yaml
network_setup:
  ros2_domain_id: 0
  rmw_implementation: rmw_fastrtps_cpp
  ros2_max_speed_m_s: 16.6666666667  # 60 km/h
control_settings:
  ros2_command_mode: speed
  stationary_hold_brake: 1.0
```

다른 환경에서 `ros2_max_speed_m_s`는 선택 항목입니다. 설정한다면 0보다 큰
유한값이어야 합니다.

## 저장소 구조

```text
assets/maps/gnu_campus_stage_D_full_usd/  실행용 캠퍼스 맵과 검증 자료
assets/vehicles/                           고카트 및 RoboRacer USD
scripts/launch_gnu_campus.py              GNU 캠퍼스 실행 진입점
scripts/launch_sim.py                     시뮬레이터, HUD, 센서, 브리지
scripts/configs/                           환경/차량 설정
ros2_ws/src/moza_gokart_control/          MOZA 및 공유 제어 노드
ros2_ws/src/moza_gokart_interfaces/       직접 actuator 메시지
docker/                                   재현 가능한 Isaac Sim 6 이미지
```

원본 포인트클라우드와 Stage B/C 맵 생성 중간 결과는 Git에서 제외합니다.
공유 저장소에는 실행용 Stage D 파일과 작은 검증 자료만 포함합니다.

## 현재 제한 사항

- 횡경사에서 정지한 뒤 차량이 옆으로 아주 조금씩 밀릴 수 있습니다. USD 갱신
  루프에서 강제로 강체 속도를 덮어쓰는 방식은 Isaac Sim 실행 안정성 문제가
  있어 사용하지 않고, 현재는 휠 브레이크 기반 정지 홀드를 유지합니다.
- 안전한 검증을 위해 MOZA actuator 액셀 출력은 현재 15%로 제한되어 있습니다.
  따라서 경사로 재출발 시 잠깐 뒤로 밀릴 수 있으며, 고속 시험 전에 출력 제한과
  차량 동역학을 추가 조정해야 합니다.
- 전 구간 및 실제 60 km/h 주행 검증은 아직 진행하지 않았습니다.

## 검증 현황

- 맵/고카트/HUD GUI 확인: 통과
- 시작 위치 정적 접촉: 통과
- ROS 2 브리지 및 필수 토픽: 통과
- 0.5 m/s 명령 후 정지: 실제 시뮬레이터 상태 기준 통과
- 60 km/h 명령 제한 단위 테스트: 통과
- MOZA 조향·액셀·브레이크·HUD 및 ROS 2 actuator 브리지: GUI 수동 확인 통과
- 실제 60 km/h 차량 동역학 및 전 구간 주행: 미실시

## 라이선스와 데이터 출처

코드는 [Apache License 2.0](LICENSE)으로 배포됩니다. OpenStreetMap에서 파생한
맵 요소에는 [OpenStreetMap 기여자](https://www.openstreetmap.org/copyright) 표시가
필요합니다. 공개 배포 전에 실측 캠퍼스 자료와 모든 외부 에셋의 재배포 권한을
확인하세요.
