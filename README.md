# capstone_ws

Go2 로봇, Ouster LiDAR, 3D SLAM/Localization 패키지를 통합하여  
3D SLAM / Localization / Visualization까지 연결하기 위한 캡스톤 프로젝트 워크스페이스입니다.

이 프로젝트는 단순히 개별 패키지를 모아둔 저장소가 아니라,  
여러 오픈소스 및 커스텀 패키지를 **`yeah_bringup` 패키지로 하나의 실행 구조로 통합**하여  
실제 로봇 시스템처럼 바로 구동할 수 있도록 구성한 것이 핵심입니다.

---

## 프로젝트 소개

본 프로젝트는 **Unitree Go2**와 **Ouster LiDAR**를 기반으로  
로봇의 주행, 센서 수집, 3D 맵 생성, 위치 추정까지 연결하는 통합 시스템을 목표로 제작했습니다.

특히 졸업작품에서는 단순 기능 구현보다도,  
여러 패키지에 흩어져 있는 기능들을 **하나의 실행 파이프라인으로 정리하고 통합하는 구조 설계**에 초점을 맞췄습니다.

이를 위해 직접 구성한 **`yeah_bringup` 패키지**를 중심으로 다음 요소들을 하나로 묶었습니다.

- Go2 구동
- Ouster LiDAR 실행
- TF 연결
- IMU 브리징
- RViz 시각화
- 향후 SLAM / Localization 연동 확장

즉, 이 저장소의 핵심은  
**“여러 기능을 개별적으로 실행하는 수준”이 아니라, 실제 시스템처럼 한 번에 실행 가능한 통합 구조를 만들었다는 점”**입니다.

---

## 핵심 특징

### 1. 통합 실행 구조 설계
기존에는 Go2, Ouster, TF, IMU 변환, RViz 등을 각각 따로 실행해야 했지만,  
본 프로젝트에서는 이를 **`yeah_bringup` 하나로 통합**하여 실행 편의성과 구조적 일관성을 높였습니다.

### 2. 실제 로봇 시스템 중심 구성
단순 시뮬레이션이 아니라 실제 Go2 로봇과 센서를 연결하여  
실환경에서 동작 가능한 구조를 기준으로 워크스페이스를 구성했습니다.

### 3. 확장 가능한 구조
현재는 센서 통합과 맵 생성/Localization 기반까지 구성했으며,  
이후 자율주행, 의미 기반 순찰, 위험 구역 인식 등으로 확장할 수 있도록 설계했습니다.

---

## `yeah_bringup` 패키지의 의미

이 프로젝트에서 가장 중요한 커스텀 패키지는 **`yeah_bringup`**입니다.

`yeah_bringup`은 여러 패키지를 단순히 포함하는 역할이 아니라,  
프로젝트 전체를 **하나의 로봇 시스템처럼 실행할 수 있도록 묶어주는 통합 진입점(entry point)** 역할을 합니다.

### `yeah_bringup`에서 통합한 요소
- `go2_bringup` 실행
- Ouster LiDAR launch 실행
- 로봇과 LiDAR 간 정적 TF 설정
- RViz 실행
- Go2 IMU를 GLIM 등에서 활용할 수 있도록 변환하는 브리지 실행

즉,  
**여러 패키지의 실행 순서와 연결 관계를 직접 설계하고, 이를 하나의 패키지로 통합한 점**이  
이 졸업작품의 중요한 구현 포인트입니다.

---

## 저장소 구성

```bash
capstone_ws/
├── Hdl-Localization-ROS2-Humble/
├── glim/
├── glim_ros2/
├── go2_bringup/
├── go2_description/
├── go2_driver/
├── go2_interfaces/
├── go2_robot/
├── go2_rviz/
├── ouster-ros/
├── unitree_api/
├── unitree_go/
├── unitree_ros2_example/
├── yeah_bringup/
└── go2_ai_mode_bridge.py

## 절대경로 수정 필요 항목

이 레포는 아래 파일에 개발 PC 기준 절대경로가 포함되어 있습니다.  
다른 장치(예: Jetson)에서 실행 전 반드시 수정하세요.

1. [rko_lio_go2.yaml](/home/taehyun/go2_ws/src/yeah_bringup/config/rko_lio_go2.yaml:15)  
수정 대상: `global_map_path`  
현재값: `/home/taehyun/go2_ws/src/yeah_bringup/map/glim_map.ply`  
수정 예시: `/home/<USER>/<WS>/src/yeah_bringup/map/glim_map.ply`

2. [glim_rosnode.cpp](/home/taehyun/go2_ws/src/glim_ros2/src/glim_rosnode.cpp:27)  
수정 대상: `base_dump_dir`  
현재값: `"/home/taehyun/go2_ws/glim_map"`  
수정 예시: `"/home/<USER>/<WS>/glim_map"` 또는 launch 파라미터로 외부화

3. [glim_ros2/config/config.json](/home/taehyun/go2_ws/src/glim_ros2/config/config.json:6)  
수정 대상: `config_path`  
현재값: `/home/taehyun/go2_ws/src/glim/dump/.../config`  
수정 예시: 현재 장치의 dump/config 경로로 변경

4. [glim_ros2/map/config/config.json](/home/taehyun/go2_ws/src/glim_ros2/map/config/config.json:6)  
수정 대상: `config_path`  
현재값: `/home/taehyun/go2_ws/src/glim/dump/.../config`  
수정 예시: 현재 장치의 dump/config 경로로 변경

절대경로 점검 명령:
`rg -n --hidden -S "/home/taehyun/go2_ws" src -g '!**/.git/**' -g '!**/dump/**'`

실행 luanch파일
ros2 launch yeah_bringup go2_ouster_rko_lio.launch.py

# Unitree 모델 경로
export UNITREE_MODEL_DIR=~/unitree_model

# ROS2/CycloneDDS 통신 설정
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                        <NetworkInterface name="enp12s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
export PATH="$HOME/.local/bin:$PATH"
export PATH="$HOME/.local/bin:$PATH"
export PATH="$HOME/.local/bin:$PATH"

#적용
source ~/.bashrc
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID
echo $CYCLONEDDS_URI
