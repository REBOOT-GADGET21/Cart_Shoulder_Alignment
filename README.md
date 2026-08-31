# Cart Shoulder Alignment

ROS 2 Humble 기반 D435 pose 검출과 Hybrid Lyapunov 카트 정렬 프로젝트입니다.

## 지원 환경

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Gazebo Harmonic 연동용 `ros_gz`
- Intel RealSense D435 / librealsense 2.55.x

## `wit_ros_imu`에 관하여

`wit_ros_imu`는 제조사가 제공한 ROS 1(catkin) 예제 패키지입니다. 이 저장소에서는 사용하지 않으며
`src/WitHighModbus_HWT9073485/` 전체가 `.gitignore`에 등록되어 있으므로 새로 clone한 저장소에는 포함되지 않습니다.
현재 프로젝트는 저장소에 포함된 ROS 2 패키지 `hwt9053_imu`를 사용합니다.

따라서 새로 clone한 컴퓨터에서는 다음처럼 skip 옵션 없이 빌드합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/cart_ws
colcon build
source install/setup.bash
```

기존 컴퓨터에만 무시된 제조사 폴더가 남아 있다면 `colcon list`에 `wit_ros_imu`가 표시될 수 있습니다.
그 컴퓨터에서만 폴더를 별도 보관하거나 기존의 `--packages-skip wit_ros_imu`를 사용할 수 있습니다.

## 시스템 및 ROS 의존성

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep libmodbus-dev
sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
cd ~/cart_ws
rosdep install --from-paths src --ignore-src -r -y
```

RealSense Linux 장치 권한과 udev 설정도 해당 컴퓨터에 설치해야 합니다. `pyrealsense2` Python 패키지만으로
커널/udev 설정까지 완료되지는 않습니다.

## Python 의존성

현재 프로젝트가 직접 사용하는 Python 패키지는 `src/requirements.txt`에 있습니다.

YOLO fine-tuning용 conda 환경:

```bash
conda create -n cart python=3.10 -y
conda activate cart
python -m pip install --upgrade pip
python -m pip install -r src/requirements.txt
```

NVIDIA GPU로 학습할 컴퓨터는 GPU 드라이버와 CUDA 버전에 맞는 PyTorch를 먼저 설치한 뒤 requirements를
설치하는 것을 권장합니다. CPU용 PyTorch 버전을 requirements에 고정하지 않은 이유도 컴퓨터마다 CUDA 조건이
다르기 때문입니다. `ultralytics`가 PyTorch와 torchvision을 의존성으로 설치하지만, GPU 학습 환경에서는 설치 후
다음을 반드시 확인합니다.

```bash
python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

ROS 실행 파일이 `/usr/bin/python3`를 사용하도록 빌드된 컴퓨터에서는 conda에만 설치된 패키지가 보이지 않을 수
있습니다. 다음 명령으로 실제 ROS Python을 확인합니다.

```bash
head -1 install/vision/lib/vision/realsense_pose
/usr/bin/python3 -c "import mediapipe, ultralytics, pyrealsense2"
```

두 번째 명령이 실패하면 ROS 실행 환경에도 같은 requirements를 설치하거나, 해당 conda 환경을 활성화한 상태에서
workspace를 깨끗하게 다시 빌드해야 합니다.

## Pose backend 선택

`src/params_setting.json`에서 둘 중 하나를 선택합니다.

```json
"vision_pose_backend": "yolo11n_pose"
```

또는:

```json
"vision_pose_backend": "mediapipe"
```

YOLO fine-tuning 결과는 다음 항목에 절대경로로 지정할 수 있습니다.

```json
"yolo_pose_model_path": "/absolute/path/to/best.pt"
```

MediaPipe backend에서는 YOLO 모델 경로를 읽거나 모델을 로드하지 않습니다.
기본 `yolo11n-pose.pt`는 Ultralytics가 처음 사용할 때 내려받으며, `.pt` weight 파일은 Git에 포함하지 않습니다.
학습한 weight를 공유해야 한다면 Git LFS 또는 별도 모델 저장소를 사용합니다.
