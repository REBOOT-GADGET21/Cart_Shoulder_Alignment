# hwt9053_imu

HWT9053-485의 `RS485_HIGH` Modbus RTU 데이터를 ROS 2 표준 IMU 메시지로 변환하는 C++ 드라이버입니다. `/imu/data` (`sensor_msgs/msg/Imu`)와 `/imu/rpy` (`geometry_msgs/msg/Vector3Stamped`, rad)를 발행합니다. 동적 TF와 `odom -> base_link`는 발행하지 않습니다.

## Build and run

```bash
cd ~/cart_ws
colcon build --packages-select hwt9053_imu
source install/setup.bash
ros2 launch hwt9053_imu hwt9053_imu.launch.py
```

launch 파일은 기본으로 패키지의 `config/hwt9053_imu.yaml`을 불러옵니다. 별도 설정 파일을 쓰는 경우에만 `params_file:=/path/to/custom.yaml`을 추가하십시오.

`config/hwt9053_imu.yaml`의 `imu_static_tf.ros__parameters`에서 장착 변환을 바꿉니다. `x/y/z`는 m, `roll/pitch/yaw`는 radian입니다. 수정 뒤에는 이 패키지만 다시 build하고 새 launch를 실행합니다. `enable_static_tf:=false`로 정적 TF를 끌 수 있습니다.

정적 TF는 `base_link -> imu_link`만 발행합니다. 센서 설정의 `frame_id`와 launch의 `imu_frame`은 같은 값으로 유지하십시오.

## udev: `/dev/imu_usb`

USB-RS485 어댑터를 연결한 뒤 실제 장치(예: `/dev/ttyUSB0`)에 맞춰 속성을 확인합니다.

```bash
udevadm info -a -n /dev/ttyUSB0 | less
sudo cp $(ros2 pkg prefix hwt9053_imu)/share/hwt9053_imu/scripts/99-hwt9053-imu.rules /etc/udev/rules.d/
sudoedit /etc/udev/rules.d/99-hwt9053-imu.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

규칙의 `idVendor`, `idProduct` (필요하면 serial)를 어댑터 값으로 바꾸고 재연결한 다음 `ls -l /dev/imu_usb`로 확인합니다. 현재 사용자를 `dialout` 그룹에 넣었다면 로그아웃/로그인이 필요합니다.

## RViz

```bash
sudo apt update
sudo apt install ros-humble-rviz-imu-plugin

rviz2
```

RViz에서 **Add** → **By topic** → `/imu/data` → **Imu**를 선택하고 Fixed Frame을 `base_link` (또는 `imu_link`)로 설정합니다. `rviz_imu_plugin`은 패키지 설치 후 사용할 수 있습니다.

## Relative versus absolute orientation

기본값 `relative_orientation: true`에서는 처음 유효한 자세 quaternion을 기준으로 저장하고 이후 `inverse(reference) * current`를 발행합니다. 시작 시 RPY가 0이 되며 Euler 각을 빼지 않으므로 yaw 경계와 3D 회전을 안전하게 처리합니다. `/imu/reset_reference` (`std_srvs/srv/Trigger`)는 다음 유효 샘플에서 기준을 새로 잡습니다.

`relative_orientation: false`이면 센서가 보고한 절대 자세를 그대로 발행합니다.

## Baud rate

지원 보드레이트는 9600, 115200, 230400입니다. 센서 장치의 보드레이트와 YAML의 `baudrate`를 반드시 같은 값으로 설정해야 합니다. 한쪽만 바꾸면 통신할 수 없습니다.

공분산의 실제 설정값이 제공되지 않으므로, ROS IMU 규약에 따라 각 covariance 배열의 첫 원소를 `-1`로 설정해 “알 수 없음”을 명시합니다.
