# ZLAC8015D 하드웨어 드라이버

이 패키지는 기존 `rear_ackermann_controller`가 발행하는 좌·우 바퀴 각속도 토픽을 받아 ZLAC8015D RS485 Modbus RTU 속도 명령으로 변환한다. `/cmd_vel` 또는 `/alignment_cmd`를 직접 구독하지 않으며, 차동 구동 역기구학도 다시 계산하지 않는다.

## 인터페이스

- 입력: `/rear_left_wheel_speed_cmd`, `/rear_right_wheel_speed_cmd` (`std_msgs/msg/Float64`, 바퀴 rad/s)
- 오도메트리 입력: `/zlac8015d/wheel_joint_states` (encoder 기반 바퀴 각도), `/imu/data` (`sensor_msgs/msg/Imu`). 차체 기하와 IMU 입력 설정은 `src/params_setting.json`을 사용한다.
- 오도메트리 출력: `/odom` 및 `odom -> base_link` TF. 위치는 encoder, yaw는 IMU orientation을 사용한다.
- 진단: `/imu/deg` (`geometry_msgs/msg/Vector3`). `x=roll`, `y=pitch`, `z=yaw`, 단위는 모두 도(deg)다.
- 출력: `/zlac8015d/left_actual_rpm`, `/zlac8015d/right_actual_rpm` (`Float64`)
- 상태: `/zlac8015d/left_fault`, `/zlac8015d/right_fault` (`UInt16`), `/zlac8015d/connected` (`Bool`), `/zlac8015d/state` (`String`)
- 명시적 fault clear: `/zlac8015d/reset_fault` (`std_srvs/srv/Trigger`)

ZLAC8015D Ver3.1의 velocity mode 레지스터 `0x200D`, control word `0x200E`, target velocity `0x2088~0x2089`, actual velocity `0x20AB~0x20AC`, fault `0x20A5~0x20A6`를 사용한다. 속도는 signed 16-bit RPM으로 `0x10` multiple-register write 한 번에 동기 전송한다. 일부 통신 문서의 `0x2031 Stop` 예시는 같은 문서의 control-word 표와 충돌하므로 사용하지 않는다.

## 설치와 빌드

```bash
sudo apt install libmodbus-dev
cd /home/jeonga/cart_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rear_ackermann_controller zlac8015d_driver
source install/setup.bash
```

`config/zlac8015d.yaml`에서 `serial_port`, 감속비, 안전 RPM, 좌우 방향을 실제 장비 값으로 바꾼다. `serial_port`에는 `/dev/ttyUSB0`보다 `/dev/serial/by-id/...`의 고정 경로를 권장한다.

`motor_ramp_rpm_per_sec: 5.0`은 ROS에서 보내는 모터 속도의 초당 최대 변화량이다.
좌우 속도를 같은 보간 비율로 갱신하여 출발, 회전 방향 전환, 회전 후 직진을
점진적으로 수행한다. 0→5 RPM은 통신 지연이 없다면 약 1초 걸린다.
소수 RPM을 내부에 유지한 뒤 전송할 때만 정수로 반올림하므로, 장치의 1 RPM
명령 해상도 자체는 바뀌지 않는다. 하드웨어 `acceleration_time_ms` 및
`deceleration_time_ms` 설정은 별도로 유지된다.
안전 해제, 명령 timeout, fault 정지는 ramp를 우회하며 다음 출발은 0부터 시작한다.
자동 정렬과 수동 `/cmd_vel` 모두 실기 드라이버에서 적용된다. 실제 속도를
피드백하는 제어기는 아니므로 마찰이나 좌우 모터 차이까지 보정하지는 않는다.
가감속 지연이 추가되므로 최종 정렬 오차와 정지 거리는 실기에서 재확인해야 한다.

## 실행

```bash
ros2 launch zlac8015d_driver hardware_drive.launch.py
```

이 launch는 `rear_ackermann_controller`와 실제 모터 드라이버만 실행한다. Gazebo의 wheel bridge, `alignment_control.launch.py`, 또는 simulation launch와 동시에 실행하지 않는다.

## 실제 자동 정렬

```bash
ros2 launch zlac8015d_driver real_alignment.launch.py
```

이 launch는 HWT9053 IMU, 실제 모터/encoder odometry, 카메라 TF, D435+YOLO, 그리고
기존 `shoulder_align_node`를 함께 실행한다. Gazebo와는 함께 실행하지 않는다.
shoulder_align_node는 가제보를 실행하는 명령어로 가제보와 실제 하드웨어를 함께 실행하지 않는다

자동 정렬 중 제어기가 `/alignment/drive_enabled`를 20 Hz로 발행한다. 카메라/YOLO/TF가
끊겨 `/shoulder_line`이 오래되거나, IMU/encoder가 끊겨 `/odom`이 오래되면 이 값은
`false`가 된다. 드라이버는 0 RPM을 쓴 뒤 ZLAC control word `0x07 Stop`으로 토크를
해제한다. 입력이 다시 정상화되면 `0x08 Enable` 뒤 새 제어 명령으로만 재출발한다.
이 안전 인터록은 `real_alignment.launch.py`에서만 강제된다. 단독 모터 시험 launch는
기존처럼 `require_safety_enable:=false`가 기본이다.

## 안전 시험 순서

1. 물리 E-stop을 준비하고 바퀴를 지면에서 완전히 띄운다.
2. 전원과 노드 실행만으로 바퀴가 회전하지 않는지 확인한다.
3. 낮은 값으로 왼쪽, 오른쪽 바퀴를 각각 시험하고 `left_motor_inverted`, `right_motor_inverted`를 확정한다.
4. 매우 낮은 속도의 직진과 제자리 회전을 확인한다.
5. wheel command를 멈춰 `command_timeout_sec` 뒤 0 RPM이 전송되는지 확인한다.
6. Ctrl+C, USB-RS485 분리, 드라이버 fault에서 정지/명령 차단을 확인한다.
7. 재연결 뒤 이전 속도가 자동 복원되지 않고 새 명령 전까지 0 RPM인지 확인한다.

`Quick Stop`은 RS485 드라이버 명령일 뿐 물리 E-stop을 대체하지 않는다. fault는 자동으로 clear하지 않으며, 원인을 제거한 후에만 사용자가 `/zlac8015d/reset_fault`를 호출해야 한다.
