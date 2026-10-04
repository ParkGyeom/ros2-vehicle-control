# ros2-vehicle-control

ROS2 Humble + Gazebo Fortress 환경에서 차동구동 차량(`vehicle_blue`)을 C++ 노드로 제어하는 패키지입니다.
오도메트리 기반 거리 주행, P제어 감속 정지, 안전 정지(fail-safe)를 구현했습니다.

## 환경
- Ubuntu 22.04 (arm64, UTM VM on Apple Silicon)
- ROS2 Humble, Ignition Gazebo Fortress, ros_gz_bridge
- C++17 (rclcpp)

## 구성
| 파일 | 설명 |
|---|---|
| `src/circle_driver.cpp` | 타이머로 Twist를 발행해 원 주행, Ctrl+C 시 정지 명령 후 종료 |
| `src/drive_distance.cpp` | 오도메트리 구독 → 목표 거리까지 P제어 감속 주행 후 정지 |
| `launch/sim.launch.py` | Gazebo + ROS↔Gazebo 브리지 실행 |
| `worlds/diff_drive.sdf` | Fortress 데모 월드 수정본 (오도메트리 1Hz → 50Hz) |

## 실행
```bash
cd ~/ros2_ws/src && git clone git@github.com:ParkGyeom/ros2-vehicle-control.git vehicle_control
cd ~/ros2_ws && colcon build --packages-select vehicle_control --symlink-install
source install/setup.bash

# 터미널 1: 시뮬레이션
ros2 launch vehicle_control sim.launch.py
# 터미널 2: 거리 주행
ros2 run vehicle_control drive_distance --ros-args -p target_distance:=2.0 -p kp:=1.5
```

파라미터: `target_distance`(m), `max_speed`(m/s, 로봇 한계 0.5로 clamp), `min_speed`, `kp`, `tolerance`

## 개발 기록

### 1. 정지 위치 오차 0.35m → 원인: 오도메트리 1Hz
- 증상: 목표 2m인데 2.348m에서 정지, 이동 거리가 0.5m 단위로만 갱신
- 분석: `ros2 topic hz`로 오도메트리 0.785Hz 측정 → 월드 파일의 `odom_publish_frequency 1` 확인
- 해결: 월드를 복사해 50Hz로 수정 → 41Hz, 정지 판단 시점 오차 0

### 2. 정지 후 미끄러짐 0.1m → P제어 감속
- 원인: 0.5 m/s에서 즉시 0을 명령해도 가속도 한계(1 m/s²) 때문에 계속 이동
- 해결: `cmd = clamp(kp × 남은거리, min_speed, max_speed)`로 감속, 정지 후 실제 속도 0을 확인하고 최종 위치 기록

| kp | 주행 시간 | 미끄러짐 | 최종 오차 |
|---|---|---|---|
| 0.5 | 10.1 s | 1.9 mm | -4.9 mm |
| 1.0 | 7.3 s | 2.0 mm | -7.1 mm |
| 1.5 | 6.5 s | 2.7 mm | -4.9 mm |
| 2.0 | 6.1 s | 1.9 mm | -5.2 mm |
| 4.0 | 5.4 s | 16.7 mm | **+24.0 mm (지나침)** |

P제어의 감속도는 `kp × v`이므로, 최고 속도 0.5 m/s에서 가속도 한계 1 m/s²를 넘지 않으려면 **kp ≤ 2**여야 합니다.
실험에서도 kp=4.0부터 목표를 지나쳤고, 여유를 두어 기본값을 1.5로 정했습니다.

### 3. Ctrl+C 시 차량이 멈추지 않음 → 안전 정지
- 원인: 기본 시그널 처리에서는 ROS2 컨텍스트가 먼저 종료돼 정지 명령을 보낼 수 없음
- 해결: `SignalHandlerOptions::None` + 직접 SIGINT 처리 + `spin_some` 루프로 "정지 명령 전송 → shutdown" 순서 보장

## License
MIT
