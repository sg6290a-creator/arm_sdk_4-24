# ARM417 Current Status

2026-04-18 기준 `armfullpk_417`에서 이번 세션 동안 바뀐 내용과 현재 남아 있는 상태를 정리한 문서다.

이 문서는 "지금 코드가 어떤 상태인지"를 빠르게 복원하기 위한 용도다.

## 1. 현재 기준 상태

- 현재 기준 워크스페이스는 `/home/frlab/ing_ws`
- 현재 기준 패키지는 `/home/frlab/ing_ws/src/armfullpk_417`
- ROS 배포판은 `Humble`
- 실하드웨어 launch 명령은 아래를 기준으로 사용

```bash
cd /home/frlab/ing_ws
source /opt/ros/humble/setup.bash
source /home/frlab/ing_ws/install/setup.bash
ros2 launch description frlab_arm_control.launch.py use_fake_hardware:=false
```

## 2. 현재 남아 있는 변경

### `src/description/config/controllers.yaml`

- `controller_manager.update_rate` 를 `100 -> 200` 으로 올림
- `joint_state_broadcaster`, `dls_controller` 설정 유지

### `src/description/launch/frlab_arm_control.launch.py`

- `use_fake_hardware` 에 따라 real/fake robot description 을 분리
- fake hardware 모드에서 실제 하드웨어 전용 파라미터를 URDF 문자열에서 제거
- `kinematics_description` 을 임시 YAML로 만들어 `dls_controller` spawner 에 주입
- `joint_state_broadcaster` 를 먼저 띄우고, 종료 이벤트 뒤 `dls_controller` 를 띄우도록 순차화
- `joint_state_broadcaster` 와 `dls_controller` 둘 다 `controllers.yaml` 을 명시적으로 받도록 변경
- `joint_state_broadcaster` spawner 는 `2.0s` delay 뒤 실행

### `src/description/urdf/ur5e_gripper.urdf`

- `ros2_control` hardware 파라미터 추가
  - `can_interface=can0`
  - `default_velocity=0.5`
  - `read_deadline_ms=6`
  - `read_poll_timeout_ms=1`
  - `perf_log_every_n_cycles=0`

### `src/ros2_hardware/include/frlab_arm_hardware/frlab_arm_hardware.hpp`

- Jazzy 전용 구조 대신 Humble `SystemInterface` 기준으로 포팅
- 내부 state/command 버퍼 추가
  - `hw_positions_`
  - `hw_velocities_`
  - `hw_efforts_`
  - `hw_position_commands_`
- 하드웨어 파라미터 멤버 추가
  - `read_deadline_ms_`
  - `read_poll_timeout_ms_`
  - `perf_log_every_n_cycles_`
- read failure 카운터와 perf 집계 변수 추가

### `src/ros2_hardware/src/frlab_arm_hardware.cpp`

- Humble 방식의 `on_init(const HardwareInfo&)` / `export_state_interfaces()` / `export_command_interfaces()` 로 변경
- URDF 하드웨어 파라미터를 읽어서 SDK init 에 전달
- `read()` 실패 시 즉시 `ERROR` 를 반환하지 않고 cached state fallback 을 사용
- read failure 로그를 throttle 로 변경
- `perf_log_every_n_cycles > 0` 이면 평균 `read/write/cycle` 시간과 read failure 수를 출력
- `write()` 는 position command + default velocity 조합으로 SDK에 전달

### `src/frlab_arm_sdk/include/arm_sdk/frlab_arm.hpp`

- `FrlabArm::init()` 에 timing 파라미터 추가
  - `read_deadline_ms`
  - `read_poll_timeout_ms`

### `src/frlab_arm_sdk/include/arm_sdk/integrated_driver.hpp`

- `IntegratedDriverConfig` 에 read loop timing 파라미터 추가
  - `read_deadline_ms`
  - `read_poll_timeout_ms`

### `src/frlab_arm_sdk/src/frlab_arm.cpp`

- SDK config 생성 시 `read_deadline_ms`, `read_poll_timeout_ms` 전달
- 현재 실제 joint 매핑은 아래 상태를 유지
  - `joint_1`: Robstride `ID=1`
  - `joint_2`: Robstride `ID=127`
  - `joint_3..6`: RMD `ID=1..4`
- header 주석의 `joint_2=ID=2` 설명과 실제 구현은 아직 불일치

### `src/frlab_arm_sdk/src/integrated_driver.cpp`

- `readAll()` 은 현재 아래 상태로 유지
  - 이전 cycle write ACK / feedback 이 RX 버퍼에 남아 있으면 먼저 소모
  - 이미 받은 joint 는 다시 query 하지 않고, 아직 못 받은 joint 만 추가 query
  - RMD 는 `0x9C` 상태응답뿐 아니라 `0xA4` 위치제어 응답도 feedback 으로 인정
  - 6축에 대해 TX burst 후 RX collect 수행
  - `read_deadline_ms` / `read_poll_timeout_ms` 사용
  - full response 를 다 받아야만 success 반환
  - partial read 는 success 처리하지 않음
- `rsSendAndRecv()` 에 Robstride 응답 ID 검증 유지
  - `resp_motor = (id >> 8) & 0xFF`
  - `sent_motor = tx_id & 0xFF`
  - 다르면 해당 프레임 무시

### `src/frlab_arm_sdk/src/socketcan_device.cpp`

- `SocketCANDevice::setUsbLatency()` 로 `latency_timer=1ms` 설정 유지

### `src/frlab_arm_sdk/CMakeLists.txt`

- `FRLAB_BUILD_SCRIPTS` 옵션 추가
- 필요 시 아래 테스트 바이너리를 빌드 가능하게 유지
  - `test_frlab_arm`
  - `test_read_only`
  - `test_integrated`
  - `test_robstride`
  - `test_comm`
  - `test_socketcan`

### `src/ros2_controller/CMakeLists.txt`

- `3_finger_gripper_sdk` C++ 경로를 두 군데 순서대로 찾도록 변경
  - 상대 경로
  - `/home/frlab/3fing_ws/src/3fingSDK/3_finger_gripper_sdk/C++`
- `hand_position_controller` include path 도 위 SDK 기준으로 정리

### `src/readme.md`

- 작업 경로를 `/home/frlab/ing_ws` 기준으로 정리
- ROS `Humble` 기준으로 명령어 정리
- 빌드/launch/CAN setup/run 명령 현재 워크스페이스 기준으로 갱신

### `ARM417_200HZ_COMPARISON.md`

- 기준 SDK와 `armfullpk_417` 내장 SDK 차이, 200Hz 대응 포인트, 검증 절차를 정리한 문서 유지

## 3. 이번 세션에서 했다가 현재는 롤백된 실험

아래 변경은 한때 넣었다가 현재는 다시 뺐다.

- Robstride extended frame 을 `RS_MSG_FEEDBACK` 이외에도 넓게 허용하는 로직
- `readAll()` 안에서 Robstride `motor_id` 매칭 방식을 바꿨던 실험 로직

즉 현재 `readAll()` 은 "stale ACK 선소비 + missing joint만 추가 query" 구조는 유지하고, 이후에 얹었던 Robstride 추가 완화만 롤백된 상태다.

## 4. 현재 실기 동작 메모

- `controller_manager` 는 현재 `200Hz` 설정으로 뜸
- `joint_state_broadcaster`, `dls_controller` 는 순차 spawner 구조로 올라가도록 정리됨
- 하지만 실하드웨어에서는 현재 기준 코드에서 `FrlabArmHardware::read()` 가 연속 실패하는 로그가 계속 남아 있음
- `/joint_states` 는 대기 중 `~200Hz` publish 가 가능하지만, 실제 `5cm` 이동 같은 DLS 제어가 들어가면 실효 rate 는 대략 `135~140Hz` 수준으로 내려감

대표 증상:

```text
[FrlabArmHardware]: read() failed N times consecutively, holding cached state.
```

- `ros2_hardware` 쪽 cached state fallback 덕분에 read failure 자체가 즉시 controller_manager 종료로 이어지지는 않음
- 다만 실제 피드백 갱신은 막혀 있으므로, 현재 기준으로는 "200Hz 설정은 반영됐지만 실기 read 안정화는 아직 미완료" 상태
- `read_deadline_ms` 를 `4 -> 6` 으로 올린 뒤 실패 증가 속도는 줄었음
  - 이전: 대략 `2초에 +400`
  - 현재: 대략 `2초에 +293`
- 따라서 시간 예산 문제는 일부 맞지만, 단독 원인으로 보이지는 않음
- 현재 해석은 `시간 부족 + 특정 축/프레임 매칭 문제` 가 같이 있는 상태에 가까움
- `ros2_control_node` 가 간헐적으로 startup 초기에 `-6` 로 죽는 경우가 있었음
  - 이건 read loop 문제와 별개로 보이는 케이스가 섞여 있었음
  - stale process cleanup 후 다시 launch 하는 방식으로 확인했음

## 5. 마지막으로 다시 반영한 상태

마지막으로 다시 install 까지 반영한 패키지는 아래다.

```bash
cmake --build /home/frlab/ing_ws/build/frlab_arm_hardware --target install -j4
```

즉 현재 실행 시 실제로 로드되는 `libfrlab_arm_hardware.so` 는 이 문서 기준 상태와 맞는다.

## 6. 자주 쓰는 명령

### build

```bash
cd /home/frlab/ing_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select description frlab_arm_hardware pin_dls_controller --cmake-clean-cache
source /home/frlab/ing_ws/install/setup.bash
```

### clean restart

```bash
bash -lc "pkill -9 -f 'frlab_arm_control.launch.py' || true; pkill -9 -f 'ros2_control_node' || true; pkill -9 -f 'controller_manager/spawner' || true; pkill -9 -f 'robot_state_publisher' || true; pkill -9 -f 'rviz2' || true; pkill -9 -f 'joint_state_broadcaster' || true; pkill -9 -f 'dls_controller' || true"
```

### real hardware launch

```bash
cd /home/frlab/ing_ws
source /opt/ros/humble/setup.bash
source /home/frlab/ing_ws/install/setup.bash
ros2 launch description frlab_arm_control.launch.py use_fake_hardware:=false
```

### fake hardware launch

```bash
cd /home/frlab/ing_ws
source /opt/ros/humble/setup.bash
source /home/frlab/ing_ws/install/setup.bash
ros2 launch description frlab_arm_control.launch.py use_fake_hardware:=true
```

### joint state 주기 확인

```bash
ros2 topic hz /joint_states
```
