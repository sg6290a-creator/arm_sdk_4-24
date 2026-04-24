# 빌드 명령어
cd /home/frlab/ing_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select description frlab_arm_hardware rbdl_dls_controller --cmake-clean-cache
source /home/frlab/ing_ws/install/setup.bash


# CAN 셋업 명령어 (slcan 펌웨어 — ttyACM0 or ttyACM1)
sudo slcand -o -c -s8 /dev/ttyACM0 can0 && sudo ip link set up can0

# candleright
lsusb | grep 1d50
sudo ip link set can0 up type can bitrate 1000000

# 런치 실행 명령어
ros2 launch description frlab_arm_control.launch.py use_fake_hardware:=false


# 현재 EE 위치 확인
ros2 run tf2_ros tf2_echo base_link gripper_palm


# 현재 자세 기준 5cm Z축 위로 
ros2 topic pub --once /dls_controller/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.146, y: -0.043, z: 0.453},
    orientation: {x: 0.104, y: -0.142, z: 0.923, w: -0.342}
  }
}"

# RT
sudo setcap cap_sys_nice+ep /opt/ros/humble/lib/controller_manager/ros2_control_node


# 현재 상태 / 현실적인 판단 (2026-04-18)

## 결론 먼저

- 무부하 대기 상태에서는 `/joint_states` 기준 `~200Hz` publish 는 실제로 나옴
- 하지만 `5cm` 이동 같은 실제 DLS 제어가 들어가면 현재 실효 update rate 는 대략 `135~140Hz` 수준으로 내려감
- 즉 "지금 코드 상태에서 실기 제어까지 포함한 안정적인 200Hz" 는 아직 달성되지 않음
- 현실적으로는 "200Hz 설정 및 idle publish 는 가능, motion 포함 full closed-loop 200Hz 는 아직 미완료" 로 보는 게 맞음

## 지금까지 확인된 사실

- `controller_manager.update_rate` 는 `200Hz` 로 동작
- `/joint_states` 는 대기 중 `~200Hz` 로 publish 되는 구간이 확인됨
- `read_deadline_ms` 를 `4ms -> 6ms` 로 올린 뒤 `read()` 연속 실패 증가 속도는 줄었음
- 이전에는 대략 `2초에 +400` 수준으로 실패 카운트가 증가했고, `6ms` 적용 뒤에는 대략 `2초에 +293` 수준으로 줄었음
- 즉 시간 예산이 촉박했던 것은 맞지만, 그것만이 유일한 원인은 아님

## 현재 해석

- `6ms` 로 늘리니까 일부 cycle 은 살아남음
- 그런데 여전히 많은 cycle 에서 6축 응답이 전부 다 안 모임
- 현재 SDK read 경로는 "6축 full response 가 모두 와야 success" 로 판단함
- 그래서 응답이 일부 늦거나 특정 프레임이 누락되면 그 cycle 전체가 실패로 집계됨

한 줄 요약:

- `시간 부족 + 특정 축/프레임 매칭 문제` 가 같이 있는 상태로 보임

## 의미

- 지금은 "통신이 완전히 끊긴 상태" 라기보다
- "응답은 오는데, 현재 read window 안에 전부 다 못 모으거나 일부 프레임을 유효 응답으로 못 잡는 상태" 에 더 가까움
- 그래서 idle 상태에서는 200Hz publish 가 보이지만, 실제 움직임이 들어가면 계산 + 통신 부담이 같이 올라가면서 실효 주기가 내려감

## 다음 우선순위

- 더 무작정 `ms` 만 늘리기보다
- 어떤 joint 가 주로 빠지는지 로그로 찍기
- 그 축이 `RMD` 인지 `Robstride` 인지 먼저 특정하기
- 그 다음에야 deadline 추가 조정이나 read 매칭 로직 수정을 판단하는 게 안전함
