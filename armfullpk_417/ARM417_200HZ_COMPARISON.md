# ARM417 200Hz Comparison

`armfullpk_417`를 실하드웨어 기준 `200Hz+`로 끌어올리기 위해, 기존 기준 패키지인 `frlab_arm_sdk`와 현재 `armfullpk_417/src/frlab_arm_sdk`를 비교한 결과와 적용한 개선 포인트를 정리한다.

## 1. 현재 100Hz 원인

- 직접 원인: [`controllers.yaml`](./src/description/config/controllers.yaml) 의 `controller_manager.update_rate` 가 `100` 으로 고정되어 있었다.
- 구조상 원인: `ros2_control` 한 주기마다 `read()` 와 `write()` 가 모두 실행되므로, SDK의 read 경로가 `5ms` 예산 안에 안정적으로 들어와야 `200Hz`가 가능하다.

## 2. 성능 관련 차이

### 기준 패키지 `frlab_arm_sdk`

- README 기준으로 `test_read_only can0 2` 에서 약 `585Hz` 수준의 고속 위치 홀드 루프를 목표로 설계되어 있다.
- Robstride 응답 검증이 들어 있어 stale frame 오인식 위험이 낮다.
- 별도 `test_read_only.cpp` 가 있어 SDK 단독 통신 성능을 ROS2와 분리해서 측정할 수 있다.

### armfull_417 내장 SDK (`src/frlab_arm_sdk`)

- 이미 `readAll()` 은 TX burst + RX collect 형태로 파이프라인화되어 있다.
- `SocketCANDevice::setUsbLatency()` 로 `latency_timer=1ms` 를 설정하는 최적화가 추가되어 있다.
- 하지만 아래 리스크가 있었다.
  - `readAll()` deadline 이 `8ms` 하드코딩
  - partial read(`collected > 0`) 도 성공으로 처리
  - `rsSendAndRecv()` 에 Robstride 응답 ID 검증이 빠져 있음
  - armfull 패키지 내부에는 `test_read_only.cpp` 가 없어 SDK 단독 성능 재현이 어려움

## 3. 하드웨어 매핑 차이

이 항목은 주기 병목 원인이 아니라 기능/배선 차이다.

- `armfullpk_417/src/frlab_arm_sdk/include/arm_sdk/frlab_arm.hpp` 는 `joint_2` 를 Robstride `ID=2` 로 설명한다.
- 하지만 실제 구현인 `armfullpk_417/src/frlab_arm_sdk/src/frlab_arm.cpp` 는 여전히 `joint_2` 를 `ID=127` 로 구성한다.
- 이번 변경에서는 주기 개선만 다루고, 실제 모터 ID 매핑은 건드리지 않았다.

## 4. 이번에 적용한 200Hz 대응 수정

### ROS2 상단

- `controller_manager.update_rate` 를 `100 -> 200` 으로 상향.

### SDK

- `IntegratedDriverConfig` 에 아래 timing 파라미터 추가
  - `read_deadline_ms` 기본 `3`
  - `read_poll_timeout_ms` 기본 `1`
- `readAll()` 을 `200Hz` 예산에 맞게 보정
  - `8ms` 고정 deadline 제거
  - full response 수집 시에만 성공 반환
  - joint별 중복 수신을 한 번만 카운트
- `rsSendAndRecv()` 에 Robstride 응답 ID 검증 복원

### ros2_hardware

- 하드웨어 파라미터 추가
  - `read_deadline_ms`
  - `read_poll_timeout_ms`
  - `perf_log_every_n_cycles`
- `read()` 실패 로그를 throttle 로 변경해서 고속 주기에서 로그가 병목이 되지 않게 조정
- cached fallback 정책은 유지하되, SDK full read 실패 시에만 fallback 이 동작하게 정리
- `perf_log_every_n_cycles > 0` 이면 평균 `read_ms`, `write_ms`, `cycle_ms`, `read_failures` 를 로그 출력

### 재현용 도구

- `armfullpk_417/src/frlab_arm_sdk/scripts/test_read_only.cpp` 추가
- armfull 패키지 내부에서 바로 SDK 단독 통신 성능을 측정할 수 있게 CMake 타깃에 포함

## 5. 실하드웨어 검증 절차

### SDK 단독

1. `armfullpk_417/src/frlab_arm_sdk` 빌드 시 `FRLAB_BUILD_SCRIPTS=ON` 으로 `test_read_only` 를 생성한다.
2. `sudo ./test_read_only can0 5` 로 `5ms` 주기 테스트를 `60초` 이상 실행한다.
3. 합격 기준
   - 평균 주기 `<= 5.0ms`
   - 응답 혼선(stale/misaligned frame) 반복 없음
   - read timeout 이 있어도 연속적으로 누적되지 않음

### ros2_control e2e

1. `use_fake_hardware:=false` 로 실제 하드웨어 launch 실행
2. URDF 하드웨어 파라미터는 기본값 사용
   - `read_deadline_ms=3`
   - `read_poll_timeout_ms=1`
   - `perf_log_every_n_cycles=0`
3. 필요 시 `perf_log_every_n_cycles=200` 정도로 켜고 평균 주기 확인
4. 합격 기준
   - 평균 `cycle_ms < 5.0`
   - `controller_manager` 가 `200Hz` 로 유지됨
   - read failure 가 발생해도 cached fallback 으로 복구되고 시스템이 즉시 죽지 않음
   - `joint_state_broadcaster` 와 `dls_controller` 가 정상 동작

## 6. 우선순위

1. `controller_manager.update_rate=200`
2. SDK read budget/응답 판정/Robstride 응답 검증 보정
3. ros2_hardware throttled logging + perf logging
4. armfull 내부 `test_read_only` 로 SDK 단독 검증
5. 필요 시 이후 단계에서 write 경로 파이프라인화 검토

## 7. 기본값

- 1차 목표는 `250Hz` 가 아니라 실하드웨어에서 안정적인 `200Hz`
- `pin_dls_controller` 의 알고리즘/게인은 1차 변경 대상에서 제외
- `latency_timer=1ms` 최적화는 유지
