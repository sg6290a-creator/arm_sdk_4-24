cmake_minimum_required(VERSION 3.14)
project(arm_sdk_cpp CXX)

# ---------------------------------------------------------------------------
# arm_sdk native layer — NO ROS dependencies.
#
# 5-layer structure:
#   usbcan_device       — USBCAN adapter wrapper (dlopen libusbcan.so)
#   rmd_protocol        — RMD motor packet build/parse
#   robstride_protocol  — Robstride motor packet build/parse
#   arm_driver          — high-level RMD arm controller
#   integrated_driver   — RMD + Robstride 통합 컨트롤러
#
# ros2/ 계층은 ArmDriver 를 has-a 멤버로 사용하는 SystemInterface 를 빌드한다.
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# ---------------------------------------------------------------------------
# arm_sdk_native — 순수 C++ 라이브러리 (ROS 의존 없음)
# ---------------------------------------------------------------------------
add_library(arm_sdk_native STATIC
  src/usbcan_device.cpp
  src/rmd_protocol.cpp
  src/robstride_protocol.cpp
  src/arm_driver.cpp
  src/integrated_driver.cpp
)
set_target_properties(arm_sdk_native PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(arm_sdk_native PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(arm_sdk_native PUBLIC cxx_std_17)

# Link pthread + dl (for dlopen of libusbcan.so)
find_package(Threads REQUIRED)
target_link_libraries(arm_sdk_native PUBLIC Threads::Threads dl)

message(STATUS "[arm_sdk/C++] arm_sdk_native built (pure C++, no ROS).")

# ---------------------------------------------------------------------------
# Optional test executables (standalone, no ROS needed)
# ---------------------------------------------------------------------------
option(ARM_SDK_BUILD_TESTS "Build native test executables" OFF)

if(ARM_SDK_BUILD_TESTS)
  add_executable(test_arm test/test_arm.cpp)
  target_link_libraries(test_arm PRIVATE arm_sdk_native)
  message(STATUS "[arm_sdk/C++] test_arm built.")

  add_executable(test_robstride test/test_robstride.cpp)
  target_link_libraries(test_robstride PRIVATE arm_sdk_native)
  message(STATUS "[arm_sdk/C++] test_robstride built.")

  add_executable(test_integrated test/test_integrated.cpp)
  target_link_libraries(test_integrated PRIVATE arm_sdk_native)
  message(STATUS "[arm_sdk/C++] test_integrated built.")

  add_executable(test_comm test/test_comm.cpp)
  target_link_libraries(test_comm PRIVATE arm_sdk_native)
  message(STATUS "[arm_sdk/C++] test_comm built.")

  add_executable(test_socketcan test/test_socketcan.cpp)
  target_link_libraries(test_socketcan PRIVATE arm_sdk_native)
  message(STATUS "[arm_sdk/C++] test_socketcan built.")
endif()
