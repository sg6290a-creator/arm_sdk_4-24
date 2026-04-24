cmake_minimum_required(VERSION 3.14)
project(frlab_arm_sdk CXX)

# ---------------------------------------------------------------------------
# frlab_arm_sdk — 6DOF Manipulator SDK (Robstride03 x2 + MyActuator X4-36 x4)
#
# Transport: SocketCAN via CANable v2.0 (slcan/native CAN)
#
# Structure:
#   include/arm_sdk/   — public headers
#   src/               — implementation
#   scripts/           — standalone test executables
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# ---------------------------------------------------------------------------
# arm_sdk_native — static library
# ---------------------------------------------------------------------------
add_library(arm_sdk_native STATIC
  src/socketcan_device.cpp
  src/rmd_protocol.cpp
  src/robstride_protocol.cpp
  src/integrated_driver.cpp
  src/frlab_arm.cpp
)

set_target_properties(arm_sdk_native PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(arm_sdk_native PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(arm_sdk_native PUBLIC cxx_std_17)

find_package(Threads REQUIRED)
target_link_libraries(arm_sdk_native PUBLIC Threads::Threads)

message(STATUS "[frlab_arm_sdk] arm_sdk_native built.")

# ---------------------------------------------------------------------------
# Test / script executables
# ---------------------------------------------------------------------------
option(FRLAB_BUILD_SCRIPTS "Build script executables" OFF)

if(FRLAB_BUILD_SCRIPTS)
  foreach(script
    test_frlab_arm
    test_read_only
    test_integrated
    test_robstride
    test_comm
    test_socketcan
  )
    add_executable(${script} scripts/${script}.cpp)
    target_link_libraries(${script} PRIVATE arm_sdk_native)
    message(STATUS "[frlab_arm_sdk] ${script} built.")
  endforeach()
endif()
