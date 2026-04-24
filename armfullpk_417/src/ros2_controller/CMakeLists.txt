cmake_minimum_required(VERSION 3.8)
project(pin_dls_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(forward_command_controller REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(std_msgs REQUIRED)

# ── 3fingSDK (plain CMake, ROS2 의존성 없음) ──────────────────────────
find_library(DXL_LIB dxl_x64_cpp PATHS /usr/local/lib REQUIRED)

set(GRIPPER_SDK_CPP_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../3fingSDK/3_finger_gripper_sdk/C++")

if(NOT EXISTS "${GRIPPER_SDK_CPP_DIR}/CMakeLists.txt")
  set(GRIPPER_SDK_CPP_DIR
    "/home/frlab/3fing_ws/src/3fingSDK/3_finger_gripper_sdk/C++")
endif()

if(NOT EXISTS "${GRIPPER_SDK_CPP_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "3_finger_gripper_sdk C++ source not found. Checked: "
    "${CMAKE_CURRENT_SOURCE_DIR}/../3fingSDK/3_finger_gripper_sdk/C++ and "
    "/home/frlab/3fing_ws/src/3fingSDK/3_finger_gripper_sdk/C++")
endif()

add_subdirectory(
  ${GRIPPER_SDK_CPP_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/gripper_sdk
)

# ── DLS controller ────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/pin_dls_solver.cpp
  src/pin_dls_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  forward_command_controller
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  Eigen3
)

target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
)

pluginlib_export_plugin_description_file(controller_interface pin_dls_controller_plugins.xml)

# ── Hand position controller (standalone node) ────────────────────────
add_executable(hand_position_controller
  src/hand_poisition_controller.cpp
)

target_include_directories(hand_position_controller PRIVATE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  ${GRIPPER_SDK_CPP_DIR}/include
  /usr/local/include
)

ament_target_dependencies(hand_position_controller
  rclcpp
  std_msgs
)

target_link_libraries(hand_position_controller
  gripper_sdk
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  TARGETS hand_position_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES pin_dls_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  controller_interface
  forward_command_controller
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  Eigen3
)

ament_package()
