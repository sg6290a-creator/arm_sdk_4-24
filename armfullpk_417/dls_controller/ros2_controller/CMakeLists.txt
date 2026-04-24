cmake_minimum_required(VERSION 3.8)
project(rbdl_dls_controller)

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
find_package(std_msgs REQUIRED)

find_path(RBDL_INCLUDE_DIR rbdl/rbdl.h
  PATHS
    /usr/local/include
    /usr/include
    /home/frlab/frlab_arm/rbdl/include
    /home/frlab/frlab_arm/rbdl/build/include
)

find_library(RBDL_LIB NAMES rbdl
  PATHS
    /usr/local/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /home/frlab/frlab_arm/rbdl/build
)

if(NOT RBDL_INCLUDE_DIR OR NOT RBDL_LIB)
  message(FATAL_ERROR
    "RBDL not found. Install librbdl or provide rbdl/rbdl.h and librbdl.so. "
    "Checked /usr/local, /usr, and /home/frlab/frlab_arm/rbdl.")
endif()

set(RBDL_INCLUDE_DIRS ${RBDL_INCLUDE_DIR})
if(EXISTS "/home/frlab/frlab_arm/rbdl/build/include")
  list(APPEND RBDL_INCLUDE_DIRS "/home/frlab/frlab_arm/rbdl/build/include")
endif()

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

add_library(${PROJECT_NAME} SHARED
  src/rbdl_dls_solver.cpp
  src/rbdl_dls_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${RBDL_INCLUDE_DIRS}
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
  ${RBDL_LIB}
)

pluginlib_export_plugin_description_file(controller_interface rbdl_dls_controller_plugins.xml)

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
  FILES rbdl_dls_controller_plugins.xml
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
