cmake_minimum_required(VERSION 3.8)
project(description)

find_package(ament_cmake REQUIRED)

# rviz 폴더를 삭제하고 config 폴더를 포함시킨다.
install(DIRECTORY 
  urdf 
  meshes 
  launch 
  config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
