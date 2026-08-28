cmake_minimum_required(VERSION 3.8)
project(dog_cartographer_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/cartographer_initial_pose_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
