cmake_minimum_required(VERSION 3.8)
project(tracer_piper_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# MoveItConfigsBuilder reads this file to resolve the URDF and SRDF without
# printing fallback warnings during launch description generation.
install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
