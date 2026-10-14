cmake_minimum_required(VERSION 3.8)
project(tracer_gazebo_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps meshes rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/set_piper_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
