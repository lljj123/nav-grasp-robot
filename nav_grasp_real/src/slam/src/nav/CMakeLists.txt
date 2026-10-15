cmake_minimum_required(VERSION 3.8)
project(nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
