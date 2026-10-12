cmake_minimum_required(VERSION 3.8)
project(slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/save_map
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
