cmake_minimum_required(VERSION 3.8)
project(agx_arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 2.0 只保留组合机器人模型实际引用的 Piper 网格资源。
install(
  DIRECTORY agx_arm_urdf/
  DESTINATION share/${PROJECT_NAME}/agx_arm_urdf/
)

ament_package()
