cmake_minimum_required(VERSION 3.8)
project(hexapod_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config worlds launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
