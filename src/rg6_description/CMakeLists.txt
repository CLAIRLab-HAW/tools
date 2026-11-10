cmake_minimum_required(VERSION 3.5)
project(rg6_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
