cmake_minimum_required(VERSION 3.5)
project(tool_description)

find_package(ament_cmake REQUIRED)

# Data only.  urdf/ and meshes/ land under share/ because RViz and the foxglove_bridge resolve
# package://tool_description/meshes/... through this install, and the robots' extras include the macros by $(find).
install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
