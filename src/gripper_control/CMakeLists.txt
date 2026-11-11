cmake_minimum_required(VERSION 3.20)
project(gripper_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# gripper_sim: any gripper model's GripperCommand surface without hardware (its model comes in as parameters).
add_executable(gripper_sim src/gripper_sim.cpp)
target_include_directories(gripper_sim PRIVATE include)
add_executable(joint_state_relay src/joint_state_relay.cpp)

target_link_libraries(gripper_sim
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${control_msgs_TARGETS}
)
target_link_libraries(joint_state_relay PUBLIC
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS gripper_sim joint_state_relay
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
