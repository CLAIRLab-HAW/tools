cmake_minimum_required(VERSION 3.20)
project(rg6_control)

find_package(ament_cmake REQUIRED)

# The RG6's own: the bridge to the OnRobot URCap with its gear table, and the MoveIt patch.  What every model shares
# (the mock, the joint_states helpers) is gripper_control's.
install(PROGRAMS scripts/rg6_grip_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
# Installed as a command, without .py: `ros2 run rg6_control rg6_moveit_patch`, the offboard and MARWIN images and
# the husky installer call it by that name.
install(PROGRAMS scripts/rg6_moveit_patch.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rg6_moveit_patch
)
install(FILES scripts/rg6_finger_kinematics.json
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
