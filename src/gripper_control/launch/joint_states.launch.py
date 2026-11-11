#!/usr/bin/env python3
"""Robot-wide joint_states bus for the a200_0553: the arm and gripper relay.

Background (multi-CM Clearpath): the wheel, arm and gripper joint_states come from THREE separate sources.  The
wheels publish on /a200_0553/platform/joint_states; joint_state_relay mirrors the clean source topics
manipulators/joint_states and manipulators/endeffectors/joint_states onto the same topic, which Clearpath's
robot_state_publisher (x2) and move_group subscribe to via the stock launch.  That one topic is the whole robot's
joint bus, each message one part of it; its readers gather by name.

IMPORTANT: a relay node of our own (NOT topic_tools relay), because this one publishes with EXPLICITLY RELIABLE
publisher QoS. move_group subscribes to platform/joint_states RELIABLE ─▶ a best-effort publisher (the topic_tools
default) would NOT arrive there ─▶ the state would be displayed correctly (best-effort RSP) but planning would fail.
See joint_state_relay.cpp.

Preconditions:
  - The arm JSB remap in clearpath_manipulators/control.launch.py is set to
    manipulators/joint_states by clearpath-custom-setup.
  - The gripper publishes on manipulators/endeffectors/joint_states.  The source is
    model's driver (rg6_grip_bridge on the robot, 5 Hz) or gripper_sim (in the container).
"""

from launch import LaunchDescription
from launch_ros.actions import Node

NAMESPACE = "a200_0553"


def generate_launch_description():
    # Arm + gripper onto platform/joint_states, RELIABLE (for the reliable move_group subscriber; see
    # joint_state_relay.cpp). The subscriptions reconnect automatically when clearpath-manipulators (and with it the
    # arm JSB) restarts.
    relay = Node(
        package="gripper_control",
        executable="joint_state_relay",
        namespace=NAMESPACE,
        name="joint_state_relay",
        output="screen",
        parameters=[
            {
                "input_topics": ["manipulators/joint_states", "manipulators/endeffectors/joint_states"],
                "output_topic": "platform/joint_states",  # ─▶ /a200_0553/platform/joint_states
                "depth": 20,
            }
        ],
    )

    return LaunchDescription([relay])
