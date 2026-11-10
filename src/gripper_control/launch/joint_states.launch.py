#!/usr/bin/env python3
"""Robot-wide joint_states aggregation + legacy bus relay for the a200_0553.

Background (multi-CM Clearpath): the wheel, arm and gripper joint_states come from THREE separate sources and are not
automatically merged into one complete /joint_states. This launch file establishes:

  1. joint_state_aggregator: merges the (partial) sources into ONE complete
     /a200_0553/joint_states (with velocity+effort) ─▶ for rosbag/Foxglove.
     Deliberately NOT as a live TF feed (an aggregator in the TF path = SPOF).

  2. joint_state_relay: mirrors the clean source topics manipulators/joint_states
     and manipulators/endeffectors/joint_states back onto
     /a200_0553/platform/joint_states, which Clearpath's robot_state_publisher (x2)
     and move_group subscribe to via the stock launch. This leaves the live
     consumers (TF/MoveIt) UNTOUCHED while arm and gripper publish in the correct
     namespace.

     IMPORTANT: a relay node of our own (NOT topic_tools relay), because this one
     publishes with EXPLICITLY RELIABLE publisher QoS. move_group subscribes to
     platform/joint_states RELIABLE ─▶ a best-effort publisher (the topic_tools
     default) would NOT arrive there ─▶ the state would be displayed correctly
     (best-effort RSP) but planning would fail. See joint_state_relay.cpp.

Preconditions:
  - The arm JSB remap in clearpath_manipulators/control.launch.py is set to
    manipulators/joint_states by clearpath-custom-setup.
  - The gripper publishes on manipulators/endeffectors/joint_states.  The source is
    model's driver (rg6_grip_bridge on the robot, 5 Hz) or gripper_sim (in the container).
The wheels stay (correctly) on platform/joint_states.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

NAMESPACE = "a200_0553"


def generate_launch_description():
    # Aggregate for observation/recording: /a200_0553/joint_states (complete).
    aggregator = Node(
        package="gripper_control",
        executable="joint_state_aggregator",
        namespace=NAMESPACE,
        name="joint_state_aggregator",
        output="screen",
        parameters=[
            {
                # relative names ─▶ resolved in the /a200_0553 namespace.
                "source_topics": [
                    "platform/joint_states",  # Raeder
                    "manipulators/joint_states",  # Arm
                    "manipulators/endeffectors/joint_states",  # gripper
                ],
                "output_topic": "joint_states",  # ─▶ /a200_0553/joint_states
                "publish_rate": 50.0,
            }
        ],
    )

    # Keep the legacy bus alive: arm + gripper back onto platform/joint_states, RELIABLE (for the reliable move_group
    # subscriber; see joint_state_relay.cpp). The subscriptions reconnect automatically when clearpath-manipulators
    # (and with it the arm JSB) restarts.
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

    return LaunchDescription([aggregator, relay])
