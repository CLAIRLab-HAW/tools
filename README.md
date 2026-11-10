# tools

ROS 2 Jazzy packages for end effectors: parallel grippers (OnRobot RG6, Robotiq 2F-85) and the tool macros.

## Layout

| Path | Type | Contents |
|---|---|---|
| `src/gripper_control` | ament_cmake | `gripper_sim`, `linkage.hpp`, `joint_state_relay`, `joint_state_aggregator` |
| `src/rg6_control` | ament_cmake | `rg6_grip_bridge.py` with `rg6_finger_kinematics.json`, `rg6_moveit_patch` |
| `src/rg6_description` | ament_cmake | OnRobot RG6 v2: URDF/xacro, meshes, `config/rg6_v2.yaml` |
| `src/tool_description` | ament_cmake | Tool macros `rg6_tool(parent)` and `key_holder(parent)`, key holder meshes |
| `scripts/` | — | `derive_finger_kinematics.py`, `derive_link_inertia.py` (not installed) |
| `tests/` | — | pytest suite |

The Robotiq 2F-85 description is `ros-jazzy-robotiq-description` (via `clearpath_manipulators_description`).

## Interface

The same names for the real driver and for `gripper_sim`, relative to the manipulator namespace `<ns>`:

| | RG6 (`rg6_v2`) | 2F-85 (`robotiq_2f_85`) | Type |
|---|---|---|---|
| Command | `rg6_gripper_controller/gripper_cmd` | `arm_0_gripper_controller/gripper_cmd` | `GripperCommand` action |
| State | `rg6/bridge_state` | `arm_0_gripper/state` | `std_msgs/String`, flat JSON |
| Driver joint | `rg6_finger_joint` | `arm_0_gripper_robotiq_85_left_knuckle_joint` | on `endeffectors/joint_states` |

- `command.position`: driver joint in rad, not the width. RG6 `0.038` open … `1.25478` closed; 2F-85 `0` … `0.8`.
- `command.max_effort`: grip force in N, clamped to the model's effort range (RG6: 25 … 120).
- State fields: `width_m`, `busy`, `grip_detected`, `status`, `safety_failed`, `last_command`.
- The action result acknowledges acceptance; the reached width is on the state topic.
- One command at a time: a second goal during a motion is rejected.

## gripper_sim

Container mock for any gripper model. Parameters: `driver_joint`, `action_name`, `state_topic`, `linkage_q_rad`,
`linkage_width_m`; optional `sim_object_width_m` (closing stops there, `grip_detected=true`).

```bash
ros2 run gripper_control gripper_sim --ros-args -r __ns:=<ns> --params-file model.yaml
```

| Mode | Width source | Settles when |
|---|---|---|
| kinematic (default) | constant-speed ramp, full stroke in ~1 s | target reached |
| plant (`plant_command_topic` set) | `plant_state_topic` (`/joint_states`) | jaws stand still after the command |

## joint_states plumbing

`launch/joint_states.launch.py` (gripper_control):

| Node | Input | Output |
|---|---|---|
| `joint_state_aggregator` | arm, gripper, platform joint states | one complete `joint_states` |
| `joint_state_relay` | `manipulators/joint_states`, `…/endeffectors/joint_states` | `platform/joint_states` |

The relay publishes RELIABLE because `move_group` subscribes RELIABLE.

## RG6

**Command path.** `rg6_grip_bridge` commands the gripper over XML-RPC through the OnRobot URCap
(`rg_grip`, port 41414 on the UR controller) and publishes `rg6_finger_joint` at 5 Hz. Without a measurement it
publishes the last measured angle, or `0.038` rad before the first one, with `status: -1`. The URCap occupies the
RTDE `tool_digital_output_mask`; `ur_robot_driver` therefore runs on an input recipe without
`tool_digital_output*`, and ROS sets no tool digital output.

**Width.** The device reports +3 … +5 mm more than the jaws open, direction-dependent, ~1.7 mm hysteresis. The
commanded value is the reference. Tool AI2/AI3 are not a width source.

**MoveIt.** `moveit.yaml` entries (controller `rg6_gripper_controller`, type `GripperCommand`, joint limits of
`rg6_finger_joint`) come from the robot's Clearpath configuration. `robot.srdf` has no such hook and is patched:

```bash
rg6_moveit_patch --setup-path /etc/clearpath
```

The patch writes a marked block into `robot.srdf` (atomic, `.bak`, idempotent): planning group `gripper`
(`rg6_finger_joint`), group states `open` (0.0) and `close` (1.25478), end effector `rg6` (`parent_link`
`arm_0_wrist_3_link`, `parent_group` `arm_0`), and `<disable_collisions>` for `moment_arm` ↔ `finger_tip` per finger.
`verify_moveit_yaml` exits 1 and lists the keys when the `moveit.yaml` entries are missing.

## Width ↔ joint

The driver joint → clear width mapping is a generated table (`linkage.table` in the model file):

| Reader | Where |
|---|---|
| `gripper_sim` | `include/gripper_control/linkage.hpp`, table passed as parameters |
| `rg6_grip_bridge` | `rg6_finger_kinematics.json`: generated copy, `0.038 … 1.25478` rad, max error `4.7e-05` m |

`scripts/derive_finger_kinematics.py` writes the table from a generated URDF; rerun it after every model change. The
upper joint limit is the zero crossing of the width.

## Build

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Tests

```bash
uv sync
uv run pytest
```

Compiles `linkage.hpp` and checks it, the bridge's table and the generators. No ROS required. Tests that compare
against files outside this repository skip when those files are absent.

## Safety

- A `GripperCommand` goal moves hardware: clear workspace, e-stop in reach.
- `busy` reads `false` for ~0.4 s after a goal is accepted; wait for both edges, not for the first `false`.

## Versioning

[CHANGELOG.md](CHANGELOG.md); `tool_description` keeps its own.

## License

`rg6_description`: see `src/rg6_description/LICENSE`. `rg6_control`, `gripper_control`: BSD-3-Clause.
`tool_description`: MIT.
