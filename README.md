# tools — the hands and tools on the flange

What the a200-0553 ([husky](https://github.com/CLAIRLab-HAW/husky)) and MARWIN 5
([marwin](https://github.com/CLAIRLab-HAW/marwin)) both carry, one colcon workspace under `src/`: the grippers
(`rg6_description`, `rg6_control`, `gripper_control`) and the tool macros (`tool_description`). On the a200-0553 it
is the clone `/home/robot/tools`, which husky's installer clones and builds.

The hand on the arm's flange, for any **gripper model**: what every model shares, the parts of the models this
workspace builds itself, and the tools that read a model's numbers off its URDF. Two models today — the **OnRobot RG6**
the a200-0553 carries, and the **Robotiq 2F-85** — and a third is a model file, not a code change.

A gripper model is described ONCE, in the robot contract: `robot/contract`'s `profiles/grippers/<model>.yaml`
(driver joint, controller, state topic, linkage table, pad arc, data sheet, touch links, TCP). A robot's profile
names the model on its flange (`gripper.model`) and the namespace its controller lives in (`gripper.ns`);
`ROBOT_CONTRACT_GRIPPER=<model>` swaps it for a process. Every consumer — the plan server, the mock below, the sims —
reads the model from there and names none.

**The real RG6 is not driven by a ROS controller.** On the robot it is commanded by **`rg6_grip_bridge`** (onboard,
Python, `src/rg6_control/scripts/rg6_grip_bridge.py`) over **XML-RPC against the OnRobot URCap**. A model Clearpath
supports itself (`clearpath_config`'s `robotiq_2f_85`, `robotiq_2f_140`, `kinova_2f_lite`, `franka_gripper`) brings
its description from `clearpath_manipulators_description`; only its model file and its tool file are this
workspace's.

## Features

- **One surface for every model, on both stages** — a `GripperCommand` action at
  `<gripper.ns>/<controller>/gripper_cmd`, a flat-JSON state topic and the driver joint on
  `manipulators/endeffectors/joint_states`, whether a real driver or the container mock answers.
- **A container mock for any model** (`gripper_control`'s `gripper_sim`): the model's driver joint, action, state
  topic and linkage table come in as parameters, read off the profile by the launcher. It offers exactly the real
  surface — no more, so nobody writes code against something that does not exist — and in **plant mode** it drives
  the jaws of a physics plant or any hand a position controller moves.
- **`joint_states` plumbing** for a multi-controller-manager robot (`gripper_control`): one complete aggregate for
  recording, plus a RELIABLE relay `move_group` actually receives.
- **A generated gear table** instead of a formula, for every model: `scripts/derive_finger_kinematics.py` walks the
  finger chain of a GENERATED URDF (mimic joints included) and writes the linkage and pad tables a model file
  carries. The pads' faces are their link frames (rg6_v2) or the innermost points of their collision meshes (2F-85).
- **The RG6's own** (`rg6_description`, `rg6_control`): the measured rg6_v2 model, the bridge to the URCap and the
  SRDF patch Clearpath's generator cannot write. Where a hand is mounted on the a200-0553 is not here but in
  husky's `husky_extras_description` (`urdf/tools/<tool>.urdf.xacro`, over
  [tool_description](src/tool_description/README.md)'s macro).

## Tech Stack

ROS 2 Jazzy, `ament_cmake` (C++17), `control_msgs`, `sensor_msgs`, `std_msgs`; the tools need `numpy`, the mesh mode
`trimesh` (the offboard container has both). No interface package of its own — the state is JSON in a
`std_msgs/String`.

### The canonical interface

Both stages — real robot and offboard container — serve the **same names**, so a caller never branches on which one
it is talking to. They come from the model file; on the a200-0553 (`<ns>` = `/a200_0553/manipulators`):

| What | RG6 (`rg6_v2`) | 2F-85 (`robotiq_2f_85`) | Type |
|---|---|---|---|
| Command | `rg6_gripper_controller/gripper_cmd` | `arm_0_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |
| State | `rg6/bridge_state` | `arm_0_gripper/state` | `std_msgs/String` (flat JSON) |
| Driver joint | `rg6_finger_joint` | `arm_0_gripper_robotiq_85_left_knuckle_joint` | on `endeffectors/joint_states` |

* **`command.position` is the driver joint in rad**, not the width (RG6: `0.038` open, `1.25478` closed; 2F-85: `0`
  open, `0.8` closed). Every receiver converts it through the model's gear table (see [Width ↔ joint](#width--joint)).
* **`max_effort` is the grip force in N**, clamped to the model's `effort_range_n`.
* The state carries `width_m`, `busy`, `grip_detected`, `status`, `safety_failed`, `last_command`. It is JSON in a
  `std_msgs/String` rather than a typed message so that a consumer needs no interface package of ours built and
  sourced — `std_msgs` is enough.

> **Read the state topic before claiming anything about the gripper.** A `GripperCommand` result acknowledges that
> the goal was *accepted*; the width reached is what the state reports afterwards. `rg6_grip_bridge` takes **one
> command at a time** and rejects a second one while moving.

### Packages

| Package | Type | Contents |
|---|---|---|
| `gripper_control` | `ament_cmake` (C++) | every model's: `gripper_sim` (container mock), `include/gripper_control/linkage.hpp`, `joint_state_relay`, `joint_state_aggregator`, `joint_states.launch.py` |
| `rg6_control` | `ament_cmake` | the RG6's: `scripts/rg6_grip_bridge.py` with its table `rg6_finger_kinematics.json`, `scripts/rg6_moveit_patch` |
| `rg6_description` | `ament_cmake` | the measured **RG6 v2** model: URDF/Xacro, visual + collision meshes, `config/rg6_v2.yaml` |
| `scripts/` | — | `derive_finger_kinematics.py`: a model's gear and pad tables from its URDF; `derive_link_inertia.py`: the RG6's `inertial:` block from the collision meshes; `rg6_details_mesh.py`: the RG6's blue ring, pivot caps and arm covers (`*_ring.dae`, `*_details.dae`) from the owner's detailed model |

The Robotiq 2F-85's description is `ros-jazzy-robotiq-description`, which `clearpath_manipulators_description`
depends on; nothing of it is vendored here.

## Installation

```bash
# from this repository's root
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Usage

### Container / simulation (no hardware)

`gripper_sim` offers the canonical interface of the model it is given and publishes the driver joint itself, so the
whole MoveIt integration is testable without a robot. The followers hang off the driver via `<mimic>` and are
derived by `robot_state_publisher` and `move_group`. The offboard mock (`deploy/offboard/scripts/mock.py`) starts it
for the model whose driver joint the arm's description carries, with that model's parameters:

```bash
ros2 run gripper_control gripper_sim --ros-args -r __ns:=/a200_0553/manipulators --params-file model.yaml
#   driver_joint, action_name, state_topic (both relative), linkage_q_rad, linkage_width_m
# emulate an object: -p sim_object_width_m:=0.05
#   closing stops at that width ─▶ grip_detected=true (as the real bridge reports it)
```

With a physics simulator behind the jaws it runs in **plant mode**: the same surface, but the ramp becomes the
command of a position controller of the driver joint, and the width is what the plant measures. A motion settles
when the jaws stand still after the command arrived; stopping short of a closing target is `grip_detected`. MARWIN
5's MuJoCo plant runs it so (deploy/marwin).

```bash
ros2 run gripper_control gripper_sim --ros-args --params-file model.yaml \
  -p plant_command_topic:=/rg6_finger_position_controller/commands   # JointGroupPositionController
# the width from -p plant_state_topic:=/joint_states (default)
```

Motion model: the width travels to the target at constant speed (the model's full stroke in ~1 s by default). What
this does *not* reproduce are a real hand's pathologies — a success from this node is marked as non-hardware truth
through the `source` field in `/motion/result`.

### Real robot

The RG6's node lives here, the service that starts it does not. `rg6_grip_bridge` runs as
`clearpath-custom-rg6-grip-bridge` — unit, wrapper and the root-owned copy under `/usr/local/bin` come from
[`husky/setup`](../husky/setup/), which deploys the file out of this workspace. Command it through the action above;
nothing here is launched by hand.

A 2F-85 on the real robot is NOT brought up yet: its hardware interface (`robotiq_driver`, not part of the image)
behind a position controller, with `gripper_sim` in plant mode serving the canonical surface on top, is the
intended route — the same node the container runs.

### joint_states plumbing (`joint_states.launch.py`)

On a multi-controller-manager Clearpath, wheel, arm and gripper joint states
come from **three** separate sources and are not merged automatically:

* **`joint_state_aggregator`** builds one complete `/a200_0553/joint_states`
  (with velocity + effort) for recording and Foxglove — deliberately *not* in
  the live TF path, where an aggregator would be a single point of failure.
* **`joint_state_relay`** mirrors `manipulators/joint_states` and
  `manipulators/endeffectors/joint_states` back onto
  `platform/joint_states`, which Clearpath's `robot_state_publisher` and
  `move_group` subscribe to. It is a node of its own rather than
  `topic_tools relay` because it publishes with **explicitly RELIABLE** QoS —
  `move_group` subscribes RELIABLE, and a best-effort publisher would never
  reach it (state displayed correctly, planning failing).

## The RG6 on the a200-0553

### How it works

```
                          this repository                        UR / OnRobot
  MoveIt, plan_server ──gripper_cmd──▶  rg6_grip_bridge ──XML-RPC──▶ OnRobot URCap ─▶ RG6
                                        rg6/bridge_state  ◀──getters──┘
  RViz / Foxglove     ◀──joint_states── rg6_grip_bridge (5 Hz)

  container, no hardware:
  MoveIt, plan_server ──gripper_cmd──▶  gripper_sim      ──▶ same two topics
```

**`joint_states` never falls silent.** The bridge publishes `rg6_finger_joint`
in every pass of its 5 Hz poll, whether the gripper measures or not: the
measured angle while it does, the last measured one while it does not, and the
open stop of the linkage table (`q_min` = 0.038 rad at 151.1 mm — the widest the
*device* reaches, not the 0.0 the SRDF's `open` state carries) until there has
been a first measurement. A held value is honest because an unpowered gripper
does not move; that it is held is visible as a `status: -1` on `bridge_state`
and as one log line when the substitution begins and one when it ends.

The reason is not cosmetic. Eight of the ten RG6 links with collision geometry
are placed by `robot_state_publisher` out of `rg6_finger_joint` (directly and
through the five `<mimic>` joints). Without the joint they have no TF at all,
and `move_group`'s `shape_mask` then cannot mask them out of the depth cloud —
the whole hand drops out of the octomap self filter and its own fingers become
obstacle voxels in front of a camera that sits on the gripper. Measured on the
a200-0553 on 2026-09-01 with the arm unpowered: ten `Missing transform for
shape mesh` lines per cloud (handles 14…23) and an occupancy update every
2.44 s instead of the 5 Hz the feed delivers, because the updater waits
`shape_transform_cache_lookup_wait_time` = 0.3 s **per** missing link.

The URCap is an RTDE client itself and occupies `tool_digital_output_mask`, so
`ur_robot_driver` runs on an **input recipe without the
`tool_digital_output*` lines**. ROS therefore cannot set a tool DO at all —
which is why the gripper is commanded through the URCap and not through the
UR tool interface.

### MoveIt

MoveIt talks to the gripper through the standard **GripperCommand** pipeline.
Clearpath regenerates `robot.srdf` and `moveit.yaml` on **every boot** and
knows nothing about OnRobot grippers, so the two halves are supplied
differently:

**`moveit.yaml` comes from `robot.yaml`** — no patching. Under
`manipulators.moveit.ros_parameters.move_group` sit the
`moveit_simple_controller_manager` entry `manipulators/rg6_gripper_controller`
(type `GripperCommand`, action_ns `gripper_cmd`) and the
`robot_description_planning.joint_limits.rg6_finger_joint` block that TOTG
needs. Clearpath's `generate_param` deep-merges them into the generated file.

**`robot.srdf` has no such hook** and is patched:

```bash
rg6_moveit_patch --setup-path /etc/clearpath   # robot (clearpath-custom-setup)
rg6_moveit_patch --setup-path /clearpath       # offboard container (entrypoint)
```

The tool writes a marker-framed block into `robot.srdf` only — atomically, with
a `.bak`, idempotent — containing the planning group `gripper`
(joint `rg6_finger_joint`), the group states `open` (0.0) and `close`
(1.25478), the end effector `rg6` (`parent_link` `arm_0_wrist_3_link`,
`parent_group` `arm_0`) and exactly two `<disable_collisions>` pairs
(`moment_arm` ↔ `finger_tip`, per finger) that Clearpath's
`moveit_collision_updater` does not find on its own.

`parent_group="arm_0"` is required: `RobotModel::buildGroupsInfoEndEffectors`
demands `hasLinkModel(parent_link)` on the parent group, and `arm_0_tool0` is
not a member of the joint-based `arm_0` group.

Afterwards **`verify_moveit_yaml`** checks that the values from `robot.yaml`
actually arrived. If they did not — a stale `robot.yaml` under `/etc/clearpath`
or from `ROBOT_YAML_URL` — it lists every missing key, names the SSOT and exits
**1**, so the case shows up at boot instead of at the first gripper goal. No
caller aborts on it; installer and offboard entrypoint log the code.

Why there is no `robot.yaml` route for the SRDF: `clearpath_config` contains
the string `srdf` nowhere, there is no counterpart to `platform.extras.urdf`,
`robot.srdf.xacro` is generated purely from the arms/grippers/lifts loops, and
`Gripper.MODEL` is a closed enum (`franka_gripper`, `kinova_2f_lite`,
`robotiq_2f_140`, `robotiq_2f_85`) that rejects an RG6 entry with a
`ValueError`.

Using it:

```python
group = MoveGroupCommander("gripper")
group.set_named_target("close"); group.go()    # stops and succeeds on contact
```

RViz: MotionPlanning panel → Planning Group `gripper` → Goal State
`open`/`close` → Plan & Execute.

## Width ↔ joint

A gripper's linkage is not linear, so the mapping between its driver joint [rad] and the clear width between the pad
faces [m] is a **generated table**, not a formula, in the model file (`linkage.table`):

* `scripts/derive_finger_kinematics.py` writes it from a GENERATED URDF — after **every** change to a model. The table
  is generated, not maintained by hand — a stale table is a visible file with a date rather than a silent drift.
  `--format yaml --reference <link>` also writes where the pads stand along that frame's +z (the model's
  `pad_offset`, before the TCP's own depth is subtracted).
* The upper joint limit is the zero crossing of the width; beyond it the fingers pass through each other in the
  model and the width grows again.
* `include/gripper_control/linkage.hpp` is the C++ reader `gripper_sim` uses, handed the model file's table, so mock,
  profile and bridge interpolate the same numbers.
* The RG6's bridge carries a generated copy, `src/rg6_control/scripts/rg6_finger_kinematics.json` (joint limits
  `0.038 … 1.25478` rad, maximum interpolation error `4.7e-05` m, measured between the two `flex_finger` frames):
  it runs on the robot, where the contract is not installed.

The width the device reports is **larger than the jaws actually are**, by
+3…+5 mm depending on direction and with ~1.7 mm of hysteresis. **The
commanded value is the reference, not the reported one.**

Tool-AI2/AI3 are *not* a width source. The raw voltages remain on
`io_and_status_controller/tool_data` and answer a different question — whether
the tool connector has power at all. AI2 has been measured as mis-calibrated by
up to 17 mm and is not a second opinion.

### Using it on the Clearpath Husky (a200-0553)

Workspace in `system.ros2.workspaces`, `io_and_status_controller` via
`robot.yaml` `ros_parameters`. Into the robot's URDF the gripper comes through
`platform.extras.urdf`, which addresses a file in
husky's `husky_extras_description`: that one hangs
[tool_description](src/tool_description/README.md)'s RG6 macro, over this package's hand, on
`arm_0_tool0`. All three workspaces have to be listed; the generator expands
`$(find tool_description)` and `$(find rg6_description)`. The gripper publishes joint states on
`manipulators/endeffectors/joint_states`.

## Safety

* A `GripperCommand` goal moves real hardware — keep the workspace clear and a
  hand on the e-stop.
* The bridge rejects a second command while one is running. Do not paper over
  that with a retry loop; wait for `busy` to fall **and** for the width to
  settle.
* `busy` still reads `false` for roughly 0.4 s after a command is accepted. A
  loop that only waits for the end of the motion returns immediately in that
  gap — wait for both edges.

## Running Tests

A model's linkage table is interpolated three times — `linkage.hpp` here, the RG6 bridge's JSON on the robot, the
`robot_contract` profile — and the RG6's lies in two places. `tests/` pins down that they agree: it compiles
`linkage.hpp` and compares it against the profile for every model, holds the bridge's copy to the RG6 model file, and
covers the generator that produces them. Neither ROS nor a robot is needed.

Cloned on its own, without the workspace around it, the cross-repo comparisons
skip — this repo cannot know where its siblings would be. Inside a workspace
they *fail* when a sibling is missing, rather than skipping: that is the case CI
scripts, and a guard that stops guarding without a word is worse than none.

```bash
uv sync && uv run pytest         # from this repository's root
```

What `gripper_sim` *does* is a different question, and one a unit test
cannot answer; that is checked end-to-end against a running stack:

```bash
uv run pytest robot/motion-server/tests/test_offboard_gripper.py    # needs the container
```

## Related

- [husky/setup](../husky/setup/README.md) — `robot.yaml` (SSOT),
  and the installer + systemd unit that roll out and start `rg6_grip_bridge`
  from this workspace
- [robot-contract](../robot/contract/README.md) — the gripper models
  (`profiles/grippers/`) and the profiles that put one on a robot's flange
- [offboard](../deploy/offboard/README.md) — the container that
  builds and runs the mock

## Versioning

[Semantic Versioning](https://semver.org/); see [CHANGELOG.md](CHANGELOG.md).
## License

See `src/rg6_description/LICENSE`. `rg6_control`, `gripper_control`: BSD-3-Clause.

