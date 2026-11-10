# tools/description

`tool_description`, the tools both robots carry, as xacro macros hung on a given `parent`.

## Features

- **`urdf/rg6.macro.xacro`**, `rg6_tool(parent)`: the OnRobot RG6 from `rg6_description`
  ([gripper](../gripper/README.md)) plus `rg6_hand_tcp`, the TCP the whole workspace addresses, 210 mm above the flange at the start of the pads.
- **`urdf/key_holder.macro.xacro`**, `key_holder(parent)`: a printed holder (ISO 9409-1-50, 63 mm) with a Rittal
  double-bit key, its TCP `key_tip`; drawn from `meshes/key_holder/`, collided as primitives.
- **No robot in here.** The a200-0553 hangs the tools on `arm_0_tool0` in husky's
  `husky_extras_description/urdf/tools/`, MARWIN 5 on the SmartShift holder in marwin's `marwin_extras_description`.

## Tech Stack

ROS 2 Jazzy · xacro · ament_cmake (data only)

## Installation

```bash
cd ~/tools/description && colcon build --packages-select tool_description   # after ~/tools/gripper
```

The offboard images build it into `/opt/tools/description`.

## Usage

```xml
<xacro:include filename="$(find tool_description)/urdf/rg6.macro.xacro"/>
<xacro:rg6_tool parent="arm_0_tool0"/>
```

## Running Tests

Data only; husky's and marwin's extras tests hold their includes to these files.

## Related

- [gripper](../gripper/README.md) — the hand the RG6 macro instantiates

## Versioning

See the [CHANGELOG](CHANGELOG.md).

## License

MIT. The key holder's meshes come from mujoco-mia (scenes/endeffector).
