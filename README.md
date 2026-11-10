# tools — what both robots carry on their flange

The hands and tools the a200-0553 and MARWIN 5 share. A tool is described here once, hung on whatever face a robot
gives it; where it sits is the robot's: [husky](https://github.com/CLAIRLab-HAW/husky)'s extras put it on the a200's
`arm_0_tool0`, [marwin](https://github.com/CLAIRLab-HAW/marwin)'s on the SmartShift tool holder.

| Directory | What it is |
|---|---|
| [`gripper/`](gripper/README.md) | The hand on the flange, any gripper model: the measured `rg6_v2` description (`rg6_description`), the RG6's bridge to the URCap and its MoveIt patch (`rg6_control`), the container mock and the `joint_states` plumbing every model shares (`gripper_control`), and the gear-table tools. |
| [`description/`](description/README.md) | `tool_description`: the RG6 and the Rittal key holder as xacro macros on a given `parent`, with the key holder's meshes. |

**On the a200-0553 this repository is the clone `/home/robot/tools`**, beside `/home/robot/husky`, whose installer
clones it and builds `gripper/` and `description/` in place; `robot.yaml` lists both workspaces.

## Running tests

```bash
uv sync
uv run ruff check --select E9,F63,F7,F82 gripper && uv run ruff format --check .
uv run pytest                             # without ROS and without a container
```

Tests that compare a file here against one in a neighboring clearpath repository skip with a named cause when there
is none. `robot/contract/tests/test_gripper_linkage_parity.py` in clearpath holds the C++ mock and the bridge's table
to the profile's gripper models.

## Versioning

Each package keeps its own `CHANGELOG.md` and its own version; this file records what happens to the repository as a
whole.
