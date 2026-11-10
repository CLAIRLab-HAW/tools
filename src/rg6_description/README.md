# rg6_description
URDF file of onrobot rg6 gripper based on the repository of [syuntoku14 /
fusion2urdf](https://github.com/syuntoku14/fusion2urdf.git) and the STEP file provided by [OnRobot](https://onrobot.com/en/downloads)

Note:

This fork's URDF is xacro-based and does not contain the `g_main` joint or the
`rg6_description.xacro` file from the original fusion2urdf output. The gripper
itself — links, finger joints and their `<mimic>` multipliers — comes from the
upstream rg6_v2 model in `urdf/onrobot_rg_upstream.urdf.xacro` (see
[LICENSE-THIRD-PARTY](../../LICENSE-THIRD-PARTY.md)), the measured limits and
mass properties from `config/rg6_v2.yaml`.

This package describes the HAND and nothing else. Where the hand is bolted onto
one particular robot — the mounting at `arm_0_tool0`, the alias link
`rg6_onrobot_rg6_base_link` that `robot.yaml` hangs the camera on, and the
`rg6_hand_tcp` frame — lives in
[husky/extras](../../../../husky/extras/README.md), over the RG6 macro in
[tool_description](../tool_description/README.md). Nothing here names a frame of the a200.
