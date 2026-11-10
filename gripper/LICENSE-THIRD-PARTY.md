# Third-party components

## onrobot_description (rg6_v2)

* Origin: <https://github.com/inria-paris-robotics-lab/onrobot_ros>, branch `ros2`
* Package: `onrobot_description` v0.5.0, maintainer Igor Kalevatykh (INRIA)
* Licence: MIT, full text in [LICENSE-UPSTREAM-MIT](LICENSE-UPSTREAM-MIT)
* Taken over on 2026-08-19: `meshes/rg6_v2/` (15 STL), `config/rg6_v2.yaml`,
  `urdf/onrobot_rg.urdf.xacro` (here `onrobot_rg_upstream.urdf.xacro`),
  `urdf/materials.urdf.xacro` (here `materials_upstream.urdf.xacro`)

### Changed against upstream

* Package paths bent to `rg6_description` (16 places).
* The `<ros2_control>` block and the `rg_gazebo` call removed, along with the
  includes for transmission and Gazebo. **Reason:** on this robot the gripper
  does not hang on the `controller_manager` but is commanded over XML-RPC to
  the OnRobot URCap (`rg6_grip_bridge`, ROBOTER-TODO R21). A second
  ros2_control system in the robot URDF would be a device nobody operates, and
  Gazebo does not run in this workspace at all.
* Collision geometry switched to the shipped `collision/` meshes. Upstream
  references the `visual/` meshes for that (`body.stl` alone has 10 486
  triangles) and leaves the smaller collision files unused beside them.
  **Exception `finger_tip`:** there `collision/` carries two halves
  (`finger_tip_1/2.stl`) instead of one file of the same name — this single
  link keeps the visual mesh (2154 triangles).
* Driver joint name parameterised (see the repo CHANGELOG).
* The link `finger_tip` carried the `<collision>` block **twice**; in the
  compiled MuJoCo model that produced two identical collision meshes. One
  removed.
* `config/rg6_v2.yaml`: `limit.upper` clamped from **1.3** to **1.25478**. At
  1.3 rad the fingers already pass through one another in the model — the
  clear width reaches zero at 1.25478 and grows again after that. Upstream
  does not notice, because nobody there derives the width from the kinematics.

The meshes depict hardware by OnRobot A/S; the copyright in the depicted
device stays there. What was taken over is the MIT-licensed ROS description,
not OnRobot's CAD.
