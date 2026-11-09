"""The driver half of the gripper: it has a self-test, and this is what makes it run in the workspace suite.

``rg6_grip_bridge`` talks XML-RPC to the OnRobot URCap, which exists only on the robot -- so nothing here reaches
the hardware.  What CAN be checked without it is what the node's own ``--selftest`` checks (it spawns a fake URCap
and drives the client against it: units, float coercion, clamping, the timeout, the status message, the linkage
table and the concurrency lock), plus the two properties this package's layout depends on.

The self-test is run as a SUBPROCESS rather than imported: it is the documented invocation, and it is the one the
installer runs before deploying the file.  Importing the module would test a different thing than what is shipped.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

BRIDGE = Path(__file__).resolve().parent.parent / "src/rg6_control/scripts/rg6_grip_bridge.py"
TABLE = BRIDGE.with_name("rg6_finger_kinematics.json")


def test_the_selftest_passes():
    out = subprocess.run([sys.executable, str(BRIDGE), "--selftest"], capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, out.stdout + out.stderr
    assert "selftest: OK" in out.stdout


def test_the_table_lies_next_to_the_bridge():
    """The default resolution is ``Path(__file__).with_name()``, and the deployment mirrors it.

    Both the CMake install rule and the robot's installer put the two files in ONE directory for exactly this
    reason.  A table that drifts away from the script does not fail loudly -- the node refuses to start, and the
    gripper is simply absent.
    """
    assert TABLE.is_file()
    assert 'with_name("rg6_finger_kinematics.json")' in BRIDGE.read_text(encoding="utf-8")


def test_the_bridge_imports_nothing_the_robot_does_not_have():
    """It must run on the robot, where this workspace's Python packages do not exist.

    ``robot_contract`` is the named case in the module docstring: it is not installable there, and a dependency
    that prevents the deployment is no safeguard.  ``rclpy`` is imported inside ``main`` on purpose, so the
    self-test runs without ROS -- that is what lets this file be a plain pytest.
    """
    imports = [
        line.split()[1].split(".")[0]
        for line in BRIDGE.read_text(encoding="utf-8").splitlines()
        if line.startswith(("import ", "from ")) and not line.startswith("from __future__")
    ]
    forbidden = {"robot_contract", "plan_bridge", "robot_sdk", "clearlog", "numpy", "yaml"}
    assert not forbidden.intersection(imports), f"module-level imports the robot has no packages for: {imports}"


@pytest.mark.parametrize(
    "name",
    ["Rg6Client", "Rg6Error", "await_settled", "FingerKinematics", "status_payload", "finger_angle_to_publish"],
)
def test_the_names_other_code_reaches_for_are_there(name: str):
    """``onboard/setup/tools/rg6_stroke_survey.py`` imports three of these by name, and the container mock
    reproduces ``status_payload``'s fields character for character (gripper_control's ``gripper_sim.cpp``)."""
    assert f"def {name}" in BRIDGE.read_text(encoding="utf-8") or f"class {name}" in BRIDGE.read_text(encoding="utf-8")
