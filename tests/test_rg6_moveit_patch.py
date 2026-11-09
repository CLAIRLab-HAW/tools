"""The SRDF patch that hooks the RG6 into MoveIt -- and above all its named posture "open".

A ``group_state`` is validated against the joint limits of the model, so the one number the SRDF and the URDF have
to agree on is the open end of the driver joint.  They did not: the patch wrote the geometric zero of the four-bar
chain while ``rg6_v2.yaml`` clamps ``limit.lower`` to the mechanical open stop at 0.038 rad, and MoveIt refused
every plan through the posture with "Goal state is out of bounds!" (measured 2026-08-28 on the a200-0553).  The
tests here tie the patch's value to the config's, in both directions: the default IS the limit, and whatever comes
out lies INSIDE it.
"""

from __future__ import annotations

import importlib.machinery
import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

_TOOL = Path(__file__).resolve().parent.parent / "src/rg6_control/scripts/rg6_moveit_patch"
_spec = importlib.util.spec_from_loader(
    "rg6_moveit_patch", importlib.machinery.SourceFileLoader("rg6_moveit_patch", str(_TOOL))
)
patch = importlib.util.module_from_spec(_spec)
sys.modules["rg6_moveit_patch"] = patch
_spec.loader.exec_module(patch)

#: A generated ``robot.srdf`` as ``generate_semantic_description`` leaves it: flat, with the arm group already in
#: it and no trace of the gripper.  Everything the patch needs is the closing tag.
FLAT_SRDF = """<?xml version="1.0" encoding="UTF-8"?>
<robot name="a200_0553">
    <group name="arm_0">
        <joint name="arm_0_shoulder_pan_joint"/>
    </group>
</robot>
"""


@pytest.fixture(scope="session")
def limits(repo_root: Path) -> dict:
    return yaml.safe_load((repo_root / "src/rg6_description/config/rg6_v2.yaml").read_text())["limit"]


@pytest.fixture()
def args(monkeypatch):
    """The command line as both call sites use it -- no arguments beyond the setup path."""
    # The workspace venv is not a ROS environment, but a developer's shell can be: pin the search to the
    # checkout so the test reads THIS repo's config and not an installed copy of some other version.
    monkeypatch.delenv("AMENT_PREFIX_PATH", raising=False)
    parsed = patch.build_parser().parse_args([])
    parsed.angle_open = patch.resolve_open_angle(parsed.model_config)
    return parsed


def test_the_open_angle_is_the_joints_own_lower_limit(args, limits):
    """The default is the config's number, not a copy of it that once was."""
    assert float(args.angle_open) == pytest.approx(limits["lower"])


def test_the_fallback_carries_the_same_number(limits):
    """The one literal left is only reachable without the config, and it may not drift away from it.

    It exists because the alternative -- writing the geometric zero when ``rg6_description`` is not installed --
    puts back exactly the posture MoveIt refuses.  A fallback that reintroduces the fault is not one.
    """
    assert float(patch.OPEN_ANGLE_FALLBACK) == pytest.approx(limits["lower"])


def test_a_missing_config_falls_back_instead_of_writing_the_geometric_zero(capsys):
    assert patch.resolve_open_angle("/nonexistent/rg6_v2.yaml") == patch.OPEN_ANGLE_FALLBACK
    assert "falls back" in capsys.readouterr().err


def test_the_config_is_found_from_this_checkout(monkeypatch, repo_root):
    """Without a sourced overlay the tool has only its own path to go on, and that is the offboard case.

    ``entrypoint.sh`` sources the rg6 overlay in a subshell around the generator run and calls the tool outside
    it, so AMENT_PREFIX_PATH is not the thing to rely on.
    """
    monkeypatch.delenv("AMENT_PREFIX_PATH", raising=False)
    found = patch.find_model_config()
    assert found is not None
    assert Path(found) == repo_root / "src/rg6_description/config/rg6_v2.yaml"


def test_the_patched_srdf_states_a_posture_inside_the_joint(tmp_path, args, limits):
    """The regression itself, end to end: what lands in the file is a posture the joint can reach."""
    srdf = tmp_path / "robot.srdf"
    srdf.write_text(FLAT_SRDF)
    assert patch.patch_srdf(str(srdf), args) is True

    root = ET.fromstring(srdf.read_text())
    states = {s.get("name"): s for s in root.findall("group_state")}
    assert set(states) == {"open", "close"}
    for name, state in states.items():
        joints = state.findall("joint")
        assert [j.get("name") for j in joints] == [f"{args.prefix}finger_joint"]
        value = float(joints[0].get("value"))
        assert limits["lower"] <= value <= limits["upper"], f"group_state '{name}' is outside the joint limits"


def test_a_block_under_another_wording_is_replaced_not_doubled(tmp_path, args):
    """The a200's SRDF on 2026-09-29: the block under the German marker, which the patch appended to a second time."""
    srdf = tmp_path / "robot.srdf"
    srdf.write_text(
        FLAT_SRDF.replace(
            "</robot>",
            "    <!-- onrobot-rg6:BEGIN (rg6_moveit_patch; nicht von Hand editieren) -->\n"
            '    <group name="gripper">\n        <joint name="rg6_finger_joint"/>\n    </group>\n'
            "    <!-- onrobot-rg6:END -->\n</robot>",
        )
    )
    assert patch.patch_srdf(str(srdf), args) is True

    text = srdf.read_text()
    assert [g.get("name") for g in ET.fromstring(text).findall("group")] == ["arm_0", "gripper"]
    assert "nicht von Hand" not in text and patch.MARKER_BEGIN in text
    assert patch.patch_srdf(str(srdf), args) is False, "a second run changes nothing"


def test_an_explicit_angle_still_wins(tmp_path):
    """The argument is the escape hatch for a hand whose stop sits elsewhere; the default may not swallow it."""
    parsed = patch.build_parser().parse_args(["--angle-open", "0.5"])
    assert parsed.angle_open == "0.5"
