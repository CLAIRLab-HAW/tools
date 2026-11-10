"""The generator behind all three copies of the linkage table.

``scripts/derive_finger_kinematics.py`` walks the four-bar finger chain out of a GENERATED URDF and samples the joint
angle -> clear width relation.  clearpath's ``test_gripper_linkage_parity.py`` pins down that its three outputs agree --
which they would also do if the generator computed the same wrong thing for all of them.  This is the other half: the
machinery itself.

The real gripper URDF only exists after a xacro run inside the container, so the fixtures here are synthetic chains
whose answer is known by hand.  What that checks is the machinery -- chain walk, pose composition, the joint limit,
the sampling -- not the rg6_v2 geometry, which is the URDF's business and not this script's.
"""

from __future__ import annotations

import json
import math
import subprocess
import sys

import derive_finger_kinematics as gen
import numpy as np
import pytest


# ---- _rpy_to_R: the rotation the whole chain rests on ---------------------------------------------------------
def test_a_zero_rotation_is_the_identity():
    assert np.allclose(gen._rpy_to_R(0.0, 0.0, 0.0), np.eye(3))


def test_a_quarter_turn_about_z_takes_x_onto_y():
    assert np.allclose(gen._rpy_to_R(0.0, 0.0, math.pi / 2) @ [1, 0, 0], [0, 1, 0], atol=1e-12)


def test_a_quarter_turn_about_y_takes_z_onto_x():
    assert np.allclose(gen._rpy_to_R(0.0, math.pi / 2, 0.0) @ [0, 0, 1], [1, 0, 0], atol=1e-12)


def test_a_quarter_turn_about_x_takes_y_onto_z():
    assert np.allclose(gen._rpy_to_R(math.pi / 2, 0.0, 0.0) @ [0, 1, 0], [0, 0, 1], atol=1e-12)


def test_the_rotation_stays_orthonormal():
    """Every pose in the chain is composed from these; a determinant drifting off 1 would scale the widths."""
    R = gen._rpy_to_R(0.3, -0.7, 1.1)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1.0)


# ---- a synthetic chain ----------------------------------------------------------------------------------------
def _urdf(tmp_path, lower="0.0", upper="1.0", name="rg6_finger_joint", fingers=("left", "right")):
    """Two fingers that really close on each other as the driving angle grows.

    Each finger is a revolute joint at the origin carrying its pad on a FIXED offset of 0.5 m -- the pad frame
    therefore travels on a circle, which is what makes the clear width a curve rather than a constant.  The right
    finger is mirrored (``rpy`` half a turn, axis reversed), so the two swing towards each other: the width comes out
    as 1.0 * |cos q|, curved enough that linear interpolation between sampling points is measurably inexact.

    That last part matters for more than realism: with both fingers rigid on a common base the distance would not
    change at all, every interpolation would be exact, and the accuracy test below would pass on nothing.
    """
    spec = {"left": ("0 0 0", "0 0 1"), "right": ("0 0 3.141592653589793", "0 0 -1")}
    joints = ""
    for i, side in enumerate(fingers):
        rpy, axis = spec[side]
        driving = name if i == 0 else f"{side}_joint"
        limit = f'<limit lower="{lower}" upper="{upper}" effort="1" velocity="1"/>' if i == 0 else ""
        # The second finger follows the first, as a real hand's do: by <mimic>, which the script reads.
        mimic = f'<mimic joint="{name}" multiplier="1"/>' if i else ""
        joints += f"""
      <joint name="{driving}" type="revolute">
        <parent link="world"/>
        <child link="{side}_prox"/>
        <origin xyz="0 0 0" rpy="{rpy}"/>
        <axis xyz="{axis}"/>
        {limit}{mimic}
      </joint>
      <joint name="{side}_tip" type="fixed">
        <parent link="{side}_prox"/>
        <child link="{side}_flex_finger"/>
        <origin xyz="0.5 0 0" rpy="0 0 0"/>
      </joint>"""
    path = tmp_path / "probe.urdf"
    path.write_text(f'<?xml version="1.0"?>\n<robot name="probe">{joints}\n</robot>\n')
    return str(path)


def test_the_joint_limit_is_read_from_both_ends(tmp_path):
    """``lower`` is not always 0 -- the RG6's mechanical open stop does not reach the geometric zero of the chain."""
    _K, limit, _root = gen._chain(_urdf(tmp_path, lower="0.038", upper="1.25"))
    assert limit == (0.038, 1.25)


def test_a_urdf_without_the_driving_joint_is_refused(tmp_path):
    """Silently sampling an empty range would hand out a one-line table nobody looks at twice."""
    with pytest.raises(SystemExit):
        gen._chain(_urdf(tmp_path, name="some_other_joint"))


def test_the_chain_walk_finds_the_finger_links(tmp_path):
    K, _limit, _root = gen._chain(_urdf(tmp_path))
    assert sorted(n for n in K if n.endswith("flex_finger")) == ["left_flex_finger", "right_flex_finger"]


def test_a_pose_at_zero_is_the_joint_origin(tmp_path):
    K, _limit, _root = gen._chain(_urdf(tmp_path))
    assert np.allclose(gen._pose(K, "left_flex_finger", 0.0)[:3, 3], [0.5, 0.0, 0.0], atol=1e-12)


def test_a_pose_follows_the_driving_joint(tmp_path):
    """A quarter turn of the base carries the finger from +x round to +y -- the chain really composes."""
    K, _limit, _root = gen._chain(_urdf(tmp_path))
    assert np.allclose(gen._pose(K, "left_flex_finger", math.pi / 2)[:3, 3], [0.0, 0.5, 0.0], atol=1e-12)


# ---- the output -----------------------------------------------------------------------------------
def _run(urdf, fmt, check=True):
    return subprocess.run(
        [sys.executable, gen.__file__, urdf, "--format", fmt], check=check, capture_output=True, text=True
    )


def _stdout(urdf, fmt):
    return _run(urdf, fmt).stdout


def test_the_json_table_spans_exactly_the_joint_limits(tmp_path):
    data = json.loads(_stdout(_urdf(tmp_path, lower="0.1", upper="0.9"), "json"))
    table = data["table_q_rad_width_m"]
    assert [table[0][0], table[-1][0]] == [0.1, 0.9]
    assert data["joint_limits_rad"] == [0.1, 0.9]


def test_the_json_table_names_the_driving_joint(tmp_path):
    assert json.loads(_stdout(_urdf(tmp_path), "json"))["joint"] == "rg6_finger_joint"


def test_the_upper_limit_is_sampled_even_when_the_step_overshoots_it(tmp_path):
    """``arange`` stops before the end; a table that omitted the closed position would clamp a whole step early."""
    table = json.loads(_stdout(_urdf(tmp_path, lower="0.0", upper="0.07"), "json"))["table_q_rad_width_m"]
    assert table[-1][0] == 0.07


def test_a_chain_without_two_gripping_faces_is_refused_by_name(tmp_path):
    """One finger cannot have a clear width; guessing one would put a plausible wrong number into all three copies.

    Asserting on the MESSAGE, not just on a non-zero exit: with the check removed the script still dies, on an
    IndexError one line later, and a test that only looked at the exit code would keep passing over it.
    """
    done = _run(_urdf(tmp_path, fingers=("left",)), "json", check=False)
    assert done.returncode != 0
    assert "expected two pad links" in done.stderr


def test_the_reported_interpolation_error_really_bounds_the_deviation(tmp_path):
    """The figure the script writes into both outputs is a promise about the table -- so it has to be measured.

    It ends up in the JSON (``max_interpolationsfehler_m``) and in the model file's comment, and everything downstream
    trusts it: the sampling step was chosen so this stays below the RG6's finger position resolution. Nothing
    checked that it is computed against the real curve rather than against a degenerate grid.
    """
    urdf = _urdf(tmp_path, lower="0.1", upper="0.9")
    data = json.loads(_stdout(urdf, "json"))
    table = data["table_q_rad_width_m"]
    reported = data["max_interpolationsfehler_m"]

    K, (qmin, qmax), _root = gen._chain(urdf)
    faces = sorted(n for n in K if n.endswith("flex_finger"))

    def width(q):
        a = gen._pose(K, faces[0], q)[:3, 3]
        b = gen._pose(K, faces[1], q)[:3, 3]
        return float(np.linalg.norm(a - b))

    fine = np.linspace(qmin, qmax, 997)  # not the script's own grid, so a degenerate one cannot hide here
    truth = np.array([width(q) for q in fine])
    approximated = np.interp(fine, [t[0] for t in table], [t[1] for t in table])
    assert float(np.abs(truth - approximated).max()) <= reported + 1e-6
    assert reported > 0.0, "a curved chain cannot be interpolated exactly -- a reported 0 means it was not measured"
