"""A gripper model's linkage table is interpolated three times, and the RG6's lies in two places.  All must agree.

The fingers of a parallel gripper are a four-bar chain with no closed form, so the joint angle -> clear width relation
is a generated TABLE (``tools/derive_finger_kinematics.py``) in the model's file.  It is read in places that cannot
import one another:

* ``robot/contract`` -- ``profiles/grippers/<model>.yaml``, ``linkage.table`` -- the workstation code, Python
* ``src/gripper_control/include/gripper_control/linkage.hpp`` -- the container mock, C++, handed the table as
  parameters by its launcher
* ``src/rg6_control/scripts/rg6_finger_kinematics.json`` -- the RG6's bridge on the robot, Python, which must not
  import the contract there and so carries a generated copy

The model file is a HAND copy of the generator's output, exactly like the arm poses were before
``test_ssot_parity.py`` -- and those had already drifted (``packed[5]`` lost a digit).  A drift here is worse than a
wrong number: the container mock would then compute a different width from the same command than the real robot,
and the entire point of the mock is that it does not.

Each place also carries its own interpolation code -- ``Linkage`` in C++, ``GripperLinkage`` in ``robot_contract``,
and ``FingerKinematics`` in the bridge.  Same table, three implementations, and only their agreement makes the table
one source rather than three.  So this compares the data AND the results, the C++ for every model.

Neither ROS nor a robot is needed; the C++ side is compiled here, and skips with a named cause where no compiler is.
"""

from __future__ import annotations

import json
import pathlib
import shutil
import subprocess

import pytest
import yaml
from table_sources import sibling

#: The RG6's model file; every other model lies beside it.
RG6_MODEL_RELPATH = "robot/contract/src/clair/robot/contract/profiles/grippers/rg6_v2.yaml"

#: The two copies of this repo, addressed locally -- only the model files live in a sibling.
_HERE = pathlib.Path(__file__).resolve().parent.parent
JSON_PATH = _HERE / "src/rg6_control/scripts/rg6_finger_kinematics.json"
BRIDGE_PATH = _HERE / "src/rg6_control/scripts/rg6_grip_bridge.py"
INCLUDE_PATH = _HERE / "src/gripper_control/include"

#: Angles to compare the three implementations at: both bounds, both sides of every kind of edge, and a spread of
#: interior points.  Values outside the range are deliberate -- the clamp is behavior, not an accident.
PROBE_ANGLES = [-1.0, 0.0, 0.038, 0.0381, 0.05, 0.3, 0.6, 0.8, 0.9, 1.0, 1.2, 1.25478, 1.3, 5.0]


def _models() -> list[str]:
    return sorted(p.stem for p in sibling(RG6_MODEL_RELPATH).parent.glob("*.yaml"))


def _model_table(model: str) -> list[list[float]]:
    raw = yaml.safe_load((sibling(RG6_MODEL_RELPATH).parent / f"{model}.yaml").read_text())
    return [[float(q), float(w)] for q, w in raw["linkage"]["table"]]


def _json_table():
    return [[float(q), float(w)] for q, w in json.loads(JSON_PATH.read_text())["table_q_rad_width_m"]]


# ---- the table itself ------------------------------------------------------------------------------------------
def test_the_json_carries_a_table_at_all():
    """Guards the tests below: an empty parse would make every comparison pass for the wrong reason."""
    assert len(_json_table()) > 2


def test_the_bridge_copy_matches_the_rg6_model():
    """The bridge's copy.  Compared with ``==``, not ``approx``: any difference means it was retyped, not generated."""
    assert _json_table() == _model_table("rg6_v2")


def test_the_declared_joint_limits_match_the_table_ends():
    table = _json_table()
    assert json.loads(JSON_PATH.read_text())["joint_limits_rad"] == [table[0][0], table[-1][0]]


@pytest.mark.parametrize("model", _models())
def test_the_width_falls_strictly_over_the_whole_table(model):
    """Both inversions search for the first sampling point at or below the target width; a rise would break them."""
    widths = [w for _q, w in _model_table(model)]
    assert all(b < a for a, b in zip(widths, widths[1:]))


@pytest.mark.parametrize("model", _models())
def test_the_angle_rises_strictly_over_the_whole_table(model):
    angles = [q for q, _w in _model_table(model)]
    assert all(b > a for a, b in zip(angles, angles[1:]))


# ---- the three implementations ---------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def cpp_probe(tmp_path_factory):
    """Compile ``linkage.hpp`` into a probe and read its results back, per model, for ``PROBE_ANGLES``.

    The header is plain C++17 with no ROS in it, so this needs a compiler and nothing else.  Without one the
    comparison is skipped by name rather than quietly dropped.
    """
    compiler = shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")
    if compiler is None:
        pytest.skip("no C++ compiler on PATH -- cannot check linkage.hpp against the Python copies")

    tmp = tmp_path_factory.mktemp("linkage_cpp")
    src = tmp / "probe.cpp"
    # stdin: n, then n (q, width) pairs; argv: the angles to probe.
    src.write_text(
        "#include <cstdio>\n"
        "#include <cstdlib>\n"
        '#include "gripper_control/linkage.hpp"\n'
        "int main(int argc, char** argv) {\n"
        "  int n = 0;\n"
        '  if (scanf("%d", &n) != 1) return 2;\n'
        "  std::vector<double> qs(n), ws(n);\n"
        '  for (int i = 0; i < n; ++i) if (scanf("%lf %lf", &qs[i], &ws[i]) != 2) return 2;\n'
        "  const gripper_control::Linkage linkage(qs, ws);\n"
        "  for (int i = 1; i < argc; ++i) {\n"
        "    const double w = linkage.width_from_angle(atof(argv[i]));\n"
        '    printf("%.12f %.12f\\n", w, linkage.angle_from_width(w));\n'
        "  }\n"
        "  return 0;\n"
        "}\n"
    )
    binary = tmp / "probe"
    subprocess.run(
        [compiler, "-std=c++17", "-O0", "-I", str(INCLUDE_PATH), str(src), "-o", str(binary)],
        check=True,
        capture_output=True,
    )

    def run(model: str) -> dict[float, tuple[float, ...]]:
        table = _model_table(model)
        stdin = f"{len(table)}\n" + "".join(f"{q!r} {w!r}\n" for q, w in table)
        out = subprocess.run(
            [str(binary), *(f"{q!r}" for q in PROBE_ANGLES)], input=stdin, check=True, capture_output=True, text=True
        ).stdout
        rows = [tuple(float(v) for v in line.split()) for line in out.splitlines()]
        assert len(rows) == len(PROBE_ANGLES)
        return dict(zip(PROBE_ANGLES, rows))

    return {model: run(model) for model in _models()}


def _linkage(model: str):
    from clair.robot.contract import RobotProfile

    return RobotProfile.load("a200_0553", gripper=model).gripper.linkage


@pytest.fixture(scope="module")
def profile_linkage():
    """The RG6's, the model the bridge carries a copy of."""
    return _linkage("rg6_v2")


@pytest.fixture(scope="module")
def bridge_kinematics():
    """``FingerKinematics`` from the gripper bridge, loaded straight from the script on the robot side."""
    import importlib.util
    import sys

    path = BRIDGE_PATH
    name = "rg6_grip_bridge_under_test"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    # Register BEFORE executing: the bridge defines dataclasses, and ``dataclasses._is_type`` resolves a field
    # annotation through ``sys.modules[cls.__module__]``.  For a module loaded by path that entry does not exist
    # yet, and the decorator dies with ``'NoneType' object has no attribute '__dict__'``.
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
        return module.FingerKinematics(str(JSON_PATH))
    finally:
        sys.modules.pop(name, None)


@pytest.mark.parametrize("model", _models())
@pytest.mark.parametrize("q", PROBE_ANGLES)
def test_the_cpp_and_the_profile_agree_on_the_width(cpp_probe, model, q):
    assert cpp_probe[model][q][0] == pytest.approx(_linkage(model).width_from_angle(q), abs=1e-12)


@pytest.mark.parametrize("q", PROBE_ANGLES)
def test_the_bridge_and_the_profile_agree_on_the_width(bridge_kinematics, profile_linkage, q):
    assert bridge_kinematics.width_from_angle(q) == pytest.approx(profile_linkage.width_from_angle(q), abs=1e-12)


@pytest.mark.parametrize("model", _models())
@pytest.mark.parametrize("q", PROBE_ANGLES)
def test_the_cpp_and_the_profile_agree_on_the_inversion(cpp_probe, model, q):
    linkage = _linkage(model)
    width = linkage.width_from_angle(q)
    assert cpp_probe[model][q][1] == pytest.approx(linkage.angle_from_width(width), abs=1e-12)


@pytest.mark.parametrize("q", PROBE_ANGLES)
def test_the_bridge_and_the_profile_agree_on_the_inversion(bridge_kinematics, profile_linkage, q):
    width = profile_linkage.width_from_angle(q)
    assert bridge_kinematics.angle_from_width(width) == pytest.approx(profile_linkage.angle_from_width(width), 1e-12)


def test_an_angle_beyond_the_stop_yields_the_stop_not_an_extrapolation(profile_linkage):
    """A command past the mechanical stop means 'as far as it goes', not a width the hardware cannot reach."""
    assert profile_linkage.width_from_angle(-5.0) == pytest.approx(profile_linkage.max_width_m)
    # ``approx`` and not ``==``: the clamp runs through the same interpolation with t = 1, so the upper end comes
    # back with a float remainder (0.00039900000000000005 against 0.000399).  A tenth of a picometre is not the
    # behavior under test -- extrapolating past the stop would be.
    assert profile_linkage.width_from_angle(5.0) == pytest.approx(profile_linkage.min_width_m)


def test_a_width_beyond_the_stroke_yields_the_stop_not_nan(profile_linkage):
    assert profile_linkage.angle_from_width(10.0) == pytest.approx(profile_linkage.open_rad)
    assert profile_linkage.angle_from_width(-1.0) == pytest.approx(profile_linkage.closed_rad)
