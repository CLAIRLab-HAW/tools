"""The generator behind the ``inertial:`` block of the gripper config.

The mass properties in ``rg6_v2.yaml`` are computed, not typed, and this is the machinery that computes them.  Two
things are worth pinning down, and they are different in kind:

* the INTEGRATOR, against shapes whose answer is known in closed form.  A box and a sphere say whether the volume
  integrals, the parallel axis shift and the products of inertia are right; the real meshes cannot, because nobody
  knows their tensor by hand.
* the RESULT that is checked in, against the two figures from the data sheet it was anchored to.  That is the half
  which catches somebody regenerating with a different mass, or the meshes changing under the config.

The centre of gravity cross-check needs the generated bundle (``urdf/robot.urdf``), which is not part of this repo --
that test skips when it is not there, the workspace convention for a repo that has to work on its own.
"""

from __future__ import annotations

import math
from pathlib import Path

import derive_link_inertia as gen
import numpy as np
import pytest
import yaml

#: Data sheet figures the checked-in block is anchored to (OnRobot RG6 manual v6.6.2 §8.3.2).  Held here as well as
#: in the profile because this repo cannot import ``robot_contract`` -- ``test_ssot_parity`` is where the two are
#: tied together.
DATASHEET_MASS_KG = 1.25
DATASHEET_COG_Z_M = 0.090

#: Resolution the config is written at, and therefore the tolerance every sum over it can hold.  Six decimals on
#: seven parts cannot land closer than a few micrograms.
MASS_TOL_KG = 1e-5


def _box_mesh(a: float, b: float, c: float, offset=(0.0, 0.0, 0.0)) -> np.ndarray:
    """The twelve triangles of an axis-aligned box, wound outwards."""
    ox, oy, oz = offset
    x, y, z = a / 2, b / 2, c / 2
    v = np.array(
        [
            [-x, -y, -z],
            [x, -y, -z],
            [x, y, -z],
            [-x, y, -z],
            [-x, -y, z],
            [x, -y, z],
            [x, y, z],
            [-x, y, z],
        ]
    ) + np.array([ox, oy, oz])
    faces = [
        (0, 2, 1),
        (0, 3, 2),  # bottom
        (4, 5, 6),
        (4, 6, 7),  # top
        (0, 1, 5),
        (0, 5, 4),  # -y
        (2, 3, 7),
        (2, 7, 6),  # +y
        (1, 2, 6),
        (1, 6, 5),  # +x
        (0, 4, 7),
        (0, 7, 3),  # -x
    ]
    return np.array([[v[i] for i in face] for face in faces])


def _sphere_mesh(radius: float, bands: int = 64) -> np.ndarray:
    """A UV sphere, fine enough that its volume is within a per mille of the analytic one."""
    tris = []
    for i in range(bands):
        t0, t1 = math.pi * i / bands, math.pi * (i + 1) / bands
        for j in range(2 * bands):
            p0, p1 = math.pi * j / bands, math.pi * (j + 1) / bands

            def point(t, p):
                return np.array(
                    [radius * math.sin(t) * math.cos(p), radius * math.sin(t) * math.sin(p), radius * math.cos(t)]
                )

            a, b, c, d = point(t0, p0), point(t0, p1), point(t1, p1), point(t1, p0)
            tris += [[a, c, b], [a, d, c]]
    return np.array(tris)


# ---- the integrator, against shapes with a closed form -------------------------------------------------------


def test_a_box_has_the_volume_its_edges_say():
    volume, _, _ = gen.mass_properties(_box_mesh(0.2, 0.3, 0.5))
    assert volume == pytest.approx(0.2 * 0.3 * 0.5, rel=1e-12)


def test_a_box_has_the_tensor_the_textbook_gives_it():
    a, b, c = 0.2, 0.3, 0.5
    _, _, inertia = gen.mass_properties(_box_mesh(a, b, c))
    volume = a * b * c
    assert inertia[0, 0] == pytest.approx(volume * (b * b + c * c) / 12.0, rel=1e-10)
    assert inertia[1, 1] == pytest.approx(volume * (a * a + c * c) / 12.0, rel=1e-10)
    assert inertia[2, 2] == pytest.approx(volume * (a * a + b * b) / 12.0, rel=1e-10)


def test_a_symmetric_shape_has_no_products_of_inertia():
    """The off-diagonal is where a sign error hides: it is zero for anything symmetric about its own axes."""
    _, _, inertia = gen.mass_properties(_box_mesh(0.2, 0.3, 0.5))
    assert inertia[0, 1] == pytest.approx(0.0, abs=1e-15)
    assert inertia[0, 2] == pytest.approx(0.0, abs=1e-15)
    assert inertia[1, 2] == pytest.approx(0.0, abs=1e-15)


def test_the_tensor_is_taken_about_the_centroid_not_the_origin():
    """A box moved off the origin keeps its tensor and moves only its centroid.

    This is the whole parallel axis shift in one assertion.  Without it the tensor would grow with the distance from
    the link frame -- which is exactly how a part 52 mm off its own origin gets a stiffness it does not have.
    """
    here = gen.mass_properties(_box_mesh(0.2, 0.3, 0.5))
    there = gen.mass_properties(_box_mesh(0.2, 0.3, 0.5, offset=(1.0, -2.0, 0.5)))
    assert there[1] == pytest.approx(np.array([1.0, -2.0, 0.5]), abs=1e-12)
    assert there[2] == pytest.approx(here[2], rel=1e-9, abs=1e-15)


def test_a_sphere_matches_its_closed_form():
    """A second shape, and a curved one: a box alone would pass even if the triangle winding were mishandled."""
    radius = 0.13
    volume, centroid, inertia = gen.mass_properties(_sphere_mesh(radius))
    assert volume == pytest.approx(4.0 / 3.0 * math.pi * radius**3, rel=2e-3)
    assert centroid == pytest.approx(np.zeros(3), abs=1e-12)
    for axis in range(3):
        assert inertia[axis, axis] == pytest.approx(0.4 * volume * radius**2, rel=3e-3)


def test_an_inward_wound_mesh_is_refused_rather_than_negated():
    """Reversed winding gives a negative volume, and a negative mass would propagate in silence."""
    with pytest.raises(ValueError, match="wound inwards"):
        gen.mass_properties(_box_mesh(0.2, 0.3, 0.5)[:, ::-1, :])


# ---- box assemblies ------------------------------------------------------------------------------------------


def test_two_equal_boxes_put_their_centre_of_mass_between_them(tmp_path):
    xacro = tmp_path / "two.urdf.xacro"
    xacro.write_text(
        '<robot name="t"><link name="pair">'
        '<collision><origin xyz="-0.5 0 0" rpy="0 0 0"/><geometry><box size="0.1 0.1 0.1"/></geometry></collision>'
        '<collision><origin xyz="0.5 0 0" rpy="0 0 0"/><geometry><box size="0.1 0.1 0.1"/></geometry></collision>'
        "</link></robot>"
    )
    com, inertia = gen.box_assembly(xacro, "pair", 2.0)
    assert com == pytest.approx(np.zeros(3), abs=1e-12)
    # Two 1 kg point-ish boxes at +/-0.5 m: the dominant term is the separation, 2 * 1 * 0.25.
    assert inertia[1, 1] == pytest.approx(2 * (1.0 * 0.25 + 1.0 * (0.01 + 0.01) / 12.0), rel=1e-9)
    assert inertia[0, 0] == pytest.approx(2 * 1.0 * (0.01 + 0.01) / 12.0, rel=1e-9)


def test_a_link_without_boxes_is_refused(tmp_path):
    xacro = tmp_path / "mesh.urdf.xacro"
    xacro.write_text(
        '<robot name="t"><link name="m"><collision><geometry>'
        '<mesh filename="package://x/y.stl"/></geometry></collision></link></robot>'
    )
    with pytest.raises(ValueError, match="only handles boxes"):
        gen.box_assembly(xacro, "m", 1.0)


# ---- the checked-in result -----------------------------------------------------------------------------------


@pytest.fixture(scope="module")
def config(repo_root: Path) -> dict:
    return yaml.safe_load((repo_root / "src/rg6_description/config/rg6_v2.yaml").read_text())


def test_every_part_carries_a_centre_of_mass(config):
    """The tensor is about the part's centre of mass, so the config has to say where that is.

    Without ``cx/cy/cz`` the xacro macro cannot write the ``<origin>``, and URDF then reads the tensor as being about
    the link frame -- a silent error, because the file still parses.
    """
    for part, values in config["inertial"].items():
        assert {"cx", "cy", "cz"} <= set(values), f"{part} has no centre of mass"


def test_the_masses_add_up_to_the_datasheet(config, repo_root):
    """The anchor the whole distribution hangs on.

    Regenerating with a different total, or a changed mesh, moves this sum -- and the sum is the one number the
    manufacturer states, so it is the one worth guarding.
    """
    inertial = config["inertial"]
    total = sum(gen.INSTANCES[part] * inertial[part]["mass"] for part in gen.INSTANCES)
    total += gen.FRAME_LINK_COUNT * gen.FRAME_LINK_MASS_KG
    assert total == pytest.approx(DATASHEET_MASS_KG, abs=MASS_TOL_KG)


def test_no_part_kept_the_upstream_placeholder(config):
    """Upstream's ``mass: 0.001`` with ``ixx = iyy = izz = 0.001`` for every part, the state this replaced."""
    for part, values in config["inertial"].items():
        assert values["mass"] != 0.001, f"{part} is back on the upstream placeholder mass"
        assert not (values["ixx"] == values["iyy"] == values["izz"] == 0.001), f"{part} has the placeholder tensor"


def test_the_tensors_obey_the_triangle_inequality(config):
    """A physically realisable body has each principal moment below the sum of the other two.

    Cheap, and it is the check that fails when a tensor is written about the wrong point or with a mixed-up axis --
    both of which produce numbers that look plausible one at a time.
    """
    for part, v in config["inertial"].items():
        moments = sorted((v["ixx"], v["iyy"], v["izz"]))
        assert moments[2] <= moments[0] + moments[1] + 1e-12, f"{part} has an impossible inertia tensor"


def test_the_centre_of_gravity_lands_where_the_datasheet_puts_it(repo_root, config):
    """The independent check: the mass distribution was fixed by VOLUME, so the centre of gravity is a free result.

    It is free in the sense that nothing in the derivation aims at it -- which is why it is worth something when it
    lands within a millimetre of the published figure.  It also settles the reading of the data sheet that the mass
    alone leaves open: the bracket is inside the 1.25 kg.
    """
    bundle = repo_root.parent.parent / "urdf" / "robot.urdf"
    if not bundle.is_file():
        pytest.skip(f"no generated bundle at {bundle} -- run urdf/generate.sh")
    parts = {part: dict(values) for part, values in config["inertial"].items()}
    cog, mass = gen.centre_of_gravity(bundle, parts, angle_rad=0.038)
    assert mass == pytest.approx(DATASHEET_MASS_KG - gen.FRAME_LINK_COUNT * gen.FRAME_LINK_MASS_KG, abs=MASS_TOL_KG)
    assert cog[2] == pytest.approx(DATASHEET_COG_Z_M, abs=0.002)
