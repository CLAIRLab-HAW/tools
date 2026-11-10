"""The details the macro hangs on as extra visuals: the blue ring, the pivot caps and the covers in the pads' gray.

``rg6_details_mesh.py`` (tools ``5642307``, no longer in the repo) wrote them from a glTF that is not in the repo, so
these tests cannot regenerate them. They hold what breaks silently instead: a ``_detail`` the macro names without its
DAE (RViz draws the hand without it and logs one line), and a DAE that lost its color (the visual carries no
``<material>``, so it would turn white).
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

#: The macro's ``bracket`` default; the ring is written in that bracket's frame.
BRACKET = "single_bracket"
#: The DAEs the macro names, in its order.
DETAILS = ["single_bracket_ring", "moment_arm_details", "truss_arm_details"]
#: The diffuse sRGB each look is written with (``LOOK`` in the tool).
COLORS = {"ring": (73 / 255, 157 / 255, 218 / 255), "pad": (0.25, 0.25, 0.25)}


def _details(repo_root: Path) -> list[str]:
    macro = (repo_root / "src/rg6_description/urdf/onrobot_rg_upstream.urdf.xacro").read_text()
    return [name.replace("${bracket}", BRACKET) for name in re.findall(r'<xacro:_detail mesh="([^"]+)"/>', macro)]


def test_the_macro_hangs_a_detail_on_the_bracket_and_both_arms(repo_root: Path) -> None:
    assert _details(repo_root) == DETAILS


@pytest.mark.parametrize("name", DETAILS)
def test_each_detail_dae_exists_and_carries_its_color(repo_root: Path, name: str) -> None:
    dae = repo_root / f"src/rg6_description/meshes/rg6_v2/visual/{name}.dae"
    root = ET.parse(dae).getroot()
    ns = {"c": root.tag.split("}")[0].strip("{")}
    assert root.find("c:asset/c:up_axis", ns).text == "Z_UP"
    diffuse = [tuple(float(v) for v in e.text.split()[:3]) for e in root.iterfind(".//c:diffuse/c:color", ns)]
    look = "ring" if name.endswith("_ring") else "pad"
    assert diffuse == [pytest.approx(COLORS[look], abs=1e-6)]
