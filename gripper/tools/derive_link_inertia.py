#!/usr/bin/env python3
"""Derives the ``inertial:`` block of a gripper config from its COLLISION meshes.

Upstream ships one placeholder for every part -- ``mass 0.001`` with ``ixx = iyy = izz = 0.001``.  Both halves are
wrong, and in opposite directions: a 1 g part is three orders of magnitude too light for a hand the data sheet puts at
1,25 kg, while 0,001 kg*m^2 is roughly four orders too HEAVY for a part that fits in 3 cm.  Whatever loads the model
then behaves like a hand made of lead sheet hung on the wrist: MuJoCo and PhysX both read ``<inertial>`` directly, and
``ur_description``'s payload has to carry it.

The way out is not to type better numbers.  It is to compute them from the geometry that is already in the repo:

  1. Each part's volume, centroid and inertia tensor come from its own collision mesh, integrated exactly over the
     closed polyhedron (Eberly, *Polyhedral Mass Properties*) -- not from a bounding box and not from a hull.
  2. ONE density for the whole assembly, fixed so that the sum over all part INSTANCES hits the data sheet mass.
     Homogeneous is wrong in detail (the motor is denser than the flex finger), but it is wrong in a way anyone can
     see and redo, and it distributes the 1,25 kg by volume rather than by taste.
  3. The result is cross-checked against the SECOND data sheet figure, the centre of gravity -- see ``--report``.
     A distribution that hits the mass and misses the CoG by a lot is telling you the homogeneity assumption broke.

Both data sheet figures live in ``robot/contract``'s profile (``gripper.datasheet.mass_kg`` /
``cog_z_m``, OnRobot RG6 operating manual v6.6.2 §8.3.2).  They are handed in here rather than read, so that this
script keeps working in a checkout that has only this repo.

    python3 tools/derive_link_inertia.py --total-mass-kg 1.25 --report

Writes the YAML block on stdout; the report goes to stderr, so a redirect keeps them apart.
"""

from __future__ import annotations

import argparse
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

#: How often each part sits in the assembled gripper.  ``safety_shield`` is built only when the macro is called with
#: ``safety_shield:=True``; the a200-0553 is not, so it carries no mass share -- but it keeps its tensor, because the
#: entry has to be there the moment somebody switches the shield on.
INSTANCES = {
    "single_bracket": 1,
    "body": 1,
    "moment_arm": 2,
    "truss_arm": 2,
    "finger_tip": 2,
    "flex_finger": 2,
    "safety_shield": 0,
}

#: The finger tip is the one part modelled as a convex DECOMPOSITION -- two meshes, one link (see the comment at the
#: finger tip in ``onrobot_rg_upstream.urdf.xacro``).  Its mass properties are the sum of both pieces, which is what
#: the physics engine sees too.
MESHES = {
    "single_bracket": ["single_bracket.stl"],
    "body": ["body.stl"],
    "moment_arm": ["moment_arm.stl"],
    "truss_arm": ["truss_arm.stl"],
    "finger_tip": ["finger_tip_1.stl", "finger_tip_2.stl"],
    "flex_finger": ["flex_finger.stl"],
    "safety_shield": ["safety_shield.stl"],
}

#: Resolution each quantity is written at.  A milligram and a micrometre are already finer than anything here is
#: worth; the tensor needs twelve places because its smallest entry is around 7e-9 kg*m^2.
MASS_DECIMALS = 6
COM_DECIMALS = 6
INERTIA_DECIMALS = 12

#: The six tensor entries, in the order the URDF ``<inertia>`` attribute list uses.
ENTRIES = ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")

#: The two frame-only links of a finger.  They carry no geometry at all, so they get no share of the data sheet mass;
#: their placeholder stays what it is and is subtracted from the budget before the density is fixed.
FRAME_LINK_MASS_KG = 0.001
FRAME_LINK_COUNT = 2


def read_stl(path: Path) -> np.ndarray:
    """Read a binary or ASCII STL and return its triangles as an ``(n, 3, 3)`` array of metres.

    :param path: the ``.stl`` file.
    :returns: vertex triples, one triangle per row.
    :raises ValueError: if the file is neither a well-formed binary nor a readable ASCII STL.
    """
    raw = path.read_bytes()
    # The 80-byte header of a binary STL may say anything, "solid" included -- so the length is what decides, not the
    # first word.  84 = header + the uint32 triangle count, 50 = 12 floats and the attribute short per triangle.
    if len(raw) >= 84:
        (count,) = struct.unpack("<I", raw[80:84])
        if len(raw) == 84 + 50 * count:
            data = np.frombuffer(raw, dtype=np.uint8, count=50 * count, offset=84).reshape(count, 50)
            # Columns 12..48 are the three vertices; the leading normal and the trailing attribute are dropped.
            return data[:, 12:48].copy().view("<f4").reshape(count, 3, 3).astype(np.float64)

    text = raw.decode("ascii", errors="strict")
    verts = [
        [float(token) for token in line.split()[1:4]] for line in text.splitlines() if line.strip().startswith("vertex")
    ]
    if not verts or len(verts) % 3:
        raise ValueError(f"{path}: neither a binary STL of the announced length nor a readable ASCII STL")
    return np.asarray(verts, dtype=np.float64).reshape(-1, 3, 3)


def mass_properties(tris: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """Exact volume, centroid and inertia tensor of a closed triangle mesh, at unit density.

    Eberly, *Polyhedral Mass Properties (Revisited)*: the volume integrals of 1, x, y, z, x^2, y^2, z^2, xy, yz and zx
    are turned into a sum over the triangles by the divergence theorem.  Exact for the polyhedron, which is the point
    -- a bounding box overstates a truss arm by more than a factor of two, and it is the tensor that decides how the
    hand swings.

    :param tris: ``(n, 3, 3)`` vertex triples of a CLOSED, outward-oriented mesh.
    :returns: ``(volume, centroid, inertia)`` -- the tensor about the CENTROID, at density 1.
    """
    x0, x1, x2 = tris[:, 0, 0], tris[:, 1, 0], tris[:, 2, 0]
    y0, y1, y2 = tris[:, 0, 1], tris[:, 1, 1], tris[:, 2, 1]
    z0, z1, z2 = tris[:, 0, 2], tris[:, 1, 2], tris[:, 2, 2]

    a1, b1, c1 = x1 - x0, y1 - y0, z1 - z0
    a2, b2, c2 = x2 - x0, y2 - y0, z2 - z0
    d0, d1, d2 = b1 * c2 - b2 * c1, a2 * c1 - a1 * c2, a1 * b2 - a2 * b1

    def sub(w0, w1, w2):
        t0 = w0 + w1
        f1 = t0 + w2
        t1 = w0 * w0
        t2 = t1 + w1 * t0
        f2 = t2 + w2 * f1
        f3 = w0 * t1 + w1 * t2 + w2 * f2
        return f1, f2, f3, f2 + w0 * (f1 + w0), f2 + w1 * (f1 + w1), f2 + w2 * (f1 + w2)

    f1x, f2x, f3x, g0x, g1x, g2x = sub(x0, x1, x2)
    f1y, f2y, f3y, g0y, g1y, g2y = sub(y0, y1, y2)
    f1z, f2z, f3z, g0z, g1z, g2z = sub(z0, z1, z2)

    volume = float((d0 * f1x).sum()) / 6.0
    ix, iy, iz = (d0 * f2x).sum() / 24.0, (d1 * f2y).sum() / 24.0, (d2 * f2z).sum() / 24.0
    ixx_, iyy_, izz_ = (d0 * f3x).sum() / 60.0, (d1 * f3y).sum() / 60.0, (d2 * f3z).sum() / 60.0
    ixy_ = (d0 * (y0 * g0x + y1 * g1x + y2 * g2x)).sum() / 120.0
    iyz_ = (d1 * (z0 * g0y + z1 * g1y + z2 * g2y)).sum() / 120.0
    izx_ = (d2 * (x0 * g0z + x1 * g1z + x2 * g2z)).sum() / 120.0

    if volume <= 0.0:
        raise ValueError("mesh volume is not positive -- open mesh, or triangles wound inwards")

    centroid = np.array([ix, iy, iz]) / volume
    # About the ORIGIN first, then shifted with the parallel axis theorem.  Products of inertia in the URDF
    # convention: ixy = -integral(x*y dm), which is why they gain (rather than lose) the shift term.
    inertia = np.array(
        [
            [iyy_ + izz_, -ixy_, -izx_],
            [-ixy_, ixx_ + izz_, -iyz_],
            [-izx_, -iyz_, ixx_ + iyy_],
        ]
    )
    cx, cy, cz = centroid
    inertia -= volume * np.array(
        [
            [cy * cy + cz * cz, -cx * cy, -cz * cx],
            [-cx * cy, cz * cz + cx * cx, -cy * cz],
            [-cz * cx, -cy * cz, cx * cx + cy * cy],
        ]
    )
    return volume, centroid, inertia


def derive(mesh_dir: Path, total_mass_kg: float) -> tuple[dict[str, dict[str, float]], float]:
    """Mass properties for every part, at the one density that makes the assembly hit ``total_mass_kg``.

    :param mesh_dir: the ``collision/`` directory of the gripper model.
    :param total_mass_kg: the data sheet mass of the whole hand.
    :returns: ``(parts, density)`` -- ``parts`` keyed as the config's ``inertial:`` block is.
    """
    geometry = {}
    for part, files in MESHES.items():
        volume = 0.0
        moment = np.zeros(3)
        pieces = []
        for name in files:
            v, c, i = mass_properties(read_stl(mesh_dir / name))
            volume += v
            moment += v * c
            pieces.append((v, c, i))
        centroid = moment / volume
        # Combining the pieces of a decomposition: each tensor moves from its own centroid to the shared one.
        inertia = np.zeros((3, 3))
        for v, c, i in pieces:
            d = c - centroid
            inertia += i + v * ((d @ d) * np.eye(3) - np.outer(d, d))
        geometry[part] = (volume, centroid, inertia)

    # The frame-only links keep their placeholder, so their mass leaves the budget before the density is fixed.
    budget = total_mass_kg - FRAME_LINK_COUNT * FRAME_LINK_MASS_KG
    displaced = sum(INSTANCES[part] * geometry[part][0] for part in geometry)
    density = budget / displaced

    parts = {}
    for part, (volume, centroid, inertia) in geometry.items():
        tensor = density * inertia
        parts[part] = {
            "mass": density * volume,
            "cx": centroid[0],
            "cy": centroid[1],
            "cz": centroid[2],
            "ixx": tensor[0, 0],
            "iyy": tensor[1, 1],
            "izz": tensor[2, 2],
            "ixy": tensor[0, 1],
            "ixz": tensor[0, 2],
            "iyz": tensor[1, 2],
        }
    return parts, density


#: Link name suffix -> config key, for reading a GENERATED URDF back.  Almost every link carries its part's own name,
#: so a suffix match spares a table that would have to be kept in step with the xacro.  ``bracket`` is the exception
#: and the reason this is a dict: the link is ``<prefix>gripper_bracket`` whatever bracket is fitted, while the config
#: key is the bracket MODEL (``single_bracket``) -- the macro takes it as a parameter.  Longest suffix first, so that
#: ``moment_arm`` is not read as ``arm``.
LINK_SUFFIX = {
    "moment_arm": "moment_arm",
    "truss_arm": "truss_arm",
    "finger_tip": "finger_tip",
    "flex_finger": "flex_finger",
    "safety_shield": "safety_shield",
    "body": "body",
    "bracket": "single_bracket",
}

#: Where the hand hangs off the arm.  The data sheet measures both its figures from this point ("base point of the
#: tool, without adapter"), so the cross-check has to start here and nowhere else.
TOOL_LINK = "arm_0_tool0"


def _rpy_to_R(r: float, p: float, y: float) -> np.ndarray:
    """Roll-pitch-yaw to a rotation matrix, in the URDF's fixed-axis order."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _axis_R(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rotation about an arbitrary axis (Rodrigues) -- the finger joints do not all turn about +y."""
    k = axis / np.linalg.norm(axis)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def centre_of_gravity(urdf: Path, parts: dict[str, dict[str, float]], angle_rad: float) -> tuple[np.ndarray, float]:
    """Assembly centre of gravity in the TOOL frame, with the derived masses on the model's own geometry.

    The second data sheet figure, and the only independent one available without a scale: the mass distribution is
    fixed by volume alone, so a centre of gravity that lands near the published one says the homogeneity assumption
    did not distort the hand -- and one that lands far away says it did.

    :param urdf: a GENERATED URDF (``urdf/robot.urdf``), for the joint origins as the model really composes them.
    :param parts: the derived properties, keyed as :func:`derive` returns them.
    :param angle_rad: driver joint angle the hand is evaluated at; every follower mimics it with multiplier 1.
    :returns: ``(cog, mass)`` -- the centre of gravity in ``arm_0_tool0`` and the mass it was taken over.
    """
    root = ET.parse(urdf).getroot()
    joints = {}
    for j in root.findall("joint"):
        origin = j.find("origin")
        xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
        rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
        axis = j.find("axis")
        joints[j.find("child").get("link")] = (
            j.find("parent").get("link"),
            np.array(xyz),
            _rpy_to_R(*rpy),
            j.get("type"),
            np.array([float(v) for v in axis.get("xyz").split()]) if axis is not None else None,
        )

    def pose(link):
        """Position and orientation of ``link`` in the tool frame, walking up until the tool is reached."""
        p, R = np.zeros(3), np.eye(3)
        while link != TOOL_LINK:
            if link not in joints:
                raise ValueError(f"{link} does not hang off {TOOL_LINK} in {urdf}")
            parent, xyz, rot, kind, axis = joints[link]
            local = rot if kind == "fixed" else rot @ _axis_R(axis, angle_rad)
            p, R, link = xyz + local @ p, local @ R, parent
        return p, R

    moment, mass = np.zeros(3), 0.0
    for link in root.findall("link"):
        name = link.get("name")
        if not name.startswith("rg6_") or link.find("inertial") is None:
            continue
        part = next((key for suffix, key in LINK_SUFFIX.items() if name.endswith(suffix)), None)
        if part is None:  # the two frame-only links: placeholder mass, no geometry, no share
            continue
        p, R = pose(name)
        com = np.array([parts[part]["cx"], parts[part]["cy"], parts[part]["cz"]])
        moment += parts[part]["mass"] * (p + R @ com)
        mass += parts[part]["mass"]
    return moment / mass, mass


def box_assembly(xacro: Path, link_name: str, mass_kg: float) -> tuple[np.ndarray, np.ndarray]:
    """Centre of mass and inertia tensor of a link whose collision geometry is a set of BOXES.

    The second shape of the same job.  ``husky_top_assembly`` is not a mesh with a density but a hand-authored
    envelope of six boxes around a portal frame, and it reaches the model with collision geometry and no
    ``<inertial>`` at all -- whereupon ``clair.twin.urdf_mujoco._ensure_inertial`` silently substitutes 0,1 kg for a
    structure that stands over half a metre tall.  Distributing a mass over the boxes BY VOLUME is not the true
    distribution either, but it is one anybody can check against the numbers in the file.

    That link lives in the NEIGHBOURING package (``onboard/extras``, the a200-0553's URDF extras), which is why this
    mode takes the xacro as an argument instead of knowing a path: it reads boxes out of any link, and the one it
    was written for is not a gripper part.

    :param xacro: the xacro carrying the link (parsed as plain XML -- the boxes are literals, no substitution).
    :param link_name: which link to read.
    :param mass_kg: total mass to distribute over the boxes.
    :returns: ``(com, inertia)`` in the LINK frame, the tensor about the centre of mass.
    """
    root = ET.parse(xacro).getroot()
    link = next((el for el in root.iter("link") if el.get("name") == link_name), None)
    if link is None:
        raise ValueError(f"{xacro}: no link {link_name!r}")

    boxes = []
    for collision in link.findall("collision"):
        box = collision.find("geometry/box")
        if box is None:
            raise ValueError(f"{link_name}: a <collision> that is not a box -- this routine only handles boxes")
        size = np.array([float(v) for v in box.get("size").split()])
        origin = collision.find("origin")
        xyz = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
        rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        boxes.append((size, xyz, _rpy_to_R(*rpy)))

    volumes = np.array([float(np.prod(size)) for size, _, _ in boxes])
    masses = mass_kg * volumes / volumes.sum()
    com = sum(m * xyz for m, (_, xyz, _) in zip(masses, boxes)) / mass_kg

    inertia = np.zeros((3, 3))
    for m, (size, xyz, R) in zip(masses, boxes):
        a, b, c = size
        local = np.diag([m * (b * b + c * c), m * (a * a + c * c), m * (a * a + b * b)]) / 12.0
        d = xyz - com
        inertia += R @ local @ R.T + m * ((d @ d) * np.eye(3) - np.outer(d, d))
    return com, inertia


def _format(value: float, decimals: int) -> str:
    """Fixed point, never exponent notation, and never a signed zero.

    The three quantities need different resolutions, which is why the caller passes one: a milligram is already
    below what any of this is worth, while the smallest tensor entry here is around 7e-9 kg*m^2 and would round to
    nothing at the same setting.  ``-0`` is suppressed on purpose -- it is one mirrored half of a symmetric part
    reading as different from its twin, in a file people compare by eye.
    """
    rounded = round(value, decimals)
    if rounded == 0.0:
        return "0.0"
    return f"{rounded:.{decimals}f}".rstrip("0").rstrip(".")


def emit(parts: dict[str, dict[str, float]]) -> str:
    """Render the ``inertial:`` block in the layout the config already uses."""
    lines = ["inertial:"]
    for part in MESHES:
        p = parts[part]
        mass = _format(p["mass"], MASS_DECIMALS)
        com = ", ".join(f"c{axis}: {_format(p['c' + axis], COM_DECIMALS)}" for axis in "xyz")
        tensor = ", ".join(f"{key}: {_format(p[key], INERTIA_DECIMALS)}" for key in ENTRIES)
        lines.append(f"  {part}: {{mass: {mass}, {com}, {tensor}}}")
    return "\n".join(lines)


def report(parts: dict[str, dict[str, float]], density: float, total_mass_kg: float) -> str:
    """The lines that let somebody judge the result instead of trusting it."""
    out = [f"density (homogeneous, over all instances): {density:.1f} kg/m^3"]
    carried = 0.0
    for part in MESHES:
        share = INSTANCES[part] * parts[part]["mass"]
        carried += share
        out.append(
            f"  {part:<15s} x{INSTANCES[part]}  {parts[part]['mass'] * 1000:8.2f} g each   {share * 1000:8.2f} g total"
        )
    out.append(f"  {'frame links':<15s} x{FRAME_LINK_COUNT}  {FRAME_LINK_MASS_KG * 1000:8.2f} g each")
    out.append(f"sum: {carried + FRAME_LINK_COUNT * FRAME_LINK_MASS_KG:.6f} kg against {total_mass_kg:.6f} kg")
    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--meshes",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "src/rg6_description/meshes/rg6_v2/collision",
        help="collision mesh directory of the model (default: the rg6_v2 meshes of this repo)",
    )
    parser.add_argument(
        "--total-mass-kg",
        type=float,
        default=1.25,
        help="data sheet mass of the whole hand -- OnRobot RG6 manual v6.6.2 §8.3.2, and the value the profile "
        "a200_0553.yaml carries as gripper.datasheet.mass_kg (default: 1.25)",
    )
    parser.add_argument("--report", action="store_true", help="write the mass distribution to stderr as well")
    parser.add_argument(
        "--check-cog",
        type=Path,
        metavar="URDF",
        help="cross-check against the data sheet centre of gravity, using the joint origins of a GENERATED URDF "
        "(urdf/robot.urdf).  Off by default: the bundle is not part of this repo",
    )
    parser.add_argument(
        "--cog-z-m",
        type=float,
        default=0.090,
        help="data sheet centre of gravity above the tool base point -- manual v6.6.2 §8.3.2, profile "
        "gripper.datasheet.cog_z_m (default: 0.090)",
    )
    parser.add_argument(
        "--angle-rad",
        type=float,
        default=0.038,
        help="driver joint angle the centre of gravity is taken at; the open stop of the rg6_v2 (default: 0.038)",
    )
    parser.add_argument(
        "--box-link",
        nargs=3,
        metavar=("XACRO", "LINK", "MASS_KG"),
        help="instead of the gripper: print the <inertial> of a link whose collision geometry is a set of boxes, "
        "with MASS_KG distributed over them by volume",
    )
    args = parser.parse_args()

    if args.box_link:
        xacro, link_name, mass = args.box_link
        com, inertia = box_assembly(Path(xacro), link_name, float(mass))
        print(
            f"<inertial>\n"
            f'  <origin xyz="{" ".join(_format(v, COM_DECIMALS) for v in com)}" rpy="0 0 0"/>\n'
            f'  <mass value="{_format(float(mass), MASS_DECIMALS)}"/>\n'
            f'  <inertia ixx="{_format(inertia[0, 0], INERTIA_DECIMALS)}" '
            f'iyy="{_format(inertia[1, 1], INERTIA_DECIMALS)}" '
            f'izz="{_format(inertia[2, 2], INERTIA_DECIMALS)}"\n'
            f'           ixy="{_format(inertia[0, 1], INERTIA_DECIMALS)}" '
            f'ixz="{_format(inertia[0, 2], INERTIA_DECIMALS)}" '
            f'iyz="{_format(inertia[1, 2], INERTIA_DECIMALS)}"/>\n'
            f"</inertial>"
        )
        return 0

    parts, density = derive(args.meshes, args.total_mass_kg)
    if args.report:
        print(report(parts, density, args.total_mass_kg), file=sys.stderr)
    if args.check_cog:
        cog, mass = centre_of_gravity(args.check_cog, parts, args.angle_rad)
        print(
            f"centre of gravity at q = {args.angle_rad} rad, over {mass:.4f} kg of geometry:\n"
            f"  x {cog[0] * 1000:+8.2f} mm   y {cog[1] * 1000:+8.2f} mm   z {cog[2] * 1000:+8.2f} mm\n"
            f"  data sheet cZ {args.cog_z_m * 1000:.1f} mm -> {(cog[2] - args.cog_z_m) * 1000:+.2f} mm",
            file=sys.stderr,
        )
    print(emit(parts))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
