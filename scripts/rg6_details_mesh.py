#!/usr/bin/env python3
# /// script
# requires-python = ">=3.12"
# dependencies = ["fast-simplification", "numpy", "pycollada", "pyyaml", "rtree", "scikit-image", "scipy", "trimesh"]
# ///
"""Writes the RG6's blue ring, its pivot caps and its arm covers as DAEs, one per link frame.

Our RG6 model (``rg6_description``, meshes from the upstream ``onrobot_rg_description``) draws every part in one color
and lacks three details the real hand has (the owner, 2026-10-10): the narrow blue ring at the hand's mounting end, the
covers on the finger arms (a rounded plate on either face of each finger's arms and a rounded inlay on the truss
arm's inner face), and the sleeves over the pivot pins where the moment and the truss arm meet the finger tip.
The pads on the finger tips are none of these and are left out. This script takes them from the owner's detailed
model of the RG6 and writes them in the frames of OUR links, so the macro hangs them on as extra visuals and keeps
every existing mesh, link and joint.

The source is an Onshape glTF export (generator "ONSHAPE BY PTC INC, 1.222", 174 bodies, 7234 primitives, 20.2 M
triangles, meters), the owner's detailed model of the RG6. It is not committed (OnRobot's CAD, 797 MB, no license
to redistribute); the owner keeps it untracked at ``tools/gripper_rg6.gltf`` in the workspace. Each body is one node
without a transform, each CAD face one primitive with its own material. The parts are found by their material
(:data:`RING_SRGB`, :data:`CAP_SRGB`, :data:`COVER_SRGB` -- the covers' glTF colors are grays), and a count other
than the expected one stops the script.

How the parts reach our frames, measured 2026-10-10. Every fit is an ICP of one of OUR link meshes onto the glTF
bodies it lies on, never an alignment by the mounting face: the vendor's model ends at its flange, ours carries more
below it.

1. **body**: our ``body.stl`` is laid onto the glTF by its bounding box (the hand's axis is our z and the glTF's x,
   the fingers open along our x and the glTF's y), slid along the axis to the best fit, and refined by ICP. The
   half-turn about the axis is ambiguous for a symmetric housing; both are fitted and the lower residual wins
   (0.044 mm RMS against 0.404 mm). Scale is 1: the glTF is in meters, and the housing is 137.1 mm long in both.
2. **bracket**: our ``single_bracket.stl`` by its own ICP, on the 77 % of its surface that lies within
   :data:`OVERLAP_M` of the glTF (0.63 mm RMS). It lands 2.73 mm along the axis from where the body's fit and our
   bracket joint put it, so the ring is placed by the bracket's fit: by the body's it would sit inside the bracket's
   flare, by the bracket's it covers the band our bracket mesh models for it.
3. **the fingers**: the glTF's opening in our joint coordinate is searched over ``q`` with our kinematics
   (``config/rg6_v2.yaml``) on the finger tips: q = 0.615 rad from our open stop, 93.1 mm between the pads. Each
   moving link's STL is then placed there and refined by its own ICP: finger tip 0.04 / 0.08 mm RMS, moment arm
   1.6 mm, truss arm 1.3 mm. The arms' residual is shape, not pose: our arm meshes are coarse (1880 triangles against
   the glTF's 394 000 for one moment arm), the median distance is 0.5 / 0.4 mm, and the fitted poses lie within
   0.5 mm and 0.15 deg (moment arm) and 1.3 mm and 1.3 deg (truss arm) of the kinematics'.

Every cap goes to the link whose placed mesh it lies closest to. The four over the finger-tip pivots of each finger
ride on the moment and the truss arm, two each, one either side of the finger tip; finger 1's are written, the macro
instantiates the finger twice, and finger 2's land within :data:`MIRROR_TOLERANCE_M` of them, which is checked. Their
centers lie 0.2 mm (truss arm) and 0.4 mm (moment arm) from our finger-tip joint. The caps are round, so whether one
turns with its pin or with the link around it shows no difference. The four over the truss arms' body pivots go to the
body and are not written: they lie inside our body's housing, and over 60 views at three openings one pixel of them
showed (:data:`HIDDEN`). The covers ride on the truss arm (:func:`find_covers` says why); the two levers between each
finger's plates carry the same gray and are not written, the plates hide them.

The DAE is written with pycollada directly, ``Z_UP``, in meters, and with a fixed :data:`STAMP`, so a second run on
the same source writes the same bytes.

    uv run tools/scripts/rg6_details_mesh.py tools/gripper_rg6.gltf \\
        tools/src/rg6_description/meshes/rg6_v2/visual
"""

from __future__ import annotations

import argparse
import base64
import datetime
import json
from pathlib import Path

import collada
import fast_simplification
import numpy as np
import trimesh
import yaml
from collada import asset, geometry, material, scene, source
from scipy import ndimage
from scipy.spatial import ConvexHull, cKDTree
from scipy.spatial.transform import Rotation
from skimage import measure

#: The package this script writes into; the STLs registered against and the kinematics are read from it.
PACKAGE = Path(__file__).resolve().parents[1] / "src" / "rg6_description"
VISUAL = PACKAGE / "meshes" / "rg6_v2" / "visual"
#: The glTF's baseColor of the ring and of the caps, sRGB 0..255, and how many bodies carry each (2026-10-10).
RING_SRGB = (73, 157, 218)
CAP_SRGB = (80, 63, 52)
RING_COUNT = 1
CAP_COUNT = 12
#: How many caps each link frame takes, over both fingers (2026-10-10: two per pivot, one each side of the link).
CAPS_PER_LINK = {"body": 4, "moment_arm": 4, "truss_arm": 4}
#: Link frames whose caps are found and checked but not written: the body's lie inside its housing (2026-10-10).
HIDDEN = ("body",)
#: Bodies above this many triangles are left out of every registration target: they are the ten coil springs inside
#: the mounting end, 1.33 M triangles each and nowhere near the links (2026-10-10).
MAX_TARGET_TRIANGLES = 500_000
#: The glTF's axes in our body frame before any fit: its x is our z (the hand's axis), its y our x (the opening).
AXES = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
#: A glTF body belongs to one of our placed meshes when the median distance of its surface to it is below this.
NEAR_M = 0.002
#: Our bracket is fitted on its points this close to the glTF only: below the vendor's flange it carries geometry the
#: vendor's model has none of (14 mm of it, 2026-10-10).
OVERLAP_M = 0.003
#: Finger 2's caps, brought into finger 1's link frames, must land this close to finger 1's.
MIRROR_TOLERANCE_M = 0.0005
#: Only the ring's rim is kept: it is a 2.1 mm thick disc with bosses and screw holes, and everything inside this
#: radius lies inside our bracket, whose cylinder is 35.5 mm in radius there (2026-10-10: 13 558 -> 914 triangles).
RING_MIN_RADIUS_M = 0.034
#: The rim's radius is 35.42..35.57 mm, our bracket's cylinder 35.5 mm: coplanar, so the rim is laid this far proud
#: of it, or the renderers flicker between the two.
RING_PROUD_M = 0.0002
#: Our arm meshes close each pivot eye with a flat face at |y| = 12.5 mm, where the vendor's arm leaves the cap's end
#: open: the caps end at |y| 12.36 to 12.57 mm, so the moment arm's face covered its caps whole and the truss arm's
#: one side of them (2026-10-10, 0 to 60 of 160 000 pixels a view). Each cap is moved along its axis until its outer
#: end lies this far beyond the face of its link around it.
CAP_PROUD_M = 0.0002
#: A cap's axis must lie this close to its link's y, the pivot axis; a wider angle means the fit went wrong.
CAP_AXIS_TOLERANCE_RAD = np.radians(2.0)
#: A cap is a sleeve whose profile is eight points (bore, outer wall, a 0.6 mm chamfer at each of the four corners),
#: tessellated in the source with 144 sections in 2304 triangles. It is written as that profile revolved in this many
#: sections: 768 triangles, 0.009 mm off the source at most (2026-10-10). Decimating does worse on these long thin
#: triangles: fast-simplification left 0.28 mm at a third fewer.
CAP_SECTIONS = 48
#: A profile point may lie this far inside the profile's convex hull; the revolve takes the hull.
PROFILE_TOLERANCE_M = 1e-6
#: How far the written part may lie from the source's, both ways.
DEVIATION_M = 0.00002
#: Normals are split where neighboring faces meet at more than this angle, so the cylinders shade round and the
#: rims stay sharp.
SMOOTH_ANGLE_RAD = np.radians(30.0)
#: Each part's look [sRGB diffuse, specular, shininess]: the ring the glTF's blue with a little gloss; the caps and the
#: covers the grip pads' gray, ``onrobot/dark_grey`` in materials_upstream.urdf.xacro (the owner, 2026-10-10). The
#: twin (``clair.twin.collada``) takes the diffuse alone; Foxglove and RViz shade with all three.
LOOK = {
    "ring": (tuple(c / 255.0 for c in RING_SRGB), 0.3, 30.0),
    "pad": ((0.25, 0.25, 0.25), 0.25, 20.0),
}
#: The glTF's materials of the covers: the rounded plates on both faces of each finger's arms in a mid gray, the
#: rounded inlay on the truss arm's inner face in a near white (2026-10-10). The owner's RG6 has them in its pads' gray.
COVER_SRGB = ((128, 128, 128), (244, 242, 245))
#: What those materials color, counted 2026-10-10: six covers (two plates and one inlay a finger), two blocks between
#: each finger's plates that the plates hide, and one part inside the mounting end. Another count stops the script.
COVER_COUNTS = {"cover": 6, "between": 2, "elsewhere": 1}
#: A cover's tessellation is a CAD export's: slivers, zero-length edges, fans over the flat faces, and no decimator
#: got below 14 000 triangles of a plate's 157 000 (fast-simplification, Open3D, pyfqmr, 2026-10-10). So each is
#: rebuilt: its surface sampled into a grid of this pitch, the shell closed and filled, the surface drawn by marching
#: cubes and only then decimated.
COVER_PITCH_M = 0.0001
#: The decimated cover may lie this far from the source at most: half a millimeter on parts 60 mm long; the grid's
#: shell puts the rebuilt surface about 0.1 mm outside the source's (2026-10-10: 0.3 mm at 1200 triangles a plate).
COVER_DEVIATION_M = 0.0004
#: The count the marching cubes' surface is first decimated to, before the bisection for the fewest triangles.
COVER_INTERMEDIATE_TRIANGLES = 20_000
#: The files written, by the link frame they are in.
FILES = {
    "single_bracket": "single_bracket_ring.dae",
    "moment_arm": "moment_arm_details.dae",
    "truss_arm": "truss_arm_details.dae",
}
#: Written into every run's ``<created>``/``<modified>``, so a second run on the same source writes the same file.
STAMP = datetime.datetime(2026, 10, 10)


class Source:
    """The glTF's bodies: triangles per node in the glTF's frame [m], and each body's baseColor."""

    def __init__(self, path: str) -> None:
        with open(path) as handle:
            gltf = json.load(handle)
        buffer = base64.b64decode(gltf["buffers"][0]["uri"].split(",", 1)[1])
        accessors, views = gltf["accessors"], gltf["bufferViews"]

        def read(index: int) -> np.ndarray:
            acc = accessors[index]
            view = views[acc["bufferView"]]
            dtype = {5126: np.float32, 5125: np.uint32, 5123: np.uint16}[acc["componentType"]]
            width = {"VEC3": 3, "SCALAR": 1}[acc["type"]]
            offset = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
            return np.frombuffer(buffer, dtype=dtype, count=acc["count"] * width, offset=offset).reshape(-1, width)

        colors = [
            tuple(round(c * 255) for c in m["pbrMetallicRoughness"].get("baseColorFactor", (1, 1, 1))[:3])
            for m in gltf["materials"]
        ]
        self.meshes: list[trimesh.Trimesh] = []
        self.colors: list[set[tuple[int, int, int]]] = []
        for node in gltf["nodes"]:
            if any(key in node for key in ("matrix", "translation", "rotation", "scale", "children")):
                raise SystemExit(f"{node['name']}: the script reads flat, untransformed nodes -- source changed?")
            vertices, faces, offset = [], [], 0
            used = set()
            for prim in gltf["meshes"][node["mesh"]]["primitives"]:
                v = read(prim["attributes"]["POSITION"]).astype(float)
                faces.append(read(prim["indices"]).reshape(-1, 3).astype(np.int64) + offset)
                vertices.append(v)
                offset += len(v)
                used.add(colors[prim["material"]])
            self.meshes.append(trimesh.Trimesh(np.concatenate(vertices), np.concatenate(faces), process=False))
            self.colors.append(used)

    def bodies_of(self, srgb: tuple[int, int, int], count: int, what: str) -> list[int]:
        """The bodies wholly in ``srgb``; exactly ``count`` of them, or the script stops."""
        hits = [index for index, used in enumerate(self.colors) if used == {srgb}]
        if len(hits) != count:
            raise SystemExit(f"{what}: {len(hits)} bodies in sRGB {srgb}, expected {count} -- source changed?")
        return hits


def kinematics() -> tuple[dict, float]:
    """The joint origins [m] and the linkage offset [rad] the macro reads from ``config/rg6_v2.yaml``."""
    config = yaml.safe_load((PACKAGE / "config" / "rg6_v2.yaml").read_text())
    return config["origin"], config["kinematics"]["offset"]


def frame(xyz, rpy=(0.0, 0.0, 0.0)) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def finger_links(q_rad: float, finger: int) -> dict[str, np.ndarray]:
    """Each finger link's pose in the body frame at ``q_rad``, as the macro builds it (every joint mimics the driver
    with multiplier 1; finger 2 is finger 1 turned by pi about the body's z)."""
    origin, offset = kinematics()

    def at(name: str) -> tuple[float, float, float]:
        return origin[name]["x"], origin[name]["y"], origin[name]["z"]

    base = frame((0, 0, 0), (0, 0, 0 if finger == 1 else np.pi))
    moment = base @ frame(at("moment_arm"), (0, offset, 0)) @ frame((0, 0, 0), (0, q_rad, 0))
    truss = base @ frame(at("truss_arm"), (0, offset, 0)) @ frame((0, 0, 0), (0, q_rad, 0))
    tip = truss @ frame(at("finger_tip"), (0, -offset, 0)) @ frame((0, 0, 0), (0, -q_rad, 0))
    return {"moment_arm": moment, "truss_arm": truss, "finger_tip": tip}


def surface(mesh: trimesh.Trimesh, count: int, seed: int) -> np.ndarray:
    return trimesh.sample.sample_surface(mesh, count, seed=seed)[0]


def distances_m(target: trimesh.Trimesh, points: np.ndarray) -> np.ndarray:
    """Each point's distance to the surface of ``target`` [m], exact (closest point on the triangles). Computed in
    millimeters: in meters ``trimesh.proximity.closest_point`` (trimesh 5.0) misses the nearest triangle of a small
    part, and a cap's own surface samples came out up to 0.16 mm off it (2026-10-10)."""
    scaled = target.copy()
    scaled.apply_scale(1000.0)
    return trimesh.proximity.closest_point(scaled, points * 1000.0)[1] / 1000.0


def rms_m(points: np.ndarray, target: trimesh.Trimesh) -> float:
    """RMS distance of ``points`` to the surface of ``target`` [m]."""
    return float(np.sqrt(np.mean(distances_m(target, points) ** 2)))


def near_bodies(source: Source, candidates: list[int], placed: np.ndarray) -> list[int]:
    """The candidate bodies whose surface lies within :data:`NEAR_M` of the placed points (median)."""
    tree = cKDTree(placed)
    return [b for b in candidates if np.median(tree.query(surface(source.meshes[b], 500, seed=b))[0]) < NEAR_M]


def fit(ours: np.ndarray, target: trimesh.Trimesh, initial: np.ndarray) -> tuple[np.ndarray, float]:
    """ICP of our points onto ``target`` from ``initial``: the rigid transform ours -> glTF and its RMS [m]."""
    matrix, _, _ = trimesh.registration.icp(
        ours, surface(target, 200_000, seed=0), initial=initial, threshold=1e-9, max_iterations=80, scale=False
    )
    return matrix, rms_m(trimesh.transform_points(ours[:5000], matrix), target)


def register(source: Source, caps: list[int], ring: list[int]) -> tuple[dict[str, np.ndarray], float]:
    """Each of our links' pose in the glTF (ours -> glTF), finger links keyed ``<link>_<finger>``, and the glTF's
    opening in our joint coordinate [rad]."""
    candidates = [
        b
        for b, mesh in enumerate(source.meshes)
        if len(mesh.faces) <= MAX_TARGET_TRIANGLES and b not in caps and b not in ring
    ]
    everything = trimesh.util.concatenate([source.meshes[b] for b in candidates])
    tree = cKDTree(surface(everything, 300_000, seed=0))

    body = surface(trimesh.load(VISUAL / "body.stl"), 20_000, seed=1)
    fits = []
    for roll in (0.0, np.pi):
        axes = AXES @ Rotation.from_euler("z", roll).as_matrix()
        initial = np.eye(4)
        initial[:3, :3] = axes
        slide = [float(np.mean(tree.query(body @ axes.T + [x, 0.0, 0.0])[0])) for x in np.arange(-0.05, 0.15, 0.001)]
        initial[0, 3] = -0.05 + 0.001 * int(np.argmin(slide))
        target = near_bodies(source, candidates, trimesh.transform_points(body, initial))
        matrix, rms = fit(body, trimesh.util.concatenate([source.meshes[b] for b in target]), initial)
        print(f"body, roll {np.degrees(roll):5.1f} deg: ICP RMS {rms * 1000:.3f} mm on {len(target)} bodies")
        fits.append((rms, matrix))
    poses = {"body": min(fits, key=lambda item: item[0])[1]}

    origin, _ = kinematics()
    on_body = poses["body"] @ frame((0.0, 0.0, -origin["single_bracket"]["z"]))
    bracket = surface(trimesh.load(VISUAL / "single_bracket.stl"), 20_000, seed=1)
    shared = bracket[tree.query(trimesh.transform_points(bracket, on_body))[0] < OVERLAP_M]
    target = near_bodies(source, candidates, trimesh.transform_points(shared, on_body))
    poses["single_bracket"], rms = fit(shared, trimesh.util.concatenate([source.meshes[b] for b in target]), on_body)
    print(f"single_bracket: ICP RMS {rms * 1000:.3f} mm on {len(target)} bodies ({len(shared)} of 20000 points shared)")
    report("single_bracket", on_body, poses["single_bracket"], "the body's fit and our bracket joint")

    tip = surface(trimesh.load(VISUAL / "finger_tip.stl"), 4000, seed=2)
    grid = np.arange(-0.3, 1.4, 0.005)
    errors = [
        np.mean(tree.query(trimesh.transform_points(tip, poses["body"] @ finger_links(q, 1)["finger_tip"]))[0])
        for q in grid
    ]
    q_rad = float(grid[int(np.argmin(errors))])
    print(f"glTF opening: q = {q_rad:.3f} rad from our open stop")

    meshes = {name: trimesh.load(VISUAL / f"{name}.stl") for name in ("moment_arm", "truss_arm", "finger_tip")}
    for finger in (1, 2):
        placed = finger_links(q_rad, finger)
        points = {name: surface(mesh, 20_000, seed=3) for name, mesh in meshes.items()}
        initial = {name: poses["body"] @ placed[name] for name in meshes}
        trees = {name: cKDTree(trimesh.transform_points(points[name], initial[name])) for name in meshes}
        owner = {}
        for b in candidates:
            sample = surface(source.meshes[b], 500, seed=b)
            distance = {name: float(np.median(t.query(sample)[0])) for name, t in trees.items()}
            name = min(distance, key=distance.get)
            if distance[name] < NEAR_M:
                owner.setdefault(name, []).append(b)
        for name in meshes:
            if name not in owner:
                raise SystemExit(f"{name}_{finger}: no glTF body near our placed link -- source changed?")
            target = trimesh.util.concatenate([source.meshes[b] for b in owner[name]])
            matrix, rms = fit(points[name], target, initial[name])
            poses[f"{name}_{finger}"] = matrix
            print(f"{name}_{finger:d}: ICP RMS {rms * 1000:.3f} mm on {len(owner[name])} bodies")
            report(f"{name}_{finger:d}", initial[name], matrix, "the body's fit and our kinematics")
    return poses, q_rad


def report(key: str, expected: np.ndarray, fitted: np.ndarray, by: str) -> None:
    """How far a link's own fit lies from where ``by`` puts it, in the link's frame."""
    offset = np.linalg.inv(expected) @ fitted
    angle_deg = np.degrees(Rotation.from_matrix(offset[:3, :3]).magnitude())
    shift_mm = np.round(offset[:3, 3] * 1000, 2).tolist()
    print(f"{'':15s} fit vs {by}: {shift_mm} mm, {angle_deg:.2f} deg")


def assign(source: Source, caps: list[int], poses: dict[str, np.ndarray]) -> dict[str, list[trimesh.Trimesh]]:
    """Each cap in the frame of the link it lies closest to; the moment and truss arms take finger 1's, checked
    against finger 2's."""
    placed = {"body": trimesh.load(VISUAL / "body.stl")}
    for finger in (1, 2):
        for name in ("moment_arm", "truss_arm", "finger_tip"):
            placed[f"{name}_{finger}"] = trimesh.load(VISUAL / f"{name}.stl")
    trees = {
        key: cKDTree(trimesh.transform_points(surface(mesh, 40_000, seed=4), poses[key]))
        for key, mesh in placed.items()
    }
    found: dict[str, list[trimesh.Trimesh]] = {}
    for b in caps:
        sample = surface(source.meshes[b], 500, seed=b)
        distance = {key: float(np.median(t.query(sample)[0])) for key, t in trees.items()}
        key = min(distance, key=distance.get)
        cap = source.meshes[b].copy()
        cap.apply_transform(np.linalg.inv(poses[key]))
        found.setdefault(key, []).append(cap)
        center_mm = np.round(cap.bounds.mean(0) * 1000, 1).tolist()
        print(f"cap {b:3d} -> {key:12s} {distance[key] * 1000:.2f} mm off it, center {center_mm} mm in its frame")
    counts = {
        link: sum(len(v) for k, v in found.items() if k.removesuffix("_1").removesuffix("_2") == link)
        for link in CAPS_PER_LINK
    }
    if counts != CAPS_PER_LINK:
        raise SystemExit(f"caps per link {counts}, expected {CAPS_PER_LINK} -- source changed?")
    for link in HIDDEN:
        print(f"{link}: {len(found[link])} caps found, not written (they lie inside it)")
    out = {}
    for link in ("moment_arm", "truss_arm"):
        first, second = found[f"{link}_1"], found[f"{link}_2"]
        centers = np.array([c.bounds.mean(0) for c in second])
        for cap in first:
            gap = np.linalg.norm(centers - cap.bounds.mean(0), axis=1).min()
            print(f"{link}: finger 2's cap {gap * 1000:.3f} mm off finger 1's")
            if gap > MIRROR_TOLERANCE_M:
                raise SystemExit(f"{link}: finger 2's cap lies {gap * 1000:.2f} mm off finger 1's -- fit failed?")
        out[link] = first
    return out


def proud(cap: trimesh.Trimesh, link: trimesh.Trimesh) -> trimesh.Trimesh:
    """``cap`` moved along its axis, the link's y, until its outer end lies :data:`CAP_PROUD_M` beyond the face of
    ``link`` around it."""
    cap = cap.copy()
    cap.merge_vertices()
    axis = cap.symmetry_axis / np.linalg.norm(cap.symmetry_axis)
    if np.arccos(min(1.0, abs(axis[1]))) > CAP_AXIS_TOLERANCE_RAD:
        raise SystemExit(f"cap: axis {np.round(axis, 3)} is not its link's y -- fit failed?")
    center = cap.center_mass
    outward = np.sign(center[1])
    radius_m = np.linalg.norm((cap.vertices - center)[:, [0, 2]], axis=1).max()
    around = np.linalg.norm(link.vertices[:, [0, 2]] - center[[0, 2]], axis=1) < radius_m
    face_m = (outward * link.vertices[around, 1]).max()
    shift_m = face_m + CAP_PROUD_M - (outward * cap.vertices[:, 1]).max()
    cap.apply_translation([0.0, outward * shift_m, 0.0])
    print(f"cap at y {center[1] * 1000:+6.2f} mm: link face at {face_m * 1000:.2f} mm, moved {shift_m * 1000:+.2f} mm")
    return cap


def find_covers(source: Source, poses: dict[str, np.ndarray]) -> list[trimesh.Trimesh]:
    """Finger 1's covers in its truss arm's frame, checked against finger 2's.

    The plates lie over the gap between the moment and the truss arm and touch neither (0.1 to 0.3 mm off both,
    2026-10-10); a rigid part can ride on only one of them, and the two slide 15 mm against each other over the
    stroke. They go on the truss arm with the inlay, which lies 90 % on it: the plates sit 0.5 mm nearer the truss
    arm's mesh, overlap its outline more (20 to 27 % against 13 to 20 %) and cut into the other arm less when closed
    (4.5 % against 13 % of a plate's surface at q = 1.2).
    """
    found: dict[str, list] = {"cover": [], "between": [], "elsewhere": []}
    body_from_gltf = np.linalg.inv(poses["body"])
    for srgb in COVER_SRGB:
        for b in (i for i, used in enumerate(source.colors) if used == {srgb}):
            center = trimesh.transform_points([source.meshes[b].centroid], body_from_gltf)[0]
            if center[2] < 0.09:  # not on a finger: inside the housing or the mounting end
                found["elsewhere"].append(b)
                continue
            finger = 1 if center[0] < 0 else 2
            link = np.linalg.inv(poses[f"truss_arm_{finger}"]) @ np.append(source.meshes[b].centroid, 1.0)
            kind = "between" if srgb == COVER_SRGB[0] and abs(link[1]) < 0.005 else "cover"
            found[kind].append((finger, b))
    counts = {kind: len(items) for kind, items in found.items()}
    if counts != COVER_COUNTS:
        raise SystemExit(f"covers: {counts}, expected {COVER_COUNTS} -- source changed?")
    print(f"covers: {sorted(b for _, b in found['cover'])}, hidden between the plates: {found['between']}")
    placed = {1: [], 2: []}
    for finger, b in found["cover"]:
        cover = source.meshes[b].copy()
        cover.apply_transform(np.linalg.inv(poses[f"truss_arm_{finger}"]))
        placed[finger].append(cover)
    centers = np.array([c.centroid for c in placed[2]])
    for cover in placed[1]:
        gap = np.linalg.norm(centers - cover.centroid, axis=1).min()
        print(f"truss_arm: finger 2's cover {gap * 1000:.3f} mm off finger 1's")
        if gap > MIRROR_TOLERANCE_M:
            raise SystemExit(f"truss_arm: finger 2's cover lies {gap * 1000:.2f} mm off finger 1's -- fit failed?")
    return placed[1]


def rebuilt(cover: trimesh.Trimesh) -> trimesh.Trimesh:
    """``cover`` rebuilt on a :data:`COVER_PITCH_M` grid and decimated to the fewest triangles within
    :data:`COVER_DEVIATION_M` of it."""
    points = surface(cover, int(cover.area / COVER_PITCH_M**2 * 6), seed=7)
    low = points.min(axis=0) - 3 * COVER_PITCH_M
    index = np.floor((points - low) / COVER_PITCH_M).astype(int)
    grid = np.zeros(index.max(axis=0) + 4, dtype=bool)
    grid[tuple(index.T)] = True
    # Two cells of dilation close the sampled shell, so the fill finds an inside; the erosion takes them back.
    grid = ndimage.binary_erosion(ndimage.binary_fill_holes(ndimage.binary_dilation(grid, iterations=2)), iterations=2)
    vertices, faces, _, _ = measure.marching_cubes(ndimage.gaussian_filter(grid.astype(np.float32), 1.0), 0.5)
    dense = trimesh.Trimesh(vertices * COVER_PITCH_M + low + COVER_PITCH_M / 2, faces[:, ::-1])
    origin = dense.centroid

    def simplified(mesh: trimesh.Trimesh, triangles: int) -> trimesh.Trimesh:
        points_mm, kept = fast_simplification.simplify(
            ((mesh.vertices - origin) * 1000.0).astype(np.float32),
            mesh.faces.astype(np.int32),
            1.0 - triangles / len(mesh.faces),
        )
        return trimesh.Trimesh(points_mm / 1000.0 + origin, kept, process=False)

    # One pass down to an intermediate count first: a pass over the marching cubes' 0.8 to 1 M triangles takes
    # seconds, one over the intermediate a fraction of that, and the bisection below runs ten of them.
    intermediate = simplified(dense, COVER_INTERMEDIATE_TRIANGLES)
    best = None
    low_count, high_count = 200, COVER_INTERMEDIATE_TRIANGLES
    while high_count - low_count > 50:
        count = (low_count + high_count) // 2
        small = simplified(intermediate, count)
        if deviation_m(cover, small) <= COVER_DEVIATION_M:
            best, high_count = small, count
        else:
            low_count = count
    if best is None:
        raise SystemExit(f"cover: no decimation within {COVER_DEVIATION_M * 1000:.2f} mm -- source changed?")
    return best


def rim(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """The ring's rim in our bracket's frame: the triangles outside :data:`RING_MIN_RADIUS_M`, laid
    :data:`RING_PROUD_M` proud radially."""
    radius = np.linalg.norm(mesh.vertices[:, :2], axis=1)
    keep = np.flatnonzero((radius[mesh.faces] >= RING_MIN_RADIUS_M).all(axis=1))
    if not len(keep):
        raise SystemExit("ring: no triangle outside the bracket's cylinder -- fit failed?")
    band = mesh.submesh([keep], append=True)
    radius = np.linalg.norm(band.vertices[:, :2], axis=1, keepdims=True)
    vertices = band.vertices.copy()
    vertices[:, :2] *= (radius + RING_PROUD_M) / radius
    return trimesh.Trimesh(vertices, band.faces, process=False)


def deviation_m(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    """The largest distance between the two surfaces, both ways: sampled points of each onto the other [m]."""
    return float(max(distances_m(a, surface(b, 4000, seed=5)).max(), distances_m(b, surface(a, 4000, seed=6)).max()))


def revolved(cap: trimesh.Trimesh) -> trimesh.Trimesh:
    """The cap as its profile revolved about its own axis in :data:`CAP_SECTIONS` sections."""
    cap = cap.copy()
    cap.merge_vertices()
    if cap.symmetry != "radial":
        raise SystemExit(f"cap: symmetry {cap.symmetry}, expected radial -- source changed?")
    axis = cap.symmetry_axis / np.linalg.norm(cap.symmetry_axis)
    side = np.cross(axis, [1.0, 0.0, 0.0] if abs(axis[0]) < 0.9 else [0.0, 1.0, 0.0])
    side /= np.linalg.norm(side)
    placed = np.eye(4)
    placed[:3, :3] = np.column_stack([side, np.cross(axis, side), axis])
    placed[:3, 3] = cap.center_mass  # on the axis for a body of revolution
    local = (cap.vertices - placed[:3, 3]) @ placed[:3, :3]
    profile = np.column_stack([np.hypot(local[:, 0], local[:, 1]), local[:, 2]])
    hull = ConvexHull(profile)
    inside_m = np.abs(hull.equations[:, :2] @ profile.T + hull.equations[:, 2:]).min(axis=0).max()
    if inside_m > PROFILE_TOLERANCE_M:
        raise SystemExit(f"cap: profile point {inside_m * 1000:.3f} mm inside its hull, not convex -- source changed?")
    outline = profile[hull.vertices]
    solid = trimesh.creation.revolve(np.vstack([outline, outline[:1]]), sections=CAP_SECTIONS)
    if solid.volume < 0:
        solid.invert()
    solid.apply_transform(placed)
    return solid


def write(parts: list[tuple[str, trimesh.Trimesh]], dae_path: Path) -> None:
    """One geometry per part and one material per look, meters, ``Z_UP``."""
    dae = collada.Collada()
    dae.assetInfo.upaxis = asset.UP_AXIS.Z_UP
    dae.assetInfo.created = dae.assetInfo.modified = STAMP
    dae.assetInfo.contributors.append(
        asset.Contributor(
            authoring_tool="tools/scripts/rg6_details_mesh.py",
            source_data="gripper_rg6.gltf (Onshape export of the OnRobot RG6, the owner's, not committed)",
        )
    )
    materials = {}
    for look in sorted({look for look, _ in parts}):
        color, specular, shininess = LOOK[look]
        effect = material.Effect(
            f"{look}-effect",
            [],
            "phong",
            diffuse=(*color, 1.0),
            specular=(specular, specular, specular, 1.0),
            shininess=shininess,
            transparent=None,
            transparency=None,
        )
        materials[look] = material.Material(f"{look}-material", look, effect)
        dae.effects.append(effect)
        dae.materials.append(materials[look])
    nodes = []
    for index, (look, mesh) in enumerate(parts):
        key = f"{look}{index}"
        shaded = trimesh.graph.smooth_shade(mesh, angle=SMOOTH_ANGLE_RAD)
        # Rounded to a micrometer and a ten-thousandth: the text is a third of what float32 digits make of it.
        points = np.round(shaded.vertices, 6).ravel()
        directions = np.round(shaded.vertex_normals, 4).ravel()
        positions = source.FloatSource(f"{key}-positions", points, ("X", "Y", "Z"))
        normals = source.FloatSource(f"{key}-normals", directions, ("X", "Y", "Z"))
        geom = geometry.Geometry(dae, f"{key}-mesh", key, [positions, normals])
        inputs = source.InputList()
        inputs.addInput(0, "VERTEX", f"#{key}-positions")
        inputs.addInput(0, "NORMAL", f"#{key}-normals")
        geom.primitives.append(geom.createTriangleSet(shaded.faces.astype(np.int32).ravel(), inputs, look))
        dae.geometries.append(geom)
        bound = scene.MaterialNode(look, materials[look], inputs=[])
        nodes.append(scene.Node(key, children=[scene.GeometryNode(geom, [bound])]))
    dae.scenes.append(scene.Scene(dae_path.stem, nodes))
    dae.scene = dae.scenes[0]
    dae.write(str(dae_path))


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("gltf", help="the owner's gripper_rg6.gltf")
    parser.add_argument("out", nargs="?", default=str(VISUAL), help="where to write the DAEs (the visual meshes)")
    args = parser.parse_args(argv)
    gltf = Source(args.gltf)
    ring = gltf.bodies_of(RING_SRGB, RING_COUNT, "ring")
    caps = gltf.bodies_of(CAP_SRGB, CAP_COUNT, "caps")
    poses, _ = register(gltf, caps, ring)
    ring_mesh = gltf.meshes[ring[0]].copy()
    ring_mesh.apply_transform(np.linalg.inv(poses["single_bracket"]))
    contents = {"single_bracket": [("ring", rim(ring_mesh))]}
    for link, found in assign(gltf, caps, poses).items():
        mesh = trimesh.load(VISUAL / f"{link}.stl")
        contents[link] = [("cap", proud(cap, mesh)) for cap in found]
    contents["truss_arm"] += [("cover", cover) for cover in find_covers(gltf, poses)]
    for link, parts in contents.items():
        written = []
        for kind, mesh in parts:
            out = {"ring": lambda m: m, "cap": revolved, "cover": rebuilt}[kind](mesh)
            deviation = deviation_m(mesh, out)
            if deviation > (COVER_DEVIATION_M if kind == "cover" else DEVIATION_M):
                raise SystemExit(f"{FILES[link]}: {kind} {deviation * 1000:.3f} mm off the source -- source changed?")
            written.append(("ring" if kind == "ring" else "pad", out))
            print(
                f"{FILES[link]:24s} {kind:5s} {len(mesh.faces):6d} -> {len(out.faces):5d} triangles, max deviation "
                f"{deviation * 1000:.4f} mm, bounds {np.round(out.bounds * 1000, 2).tolist()} mm"
            )
        path = Path(args.out) / FILES[link]
        write(written, path)
        print(f"{path}: {path.stat().st_size} bytes")


if __name__ == "__main__":
    raise SystemExit(main())
