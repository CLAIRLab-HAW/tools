#!/usr/bin/env python3
"""Derives a gripper model's joint angle ─▶ gripping width table from a GENERATED URDF.

Why from the URDF and not from a formula:  the fingers of a parallel gripper are a four-bar chain (the rg6_v2's
moment_arm ─▶ truss_arm ─▶ finger_tip ─▶ flex_finger, the 2F-85's knuckle ─▶ finger ─▶ finger_tip), every joint tied
to the driver via ``mimic``.  There is no closed formula for it, and an approximation placed alongside would be
exactly the second version on which the old model and its driver have already drifted apart once (R19).

Any model, named by its driver joint and its two pad links.  Where a pad's gripping face is, is either the pad link's
origin (``--pad-face origin``, the rg6_v2, whose flex_finger frames sit on the faces) or the innermost points of its
collision mesh (``--pad-face mesh``, the 2F-85, whose finger_tip frames sit at the pivot).  With ``--reference`` the
script also reports where the pads stand along that frame's +z (the model file's ``pad_offset`` before the TCP's own
depth is subtracted) and how tall the gripping face is.

    xacro robot.urdf.xacro > /tmp/robot.urdf
    python3 derive_finger_kinematics.py /tmp/robot.urdf --driver arm_0_gripper_robotiq_85_left_knuckle_joint \\
        --pads arm_0_gripper_robotiq_85_left_finger_tip_link arm_0_gripper_robotiq_85_right_finger_tip_link \\
        --pad-face mesh --reference arm_0_gripper_robotiq_85_base_link --format yaml

Why a TABLE and not an import:  the gripper bridge runs on the robot and is not supposed to import anything there that
does not belong to the robot.  A table is data, not a package -- it costs no dependency, and an outdated table is a
visible file with a date instead of a silent deviation.

    xacro robot.urdf.xacro > /tmp/robot.urdf
    python3 derive_finger_kinematics.py /tmp/robot.urdf \\
        > src/rg6_control/scripts/rg6_finger_kinematics.json

Two output formats, ONE source: ``yaml`` is what a model file under robot/contract's ``profiles/grippers/`` carries
(the linkage and where the pads are), ``json`` the copy the RG6's bridge reads on the robot.  The container mock,
``gripper_control``'s ``gripper_sim``, takes the model file's table as parameters and needs no copy of its own.
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET

import numpy as np

#: Step size of the sampling points.  0,05 rad keeps the interpolation error
#: below 0,05 mm; that is half the finger position resolution of the RG6.
STEP_RAD = 0.05
#: The rg6_v2's driver joint, the default model of this script.
DRIVER = "rg6_finger_joint"
#: A mesh vertex this close to a pad's innermost one belongs to its gripping face [m].
FACE_BAND_M = 0.001


def _rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y))
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _chain(urdf, driver=DRIVER):
    root = ET.parse(urdf).getroot()
    K, limit = {}, None
    for j in root.findall("joint"):
        par, ch, o = j.find("parent"), j.find("child"), j.find("origin")
        if j.get("name") == driver and j.find("limit") is not None:
            # Both ends, not just the upper one.  ``lower`` is not always 0: the mechanical open stop of the RG6 does
            # not reach the geometric zero of the chain (measured 2026-08-27, see rg6_v2.yaml), and a table that
            # starts at 0 anyway would hand out an opening the hardware cannot deliver -- while the URDF, which does
            # read ``lower``, refuses it.  Two truths about the same joint.
            limit = (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
        if par is None or ch is None:
            continue
        ax, mimic = j.find("axis"), j.find("mimic")
        if j.get("name") == driver:
            factor = 1.0
        elif mimic is not None and mimic.get("joint") == driver:
            factor = float(mimic.get("multiplier") or 1.0)
        else:
            factor = 0.0  # neither the driver nor tied to it: the hand holds it still
        K[ch.get("link")] = dict(
            parent=par.get("link"),
            typ=j.get("type"),
            factor=factor,
            offset=float(mimic.get("offset") or 0.0) if mimic is not None and factor else 0.0,
            xyz=np.array([float(v) for v in ((o.get("xyz") if o is not None else None) or "0 0 0").split()]),
            rpy=np.array([float(v) for v in ((o.get("rpy") if o is not None else None) or "0 0 0").split()]),
            axis=np.array([float(v) for v in ((ax.get("xyz") if ax is not None else None) or "0 0 0").split()]),
        )
    if limit is None:
        raise SystemExit(f"{driver} has no joint limit in the URDF")
    return K, limit, root


def _pose(K, goal, q):
    T, chain, link = np.eye(4), [], goal
    while link in K:
        chain.append(K[link])
        link = K[link]["parent"]
    for j in reversed(chain):
        A = np.eye(4)
        A[:3, :3] = _rpy_to_R(*j["rpy"])
        A[:3, 3] = j["xyz"]
        if j["typ"] in ("revolute", "continuous") and j["factor"]:
            a = j["factor"] * q + j["offset"]
            k = j["axis"] / np.linalg.norm(j["axis"])
            S = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            B = np.eye(4)
            B[:3, :3] = np.eye(3) + np.sin(a) * S + (1 - np.cos(a)) * S @ S
            A = A @ B
        T = T @ A
    return T


def _pad_points(root, link: str) -> np.ndarray:
    """The vertices of ``link``'s collision meshes in its own frame (trimesh, the container has it)."""
    import trimesh  # only the mesh mode needs it, and only the container has it

    pts = []
    for el in root.findall("link"):
        if el.get("name") != link:
            continue
        for col in el.findall("collision"):
            mesh, o = col.find("geometry/mesh"), col.find("origin")
            if mesh is None:
                continue
            uri = mesh.get("filename")
            if uri.startswith("package://"):
                from ament_index_python.packages import get_package_share_directory

                pkg, rel = uri[len("package://") :].split("/", 1)
                uri = f"{get_package_share_directory(pkg)}/{rel}"
            v = np.asarray(trimesh.load(uri.removeprefix("file://"), force="mesh").vertices)
            v = v * np.array([float(x) for x in (mesh.get("scale") or "1 1 1").split()])
            A = np.eye(4)
            if o is not None:
                A[:3, :3] = _rpy_to_R(*[float(x) for x in (o.get("rpy") or "0 0 0").split()])
                A[:3, 3] = [float(x) for x in (o.get("xyz") or "0 0 0").split()]
            pts.append(v @ A[:3, :3].T + A[:3, 3])
    if not pts:
        raise SystemExit(f"{link} has no collision mesh to find its gripping face on")
    return np.vstack(pts)


def _in(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return pts @ T[:3, :3].T + T[:3, 3]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("urdf", help="generated URDF carrying the gripper model")
    ap.add_argument("--driver", default=DRIVER, help=f"the hand's driver joint (default {DRIVER})")
    ap.add_argument("--pads", nargs=2, metavar="LINK", help="the two pad links (default: the two *flex_finger links)")
    ap.add_argument("--pad-face", choices=("origin", "mesh"), default="origin", help="where a pad's gripping face is")
    ap.add_argument("--reference", help="frame whose +z the pads' depth is reported along (pad_offset, pad height)")
    ap.add_argument(
        "--format",
        choices=("json", "yaml"),
        default="json",
        help="json = table for the RG6's bridge (default), yaml = the model file's linkage and pad depths",
    )
    a = ap.parse_args(argv)
    K, (qmin, qmax), root = _chain(a.urdf, a.driver)
    faces = a.pads or sorted(n for n in K if n.endswith("flex_finger"))
    if len(faces) != 2:
        raise SystemExit(f"expected two pad links, found: {faces}")
    meshes = [_pad_points(root, f) for f in faces] if a.pad_face == "mesh" else None

    def pads(q: float) -> tuple[np.ndarray, np.ndarray]:
        """Both gripping faces in the reference frame (the chain's root without one), as point sets."""
        ref = np.linalg.inv(_pose(K, a.reference, q)) if a.reference else np.eye(4)
        out = []
        for i, f in enumerate(faces):
            T = ref @ _pose(K, f, q)
            out.append(_in(T, meshes[i]) if meshes else T[:3, 3][None, :])
        return out[0], out[1]

    # The closing axis: from one pad to the other with the hand at its widest.
    l0, r0 = pads(qmin)
    axis = r0.mean(axis=0) - l0.mean(axis=0)
    axis /= np.linalg.norm(axis)

    def faces_at(q: float) -> tuple[np.ndarray, np.ndarray]:
        left, right = pads(q)
        lx, rx = left @ axis, right @ axis
        return left[lx >= lx.max() - FACE_BAND_M], right[rx <= rx.min() + FACE_BAND_M]

    def width(q: float) -> float:
        left, right = faces_at(q)
        return float((right @ axis).min() - (left @ axis).max())

    qs = list(np.arange(qmin, qmax, STEP_RAD)) + [qmax]
    tab = [[round(float(q), 5), round(width(q), 6)] for q in qs]
    # The upper bound: where the width crosses zero, as for the rg6_v2 -- beyond it the fingers pass through each other.
    while len(tab) > 2 and tab[-1][1] <= 0.0:
        tab.pop()

    fine = np.linspace(qmin, tab[-1][0], 400)
    error = np.abs(np.array([width(q) for q in fine]) - np.interp(fine, [t[0] for t in tab], [t[1] for t in tab])).max()

    if a.format == "yaml":
        if not a.reference:
            raise SystemExit("--format yaml needs --reference: the pads' depth is measured along its +z")

        def depth(q: float) -> tuple[float, float]:
            """The pads' center along the reference's +z, and how tall the gripping face is [m]."""
            zs = np.concatenate([f[:, 2] for f in faces_at(q)])
            return float(zs.mean()), float(zs.max() - zs.min())

        # Widest first, from the open hand -- the TCP's end of the arc -- down to the shut one.
        samples = (0.14, 0.12, 0.10, 0.085, 0.08, 0.06, 0.045, 0.03, 0.02, 0.01, 0.0)
        widths = [tab[0][1]] + [w for w in samples if w < tab[0][1]]
        qw = [float(np.interp(-w, [-t[1] for t in tab], [t[0] for t in tab])) for w in widths]
        sys.stdout.write(f"# driver {a.driver}, pads {faces[0]} / {faces[1]}, face from {a.pad_face}\n")
        sys.stdout.write(f"# depth along {a.reference} +z; max interpolation error {error * 1000:.3f} mm\n")
        sys.stdout.write(f"pad_height_m: {depth(qw[0])[1]:.4f}\npad_depth:  # [width_m, depth_m]\n")
        for w, q in zip(widths, qw):
            sys.stdout.write(f"  - [{w:.6f}, {depth(q)[0]:.6f}]\n")
        sys.stdout.write("linkage:\n  table:\n")
        for q, w in tab:
            sys.stdout.write(f"    - [ {q:.5f}, {w:.6f} ]\n")
        return 0

    json.dump(
        {
            "kommentar": [
                f"Joint angle {a.driver} [rad] -> clear width between the pad",
                "faces [m], measured between the two flex_finger meshes.",
                "GENERATED, not maintained: scripts/derive_finger_kinematics.py from the",
                "generated URDF.  Regenerate after every change to the gripper model.",
                "The lower bound is the mechanical open stop; below it the chain",
                "computes a wider opening than the hardware reaches.",
                "The upper bound is the zero crossing of the width; beyond it the",
                "fingers drive through one another in the model and the width grows again.",
            ],
            "joint": a.driver,
            "joint_limits_rad": [round(qmin, 5), round(qmax, 5)],
            "max_interpolationsfehler_m": round(float(error), 6),
            "table_q_rad_width_m": tab,
        },
        sys.stdout,
        indent=1,
    )
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
