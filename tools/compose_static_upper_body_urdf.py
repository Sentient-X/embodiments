#!/usr/bin/env python3
"""Build the Sentient static upper body URDF from its source description.

The source is ``assets/sentient_static_upper_body/source/allisa_bot.urdf``, byte for byte: the
description the robot's operator selected on 2026-10-25, as Subraj's ``humanoid_pkg`` ships it
(``urdf/allisa_bot.urdf``, sha256 ``039b0b7d…``). That file is allisa_bot's CAD export (source
sha256 ``623305bb…``) with each of its 25 movable joints given the exact limits of the earlier
``allisa_feedback`` description, which "the operator rejected" for its geometry and keeps only
as a joint-limit donor (``source/allisa_bot_integration.md`` and
``source/allisa_bot_model_provenance.json``, copied beside it). Its meshes are the package's
display meshes, decimated from the CAD with original units and frames (the provenance file
records each one's source and display sha256 and bounds change).

The rendering is the source plus only what simulation and the registry need; the changes, and
the evidence that would replace each, are in ``known/static_upper_body.py``:

* every link with geometry gets its visual meshes' convex hull as its collision (the source
  has none) and that hull's mass properties at one density (it has no inertials);
* visual names are made unique and meshes are referenced as ``meshes/<name>.stl``;
* the four wrist pitch and roll joints' limits are negated: allisa_bot reverses those axes
  against the motors, whose confirmed limits the donor carries in motor coordinates, and the
  hardware profile reads each of those motors with sign -1;
* the ``<sensor>`` elements (Gazebo's, not URDF's) are removed; the registry declares the ZED;
* a ``ZED_camera`` frame (Stereolabs' ROS convention, x out of the lenses, y left) and both
  eyes' optical frames, 60 mm either side of it, are added on the source's
  ``zed2i_left_optical_frame``, so the left eye's frame is the source's own.

Run from the package root:
``uv run --with numpy --with scipy python tools/compose_static_upper_body_urdf.py``.
``tests/test_static_upper_body.py`` re-renders and compares bytes.
"""

from __future__ import annotations

import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import ConvexHull  # pyright: ignore[reportMissingTypeStubs]  # no stubs

from sx_embodiments.assembly import Pose
from sx_embodiments.known.zed import EYES, OPTICAL_RPY, eye_offset, optical_frame

ROOT = Path(__file__).resolve().parents[1]
ASSET = ROOT / "assets/sentient_static_upper_body"
SOURCE = ASSET / "source/allisa_bot.urdf"
TARGET = ASSET / "sentient_static_upper_body.urdf"
SOURCE_MESH_PREFIX = "package://humanoid_pkg/meshes/allisa_bot/"
#: The source's frame on the ZED's left eye, optical convention (z out of the lens).
SOURCE_ZED_EYE = "zed2i_left_optical_frame"
#: The neck's camera frame, a ZED 2i in Stereolabs' ROS convention (x out of the lenses, y left).
ZED_FRAME = "ZED_camera"
#: The joints allisa_bot turns against their motors; their donor limits are motor coordinates.
REVERSED_JOINTS = (
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
)

# One density for every link's convex hull. The hulls overstate solid volume, so this is
# below aluminium; it is an estimate for simulation until the CAD's mass properties arrive.
HULL_DENSITY_KG_M3 = 600.0

Vec = NDArray[np.float64]


def _floats(text: str | None, default: tuple[float, float, float]) -> Vec:
    return np.array([float(v) for v in text.split()] if text else default, dtype=np.float64)


def _rpy(rpy: Vec) -> Vec:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _negated(number: str) -> str:
    """``-number``, spelled with the source's digits."""
    return number.removeprefix("-") if number.startswith("-") else f"-{number}"


def _matrix(pose: Pose) -> Vec:
    out = np.eye(4)
    out[:3, :3] = _rpy(np.array(pose.rpy, dtype=np.float64))
    out[:3, 3] = pose.xyz
    return out


def _set_origin(element: ET.Element, matrix: Vec) -> None:
    """Write ``matrix`` as ``element``'s origin, URDF's fixed-axis roll, pitch, yaw."""
    r = matrix[:3, :3]
    rpy = (
        math.atan2(r[2, 1], r[2, 2]),
        -math.asin(max(-1.0, min(1.0, r[2, 0]))),
        math.atan2(r[1, 0], r[0, 0]),
    )
    xyz = np.where(np.abs(matrix[:3, 3]) < 1e-12, 0.0, matrix[:3, 3])
    ET.SubElement(element, "origin", {"xyz": _fmt(xyz), "rpy": _fmt(np.array(rpy))})


def _read_stl(path: Path) -> Vec:
    data = path.read_bytes()
    (count,) = struct.unpack_from("<I", data, 80)
    if len(data) != 84 + 50 * count:
        raise ValueError(f"{path.name}: not a binary STL")
    record = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    tris = np.frombuffer(data, dtype=record, offset=84)
    return tris["v"].reshape(-1, 3).astype(np.float64)


def _write_stl(path: Path, vertices: Vec, faces: NDArray[np.int32]) -> None:
    out = bytearray(b"sentient static upper body collision hull".ljust(80, b"\0"))
    out += struct.pack("<I", len(faces))
    for face in faces:
        a, b, c = vertices[face]
        normal: Vec = np.asarray(np.cross(b - a, c - a), dtype=np.float64)
        normal = normal / (float(np.sqrt(normal @ normal)) or 1.0)
        out += struct.pack("<12fH", *normal, *a, *b, *c, 0)
    path.write_bytes(bytes(out))


def _hull(points: Vec) -> tuple[Vec, NDArray[np.int32]]:
    hull = ConvexHull(points)
    vertices = points[hull.vertices]
    index = {old: new for new, old in enumerate(hull.vertices)}
    centre = vertices.mean(axis=0)
    faces: list[list[int]] = []
    for simplex in hull.simplices:
        face = [index[i] for i in simplex]
        a, b, c = vertices[face]
        if np.dot(np.cross(b - a, c - a), a - centre) < 0:
            face = [face[0], face[2], face[1]]
        faces.append(face)
    return np.round(vertices, 7), np.array(sorted(faces), dtype=np.int32)


def _mass_properties(
    vertices: Vec, faces: NDArray[np.int32], density: float
) -> tuple[float, Vec, Vec]:
    """Mass, centre of mass and inertia about it of a closed outward-wound triangle mesh."""
    volume = 0.0
    first = np.zeros(3)
    second = np.zeros((3, 3))
    for face in faces:
        a, b, c = vertices[face]
        v = np.dot(a, np.cross(b, c)) / 6.0
        volume += v
        first += v * (a + b + c) / 4.0
        s = a + b + c
        second += v / 20.0 * (np.outer(a, a) + np.outer(b, b) + np.outer(c, c) + np.outer(s, s))
    com = first / volume
    covariance = second - volume * np.outer(com, com)
    inertia = (np.trace(covariance) * np.eye(3) - covariance) * density
    return volume * density, com, inertia


def _link_points(link: ET.Element) -> Vec | None:
    points: list[Vec] = []
    for visual in link.findall("visual"):
        mesh = visual.find("geometry/mesh")
        if mesh is None:
            continue
        origin = visual.find("origin")
        xyz = _floats(origin.get("xyz") if origin is not None else None, (0, 0, 0))
        rpy = _floats(origin.get("rpy") if origin is not None else None, (0, 0, 0))
        scale = _floats(mesh.get("scale"), (1, 1, 1))
        raw = _read_stl(ASSET / mesh.get("filename", ""))
        points.append((raw * scale) @ _rpy(rpy).T + xyz)
    return np.concatenate(points) if points else None


def _fmt(values: Vec) -> str:
    return " ".join(f"{v:.9g}" for v in values)


def render() -> dict[Path, bytes]:
    robot = ET.parse(SOURCE).getroot()
    robot.set("name", "sentient_static_upper_body")
    outputs: dict[Path, bytes] = {}

    for sensor in robot.findall("sensor"):
        robot.remove(sensor)
    for joint in robot.findall("joint"):
        limit = joint.find("limit")
        if joint.get("name") in REVERSED_JOINTS and limit is not None:
            lower, upper = limit.get("lower", ""), limit.get("upper", "")
            limit.set("lower", _negated(upper))
            limit.set("upper", _negated(lower))

    for link in robot.findall("link"):
        name = link.get("name", "")
        for i, visual in enumerate(link.findall("visual")):
            visual.set("name", f"{name}_visual_{i}")
            mesh = visual.find("geometry/mesh")
            if mesh is not None:
                source = mesh.get("filename", "").removeprefix(SOURCE_MESH_PREFIX)
                mesh.set("filename", f"meshes/{source}")
        points = _link_points(link)
        if points is None:
            continue
        vertices, faces = _hull(points)
        mass, com, inertia = _mass_properties(vertices, faces, HULL_DENSITY_KG_M3)
        if link.find("inertial") is not None:
            raise ValueError(f"{name}: the source now carries an inertial; stop estimating it")
        inertial = ET.Element("inertial")
        ET.SubElement(inertial, "origin", {"xyz": _fmt(com), "rpy": "0 0 0"})
        ET.SubElement(inertial, "mass", {"value": f"{mass:.6g}"})
        ET.SubElement(
            inertial,
            "inertia",
            {
                f"i{'xyz'[i]}{'xyz'[j]}": f"{inertia[i, j]:.6g}"
                for i, j in ((0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2))
            },
        )
        link.insert(0, inertial)
        if link.find("collision") is None:
            relpath = f"meshes/collision/{name}.stl"
            stl = ET.Element("collision", {"name": f"{name}_hull"})
            ET.SubElement(stl, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            geometry = ET.SubElement(stl, "geometry")
            ET.SubElement(geometry, "mesh", {"filename": relpath})
            link.append(stl)
            path = ASSET / relpath
            _write_stl(path, vertices, faces)
            outputs[path] = path.read_bytes()

    # The ZED's frame, placed so its left eye's optical frame is the source's, and both eyes.
    if robot.find(f"link[@name='{SOURCE_ZED_EYE}']") is None:
        raise ValueError(f"the source no longer carries {SOURCE_ZED_EYE}")
    left = _matrix(Pose((0.0, eye_offset("left"), 0.0), OPTICAL_RPY))
    ET.SubElement(robot, "link", {"name": ZED_FRAME})
    joint = ET.SubElement(
        robot, "joint", {"name": f"{SOURCE_ZED_EYE}_to_{ZED_FRAME}", "type": "fixed"}
    )
    _set_origin(joint, np.linalg.inv(left))
    ET.SubElement(joint, "parent", {"link": SOURCE_ZED_EYE})
    ET.SubElement(joint, "child", {"link": ZED_FRAME})
    for eye in EYES:
        name = optical_frame(eye)
        ET.SubElement(
            robot,
            "link",
            {
                "name": name,
                "data-preview-kind": "camera",
                "data-preview-label": f"Head camera, {eye}",
                "data-frame-convention": "camera_optical",
            },
        )
        joint = ET.SubElement(robot, "joint", {"name": f"{ZED_FRAME}_to_{name}", "type": "fixed"})
        ET.SubElement(joint, "origin", Pose((0.0, eye_offset(eye), 0.0), OPTICAL_RPY).origin())
        ET.SubElement(joint, "parent", {"link": ZED_FRAME})
        ET.SubElement(joint, "child", {"link": name})

    ET.indent(robot, space="  ")
    xml = '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(robot, encoding="unicode")
    outputs[TARGET] = (xml + "\n").encode()
    return outputs


def main() -> None:
    (ASSET / "meshes/collision").mkdir(parents=True, exist_ok=True)
    for path, data in render().items():
        path.write_bytes(data)
        sys.stdout.write(f"wrote {path.relative_to(ROOT)}\n")


if __name__ == "__main__":
    main()
