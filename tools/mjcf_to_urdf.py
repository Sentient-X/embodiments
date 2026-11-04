"""Deterministically project a one-joint-per-body MJCF kinematic tree to URDF.

This narrow generator exists for the official YOR description, whose upstream publishes the
complete robot as MJCF and its individual Piper arms as URDF. It refuses MJCF constructs it
cannot preserve instead of guessing geometry or kinematics.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


class UnsupportedMjcfError(ValueError):
    pass


def _numbers(value: str | None, default: str) -> tuple[float, ...]:
    return tuple(float(item) for item in (value or default).split())


_Matrix = tuple[tuple[float, float, float], ...]


@dataclass(frozen=True, slots=True)
class _Angles:
    """The ``<compiler>`` facts that decide what an MJCF ``euler`` attribute means."""

    radians: bool
    sequence: str

    @classmethod
    def of(cls, source: ET.Element) -> _Angles:
        compiler = source.find("compiler")
        angle = "degree" if compiler is None else compiler.get("angle", "degree")
        sequence = "xyz" if compiler is None else compiler.get("eulerseq", "xyz")
        if angle not in {"degree", "radian"}:
            raise UnsupportedMjcfError(f"unknown compiler angle {angle!r}")
        if len(sequence) != 3 or any(axis not in "xyzXYZ" for axis in sequence):
            raise UnsupportedMjcfError(f"unknown compiler eulerseq {sequence!r}")
        return cls(radians=angle == "radian", sequence=sequence)


def _multiply(a: _Matrix, b: _Matrix) -> _Matrix:
    return tuple(
        tuple(sum(a[row][k] * b[k][column] for k in range(3)) for column in range(3))
        for row in range(3)
    )


def _about(axis: str, angle: float) -> _Matrix:
    c, s = math.cos(angle), math.sin(angle)
    if axis == "x":
        return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))
    if axis == "y":
        return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def euler_matrix(angles: tuple[float, ...], sequence: str) -> _Matrix:
    """MuJoCo's ``euler`` rotation: lower-case axes rotate with the frame, upper-case are fixed."""
    rotation: _Matrix = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    for axis, angle in zip(sequence, angles, strict=True):
        step = _about(axis.lower(), angle)
        rotation = _multiply(rotation, step) if axis.islower() else _multiply(step, rotation)
    return rotation


def _quat_matrix(w: float, x: float, y: float, z: float) -> _Matrix:
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = (value / norm for value in (w, x, y, z))
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def urdf_rpy(rotation: _Matrix) -> tuple[float, float, float]:
    """URDF ``rpy``: fixed-axis roll, pitch, yaw with ``R = Rz(yaw) Ry(pitch) Rx(roll)``."""
    sine = max(-1.0, min(1.0, -rotation[2][0]))
    pitch = math.asin(sine)
    if abs(sine) < 1.0 - 1e-12:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:  # gimbal lock: only roll -/+ yaw is defined, so fold it into yaw
        roll = 0.0
        yaw = math.atan2(-rotation[0][1], rotation[1][1])
    return roll, pitch, yaw


def _rpy(quat: str | None, euler: str | None, angles: _Angles) -> str:
    if euler is not None:
        values = _numbers(euler, "0 0 0")
        if len(values) != 3:
            raise UnsupportedMjcfError(f"euler needs three angles, got {euler!r}")
        if not angles.radians:
            values = tuple(math.radians(value) for value in values)
        rotation = euler_matrix(values, angles.sequence)
    else:
        w, x, y, z = _numbers(quat, "1 0 0 0")
        rotation = _quat_matrix(w, x, y, z)
    roll, pitch, yaw = urdf_rpy(rotation)
    # Adding 0.0 turns a negative zero into the "0" the URDF has always carried.
    return " ".join(f"{angle + 0.0:.12g}" for angle in (roll, pitch, yaw))


def _origin(parent: ET.Element, source: ET.Element, angles: _Angles) -> None:
    for unsupported in ("axisangle", "xyaxes", "zaxis"):
        if source.get(unsupported) is not None:
            raise UnsupportedMjcfError(
                f"{source.tag}: orientation by {unsupported} is not supported"
            )
    ET.SubElement(
        parent,
        "origin",
        {
            "xyz": source.get("pos", "0 0 0"),
            "rpy": _rpy(source.get("quat"), source.get("euler"), angles),
        },
    )


def _link(
    root: ET.Element,
    body: ET.Element,
    meshes: dict[str, tuple[str, str | None]],
    materials: dict[str, str],
    angles: _Angles,
) -> None:
    name = body.get("name")
    if name is None:
        raise UnsupportedMjcfError("every YOR body must have a name")
    link = ET.SubElement(root, "link", {"name": name})
    inertial = body.find("inertial")
    if inertial is not None:
        target = ET.SubElement(link, "inertial")
        _origin(target, inertial, angles)
        ET.SubElement(target, "mass", {"value": inertial.get("mass", "0")})
        diagonal = _numbers(inertial.get("diaginertia"), "0 0 0")
        ET.SubElement(
            target,
            "inertia",
            {
                "ixx": str(diagonal[0]),
                "iyy": str(diagonal[1]),
                "izz": str(diagonal[2]),
                "ixy": "0",
                "ixz": "0",
                "iyz": "0",
            },
        )
    for geom in body.findall("geom"):
        mesh_name = geom.get("mesh")
        if geom.get("class") != "visual" or mesh_name is None:
            continue
        if mesh_name not in meshes:
            raise UnsupportedMjcfError(f"unknown mesh {mesh_name!r}")
        visual = ET.SubElement(link, "visual")
        _origin(visual, geom, angles)
        geometry = ET.SubElement(visual, "geometry")
        filename, scale = meshes[mesh_name]
        attributes = {"filename": f"package://sx-embodiments/yor/{filename}"}
        if scale is not None:
            attributes["scale"] = scale
        ET.SubElement(geometry, "mesh", attributes)
        material_name = geom.get("material")
        if material_name is not None and material_name in materials:
            material = ET.SubElement(visual, "material", {"name": material_name})
            ET.SubElement(material, "color", {"rgba": materials[material_name]})


def _joint(root: ET.Element, parent: str, body: ET.Element, angles: _Angles) -> None:
    child = body.get("name")
    if child is None:
        raise UnsupportedMjcfError("every YOR body must have a name")
    joints = body.findall("joint")
    if len(joints) > 1:
        raise UnsupportedMjcfError(f"{child}: multiple joints per body are not supported")
    if not joints:
        name = f"{parent}_to_{child}"
        joint = ET.SubElement(root, "joint", {"name": name, "type": "fixed"})
    else:
        source = joints[0]
        name = source.get("name")
        if name is None:
            raise UnsupportedMjcfError(f"{child}: joint must have a name")
        kind = source.get("type", "hinge")
        limits = source.get("range")
        if kind == "slide":
            urdf_kind = "prismatic"
        elif kind == "hinge" and limits is not None:
            urdf_kind = "revolute"
        elif kind == "hinge":
            urdf_kind = "continuous"
        else:
            raise UnsupportedMjcfError(f"{name}: unsupported joint type {kind!r}")
        joint = ET.SubElement(root, "joint", {"name": name, "type": urdf_kind})
        ET.SubElement(joint, "axis", {"xyz": source.get("axis", "0 0 1")})
        if limits is not None:
            lower, upper = limits.split()
            ET.SubElement(
                joint,
                "limit",
                {"lower": lower, "upper": upper, "effort": "100", "velocity": "10"},
            )
    _origin(joint, body, angles)
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})


def convert(path: Path) -> ET.Element:
    source = ET.parse(path).getroot()
    asset = source.find("asset")
    worldbody = source.find("worldbody")
    if asset is None or worldbody is None:
        raise UnsupportedMjcfError("MJCF needs asset and worldbody sections")
    meshes = {
        mesh.attrib["name"]: (mesh.attrib["file"], mesh.get("scale"))
        for mesh in asset.findall("mesh")
    }
    materials = {
        material.attrib["name"]: material.attrib["rgba"]
        for material in asset.findall("material")
        if "rgba" in material.attrib
    }
    angles = _Angles.of(source)
    root = ET.Element("robot", {"name": source.get("model", "yor")})

    def visit(body: ET.Element, parent: str) -> None:
        name = body.get("name")
        if name is None:
            raise UnsupportedMjcfError("every YOR body must have a name")
        if body.find("freejoint") is not None:
            raise UnsupportedMjcfError("only the root body may carry a freejoint")
        else:
            _joint(root, parent, body, angles)
        _link(root, body, meshes, materials, angles)
        for child in body.findall("body"):
            visit(child, name)

    bodies = worldbody.findall("body")
    if len(bodies) != 1:
        raise UnsupportedMjcfError("YOR MJCF must have exactly one root body")
    root_body = bodies[0]
    root_name = root_body.get("name")
    if root_name is None:
        raise UnsupportedMjcfError("the YOR root body must have a name")
    ET.SubElement(root, "link", {"name": "yor_origin"})
    root_joint = ET.SubElement(
        root,
        "joint",
        {"name": f"yor_origin_to_{root_name}", "type": "fixed"},
    )
    _origin(root_joint, root_body, angles)
    ET.SubElement(root_joint, "parent", {"link": "yor_origin"})
    ET.SubElement(root_joint, "child", {"link": root_name})
    _link(root, root_body, meshes, materials, angles)
    for child in root_body.findall("body"):
        visit(child, root_name)
    ET.indent(root, space="  ")
    return root


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mjcf", type=Path)
    args = parser.parse_args()
    root = convert(args.mjcf)
    sys.stdout.buffer.write(ET.tostring(root, encoding="utf-8", xml_declaration=True))
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
