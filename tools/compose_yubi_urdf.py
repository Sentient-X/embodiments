#!/usr/bin/env python3
"""Compose the two authoritative YUBI CAD exports into one bimanual URDF.

The source descriptions deliberately stay as the two per-hand Onshape exports. This
generator only namespaces their link/joint/material names, rewrites the closed mesh
bundle, and adds the Quest tracking frames needed by recorded controller poses.
"""

from __future__ import annotations

import copy
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "assets/yubi_description"
TARGET = DESCRIPTION / "urdf/yubi_hands.urdf"
SOURCE = {
    "left": DESCRIPTION / "source/yubi_left_gripper.urdf",
    "right": DESCRIPTION / "source/yubi_right_gripper.urdf",
}

_SHARED_MESHES = {
    "link_base_link.stl": "link_base_link.stl",
    "link_connector_left.stl": "link_connector.stl",
    "link_connector_right.stl": "link_connector.stl",
    "link_left_finger.stl": "link_left_finger.stl",
    "link_left_nail.stl": "link_left_nail.stl",
    "link_right_finger.stl": "link_right_finger.stl",
    "link_right_nail.stl": "link_right_nail.stl",
}
_SIDE_MESHES = {
    "left": {
        "link_controller_left.stl": "link_controller.stl",
        "link_housing_left.stl": "link_housing.stl",
    },
    "right": {
        "link_controller_right.stl": "link_controller.stl",
        "link_housing.stl": "link_housing.stl",
    },
}


def _link_name(side: str, source_name: str) -> str:
    if source_name == "world":
        return f"{side}_hand_root"
    if source_name == "camera_link":
        return f"{side}_hand_cam_optical"
    if source_name == f"controller_{side}":
        return f"{side}_controller_link"
    return f"{side}_{source_name}"


def _mesh_path(side: str, filename: str) -> str:
    basename = PurePosixPath(filename).name
    if basename in _SHARED_MESHES:
        relative = f"shared/{_SHARED_MESHES[basename]}"
    elif basename in _SIDE_MESHES[side]:
        relative = f"{side}/{_SIDE_MESHES[side][basename]}"
    else:
        raise ValueError(f"{side} YUBI source references an unknown mesh {filename!r}")
    target = DESCRIPTION / "meshes" / relative
    if not target.is_file():
        raise ValueError(f"YUBI mesh bundle is missing {target.relative_to(ROOT)}")
    return f"package://yubi_description/meshes/{relative}"


def _canonical_elements(side: str) -> list[ET.Element]:
    source_root = ET.parse(SOURCE[side]).getroot()
    required_links = {"world", "base_link", "camera_link", f"controller_{side}"}
    links = {link.get("name") for link in source_root.findall("link")}
    if not required_links <= links:
        raise ValueError(f"{SOURCE[side].name} lacks links {sorted(required_links - links)}")

    elements = [copy.deepcopy(element) for element in source_root]
    for element in elements:
        if element.tag == "link":
            source_name = element.get("name")
            if source_name is None:
                raise ValueError(f"{SOURCE[side].name} contains an unnamed link")
            element.set("name", _link_name(side, source_name))
            if source_name == "camera_link":
                element.set("data-frame-convention", "camera_optical")
        elif element.tag == "joint":
            source_name = element.get("name")
            if source_name is None:
                raise ValueError(f"{SOURCE[side].name} contains an unnamed joint")
            element.set("name", f"{side}_{source_name}")
            for relation in ("parent", "child"):
                frame = element.find(relation)
                if frame is None or frame.get("link") is None:
                    raise ValueError(f"{source_name!r} lacks its {relation} link")
                frame.set("link", _link_name(side, frame.get("link", "")))
            mimic = element.find("mimic")
            if mimic is not None:
                source_joint = mimic.get("joint")
                if source_joint is None:
                    raise ValueError(f"{source_name!r} has an unnamed mimic source")
                mimic.set("joint", f"{side}_{source_joint}")

        for material in element.iter("material"):
            name = material.get("name")
            if name:
                material.set("name", f"{side}_{name}")
        for mesh in element.iter("mesh"):
            filename = mesh.get("filename")
            if filename is None:
                raise ValueError(f"{SOURCE[side].name} contains a mesh without a filename")
            mesh.set("filename", _mesh_path(side, filename))
    return elements


def _tracking_frame(side: str) -> tuple[ET.Element, ET.Element]:
    link = ET.Element("link", {"name": f"quest_{side}_controller"})
    joint = ET.Element("joint", {"name": f"{side}_controller_tracking_frame", "type": "fixed"})
    # The MCAP tracking pose lands on this Quest frame. In the provided viewer,
    # T_tracking_controller_link = Rz(+90 deg), hence the URDF's inverse
    # controller_link -> tracking-frame relation below.
    ET.SubElement(
        joint,
        "origin",
        {"xyz": "0 0 0", "rpy": "0 0 -1.5707963267948966"},
    )
    ET.SubElement(joint, "parent", {"link": f"{side}_controller_link"})
    ET.SubElement(joint, "child", {"link": f"quest_{side}_controller"})
    return link, joint


def _floating_mount(side: str, y: float) -> ET.Element:
    joint = ET.Element("joint", {"name": f"quest_origin_to_{side}", "type": "floating"})
    ET.SubElement(joint, "origin", {"xyz": f"0 {y:g} 0", "rpy": "0 0 0"})
    ET.SubElement(joint, "parent", {"link": "quest_origin"})
    ET.SubElement(joint, "child", {"link": f"{side}_hand_root"})
    return joint


def _validate_tree(robot: ET.Element) -> None:
    links = [link.get("name") for link in robot.findall("link")]
    joints = [joint.get("name") for joint in robot.findall("joint")]
    if None in links or len(links) != len(set(links)):
        raise ValueError("composed YUBI URDF has unnamed or duplicate links")
    if None in joints or len(joints) != len(set(joints)):
        raise ValueError("composed YUBI URDF has unnamed or duplicate joints")

    parents: dict[str, str] = {}
    for joint in robot.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"joint {joint.get('name')!r} is disconnected")
        parent_name, child_name = parent.get("link"), child.get("link")
        if parent_name is None or child_name is None:
            raise ValueError(f"joint {joint.get('name')!r} has an unnamed link")
        if parent_name not in links or child_name not in links:
            raise ValueError(f"joint {joint.get('name')!r} references an unknown link")
        if child_name in parents:
            raise ValueError(f"link {child_name!r} has more than one parent")
        parents[child_name] = parent_name
    roots = set(links) - set(parents)
    if roots != {"quest_origin"}:
        raise ValueError(f"composed YUBI URDF needs quest_origin as its sole root, got {roots}")

    reached = {"quest_origin"}
    while True:
        discovered = {child for child, parent in parents.items() if parent in reached}
        if discovered <= reached:
            break
        reached.update(discovered)
    if reached != set(links):
        raise ValueError(f"composed YUBI URDF has unreachable links {set(links) - reached}")


def render() -> bytes:
    robot = ET.Element("robot", {"name": "yubi_hands"})
    robot.append(ET.Element("link", {"name": "quest_origin"}))
    for side in ("left", "right"):
        robot.extend(_canonical_elements(side))
        tracking_link, tracking_joint = _tracking_frame(side)
        robot.extend((tracking_link, tracking_joint))
    robot.extend((_floating_mount("left", 0.16), _floating_mount("right", -0.16)))
    _validate_tree(robot)
    ET.indent(robot, space="  ")
    body = ET.tostring(robot, encoding="unicode", short_empty_elements=True)
    header = (
        '<?xml version="1.0" encoding="utf-8"?>\n'
        "<!-- Generated by tools/compose_yubi_urdf.py from the two "
        "assets/yubi_description/source/*.urdf CAD exports. -->\n"
    )
    return (header + body + "\n").encode()


def main() -> None:
    TARGET.write_bytes(render())


if __name__ == "__main__":
    main()
