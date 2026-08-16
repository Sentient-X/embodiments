#!/usr/bin/env python3
"""Build the canonical bimanual DAS/UMI URDF from the pinned vendor jaw URDF."""

from __future__ import annotations

import copy
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets/das_gripper_with_vr/urdf/DAS_Gripper_urdf.urdf"
TARGET = ROOT / "assets/das_gripper_with_vr/urdf/DAS_UMI_V4.urdf"


def _prefix_tree(source: ET.Element, side: str) -> list[ET.Element]:
    prefix = f"{side}_"
    output: list[ET.Element] = []
    for child in source:
        item = copy.deepcopy(child)
        if item.tag in {"link", "joint", "transmission", "gazebo"}:
            for key in ("name", "reference"):
                value = item.get(key)
                if value:
                    item.set(key, f"{prefix}{value}")
        for element in item.iter():
            if element.tag in {"parent", "child"} and element.get("link"):
                element.set("link", f"{prefix}{element.get('link')}")
            elif element.tag == "mimic" and element.get("joint"):
                element.set("joint", f"{prefix}{element.get('joint')}")
            elif element.tag == "joint" and element is not item and element.get("name"):
                element.set("name", f"{prefix}{element.get('name')}")
        output.append(item)
    return output


def render() -> bytes:
    source = ET.parse(SOURCE).getroot()
    robot = ET.Element("robot", {"name": "das_umi_v4"})
    head = ET.SubElement(robot, "link", {"name": "quest3s_head"})
    visual = ET.SubElement(head, "visual")
    ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "1.5707 0 3.14159"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geometry,
        "mesh",
        {"filename": "package://sx-embodiments/quest_ego/meshes/quest3mesh.obj"},
    )
    material = ET.SubElement(visual, "material", {"name": "quest_shell"})
    ET.SubElement(material, "color", {"rgba": "0.78 0.79 0.82 1"})
    for side, frame, xyz in (
        ("left", "quest3s_left_camera_optical", "-0.032 0.075 -0.011"),
        ("right", "quest3s_right_camera_optical", "0.032 0.075 -0.011"),
    ):
        ET.SubElement(
            robot,
            "link",
            {"name": frame, "data-frame-convention": "camera_optical"},
        )
        head_camera = ET.SubElement(
            robot,
            "joint",
            {"name": f"quest3s_head_to_{side}_camera_optical", "type": "fixed"},
        )
        ET.SubElement(
            head_camera,
            "origin",
            {"xyz": xyz, "rpy": "-1.5707963267948966 0 0"},
        )
        ET.SubElement(head_camera, "parent", {"link": "quest3s_head"})
        ET.SubElement(head_camera, "child", {"link": frame})
    nominal_origins = {"left": "0 0.16 0", "right": "0 -0.16 0"}
    for side in ("left", "right"):
        robot.extend(_prefix_tree(source, side))
        mount = ET.SubElement(
            robot,
            "joint",
            {"name": f"head_to_{side}_rig", "type": "floating"},
        )
        ET.SubElement(
            mount,
            "origin",
            {"xyz": nominal_origins[side], "rpy": "0 0 0"},
        )
        ET.SubElement(mount, "parent", {"link": "quest3s_head"})
        ET.SubElement(mount, "child", {"link": f"{side}_world"})
    ET.indent(robot, space="  ")
    return ET.tostring(robot, encoding="utf-8", xml_declaration=True) + b"\n"


def main() -> None:
    TARGET.write_bytes(render())


if __name__ == "__main__":
    main()
