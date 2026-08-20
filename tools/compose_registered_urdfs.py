#!/usr/bin/env python3
"""Render the complete multi-device URDFs in the built-in registry.

The vendor files describe one device. These deterministic projections preserve every
vendor link, joint, surface, and limit while prefixing names and placing the independent
roots in a useful nominal inspection pose. Runtime calibration remains authoritative for
the actual base-to-base transforms.
"""

from __future__ import annotations

import copy
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Instance:
    name: str
    xyz: str
    rpy: str = "0 0 0"
    joint_type: str = "fixed"


def _source_root(source: ET.Element) -> str:
    links = {link.get("name") for link in source.findall("link")}
    children = {
        child.get("link")
        for joint in source.findall("joint")
        if (child := joint.find("child")) is not None
    }
    roots = links - children
    if len(roots) != 1:
        raise ValueError(f"source URDF needs exactly one root link, got {sorted(roots)}")
    root = roots.pop()
    if root is None:
        raise ValueError("source URDF root link has no name")
    return root


def _prefixed_children(source: ET.Element, prefix: str) -> list[ET.Element]:
    output: list[ET.Element] = []
    for child in source:
        item = copy.deepcopy(child)
        for element in item.iter():
            if element.tag in {"link", "joint", "transmission", "actuator", "material"}:
                name = element.get("name")
                if name:
                    element.set("name", f"{prefix}{name}")
            if element.tag in {"parent", "child"}:
                link = element.get("link")
                if link:
                    element.set("link", f"{prefix}{link}")
            if element.tag == "mimic":
                joint = element.get("joint")
                if joint:
                    element.set("joint", f"{prefix}{joint}")
            if element.tag == "gazebo":
                reference = element.get("reference")
                if reference:
                    element.set("reference", f"{prefix}{reference}")
        output.append(item)
    return output


def _mount(
    robot: ET.Element,
    *,
    parent: str,
    child: str,
    name: str,
    xyz: str,
    rpy: str = "0 0 0",
    joint_type: str = "fixed",
) -> None:
    joint = ET.SubElement(robot, "joint", {"name": name, "type": joint_type})
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})


def _compose(
    source_path: Path,
    target_path: Path,
    *,
    robot_name: str,
    root_name: str,
    instances: tuple[Instance, ...],
) -> ET.Element:
    source = ET.parse(source_path).getroot()
    source_root = _source_root(source)
    robot = ET.Element("robot", {"name": robot_name})
    robot.append(
        ET.Comment(
            " Independent device roots use a nominal inspection layout; measured station "
            "extrinsics remain recording-time facts. "
        )
    )
    ET.SubElement(robot, "link", {"name": root_name})
    for instance in instances:
        prefix = f"{instance.name}_"
        robot.extend(_prefixed_children(source, prefix))
        _mount(
            robot,
            parent=root_name,
            child=f"{prefix}{source_root}",
            name=f"{root_name}_to_{instance.name}",
            xyz=instance.xyz,
            rpy=instance.rpy,
            joint_type=instance.joint_type,
        )
    ET.indent(robot, space="  ")
    target_path.write_bytes(ET.tostring(robot, encoding="utf-8", xml_declaration=True) + b"\n")
    return robot


def _frame(
    robot: ET.Element,
    *,
    parent: str,
    name: str,
    xyz: str = "0 0 0",
    rpy: str = "0 0 0",
    joint_type: str = "fixed",
    preview_kind: str | None = None,
    preview_label: str | None = None,
) -> None:
    attributes = {"name": name}
    if preview_kind is not None:
        attributes["data-preview-kind"] = preview_kind
    if preview_label is not None:
        attributes["data-preview-label"] = preview_label
    ET.SubElement(robot, "link", attributes)
    _mount(
        robot,
        parent=parent,
        child=name,
        name=f"{parent}_to_{name}",
        xyz=xyz,
        rpy=rpy,
        joint_type=joint_type,
    )


def _piper_description(source_path: Path, target_path: Path) -> None:
    """Project the MJCF's declared finger equality into the canonical URDF."""
    robot = ET.parse(source_path).getroot()
    joint8 = next(
        (joint for joint in robot.findall("joint") if joint.get("name") == "joint8"),
        None,
    )
    if joint8 is None:
        raise ValueError("Piper source URDF has no joint8")
    existing = joint8.find("mimic")
    if existing is not None:
        joint8.remove(existing)
    ET.SubElement(
        joint8,
        "mimic",
        {"joint": "joint7", "multiplier": "-1", "offset": "0"},
    )
    ET.indent(robot, space="  ")
    target_path.write_bytes(ET.tostring(robot, encoding="utf-8", xml_declaration=True) + b"\n")


def render() -> None:
    _compose(
        ROOT / "assets/so101/so101.urdf",
        ROOT / "assets/so101/bimanual_so101.urdf",
        robot_name="bimanual_so101",
        root_name="bimanual_origin",
        instances=(Instance("left", "0 0.25 0"), Instance("right", "0 -0.25 0")),
    )

    b601_source = ROOT / "assets/b601_dm/reBot_B601_DM_with_gripper.urdf"
    _compose(
        b601_source,
        ROOT / "assets/b601_dm/bimanual_B601_DM.urdf",
        robot_name="bimanual_rebot_b601_dm",
        root_name="bimanual_origin",
        instances=(Instance("left", "0 0.5 0"), Instance("right", "0 -0.5 0")),
    )
    station = _compose(
        b601_source,
        ROOT / "assets/b601_dm/B601_DM_station.urdf",
        robot_name="rebot_b601_dm_station",
        root_name="station_origin",
        instances=(Instance("left", "0 0.5 0"), Instance("right", "0 -0.5 0")),
    )
    _frame(
        station,
        parent="station_origin",
        name="left_leader",
        xyz="-0.55 0.5 0",
        joint_type="floating",
        preview_kind="leader",
        preview_label="Left leader · surface unavailable",
    )
    _frame(
        station,
        parent="station_origin",
        name="right_leader",
        xyz="-0.55 -0.5 0",
        joint_type="floating",
        preview_kind="leader",
        preview_label="Right leader · surface unavailable",
    )
    _frame(
        station,
        parent="left_link6",
        name="left_wrist_camera",
        preview_kind="camera",
        preview_label="Left wrist camera",
    )
    _frame(
        station,
        parent="right_link6",
        name="right_wrist_camera",
        preview_kind="camera",
        preview_label="Right wrist camera",
    )
    _frame(
        station,
        parent="station_origin",
        name="top_camera",
        xyz="-0.4 0 1.2",
        rpy="0 0.65 0",
        joint_type="floating",
        preview_kind="camera",
        preview_label="Top camera",
    )
    ET.indent(station, space="  ")
    (ROOT / "assets/b601_dm/B601_DM_station.urdf").write_bytes(
        ET.tostring(station, encoding="utf-8", xml_declaration=True) + b"\n"
    )

    piper_upstream = ROOT / "assets/official/agilex_piper/piper_description.urdf"
    piper_source = ROOT / "assets/official/agilex_piper/piper_canonical.urdf"
    _piper_description(piper_upstream, piper_source)
    piper_station = _compose(
        piper_source,
        ROOT / "assets/official/agilex_piper/piper_station.urdf",
        robot_name="piper_station",
        root_name="station_origin",
        instances=(Instance("leader", "0 0.42 0"), Instance("follower", "0 -0.42 0")),
    )
    _frame(
        piper_station,
        parent="station_origin",
        name="front_camera",
        xyz="-0.35 0 0.85",
        rpy="0 0.65 0",
        joint_type="floating",
        preview_kind="camera",
        preview_label="Front camera",
    )
    ET.indent(piper_station, space="  ")
    (ROOT / "assets/official/agilex_piper/piper_station.urdf").write_bytes(
        ET.tostring(piper_station, encoding="utf-8", xml_declaration=True) + b"\n"
    )


if __name__ == "__main__":
    render()
