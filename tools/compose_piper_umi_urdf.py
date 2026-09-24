#!/usr/bin/env python3
"""Compose the authored Piper UMI hands without changing their mesh geometry.

Like the Yubi composer, this gives each hand its own namespace and floating root.
The authored sources carry the requested camera rotation and opening-positive
slider rebase. These model conventions do not prove physical calibration.
Slider travel allows a nominal 100 mm total opening.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath

DESCRIPTION = Path(__file__).resolve().parents[1] / "assets/piper_umi_description"


class CompositionError(ValueError):
    """The supplied hands cannot form a self-contained robot description."""


def link_name(side: str, name: str) -> str:
    return f"{side}_hand_root" if name == "root" else f"{side}_{name}"


def validate_tree(robot: ET.Element) -> None:
    links = [link.get("name") for link in robot.findall("link")]
    joints = robot.findall("joint")
    names = [joint.get("name") for joint in joints]
    if any(not name for name in (*links, *names)):
        raise CompositionError("links and joints must be named")
    if len(set(links)) != len(links) or len(set(names)) != len(names):
        raise CompositionError("duplicate link or joint names")
    parents: dict[str, str] = {}
    mimics: dict[str, str] = {}
    for joint in joints:
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None:
            raise CompositionError("joint lacks parent or child")
        p, c = parent.get("link", ""), child.get("link", "")
        if p not in links or c not in links or c in parents:
            raise CompositionError("unknown link or multiple parents")
        parents[c] = p
        mimic = joint.find("mimic")
        if mimic is not None and mimic.get("joint") not in names:
            raise CompositionError("unknown mimic joint")
        if mimic is not None:
            mimics[joint.attrib["name"]] = mimic.attrib["joint"]
    for name in mimics:
        visited: set[str] = set()
        current = name
        while current in mimics:
            if current in visited:
                raise CompositionError("cyclic mimic joint dependency")
            visited.add(current)
            current = mimics[current]
    if set(links) - set(parents) != {"quest_origin"}:
        raise CompositionError("quest_origin must be the sole root")
    reached = {"quest_origin"}
    while True:
        discovered = {child for child, parent in parents.items() if parent in reached}
        if discovered <= reached:
            break
        reached.update(discovered)
    if reached != set(links):
        raise CompositionError("disconnected or cyclic link graph")


def render(description: Path = DESCRIPTION) -> bytes:
    robot = ET.Element("robot", {"name": "piper_umi_hands"})
    ET.SubElement(robot, "link", {"name": "quest_origin"})
    for side in ("left", "right"):
        source = description / "source" / f"piper_umi_{side}.urdf"
        hand = ET.parse(source).getroot()
        required = {"root", "camera_link", "vrcontroller"}
        if not required <= {link.get("name") for link in hand.findall("link")}:
            raise CompositionError(f"{side} hand lacks root, camera or controller frame")
        for original in hand:
            element = copy.deepcopy(original)
            if element.tag == "link":
                element.set("name", link_name(side, element.attrib["name"]))
            elif element.tag == "joint":
                if original.attrib["name"] in ("slider_1", "slider_2"):
                    limit = element.find("limit")
                    if limit is None:
                        raise CompositionError("Piper slider lacks travel limits")
                    driver = original.attrib["name"] == "slider_1"
                    limit.set("lower", "0" if driver else "-0.05")
                    limit.set("upper", "0.05" if driver else "0")
                element.set("name", f"{side}_{element.attrib['name']}")
                for relation in ("parent", "child"):
                    frame = element.find(relation)
                    if frame is None or not frame.get("link"):
                        raise CompositionError(f"{side} joint lacks {relation}")
                    frame.set("link", link_name(side, frame.attrib["link"]))
                mimic = element.find("mimic")
                if mimic is not None:
                    mimic.set("joint", f"{side}_{mimic.attrib['joint']}")
            for material in element.iter("material"):
                if material.get("name"):
                    material.set("name", f"{side}_{material.attrib['name']}")
            for mesh in element.iter("mesh"):
                uri = mesh.get("filename", "")
                prefix = "package://umi/meshes/"
                if not uri.startswith(prefix):
                    raise CompositionError(f"unsupported source mesh URI: {uri}")
                relative = PurePosixPath(uri.removeprefix(prefix))
                if relative.is_absolute() or ".." in relative.parts:
                    raise CompositionError(f"unsafe mesh path: {uri}")
                packaged = PurePosixPath("meshes") / side / relative
                if not (description / str(packaged)).is_file():
                    raise CompositionError(f"missing {side} mesh: {relative}")
                mesh.set("filename", f"package://piper_umi_description/{packaged}")
            robot.append(element)
        mount = ET.SubElement(
            robot, "joint", {"name": f"quest_origin_to_{side}", "type": "floating"}
        )
        ET.SubElement(mount, "parent", {"link": "quest_origin"})
        ET.SubElement(mount, "child", {"link": f"{side}_hand_root"})
    validate_tree(robot)
    ET.indent(robot, space="  ")
    return ET.tostring(robot, encoding="utf-8", xml_declaration=True) + b"\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify the committed output")
    args = parser.parse_args()
    output = DESCRIPTION / "urdf/piper_umi_hands.urdf"
    data = render()
    declarations = DESCRIPTION.parents[1] / "sx_embodiments/known/_piper_umi_assets.py"
    entries = [("urdf/piper_umi_hands.urdf", data)]
    entries.extend(
        (str(path.relative_to(DESCRIPTION)), path.read_bytes())
        for path in sorted((DESCRIPTION / "meshes").rglob("*.stl"))
    )
    lines = [
        '"""Generated local bundle identities; upstream CAD URLs remain unspecified."""',
        "",
        "# Regenerate with tools/compose_piper_umi_urdf.py.",
        "ASSET_IDENTITIES: tuple[tuple[str, str, int], ...] = (",
    ]
    for relative, payload in entries:
        lines.extend([
            "    (",
            f'        "{relative}",',
            f'        "{hashlib.sha256(payload).hexdigest()}",',
            f"        {len(payload)},",
            "    ),",
        ])
    lines.extend([")", ""])
    identities = "\n".join(lines).encode()
    if args.check:
        if not output.is_file() or output.read_bytes() != data:
            raise CompositionError("combined Piper UMI URDF needs regeneration")
        if not declarations.is_file() or declarations.read_bytes() != identities:
            raise CompositionError("Piper UMI asset identities need regeneration")
    else:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_bytes(data)
        declarations.write_bytes(identities)


if __name__ == "__main__":
    main()
