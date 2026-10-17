#!/usr/bin/env python3
"""Render the registry's generated URDFs: every rooted build, and Piper's canonical form.

A body built from several modules declares its :class:`~sx_embodiments.mechanical.Build`
beside its components in ``sx_embodiments/known``; this writes each rendering to the path of
the body's description asset. ``tests/test_assembly.py`` fails when a committed file differs
from its declaration, so run this after changing one and update the asset's digest.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from sx_embodiments.assets import AssetFormat, AssetRole
from sx_embodiments.known import definitions

ROOT = Path(__file__).resolve().parents[1]


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
    _piper_description(
        ROOT / "assets/official/agilex_piper/piper_description.urdf",
        ROOT / "assets/official/agilex_piper/piper_canonical.urdf",
    )
    for definition in definitions():
        if definition.build.root is None:
            continue
        (description,) = (
            asset
            for asset in definition.extra_assets
            if asset.role is AssetRole.DESCRIPTION and asset.format is AssetFormat.URDF
        )
        (ROOT / "assets" / description.relpath).write_bytes(definition.build.urdf())


if __name__ == "__main__":
    render()
