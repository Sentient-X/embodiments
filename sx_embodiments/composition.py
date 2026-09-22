"""Compose complete bodies without a special case for pairs or robot families."""

from __future__ import annotations

import copy
import math
import re
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass, replace

from sx_contracts import AssetFormat, AssetRole, ProvenancedAsset

from .assets import description_asset_uri, generated_description
from .compose import Component, MountedOn, OperatorMount, RootMount
from .description import joint_names
from .embodiment import Embodiment
from .errors import CompositionError
from .identity import EmbodimentKind, EmbodimentName, Lineage


@dataclass(frozen=True, slots=True)
class PlacedEmbodiment:
    """A complete body placed in the composite root frame, metres and radians."""

    embodiment: Embodiment
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if any(
            len(vector) != 3 or not all(map(math.isfinite, vector))
            for vector in (self.xyz, self.rpy)
        ):
            raise CompositionError("placement", "xyz and rpy must each contain three finite values")


def compose_embodiments(
    name: str,
    members: Mapping[str, PlacedEmbodiment],
    *,
    label: str,
    kind: EmbodimentKind = EmbodimentKind.ROBOT,
) -> Embodiment:
    """Namespace and place any number of bodies, preserving optics and joint facts.

    Member order is state order. The resulting body is a normal schema-14 Embodiment;
    generated URDF bytes are retained in the digest-checked asset cache. This operation
    declares geometry only: it neither commissions devices nor grants motion authority.
    """
    if not members:
        raise CompositionError(name, "composition needs at least one member")
    robot = ET.Element("robot", {"name": name})
    ET.SubElement(robot, "link", {"name": "assembly"})
    components: list[Component] = []
    assets: dict[tuple[str, str], ProvenancedAsset] = {}
    operator_mounts: list[OperatorMount] = []
    for namespace, placed in members.items():
        if re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", namespace) is None:
            raise CompositionError(name, f"invalid member namespace {namespace!r}")
        body = placed.embodiment
        prefix = namespace + "/"
        source = ET.fromstring(body.urdf_bytes)
        links = {link.attrib["name"] for link in source.findall("link")}
        children = {child.attrib["link"] for child in source.findall("joint/child")}
        roots = links - children
        if len(roots) != 1:
            raise CompositionError(
                name, f"{namespace}: description needs one root, got {sorted(roots)}"
            )
        for element in source:
            item = copy.deepcopy(element)
            for node in item.iter():
                if (
                    node.tag in {"link", "joint", "material", "transmission", "actuator"}
                    and "name" in node.attrib
                ):
                    node.set("name", prefix + node.attrib["name"])
                if node.tag in {"parent", "child"} and "link" in node.attrib:
                    node.set("link", prefix + node.attrib["link"])
                if node.tag == "mimic" and "joint" in node.attrib:
                    node.set("joint", prefix + node.attrib["joint"])
                if node.tag == "gazebo" and "reference" in node.attrib:
                    node.set("reference", prefix + node.attrib["reference"])
                if node.tag in {"mesh", "texture"} and "filename" in node.attrib:
                    filename = node.attrib["filename"]
                    node.set("filename", description_asset_uri(body.urdf.uri, filename))
            robot.append(item)
        joint = ET.SubElement(robot, "joint", {"name": namespace + "/mount", "type": "fixed"})
        ET.SubElement(joint, "parent", {"link": "assembly"})
        ET.SubElement(joint, "child", {"link": prefix + next(iter(roots))})
        ET.SubElement(
            joint,
            "origin",
            {
                "xyz": " ".join(map(str, placed.xyz)),
                "rpy": " ".join(map(str, placed.rpy)),
            },
        )
        components.extend(_prefix_component(component, prefix) for component in body.components)
        operator_mounts.extend(
            replace(
                mount,
                root_frame=prefix + mount.root_frame,
                attachment_frame=prefix + mount.attachment_frame,
            )
            for mount in body.operator_mounts
        )
        for asset in body.assets:
            retained = (
                replace(asset, asset=replace(asset.asset, role=AssetRole.OTHER))
                if (
                    asset.asset.format is AssetFormat.URDF
                    and asset.asset.role is AssetRole.DESCRIPTION
                )
                else asset
            )
            assets[(retained.uri, str(retained.sha256))] = retained
    urdf = ET.tostring(robot, encoding="utf-8", xml_declaration=True)
    description = generated_description(urdf, tuple(assets.values()))
    composed = Embodiment(
        name=EmbodimentName(name),
        label=label,
        kind=kind,
        lineage=Lineage(family="composition"),
        components=tuple(components),
        assets=(*assets.values(), description),
        operator_mounts=tuple(operator_mounts),
    ).with_assets((*assets.values(), description), urdf=urdf)
    joint_names(composed.components, composed.state, urdf)
    return composed


def _prefix_component(component: Component, prefix: str) -> Component:
    # Parts retain local coordinates and product identity. Only instance names and
    # physical frames are namespaced; description.joint_names binds them to the URDF.
    mount = component.mount
    return replace(
        component,
        instance=prefix.replace("/", ".") + component.instance,
        mount=(
            MountedOn(prefix.replace("/", ".") + mount.parent, prefix + mount.frame)
            if isinstance(mount, MountedOn)
            else RootMount(prefix + mount.frame)
        ),
    )
