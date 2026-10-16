"""Compose complete bodies without a special case for pairs or robot families."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, replace
from functools import partial
from xml.etree import ElementTree as ET

from sx_contracts import AssetFormat, AssetRole, ProvenancedAsset

from .assembly import Assembly, Member, Pose
from .assets import description_asset_uri, generated_description
from .compose import Component, OperatorMount
from .description import joint_names
from .embodiment import Embodiment
from .errors import CompositionError
from .identity import EmbodimentKind, EmbodimentName, Lineage


@dataclass(frozen=True, slots=True)
class PlacedEmbodiment:
    """A complete body placed on the composite root or a member's link, metres and radians."""

    embodiment: Embodiment
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    #: A namespaced link of an earlier member ("torso/head_link"); the root when None.
    parent: str | None = None

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

    Member order is state order. A member mounts on the composite root or, through
    PlacedEmbodiment.parent, on any link of a member placed before it (a camera head on
    a humanoid's neck). The resulting body is a normal schema-14 Embodiment; generated URDF
    bytes are retained in the digest-checked asset cache. This operation declares geometry
    only: it neither commissions devices nor grants motion authority.
    """
    if not members:
        raise CompositionError(name, "composition needs at least one member")
    assembly = Assembly(
        robot=name,
        root="assembly",
        separator="/",
        members=tuple(
            Member(
                namespace,
                placed.embodiment.urdf_bytes,
                Pose(placed.xyz, placed.rpy),
                parent=placed.parent,
                mesh_uri=partial(description_asset_uri, placed.embodiment.urdf.uri),
            )
            for namespace, placed in members.items()
        ),
    )
    urdf = assembly.urdf()
    parents = {
        child.attrib["link"]: parent.attrib["link"]
        for joint in ET.fromstring(urdf).findall("joint")
        if (child := joint.find("child")) is not None
        and (parent := joint.find("parent")) is not None
    }
    components: list[Component] = []
    assets: dict[tuple[str, str], ProvenancedAsset] = {}
    operator_mounts: list[OperatorMount] = []
    for namespace, placed in members.items():
        body = placed.embodiment
        prefix = assembly.prefix(namespace)
        carrier = (
            None if placed.parent is None else _carrier(name, placed.parent, parents, components)
        )
        components.extend(
            assembly.components_of(
                namespace, body.components, instance_separator=".", carrier=carrier
            )
        )
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


def _carrier(
    name: str, link: str, parents: Mapping[str, str], components: list[Component]
) -> str | None:
    """The component whose joints move ``link``: the nearest strict ancestor mount frame.

    ``None`` when no component's joints lie above the link (it hangs off a fixed root).
    """
    current = link
    while current in parents:
        current = parents[current]
        owners = [c.instance for c in components if c.mount.frame == current]
        if len(owners) > 1:
            raise CompositionError(
                name, f"{link!r} could ride on any of {owners}; mount on a link below one"
            )
        if owners:
            return owners[0]
    return None
