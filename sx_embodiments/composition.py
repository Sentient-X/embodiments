"""Compose complete bodies without a special case for pairs or robot families."""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass, replace
from functools import partial
from xml.etree import ElementTree as ET

from sx_contracts import AssetFormat, AssetRole, ProvenancedAsset

from .assembly import Assembly, Member, Pose
from .assets import PackagedAsset, description_asset_uri, generated_description
from .compose import Component, OperatorMount
from .description import joint_names
from .embodiment import Embodiment, portable_component
from .errors import CompositionError
from .identity import EmbodimentKind, EmbodimentName, Lineage
from .mechanical import Build, Module, carrier, source
from .parts import ChainSpec, GripperSpec, MobileBaseSpec


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
    rendered = ET.fromstring(urdf)
    parents = {
        child.attrib["link"]: (parent.attrib["link"], joint.attrib["name"])
        for joint in rendered.findall("joint")
        if (child := joint.find("child")) is not None
        and (parent := joint.find("parent")) is not None
    }
    # Which component's motion each rendered joint is: its state coordinates, and the mimic
    # joints that follow one of them.
    owners = {
        assembly.prefix(namespace) + joint: f"{namespace}.{coordinate.instance}"
        for namespace, placed in members.items()
        for coordinate, joint in zip(
            placed.embodiment.state.coordinates, placed.embodiment.joint_names, strict=True
        )
    }
    for joint in rendered.findall("joint"):
        mimic = joint.find("mimic")
        if mimic is not None and mimic.get("joint") in owners:
            owners[joint.attrib["name"]] = owners[mimic.attrib["joint"]]
    components: list[Component] = []
    assets: dict[tuple[str, str], ProvenancedAsset] = {}
    operator_mounts: list[OperatorMount] = []
    for namespace, placed in members.items():
        body = placed.embodiment
        prefix = assembly.prefix(namespace)
        rides = None if placed.parent is None else carrier(placed.parent, parents, owners)
        components.extend(
            assembly.components_of(
                namespace, body.components, instance_separator=".", carrier=rides
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


def compose_build(
    build: Build,
    *,
    name: str,
    label: str,
    kind: EmbodimentKind = EmbodimentKind.ROBOT,
    sources: tuple[ProvenancedAsset, ...] = (),
) -> Embodiment:
    """A body made of modules at runtime: a gripper on another robot's flange, a camera on a
    wrist, a neck in a head's place, as one normal Embodiment.

    The build renders with each module's mesh references resolved to its packaged source, and
    the generated description is retained like :func:`compose_embodiments`' with every source's
    licence. Every interface check is the build's own; this declares geometry only.
    ``sources`` are the files of modules authored outside the registry (their description
    bytes name their meshes by Catalog location), retained beside the packaged ones.
    """
    if build.root is None:
        raise CompositionError(name, "a runtime composition is a rooted build")
    urdf = build.assembly(mesh_uri=_mesh_uri).urdf()
    components = build.components()
    retained_sources: dict[tuple[str, str], ProvenancedAsset] = {}
    packaged = [
        asset
        for component in components
        if isinstance(component.part, ChainSpec | GripperSpec | MobileBaseSpec)
        for asset in component.part.assets
    ]
    for entry in (*build.placed, *build.mates):
        file = source(entry.module.description)
        if isinstance(file, PackagedAsset):
            packaged.append(file)
    for provenanced in (*(asset.provenanced_asset() for asset in packaged), *sources):
        retained = replace(provenanced, asset=replace(provenanced.asset, role=AssetRole.OTHER))
        retained_sources[(retained.uri, str(retained.sha256))] = retained
    description = generated_description(urdf, tuple(retained_sources.values()))
    assets = (*retained_sources.values(), description)
    composed = Embodiment(
        name=EmbodimentName(name),
        label=label,
        kind=kind,
        lineage=Lineage(family="composition"),
        components=tuple(portable_component(component) for component in components),
        assets=assets,
    ).with_assets(assets, urdf=urdf)
    joint_names(composed.components, composed.state, urdf)
    return composed


def _mesh_uri(module: Module) -> Callable[[str], str] | None:
    file = source(module.description)
    if not isinstance(file, PackagedAsset):
        return None
    return partial(description_asset_uri, file.provenanced_asset().uri)
