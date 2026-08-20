"""The one authoring door into :class:`Embodiment`: assemble a body from qualified parts.

``assemble`` is the second construction site the customer-embodiments plan admits, and
it is a door, not an open constructor: the input is an untrusted wire definition naming
registered parts, the output is a complete validated embodiment whose id is derived.
Every refusal is typed and names what the author must fix — the unknown part, the
unbound axis, the joint the description disagrees about. Its only production consumer
is the Experience embodiment door.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Mapping
from functools import cache

from sx_contracts import decode

from .compose import Component, body_component
from .embodiment import (
    Embodiment,
    _parse_assets,  # pyright: ignore[reportPrivateUsage]
    _parse_base_mount,  # pyright: ignore[reportPrivateUsage]
    _parse_component_mount,  # pyright: ignore[reportPrivateUsage]
    _parse_lineage,  # pyright: ignore[reportPrivateUsage]
    _parse_rates,  # pyright: ignore[reportPrivateUsage]
    _validate_urdf,  # pyright: ignore[reportPrivateUsage]
)
from .errors import AssemblyError, EmbodimentSchemaError
from .identity import EmbodimentKind, EmbodimentName, PartId
from .known import embodiments
from .layout import Bounds
from .parts import ArmSpec, GripperSpec, JointGroupSpec, MobileBaseSpec

ComposablePart = ArmSpec | JointGroupSpec | GripperSpec | MobileBaseSpec

_MOVABLE_JOINTS = {"revolute", "prismatic", "continuous"}
_LIMIT_TOLERANCE = 1e-6


@cache
def composable_parts() -> Mapping[PartId, ComposablePart]:
    """Every body part the production bodies are built from, keyed by part id.

    Derived from the production registry, never hand-enumerated: a part exists here
    because a governed production body already uses it (development-tier bodies are
    not customer-composable), and one part id names one part value — two bodies
    disagreeing about the same id is a registry defect, refused loudly.
    """

    parts: dict[PartId, ComposablePart] = {}
    for name in embodiments:
        for component in embodiments[name].components:
            part = component.part
            if not isinstance(part, ComposablePart):
                continue
            stored = parts.get(part.part_id)
            if stored is not None and stored != part:
                raise AssemblyError(
                    str(part.part_id),
                    "two registered bodies disagree about this part's facts",
                )
            parts[part.part_id] = part
    return parts


def assemble(document: Mapping[str, object], *, urdf: bytes) -> Embodiment:
    """Mint one embodiment from an untrusted wire definition, or refuse naming the gap.

    ``urdf`` is the composed description's exact bytes; the definition's assets must
    reference it (sha-verified) as the one DESCRIPTION URDF. Every actuated axis of the
    assembled body must carry a qualified actuator binding — an assembled body is
    drivable by construction or it is not minted.
    """

    entry = decode.exactly(
        document,
        {"name", "label", "kind", "lineage", "attachments", "assets", "rates", "base_mount"},
        where="definition",
        error=EmbodimentSchemaError,
    )
    name = decode.text(entry, "name")
    try:
        kind = EmbodimentKind(decode.text(entry, "kind"))
    except ValueError as exc:
        raise AssemblyError(name, "kind is unknown") from exc
    if kind is not EmbodimentKind.ROBOT:
        raise AssemblyError(name, "the door assembles robots; stations stay first-party")
    parts = composable_parts()
    components: list[Component] = []
    for raw in decode.documents(entry, "attachments"):
        attachment = decode.exactly(raw, {"instance", "part_id", "mount"})
        part_id = PartId(decode.text(attachment, "part_id"))
        part = parts.get(part_id)
        if part is None:
            known = ", ".join(sorted(str(part_id) for part_id in parts))
            raise AssemblyError(
                name,
                f"part {str(part_id)!r} is not in the composable catalog; known parts: {known}",
            )
        components.append(
            body_component(
                decode.text(attachment, "instance"),
                part,
                _parse_component_mount(decode.mapping(attachment, "mount")),
            )
        )
    try:
        embodiment = Embodiment(
            name=EmbodimentName(name),
            label=decode.text(entry, "label"),
            kind=kind,
            lineage=_parse_lineage(decode.mapping(entry, "lineage")),
            components=tuple(components),
            assets=_parse_assets(entry),
            rates=_parse_rates(entry),
            base_mount=_parse_base_mount(entry),
        )
    except EmbodimentSchemaError:
        raise
    except ValueError as exc:
        raise EmbodimentSchemaError(f"invalid definition: {exc}") from exc
    _require_complete_actuation(embodiment)
    _validate_urdf(
        embodiment.name,
        tuple(asset.asset for asset in embodiment.assets),
        urdf,
        components=embodiment.components,
        operator_mounts=embodiment.operator_mounts,
    )
    _validate_joint_parity(embodiment, urdf)
    return embodiment


def _require_complete_actuation(embodiment: Embodiment) -> None:
    for coordinate in embodiment.state.coordinates:
        if coordinate.axis.actuator is None:
            raise AssemblyError(
                str(embodiment.name),
                f"axis {coordinate.instance}/{coordinate.joint_name} carries no actuator"
                " binding; an assembled body is drivable by construction",
            )


def _validate_joint_parity(embodiment: Embodiment, urdf: bytes) -> None:
    """The description asset and the declared parts agree about every actuated joint."""

    root = ET.fromstring(urdf)
    joints: dict[str, ET.Element] = {}
    for joint in root.iter("joint"):
        joint_name = joint.get("name")
        if joint_name and joint.get("type") in _MOVABLE_JOINTS:
            joints[joint_name] = joint
    for coordinate in embodiment.state.coordinates:
        joint = joints.get(coordinate.joint_name)
        if joint is None:
            raise EmbodimentSchemaError(
                f"{embodiment.name}: joint {coordinate.joint_name!r} is not a movable joint"
                " in the description"
            )
        bounds = coordinate.axis.bounds
        if not isinstance(bounds, Bounds):
            continue
        limit = joint.find("limit")
        lower = limit.get("lower") if limit is not None else None
        upper = limit.get("upper") if limit is not None else None
        if lower is None or upper is None:
            raise EmbodimentSchemaError(
                f"{embodiment.name}: joint {coordinate.joint_name!r} declares bounds the"
                " description does not limit"
            )
        if (
            abs(float(lower) - bounds.lower) > _LIMIT_TOLERANCE
            or abs(float(upper) - bounds.upper) > _LIMIT_TOLERANCE
        ):
            raise EmbodimentSchemaError(
                f"{embodiment.name}: joint {coordinate.joint_name!r} limits disagree with"
                f" the declared bounds [{bounds.lower}, {bounds.upper}]"
            )
