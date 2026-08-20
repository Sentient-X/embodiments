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
from math import isfinite

from sx_contracts import decode

from .compose import Component, body_component
from .embodiment import (
    Embodiment,
    _parse_assets,  # pyright: ignore[reportPrivateUsage]
    _parse_base_mount,  # pyright: ignore[reportPrivateUsage]
    _parse_component_mount,  # pyright: ignore[reportPrivateUsage]
    _parse_lineage,  # pyright: ignore[reportPrivateUsage]
    _parse_part,  # pyright: ignore[reportPrivateUsage]
    _parse_rates,  # pyright: ignore[reportPrivateUsage]
    _part_to_dict,  # pyright: ignore[reportPrivateUsage]
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


def part_from_dict(document: Mapping[str, object]) -> ComposablePart:
    """Parse one stored part document through the shared codec, body parts only."""

    part = _parse_part(decode.document(document, where="part", error=EmbodimentSchemaError))
    if not isinstance(part, ComposablePart):
        raise AssemblyError(
            str(part.part_id),
            "only body parts are admissible: arm, joint_group, gripper, mobile_base",
        )
    return part


def part_to_dict(part: ComposablePart) -> dict[str, object]:
    """The canonical wire document of one part — the codec's own rendering."""

    return _part_to_dict(part)


def admit_part(
    document: Mapping[str, object], *, urdf: bytes, mesh_paths: frozenset[str] = frozenset()
) -> ComposablePart:
    """Admit one customer part, or refuse naming exactly what the author must fix.

    The part document is parsed by the shared codec; every actuated axis must carry a
    qualified actuator binding (drivable by construction — the motor restriction);
    and the part's own description fragment must agree with the declared facts: every
    declared joint movable with matching limits, no *undeclared* movable joint (a
    hidden degree of freedom is absent from the composed body's state vector), every
    declared mimic present with its multiplier, every movable joint's child link
    carrying a finite positive-mass inertial (simulation dies on massless — and worse,
    on NaN — bodies, so the door refuses them before hardware), and every mesh
    reference matching a staged path exactly (the closure law — no reference leaves
    the staged asset set, and no basename stands in for a path).
    """

    part = part_from_dict(document)
    subject = str(part.part_id)
    for axis in part.layout.axes:
        if axis.actuator is None:
            raise AssemblyError(
                subject,
                f"axis {axis.name!r} carries no actuator binding; an admitted part is"
                " drivable by construction",
            )
    try:
        root = ET.fromstring(urdf)
    except ET.ParseError as exc:
        raise EmbodimentSchemaError(f"{subject}: part description is invalid XML") from exc
    joints = _movable_joints(root)
    _require_joint_agreement(
        subject,
        tuple((axis.name, axis.bounds) for axis in part.layout.axes),
        joints,
    )
    declared_joints = {axis.name for axis in part.layout.axes}
    if isinstance(part, GripperSpec):
        declared_joints |= {mimic.joint_name for mimic in part.mimic_joints}
    undeclared = sorted(set(joints) - declared_joints)
    if undeclared:
        names = ", ".join(repr(name) for name in undeclared)
        raise EmbodimentSchemaError(
            f"{subject}: the description carries movable joints the part does not"
            f" declare ({names}); a hidden degree of freedom never reaches the"
            " composed body's state vector"
        )
    if isinstance(part, GripperSpec):
        for mimic in part.mimic_joints:
            joint = next((j for j in root.iter("joint") if j.get("name") == mimic.joint_name), None)
            element = joint.find("mimic") if joint is not None else None
            if element is None or element.get("joint") != mimic.of:
                raise EmbodimentSchemaError(
                    f"{subject}: declared mimic {mimic.joint_name!r} following"
                    f" {mimic.of!r} is absent from the description"
                )
            # A mimic's multiplier IS the coupling: a part claiming 100x against a
            # description that says 1x describes different hardware.
            multiplier = _finite(element.get("multiplier"), subject, "mimic multiplier", 1.0)
            if abs(multiplier - mimic.multiplier) > _LIMIT_TOLERANCE:
                raise EmbodimentSchemaError(
                    f"{subject}: mimic {mimic.joint_name!r} declares multiplier"
                    f" {mimic.multiplier} but the description couples it at {multiplier}"
                )
    for joint in joints.values():
        child = joint.find("child")
        child_link = child.get("link") if child is not None else None
        if child_link is None:
            raise EmbodimentSchemaError(
                f"{subject}: movable joint {joint.get('name')!r} declares no child link"
            )
        link = next((el for el in root.iter("link") if el.get("name") == child_link), None)
        mass = link.find("inertial/mass") if link is not None else None
        mass_value = mass.get("value") if mass is not None else None
        if mass_value is None:
            raise EmbodimentSchemaError(
                f"{subject}: link {child_link!r} needs a positive-mass inertial —"
                " simulation refuses massless moving bodies"
            )
        # `_finite` refuses NaN and infinity before the comparison: `nan <= 0.0` is
        # False, so an unguarded compare admits the one mass every solver dies on.
        if _finite(mass_value, subject, f"mass of link {child_link!r}") <= 0.0:
            raise EmbodimentSchemaError(
                f"{subject}: link {child_link!r} needs a positive-mass inertial —"
                " simulation refuses massless moving bodies"
            )
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        if filename is None:
            continue
        # Exact match against a staged logical path, and nothing else: a basename
        # fallback let `package://x/../../../etc/passwd.stl` resolve to a staged
        # `passwd.stl`, which is the closure law's whole point defeated.
        normalized = filename.split("://", 1)[-1].lstrip("/")
        if ".." in normalized.split("/") or normalized not in mesh_paths:
            raise EmbodimentSchemaError(
                f"{subject}: mesh reference {filename!r} does not match a staged asset"
                " path exactly (the closure law)"
            )
    return part


def _finite(value: str | None, subject: str, what: str, default: float | None = None) -> float:
    """Parse one description number, or refuse — typed, never a bare ``ValueError``.

    Every numeric attribute in an untrusted description reaches this: a non-number
    used to escape as ``ValueError`` past the door's ``EmbodimentError`` handler (a
    500), and NaN/infinity used to pass every comparison written against them.
    """

    if value is None:
        if default is not None:
            return default
        raise EmbodimentSchemaError(f"{subject}: {what} is missing")
    try:
        parsed = float(value)
    except ValueError as exc:
        raise EmbodimentSchemaError(f"{subject}: {what} is not a number: {value!r}") from exc
    if not isfinite(parsed):
        raise EmbodimentSchemaError(f"{subject}: {what} must be finite, got {value!r}")
    return parsed


def assemble(
    document: Mapping[str, object],
    *,
    urdf: bytes,
    parts: Mapping[PartId, ComposablePart] | None = None,
) -> Embodiment:
    """Mint one embodiment from an untrusted wire definition, or refuse naming the gap.

    ``urdf`` is the composed description's exact bytes; the definition's assets must
    reference it (sha-verified) as the one DESCRIPTION URDF. Every actuated axis of the
    assembled body must carry a qualified actuator binding — an assembled body is
    drivable by construction or it is not minted. ``parts`` is the resolvable catalog:
    the production parts by default; the door passes the tenant's admitted parts
    merged over them.
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
    if parts is None:
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


def _movable_joints(root: ET.Element) -> dict[str, ET.Element]:
    joints: dict[str, ET.Element] = {}
    for joint in root.iter("joint"):
        joint_name = joint.get("name")
        if joint_name and joint.get("type") in _MOVABLE_JOINTS:
            joints[joint_name] = joint
    return joints


def _require_joint_agreement(
    subject: str,
    declared: tuple[tuple[str, object], ...],
    joints: dict[str, ET.Element],
) -> None:
    for joint_name, bounds in declared:
        joint = joints.get(joint_name)
        if joint is None:
            raise EmbodimentSchemaError(
                f"{subject}: joint {joint_name!r} is not a movable joint in the description"
            )
        if not isinstance(bounds, Bounds):
            continue
        limit = joint.find("limit")
        lower = limit.get("lower") if limit is not None else None
        upper = limit.get("upper") if limit is not None else None
        if lower is None or upper is None:
            raise EmbodimentSchemaError(
                f"{subject}: joint {joint_name!r} declares bounds the description does not limit"
            )
        # Parsed through `_finite`: `abs(nan - x) > tolerance` is False, so a NaN limit
        # would agree with every declared bound rather than disagreeing with all of them.
        if (
            abs(_finite(lower, subject, f"lower limit of joint {joint_name!r}") - bounds.lower)
            > _LIMIT_TOLERANCE
            or abs(_finite(upper, subject, f"upper limit of joint {joint_name!r}") - bounds.upper)
            > _LIMIT_TOLERANCE
        ):
            raise EmbodimentSchemaError(
                f"{subject}: joint {joint_name!r} limits disagree with the declared"
                f" bounds [{bounds.lower}, {bounds.upper}]"
            )


def _validate_joint_parity(embodiment: Embodiment, urdf: bytes) -> None:
    """The description asset and the declared parts agree about every actuated joint."""

    _require_joint_agreement(
        str(embodiment.name),
        tuple(
            (coordinate.joint_name, coordinate.axis.bounds)
            for coordinate in embodiment.state.coordinates
        ),
        _movable_joints(ET.fromstring(urdf)),
    )
