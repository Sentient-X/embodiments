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
from dataclasses import replace
from functools import cache
from math import isfinite

from sx_contracts import decode

from .assembly import Pose
from .compose import Component, RootMount, body_component, sensor_component
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
from .errors import AssemblyError, EmbodimentSchemaError, SocketError
from .identity import EmbodimentKind, EmbodimentName, PartId
from .known import embodiments
from .layout import Bounds, UndocumentedDrive
from .mechanical import MECHANICAL_INTERFACES, InterfaceId, Module, ModuleId, Socket, carrier
from .parts import CameraSpec, ChainSpec, ForceTorqueSpec, GripperSpec, Limb, MobileBaseSpec

ComposablePart = ChainSpec | GripperSpec | MobileBaseSpec | CameraSpec | ForceTorqueSpec

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
    """Parse one stored part document through the shared codec, body and calibrated sensor parts."""

    part = _parse_part(decode.document(document, where="part", error=EmbodimentSchemaError))
    if not isinstance(part, ComposablePart):
        raise AssemblyError(
            str(part.part_id),
            "admissible parts are chain, gripper, mobile_base, camera, force_torque",
        )
    return part


def part_to_dict(part: ComposablePart) -> dict[str, object]:
    """The canonical wire document of one part — the codec's own rendering."""

    return _part_to_dict(part)


def admit_part(
    document: Mapping[str, object], *, urdf: bytes, mesh_paths: frozenset[str] = frozenset()
) -> ComposablePart:
    """Admit one customer part, or refuse naming exactly what the author must fix.

    The part document is parsed by the shared codec; every axis must explicitly state
    its drive relation, which may be direct, integrated, or passive but not undocumented;
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
    axes = part.layout.axes if isinstance(part, (ChainSpec, GripperSpec, MobileBaseSpec)) else ()
    for axis in axes:
        if isinstance(axis.actuation, UndocumentedDrive):
            raise AssemblyError(
                subject,
                f"axis {axis.name!r} has undocumented drive facts; declare direct,"
                " integrated, or passive actuation",
            )
    try:
        root = ET.fromstring(urdf)
    except ET.ParseError as exc:
        raise EmbodimentSchemaError(f"{subject}: part description is invalid XML") from exc
    joints = _movable_joints(root)
    _require_joint_agreement(
        subject,
        tuple((axis.name, axis.bounds) for axis in axes),
        joints,
    )
    declared_joints = {axis.name for axis in axes}
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
    _require_mesh_closure(subject, root, mesh_paths)
    return part


def _require_mesh_closure(subject: str, root: ET.Element, mesh_paths: frozenset[str]) -> None:
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


def admit_module(
    module_id: str,
    label: str,
    *,
    urdf: bytes,
    plug: InterfaceId | None,
    sockets: tuple[Socket, ...],
    part: Mapping[str, object] | None = None,
    mesh_paths: frozenset[str] = frozenset(),
) -> Module:
    """Admit one authored module (an adapter plate, a bracket, a gripper), or refuse.

    The module joins by registered mechanical interfaces only: its plug and every socket
    name an interface in :data:`~sx_embodiments.mechanical.MECHANICAL_INTERFACES`, and each
    socket sits on a link of the description. Its root link is the plug face. With ``part``
    the module carries that one component on its root link, admitted by :func:`admit_part`;
    without one it is a passive element, so its description moves nothing and every link
    with mass has a finite positive one. A socket below a movable joint rides on the
    component, whatever the caller said about carriers.
    """

    for interface in (plug, *(socket.interface for socket in sockets)):
        if interface is not None and interface not in MECHANICAL_INTERFACES:
            raise AssemblyError(
                module_id, f"{interface!r} is not a registered mechanical interface"
            )
    try:
        root = ET.fromstring(urdf)
    except ET.ParseError as exc:
        raise EmbodimentSchemaError(f"{module_id}: module description is invalid XML") from exc
    links = {link.get("name") for link in root.iter("link")}
    for socket in sockets:
        if socket.link not in links:
            raise SocketError(
                module_id, f"{socket.name!r} sits on {socket.link!r}, which the description lacks"
            )
    movable = _movable_joints(root)
    components: tuple[Component, ...] = ()
    if part is None:
        if movable:
            raise AssemblyError(
                module_id,
                "a passive module moves nothing; its description has movable joints "
                + ", ".join(sorted(movable)),
            )
        masses = [_finite(mass.get("value"), module_id, "link mass") for mass in root.iter("mass")]
        if not masses or any(mass <= 0.0 for mass in masses):
            raise EmbodimentSchemaError(
                f"{module_id}: a passive module needs a finite positive-mass inertial"
            )
        _require_mesh_closure(module_id, root, mesh_paths)
    else:
        admitted = admit_part(part, urdf=urdf, mesh_paths=mesh_paths)
        frame = _root_link(module_id, root)
        match admitted:
            case GripperSpec():
                components = (body_component("gripper", admitted, RootMount(frame)),)
            case CameraSpec() | ForceTorqueSpec():
                components = (sensor_component("sensor", admitted, RootMount(frame)),)
            case _:
                raise AssemblyError(
                    module_id,
                    "a chain or mobile base is composed through embodiment.assemble,"
                    " not mated as one module",
                )
    parents = {
        child.get("link", ""): (parent.get("link", ""), joint.get("name", ""))
        for joint in root.iter("joint")
        if (parent := joint.find("parent")) is not None
        and (child := joint.find("child")) is not None
    }
    owners = dict.fromkeys(movable, components[0].instance) if components else {}
    return Module(
        module_id=ModuleId(module_id),
        label=label,
        description=urdf,
        plug=plug,
        sockets=tuple(
            replace(socket, carrier=carrier(socket.link, parents, owners)) for socket in sockets
        ),
        components=components,
    )


MODULE_SCHEMA = "sx.authored-module/v1"


def module_to_dict(module: Module) -> dict[str, object]:
    """The canonical document of an authored module: what it joins by and the part it carries.

    Its description and files travel beside it as their own bytes; ``module_from_dict``
    admits the document against exactly those, so one document names one lawful module.
    """
    components = [entry for entry in module.components if isinstance(entry, Component)]
    part = components[0].part if components else None
    if len(components) > 1 or (part is not None and not isinstance(part, ComposablePart)):
        raise AssemblyError(str(module.module_id), "an authored module carries at most one part")
    return {
        "schema": MODULE_SCHEMA,
        "module_id": str(module.module_id),
        "label": module.label,
        "plug": module.plug,
        "sockets": [
            {
                "name": socket.name,
                "interface": str(socket.interface),
                "link": socket.link,
                "xyz": list(socket.pose.xyz),
                "rpy": list(socket.pose.rpy),
            }
            for socket in module.sockets
        ],
        "part": None if part is None else part_to_dict(part),
    }


def module_from_dict(
    document: Mapping[str, object], *, urdf: bytes, mesh_paths: frozenset[str] = frozenset()
) -> Module:
    """Parse and admit an authored module's document against its exact description."""
    entry = decode.exactly(
        document,
        {"schema", "module_id", "label", "plug", "sockets", "part"},
        where="module",
        error=EmbodimentSchemaError,
    )
    if decode.text(entry, "schema") != MODULE_SCHEMA:
        raise EmbodimentSchemaError(f"module: schema must be {MODULE_SCHEMA!r}")
    module_id = decode.text(entry, "module_id")
    plug = decode.optional_text(entry, "plug")
    sockets: list[Socket] = []
    for raw in decode.documents(entry, "sockets"):
        site = decode.exactly(
            raw, {"name", "interface", "link", "xyz", "rpy"}, error=EmbodimentSchemaError
        )
        xyz = decode.numbers(site, "xyz")
        rpy = decode.numbers(site, "rpy")
        if len(xyz) != 3 or len(rpy) != 3:
            raise EmbodimentSchemaError(f"{module_id}: a socket pose is three xyz and three rpy")
        sockets.append(
            Socket(
                decode.text(site, "name"),
                InterfaceId(decode.text(site, "interface")),
                decode.text(site, "link"),
                Pose((xyz[0], xyz[1], xyz[2]), (rpy[0], rpy[1], rpy[2])),
            )
        )
    part = entry.get("part")
    return admit_module(
        module_id,
        decode.text(entry, "label"),
        urdf=urdf,
        plug=None if plug is None else InterfaceId(plug),
        sockets=tuple(sockets),
        part=None if part is None else decode.mapping(entry, "part"),
        mesh_paths=mesh_paths,
    )


def _root_link(subject: str, root: ET.Element) -> str:
    children = {child.get("link") for child in root.iter("child")}
    roots = [
        name
        for link in root.iter("link")
        if (name := link.get("name")) is not None and name not in children
    ]
    if len(roots) != 1:
        raise EmbodimentSchemaError(f"{subject}: a module description has exactly one root link")
    return roots[0]


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
    reference it (sha-verified) as the one DESCRIPTION URDF. Every native axis must
    declare direct, integrated or passive actuation.
    A declaration does not prove runtime driver availability. ``parts`` is the resolvable catalog:
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
    if kind is EmbodimentKind.TELEOP_STATION:
        raise AssemblyError(
            name, "session leader/follower roles must be bound separately from assembly"
        )
    if parts is None:
        parts = composable_parts()
    components: list[Component] = []
    for raw in decode.documents(entry, "attachments"):
        attachment = decode.exactly(raw, {"instance", "part_id", "mount", "limb"})
        part_id = PartId(decode.text(attachment, "part_id"))
        part = parts.get(part_id)
        if part is None:
            known = ", ".join(sorted(str(part_id) for part_id in parts))
            raise AssemblyError(
                name,
                f"part {str(part_id)!r} is not in the composable catalog; known parts: {known}",
            )
        instance = decode.text(attachment, "instance")
        mount = _parse_component_mount(decode.mapping(attachment, "mount"))
        limb_value = decode.optional_text(attachment, "limb")
        try:
            limb = Limb(limb_value) if limb_value is not None else None
        except ValueError as exc:
            raise AssemblyError(name, f"{instance!r}: limb is unknown") from exc
        if limb is Limb.UNDOCUMENTED:
            raise AssemblyError(name, f"{instance!r}: say which limb this chain serves")
        if isinstance(part, (CameraSpec, ForceTorqueSpec)):
            if limb is not None:
                raise AssemblyError(name, f"{instance!r}: a sensor serves no limb")
            component = sensor_component(instance, part, mount)
        else:
            component = body_component(instance, part, mount, limb)
        components.append(component)
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
    _require_declared_actuation(embodiment)
    _validate_urdf(
        embodiment.name,
        tuple(asset.asset for asset in embodiment.assets),
        urdf,
        components=embodiment.components,
        operator_mounts=embodiment.operator_mounts,
    )
    _validate_joint_parity(embodiment, urdf)
    return embodiment


def _require_declared_actuation(embodiment: Embodiment) -> None:
    for coordinate in embodiment.state.coordinates:
        if isinstance(coordinate.axis.actuation, UndocumentedDrive):
            raise AssemblyError(
                str(embodiment.name),
                f"axis {coordinate.instance}/{coordinate.joint_name} has undocumented drive"
                " facts; declare direct, integrated, or passive actuation",
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
