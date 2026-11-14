#!/usr/bin/env python3
"""Fit a reference arm model's gravity and friction onto the description, as one ``dynamics``
calibration (``sx_embodiments.calibration.DynamicsCalibration``).

The reference is an arm model accepted on the robot outside sx: the November 14 seven-joint
left-arm handoff (its gravity geometry, calibrated link masses and centres, and the Coulomb
friction its controller fed forward). Its files are read as text and AST, never imported, and
only when its manifest verifies them. The output is the same document ``sx-identify`` fits,
in the description's own joint coordinates, so the station applies it through
``Dynamics::calibrated``, ``ModelFeedForward`` and ``RestModel`` like any identified fit.

How the reference becomes link parameters:

1. Each joint's reference coordinate is ``q_ref = scale * q + offset``, from the reference's
   encoder offset and sign and the robot's drive (``actuator = sign * direction * (q - zero)
   * reduction``); torque maps back by virtual work, ``tau = scale * tau_ref``.
2. Each reference link's mass and centre, placed in the description's link frame at the
   description's zero pose, is the prior. Rigidly carried links the reference lumps in (the
   gripper under the wrist) are taken out of the link that carries them.
3. Gravity torque is linear in each link's mass and first moment, so a least-squares fit of
   the seven links' first moments (masses stay at the prior: gravity alone cannot tell a
   link's mass from its parent's first moment) to the reference's gravity over the joint
   range makes the description reproduce it. Inertia tensors keep the description's shape,
   scaled to the fitted mass and moved to the fitted centre; gravity does not read them.

The fit's worst torque residual over a held-out set of poses is the calibration's residual
and must stay below ``--max-residual-nm``. It is near zero only when the drive's zeros agree
with the reference's encoder offsets; a disagreement (a nominal drive) leaves a residual the
receipt reports per joint. The accepted run's gravity share (``gravity_ratio``), its
feed-forward slew and its per-joint torque clamps are controller settings, not the arm's
physics, and are not carried: the receipt lists them.

No file is published to Fleet by this command.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from datetime import UTC, datetime
from pathlib import Path
from typing import Final, cast

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sx_contracts import Sha256Digest  # noqa: E402

from sx_embodiments.calibrate.dynamics import (  # noqa: E402
    PER_LINK,
    Array,
    RigidBodyTree,
    _about,  # pyright: ignore[reportPrivateUsage] - the one rotation the regressor uses
)
from sx_embodiments.calibration import (  # noqa: E402
    CalibrationMethod,
    DynamicsCalibration,
    Evidence,
    EvidenceKind,
    JointFriction,
    LinkInertia,
    Measurement,
    Residual,
    ResidualUnit,
    UnitSerial,
    calibration_from_dict,
    calibration_to_dict,
    is_physical_inertia,
)
from sx_embodiments.kinematics import JointType  # noqa: E402

# The reviewed clamp/deadband/friction policy the reference controller applied. Its friction
# table and elbow deadband are read below; a changed policy needs this importer reviewed.
_REFERENCE_TORQUE_POLICY: Final = "21b2a276d0bc4aac2890a71260780c1e0509eee1cbdb85f40f6e434b7a54059a"
# The reference's smooth sign: tanh(q̇ / 0.01) on every joint, and on the elbow
# tanh((|q̇| - 0.02) / 0.03), motor rad/s. The calibration keeps one stiction velocity.
_REFERENCE_STICTION: Final = 0.01
_ELBOW_INDEX: Final = 3
_ELBOW_DEADBAND: Final = 0.02
# Poses the fit and its held-out check sample, uniformly inside both models' joint ranges.
_FIT_POSES: Final = 96
_CHECK_POSES: Final = 48


class ReferenceImportError(ValueError):
    """The reference or the robot's files do not support a calibration."""


@dataclass(frozen=True)
class ReferenceJoint:
    name: str
    parent: str
    child: str
    xyz: tuple[float, ...]
    rpy: tuple[float, ...]
    axis: tuple[float, ...]
    lower: float
    upper: float


def _assignment(tree: ast.Module, name: str) -> ast.expr:
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == name for target in node.targets
        ):
            return node.value
    raise ReferenceImportError(f"reference source has no literal {name}")


def _literal(node: ast.AST) -> object:
    if (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "np"
        and node.func.attr == "array"
        and node.args
    ):
        return ast.literal_eval(node.args[0])
    return ast.literal_eval(node)


def _vector(node: ast.AST) -> tuple[float, ...]:
    raw = _literal(node)
    if not isinstance(raw, tuple | list):
        raise ReferenceImportError("reference joint vector has three values")
    values = cast("tuple[object, ...] | list[object]", raw)
    if len(values) != 3 or any(
        isinstance(value, bool) or not isinstance(value, int | float) for value in values
    ):
        raise ReferenceImportError("reference joint vector has three numbers")
    vector = tuple(float(cast("float", value)) for value in values)
    if not all(math.isfinite(value) for value in vector):
        raise ReferenceImportError("reference joint vector is finite")
    return vector


def _reference_joints(tree: ast.Module) -> tuple[ReferenceJoint, ...]:
    sequence = _assignment(tree, "LEFT_ARM_JOINTS")
    if not isinstance(sequence, ast.Tuple) or len(sequence.elts) != 7:
        raise ReferenceImportError("reference source must define its seven-joint arm")
    joints: list[ReferenceJoint] = []
    for call in sequence.elts:
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name):
            raise ReferenceImportError("reference joints are explicit JointKinematics calls")
        if call.func.id != "JointKinematics" or call.args:
            raise ReferenceImportError("reference joint constructor changed")
        fields = {item.arg: item.value for item in call.keywords}
        joints.append(
            ReferenceJoint(
                name=str(_literal(fields["name"])),
                parent=str(_literal(fields["parent_link"])),
                child=str(_literal(fields["child_link"])),
                xyz=_vector(fields["xyz"]),
                rpy=_vector(fields["rpy"]),
                axis=_vector(fields["axis"]),
                lower=float(cast("float", _literal(fields["lower_limit"]))),
                upper=float(cast("float", _literal(fields["upper_limit"]))),
            )
        )
    return tuple(joints)


def _source_urdf(
    joints: tuple[ReferenceJoint, ...], inertials: Mapping[str, object], mass_scale: float
) -> str:
    root = ET.Element("robot", name="accepted_left_arm_gravity")
    ET.SubElement(root, "link", name=joints[0].parent)

    def text(values: tuple[float, ...] | list[float]) -> str:
        return " ".join(format(value, ".17g") for value in values)

    for index, joint in enumerate(joints):
        link = ET.SubElement(root, "link", name=joint.child)
        inertial = ET.SubElement(link, "inertial")
        entry = cast("Mapping[str, object]", inertials[joint.child])
        mass = float(cast("float", entry["mass_kg"])) * mass_scale
        centre = cast("list[float]", entry["com_m"])
        if not math.isfinite(mass) or mass <= 0 or len(centre) != 3:
            raise ReferenceImportError("reference mass/centre is invalid")
        if not all(math.isfinite(value) for value in centre):
            raise ReferenceImportError("reference centre is not finite")
        ET.SubElement(inertial, "origin", xyz=text(centre))
        ET.SubElement(inertial, "mass", value=format(mass, ".17g"))
        # Positive physical placeholder; this source description is used for gravity ONLY.
        tensor = format(mass * 1e-6, ".17g")
        ET.SubElement(
            inertial, "inertia", ixx=tensor, iyy=tensor, izz=tensor, ixy="0", ixz="0", iyz="0"
        )
        variable = ET.SubElement(root, "joint", name=joint.name, type="revolute")
        ET.SubElement(variable, "parent", link=joint.parent)
        ET.SubElement(variable, "child", link=joint.child)
        rpy = joint.rpy
        if index == 3:
            if joint.axis != (0.0, -1.0, 0.0) or rpy != (0.0, 0.0, 0.0):
                raise ReferenceImportError("reference elbow convention changed")
            # R(-Y, q + pi/2) = R(-Y, pi/2) R(-Y, q).
            rpy = (0.0, -math.pi / 2, 0.0)
        ET.SubElement(variable, "origin", xyz=text(joint.xyz), rpy=text(rpy))
        ET.SubElement(variable, "axis", xyz=text(joint.axis))
        ET.SubElement(
            variable,
            "limit",
            lower=str(joint.lower),
            upper=str(joint.upper),
            effort="1",
            velocity="1",
        )
    return ET.tostring(root, encoding="unicode")


def _checked_sources(reference: Path) -> dict[str, bytes]:
    manifest = json.loads((reference.parent / "MANIFEST.json").read_text())
    records = {item["path"]: item for item in manifest["files"]}
    sources: dict[str, bytes] = {}
    for name in (
        "left_arm_dynamics.py",
        "left_arm_freedrive.py",
        "can_bus.py",
        "left_arm_calibrated_config.json",
        "left_arm_calibrated_inertials.json",
    ):
        data = (reference / name).read_bytes()
        record = records.get(f"reference_snapshot/{name}")
        if (
            record is None
            or record["bytes"] != len(data)
            or record["sha256"] != hashlib.sha256(data).hexdigest()
        ):
            raise ReferenceImportError(f"bundle manifest does not verify {name}")
        sources[name] = data
    return sources


@dataclass(frozen=True)
class Mapped:
    """One reference joint on the description: ``q_ref = scale * q + offset``."""

    joint: str
    scale: float
    offset: float
    port: str


def _commissioned_ports(commissioning_bytes: bytes, instance: object) -> dict[str, str]:
    raw: object = json.loads(commissioning_bytes)
    if not isinstance(raw, Mapping):
        raise ReferenceImportError("commissioning is an object")
    commissioning = cast("Mapping[str, object]", raw)
    required = {"schema", "instance", "pod_id", "pod_revision", "ports"}
    optional = {"out_of_service", "unidentified", "rails", "derate", "exit_poses"}
    if (
        required - set(commissioning)
        or set(commissioning) - required - optional
        or commissioning["schema"] != "sx.station-commissioning/v1"
    ):
        raise ReferenceImportError("commissioning is not a known station commissioning document")
    if commissioning["instance"] != instance:
        raise ReferenceImportError("commissioning and drive name different instances")
    revision = commissioning["pod_revision"]
    if type(revision) is not int or not 0 < revision <= 2**32 - 1:
        raise ReferenceImportError("commissioning revision is a positive u32")
    pod = commissioning["pod_id"]
    if not isinstance(pod, str) or not pod or any(c.isspace() for c in pod):
        raise ReferenceImportError("commissioning pod identity is nonempty without whitespace")
    raw_ports = commissioning["ports"]
    if not isinstance(raw_ports, Mapping) or not raw_ports:
        raise ReferenceImportError("commissioning ports is a nonempty mapping")
    ports: dict[str, str] = {}
    for port, unit in cast("Mapping[object, object]", raw_ports).items():
        if (
            not isinstance(port, str)
            or not isinstance(unit, str)
            or not port
            or not unit
            or any(c.isspace() for c in port + unit)
        ):
            raise ReferenceImportError(
                "commissioned ports and bus identities contain no whitespace"
            )
        ports[port] = unit
    return ports


def _poses(
    tree: RigidBodyTree, joints: Sequence[str], reference: RigidBodyTree, maps: Sequence[Mapped]
) -> tuple[Array, Array]:
    """Each joint's range inside both models' limits, in the description's coordinates."""
    lower: list[float] = []
    upper: list[float] = []
    for name, mapped in zip(joints, maps, strict=True):
        own = tree.limits[name]
        theirs = reference.limits[name]
        if own is None or theirs is None:
            raise ReferenceImportError(f"{name} has no range to sample")
        ends = sorted(
            ((theirs[0] - mapped.offset) / mapped.scale, (theirs[1] - mapped.offset) / mapped.scale)
        )
        low, high = max(own[0], ends[0]), min(own[1], ends[1])
        if not low < high:
            raise ReferenceImportError(f"{name}: the two models' ranges do not overlap")
        lower.append(low)
        upper.append(high)
    return np.array(lower), np.array(upper)


def _frames(tree: RigidBodyTree, q: Array) -> dict[int, Array]:
    """Each link's pose in the root frame at ``q`` (one value per ``tree.axes``)."""
    placed: dict[int, Array] = {tree.root: np.eye(4)}
    for joint in tree.joints:
        local = joint.origin.copy()
        if joint.leader is not None and joint.kind in (JointType.REVOLUTE, JointType.CONTINUOUS):
            turn = np.eye(4)
            turn[:3, :3] = _about(joint.axis, joint.multiplier * q[joint.leader] + joint.offset)
            local = local @ turn
        placed[joint.child] = placed[joint.parent] @ local
    return placed


def _carried(tree: RigidBodyTree, link: int, fitted: set[int]) -> list[int]:
    """Links with mass below ``link`` that no other fitted link carries: what the reference
    lumps into it (a gripper under the wrist)."""
    out: list[int] = []
    frontier = [link]
    while frontier:
        at = frontier.pop()
        for joint in tree.joints:
            if joint.parent == at and joint.child not in fitted:
                frontier.append(joint.child)
                if tree.prior[PER_LINK * joint.child] > 0.0:
                    out.append(joint.child)
    return out


def _gravity_rows(
    tree: RigidBodyTree, columns: Sequence[int], rows: Sequence[int], q: Array
) -> tuple[Array, Array]:
    """At ``q``: the gravity regressor's fitted ``columns`` on ``rows``, and the torque the
    unfitted parameters (the description's) add there."""
    rest = np.zeros(len(tree.axes))
    regressor = tree.regressor(q, rest, rest)[list(rows)]
    theta = np.concatenate([tree.prior, np.zeros(regressor.shape[1] - len(tree.prior))])
    fixed = theta.copy()
    fixed[list(columns)] = 0.0
    return regressor[:, list(columns)], regressor @ fixed


def _inertia(tree: RigidBodyTree, link: int, mass: float, first_moment: Array) -> tuple[float, ...]:
    """The description's tensor about its centre, scaled to ``mass``, about the new centre,
    then about the link origin: physical whenever the description's is."""
    at = PER_LINK * link
    design_mass = float(tree.prior[at])
    design_centre = tree.prior[at + 1 : at + 4] / design_mass
    xx, xy, xz, yy, yz, zz = tree.prior[at + 4 : at + 10]
    about_origin = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])

    def shift(m: float, c: Array) -> Array:
        return m * (float(c @ c) * np.eye(3) - np.outer(c, c))

    about_centre = (about_origin - shift(design_mass, design_centre)) * (mass / design_mass)
    centre = first_moment / mass
    tensor = about_centre + shift(mass, centre)
    return (
        float(tensor[0, 0]),
        float(tensor[0, 1]),
        float(tensor[0, 2]),
        float(tensor[1, 1]),
        float(tensor[1, 2]),
        float(tensor[2, 2]),
    )


def build_calibration(
    reference: Path,
    urdf: bytes,
    drive_bytes: bytes,
    bindings: bytes,
    commissioning_bytes: bytes,
    *,
    mass_scale: float,
    friction_multiplier: float,
    max_residual_nm: float,
) -> tuple[DynamicsCalibration, dict[str, object]]:
    """The calibration and its receipt; every run-dependent multiplier is explicit."""
    if not math.isfinite(mass_scale) or mass_scale <= 0:
        raise ReferenceImportError("mass scale is positive and finite")
    if not math.isfinite(friction_multiplier) or friction_multiplier < 0:
        raise ReferenceImportError("friction multiplier is nonnegative and finite")
    if not math.isfinite(max_residual_nm) or max_residual_nm <= 0:
        raise ReferenceImportError("the residual bound is positive and finite")
    sources = _checked_sources(reference)
    model = ast.parse(sources["left_arm_dynamics.py"])
    controller = ast.parse(sources["left_arm_freedrive.py"])
    policies = [
        node
        for node in ast.walk(controller)
        if isinstance(node, ast.FunctionDef) and node.name == "urdf_to_motor_torque"
    ]
    if (
        len(policies) != 1
        or hashlib.sha256(ast.unparse(policies[0]).encode()).hexdigest() != _REFERENCE_TORQUE_POLICY
    ):
        raise ReferenceImportError("reference torque policy changed; review the importer first")
    # Refuse a changed elbow expression rather than silently importing a different convention.
    elbow = [
        node
        for node in ast.walk(model)
        if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "q_geom[3]"
    ]
    if len(elbow) != 1 or ast.unparse(elbow[0].value) != "float(q[3]) + math.pi / 2.0":
        raise ReferenceImportError("reference elbow geometry expression changed")
    joints = _reference_joints(model)
    friction = cast("dict[str, float]", _literal(_assignment(controller, "FRICTION_TORQUE")))
    slew = float(cast("float", _literal(_assignment(controller, "TORQUE_SLEW_RATE_NM_S"))))
    calibrated = json.loads(sources["left_arm_calibrated_config.json"])
    config = calibrated["joints"]
    inertials = json.loads(sources["left_arm_calibrated_inertials.json"])
    drive = json.loads(drive_bytes)
    binding = json.loads(bindings)
    if drive.get("schema") != "sx.instance-drive/v1" or drive.get("embodiment") != binding["id"]:
        raise ReferenceImportError("drive and bindings name different bodies")
    ports = _commissioned_ports(commissioning_bytes, drive.get("instance"))
    driven = {item["joint"]: item for item in drive["joints"]}
    axes = {item["joint"]: item for item in binding["axes"]}
    canonical_names = {item.attrib["name"] for item in ET.fromstring(urdf).findall("joint")}
    maps: list[Mapped] = []
    for joint in joints:
        if joint.name not in canonical_names or joint.name not in axes or joint.name not in driven:
            raise ReferenceImportError(f"the description, bindings or drive has no {joint.name}")
        axis = axes[joint.name]
        port = axis.get("port")
        if not isinstance(port, str) or port not in ports:
            raise ReferenceImportError(f"selected joint {joint.name} has no commissioned bus")
        direction = driven[joint.name]["direction"]
        sign = config[joint.name]["sign"]
        if direction not in (-1, 1) or sign not in (-1.0, 1.0):
            raise ReferenceImportError("joint direction/sign must be +1 or -1")
        # actuator = gain * (q - zero); q_ref = sign * (actuator - q_offset_motor).
        gain = float(axis["sign"]) * direction * float(axis["reduction"])
        if not math.isfinite(gain) or gain == 0:
            raise ReferenceImportError("actuator conversion has a nonzero finite gain")
        offset = -sign * (
            gain * float(driven[joint.name]["zero"]) + config[joint.name]["q_offset_motor_rad"]
        )
        maps.append(Mapped(joint.name, sign * gain, offset, port))
    if len({abs(item.scale) for item in maps}) != 1:
        raise ReferenceImportError("one stiction velocity needs one reduction across the arm")
    names = [item.joint for item in maps]
    scale = np.array([item.scale for item in maps])
    offset = np.array([item.offset for item in maps])

    tree = RigidBodyTree.of(urdf)
    source = RigidBodyTree.of(_source_urdf(joints, inertials, mass_scale).encode())
    if source.axes != tuple(names):
        raise ReferenceImportError("the reference's joints are not the arm's, in order")
    rows = [tree.axes.index(name) for name in names]
    links = [joint.child for joint in joints]
    fitted = [tree.links.index(link) for link in links]
    source_theta = np.concatenate([source.prior, np.zeros(3 * len(source.axes))])

    # The prior: each reference link's mass and centre in the description's link frame, at
    # the description's zero pose, less what the description hangs under it that the
    # reference lumped in.
    zero = np.zeros(len(tree.axes))
    own = _frames(tree, zero)
    theirs = _frames(source, offset)
    root = tree.links.index(joints[0].parent)
    their_root = source.links.index(joints[0].parent)
    prior = np.concatenate([tree.prior, np.zeros(3 * len(tree.axes))])
    for link, name in zip(fitted, links, strict=True):
        at, there = PER_LINK * link, PER_LINK * source.links.index(name)
        mass = float(source_theta[there])
        relative = np.linalg.inv(np.linalg.inv(own[root]) @ own[link]) @ (
            np.linalg.inv(theirs[their_root]) @ theirs[source.links.index(name)]
        )
        centre = (relative @ np.append(source_theta[there + 1 : there + 4] / mass, 1.0))[:3]
        moment = mass * centre
        for carried in _carried(tree, link, set(fitted)):
            m = float(tree.prior[PER_LINK * carried])
            c = tree.prior[PER_LINK * carried + 1 : PER_LINK * carried + 4] / m
            placed = (np.linalg.inv(own[link]) @ own[carried] @ np.append(c, 1.0))[:3]
            mass -= m
            moment = moment - m * placed
        if mass <= 0.0:
            raise ReferenceImportError(f"{name}: the reference is lighter than what it carries")
        prior[at] = mass
        prior[at + 1 : at + 4] = moment

    def reference_gravity(q: Array) -> Array:
        rest = np.zeros(len(source.axes))
        return scale * (source.regressor(scale * q + offset, rest, rest) @ source_theta)

    lower, upper = _poses(tree, names, source, maps)
    moments = [PER_LINK * link + k for link in fitted for k in (1, 2, 3)]
    masses = [PER_LINK * link for link in fitted]
    generator = np.random.default_rng(0)
    blocks: list[Array] = []
    targets: list[Array] = []
    for _ in range(_FIT_POSES):
        q = np.zeros(len(tree.axes))
        q[rows] = generator.uniform(lower, upper)
        block, fixed = _gravity_rows(tree, [*masses, *moments], rows, q)
        # The prior's masses stay: only first moments are fitted.
        blocks.append(block[:, len(masses) :])
        targets.append(reference_gravity(q[rows]) - fixed - block[:, : len(masses)] @ prior[masses])
    a, b = np.vstack(blocks), np.concatenate(targets)
    # A small pull toward the prior settles directions gravity cannot see.
    pull = 1e-6 * float(np.trace(a.T @ a)) / len(moments)
    solved = np.linalg.solve(a.T @ a + pull * np.eye(len(moments)), a.T @ b + pull * prior[moments])
    theta = prior.copy()
    theta[moments] = solved

    def calibrated_gravity(q: Array) -> Array:
        rest = np.zeros(len(tree.axes))
        return (tree.regressor(q, rest, rest) @ theta)[rows]

    worst = np.zeros(len(names))
    check = np.random.default_rng(1)
    for _ in range(_CHECK_POSES):
        q = np.zeros(len(tree.axes))
        q[rows] = check.uniform(lower, upper)
        worst = np.maximum(worst, np.abs(calibrated_gravity(q) - reference_gravity(q[rows])))
    residual = float(worst.max())
    if residual > max_residual_nm:
        raise ReferenceImportError(
            f"the description reproduces the reference's gravity only to {residual:.4g} N·m "
            f"(bound {max_residual_nm:g}); per joint "
            + ", ".join(f"{n} {w:.3g}" for n, w in zip(names, worst, strict=True))
            + ". A drive whose zeros disagree with the reference's encoder offsets does this; "
            "see the receipt's offsets"
        )

    link_inertias: list[LinkInertia] = []
    plausibility: dict[str, dict[str, float]] = {}
    for link, name in zip(fitted, links, strict=True):
        at = PER_LINK * link
        mass, moment = float(theta[at]), theta[at + 1 : at + 4]
        inertia = LinkInertia(
            link=name,
            mass=mass,
            first_moment=(float(moment[0]), float(moment[1]), float(moment[2])),
            inertia=_inertia(tree, link, mass, moment),
        )
        if not is_physical_inertia(inertia.mass, inertia.first_moment, inertia.inertia):
            raise ReferenceImportError(f"{name}: the fitted mass properties are not physical")
        link_inertias.append(inertia)
        design = float(tree.prior[at])
        plausibility[name] = {
            "mass_kg": mass,
            "description_mass_kg": design,
            "centre_shift_m": float(
                np.linalg.norm(moment / mass - tree.prior[at + 1 : at + 4] / design)
            ),
        }
    width = abs(maps[0].scale)
    frictions = tuple(
        JointFriction(
            joint=item.joint,
            coulomb=float(friction[item.joint]) * friction_multiplier * width,
            viscous=0.0,
            armature=0.0,
            velocity_deadband=(_ELBOW_DEADBAND if index == _ELBOW_INDEX else 0.0) / width,
        )
        for index, item in enumerate(maps)
    )
    measured_at = datetime.fromtimestamp(float(calibrated["timestamp"]), UTC)
    manifest = hashlib.sha256((reference.parent / "MANIFEST.json").read_bytes()).hexdigest()
    calibration = DynamicsCalibration(
        description_sha256=Sha256Digest(hashlib.sha256(urdf).hexdigest()),
        stiction_velocity=_REFERENCE_STICTION / width,
        links=tuple(sorted(link_inertias, key=lambda item: item.link)),
        joints=tuple(sorted(frictions, key=lambda item: item.joint)),
        units=tuple(sorted({UnitSerial(ports[item.port]) for item in maps})),
        measurement=Measurement(
            method=CalibrationMethod.REFERENCE_MODEL_FIT,
            measured_at=measured_at,
            residual=Residual(residual, ResidualUnit.NEWTON_METRE),
            evidence=(Evidence(EvidenceKind.SEALED_DOCUMENT, manifest),),
        ),
    )
    receipt: dict[str, object] = {
        "source_files": {name: hashlib.sha256(data).hexdigest() for name, data in sources.items()},
        "manifest_sha256": manifest,
        "drive_sha256": hashlib.sha256(drive_bytes).hexdigest(),
        "parameters": {
            "mass_scale": mass_scale,
            "friction_multiplier": friction_multiplier,
            "max_residual_nm": max_residual_nm,
        },
        "offsets_rad": {item.joint: item.offset for item in maps},
        "scales": {item.joint: item.scale for item in maps},
        "residual_nm": {n: float(w) for n, w in zip(names, worst, strict=True)},
        "links": plausibility,
        "not_carried": {
            "gravity_ratio": "the accepted run fed forward a share of gravity; the calibration "
            "is the arm's physics, and the station feeds all of it forward",
            "torque_slew_nm_s": slew,
            "torque_clamps_nm": "0.4 forearm yaw and wrist roll, 0.75 wrist pitch, a fraction "
            "of each motor's limit: the station's feed-forward bound and the governor hold it",
            "elbow_transition_rad_s": "0.03 in the reference; the calibration keeps one "
            f"stiction velocity, {_REFERENCE_STICTION / width}",
        },
        "limits": [
            "masses are the reference's calibrated values; gravity cannot separate a link's "
            "mass from its parent's first moment, so only first moments are fitted",
            "inertia tensors keep the description's shape; nothing here identifies them",
            "the reference's friction was fed forward while hand-guiding; as tracking "
            "feed-forward it follows the reference velocity",
            "the reference hardcodes wire PMAX/VMAX/TMAX; the driver reads the registers",
        ],
    }
    return calibration, receipt


def superseded(existing: DynamicsCalibration, by: DynamicsCalibration) -> DynamicsCalibration:
    """``existing`` without every link and joint ``by`` names: a robot carries one fit per
    link and joint, so a calibration that takes over part of an older one (this arm, out of a
    whole-body fit) ships with the older one trimmed. Everything else is kept as fitted."""

    if existing.description_sha256 != by.description_sha256:
        raise ReferenceImportError("the two calibrations name different descriptions")
    links = {item.link for item in by.links}
    joints = {item.joint for item in by.joints}
    links_kept = tuple(item for item in existing.links if item.link not in links)
    joints_kept = tuple(item for item in existing.joints if item.joint not in joints)
    if not links_kept and not joints_kept:
        raise ReferenceImportError("the new calibration takes over all of the old one")
    return replace(existing, links=links_kept, joints=joints_kept)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    for argument in ("reference-root", "urdf", "drive", "bindings", "commissioning", "out"):
        parser.add_argument(f"--{argument}", type=Path, required=True)
    parser.add_argument(
        "--mass-scale", type=float, required=True, help="The accepted run's mass multiplier"
    )
    parser.add_argument(
        "--friction-multiplier",
        type=float,
        required=True,
        help="The accepted run's friction multiplier",
    )
    parser.add_argument(
        "--supersede",
        type=Path,
        help="A dynamics calibration the robot already carries: written beside --out without "
        "what the new one names, as <name>.trimmed.dynamics.json",
    )
    parser.add_argument(
        "--max-residual-nm",
        type=float,
        default=1e-3,
        help="Refuse a fit whose worst gravity residual exceeds this, N·m",
    )
    args = parser.parse_args()
    calibration, receipt = build_calibration(
        args.reference_root,
        args.urdf.read_bytes(),
        args.drive.read_bytes(),
        args.bindings.read_bytes(),
        args.commissioning.read_bytes(),
        mass_scale=args.mass_scale,
        friction_multiplier=args.friction_multiplier,
        max_residual_nm=args.max_residual_nm,
    )
    if not args.out.name.endswith(".dynamics.json"):
        raise ReferenceImportError("the station reads <name>.dynamics.json")
    args.out.write_text(json.dumps(calibration_to_dict(calibration), indent=2) + "\n")
    args.out.with_name(args.out.name.removesuffix(".dynamics.json") + ".receipt.json").write_text(
        json.dumps(receipt, indent=2) + "\n"
    )
    if args.supersede is not None:
        existing = calibration_from_dict(json.loads(args.supersede.read_text()))
        if not isinstance(existing, DynamicsCalibration):
            raise ReferenceImportError("--supersede names a dynamics calibration")
        trimmed = args.supersede.name.removesuffix(".dynamics.json") + ".trimmed.dynamics.json"
        args.out.with_name(trimmed).write_text(
            json.dumps(calibration_to_dict(superseded(existing, calibration)), indent=2) + "\n"
        )
    summary = {"residual_nm": receipt["residual_nm"], "offsets_rad": receipt["offsets_rad"]}
    sys.stdout.write(json.dumps(summary) + "\n")


if __name__ == "__main__":
    main()
