#!/usr/bin/env python3
"""Import the accepted seven-joint handoff without importing any hardware module.

This imports its gravity geometry and compensation parameters, not its freedrive
state machine or bus backend. The tracking adaptation assists desired velocity;
HOLD/FREE selection and motor gains are not copied. Output is bound to the exact
production URDF and drive bytes. No file is published to Fleet by this command.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Final, cast

from sx_contracts import Sha256Digest

from sx_embodiments.compensation import (
    CompensationError,
    CompensationJoint,
    QuasiStaticCompensation,
    compensation_to_dict,
)

# The adapter below imports this reviewed clamp/deadband policy, not arbitrary controller code.
_REFERENCE_TORQUE_POLICY: Final = "21b2a276d0bc4aac2890a71260780c1e0509eee1cbdb85f40f6e434b7a54059a"


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
    raise CompensationError(f"reference source has no literal {name}")


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
        raise CompensationError("reference joint vector has three values")
    values = cast("tuple[object, ...] | list[object]", raw)
    if len(values) != 3 or any(
        isinstance(value, bool) or not isinstance(value, int | float) for value in values
    ):
        raise CompensationError("reference joint vector has three numbers")
    vector = tuple(float(cast("float", value)) for value in values)
    if not all(math.isfinite(value) for value in vector):
        raise CompensationError("reference joint vector is finite")
    return vector


def _reference_joints(tree: ast.Module) -> tuple[ReferenceJoint, ...]:
    sequence = _assignment(tree, "LEFT_ARM_JOINTS")
    if not isinstance(sequence, ast.Tuple) or len(sequence.elts) != 7:
        raise CompensationError("reference source must define its seven-joint arm")
    joints: list[ReferenceJoint] = []
    for call in sequence.elts:
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name):
            raise CompensationError("reference joints are explicit JointKinematics calls")
        if call.func.id != "JointKinematics" or call.args:
            raise CompensationError("reference joint constructor changed")
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
            raise CompensationError("reference mass/centre is invalid")
        if not all(math.isfinite(value) for value in centre):
            raise CompensationError("reference centre is not finite")
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
                raise CompensationError("reference elbow convention changed")
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
            raise CompensationError(f"bundle manifest does not verify {name}")
        sources[name] = data
    return sources


def build_compensation(
    reference: Path,
    urdf: bytes,
    drive_bytes: bytes,
    bindings: bytes,
    commissioning_bytes: bytes,
    *,
    gravity_ratio: float,
    mass_scale: float,
    friction_multiplier: float,
    max_torque_fraction: float,
) -> tuple[QuasiStaticCompensation, dict[str, object]]:
    """Build a bound artifact; all run-dependent multipliers must be explicit."""
    if not math.isfinite(mass_scale) or mass_scale <= 0:
        raise CompensationError("mass scale is positive and finite")
    if not math.isfinite(friction_multiplier) or friction_multiplier < 0:
        raise CompensationError("friction multiplier is nonnegative and finite")
    if not math.isfinite(max_torque_fraction) or not 0 < max_torque_fraction <= 1:
        raise CompensationError("maximum torque fraction is greater than zero and at most one")
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
        or hashlib.sha256(
            ast.unparse(policies[0]).encode(),
        ).hexdigest()
        != _REFERENCE_TORQUE_POLICY
    ):
        raise CompensationError("reference torque policy changed; review the importer first")
    bus = ast.parse(sources["can_bus.py"])
    # Refuse a changed elbow expression rather than silently importing a different convention.
    elbow = [
        node
        for node in ast.walk(model)
        if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "q_geom[3]"
    ]
    if len(elbow) != 1 or ast.unparse(elbow[0].value) != "float(q[3]) + math.pi / 2.0":
        raise CompensationError("reference elbow geometry expression changed")
    joints = _reference_joints(model)
    friction = cast("dict[str, float]", _literal(_assignment(controller, "FRICTION_TORQUE")))
    slew = float(cast("float", _literal(_assignment(controller, "TORQUE_SLEW_RATE_NM_S"))))
    motors_ast = _assignment(bus, "LEFT_ARM_MOTORS")
    if not isinstance(motors_ast, ast.Tuple):
        raise CompensationError("reference motors are an explicit tuple")
    motors: dict[str, tuple[float, float]] = {}
    for motor in motors_ast.elts:
        if not isinstance(motor, ast.Call) or len(motor.args) != 8:
            raise CompensationError("reference MotorConfig changed")
        name = str(_literal(motor.args[0]))
        motors[name] = (
            float(cast("float", _literal(motor.args[6]))),
            float(cast("float", _literal(motor.args[7]))),
        )
    config = json.loads(sources["left_arm_calibrated_config.json"])["joints"]
    inertials = json.loads(sources["left_arm_calibrated_inertials.json"])
    drive = json.loads(drive_bytes)
    binding = json.loads(bindings)
    if drive.get("schema") != "sx.instance-drive/v1" or drive.get("embodiment") != binding["id"]:
        raise CompensationError("drive and bindings name different bodies")
    driven = {item["joint"]: item for item in drive["joints"]}
    axes = {item["joint"]: item for item in binding["axes"]}
    raw_commissioning: object = json.loads(commissioning_bytes)
    if not isinstance(raw_commissioning, Mapping):
        raise CompensationError("commissioning is an object")
    commissioning = cast("Mapping[str, object]", raw_commissioning)
    required = {"schema", "instance", "pod_id", "pod_revision", "ports"}
    optional = {"out_of_service", "unidentified", "rails", "derate", "exit_poses"}
    if (
        required - set(commissioning)
        or set(commissioning) - required - optional
        or commissioning["schema"] != "sx.station-commissioning/v1"
    ):
        raise CompensationError("commissioning is not a known station commissioning document")
    if commissioning["instance"] != drive.get("instance"):
        raise CompensationError("commissioning and drive name different instances")
    revision = commissioning["pod_revision"]
    if type(revision) is not int or not 0 < revision <= 2**32 - 1:
        raise CompensationError("commissioning revision is a positive u32")
    pod = commissioning["pod_id"]
    if not isinstance(pod, str) or not pod or any(c.isspace() for c in pod):
        raise CompensationError("commissioning pod identity is nonempty without whitespace")
    raw_ports = commissioning["ports"]
    if not isinstance(raw_ports, Mapping) or not raw_ports:
        raise CompensationError("commissioning ports is a nonempty mapping")
    port_values = cast("Mapping[object, object]", raw_ports)
    ports: dict[str, str] = {}
    for port, unit in port_values.items():
        if (
            not isinstance(port, str)
            or not isinstance(unit, str)
            or not port
            or not unit
            or any(c.isspace() for c in port + unit)
        ):
            raise CompensationError("commissioned ports and bus identities contain no whitespace")
        ports[port] = unit
    canonical_names = {item.attrib["name"] for item in ET.fromstring(urdf).findall("joint")}
    units: set[str] = set()
    mappings: list[CompensationJoint] = []
    for index, joint in enumerate(joints):
        if joint.name not in canonical_names:
            raise CompensationError(f"canonical URDF has no {joint.name}")
        axis = axes[joint.name]
        calibrated = driven[joint.name]
        port = axis.get("port")
        if not isinstance(port, str) or port not in ports:
            raise CompensationError(f"selected joint {joint.name} has no commissioned bus")
        units.add(ports[port])
        source = config[joint.name]
        direction = calibrated["direction"]
        sign = source["sign"]
        if direction not in (-1, 1) or sign not in (-1.0, 1.0):
            raise CompensationError("joint direction/sign must be +1 or -1")
        gain = float(axis["sign"]) * direction * float(axis["reduction"])
        if not math.isfinite(gain) or gain == 0:
            raise CompensationError("actuator conversion has a nonzero finite gain")
        wire_limit, effort_limit = motors[joint.name]
        motor_max = max_torque_fraction * min(wire_limit, effort_limit)
        if index in (4, 6):
            motor_max = min(motor_max, 0.40)
        elif index == 5:
            motor_max = min(motor_max, 0.75)
        mappings.append(
            CompensationJoint(
                joint=joint.name,
                source_joint=joint.name,
                scale=sign * gain,
                offset=-sign * (gain * float(calibrated["zero"]) + source["q_offset_motor_rad"]),
                coulomb=float(friction[joint.name]) * friction_multiplier * abs(gain),
                velocity_deadband=(0.02 if index == 3 else 0.0) / abs(gain),
                transition_velocity=(0.03 if index == 3 else 0.01) / abs(gain),
                max_torque=motor_max * abs(gain),
            )
        )
    if len({abs(item.scale) for item in mappings}) != 1 or abs(mappings[0].scale) != 1:
        raise CompensationError("this imported slew law requires direct-drive reductions of one")
    artifact = QuasiStaticCompensation(
        description_sha256=Sha256Digest(hashlib.sha256(urdf).hexdigest()),
        drive_sha256=Sha256Digest(hashlib.sha256(drive_bytes).hexdigest()),
        source_urdf=_source_urdf(joints, inertials, mass_scale),
        units=tuple(sorted(units)),
        gravity_scale=gravity_ratio,
        torque_slew_nm_s=slew,
        joints=tuple(mappings),
    )
    provenance: dict[str, object] = {
        "source_files": {name: hashlib.sha256(data).hexdigest() for name, data in sources.items()},
        "parameters": {
            "gravity_ratio": gravity_ratio,
            "mass_scale": mass_scale,
            "friction_multiplier": friction_multiplier,
            "max_torque_fraction": max_torque_fraction,
        },
        "limits": [
            "gravity-only source inertias are placeholders, not a dynamic identification",
            "tracking friction follows desired velocity; freedrive state machine not imported",
            "reference torque projection uses a slightly nonunit shoulder axis",
            "wire scaling and selected accepted-run arguments require runtime reconciliation",
        ],
    }
    return artifact, provenance


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for argument in ("reference-root", "urdf", "drive", "bindings", "commissioning", "out"):
        parser.add_argument(f"--{argument}", type=Path, required=True)
    multipliers = {
        "gravity-ratio": "Explicit accepted-run gravity multiplier: greater than zero, at most one",
        "mass-scale": "Explicit accepted-run mass multiplier: positive; gravity-only model",
        "friction-multiplier": "Explicit accepted-run friction multiplier: nonnegative",
        "max-torque-fraction": "Explicit accepted-run torque fraction: 0 < fraction <= 1",
    }
    for argument, help_text in multipliers.items():
        parser.add_argument(f"--{argument}", type=float, required=True, help=help_text)
    args = parser.parse_args()
    artifact, provenance = build_compensation(
        args.reference_root,
        args.urdf.read_bytes(),
        args.drive.read_bytes(),
        args.bindings.read_bytes(),
        args.commissioning.read_bytes(),
        gravity_ratio=args.gravity_ratio,
        mass_scale=args.mass_scale,
        friction_multiplier=args.friction_multiplier,
        max_torque_fraction=args.max_torque_fraction,
    )
    args.out.write_text(json.dumps(compensation_to_dict(artifact), indent=2) + "\n")
    args.out.with_suffix(".provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")


if __name__ == "__main__":
    main()
