"""A quasi-static compensation model bound to one description and one resolved drive.

The source description may use different joint frames. Each affine map owns both
its position conversion and the power-preserving torque conversion. This artifact
contains no actuator access and does not qualify source inertia for dynamic control.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Final, cast

from sx_contracts import Sha256Digest
from sx_contracts.identity import JsonValue

COMPENSATION_SCHEMA: Final = "sx.quasi-static-compensation/v1"
COMPENSATION_NAME: Final = "teleop-compensation"


class CompensationError(ValueError):
    """The artifact does not describe a finite, uniquely mapped compensation."""


@dataclass(frozen=True)
class CompensationJoint:
    joint: str
    source_joint: str
    scale: float
    offset: float
    coulomb: float
    velocity_deadband: float
    transition_velocity: float
    max_torque: float

    def __post_init__(self) -> None:
        if not self.joint or not self.source_joint:
            raise CompensationError("joint names are nonempty")
        if not all(
            math.isfinite(value)
            for value in (
                self.scale,
                self.offset,
                self.coulomb,
                self.velocity_deadband,
                self.transition_velocity,
                self.max_torque,
            )
        ):
            raise CompensationError("compensation joint values are finite")
        if self.scale == 0 or self.coulomb < 0 or self.velocity_deadband < 0:
            raise CompensationError("scale is nonzero; friction and deadband are nonnegative")
        if self.transition_velocity <= 0 or self.max_torque <= 0:
            raise CompensationError("transition velocity and maximum torque are positive")


@dataclass(frozen=True)
class QuasiStaticCompensation:
    description_sha256: Sha256Digest
    drive_sha256: Sha256Digest
    source_urdf: str
    units: tuple[str, ...]
    gravity_scale: float
    torque_slew_nm_s: float
    joints: tuple[CompensationJoint, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "description_sha256", Sha256Digest(self.description_sha256))
        object.__setattr__(self, "drive_sha256", Sha256Digest(self.drive_sha256))
        if not math.isfinite(self.gravity_scale) or not 0 < self.gravity_scale <= 1:
            raise CompensationError("gravity scale is greater than zero and at most one")
        if not math.isfinite(self.torque_slew_nm_s) or self.torque_slew_nm_s <= 0:
            raise CompensationError("torque slew is positive and finite")
        if (
            not self.units
            or list(self.units) != sorted(set(self.units))
            or any(not unit or any(c.isspace() for c in unit) for unit in self.units)
        ):
            raise CompensationError("units are nonempty, unique sorted bus identities")
        if not self.joints:
            raise CompensationError("compensation names at least one joint")
        for names in (
            [item.joint for item in self.joints],
            [item.source_joint for item in self.joints],
        ):
            if len(names) != len(set(names)):
                raise CompensationError("canonical and source joints are unique")
        try:
            root = ET.fromstring(self.source_urdf)
        except ET.ParseError as error:
            raise CompensationError("source URDF is malformed") from error
        if root.tag != "robot":
            raise CompensationError("source description is a URDF robot")
        source_joints = {
            joint.attrib.get("name")
            for joint in root.findall("joint")
            if joint.attrib.get("type") != "fixed"
        }
        if source_joints != {item.source_joint for item in self.joints}:
            raise CompensationError("every source degree of freedom is mapped exactly once")


def _object(value: object, field: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise CompensationError(f"{field} is an object")
    parsed = cast("Mapping[object, object]", value)
    if any(not isinstance(key, str) for key in parsed):
        raise CompensationError(f"{field} has nontext keys")
    return cast("Mapping[str, object]", parsed)


def _text(document: Mapping[str, object], field: str) -> str:
    value = document.get(field)
    if not isinstance(value, str) or not value:
        raise CompensationError(f"{field} is nonempty text")
    return value


def _number(document: Mapping[str, object], field: str) -> float:
    value = document.get(field)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise CompensationError(f"{field} is a finite number")
    try:
        number = float(value)
    except OverflowError as error:
        raise CompensationError(f"{field} is a finite number") from error
    if not math.isfinite(number):
        raise CompensationError(f"{field} is a finite number")
    return number


def compensation_from_dict(document: Mapping[str, object]) -> QuasiStaticCompensation:
    """Parse artifact bytes once; refuse unknown fields and ambiguous mappings."""
    fields = {
        "schema",
        "description_sha256",
        "drive_sha256",
        "source_urdf",
        "gravity_scale",
        "torque_slew_nm_s",
        "joints",
        "units",
    }
    if set(document) != fields or document.get("schema") != COMPENSATION_SCHEMA:
        raise CompensationError("not an exact quasi-static compensation document")
    raw_units = document.get("units")
    if not isinstance(raw_units, list):
        raise CompensationError("units is a list of bus identities")
    units: list[str] = []
    for unit in cast("list[object]", raw_units):
        if not isinstance(unit, str):
            raise CompensationError("units contains only bus identities")
        units.append(unit)
    raw_joints = document.get("joints")
    if not isinstance(raw_joints, list):
        raise CompensationError("joints is a list")
    joints: list[CompensationJoint] = []
    joint_fields = {
        "joint",
        "source_joint",
        "scale",
        "offset",
        "coulomb",
        "velocity_deadband",
        "transition_velocity",
        "max_torque",
    }
    for raw in cast("list[object]", raw_joints):
        item = _object(raw, "joint")
        if set(item) != joint_fields:
            raise CompensationError("compensation joint has missing or unknown fields")
        joints.append(
            CompensationJoint(
                joint=_text(item, "joint"),
                source_joint=_text(item, "source_joint"),
                scale=_number(item, "scale"),
                offset=_number(item, "offset"),
                coulomb=_number(item, "coulomb"),
                velocity_deadband=_number(item, "velocity_deadband"),
                transition_velocity=_number(item, "transition_velocity"),
                max_torque=_number(item, "max_torque"),
            )
        )
    try:
        return QuasiStaticCompensation(
            description_sha256=Sha256Digest(_text(document, "description_sha256")),
            drive_sha256=Sha256Digest(_text(document, "drive_sha256")),
            source_urdf=_text(document, "source_urdf"),
            units=tuple(units),
            gravity_scale=_number(document, "gravity_scale"),
            torque_slew_nm_s=_number(document, "torque_slew_nm_s"),
            joints=tuple(joints),
        )
    except ValueError as error:
        raise CompensationError(str(error)) from error


def compensation_to_dict(compensation: QuasiStaticCompensation) -> dict[str, JsonValue]:
    return {
        "schema": COMPENSATION_SCHEMA,
        "description_sha256": compensation.description_sha256,
        "drive_sha256": compensation.drive_sha256,
        "source_urdf": compensation.source_urdf,
        "units": list(compensation.units),
        "gravity_scale": compensation.gravity_scale,
        "torque_slew_nm_s": compensation.torque_slew_nm_s,
        "joints": [
            {
                "joint": item.joint,
                "source_joint": item.source_joint,
                "scale": item.scale,
                "offset": item.offset,
                "coulomb": item.coulomb,
                "velocity_deadband": item.velocity_deadband,
                "transition_velocity": item.transition_velocity,
                "max_torque": item.max_torque,
            }
            for item in compensation.joints
        ],
    }
