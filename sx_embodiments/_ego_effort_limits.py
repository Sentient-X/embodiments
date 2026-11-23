"""Source-bound joint effort budgets for unloaded offline trajectory fitting."""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass, fields
from pathlib import Path

from sx_contracts import decode

from .kinematics import JointType, Kinematics


@dataclass(frozen=True, slots=True)
class OperatingVelocity:
    maximum: float
    unit: str
    basis: str

    @classmethod
    def read(cls, value: object, expected: str, urdf_limit: float) -> OperatingVelocity:
        doc = decode.exactly(value, {f.name for f in fields(cls)}, where="operating velocity")
        maximum = decode.number(doc, "maximum")
        unit = decode.text(doc, "unit")
        basis = decode.text(doc, "basis").strip()
        if unit != expected or not basis:
            raise ValueError(f"operating velocity needs {expected} units and a source basis")
        if maximum <= 0 or (urdf_limit > 0 and maximum > urdf_limit):
            raise ValueError("operating velocity must be positive and cannot loosen a URDF limit")
        return cls(maximum, unit, basis)


@dataclass(frozen=True, slots=True)
class EffortLimit:
    joint: str
    unit: str
    maximum: float
    basis: str
    reflected_inertia_kg_m2: float | None
    operating_velocity: OperatingVelocity | None = None


@dataclass(frozen=True, slots=True)
class EffortLimits:
    urdf_sha256: str
    joints: tuple[EffortLimit, ...]


def load_effort_limits(path: Path, urdf: Path) -> EffortLimits:
    """Refuse stale geometry, invalid axes or units, and limits above the URDF ceiling."""

    def unique(pairs: list[tuple[str, object]]) -> dict[str, object]:
        result = dict(pairs)
        if len(result) != len(pairs):
            raise ValueError("duplicate key in effort limits")
        return result

    doc = decode.exactly(
        json.loads(path.read_bytes(), object_pairs_hook=unique),
        {"urdf_sha256", "joints"},
        where="effort limits",
    )
    source = urdf.read_bytes()
    digest = decode.text(doc, "urdf_sha256")
    if digest != hashlib.sha256(source).hexdigest():
        raise ValueError("effort limits do not match the URDF SHA-256")
    tree = Kinematics.of(source)
    limits = {
        j.attrib["name"]: limit
        for j in ET.fromstring(source).findall("joint")  # nosec B314 - input robot description
        if (limit := j.find("limit")) is not None
    }
    rows: dict[str, EffortLimit] = {}
    for obj in decode.sequence(doc, "joints"):
        raw = decode.document(obj, where="joint effort limit")
        # Existing source-bound ratings predate the optional governor evidence.
        row = decode.exactly(
            {"operating_velocity": None, **raw},
            {f.name for f in fields(EffortLimit)},
            where="joint effort limit",
        )
        joint = decode.text(row, "joint")
        if joint in rows or joint not in tree.independent:
            raise ValueError(f"effort limits require a unique independent joint: {joint}")
        unit = decode.text(row, "unit")
        expected = "N" if tree.joints[joint].type == JointType.PRISMATIC else "Nm"
        if unit != expected:
            raise ValueError(f"{joint} effort needs units of {expected}")
        maximum = decode.number(row, "maximum")
        limit = limits.get(joint)
        urdf_limit = float(limit.get("effort", "nan")) if limit is not None else 0
        if maximum <= 0 or (urdf_limit > 0 and maximum > urdf_limit):
            raise ValueError(f"{joint} effort must be positive and cannot loosen a URDF limit")
        basis = decode.text(row, "basis").strip()
        if not basis:
            raise ValueError(f"{joint} effort needs its source and rating basis")
        rotor = decode.optional_number(row, "reflected_inertia_kg_m2")
        if rotor is not None and (rotor <= 0 or unit != "Nm"):
            raise ValueError(
                f"{joint} reflected rotational inertia requires a positive kg m² value"
            )
        velocity = None
        if row["operating_velocity"] is not None:
            velocity = OperatingVelocity.read(
                row["operating_velocity"],
                "m/s" if expected == "N" else "rad/s",
                float(limit.get("velocity", "nan")) if limit is not None else 0,
            )
        rows[joint] = EffortLimit(joint, unit, maximum, basis, rotor, velocity)
    if not rows:
        raise ValueError("effort limits contain no joint ratings")
    return EffortLimits(digest, tuple(rows.values()))
