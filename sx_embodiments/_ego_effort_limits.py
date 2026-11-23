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
class EffortLimit:
    joint: str
    unit: str
    maximum: float
    basis: str
    reflected_inertia_kg_m2: float | None


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
        j.attrib["name"]: float(limit.get("effort", "nan"))
        for j in ET.fromstring(source).findall("joint")  # nosec B314 - input robot description
        if (limit := j.find("limit")) is not None
    }
    rows: dict[str, EffortLimit] = {}
    for obj in decode.sequence(doc, "joints"):
        row = decode.exactly(obj, {f.name for f in fields(EffortLimit)}, where="joint effort limit")
        joint = decode.text(row, "joint")
        if joint in rows or joint not in tree.independent:
            raise ValueError(f"effort limits require a unique independent joint: {joint}")
        unit = decode.text(row, "unit")
        expected = "N" if tree.joints[joint].type == JointType.PRISMATIC else "Nm"
        if unit != expected:
            raise ValueError(f"{joint} effort needs units of {expected}")
        maximum = decode.number(row, "maximum")
        urdf_limit = limits.get(joint, 0)
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
        rows[joint] = EffortLimit(joint, unit, maximum, basis, rotor)
    if not rows:
        raise ValueError("effort limits contain no joint ratings")
    return EffortLimits(digest, tuple(rows.values()))
