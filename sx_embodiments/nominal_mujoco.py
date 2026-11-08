"""Portable nominal MuJoCo projection of an exact canonical body.

This optional exporter is standard-library only. It preserves description geometry and
inertials; a consumer compiles the result with MuJoCo. Control assumptions belong to an
explicit separately hashed profile, never to the physical body's drive facts. There is
no physical qualification, balance guarantee or automatic choice of support links here.
"""

from __future__ import annotations

import hashlib
import math
import os
import shutil
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from enum import StrEnum
from pathlib import Path
from typing import Literal

from sx_contracts.identity import canonical_json, content_digest

from .assets import description_asset_uri
from .embodiment import Embodiment
from .kinematics import JointType, Kinematics, Transform, from_xyz_rpy
from .layout import Bounds, CoordinateUnit
from .materialize import materialize


class NominalExportError(ValueError):
    """A source fact or explicit nominal assumption is absent or inconsistent."""


def _require(condition: object, message: str) -> None:
    if not condition:
        raise NominalExportError(message)


def _finite(values: tuple[float, ...]) -> bool:
    return all(math.isfinite(x) for x in values)


def _numbers(values: tuple[float, ...]) -> str:
    return " ".join(format(x, ".17g") for x in values)


@dataclass(frozen=True, slots=True)
class NominalJoint:
    name: str
    kp: float
    kd: float
    effort: float
    armature: float
    default_position: float
    action_scale: float

    def __post_init__(self) -> None:
        _require(
            self.name
            and _finite(
                (
                    self.kp,
                    self.kd,
                    self.effort,
                    self.armature,
                    self.default_position,
                    self.action_scale,
                )
            ),
            "Invalid nominal joint",
        )
        _require(
            self.kp > 0
            and self.kd >= 0
            and self.effort > 0
            and self.armature >= 0
            and self.action_scale > 0,
            "Invalid nominal control parameters",
        )


@dataclass(frozen=True, slots=True)
class SupportContact:
    link: str
    height_m: float

    def __post_init__(self) -> None:
        _require(
            self.link and math.isfinite(self.height_m) and self.height_m >= 0,
            "Contact requires an explicit link and nonnegative height",
        )


class CollisionMode(StrEnum):
    FULL = "full"
    GROUND_ONLY = "ground_only"


@dataclass(frozen=True, slots=True)
class NominalProfile:
    expected_embodiment_id: str
    joints: tuple[NominalJoint, ...]
    passive_holds: tuple[tuple[str, float], ...]
    tracked_links: tuple[str, ...]
    contacts: tuple[SupportContact, ...]
    root_position: tuple[float, float, float]
    root_quaternion: tuple[float, float, float, float]
    timestep: float
    gravity: tuple[float, float, float]
    friction: tuple[float, float, float]
    assumptions: tuple[str, ...]
    collision_mode: CollisionMode = CollisionMode.FULL

    def __post_init__(self) -> None:
        _require(isinstance(self.collision_mode, CollisionMode), "Explicit collision mode required")
        _require(
            isinstance(self.joints, tuple)
            and isinstance(self.passive_holds, tuple)
            and isinstance(self.tracked_links, tuple)
            and isinstance(self.contacts, tuple)
            and isinstance(self.assumptions, tuple),
            "Profiles require immutable tuples",
        )
        _require(
            self.joints and len({j.name for j in self.joints}) == len(self.joints),
            "Repeated or absent joint profile",
        )
        _require(
            all(
                isinstance(pair, tuple) and len(pair) == 2 and pair[0] and math.isfinite(pair[1])
                for pair in self.passive_holds
            )
            and len(dict(self.passive_holds)) == len(self.passive_holds),
            "Invalid passive holds",
        )
        _require(
            self.tracked_links and len(set(self.tracked_links)) == len(self.tracked_links),
            "Repeated or absent tracked links",
        )
        _require(
            self.contacts
            and len({c.link for c in self.contacts}) == len(self.contacts)
            and all(c.link in self.tracked_links for c in self.contacts),
            "Contact links must be distinct tracked links",
        )
        _require(
            self.assumptions and all(isinstance(a, str) and a.strip() for a in self.assumptions),
            "Document the nominal assumptions explicitly",
        )
        for values, length in (
            (self.root_position, 3),
            (self.root_quaternion, 4),
            (self.gravity, 3),
            (self.friction, 3),
        ):
            _require(
                isinstance(values, tuple) and len(values) == length and _finite(values),
                "Invalid profile vector",
            )
        _require(
            abs(sum(v * v for v in self.root_quaternion) - 1) <= 1e-9,
            "Root quaternion must be unit wxyz",
        )
        _require(
            math.isfinite(self.timestep)
            and self.timestep > 0
            and all(x >= 0 for x in self.friction),
            "Invalid timestep or friction",
        )

    @property
    def sha256(self) -> str:
        return str(content_digest(asdict(self)))


@dataclass(frozen=True, slots=True)
class ExportReceipt:
    embodiment_id: str
    urdf_sha256: str
    profile_sha256: str
    scene_sha256: str
    joint_names: tuple[str, ...]
    profile: NominalProfile
    files: tuple[tuple[str, str], ...]
    scope: str = "nominal simulation only; no physical qualification"
    schema: Literal["sx-embodiments.nominal-mujoco/v2"] = "sx-embodiments.nominal-mujoco/v2"
    rotation_convention: Literal["urdf-rpy-extrinsic-XYZ"] = "urdf-rpy-extrinsic-XYZ"

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def _origin(source: ET.Element | None, dest: ET.Element) -> None:
    if source is not None:
        dest.set("pos", source.get("xyz", "0 0 0"))
        dest.set("euler", source.get("rpy", "0 0 0"))


def _matrix_origin(matrix: Transform, dest: ET.Element) -> None:
    horizontal = math.hypot(matrix[0], matrix[4])
    pitch = math.atan2(-matrix[8], horizontal)
    if horizontal > 1e-12:
        roll, yaw = math.atan2(matrix[9], matrix[10]), math.atan2(matrix[4], matrix[0])
    else:
        roll, yaw = 0.0, math.atan2(-matrix[1], matrix[5])
    dest.set("pos", _numbers((matrix[3], matrix[7], matrix[11])))
    dest.set("euler", _numbers((roll, pitch, yaw)))


def export_nominal_mujoco(
    body: Embodiment, profile: NominalProfile, destination: Path
) -> ExportReceipt:
    """Write a new portable bundle atomically; never overwrite source or an existing bundle.

    Explicit passive holds are projected through canonical FK into fixed link origins.
    Every native joint remains independently actuated. Unsupported geometry, mimic
    coordinates and unaccounted motion refuse instead of substituting another robot.
    The compiled-model check belongs to the simulator consumer, before training.
    """
    _require(
        str(body.id) == profile.expected_embodiment_id, "Canonical embodiment identity mismatch"
    )
    names = tuple(body.joint_names)
    _require(
        tuple(j.name for j in profile.joints) == names,
        "Control profile must use exact native joint order",
    )
    urdf = body.urdf_bytes
    sha = hashlib.sha256(urdf).hexdigest()
    _require(sha == str(body.urdf.asset.content.sha256), "Canonical URDF hash mismatch")
    kin = Kinematics.of(urdf)
    _require(
        all(j.mimic is None for j in kin.joints.values()),
        "Mimic coordinates require an explicit supported projection",
    )
    _require(
        all(
            j.type in (JointType.FIXED, JointType.REVOLUTE, JointType.CONTINUOUS)
            for j in kin.joints.values()
        ),
        "Only angular scalar coordinates are supported",
    )
    passive = dict(profile.passive_holds)
    _require(
        set(kin.independent) == set(names) | set(passive) and not set(names) & set(passive),
        "Every non-native moving coordinate requires an explicit passive hold",
    )
    roots = kin.links - kin.parent_joint.keys()
    _require(len(roots) == 1, "Exactly one URDF root required")
    root_name = next(iter(roots))
    _require(
        set(profile.tracked_links) <= kin.links, "Tracked links must exist in the canonical body"
    )
    source = ET.fromstring(urdf)
    nodes = {j.attrib["name"]: j for j in source.findall("joint")}
    bounds: dict[str, tuple[float, float]] = {}
    for coordinate, nominal in zip(body.state.coordinates, profile.joints, strict=True):
        axis = coordinate.axis
        _require(
            axis.unit is CoordinateUnit.RADIAN and isinstance(axis.bounds, Bounds),
            "Bounded angular native state required",
        )
        if not isinstance(axis.bounds, Bounds):
            raise NominalExportError("Native coordinate bounds missing")
        bounds[nominal.name] = (axis.bounds.lower, axis.bounds.upper)
        _require(
            axis.bounds.lower <= nominal.default_position <= axis.bounds.upper,
            "Default position outside canonical limits",
        )
    for name in kin.independent:
        limit = nodes[name].find("limit")
        _require(
            limit is not None and "lower" in limit.attrib and "upper" in limit.attrib,
            f"{name}: limits absent",
        )
        if limit is None:
            raise NominalExportError("Limits absent")
        lo, hi = float(limit.attrib["lower"]), float(limit.attrib["upper"])
        _require(_finite((lo, hi)) and lo < hi, f"{name}: invalid limits")
        if name in bounds:
            _require(
                all(abs(a - b) <= 1e-9 for a, b in zip(bounds[name], (lo, hi), strict=True)),
                "Native and description bounds disagree",
            )
        else:
            _require(lo <= passive[name] <= hi, "Passive hold outside description limits")
    destination = Path(destination).absolute()
    _require(not destination.exists(), "Destination already exists")
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=".nominal-", dir=destination.parent))
    try:
        receipt = _render(body, profile, staging, source, kin, root_name, bounds, sha)
        os.rename(staging, destination)
        return receipt
    except BaseException:
        # Only our newly created derived staging directory is removed. Never source/raw.
        shutil.rmtree(staging)
        raise


def _render(
    body: Embodiment,
    profile: NominalProfile,
    staging: Path,
    source: ET.Element,
    kin: Kinematics,
    root_name: str,
    bounds: dict[str, tuple[float, float]],
    sha: str,
) -> ExportReceipt:
    assets = materialize(body).resolve()
    xml = ET.Element("mujoco", model=str(body.name))
    ET.SubElement(
        xml,
        "compiler",
        angle="radian",
        # URDF RPY rotates about fixed X, Y, Z axes. MuJoCo lowercase
        # xyz means moving axes and corrupts every mixed-axis origin.
        eulerseq="XYZ",
        inertiafromgeom="false",
        fusestatic="false",
        autolimits="true",
    )
    ET.SubElement(
        xml,
        "option",
        timestep=str(profile.timestep),
        gravity=_numbers(profile.gravity),
        integrator="implicitfast",
    )
    asset_xml = ET.SubElement(xml, "asset")
    world = ET.SubElement(xml, "worldbody")
    ET.SubElement(
        world,
        "geom",
        name="nominal_floor",
        contype="1",
        conaffinity="2" if profile.collision_mode is CollisionMode.GROUND_ONLY else "1",
        type="plane",
        size="0 0 0.1",
        friction=_numbers(profile.friction),
    )
    links = {node.attrib["name"]: node for node in source.findall("link")}
    nodes = {node.attrib["name"]: node for node in source.findall("joint")}
    nominal = {j.name: j for j in profile.joints}
    passive = dict(profile.passive_holds)
    mesh_names: dict[tuple[str, str], str] = {}
    files: dict[str, str] = {}

    def geometry(
        element: ET.Element, out: ET.Element, visual: bool, index: int, link_name: str
    ) -> None:
        geometry_node = element.find("geometry")
        _require(
            geometry_node is not None and len(geometry_node) == 1,
            "Geometry must contain exactly one shape",
        )
        if geometry_node is None:
            raise NominalExportError("Geometry missing")
        shape = geometry_node[0]
        g = ET.SubElement(
            out,
            "geom",
            name=f"{link_name}/{'visual' if visual else 'collision'}/{index}",
            contype="0"
            if visual
            else ("2" if profile.collision_mode is CollisionMode.GROUND_ONLY else "1"),
            conaffinity="0" if visual else "1",
            group="1" if visual else "0",
            friction=_numbers(profile.friction),
        )
        _origin(element.find("origin"), g)
        if shape.tag == "mesh":
            uri = description_asset_uri(body.urdf.uri, shape.attrib["filename"])
            prefix = "package://sx-embodiments/"
            _require(uri.startswith(prefix), "Mesh must resolve within canonical asset closure")
            path = (assets / uri.removeprefix(prefix)).resolve()
            _require(
                path.is_relative_to(assets) and path.is_file(), "Mesh missing from verified closure"
            )
            _require(
                path.suffix.lower() in (".stl", ".obj", ".msh"), "Unsupported MuJoCo mesh format"
            )
            data = path.read_bytes()
            digest = hashlib.sha256(data).hexdigest()
            rel = f"meshes/{digest}{path.suffix.lower()}"
            if rel not in files:
                (staging / "meshes").mkdir(exist_ok=True)
                (staging / rel).write_bytes(data)
                files[rel] = digest
            scale = shape.get("scale", "1 1 1")
            key = (rel, scale)
            if key not in mesh_names:
                mesh_names[key] = f"mesh{len(mesh_names)}"
                ET.SubElement(
                    asset_xml, "mesh", name=mesh_names[key], file=rel, scale=scale, inertia="shell"
                )
            g.set("type", "mesh")
            g.set("mesh", mesh_names[key])
        elif shape.tag == "box":
            g.set("type", "box")
            g.set("size", _numbers(tuple(float(v) / 2 for v in shape.attrib["size"].split())))
        elif shape.tag == "sphere":
            g.set("type", "sphere")
            g.set("size", shape.attrib["radius"])
        elif shape.tag == "cylinder":
            g.set("type", "cylinder")
            g.set("size", f"{shape.attrib['radius']} {float(shape.attrib['length']) / 2}")
        else:
            raise NominalExportError(f"Unsupported geometry {shape.tag}")
        color = element.find("material/color")
        if color is not None:
            g.set("rgba", color.attrib["rgba"])

    def link(name: str, parent: ET.Element) -> None:
        out = ET.SubElement(parent, "body", name=name)
        if name == root_name:
            out.set("pos", _numbers(profile.root_position))
            out.set("quat", _numbers(profile.root_quaternion))
            ET.SubElement(out, "freejoint", name="floating_root")
        else:
            joint = kin.joints[kin.parent_joint[name]]
            if joint.name in passive:
                _matrix_origin(kin.pose(joint.parent, joint.child, passive), out)
            else:
                _origin(nodes[joint.name].find("origin"), out)
            if joint.name in nominal:
                settings = nominal[joint.name]
                ET.SubElement(
                    out,
                    "joint",
                    name=joint.name,
                    type="hinge",
                    pos="0 0 0",
                    axis=_numbers(joint.axis),
                    range=_numbers(bounds[joint.name]),
                    armature=str(settings.armature),
                    damping="0",
                )
        inertial = links[name].find("inertial")
        if inertial is not None:
            mass, inertia = inertial.find("mass"), inertial.find("inertia")
            _require(mass is not None and inertia is not None, "Incomplete source inertial")
            if mass is None or inertia is None:
                raise NominalExportError("Incomplete inertial")
            mass_value = float(mass.attrib["value"])
            _require(math.isfinite(mass_value) and mass_value >= 0, "Invalid source mass")
            if mass_value > 0:
                values = [
                    float(inertia.attrib[k]) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
                ]
                _require(all(math.isfinite(v) for v in values), "Nonfinite source inertia")
                tensor = (
                    (values[0], values[3], values[4]),
                    (values[3], values[1], values[5]),
                    (values[4], values[5], values[2]),
                )
                origin = inertial.find("origin")
                angles = tuple(
                    float(v)
                    for v in ("0 0 0" if origin is None else origin.get("rpy", "0 0 0")).split()
                )
                _require(len(angles) == 3, "Invalid inertial rotation")
                rotation = from_xyz_rpy((0.0, 0.0, 0.0), (angles[0], angles[1], angles[2]))
                # MJCF fullinertia is expressed in the link frame; URDF's tensor is
                # expressed in the independently rotated inertial frame.
                rotated = tuple(
                    tuple(
                        sum(
                            rotation[4 * i + k] * tensor[k][column] * rotation[4 * j + column]
                            for k in range(3)
                            for column in range(3)
                        )
                        for j in range(3)
                    )
                    for i in range(3)
                )
                ET.SubElement(
                    out,
                    "inertial",
                    mass=str(mass_value),
                    pos="0 0 0" if origin is None else origin.get("xyz", "0 0 0"),
                    fullinertia=_numbers(
                        (
                            rotated[0][0],
                            rotated[1][1],
                            rotated[2][2],
                            rotated[0][1],
                            rotated[0][2],
                            rotated[1][2],
                        )
                    ),
                )
        for tag, visual in (("visual", True), ("collision", False)):
            for i, element in enumerate(links[name].findall(tag)):
                geometry(element, out, visual, i, name)
        for joint in kin.joints.values():
            if joint.parent == name:
                link(joint.child, out)

    link(root_name, world)
    actuators = ET.SubElement(xml, "actuator")
    for joint in profile.joints:
        ET.SubElement(
            actuators,
            "position",
            name=f"control/{joint.name}",
            joint=joint.name,
            kp=str(joint.kp),
            kv=str(joint.kd),
            ctrllimited="true",
            ctrlrange=_numbers(bounds[joint.name]),
            forcelimited="true",
            forcerange=_numbers((-joint.effort, joint.effort)),
        )
    scene = ET.tostring(xml, encoding="utf-8", xml_declaration=True)
    (staging / "scene.xml").write_bytes(scene)
    scene_sha = hashlib.sha256(scene).hexdigest()
    files["scene.xml"] = scene_sha
    (staging / "canonical.urdf").write_bytes(body.urdf_bytes)
    files["canonical.urdf"] = sha
    document = body.to_json().encode()
    (staging / "embodiment.json").write_bytes(document)
    files["embodiment.json"] = hashlib.sha256(document).hexdigest()
    receipt = ExportReceipt(
        str(body.id),
        sha,
        profile.sha256,
        scene_sha,
        tuple(body.joint_names),
        profile,
        tuple(sorted(files.items())),
    )
    (staging / "receipt.json").write_text(canonical_json(asdict(receipt)))
    return receipt


def nominal_profile_from_json(payload: bytes) -> NominalProfile:
    """Read the nominal artifact at its owner; unknown or duplicate fields refuse."""
    import json
    from dataclasses import fields

    from sx_contracts import decode

    def pairs(items: list[tuple[str, object]]) -> dict[str, object]:
        result: dict[str, object] = {}
        for key, value in items:
            _require(key not in result, "Duplicate nominal profile key")
            result[key] = value
        return result

    document = json.loads(payload, object_pairs_hook=pairs)
    # Initial exporter artifacts predate this explicit collision policy. Preserve
    # their original all-pairs semantics; newly serialized profiles name it.
    if isinstance(document, dict) and "collision_mode" not in document:
        document["collision_mode"] = CollisionMode.FULL.value
    raw = decode.exactly(
        document,
        {f.name for f in fields(NominalProfile)},
        where="nominal profile",
        error=NominalExportError,
    )
    joints = []
    for value in decode.sequence(raw, "joints"):
        row = decode.exactly(
            value,
            {f.name for f in fields(NominalJoint)},
            where="nominal joint",
            error=NominalExportError,
        )
        joints.append(
            NominalJoint(
                decode.text(row, "name"),
                decode.number(row, "kp"),
                decode.number(row, "kd"),
                decode.number(row, "effort"),
                decode.number(row, "armature"),
                decode.number(row, "default_position"),
                decode.number(row, "action_scale"),
            )
        )
    holds: list[tuple[str, float]] = []
    for value in decode.sequence(raw, "passive_holds"):
        _require(isinstance(value, (list, tuple)) and len(value) == 2, "Invalid passive hold")
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise NominalExportError("Invalid passive hold")
        row = decode.document({"name": value[0], "position": value[1]}, error=NominalExportError)
        holds.append((decode.text(row, "name"), decode.number(row, "position")))
    contacts = []
    for value in decode.sequence(raw, "contacts"):
        row = decode.exactly(
            value,
            {f.name for f in fields(SupportContact)},
            where="support contact",
            error=NominalExportError,
        )
        contacts.append(SupportContact(decode.text(row, "link"), decode.number(row, "height_m")))

    def triple(key: str) -> tuple[float, float, float]:
        vector = decode.numbers(raw, key)
        _require(len(vector) == 3, "Profile vector requires three numbers")
        return vector[0], vector[1], vector[2]

    quaternion = decode.numbers(raw, "root_quaternion")
    _require(len(quaternion) == 4, "Root quaternion needs four numbers")
    return NominalProfile(
        decode.text(raw, "expected_embodiment_id"),
        tuple(joints),
        tuple(holds),
        decode.texts(raw, "tracked_links"),
        tuple(contacts),
        triple("root_position"),
        (quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
        decode.number(raw, "timestep"),
        triple("gravity"),
        triple("friction"),
        decode.texts(raw, "assumptions"),
        CollisionMode(decode.text(raw, "collision_mode")),
    )
