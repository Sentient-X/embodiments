"""One immutable embodiment over one authoritative typed component graph."""

import dataclasses
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import cast

from sx_capabilities import CapabilityProfile, ComponentCapabilities

from .assets import (
    AssetFormat,
    AssetProvenance,
    AssetRef,
    AssetRole,
    EmbodiedAsset,
    PackagedAsset,
)
from .compose import (
    Component,
    ComponentRole,
    MountFrame,
    _EmbodimentDefinition,
    camera_bindings,
    state_space,
    validate_components,
)
from .curves import Curve1D
from .errors import AssetIntegrityError, EmbodimentSchemaError, LayoutError, MissingUrdfError
from .identity import EmbodimentId, EmbodimentKind, EmbodimentName, Lineage, PartId
from .layout import CoordinateUnit, StateSpace
from .parts import (
    ArmSpec,
    CameraBinding,
    CameraModality,
    CameraSpec,
    ControlRates,
    DeviceSpec,
    ForceTorqueSpec,
    GripperSpec,
    JointGroupSpec,
    LensProjection,
    MimicJoint,
    MobileBaseSpec,
    Part,
    PhysicalSpec,
    SensorModel,
)

SCHEMA_VERSION = 8


@dataclass(frozen=True, slots=True)
class Embodiment:
    """A complete hardware revision.

    ``components`` is the only stored morphology. State order, camera bindings,
    capabilities, and single-arm runtime facts are derived from that graph.
    ``id`` is the SHA-256 of the complete canonical document; friendly names are
    registry and catalog aliases, never a second revision identity.
    """

    name: EmbodimentName
    label: str
    kind: EmbodimentKind
    lineage: Lineage
    components: tuple[Component, ...]
    assets: tuple[EmbodiedAsset, ...]
    rates: ControlRates | None = None
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        if type(self.schema_version) is not int or self.schema_version != SCHEMA_VERSION:
            raise EmbodimentSchemaError(
                f"unsupported embodiment schema_version: {self.schema_version!r}"
            )
        if not str(self.name).strip():
            raise EmbodimentSchemaError("embodiment name must not be empty")
        if not self.label.strip():
            raise EmbodimentSchemaError("embodiment label must not be empty")
        if not self.lineage.family.strip():
            raise EmbodimentSchemaError("embodiment family must not be empty")
        if not self.assets:
            raise EmbodimentSchemaError("embodiment must reference at least one asset")
        identities = {
            (asset.uri, asset.sha256, asset.role, asset.logical_path) for asset in self.assets
        }
        if len(identities) != len(self.assets):
            raise EmbodimentSchemaError("embodiment contains duplicate asset references")
        urdfs = [
            asset
            for asset in self.assets
            if asset.format is AssetFormat.URDF and asset.role is AssetRole.DESCRIPTION
        ]
        if len(urdfs) != 1:
            raise MissingUrdfError(str(self.name), len(urdfs))
        validate_components(str(self.name), self.kind, self.components)

    @property
    def id(self) -> EmbodimentId:
        """The exact content-addressed identity of this revision."""
        return EmbodimentId(hashlib.sha256(self.canonical_json().encode("utf-8")).hexdigest())

    @property
    def state(self) -> StateSpace:
        return state_space(str(self.name), self.components)

    @property
    def cameras(self) -> tuple[CameraBinding, ...]:
        return camera_bindings(self.components)

    @property
    def capabilities(self) -> CapabilityProfile:
        return CapabilityProfile(
            tuple(
                ComponentCapabilities(component.component_id, component.capabilities)
                for component in self.components
                if component.capabilities
            )
        )

    @property
    def policy_hz(self) -> float | None:
        return self.rates.policy_hz if self.rates is not None else None

    @property
    def urdf(self) -> EmbodiedAsset:
        return next(
            asset
            for asset in self.assets
            if asset.format is AssetFormat.URDF and asset.role is AssetRole.DESCRIPTION
        )

    @property
    def urdf_path(self) -> Path:
        """Verified local path of the authoritative description asset."""
        return self.urdf.local_path()

    @property
    def urdf_bytes(self) -> bytes:
        """Verified bytes of the authoritative description asset."""
        return self.urdf.read_bytes()

    @property
    def single_arm(self) -> ArmSpec:
        arms = tuple(
            component.part
            for component in self.components
            if component.role is ComponentRole.BODY and isinstance(component.part, ArmSpec)
        )
        if len(arms) != 1:
            raise LayoutError(str(self.name), "operation requires exactly one arm")
        return arms[0]

    @property
    def single_gripper(self) -> GripperSpec:
        grippers = tuple(
            component.part
            for component in self.components
            if component.role is ComponentRole.BODY and isinstance(component.part, GripperSpec)
        )
        if len(grippers) != 1:
            raise LayoutError(str(self.name), "operation requires exactly one gripper")
        return grippers[0]

    @property
    def gripper_travel_m(self) -> tuple[float, float]:
        travel = self.single_gripper.travel_m
        if travel is None:
            raise LayoutError(str(self.name), "gripper travel is not declared")
        return travel

    @property
    def gripper_max_width_m(self) -> float:
        return self.gripper_travel_m[1]

    @property
    def has_mobile_base(self) -> bool:
        return any(
            component.role is ComponentRole.BODY and isinstance(component.part, MobileBaseSpec)
            for component in self.components
        )

    def with_assets(self, assets: tuple[AssetRef, ...], *, urdf: bytes) -> "Embodiment":
        """Return this morphology bound to another verified asset bundle."""
        _validate_urdf(self.name, assets, urdf)
        return dataclasses.replace(
            self, assets=tuple(EmbodiedAsset.from_ref(asset) for asset in assets)
        )

    def canonical_json(self) -> str:
        """Canonical content bytes used to derive ``id``; the id is not self-hashed."""
        return json.dumps(
            self._content_dict(), ensure_ascii=False, separators=(",", ":"), sort_keys=True
        )

    def to_dict(self) -> dict[str, object]:
        return {"id": str(self.id), **self._content_dict()}

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, separators=(",", ":"), sort_keys=True)

    def _content_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "name": str(self.name),
            "label": self.label,
            "kind": self.kind.value,
            "family": self.lineage.family,
            "variant": self.lineage.variant,
            "revision": self.lineage.revision,
            "components": [_component_to_dict(component) for component in self.components],
            "rates": (
                {"policy_hz": self.rates.policy_hz, "low_level_hz": self.rates.low_level_hz}
                if self.rates is not None
                else None
            ),
            "assets": [_asset_to_dict(asset) for asset in self.assets],
        }

    @classmethod
    def from_dict(cls, document: Mapping[str, object]) -> "Embodiment":
        _exact_keys(
            document,
            {
                "id",
                "schema_version",
                "name",
                "label",
                "kind",
                "family",
                "variant",
                "revision",
                "components",
                "rates",
                "assets",
            },
            "embodiment",
        )
        version = _require(document, "schema_version")
        if isinstance(version, bool) or not isinstance(version, int) or version != SCHEMA_VERSION:
            raise EmbodimentSchemaError(f"unsupported embodiment schema_version: {version!r}")
        raw_id = _string(document, "id", "embodiment")
        try:
            expected_id = EmbodimentId(raw_id)
            kind = EmbodimentKind(_string(document, "kind", "embodiment"))
        except ValueError as exc:
            raise EmbodimentSchemaError(f"invalid embodiment identity or kind: {exc}") from exc
        embodiment = cls(
            name=EmbodimentName(_string(document, "name", "embodiment")),
            label=_string(document, "label", "embodiment"),
            kind=kind,
            lineage=Lineage(
                family=_string(document, "family", "embodiment"),
                variant=_string(document, "variant", "embodiment"),
                revision=_string(document, "revision", "embodiment"),
            ),
            components=_parse_components(_require(document, "components")),
            rates=_parse_rates(_require(document, "rates")),
            assets=_parse_assets(_require(document, "assets")),
        )
        if embodiment.id != expected_id:
            raise EmbodimentSchemaError("embodiment id does not match its canonical content")
        return embodiment

    @classmethod
    def from_json(cls, value: str) -> "Embodiment":
        try:
            document: object = json.loads(value)
        except json.JSONDecodeError as exc:
            raise EmbodimentSchemaError("embodiment is not valid JSON") from exc
        if not isinstance(document, Mapping):
            raise EmbodimentSchemaError("embodiment JSON must contain an object")
        return cls.from_dict(cast(Mapping[str, object], document))


def _component_to_dict(component: Component) -> dict[str, object]:
    part = component.part
    wire: dict[str, object] = {
        "name": component.instance,
        "role": component.role.value,
        "parent": component.mount.parent_instance or None,
        "frame": component.mount.frame or component.instance,
        "part_id": str(part.part_id),
    }
    if isinstance(part, ArmSpec):
        return wire | {
            "type": "arm",
            "joints": _joint_rows(
                part.joint_names,
                part.joint_units,
                part.joint_lower,
                part.joint_upper,
                part.home_joints,
            ),
            "physical": _physical_to_dict(part.physical),
        }
    if isinstance(part, JointGroupSpec):
        return wire | {
            "type": "joint_group",
            "joints": _joint_rows(
                part.joint_names,
                part.joint_units,
                part.joint_lower,
                part.joint_upper,
                part.home_joints,
            ),
        }
    if isinstance(part, GripperSpec):
        return wire | {
            "type": "gripper",
            "joints": _joint_rows(
                part.joint_names, part.joint_units, part.joint_lower, part.joint_upper
            ),
            "travel_m": list(part.travel_m) if part.travel_m is not None else None,
            "mimic_joints": [
                {"name": mimic.joint_name, "of": mimic.of, "multiplier": mimic.multiplier}
                for mimic in part.mimic_joints
            ],
            "gap_curve": (
                {"input": list(part.gap_curve.x), "output": list(part.gap_curve.y)}
                if part.gap_curve is not None
                else None
            ),
            "physical": _physical_to_dict(part.physical),
        }
    if isinstance(part, CameraSpec):
        return wire | {
            "type": "camera",
            "sensor_model": part.model.value,
            "modality": part.modality.value,
            "fps": part.fps,
            "projection": part.projection.value,
            "resolution": list(part.resolution) if part.resolution is not None else None,
        }
    if isinstance(part, MobileBaseSpec):
        return wire | {
            "type": "mobile_base",
            "channels": [
                {"name": name, "unit": unit.value}
                for name, unit in zip(part.channel_names, part.channel_units, strict=True)
            ],
        }
    if isinstance(part, ForceTorqueSpec):
        return wire | {"type": "force_torque", "rate_hz": part.rate_hz}
    if isinstance(part, DeviceSpec):
        return wire | {"type": "device", "description": part.description}
    raise EmbodimentSchemaError(f"unsupported component part {part!r}")


def _parse_components(raw: object) -> tuple[Component, ...]:
    if not isinstance(raw, list):
        raise EmbodimentSchemaError("components must be a list")
    return tuple(
        _parse_component(item, f"components[{index}]")
        for index, item in enumerate(cast(list[object], raw))
    )


def _parse_component(raw: object, label: str) -> Component:
    entry = _mapping(raw, label)
    common = {"name", "role", "parent", "frame", "part_id", "type"}
    kind = _string(entry, "type", label)
    expected_by_kind = {
        "arm": common | {"joints", "physical"},
        "joint_group": common | {"joints"},
        "gripper": common | {"joints", "travel_m", "mimic_joints", "gap_curve", "physical"},
        "camera": common | {"sensor_model", "modality", "fps", "projection", "resolution"},
        "mobile_base": common | {"channels"},
        "force_torque": common | {"rate_hz"},
        "device": common | {"description"},
    }
    expected = expected_by_kind.get(kind)
    if expected is None:
        raise EmbodimentSchemaError(f"{label}.type is unknown: {kind!r}")
    _exact_keys(entry, expected, label)
    part_id = PartId(_string(entry, "part_id", label))
    try:
        part = _parse_part(kind, part_id, entry, label)
        role = ComponentRole(_string(entry, "role", label))
    except ValueError as exc:
        raise EmbodimentSchemaError(f"invalid component vocabulary in {label}") from exc
    parent = _require(entry, "parent")
    if parent is not None and not isinstance(parent, str):
        raise EmbodimentSchemaError(f"{label}.parent must be a string or null")
    return Component(
        instance=_string(entry, "name", label),
        part=part,
        role=role,
        mount=MountFrame(
            parent_instance=parent or "",
            frame=_string(entry, "frame", label),
        ),
    )


def _parse_part(kind: str, part_id: PartId, entry: Mapping[str, object], label: str) -> Part:
    if kind in {"arm", "joint_group", "gripper"}:
        names, units, lower, upper, homes = _parse_joints(_require(entry, "joints"), label)
        if kind == "arm":
            return ArmSpec(
                part_id,
                names,
                units,
                lower,
                upper,
                _require_homes(homes, label),
                physical=_parse_physical(entry["physical"], label),
            )
        if kind == "joint_group":
            return JointGroupSpec(
                part_id, names, units, lower, upper, _require_homes(homes, label)
            )
        return GripperSpec(
            part_id,
            names,
            units,
            lower,
            upper,
            travel_m=_parse_pair(entry["travel_m"], f"{label}.travel_m"),
            mimic_joints=_parse_mimics(entry["mimic_joints"], label),
            gap_curve=_parse_curve(entry["gap_curve"], label),
            physical=_parse_physical(entry["physical"], label),
        )
    if kind == "camera":
        return CameraSpec(
            part_id,
            SensorModel(_string(entry, "sensor_model", label)),
            CameraModality(_string(entry, "modality", label)),
            _number(entry["fps"], f"{label}.fps"),
            LensProjection(_string(entry, "projection", label)),
            _parse_resolution(entry["resolution"], label),
        )
    if kind == "mobile_base":
        names, units = _parse_base_channels(entry["channels"], label)
        return MobileBaseSpec(part_id, names, units)
    if kind == "force_torque":
        return ForceTorqueSpec(part_id, _optional_number(entry["rate_hz"], f"{label}.rate_hz"))
    return DeviceSpec(part_id, _string(entry, "description", label))


def _joint_rows(
    names: tuple[str, ...],
    units: tuple[CoordinateUnit, ...],
    lower: tuple[float, ...],
    upper: tuple[float, ...],
    homes: tuple[float, ...] | None = None,
) -> list[dict[str, object]]:
    return [
        {
            "name": name,
            "unit": units[index].value,
            "lower": lo,
            "upper": hi,
            **({"home": homes[index]} if homes is not None else {}),
        }
        for index, (name, lo, hi) in enumerate(zip(names, lower, upper, strict=True))
    ]


def _parse_joints(
    raw: object, label: str
) -> tuple[
    tuple[str, ...],
    tuple[CoordinateUnit, ...],
    tuple[float, ...],
    tuple[float, ...],
    tuple[float | None, ...],
]:
    if not isinstance(raw, list) or not raw:
        raise EmbodimentSchemaError(f"{label}.joints must be a non-empty list")
    rows = tuple(_mapping(row, f"{label}.joints") for row in cast(list[object], raw))
    for row in rows:
        if set(row) not in (
            {"name", "unit", "lower", "upper"},
            {"name", "unit", "lower", "upper", "home"},
        ):
            raise EmbodimentSchemaError(f"{label}.joints has invalid fields")
    try:
        units = tuple(
            CoordinateUnit(_string(row, "unit", f"{label}.joints")) for row in rows
        )
    except ValueError as exc:
        raise EmbodimentSchemaError(f"{label}.joints contains an unknown unit") from exc
    return (
        tuple(_string(row, "name", f"{label}.joints") for row in rows),
        units,
        tuple(_number(row["lower"], f"{label}.joints.lower") for row in rows),
        tuple(_number(row["upper"], f"{label}.joints.upper") for row in rows),
        tuple(_optional_number(row.get("home"), f"{label}.joints.home") for row in rows),
    )


def _parse_base_channels(
    raw: object, label: str
) -> tuple[tuple[str, ...], tuple[CoordinateUnit, ...]]:
    rows = tuple(_mapping(row, f"{label}.channels") for row in _list(raw, label))
    for row in rows:
        _exact_keys(row, {"name", "unit"}, f"{label}.channels")
    try:
        units = tuple(
            CoordinateUnit(_string(row, "unit", f"{label}.channels")) for row in rows
        )
    except ValueError as exc:
        raise EmbodimentSchemaError(f"{label}.channels contains an unknown unit") from exc
    return (
        tuple(_string(row, "name", f"{label}.channels") for row in rows),
        units,
    )


def _require_homes(values: tuple[float | None, ...], label: str) -> tuple[float, ...]:
    if any(value is None for value in values):
        raise EmbodimentSchemaError(f"{label}.joints require home values")
    return cast(tuple[float, ...], values)


def _physical_to_dict(value: PhysicalSpec | None) -> dict[str, float | None] | None:
    if value is None:
        return None
    return {"payload_kg": value.payload_kg, "reach_m": value.reach_m, "mass_kg": value.mass_kg}


def _parse_physical(raw: object, label: str) -> PhysicalSpec | None:
    if raw is None:
        return None
    entry = _mapping(raw, f"{label}.physical")
    _exact_keys(entry, {"payload_kg", "reach_m", "mass_kg"}, f"{label}.physical")
    return PhysicalSpec(
        payload_kg=_optional_number(entry["payload_kg"], f"{label}.physical.payload_kg"),
        reach_m=_optional_number(entry["reach_m"], f"{label}.physical.reach_m"),
        mass_kg=_optional_number(entry["mass_kg"], f"{label}.physical.mass_kg"),
    )


def _parse_mimics(raw: object, label: str) -> tuple[MimicJoint, ...]:
    if not isinstance(raw, list):
        raise EmbodimentSchemaError(f"{label}.mimic_joints must be a list")
    result: list[MimicJoint] = []
    for item in cast(list[object], raw):
        entry = _mapping(item, f"{label}.mimic_joints")
        _exact_keys(entry, {"name", "of", "multiplier"}, f"{label}.mimic_joints")
        result.append(
            MimicJoint(
                _string(entry, "name", label),
                _string(entry, "of", label),
                _number(entry["multiplier"], f"{label}.mimic_joints.multiplier"),
            )
        )
    return tuple(result)


def _parse_curve(raw: object, label: str) -> Curve1D | None:
    if raw is None:
        return None
    entry = _mapping(raw, f"{label}.gap_curve")
    _exact_keys(entry, {"input", "output"}, f"{label}.gap_curve")
    return Curve1D(
        tuple(_number(value, f"{label}.gap_curve.input") for value in _list(entry["input"], label)),
        tuple(
            _number(value, f"{label}.gap_curve.output") for value in _list(entry["output"], label)
        ),
    )


def _asset_to_dict(asset: EmbodiedAsset) -> dict[str, object]:
    provenance = asset.provenance
    return {
        "uri": asset.uri,
        "sha256": asset.sha256,
        "format": asset.format.value,
        "role": asset.role.value,
        "media_type": asset.media_type,
        "byte_size": asset.byte_size,
        **({"logical_path": str(asset.logical_path)} if asset.logical_path is not None else {}),
        "provenance": {
            "repository": provenance.repository,
            "revision": provenance.revision,
            "path": provenance.path,
            "license_id": provenance.license_id,
            "generator": provenance.generator,
        },
    }


def _parse_assets(raw: object) -> tuple[EmbodiedAsset, ...]:
    if not isinstance(raw, list):
        raise EmbodimentSchemaError("assets must be a list")
    return tuple(EmbodiedAsset.from_ref(_parse_asset(item)) for item in cast(list[object], raw))


def _parse_asset(raw: object) -> AssetRef:
    entry = _mapping(raw, "asset")
    required = {"uri", "sha256", "format", "role", "media_type", "byte_size", "provenance"}
    _allowed_keys(entry, required, required | {"logical_path"}, "asset")
    provenance_entry = _mapping(entry["provenance"], "asset.provenance")
    _exact_keys(
        provenance_entry,
        {"repository", "revision", "path", "license_id", "generator"},
        "asset.provenance",
    )
    generator = provenance_entry["generator"]
    if generator is not None and not isinstance(generator, str):
        raise EmbodimentSchemaError("asset.provenance.generator must be a string or null")
    logical_path = entry.get("logical_path")
    if logical_path is not None and not isinstance(logical_path, str):
        raise EmbodimentSchemaError("asset.logical_path must be a string or null")
    media_type = entry["media_type"]
    if media_type is not None and not isinstance(media_type, str):
        raise EmbodimentSchemaError("asset.media_type must be a string or null")
    try:
        return AssetRef(
            uri=_string(entry, "uri", "asset"),
            sha256=_string(entry, "sha256", "asset"),
            format=AssetFormat(_string(entry, "format", "asset")),
            role=AssetRole(_string(entry, "role", "asset")),
            media_type=media_type,
            byte_size=_optional_int(entry["byte_size"], "asset.byte_size"),
            logical_path=PurePosixPath(logical_path) if logical_path is not None else None,
            provenance=AssetProvenance(
                repository=_string(provenance_entry, "repository", "asset.provenance"),
                revision=_string(provenance_entry, "revision", "asset.provenance"),
                path=_string(provenance_entry, "path", "asset.provenance"),
                license_id=_string(provenance_entry, "license_id", "asset.provenance"),
                generator=generator,
            ),
        )
    except (AssetIntegrityError, ValueError) as exc:
        raise EmbodimentSchemaError(f"invalid asset: {exc}") from exc


def _parse_rates(raw: object) -> ControlRates | None:
    if raw is None:
        return None
    entry = _mapping(raw, "rates")
    _exact_keys(entry, {"policy_hz", "low_level_hz"}, "rates")
    return ControlRates(
        policy_hz=_number(entry["policy_hz"], "rates.policy_hz"),
        low_level_hz=_optional_number(entry["low_level_hz"], "rates.low_level_hz"),
    )


def _parse_resolution(raw: object, label: str) -> tuple[int, int] | None:
    if raw is None:
        return None
    values = _list(raw, f"{label}.resolution")
    if len(values) != 2 or any(
        isinstance(value, bool) or not isinstance(value, int) for value in values
    ):
        raise EmbodimentSchemaError(f"{label}.resolution must contain two integers")
    width, height = cast(list[int], values)
    if width <= 0 or height <= 0:
        raise EmbodimentSchemaError(f"{label}.resolution must be positive")
    return width, height


def _parse_pair(raw: object, label: str) -> tuple[float, float] | None:
    if raw is None:
        return None
    values = _list(raw, label)
    if len(values) != 2:
        raise EmbodimentSchemaError(f"{label} must contain two numbers")
    return _number(values[0], label), _number(values[1], label)


def _portable_component(component: Component) -> Component:
    part = component.part
    if isinstance(part, ArmSpec | JointGroupSpec | GripperSpec | MobileBaseSpec):
        part = dataclasses.replace(part, assets=())
    mount = component.mount
    if not mount.frame:
        mount = dataclasses.replace(mount, frame=component.instance)
    return dataclasses.replace(component, part=part, mount=mount)


def embodiment_from_definition(definition: _EmbodimentDefinition) -> Embodiment:
    refs: list[AssetRef] = []
    seen: set[tuple[str, str]] = set()
    for asset in _packaged_assets(definition):
        key = (asset.relpath, asset.sha256)
        if key not in seen:
            refs.append(asset.ref())
            seen.add(key)
    urdf = authoritative_urdf(definition)
    _validate_urdf(definition.embodiment_id, tuple(refs), urdf.path().read_bytes())
    return Embodiment(
        name=definition.embodiment_id,
        label=definition.name,
        kind=definition.kind,
        lineage=definition.lineage,
        components=tuple(_portable_component(component) for component in definition.attachments),
        assets=tuple(EmbodiedAsset.from_ref(ref) for ref in refs),
        rates=definition.rates,
    )


def authoritative_urdf(definition: _EmbodimentDefinition) -> PackagedAsset:
    matches = tuple(
        {
            (asset.relpath, asset.sha256): asset
            for asset in _packaged_assets(definition)
            if asset.format is AssetFormat.URDF and asset.role is AssetRole.DESCRIPTION
        }.values()
    )
    if len(matches) != 1:
        raise MissingUrdfError(str(definition.embodiment_id), len(matches))
    return matches[0]


def _packaged_assets(definition: _EmbodimentDefinition) -> list[PackagedAsset]:
    assets: list[PackagedAsset] = []
    for component in definition.attachments:
        part = component.part
        if isinstance(part, ArmSpec | JointGroupSpec | GripperSpec | MobileBaseSpec):
            assets.extend(part.assets)
    assets.extend(definition.extra_assets)
    return assets


def _validate_urdf(name: EmbodimentName, assets: tuple[AssetRef, ...], urdf: bytes) -> None:
    urdfs = tuple(
        asset
        for asset in assets
        if asset.format is AssetFormat.URDF and asset.role is AssetRole.DESCRIPTION
    )
    if len(urdfs) != 1:
        raise MissingUrdfError(str(name), len(urdfs))
    actual = hashlib.sha256(urdf).hexdigest()
    if actual != urdfs[0].sha256:
        raise AssetIntegrityError(
            f"{urdfs[0].uri}: authoritative URDF bytes have sha256 {actual}, "
            f"expected {urdfs[0].sha256}"
        )
    try:
        root = ET.fromstring(urdf)
    except ET.ParseError as exc:
        raise EmbodimentSchemaError(f"{name}: authoritative URDF is invalid XML") from exc
    if root.tag != "robot" or not root.attrib.get("name", "").strip():
        raise EmbodimentSchemaError(f"{name}: authoritative URDF root must be a named <robot>")


def _require(document: Mapping[str, object], key: str) -> object:
    if key not in document:
        raise EmbodimentSchemaError(f"missing required key {key!r}")
    return document[key]


def _mapping(raw: object, label: str) -> Mapping[str, object]:
    if not isinstance(raw, Mapping):
        raise EmbodimentSchemaError(f"{label} must be an object")
    return cast(Mapping[str, object], raw)


def _list(raw: object, label: str) -> list[object]:
    if not isinstance(raw, list):
        raise EmbodimentSchemaError(f"{label} must be a list")
    return cast(list[object], raw)


def _string(document: Mapping[str, object], key: str, label: str) -> str:
    value = _require(document, key)
    if not isinstance(value, str):
        raise EmbodimentSchemaError(f"{label}.{key} must be a string")
    return value


def _string_list(raw: object, label: str) -> tuple[str, ...]:
    values = _list(raw, label)
    if any(not isinstance(value, str) for value in values):
        raise EmbodimentSchemaError(f"{label} must contain strings")
    return tuple(cast(list[str], values))


def _number(raw: object, label: str) -> float:
    if isinstance(raw, bool) or not isinstance(raw, int | float):
        raise EmbodimentSchemaError(f"{label} must be a number")
    value = float(raw)
    if not math.isfinite(value):
        raise EmbodimentSchemaError(f"{label} must be finite")
    return value


def _optional_number(raw: object, label: str) -> float | None:
    return None if raw is None else _number(raw, label)


def _optional_int(raw: object, label: str) -> int | None:
    if raw is None:
        return None
    if isinstance(raw, bool) or not isinstance(raw, int):
        raise EmbodimentSchemaError(f"{label} must be an integer or null")
    return raw


def _exact_keys(document: Mapping[str, object], expected: set[str], label: str) -> None:
    actual = set(document)
    if actual != expected:
        raise EmbodimentSchemaError(
            f"{label} keys differ: missing={sorted(expected - actual)}, "
            f"unknown={sorted(actual - expected)}"
        )


def _allowed_keys(
    document: Mapping[str, object], required: set[str], allowed: set[str], label: str
) -> None:
    actual = set(document)
    if not required <= actual or not actual <= allowed:
        raise EmbodimentSchemaError(
            f"{label} keys differ: missing={sorted(required - actual)}, "
            f"unknown={sorted(actual - allowed)}"
        )
