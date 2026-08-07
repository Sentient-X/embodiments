"""One immutable embodiment over one authoritative typed component graph."""

from __future__ import annotations

import dataclasses
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import cast

from sx_contracts import CapabilityProfile, CapabilitySet, ComponentCapabilities
from sx_contracts.assets import (
    AssetFormat,
    AssetIntegrityError,
    AssetProvenance,
    AssetRef,
    AssetRole,
    ProvenancedAsset,
)
from sx_contracts.content import ContentBlob, Sha256Digest
from sx_contracts.identity import canonical_json as canonical_document
from sx_contracts.identity import content_id

from .assets import PackagedAsset, resolve_asset
from .compose import (
    BaseMount,
    BodyAttachment,
    Component,
    ComponentRole,
    EmbodimentDefinition,
    LeaderAttachment,
    MountedOn,
    MountKind,
    RootMount,
    SensorAttachment,
    camera_bindings,
    state_space,
    validate_components,
)
from .curves import Curve1D, Knot
from .errors import EmbodimentSchemaError, LayoutError, MissingUrdfError
from .identity import EmbodimentId, EmbodimentKind, EmbodimentName, Lineage, PartId
from .layout import Bounds, CoordinateUnit, JointAxis, JointLayout, StateSpace, Unbounded
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

SCHEMA_VERSION = 11


@dataclass(frozen=True, slots=True)
class Embodiment:
    """A complete, content-addressed hardware revision.

    The component graph is the sole morphology. State order, roles, camera bindings,
    capabilities, and single-arm projections are derived from it. Friendly names are
    catalog aliases; ``id`` is the digest of the complete schema-11 document.

    **Construction is registry-internal.** Outside this package an embodiment is obtained,
    never assembled: ``embodiments[name]`` for a registered revision, ``from_dict``/
    ``from_json`` for a stored document, ``with_assets`` for external-corpus ingest. The
    arguments this constructor requires — components, lineage, kind, packaged assets — are
    not on the public surface (see :mod:`sx_embodiments`), so a new revision is a change to
    :mod:`sx_embodiments.known`, not a literal at a call site. Hardware facts have one
    owner; a second construction site is a second source of truth.
    """

    name: EmbodimentName
    label: str
    kind: EmbodimentKind
    lineage: Lineage
    components: tuple[Component, ...]
    assets: tuple[ProvenancedAsset, ...]
    rates: ControlRates | None = None
    base_mount: BaseMount | None = None
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
            (
                asset.asset.location,
                asset.asset.content,
                asset.asset.role,
                asset.asset.logical_path,
                asset.provenance,
            )
            for asset in self.assets
        }
        if len(identities) != len(self.assets):
            raise EmbodimentSchemaError("embodiment contains duplicate asset references")
        urdfs = tuple(
            asset
            for asset in self.assets
            if asset.asset.format is AssetFormat.URDF
            and asset.asset.role is AssetRole.DESCRIPTION
        )
        if len(urdfs) != 1:
            raise MissingUrdfError(str(self.name), len(urdfs))
        validate_components(str(self.name), self.kind, self.components)

    @property
    def id(self) -> EmbodimentId:
        return content_id(EmbodimentId, self._content_dict())

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
                ComponentCapabilities(
                    component.component_id,
                    CapabilitySet(component.capabilities),
                )
                for component in self.components
                if component.capabilities
            )
        )

    @property
    def policy_hz(self) -> float | None:
        return self.rates.policy_hz if self.rates is not None else None

    @property
    def urdf(self) -> ProvenancedAsset:
        return next(
            asset
            for asset in self.assets
            if asset.asset.format is AssetFormat.URDF
            and asset.asset.role is AssetRole.DESCRIPTION
        )

    @property
    def urdf_path(self) -> Path:
        """Verified local path of the authoritative packaged description."""

        return resolve_asset(self.urdf.asset)

    @property
    def urdf_bytes(self) -> bytes:
        return self.urdf_path.read_bytes()

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
            if component.role is ComponentRole.BODY
            and isinstance(component.part, GripperSpec)
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
    def grasp_centre_m(self) -> tuple[float, float, float]:
        centre = self.single_gripper.grasp_centre_m
        if centre is None:
            raise LayoutError(str(self.name), "gripper grasp centre is not declared")
        return centre

    @property
    def ready_joints(self) -> tuple[float, ...]:
        ready = self.single_arm.ready
        if ready is None:
            raise LayoutError(str(self.name), "arm ready configuration is not declared")
        return ready

    @property
    def has_mobile_base(self) -> bool:
        return any(
            component.role is ComponentRole.BODY
            and isinstance(component.part, MobileBaseSpec)
            for component in self.components
        )

    def with_assets(
        self,
        assets: tuple[ProvenancedAsset, ...],
        *,
        urdf: bytes,
    ) -> Embodiment:
        """Bind the morphology to another complete, provenance-bearing asset set."""

        refs = tuple(asset.asset for asset in assets)
        _validate_urdf(self.name, refs, urdf)
        return dataclasses.replace(self, assets=assets)

    def canonical_json(self) -> str:
        return canonical_document(self._content_dict())

    def to_dict(self) -> dict[str, object]:
        return {"id": str(self.id), **self._content_dict()}

    def to_json(self) -> str:
        return canonical_document(self.to_dict())

    def _content_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "name": str(self.name),
            "label": self.label,
            "kind": self.kind.value,
            "lineage": {
                "family": self.lineage.family,
                "variant": self.lineage.variant,
                "revision": self.lineage.revision,
            },
            "components": [_component_to_dict(component) for component in self.components],
            "rates": _rates_to_dict(self.rates),
            "base_mount": _base_mount_to_dict(self.base_mount),
            "assets": [_provenanced_asset_to_dict(asset) for asset in self.assets],
        }

    @classmethod
    def from_dict(cls, document: Mapping[str, object]) -> Embodiment:
        _exact_keys(
            document,
            {
                "id",
                "schema_version",
                "name",
                "label",
                "kind",
                "lineage",
                "components",
                "rates",
                "base_mount",
                "assets",
            },
            "embodiment",
        )
        version = _require(document, "schema_version")
        if type(version) is not int or version != SCHEMA_VERSION:
            raise EmbodimentSchemaError(f"unsupported embodiment schema_version: {version!r}")
        try:
            expected_id = EmbodimentId(_string(document, "id", "embodiment"))
            kind = EmbodimentKind(_string(document, "kind", "embodiment"))
            lineage = _parse_lineage(_require(document, "lineage"))
            embodiment = cls(
                name=EmbodimentName(_string(document, "name", "embodiment")),
                label=_string(document, "label", "embodiment"),
                kind=kind,
                lineage=lineage,
                components=_parse_components(_require(document, "components")),
                rates=_parse_rates(_require(document, "rates")),
                base_mount=_parse_base_mount(_require(document, "base_mount")),
                assets=_parse_assets(_require(document, "assets")),
            )
        except EmbodimentSchemaError:
            raise
        except ValueError as exc:
            raise EmbodimentSchemaError(f"invalid embodiment: {exc}") from exc
        if embodiment.id != expected_id:
            raise EmbodimentSchemaError("embodiment id does not match its canonical content")
        return embodiment

    @classmethod
    def from_json(cls, value: str) -> Embodiment:
        try:
            document: object = json.loads(value)
        except json.JSONDecodeError as exc:
            raise EmbodimentSchemaError("embodiment is not valid JSON") from exc
        if not isinstance(document, Mapping):
            raise EmbodimentSchemaError("embodiment JSON must contain an object")
        return cls.from_dict(cast(Mapping[str, object], document))


def _component_to_dict(component: Component) -> dict[str, object]:
    attachment = component.attachment
    if isinstance(attachment, BodyAttachment):
        attachment_kind = "body"
    elif isinstance(attachment, LeaderAttachment):
        attachment_kind = "leader"
    else:
        attachment_kind = "sensor"
    mount: dict[str, object]
    if isinstance(component.mount, RootMount):
        mount = {"kind": "root", "frame": component.mount.frame}
    else:
        mount = {
            "kind": "mounted_on",
            "parent": component.mount.parent,
            "frame": component.mount.frame,
        }
    return {
        "instance": component.instance,
        "attachment": {
            "kind": attachment_kind,
            "part": _part_to_dict(attachment.part),
        },
        "mount": mount,
    }


def _part_to_dict(part: Part) -> dict[str, object]:
    common: dict[str, object] = {"part_id": str(part.part_id)}
    if isinstance(part, ArmSpec):
        return common | {
            "kind": "arm",
            "layout": _layout_to_dict(part.layout),
            "home": list(part.home),
            "ready": list(part.ready) if part.ready is not None else None,
            "physical": _physical_to_dict(part.physical),
        }
    if isinstance(part, JointGroupSpec):
        return common | {
            "kind": "joint_group",
            "layout": _layout_to_dict(part.layout),
            "home": list(part.home),
        }
    if isinstance(part, GripperSpec):
        return common | {
            "kind": "gripper",
            "layout": _layout_to_dict(part.layout),
            "travel_m": list(part.travel_m) if part.travel_m is not None else None,
            "grasp_centre_m": (
                list(part.grasp_centre_m) if part.grasp_centre_m is not None else None
            ),
            "mimic_joints": [
                {"name": mimic.joint_name, "of": mimic.of, "multiplier": mimic.multiplier}
                for mimic in part.mimic_joints
            ],
            "gap_curve": (
                [{"x": knot.x, "y": knot.y} for knot in part.gap_curve.knots]
                if part.gap_curve is not None
                else None
            ),
            "physical": _physical_to_dict(part.physical),
        }
    if isinstance(part, CameraSpec):
        return common | {
            "kind": "camera",
            "sensor_model": part.model.value,
            "modality": part.modality.value,
            "fps": part.fps,
            "projection": part.projection.value,
            "resolution": list(part.resolution) if part.resolution is not None else None,
        }
    if isinstance(part, MobileBaseSpec):
        return common | {"kind": "mobile_base", "layout": _layout_to_dict(part.layout)}
    if isinstance(part, ForceTorqueSpec):
        return common | {"kind": "force_torque", "rate_hz": part.rate_hz}
    return common | {"kind": "device", "description": part.description}


def _layout_to_dict(layout: JointLayout) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for axis in layout.axes:
        bounds: dict[str, object]
        if isinstance(axis.bounds, Bounds):
            bounds = {
                "kind": "bounded",
                "lower": axis.bounds.lower,
                "upper": axis.bounds.upper,
            }
        else:
            bounds = {"kind": "unbounded"}
        rows.append({"name": axis.name, "unit": axis.unit.value, "bounds": bounds})
    return rows


def _parse_components(raw: object) -> tuple[Component, ...]:
    rows = _list(raw, "components")
    return tuple(
        _parse_component(row, f"components[{index}]") for index, row in enumerate(rows)
    )


def _parse_component(raw: object, label: str) -> Component:
    entry = _mapping(raw, label)
    _exact_keys(entry, {"instance", "attachment", "mount"}, label)
    attachment_entry = _mapping(entry["attachment"], f"{label}.attachment")
    _exact_keys(attachment_entry, {"kind", "part"}, f"{label}.attachment")
    attachment_kind = _string(attachment_entry, "kind", f"{label}.attachment")
    part = _parse_part(attachment_entry["part"], f"{label}.attachment.part")
    if attachment_kind == "body":
        attachment = BodyAttachment(part)
    elif attachment_kind == "leader":
        attachment = LeaderAttachment(part)
    elif attachment_kind == "sensor":
        if not isinstance(part, CameraSpec | ForceTorqueSpec):
            raise EmbodimentSchemaError(f"{label}.attachment sensor has a non-sensor part")
        attachment = SensorAttachment(part)
    else:
        raise EmbodimentSchemaError(
            f"{label}.attachment.kind is unknown: {attachment_kind!r}"
        )
    return Component(
        instance=_string(entry, "instance", label),
        attachment=attachment,
        mount=_parse_component_mount(entry["mount"], f"{label}.mount"),
    )


def _parse_component_mount(raw: object, label: str) -> RootMount | MountedOn:
    entry = _mapping(raw, label)
    kind = _string(entry, "kind", label)
    if kind == "root":
        _exact_keys(entry, {"kind", "frame"}, label)
        return RootMount(_string(entry, "frame", label))
    if kind == "mounted_on":
        _exact_keys(entry, {"kind", "parent", "frame"}, label)
        return MountedOn(
            parent=_string(entry, "parent", label),
            frame=_string(entry, "frame", label),
        )
    raise EmbodimentSchemaError(f"{label}.kind is unknown: {kind!r}")


def _parse_part(raw: object, label: str) -> Part:
    entry = _mapping(raw, label)
    kind = _string(entry, "kind", label)
    common = {"kind", "part_id"}
    expected_by_kind = {
        "arm": common | {"layout", "home", "ready", "physical"},
        "joint_group": common | {"layout", "home"},
        "gripper": common
        | {
            "layout",
            "travel_m",
            "grasp_centre_m",
            "mimic_joints",
            "gap_curve",
            "physical",
        },
        "camera": common
        | {"sensor_model", "modality", "fps", "projection", "resolution"},
        "mobile_base": common | {"layout"},
        "force_torque": common | {"rate_hz"},
        "device": common | {"description"},
    }
    expected = expected_by_kind.get(kind)
    if expected is None:
        raise EmbodimentSchemaError(f"{label}.kind is unknown: {kind!r}")
    _exact_keys(entry, expected, label)
    part_id = PartId(_string(entry, "part_id", label))
    try:
        if kind == "arm":
            return ArmSpec(
                part_id=part_id,
                layout=_parse_layout(entry["layout"], label),
                home=_parse_vector(entry["home"], f"{label}.home"),
                ready=_parse_optional_vector(entry["ready"], f"{label}.ready"),
                physical=_parse_physical(entry["physical"], label),
            )
        if kind == "joint_group":
            return JointGroupSpec(
                part_id=part_id,
                layout=_parse_layout(entry["layout"], label),
                home=_parse_vector(entry["home"], f"{label}.home"),
            )
        if kind == "gripper":
            return GripperSpec(
                part_id=part_id,
                layout=_parse_layout(entry["layout"], label),
                travel_m=_parse_pair(entry["travel_m"], f"{label}.travel_m"),
                grasp_centre_m=_parse_triple(
                    entry["grasp_centre_m"], f"{label}.grasp_centre_m"
                ),
                mimic_joints=_parse_mimics(entry["mimic_joints"], label),
                gap_curve=_parse_curve(entry["gap_curve"], label),
                physical=_parse_physical(entry["physical"], label),
            )
        if kind == "camera":
            return CameraSpec(
                part_id=part_id,
                model=SensorModel(_string(entry, "sensor_model", label)),
                modality=CameraModality(_string(entry, "modality", label)),
                fps=_number(entry["fps"], f"{label}.fps"),
                projection=LensProjection(_string(entry, "projection", label)),
                resolution=_parse_resolution(entry["resolution"], label),
            )
        if kind == "mobile_base":
            return MobileBaseSpec(part_id=part_id, layout=_parse_layout(entry["layout"], label))
        if kind == "force_torque":
            return ForceTorqueSpec(
                part_id=part_id,
                rate_hz=_optional_number(entry["rate_hz"], f"{label}.rate_hz"),
            )
        return DeviceSpec(
            part_id=part_id,
            description=_string(entry, "description", label),
        )
    except EmbodimentSchemaError:
        raise
    except ValueError as exc:
        raise EmbodimentSchemaError(f"invalid {label}: {exc}") from exc


def _parse_layout(raw: object, label: str) -> JointLayout:
    rows = _list(raw, f"{label}.layout")
    axes: list[JointAxis] = []
    for index, raw_axis in enumerate(rows):
        axis_label = f"{label}.layout[{index}]"
        entry = _mapping(raw_axis, axis_label)
        _exact_keys(entry, {"name", "unit", "bounds"}, axis_label)
        try:
            unit = CoordinateUnit(_string(entry, "unit", axis_label))
        except ValueError as exc:
            raise EmbodimentSchemaError(f"{axis_label}.unit is unknown") from exc
        bounds_entry = _mapping(entry["bounds"], f"{axis_label}.bounds")
        bounds_kind = _string(bounds_entry, "kind", f"{axis_label}.bounds")
        if bounds_kind == "bounded":
            _exact_keys(
                bounds_entry,
                {"kind", "lower", "upper"},
                f"{axis_label}.bounds",
            )
            bounds = Bounds(
                _number(bounds_entry["lower"], f"{axis_label}.bounds.lower"),
                _number(bounds_entry["upper"], f"{axis_label}.bounds.upper"),
            )
        elif bounds_kind == "unbounded":
            _exact_keys(bounds_entry, {"kind"}, f"{axis_label}.bounds")
            bounds = Unbounded()
        else:
            raise EmbodimentSchemaError(
                f"{axis_label}.bounds.kind is unknown: {bounds_kind!r}"
            )
        axes.append(JointAxis(_string(entry, "name", axis_label), unit, bounds))
    return JointLayout(tuple(axes))


def _physical_to_dict(value: PhysicalSpec | None) -> dict[str, float | None] | None:
    if value is None:
        return None
    return {
        "payload_kg": value.payload_kg,
        "reach_m": value.reach_m,
        "mass_kg": value.mass_kg,
    }


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
    result: list[MimicJoint] = []
    for index, raw_mimic in enumerate(_list(raw, f"{label}.mimic_joints")):
        mimic_label = f"{label}.mimic_joints[{index}]"
        entry = _mapping(raw_mimic, mimic_label)
        _exact_keys(entry, {"name", "of", "multiplier"}, mimic_label)
        result.append(
            MimicJoint(
                joint_name=_string(entry, "name", mimic_label),
                of=_string(entry, "of", mimic_label),
                multiplier=_number(entry["multiplier"], f"{mimic_label}.multiplier"),
            )
        )
    return tuple(result)


def _parse_curve(raw: object, label: str) -> Curve1D | None:
    if raw is None:
        return None
    knots: list[Knot] = []
    for index, raw_knot in enumerate(_list(raw, f"{label}.gap_curve")):
        knot_label = f"{label}.gap_curve[{index}]"
        entry = _mapping(raw_knot, knot_label)
        _exact_keys(entry, {"x", "y"}, knot_label)
        knots.append(
            Knot(
                _number(entry["x"], f"{knot_label}.x"),
                _number(entry["y"], f"{knot_label}.y"),
            )
        )
    return Curve1D(tuple(knots))


def _provenanced_asset_to_dict(value: ProvenancedAsset) -> dict[str, object]:
    asset = value.asset
    provenance = value.provenance
    return {
        "asset": {
            "location": asset.location,
            "content": {
                "sha256": str(asset.content.sha256),
                "size_bytes": asset.content.size_bytes,
            },
            "format": asset.format.value,
            "role": asset.role.value,
            "media_type": asset.media_type,
            "logical_path": (
                str(asset.logical_path) if asset.logical_path is not None else None
            ),
        },
        "provenance": {
            "repository": provenance.repository,
            "revision": provenance.revision,
            "path": provenance.path,
            "license_id": provenance.license_id,
            "generator": provenance.generator,
        },
    }


def _parse_assets(raw: object) -> tuple[ProvenancedAsset, ...]:
    return tuple(
        _parse_provenanced_asset(item, f"assets[{index}]")
        for index, item in enumerate(_list(raw, "assets"))
    )


def _parse_provenanced_asset(raw: object, label: str) -> ProvenancedAsset:
    entry = _mapping(raw, label)
    _exact_keys(entry, {"asset", "provenance"}, label)
    asset_entry = _mapping(entry["asset"], f"{label}.asset")
    _exact_keys(
        asset_entry,
        {"location", "content", "format", "role", "media_type", "logical_path"},
        f"{label}.asset",
    )
    content_entry = _mapping(asset_entry["content"], f"{label}.asset.content")
    _exact_keys(content_entry, {"sha256", "size_bytes"}, f"{label}.asset.content")
    provenance_entry = _mapping(entry["provenance"], f"{label}.provenance")
    _exact_keys(
        provenance_entry,
        {"repository", "revision", "path", "license_id", "generator"},
        f"{label}.provenance",
    )
    media_type = asset_entry["media_type"]
    if media_type is not None and not isinstance(media_type, str):
        raise EmbodimentSchemaError(f"{label}.asset.media_type must be a string or null")
    logical_path = asset_entry["logical_path"]
    if logical_path is not None and not isinstance(logical_path, str):
        raise EmbodimentSchemaError(f"{label}.asset.logical_path must be a string or null")
    generator = provenance_entry["generator"]
    if generator is not None and not isinstance(generator, str):
        raise EmbodimentSchemaError(f"{label}.provenance.generator must be a string or null")
    try:
        asset = AssetRef(
            location=_string(asset_entry, "location", f"{label}.asset"),
            content=ContentBlob(
                sha256=Sha256Digest(
                    _string(content_entry, "sha256", f"{label}.asset.content")
                ),
                size_bytes=_integer(
                    content_entry["size_bytes"], f"{label}.asset.content.size_bytes"
                ),
            ),
            format=AssetFormat(_string(asset_entry, "format", f"{label}.asset")),
            role=AssetRole(_string(asset_entry, "role", f"{label}.asset")),
            media_type=media_type,
            logical_path=(PurePosixPath(logical_path) if logical_path is not None else None),
        )
        provenance = AssetProvenance(
            repository=_string(provenance_entry, "repository", f"{label}.provenance"),
            revision=_string(provenance_entry, "revision", f"{label}.provenance"),
            path=_string(provenance_entry, "path", f"{label}.provenance"),
            license_id=_string(provenance_entry, "license_id", f"{label}.provenance"),
            generator=generator,
        )
    except ValueError as exc:
        raise EmbodimentSchemaError(f"invalid {label}: {exc}") from exc
    return ProvenancedAsset(asset=asset, provenance=provenance)


def _rates_to_dict(value: ControlRates | None) -> dict[str, float | None] | None:
    if value is None:
        return None
    return {"policy_hz": value.policy_hz, "low_level_hz": value.low_level_hz}


def _parse_rates(raw: object) -> ControlRates | None:
    if raw is None:
        return None
    entry = _mapping(raw, "rates")
    _exact_keys(entry, {"policy_hz", "low_level_hz"}, "rates")
    return ControlRates(
        policy_hz=_number(entry["policy_hz"], "rates.policy_hz"),
        low_level_hz=_optional_number(entry["low_level_hz"], "rates.low_level_hz"),
    )


def _base_mount_to_dict(value: BaseMount | None) -> dict[str, object] | None:
    if value is None:
        return None
    return {
        "kind": value.kind.value,
        "frame": value.frame,
        "half_extents": list(value.half_extents),
        "centre": list(value.centre),
        "clearance_m": value.clearance_m,
    }


def _parse_base_mount(raw: object) -> BaseMount | None:
    if raw is None:
        return None
    entry = _mapping(raw, "base_mount")
    _exact_keys(entry, {"kind", "frame", "half_extents", "centre", "clearance_m"}, "base_mount")
    try:
        kind = MountKind(_string(entry, "kind", "base_mount"))
    except ValueError as exc:
        raise EmbodimentSchemaError("base_mount.kind is unknown") from exc
    half = _required_pair(entry["half_extents"], "base_mount.half_extents")
    centre = _required_pair(entry["centre"], "base_mount.centre")
    return BaseMount(
        kind=kind,
        frame=_string(entry, "frame", "base_mount"),
        half_extents=half,
        centre=centre,
        clearance_m=_number(entry["clearance_m"], "base_mount.clearance_m"),
    )


def _parse_lineage(raw: object) -> Lineage:
    entry = _mapping(raw, "lineage")
    _exact_keys(entry, {"family", "variant", "revision"}, "lineage")
    return Lineage(
        family=_string(entry, "family", "lineage"),
        variant=_string(entry, "variant", "lineage"),
        revision=_string(entry, "revision", "lineage"),
    )


def _portable_component(component: Component) -> Component:
    part = component.part
    if isinstance(part, ArmSpec | JointGroupSpec | GripperSpec | MobileBaseSpec):
        part = dataclasses.replace(part, assets=())
    attachment = dataclasses.replace(component.attachment, part=part)
    return dataclasses.replace(component, attachment=attachment)


def embodiment_from_definition(definition: EmbodimentDefinition) -> Embodiment:
    assets: list[ProvenancedAsset] = []
    seen: set[tuple[str, str]] = set()
    for packaged in _packaged_assets(definition):
        key = (packaged.relpath, packaged.sha256)
        if key not in seen:
            assets.append(packaged.provenanced_asset())
            seen.add(key)
    urdf = authoritative_urdf(definition)
    refs = tuple(asset.asset for asset in assets)
    _validate_urdf(definition.name, refs, urdf.path().read_bytes())
    return Embodiment(
        name=definition.name,
        label=definition.label,
        kind=definition.kind,
        lineage=definition.lineage,
        components=tuple(_portable_component(component) for component in definition.attachments),
        assets=tuple(assets),
        rates=definition.rates,
        base_mount=definition.base_mount,
    )


def authoritative_urdf(definition: EmbodimentDefinition) -> PackagedAsset:
    matches = tuple(
        {
            (asset.relpath, asset.sha256): asset
            for asset in _packaged_assets(definition)
            if asset.format is AssetFormat.URDF and asset.role is AssetRole.DESCRIPTION
        }.values()
    )
    if len(matches) != 1:
        raise MissingUrdfError(str(definition.name), len(matches))
    return matches[0]


def _packaged_assets(definition: EmbodimentDefinition) -> list[PackagedAsset]:
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
            f"{urdfs[0].location}: authoritative URDF bytes have sha256 {actual}, "
            f"expected {urdfs[0].sha256}"
        )
    try:
        root = ET.fromstring(urdf)
    except ET.ParseError as exc:
        raise EmbodimentSchemaError(f"{name}: authoritative URDF is invalid XML") from exc
    if root.tag != "robot" or not root.attrib.get("name", "").strip():
        raise EmbodimentSchemaError(f"{name}: authoritative URDF root must be a named <robot>")


def _parse_resolution(raw: object, label: str) -> tuple[int, int] | None:
    if raw is None:
        return None
    values = _list(raw, f"{label}.resolution")
    if len(values) != 2:
        raise EmbodimentSchemaError(f"{label}.resolution must contain two integers")
    width = _integer(values[0], f"{label}.resolution[0]")
    height = _integer(values[1], f"{label}.resolution[1]")
    if width <= 0 or height <= 0:
        raise EmbodimentSchemaError(f"{label}.resolution must be positive")
    return width, height


def _parse_vector(raw: object, label: str) -> tuple[float, ...]:
    return tuple(_number(value, label) for value in _list(raw, label))


def _parse_optional_vector(raw: object, label: str) -> tuple[float, ...] | None:
    return None if raw is None else _parse_vector(raw, label)


def _required_pair(raw: object, label: str) -> tuple[float, float]:
    result = _parse_pair(raw, label)
    if result is None:
        raise EmbodimentSchemaError(f"{label} must contain two numbers")
    return result


def _parse_pair(raw: object, label: str) -> tuple[float, float] | None:
    if raw is None:
        return None
    values = _list(raw, label)
    if len(values) != 2:
        raise EmbodimentSchemaError(f"{label} must contain two numbers")
    return _number(values[0], label), _number(values[1], label)


def _parse_triple(raw: object, label: str) -> tuple[float, float, float] | None:
    if raw is None:
        return None
    values = _list(raw, label)
    if len(values) != 3:
        raise EmbodimentSchemaError(f"{label} must contain three numbers")
    return (
        _number(values[0], label),
        _number(values[1], label),
        _number(values[2], label),
    )


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


def _number(raw: object, label: str) -> float:
    if isinstance(raw, bool) or not isinstance(raw, int | float):
        raise EmbodimentSchemaError(f"{label} must be a number")
    value = float(raw)
    if not math.isfinite(value):
        raise EmbodimentSchemaError(f"{label} must be finite")
    return value


def _optional_number(raw: object, label: str) -> float | None:
    return None if raw is None else _number(raw, label)


def _integer(raw: object, label: str) -> int:
    if isinstance(raw, bool) or not isinstance(raw, int):
        raise EmbodimentSchemaError(f"{label} must be an integer")
    return raw


def _exact_keys(document: Mapping[str, object], expected: set[str], label: str) -> None:
    actual = set(document)
    if actual != expected:
        raise EmbodimentSchemaError(
            f"{label} keys differ: missing={sorted(expected - actual)}, "
            f"unknown={sorted(actual - expected)}"
        )
