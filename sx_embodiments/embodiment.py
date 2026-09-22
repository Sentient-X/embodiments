"""One immutable embodiment over one authoritative typed component graph."""

from __future__ import annotations

import copy
import dataclasses
import hashlib
import json
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import cast

from sx_contracts import CapabilityProfile, CapabilitySet, ComponentCapabilities, decode
from sx_contracts.assets import (
    AssetFormat,
    AssetProvenance,
    AssetRef,
    AssetRole,
    ProvenancedAsset,
)
from sx_contracts.content import ContentBlob, Sha256Digest
from sx_contracts.identity import JsonObject, content_id
from sx_contracts.identity import canonical_json as canonical_document

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
    OperatorMount,
    OperatorSite,
    RootMount,
    SensorAttachment,
    camera_bindings,
    state_space,
    validate_components,
    validate_operator_mounts,
)
from .curves import Curve1D, Knot
from .errors import (
    AssetDigestMismatchError,
    EmbodimentSchemaError,
    LayoutError,
    MissingUrdfError,
)
from .identity import EmbodimentId, EmbodimentKind, EmbodimentName, Lineage, PartId
from .layout import (
    ActuationBinding,
    ActuatorBinding,
    ActuatorBus,
    ActuatorFeedback,
    ActuatorModel,
    Bounds,
    CoordinateUnit,
    DirectDrive,
    EncoderReadout,
    IntegratedDrive,
    JointAxis,
    JointLayout,
    ObservationBinding,
    Passive,
    StateSpace,
    Unbounded,
    UndocumentedDrive,
    Unobserved,
    VendorReadout,
)
from .parts import (
    ArmSpec,
    CameraBinding,
    CameraModality,
    CameraOptics,
    CameraOpticsAuthority,
    CameraSpec,
    ControlRates,
    DeviceSpec,
    FactSource,
    ForceTorqueSpec,
    GraspKind,
    GripperSpec,
    JointGroupSpec,
    MimicJoint,
    MobileBaseSpec,
    Part,
    PhysicalSpec,
    SensorModel,
)

SCHEMA_VERSION = 14


@dataclass(frozen=True, slots=True)
class Embodiment:
    """A complete, content-addressed hardware revision.

    The component graph is the sole morphology. State order, roles, camera bindings,
    capabilities, and single-arm projections are derived from it. Friendly names are
    catalog aliases; ``id`` is the digest of the complete schema-14 document.

    Constructed through the registry, validated assembly, composition, or the checked
    document codec. Derived state, camera and capability views share these same facts.
    """

    name: EmbodimentName
    label: str
    kind: EmbodimentKind
    lineage: Lineage
    components: tuple[Component, ...]
    assets: tuple[ProvenancedAsset, ...]
    rates: ControlRates | None = None
    base_mount: BaseMount | None = None
    operator_mounts: tuple[OperatorMount, ...] = ()
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
            if asset.asset.format is AssetFormat.URDF and asset.asset.role is AssetRole.DESCRIPTION
        )
        if len(urdfs) != 1:
            raise MissingUrdfError(str(self.name), len(urdfs))
        validate_components(str(self.name), self.kind, self.components)
        validate_operator_mounts(str(self.name), self.kind, self.operator_mounts)

    @property
    def id(self) -> EmbodimentId:
        return content_id(EmbodimentId, cast("JsonObject", self._content_dict()))

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
            if asset.asset.format is AssetFormat.URDF and asset.asset.role is AssetRole.DESCRIPTION
        )

    @property
    def urdf_path(self) -> Path:
        """Verified local path of the authoritative packaged description."""

        path = resolve_asset(self.urdf.asset)
        _validate_urdf(
            self.name,
            tuple(asset.asset for asset in self.assets),
            path.read_bytes(),
            components=self.components,
            operator_mounts=self.operator_mounts,
        )
        return path

    @property
    def urdf_bytes(self) -> bytes:
        return self.urdf_path.read_bytes()

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Unique URDF joint names, in exactly the native state order."""
        from .description import joint_names

        return joint_names(self.components, self.state, self.urdf_bytes)

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
            component.role is ComponentRole.BODY and isinstance(component.part, MobileBaseSpec)
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
        _validate_urdf(
            self.name,
            refs,
            urdf,
            components=self.components,
            operator_mounts=self.operator_mounts,
        )
        return dataclasses.replace(self, assets=assets)

    def canonical_json(self) -> str:
        return canonical_document(cast("JsonObject", self._content_dict()))

    def to_dict(self) -> dict[str, object]:
        return {"id": str(self.id), **self._content_dict()}

    def to_json(self) -> str:
        return canonical_document(cast("JsonObject", self.to_dict()))

    def _content_dict(self) -> dict[str, object]:
        # The object-typed projection is JSON by construction; canonical_json
        # re-validates every leaf at runtime, so the cast at the callers is safe.
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
            "operator_mounts": [_operator_mount_to_dict(mount) for mount in self.operator_mounts],
            "assets": [_provenanced_asset_to_dict(asset) for asset in self.assets],
        }

    @classmethod
    def from_dict(cls, document: Mapping[str, object]) -> Embodiment:
        entry = decode.exactly(
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
                "operator_mounts",
                "assets",
            },
            where="embodiment",
            error=EmbodimentSchemaError,
        )
        version = decode.integer(entry, "schema_version")
        if version != SCHEMA_VERSION:
            raise EmbodimentSchemaError(f"unsupported embodiment schema_version: {version!r}")
        try:
            expected_id = EmbodimentId(decode.text(entry, "id"))
            kind = EmbodimentKind(decode.text(entry, "kind"))
            embodiment = cls(
                name=EmbodimentName(decode.text(entry, "name")),
                label=decode.text(entry, "label"),
                kind=kind,
                lineage=_parse_lineage(decode.mapping(entry, "lineage")),
                components=_parse_components(entry),
                rates=_parse_rates(entry),
                base_mount=_parse_base_mount(entry),
                operator_mounts=_parse_operator_mounts(entry),
                assets=_parse_assets(entry),
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
        return cls.from_dict(
            decode.document(document, where="embodiment", error=EmbodimentSchemaError)
        )


@dataclass(frozen=True, slots=True)
class EmbodimentMigration:
    """One verified content-identity edge emitted by an embodiment converter."""

    source_id: EmbodimentId
    embodiment: Embodiment

    @property
    def target_id(self) -> EmbodimentId:
        return self.embodiment.id


def convert_v13_to_v14(document: Mapping[str, object]) -> EmbodimentMigration:
    """Convert one exact schema-13 embodiment without inventing missing access facts.

    A prior actuator proves a direct drive only. Schema 13 never stated how a coordinate
    was observed, so every converted axis records that absence independently. Missing
    actuator facts remain undocumented; the converter never guesses feedback, integrated
    control, or passivity.
    """

    source = decode.exactly(
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
            "operator_mounts",
            "assets",
        },
        where="schema-13 embodiment",
        error=EmbodimentSchemaError,
    )
    if decode.integer(source, "schema_version") != 13:
        raise EmbodimentSchemaError("converter requires embodiment schema_version 13")
    source_id = EmbodimentId(decode.text(source, "id"))
    source_content = {key: value for key, value in source.items() if key != "id"}
    try:
        actual_source_id = content_id(EmbodimentId, cast("JsonObject", source_content))
    except ValueError as exc:
        raise EmbodimentSchemaError("schema-13 embodiment is not canonical JSON") from exc
    if actual_source_id != source_id:
        raise EmbodimentSchemaError("schema-13 embodiment id does not match its canonical content")

    converted = copy.deepcopy(dict(document))
    converted["schema_version"] = SCHEMA_VERSION
    converted_components = cast(list[dict[str, object]], converted["components"])
    for component_index, component in enumerate(decode.documents(source, "components")):
        attachment = decode.mapping(component, "attachment")
        part = decode.mapping(attachment, "part")
        if "layout" not in part:
            continue
        axes = decode.documents(part, "layout")
        converted_part = cast(
            dict[str, object],
            cast(dict[str, object], converted_components[component_index]["attachment"])["part"],
        )
        converted_axes = cast(list[dict[str, object]], converted_part["layout"])
        for axis_index, axis in enumerate(axes):
            legacy_axis = decode.exactly(axis, {"name", "unit", "bounds", "actuator"})
            actuator = legacy_axis["actuator"]
            if actuator is not None:
                try:
                    _parse_actuator(
                        decode.document(
                            actuator,
                            where=f"{axis.where}.actuator",
                            error=EmbodimentSchemaError,
                        )
                    )
                except LayoutError as error:
                    raise EmbodimentSchemaError(
                        f"{axis.where}.actuator is invalid: {error}"
                    ) from error
            raw_axis = converted_axes[axis_index]
            raw_axis.pop("actuator")
            raw_axis["observation"] = {
                "kind": "unobserved",
                "reason": "schema 13 did not declare an observation path",
            }
            if actuator is None:
                raw_axis["actuation"] = {
                    "kind": "undocumented",
                    "reason": "schema 13 omitted a per-axis actuator binding",
                }
            else:
                raw_axis["actuation"] = {"kind": "direct", "actuator": actuator}
    content = {key: value for key, value in converted.items() if key != "id"}
    converted["id"] = str(content_id(EmbodimentId, cast("JsonObject", content)))
    # Parse now so the converter itself fails on malformed legacy content and emits
    # one complete old -> new identity edge for persistence owners.
    return EmbodimentMigration(source_id, Embodiment.from_dict(converted))


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
            "grasp": part.grasp.value,
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
        optics = part.optics
        return common | {
            "kind": "camera",
            "sensor_model": part.model.value,
            "modality": part.modality.value,
            "fps": part.fps,
            "optics": {
                "width": optics.width,
                "height": optics.height,
                "image_from_camera": list(optics.image_from_camera),
                "distortion_model": optics.distortion_model,
                "distortion_coefficients": list(optics.distortion_coefficients),
                "authority": optics.authority.value,
                "source": {
                    "repository": optics.source.repository,
                    "revision": optics.source.revision,
                    "path": optics.source.path,
                },
            },
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
        rows.append(
            {
                "name": axis.name,
                "unit": axis.unit.value,
                "bounds": bounds,
                "observation": _observation_to_dict(axis.observation),
                "actuation": _actuation_to_dict(axis.actuation),
            }
        )
    return rows


def _actuator_to_dict(binding: ActuatorBinding) -> dict[str, object]:
    return {
        "model": binding.model.value,
        "bus": binding.bus.value,
        "bus_id": binding.bus_id,
        "sign": binding.sign,
        "zero_offset": binding.zero_offset,
        "reduction": binding.reduction,
    }


def _observation_to_dict(value: ObservationBinding) -> dict[str, object]:
    if isinstance(value, ActuatorFeedback):
        return {"kind": "actuator_feedback"}
    if isinstance(value, VendorReadout):
        return {"kind": "vendor_readout", "interface": value.interface}
    if isinstance(value, EncoderReadout):
        return {"kind": "encoder", "sensor": value.sensor}
    return {"kind": "unobserved", "reason": value.reason}


def _actuation_to_dict(value: ActuationBinding) -> dict[str, object]:
    if isinstance(value, DirectDrive):
        return {"kind": "direct", "actuator": _actuator_to_dict(value.actuator)}
    if isinstance(value, IntegratedDrive):
        return {"kind": "integrated", "controller": value.controller, "group": value.group}
    if isinstance(value, Passive):
        return {"kind": "passive", "reason": value.reason}
    return {"kind": "undocumented", "reason": value.reason}


def _parse_components(document: decode.Document) -> tuple[Component, ...]:
    return tuple(_parse_component(entry) for entry in decode.documents(document, "components"))


def _parse_component(document: decode.Document) -> Component:
    entry = decode.exactly(document, {"instance", "attachment", "mount"})
    attachment_entry = decode.exactly(decode.mapping(entry, "attachment"), {"kind", "part"})
    attachment_kind = decode.text(attachment_entry, "kind")
    part = _parse_part(decode.mapping(attachment_entry, "part"))
    if attachment_kind == "body":
        attachment = BodyAttachment(part)
    elif attachment_kind == "leader":
        attachment = LeaderAttachment(part)
    elif attachment_kind == "sensor":
        if not isinstance(part, CameraSpec | ForceTorqueSpec):
            raise EmbodimentSchemaError(f"{attachment_entry.where} sensor has a non-sensor part")
        attachment = SensorAttachment(part)
    else:
        raise EmbodimentSchemaError(
            f"{attachment_entry.where}.kind is unknown: {attachment_kind!r}"
        )
    return Component(
        instance=decode.text(entry, "instance"),
        attachment=attachment,
        mount=_parse_component_mount(decode.mapping(entry, "mount")),
    )


def _parse_component_mount(document: decode.Document) -> RootMount | MountedOn:
    kind = decode.text(document, "kind")
    if kind == "root":
        return RootMount(decode.text(decode.exactly(document, {"kind", "frame"}), "frame"))
    if kind == "mounted_on":
        entry = decode.exactly(document, {"kind", "parent", "frame"})
        return MountedOn(
            parent=decode.text(entry, "parent"),
            frame=decode.text(entry, "frame"),
        )
    raise EmbodimentSchemaError(f"{document.where}.kind is unknown: {kind!r}")


def _parse_part(document: decode.Document) -> Part:
    kind = decode.text(document, "kind")
    common = {"kind", "part_id"}
    expected_by_kind = {
        "arm": common | {"layout", "home", "ready", "physical"},
        "joint_group": common | {"layout", "home"},
        "gripper": common
        | {
            "grasp",
            "layout",
            "travel_m",
            "grasp_centre_m",
            "mimic_joints",
            "gap_curve",
            "physical",
        },
        "camera": common | {"sensor_model", "modality", "fps", "optics"},
        "mobile_base": common | {"layout"},
        "force_torque": common | {"rate_hz"},
        "device": common | {"description"},
    }
    expected = expected_by_kind.get(kind)
    if expected is None:
        raise EmbodimentSchemaError(f"{document.where}.kind is unknown: {kind!r}")
    entry = decode.exactly(document, expected)
    part_id = PartId(decode.text(entry, "part_id"))
    try:
        if kind == "arm":
            return ArmSpec(
                part_id=part_id,
                layout=_parse_layout(entry),
                home=decode.numbers(entry, "home"),
                ready=_parse_optional_vector(entry, "ready"),
                physical=_parse_physical(entry),
            )
        if kind == "joint_group":
            return JointGroupSpec(
                part_id=part_id,
                layout=_parse_layout(entry),
                home=decode.numbers(entry, "home"),
            )
        if kind == "gripper":
            try:
                grasp = GraspKind(decode.text(entry, "grasp"))
            except ValueError as exc:
                raise EmbodimentSchemaError(f"{entry.where}.grasp is unknown") from exc
            return GripperSpec(
                part_id=part_id,
                layout=_parse_layout(entry),
                travel_m=_optional_pair(entry, "travel_m"),
                grasp_centre_m=_optional_triple(entry, "grasp_centre_m"),
                mimic_joints=_parse_mimics(entry),
                gap_curve=_parse_curve(entry),
                physical=_parse_physical(entry),
                grasp=grasp,
            )
        if kind == "camera":
            optics = decode.exactly(
                decode.mapping(entry, "optics"),
                {
                    "width",
                    "height",
                    "image_from_camera",
                    "distortion_model",
                    "distortion_coefficients",
                    "authority",
                    "source",
                },
            )
            source = decode.exactly(
                decode.mapping(optics, "source"),
                {"repository", "revision", "path"},
            )
            return CameraSpec(
                part_id=part_id,
                model=SensorModel(decode.text(entry, "sensor_model")),
                modality=CameraModality(decode.text(entry, "modality")),
                fps=decode.number(entry, "fps"),
                optics=CameraOptics(
                    width=decode.integer(optics, "width"),
                    height=decode.integer(optics, "height"),
                    image_from_camera=decode.numbers(optics, "image_from_camera"),
                    distortion_model=decode.text(optics, "distortion_model"),
                    distortion_coefficients=decode.numbers(optics, "distortion_coefficients"),
                    authority=CameraOpticsAuthority(decode.text(optics, "authority")),
                    source=FactSource(
                        repository=decode.text(source, "repository"),
                        revision=decode.text(source, "revision"),
                        path=decode.text(source, "path"),
                    ),
                ),
            )
        if kind == "mobile_base":
            return MobileBaseSpec(part_id=part_id, layout=_parse_layout(entry))
        if kind == "force_torque":
            return ForceTorqueSpec(
                part_id=part_id,
                rate_hz=decode.optional_number(entry, "rate_hz"),
            )
        return DeviceSpec(
            part_id=part_id,
            description=decode.text(entry, "description"),
        )
    except EmbodimentSchemaError:
        raise
    except ValueError as exc:
        raise EmbodimentSchemaError(f"invalid {entry.where}: {exc}") from exc


def _parse_layout(document: decode.Document) -> JointLayout:
    axes: list[JointAxis] = []
    for axis in decode.documents(document, "layout"):
        entry = decode.exactly(axis, {"name", "unit", "bounds", "observation", "actuation"})
        try:
            unit = CoordinateUnit(decode.text(entry, "unit"))
        except ValueError as exc:
            raise EmbodimentSchemaError(f"{entry.where}.unit is unknown") from exc
        bounds_entry = decode.mapping(entry, "bounds")
        bounds_kind = decode.text(bounds_entry, "kind")
        if bounds_kind == "bounded":
            bounded = decode.exactly(bounds_entry, {"kind", "lower", "upper"})
            bounds = Bounds(decode.number(bounded, "lower"), decode.number(bounded, "upper"))
        elif bounds_kind == "unbounded":
            decode.exactly(bounds_entry, {"kind"})
            bounds = Unbounded()
        else:
            raise EmbodimentSchemaError(f"{bounds_entry.where}.kind is unknown: {bounds_kind!r}")
        axes.append(
            JointAxis(
                decode.text(entry, "name"),
                unit,
                bounds,
                observation=_parse_observation(decode.mapping(entry, "observation")),
                actuation=_parse_actuation(decode.mapping(entry, "actuation")),
            )
        )
    return JointLayout(tuple(axes))


def _parse_actuator(entry: decode.Document) -> ActuatorBinding:
    binding = decode.exactly(
        entry,
        {"model", "bus", "bus_id", "sign", "zero_offset", "reduction"},
    )
    try:
        model = ActuatorModel(decode.text(binding, "model"))
    except ValueError as exc:
        # An unknown model on the wire is an unqualified motor: fail closed, and say
        # what the qualified vocabulary is — that message is the product boundary.
        qualified = ", ".join(sorted(member.value for member in ActuatorModel))
        raise EmbodimentSchemaError(
            f"{binding.where}.model names an unqualified actuator; qualified models are:"
            f" {qualified}"
        ) from exc
    try:
        bus = ActuatorBus(decode.text(binding, "bus"))
    except ValueError as exc:
        raise EmbodimentSchemaError(f"{binding.where}.bus is unknown") from exc
    return ActuatorBinding(
        model=model,
        bus=bus,
        bus_id=decode.integer(binding, "bus_id"),
        sign=decode.integer(binding, "sign"),
        zero_offset=decode.number(binding, "zero_offset"),
        reduction=decode.number(binding, "reduction"),
    )


def _parse_observation(entry: decode.Document) -> ObservationBinding:
    kind = decode.text(entry, "kind")
    if kind == "actuator_feedback":
        decode.exactly(entry, {"kind"})
        return ActuatorFeedback()
    if kind == "vendor_readout":
        value = decode.exactly(entry, {"kind", "interface"})
        return VendorReadout(decode.text(value, "interface"))
    if kind == "encoder":
        value = decode.exactly(entry, {"kind", "sensor"})
        return EncoderReadout(decode.text(value, "sensor"))
    if kind == "unobserved":
        value = decode.exactly(entry, {"kind", "reason"})
        return Unobserved(decode.text(value, "reason"))
    raise EmbodimentSchemaError(f"{entry.where}.kind is unknown: {kind!r}")


def _parse_actuation(entry: decode.Document) -> ActuationBinding:
    kind = decode.text(entry, "kind")
    if kind == "direct":
        value = decode.exactly(entry, {"kind", "actuator"})
        return DirectDrive(_parse_actuator(decode.mapping(value, "actuator")))
    if kind == "integrated":
        value = decode.exactly(entry, {"kind", "controller", "group"})
        return IntegratedDrive(decode.text(value, "controller"), decode.text(value, "group"))
    if kind == "passive":
        value = decode.exactly(entry, {"kind", "reason"})
        return Passive(decode.text(value, "reason"))
    if kind == "undocumented":
        value = decode.exactly(entry, {"kind", "reason"})
        return UndocumentedDrive(decode.text(value, "reason"))
    raise EmbodimentSchemaError(f"{entry.where}.kind is unknown: {kind!r}")


def _physical_to_dict(value: PhysicalSpec | None) -> dict[str, float | None] | None:
    if value is None:
        return None
    return {
        "payload_kg": value.payload_kg,
        "reach_m": value.reach_m,
        "mass_kg": value.mass_kg,
    }


def _parse_physical(document: decode.Document) -> PhysicalSpec | None:
    if document["physical"] is None:
        return None
    entry = decode.exactly(
        decode.mapping(document, "physical"), {"payload_kg", "reach_m", "mass_kg"}
    )
    return PhysicalSpec(
        payload_kg=decode.optional_number(entry, "payload_kg"),
        reach_m=decode.optional_number(entry, "reach_m"),
        mass_kg=decode.optional_number(entry, "mass_kg"),
    )


def _parse_mimics(document: decode.Document) -> tuple[MimicJoint, ...]:
    result: list[MimicJoint] = []
    for mimic in decode.documents(document, "mimic_joints"):
        entry = decode.exactly(mimic, {"name", "of", "multiplier"})
        result.append(
            MimicJoint(
                joint_name=decode.text(entry, "name"),
                of=decode.text(entry, "of"),
                multiplier=decode.number(entry, "multiplier"),
            )
        )
    return tuple(result)


def _parse_curve(document: decode.Document) -> Curve1D | None:
    if document["gap_curve"] is None:
        return None
    knots = [
        Knot(decode.number(knot, "x"), decode.number(knot, "y"))
        for knot in (
            decode.exactly(raw, {"x", "y"}) for raw in decode.documents(document, "gap_curve")
        )
    ]
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
            "logical_path": (str(asset.logical_path) if asset.logical_path is not None else None),
        },
        "provenance": {
            "repository": provenance.repository,
            "revision": provenance.revision,
            "path": provenance.path,
            "license_id": provenance.license_id,
            "generator": provenance.generator,
        },
    }


def _parse_assets(document: decode.Document) -> tuple[ProvenancedAsset, ...]:
    return tuple(_parse_provenanced_asset(item) for item in decode.documents(document, "assets"))


def _parse_provenanced_asset(document: decode.Document) -> ProvenancedAsset:
    entry = decode.exactly(document, {"asset", "provenance"})
    asset_entry = decode.exactly(
        decode.mapping(entry, "asset"),
        {"location", "content", "format", "role", "media_type", "logical_path"},
    )
    content_entry = decode.exactly(decode.mapping(asset_entry, "content"), {"sha256", "size_bytes"})
    provenance_entry = decode.exactly(
        decode.mapping(entry, "provenance"),
        {"repository", "revision", "path", "license_id", "generator"},
    )
    logical_path = decode.optional_text(asset_entry, "logical_path")
    try:
        asset = AssetRef(
            location=decode.text(asset_entry, "location"),
            content=ContentBlob(
                sha256=Sha256Digest(decode.text(content_entry, "sha256")),
                size_bytes=decode.integer(content_entry, "size_bytes"),
            ),
            format=AssetFormat(decode.text(asset_entry, "format")),
            role=AssetRole(decode.text(asset_entry, "role")),
            media_type=decode.optional_text(asset_entry, "media_type"),
            logical_path=(PurePosixPath(logical_path) if logical_path is not None else None),
        )
        provenance = AssetProvenance(
            repository=decode.text(provenance_entry, "repository"),
            revision=decode.text(provenance_entry, "revision"),
            path=decode.text(provenance_entry, "path"),
            license_id=decode.text(provenance_entry, "license_id"),
            generator=decode.optional_text(provenance_entry, "generator"),
        )
    except ValueError as exc:
        raise EmbodimentSchemaError(f"invalid {entry.where}: {exc}") from exc
    return ProvenancedAsset(asset=asset, provenance=provenance)


def _rates_to_dict(value: ControlRates | None) -> dict[str, float | None] | None:
    if value is None:
        return None
    return {"policy_hz": value.policy_hz, "low_level_hz": value.low_level_hz}


def _parse_rates(document: decode.Document) -> ControlRates | None:
    if document["rates"] is None:
        return None
    entry = decode.exactly(decode.mapping(document, "rates"), {"policy_hz", "low_level_hz"})
    return ControlRates(
        policy_hz=decode.number(entry, "policy_hz"),
        low_level_hz=decode.optional_number(entry, "low_level_hz"),
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


def _parse_base_mount(document: decode.Document) -> BaseMount | None:
    if document["base_mount"] is None:
        return None
    entry = decode.exactly(
        decode.mapping(document, "base_mount"),
        {"kind", "frame", "half_extents", "centre", "clearance_m"},
    )
    try:
        kind = MountKind(decode.text(entry, "kind"))
    except ValueError as exc:
        raise EmbodimentSchemaError(f"{entry.where}.kind is unknown") from exc
    return BaseMount(
        kind=kind,
        frame=decode.text(entry, "frame"),
        half_extents=_pair(entry, "half_extents"),
        centre=_pair(entry, "centre"),
        clearance_m=decode.number(entry, "clearance_m"),
    )


def _operator_mount_to_dict(value: OperatorMount) -> dict[str, str]:
    return {
        "site": value.site.value,
        "root_frame": value.root_frame,
        "attachment_frame": value.attachment_frame,
    }


def _parse_operator_mounts(document: decode.Document) -> tuple[OperatorMount, ...]:
    mounts: list[OperatorMount] = []
    for raw in decode.documents(document, "operator_mounts"):
        entry = decode.exactly(raw, {"site", "root_frame", "attachment_frame"})
        try:
            site = OperatorSite(decode.text(entry, "site"))
        except ValueError as exc:
            raise EmbodimentSchemaError(f"{entry.where}.site is unknown") from exc
        mounts.append(
            OperatorMount(
                site=site,
                root_frame=decode.text(entry, "root_frame"),
                attachment_frame=decode.text(entry, "attachment_frame"),
            )
        )
    return tuple(mounts)


def _parse_lineage(document: decode.Document) -> Lineage:
    entry = decode.exactly(document, {"family", "variant", "revision"})
    return Lineage(
        family=decode.text(entry, "family"),
        variant=decode.text(entry, "variant"),
        revision=decode.text(entry, "revision"),
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
    for packaged in packaged_assets(definition):
        key = (packaged.relpath, packaged.sha256)
        if key not in seen:
            assets.append(packaged.provenanced_asset())
            seen.add(key)
    return Embodiment(
        name=definition.name,
        label=definition.label,
        kind=definition.kind,
        lineage=definition.lineage,
        components=tuple(_portable_component(component) for component in definition.attachments),
        assets=tuple(assets),
        rates=definition.rates,
        base_mount=definition.base_mount,
        operator_mounts=definition.operator_mounts,
    )


def packaged_assets(definition: EmbodimentDefinition) -> list[PackagedAsset]:
    """Every asset a definition declares, still carrying its declared provenance.

    Public because the audience of an asset directory is read off the declarations
    themselves (`known.asset_audiences`), and a declaration's licence survives only on
    the `PackagedAsset` — `Embodiment.assets` has already been flattened to portable
    refs by then. Duplicates are kept: deduplication is the caller's law, not this one's.
    """
    assets: list[PackagedAsset] = []
    for component in definition.attachments:
        part = component.part
        if isinstance(part, ArmSpec | JointGroupSpec | GripperSpec | MobileBaseSpec):
            assets.extend(part.assets)
    assets.extend(definition.extra_assets)
    return assets


def _validate_urdf(
    name: EmbodimentName,
    assets: tuple[AssetRef, ...],
    urdf: bytes,
    *,
    components: tuple[Component, ...],
    operator_mounts: tuple[OperatorMount, ...],
) -> None:
    urdfs = tuple(
        asset
        for asset in assets
        if asset.format is AssetFormat.URDF and asset.role is AssetRole.DESCRIPTION
    )
    if len(urdfs) != 1:
        raise MissingUrdfError(str(name), len(urdfs))
    actual = hashlib.sha256(urdf).hexdigest()
    if actual != urdfs[0].sha256:
        # AssetDigestMismatchError, not the bare integrity parent: it subclasses
        # EmbodimentError too, so every boundary that maps this package's refusals to a
        # typed rejection (the embodiment door's 422) catches the forgery/mistake case.
        raise AssetDigestMismatchError(urdfs[0].location, urdfs[0].sha256, actual)
    try:
        root = ET.fromstring(urdf)
    except ET.ParseError as exc:
        raise EmbodimentSchemaError(f"{name}: authoritative URDF is invalid XML") from exc
    if root.tag != "robot" or not root.attrib.get("name", "").strip():
        raise EmbodimentSchemaError(f"{name}: authoritative URDF root must be a named <robot>")
    links = {link.get("name"): link for link in root.iter("link") if link.get("name")}
    camera_frames = {
        component.mount.frame for component in components if isinstance(component.part, CameraSpec)
    }
    missing_cameras = camera_frames - links.keys()
    if missing_cameras:
        raise EmbodimentSchemaError(
            f"{name}: camera frames are absent from the authoritative URDF: "
            f"{', '.join(sorted(missing_cameras))}"
        )
    non_optical = {
        frame
        for frame in camera_frames
        if links[frame].get("data-frame-convention") != "camera_optical"
    }
    if non_optical:
        raise EmbodimentSchemaError(
            f"{name}: camera frames are not declared camera_optical: "
            f"{', '.join(sorted(non_optical))}"
        )
    operator_frames = {
        frame for mount in operator_mounts for frame in (mount.root_frame, mount.attachment_frame)
    }
    missing_operator_frames = operator_frames - links.keys()
    if missing_operator_frames:
        raise EmbodimentSchemaError(
            f"{name}: operator mount frames are absent from the authoritative URDF: "
            f"{', '.join(sorted(missing_operator_frames))}"
        )


# -- the arities this schema fixes ------------------------------------------------------
#
# `decode` reads a vector; how long that vector must be is this schema's fact, so the
# arity checks live here. Each returns the exact tuple type its part field is declared
# as, which is what keeps the constructors honest without a cast.


def _parse_optional_vector(document: decode.Document, key: str) -> tuple[float, ...] | None:
    return None if document[key] is None else decode.numbers(document, key)


def _pair(document: decode.Document, key: str) -> tuple[float, float]:
    values = decode.numbers(document, key)
    if len(values) != 2:
        raise EmbodimentSchemaError(f"{document.where}.{key} must contain two numbers")
    return values[0], values[1]


def _optional_pair(document: decode.Document, key: str) -> tuple[float, float] | None:
    return None if document[key] is None else _pair(document, key)


def _optional_triple(document: decode.Document, key: str) -> tuple[float, float, float] | None:
    if document[key] is None:
        return None
    values = decode.numbers(document, key)
    if len(values) != 3:
        raise EmbodimentSchemaError(f"{document.where}.{key} must contain three numbers")
    return values[0], values[1], values[2]
