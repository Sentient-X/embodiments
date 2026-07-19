"""Immutable, storage-neutral episode-ready embodiment manifests (wire version 3).

Runtime kinematics, simulator adapters, network fetching, and serialization intentionally
live at consumer boundaries. Version 3 makes the authoritative URDF, action layout, asset
provenance, and camera mounts mandatory. Historical v2 documents remain catalog legacy rows;
this package neither emits nor silently upgrades records missing those facts.
"""

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import PurePosixPath
from typing import cast

from .assets import AssetFormat, AssetProvenance, AssetRef, AssetRole, PackagedAsset
from .compose import EmbodimentSpec, camera_bindings, flat_layout
from .errors import AssetIntegrityError, ManifestSchemaError, MissingUrdfError
from .identity import (
    EmbodimentId,
    EmbodimentKind,
    EmbodimentManifestDigest,
    EmbodimentRef,
    Lineage,
    PartId,
)
from .layout import ChannelKind, ChannelSlot, FlatLayout
from .parts import (
    ArmSpec,
    CameraBinding,
    CameraModality,
    CameraSpec,
    ControlRates,
    GripperSpec,
    JointGroupSpec,
    LensProjection,
    MobileBaseSpec,
    SensorModel,
)

SCHEMA_VERSION = 3


@dataclass(frozen=True, slots=True)
class ExecutionCapabilities:
    """Task-admission facts derived from the physical composition."""

    manipulator_count: int
    mobile_base: bool

    def __post_init__(self) -> None:
        if isinstance(self.manipulator_count, bool) or self.manipulator_count < 0:
            raise ManifestSchemaError("manipulator_count must be a non-negative integer")
        if type(self.mobile_base) is not bool:
            raise ManifestSchemaError("mobile_base must be a boolean")


@dataclass(frozen=True, slots=True)
class EmbodimentManifest:
    """Versioned identity, asset bundle, and derived structure for one embodiment."""

    embodiment_id: EmbodimentId
    name: str
    assets: tuple[AssetRef, ...]
    schema_version: int = SCHEMA_VERSION
    dof: int | None = None
    link_count: int | None = None
    policy_hz: float | None = None  # control-loop / policy-query rate; None when unbound
    kind: EmbodimentKind | None = None
    lineage: Lineage | None = None
    layout: FlatLayout | None = None
    capabilities: ExecutionCapabilities | None = None
    cameras: tuple[CameraBinding, ...] = ()
    rates: ControlRates | None = None

    def __post_init__(self) -> None:
        eid = str(self.embodiment_id)
        if not eid.strip():
            raise ManifestSchemaError("embodiment_id must not be empty")
        if not self.name.strip():
            raise ManifestSchemaError("embodiment name must not be empty")
        if not self.assets:
            raise ManifestSchemaError("embodiment must reference at least one asset")
        if type(self.schema_version) is not int:
            raise ManifestSchemaError("embodiment schema_version must be an integer")
        if self.schema_version != SCHEMA_VERSION:
            raise ManifestSchemaError(
                f"unsupported embodiment schema_version: {self.schema_version}"
            )
        # One blob may intentionally occur at multiple logical bundle paths.
        identities = {
            (asset.uri, asset.sha256, asset.role, asset.logical_path) for asset in self.assets
        }
        if len(identities) != len(self.assets):
            raise ManifestSchemaError("embodiment contains duplicate asset references")
        urdfs = [
            asset
            for asset in self.assets
            if asset.format is AssetFormat.URDF and asset.role is AssetRole.DESCRIPTION
        ]
        if len(urdfs) != 1:
            raise MissingUrdfError(eid, len(urdfs))
        if any(asset.provenance is None for asset in self.assets):
            raise ManifestSchemaError("every manifest asset must carry provenance")
        if (
            self.kind is None
            or self.lineage is None
            or self.layout is None
            or self.capabilities is None
        ):
            raise ManifestSchemaError(
                "manifest kind, lineage, action layout, and capabilities are mandatory"
            )
        if self.dof is None or self.dof < 0:
            raise ManifestSchemaError("embodiment dof is mandatory and must be non-negative")
        if self.link_count is None or self.link_count <= 0:
            raise ManifestSchemaError("embodiment link_count is mandatory and must be positive")
        if self.policy_hz is not None and (
            not math.isfinite(self.policy_hz) or self.policy_hz <= 0.0
        ):
            raise ManifestSchemaError("embodiment policy_hz must be positive and finite")
        if self.layout.embodiment_id != self.embodiment_id:
            raise ManifestSchemaError("layout embodiment_id must match the manifest")
        if self.dof != self.layout.action_dim:
            raise ManifestSchemaError("embodiment dof must match the layout width")
        if (
            self.rates is not None
            and self.policy_hz is not None
            and self.policy_hz != self.rates.policy_hz
        ):
            raise ManifestSchemaError("policy_hz must match rates.policy_hz")
        camera_names = [binding.name for binding in self.cameras]
        if len(set(camera_names)) != len(camera_names):
            raise ManifestSchemaError("embodiment camera names must be unique")
        if any(not binding.frame.strip() for binding in self.cameras):
            raise ManifestSchemaError("every embodiment camera needs an explicit mount frame")
        if any(not math.isfinite(binding.camera.fps) for binding in self.cameras):
            raise ManifestSchemaError("embodiment camera fps must be finite")
        if self.rates is not None and (
            not math.isfinite(self.rates.policy_hz)
            or (self.rates.low_level_hz is not None and not math.isfinite(self.rates.low_level_hz))
        ):
            raise ManifestSchemaError("embodiment control rates must be finite")

    def to_dict(self) -> dict[str, object]:
        """Return the canonical JSON-compatible wire representation."""
        return {
            "schema_version": self.schema_version,
            "embodiment_id": str(self.embodiment_id),
            "name": self.name,
            "dof": self.dof,
            "link_count": self.link_count,
            "policy_hz": self.policy_hz,
            "kind": self.kind.value if self.kind is not None else None,
            "lineage": (
                {
                    "family": self.lineage.family,
                    "variant": self.lineage.variant,
                    "revision": self.lineage.revision,
                }
                if self.lineage is not None
                else None
            ),
            "layout": (
                [
                    {
                        "index": slot.index,
                        "instance": slot.instance,
                        "part_id": str(slot.part_id),
                        "joint_name": slot.joint_name,
                        "kind": slot.kind.value,
                    }
                    for slot in self.layout.slots
                ]
                if self.layout is not None
                else None
            ),
            "capabilities": (
                {
                    "manipulator_count": self.capabilities.manipulator_count,
                    "mobile_base": self.capabilities.mobile_base,
                }
                if self.capabilities is not None
                else None
            ),
            "cameras": [
                {
                    "name": binding.name,
                    "part_id": str(binding.camera.part_id),
                    "model": binding.camera.model.value,
                    "modality": binding.camera.modality.value,
                    "fps": binding.camera.fps,
                    "projection": binding.camera.projection.value,
                    "resolution": (
                        list(binding.camera.resolution)
                        if binding.camera.resolution is not None
                        else None
                    ),
                    "parent_instance": binding.parent_instance,
                    "frame": binding.frame,
                }
                for binding in self.cameras
            ],
            "rates": (
                {"policy_hz": self.rates.policy_hz, "low_level_hz": self.rates.low_level_hz}
                if self.rates is not None
                else None
            ),
            "assets": [
                {
                    "uri": asset.uri,
                    "sha256": asset.sha256,
                    "format": asset.format.value,
                    "role": asset.role.value,
                    "media_type": asset.media_type,
                    "byte_size": asset.byte_size,
                    # Omitted (not null) when unset: single-file manifests keep the
                    # byte-identical canonical JSON their registered digests were
                    # minted from; only multi-file bundle members carry the key.
                    **(
                        {"logical_path": str(asset.logical_path)}
                        if asset.logical_path is not None
                        else {}
                    ),
                    "provenance": (
                        {
                            "repository": asset.provenance.repository,
                            "revision": asset.provenance.revision,
                            "path": asset.provenance.path,
                            "license_id": asset.provenance.license_id,
                            "generator": asset.provenance.generator,
                        }
                        if asset.provenance is not None
                        else None
                    ),
                }
                for asset in self.assets
            ],
        }

    def canonical_json(self) -> str:
        """Return the deterministic UTF-8 JSON text whose digest identifies this revision."""
        return json.dumps(self.to_dict(), ensure_ascii=False, separators=(",", ":"), sort_keys=True)

    def digest(self) -> EmbodimentManifestDigest:
        """Content identity of the complete manifest, including every referenced asset digest."""
        return EmbodimentManifestDigest(
            hashlib.sha256(self.canonical_json().encode("utf-8")).hexdigest()
        )

    def ref(self) -> EmbodimentRef:
        """Return the body-and-revision reference consumers carry across lifecycle gates."""
        return EmbodimentRef(self.embodiment_id, self.digest())


def _require(document: Mapping[str, object], key: str) -> object:
    if key not in document:
        raise ManifestSchemaError(f"manifest document missing required key {key!r}")
    return document[key]


def manifest_from_dict(document: Mapping[str, object]) -> EmbodimentManifest:
    """Parse the complete v3 wire form, failing closed on malformed structure."""
    version = _require(document, "schema_version")
    if isinstance(version, bool) or not isinstance(version, int) or version != SCHEMA_VERSION:
        raise ManifestSchemaError(f"unsupported embodiment schema_version: {version!r}")
    embodiment_id = _require(document, "embodiment_id")
    name = _require(document, "name")
    raw_assets = _require(document, "assets")
    if not isinstance(embodiment_id, str) or not isinstance(name, str):
        raise ManifestSchemaError("embodiment_id and name must be strings")
    if not isinstance(raw_assets, list):
        raise ManifestSchemaError("assets must be a list")
    assets = tuple(_parse_asset_entry(entry) for entry in cast(list[object], raw_assets))
    parsed_id = EmbodimentId(embodiment_id)
    return EmbodimentManifest(
        embodiment_id=parsed_id,
        name=name,
        assets=assets,
        dof=_optional_int(document.get("dof"), "dof"),
        link_count=_optional_int(document.get("link_count"), "link_count"),
        policy_hz=_optional_float(document.get("policy_hz"), "policy_hz"),
        kind=_parse_kind(_require(document, "kind")),
        lineage=_parse_lineage(_require(document, "lineage")),
        layout=_parse_layout(_require(document, "layout"), parsed_id),
        capabilities=_parse_capabilities(_require(document, "capabilities")),
        cameras=_parse_cameras(_require(document, "cameras")),
        rates=_parse_rates(_require(document, "rates")),
    )


def _mapping(raw: object, label: str) -> Mapping[str, object]:
    if not isinstance(raw, Mapping):
        raise ManifestSchemaError(f"{label} must be a mapping")
    return cast(Mapping[str, object], raw)


def _string(entry: Mapping[str, object], key: str, label: str) -> str:
    value = _require(entry, key)
    if not isinstance(value, str):
        raise ManifestSchemaError(f"{label}.{key} must be a string")
    return value


def _parse_kind(raw: object) -> EmbodimentKind | None:
    if raw is None:
        return None
    if not isinstance(raw, str):
        raise ManifestSchemaError("kind must be a string or null")
    try:
        return EmbodimentKind(raw)
    except ValueError as exc:
        raise ManifestSchemaError(f"unknown embodiment kind {raw!r}") from exc


def _parse_lineage(raw: object) -> Lineage | None:
    if raw is None:
        return None
    entry = _mapping(raw, "lineage")
    return Lineage(
        family=_string(entry, "family", "lineage"),
        variant=_string(entry, "variant", "lineage"),
        revision=_string(entry, "revision", "lineage"),
    )


def _parse_layout(raw: object, embodiment_id: EmbodimentId) -> FlatLayout | None:
    if raw is None:
        return None
    if not isinstance(raw, list):
        raise ManifestSchemaError("layout must be a list or null")
    slots: list[ChannelSlot] = []
    for offset, item in enumerate(cast(list[object], raw)):
        entry = _mapping(item, f"layout[{offset}]")
        index = _require(entry, "index")
        if isinstance(index, bool) or not isinstance(index, int):
            raise ManifestSchemaError(f"layout[{offset}].index must be an integer")
        raw_kind = _string(entry, "kind", f"layout[{offset}]")
        try:
            kind = ChannelKind(raw_kind)
        except ValueError as exc:
            raise ManifestSchemaError(f"unknown channel kind {raw_kind!r}") from exc
        slots.append(
            ChannelSlot(
                index=index,
                instance=_string(entry, "instance", f"layout[{offset}]"),
                part_id=PartId(_string(entry, "part_id", f"layout[{offset}]")),
                joint_name=_string(entry, "joint_name", f"layout[{offset}]"),
                kind=kind,
            )
        )
    return FlatLayout(embodiment_id=embodiment_id, slots=tuple(slots))


def _parse_cameras(raw: object) -> tuple[CameraBinding, ...]:
    if not isinstance(raw, list):
        raise ManifestSchemaError("cameras must be a list")
    cameras: list[CameraBinding] = []
    for offset, item in enumerate(cast(list[object], raw)):
        label = f"cameras[{offset}]"
        entry = _mapping(item, label)
        raw_model = _string(entry, "model", label)
        raw_modality = _string(entry, "modality", label)
        try:
            model = SensorModel(raw_model)
            modality = CameraModality(raw_modality)
            projection = LensProjection(_string(entry, "projection", label))
        except ValueError as exc:
            raise ManifestSchemaError(f"unknown camera vocabulary in {label}") from exc
        fps = _optional_float(_require(entry, "fps"), f"{label}.fps")
        if fps is None:
            raise ManifestSchemaError(f"{label}.fps must be a number")
        cameras.append(
            CameraBinding(
                name=_string(entry, "name", label),
                camera=CameraSpec(
                    part_id=PartId(_string(entry, "part_id", label)),
                    model=model,
                    modality=modality,
                    fps=fps,
                    projection=projection,
                    resolution=_parse_resolution(_require(entry, "resolution"), label),
                ),
                parent_instance=_string(entry, "parent_instance", label),
                frame=_string(entry, "frame", label),
            )
        )
    return tuple(cameras)


def _parse_capabilities(raw: object) -> ExecutionCapabilities | None:
    if raw is None:
        return None
    entry = _mapping(raw, "capabilities")
    manipulator_count = _require(entry, "manipulator_count")
    mobile_base = _require(entry, "mobile_base")
    if isinstance(manipulator_count, bool) or not isinstance(manipulator_count, int):
        raise ManifestSchemaError("capabilities.manipulator_count must be an integer")
    if type(mobile_base) is not bool:
        raise ManifestSchemaError("capabilities.mobile_base must be a boolean")
    return ExecutionCapabilities(
        manipulator_count=manipulator_count,
        mobile_base=mobile_base,
    )


def _parse_rates(raw: object) -> ControlRates | None:
    if raw is None:
        return None
    entry = _mapping(raw, "rates")
    policy_hz = _optional_float(_require(entry, "policy_hz"), "rates.policy_hz")
    if policy_hz is None:
        raise ManifestSchemaError("rates.policy_hz must be a number")
    return ControlRates(
        policy_hz=policy_hz,
        low_level_hz=_optional_float(_require(entry, "low_level_hz"), "rates.low_level_hz"),
    )


def _parse_asset_entry(raw: object) -> AssetRef:
    if not isinstance(raw, Mapping):
        raise ManifestSchemaError(f"asset entry must be a mapping, got {type(raw).__name__}")
    entry = cast(Mapping[str, object], raw)
    uri = entry.get("uri")
    sha256 = entry.get("sha256")
    fmt = entry.get("format")
    role = entry.get("role")
    if (
        not isinstance(uri, str)
        or not isinstance(sha256, str)
        or not isinstance(fmt, str)
        or not isinstance(role, str)
    ):
        raise ManifestSchemaError("asset entry needs string uri/sha256/format/role")
    media_type = entry.get("media_type")
    if media_type is not None and not isinstance(media_type, str):
        raise ManifestSchemaError("asset media_type must be a string or null")
    logical_path = entry.get("logical_path")
    if logical_path is not None and not isinstance(logical_path, str):
        raise ManifestSchemaError("asset logical_path must be a string or null")
    provenance = _parse_provenance(_require(entry, "provenance"))
    try:
        return AssetRef(
            uri=uri,
            sha256=sha256,
            format=_parse_asset_format(fmt),
            role=_parse_asset_role(role),
            media_type=media_type,
            byte_size=_optional_int(entry.get("byte_size"), "byte_size"),
            logical_path=PurePosixPath(logical_path) if logical_path is not None else None,
            provenance=provenance,
        )
    except AssetIntegrityError as exc:
        raise ManifestSchemaError(f"invalid asset entry: {exc}") from exc


def _parse_provenance(raw: object) -> AssetProvenance:
    entry = _mapping(raw, "provenance")
    generator = _require(entry, "generator")
    if generator is not None and not isinstance(generator, str):
        raise ManifestSchemaError("provenance.generator must be a string or null")
    return AssetProvenance(
        repository=_string(entry, "repository", "provenance"),
        revision=_string(entry, "revision", "provenance"),
        path=_string(entry, "path", "provenance"),
        license_id=_string(entry, "license_id", "provenance"),
        generator=generator,
    )


def _parse_resolution(raw: object, label: str) -> tuple[int, int] | None:
    if raw is None:
        return None
    if not isinstance(raw, list):
        raise ManifestSchemaError(f"{label}.resolution must be [width, height] or null")
    dimensions = cast(list[object], raw)
    if len(dimensions) != 2:
        raise ManifestSchemaError(f"{label}.resolution must be [width, height] or null")
    width, height = dimensions
    if (
        isinstance(width, bool)
        or not isinstance(width, int)
        or isinstance(height, bool)
        or not isinstance(height, int)
        or width <= 0
        or height <= 0
    ):
        raise ManifestSchemaError(f"{label}.resolution dimensions must be positive integers")
    return width, height


def _optional_int(value: object, key: str) -> int | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int):
        raise ManifestSchemaError(f"{key} must be an integer or null")
    return value


def _optional_float(value: object, key: str) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ManifestSchemaError(f"{key} must be a number or null")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ManifestSchemaError(f"{key} must be finite")
    return parsed


def _parse_asset_format(value: str) -> AssetFormat:
    try:
        return AssetFormat(value)
    except ValueError as exc:
        raise ManifestSchemaError(f"unknown asset format {value!r}") from exc


def _parse_asset_role(value: str) -> AssetRole:
    try:
        return AssetRole(value)
    except ValueError as exc:
        raise ManifestSchemaError(f"unknown asset role {value!r}") from exc


def manifest_for(spec: EmbodimentSpec) -> EmbodimentManifest:
    """Derive the wire manifest from a spec at an explicit wiring site.

    Resolves packaged assets on disk (fails closed when the asset tree is unavailable) and
    dedups the parts' asset bundles.
    """
    refs: list[AssetRef] = []
    seen: set[tuple[str, str]] = set()
    packaged = _packaged_assets(spec)
    for asset in packaged:
        key = (asset.relpath, asset.sha256)
        if key in seen:
            continue
        seen.add(key)
        refs.append(asset.ref())
    layout = flat_layout(spec) if spec.layout_declared() else None
    urdf = authoritative_urdf(spec)
    try:
        urdf_root = ET.fromstring(urdf.path().read_bytes())
    except ET.ParseError as exc:
        raise ManifestSchemaError(
            f"{spec.embodiment_id}: authoritative URDF is invalid XML"
        ) from exc
    link_count = sum(1 for _ in urdf_root.iter("link"))
    body = spec.body_attachments()
    return EmbodimentManifest(
        embodiment_id=spec.embodiment_id,
        name=spec.name,
        assets=tuple(refs),
        dof=layout.action_dim if layout is not None else None,
        link_count=link_count,
        policy_hz=spec.rates.policy_hz if spec.rates is not None else None,
        kind=spec.kind,
        lineage=spec.lineage,
        layout=layout,
        capabilities=ExecutionCapabilities(
            manipulator_count=sum(isinstance(item.part, GripperSpec) for item in body),
            mobile_base=any(isinstance(item.part, MobileBaseSpec) for item in body),
        ),
        cameras=camera_bindings(spec),
        rates=spec.rates,
    )


def authoritative_urdf(spec: EmbodimentSpec) -> PackagedAsset:
    """Return the one packaged URDF description for an episode-ready embodiment."""
    matches = tuple(
        {
            (asset.relpath, asset.sha256): asset
            for asset in _packaged_assets(spec)
            if asset.format is AssetFormat.URDF and asset.role is AssetRole.DESCRIPTION
        }.values()
    )
    if len(matches) != 1:
        raise MissingUrdfError(str(spec.embodiment_id), len(matches))
    return matches[0]


def _packaged_assets(spec: EmbodimentSpec) -> list[PackagedAsset]:
    packaged: list[PackagedAsset] = []
    for attachment in spec.attachments:
        if isinstance(attachment.part, ArmSpec | JointGroupSpec | GripperSpec | MobileBaseSpec):
            packaged.extend(attachment.part.assets)
    packaged.extend(spec.extra_assets)
    return packaged
