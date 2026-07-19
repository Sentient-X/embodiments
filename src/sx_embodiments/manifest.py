"""Immutable, storage-neutral embodiment manifests (wire schema_version 2).

Runtime kinematics, simulator adapters, network fetching, and serialization intentionally
live at consumer boundaries. These records are the common vocabulary exchanged by those
implementations. Version 2 adds the derived structure (kind, lineage, channel layout,
cameras, rates) to the v1 identity + asset bundle; v1 is no longer emitted, and readers of
either version fail closed on the other.
"""

import hashlib
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import PurePosixPath
from typing import cast

from .assets import AssetFormat, AssetRef, AssetRole, PackagedAsset
from .compose import EmbodimentSpec, camera_bindings, flat_layout
from .errors import AssetIntegrityError, ManifestSchemaError
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
    MobileBaseSpec,
    SensorModel,
)

SCHEMA_VERSION = 2


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
        if self.dof is not None and self.dof < 0:
            raise ManifestSchemaError("embodiment dof must be non-negative")
        if self.link_count is not None and self.link_count <= 0:
            raise ManifestSchemaError("embodiment link_count must be positive")
        if self.policy_hz is not None and (
            not math.isfinite(self.policy_hz) or self.policy_hz <= 0.0
        ):
            raise ManifestSchemaError("embodiment policy_hz must be positive and finite")
        if self.layout is not None:
            if self.layout.embodiment_id != self.embodiment_id:
                raise ManifestSchemaError("layout embodiment_id must match the manifest")
            if self.dof is not None and self.dof != self.layout.action_dim:
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
        if any(not math.isfinite(binding.camera.fps) for binding in self.cameras):
            raise ManifestSchemaError("embodiment camera fps must be finite")
        if self.rates is not None and (
            not math.isfinite(self.rates.policy_hz)
            or (self.rates.low_level_hz is not None and not math.isfinite(self.rates.low_level_hz))
        ):
            raise ManifestSchemaError("embodiment control rates must be finite")
        # The same content blob may appear at multiple logical paths inside one bundle;
        # a duplicate is only a member with the SAME (uri, sha256, role, logical_path).
        identities = {
            (asset.uri, asset.sha256, asset.role, asset.logical_path) for asset in self.assets
        }
        if len(identities) != len(self.assets):
            raise ManifestSchemaError("embodiment contains duplicate asset references")

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
    """Parse the complete v2 wire form, failing closed on malformed structure."""
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
                ),
            )
        )
    return tuple(cameras)


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
    try:
        return AssetRef(
            uri=uri,
            sha256=sha256,
            format=_parse_asset_format(fmt),
            role=_parse_asset_role(role),
            media_type=media_type,
            byte_size=_optional_int(entry.get("byte_size"), "byte_size"),
            logical_path=PurePosixPath(logical_path) if logical_path is not None else None,
        )
    except AssetIntegrityError as exc:
        raise ManifestSchemaError(f"invalid asset entry: {exc}") from exc


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
    packaged: list[PackagedAsset] = []
    for attachment in spec.attachments:
        if isinstance(attachment.part, ArmSpec | JointGroupSpec | GripperSpec | MobileBaseSpec):
            packaged.extend(attachment.part.assets)
    packaged.extend(spec.extra_assets)
    for asset in packaged:
        key = (asset.relpath, asset.sha256)
        if key in seen:
            continue
        seen.add(key)
        refs.append(asset.ref())
    layout = flat_layout(spec) if spec.layout_declared() else None
    return EmbodimentManifest(
        embodiment_id=spec.embodiment_id,
        name=spec.name,
        assets=tuple(refs),
        dof=layout.action_dim if layout is not None else None,
        policy_hz=spec.rates.policy_hz if spec.rates is not None else None,
        kind=spec.kind,
        lineage=spec.lineage,
        layout=layout,
        cameras=camera_bindings(spec),
        rates=spec.rates,
    )
