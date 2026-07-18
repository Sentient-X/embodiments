"""Immutable, storage-neutral embodiment manifests (wire schema_version 2).

Runtime kinematics, simulator adapters, network fetching, and serialization intentionally
live at consumer boundaries. These records are the common vocabulary exchanged by those
implementations. Version 2 adds the derived structure (kind, lineage, channel layout,
cameras, rates) to the v1 identity + asset bundle; v1 is no longer emitted, and readers of
either version fail closed on the other.
"""

from collections.abc import Mapping
from dataclasses import dataclass
from typing import cast

from .assets import AssetFormat, AssetRef, AssetRole, PackagedAsset
from .compose import EmbodimentSpec, camera_bindings, flat_layout
from .errors import ManifestSchemaError
from .identity import EmbodimentId, EmbodimentKind, Lineage
from .layout import FlatLayout
from .parts import (
    ArmSpec,
    CameraBinding,
    ControlRates,
    GripperSpec,
    JointGroupSpec,
    MobileBaseSpec,
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
        if self.schema_version != SCHEMA_VERSION:
            raise ManifestSchemaError(
                f"unsupported embodiment schema_version: {self.schema_version}"
            )
        if self.dof is not None and self.dof < 0:
            raise ManifestSchemaError("embodiment dof must be non-negative")
        if self.link_count is not None and self.link_count <= 0:
            raise ManifestSchemaError("embodiment link_count must be positive")
        if self.policy_hz is not None and self.policy_hz <= 0.0:
            raise ManifestSchemaError("embodiment policy_hz must be positive")
        identities = {(asset.uri, asset.sha256, asset.role) for asset in self.assets}
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
                    "model": binding.camera.model.value,
                    "modality": binding.camera.modality.value,
                    "fps": binding.camera.fps,
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
                }
                for asset in self.assets
            ],
        }


def _require(document: Mapping[str, object], key: str) -> object:
    if key not in document:
        raise ManifestSchemaError(f"manifest document missing required key {key!r}")
    return document[key]


def manifest_from_dict(document: Mapping[str, object]) -> EmbodimentManifest:
    """Parse the v2 wire form, failing closed on any other version or malformed shape.

    The derived sections (kind/lineage/layout/cameras/rates) are wire-emitted context for
    non-Python consumers; the parsed record retains identity, descriptive facts, and the
    asset bundle — consumers needing the derived structure resolve the id against the
    registry, the single source.
    """
    version = _require(document, "schema_version")
    if version != SCHEMA_VERSION:
        raise ManifestSchemaError(f"unsupported embodiment schema_version: {version!r}")
    embodiment_id = _require(document, "embodiment_id")
    name = _require(document, "name")
    raw_assets = _require(document, "assets")
    if not isinstance(embodiment_id, str) or not isinstance(name, str):
        raise ManifestSchemaError("embodiment_id and name must be strings")
    if not isinstance(raw_assets, list):
        raise ManifestSchemaError("assets must be a list")
    assets = tuple(_parse_asset_entry(entry) for entry in cast(list[object], raw_assets))
    return EmbodimentManifest(
        embodiment_id=EmbodimentId(embodiment_id),
        name=name,
        assets=assets,
        dof=_optional_int(document.get("dof"), "dof"),
        link_count=_optional_int(document.get("link_count"), "link_count"),
        policy_hz=_optional_float(document.get("policy_hz"), "policy_hz"),
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
    return AssetRef(
        uri=uri,
        sha256=sha256,
        format=_parse_asset_format(fmt),
        role=_parse_asset_role(role),
        media_type=media_type,
        byte_size=_optional_int(entry.get("byte_size"), "byte_size"),
    )


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
    return float(value)


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
