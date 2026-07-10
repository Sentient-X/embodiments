"""Immutable, storage-neutral embodiment manifests.

Runtime kinematics, simulator adapters, network fetching, and serialization intentionally live at
consumer boundaries. These records are the common vocabulary exchanged by those implementations.
"""

from dataclasses import dataclass
from enum import StrEnum
from typing import NewType
from urllib.parse import urlparse

EmbodimentId = NewType("EmbodimentId", str)


class AssetFormat(StrEnum):
    """Portable formats understood by the robotics data platform."""

    URDF = "urdf"
    MJCF = "mjcf"
    USD = "usd"
    USDA = "usda"
    USDC = "usdc"
    USDZ = "usdz"
    MESH = "mesh"
    CALIBRATION = "calibration"
    OTHER = "other"


class AssetRole(StrEnum):
    """How an asset participates in an embodiment bundle."""

    DESCRIPTION = "description"
    GEOMETRY = "geometry"
    COLLISION = "collision"
    CALIBRATION = "calibration"
    TEXTURE = "texture"
    CONTROLLER = "controller"
    OTHER = "other"


@dataclass(frozen=True, slots=True)
class AssetRef:
    """Content-addressed robotics asset at a catalog-resolvable URI."""

    uri: str
    sha256: str
    format: AssetFormat
    role: AssetRole
    media_type: str | None = None
    byte_size: int | None = None

    def __post_init__(self) -> None:
        parsed = urlparse(self.uri)
        if not parsed.scheme:
            raise ValueError("asset uri must be absolute and include a scheme")
        digest = self.sha256.lower()
        if (
            self.sha256 != digest
            or len(digest) != 64
            or any(char not in "0123456789abcdef" for char in digest)
        ):
            raise ValueError("asset sha256 must be 64 lowercase hexadecimal characters")
        if self.byte_size is not None and self.byte_size < 0:
            raise ValueError("asset byte_size must be non-negative")


@dataclass(frozen=True, slots=True)
class EmbodimentManifest:
    """Versioned identity and asset bundle for one robot embodiment revision."""

    embodiment_id: EmbodimentId
    name: str
    assets: tuple[AssetRef, ...]
    schema_version: int = 1
    dof: int | None = None
    link_count: int | None = None

    def __post_init__(self) -> None:
        if not self.embodiment_id.strip():
            raise ValueError("embodiment_id must not be empty")
        if not self.name.strip():
            raise ValueError("embodiment name must not be empty")
        if not self.assets:
            raise ValueError("embodiment must reference at least one asset")
        if self.schema_version != 1:
            raise ValueError(f"unsupported embodiment schema_version: {self.schema_version}")
        if self.dof is not None and self.dof < 0:
            raise ValueError("embodiment dof must be non-negative")
        if self.link_count is not None and self.link_count <= 0:
            raise ValueError("embodiment link_count must be positive")
        identities = {(asset.uri, asset.sha256, asset.role) for asset in self.assets}
        if len(identities) != len(self.assets):
            raise ValueError("embodiment contains duplicate asset references")
