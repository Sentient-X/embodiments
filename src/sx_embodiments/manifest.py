"""Immutable, storage-neutral embodiment manifests.

Runtime kinematics, simulator adapters, network fetching, and serialization intentionally live at
consumer boundaries. These records are the common vocabulary exchanged by those implementations.
"""

import dataclasses
import hashlib
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path
from typing import NewType, Self
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

    @classmethod
    def from_bytes(
        cls,
        content: bytes,
        *,
        uri: str,
        format: AssetFormat,
        role: AssetRole,
        media_type: str | None = None,
    ) -> Self:
        """Create a canonical reference after hashing bytes at an ingest boundary."""
        return cls(
            uri=uri,
            sha256=hashlib.sha256(content).hexdigest(),
            format=format,
            role=role,
            media_type=media_type,
            byte_size=len(content),
        )

    @classmethod
    def from_path(
        cls,
        path: Path,
        *,
        format: AssetFormat,
        role: AssetRole,
        media_type: str | None = None,
    ) -> Self:
        """Hash a local asset and bind it to its absolute file URI."""
        resolved = path.resolve(strict=True)
        return cls.from_bytes(
            resolved.read_bytes(),
            uri=resolved.as_uri(),
            format=format,
            role=role,
            media_type=media_type,
        )


@dataclass(frozen=True, slots=True)
class EmbodimentManifest:
    """Versioned identity and asset bundle for one robot embodiment revision."""

    embodiment_id: EmbodimentId
    name: str
    assets: tuple[AssetRef, ...]
    schema_version: int = 1
    dof: int | None = None
    link_count: int | None = None
    policy_hz: float | None = None  # control-loop / policy-query rate; None when unbound

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
        if self.policy_hz is not None and self.policy_hz <= 0.0:
            raise ValueError("embodiment policy_hz must be positive")
        identities = {(asset.uri, asset.sha256, asset.role) for asset in self.assets}
        if len(identities) != len(self.assets):
            raise ValueError("embodiment contains duplicate asset references")

    def to_dict(self) -> dict[str, object]:
        """Return the canonical JSON-compatible wire representation."""
        return {
            "schema_version": self.schema_version,
            "embodiment_id": str(self.embodiment_id),
            "name": self.name,
            "dof": self.dof,
            "link_count": self.link_count,
            "policy_hz": self.policy_hz,
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


@dataclass(frozen=True, slots=True)
class Embodiment:
    """Validated robot-body invariants shared by real and simulated runtimes."""

    embodiment_id: EmbodimentId
    dof: int
    joint_lower: tuple[float, ...]
    joint_upper: tuple[float, ...]
    home_joints: tuple[float, ...]
    gripper_travel_m: tuple[float, float]
    policy_hz: float
    mobile_base: bool
    urdf_path: Path | None = None

    def __post_init__(self) -> None:
        if not self.embodiment_id.strip():
            raise ValueError("embodiment_id must not be empty")
        if self.dof <= 0:
            raise ValueError(f"{self.embodiment_id}: dof must be positive")
        for name, values in (
            ("joint_lower", self.joint_lower),
            ("joint_upper", self.joint_upper),
            ("home_joints", self.home_joints),
        ):
            if len(values) != self.dof:
                raise ValueError(
                    f"{self.embodiment_id}: {name} must have length {self.dof}, got {len(values)}"
                )
        if any(
            lower >= upper for lower, upper in zip(self.joint_lower, self.joint_upper, strict=True)
        ):
            raise ValueError(f"{self.embodiment_id}: joint_lower must be < joint_upper")
        if any(
            home < lower or home > upper
            for home, lower, upper in zip(
                self.home_joints, self.joint_lower, self.joint_upper, strict=True
            )
        ):
            raise ValueError(f"{self.embodiment_id}: home_joints must lie within the limits")
        lo, hi = self.gripper_travel_m
        if not 0.0 <= lo < hi:
            raise ValueError(f"{self.embodiment_id}: gripper travel must satisfy 0 <= lo < hi")
        if self.policy_hz <= 0.0:
            raise ValueError(f"{self.embodiment_id}: policy_hz must be positive")
        if self.urdf_path is not None and not self.urdf_path.is_file():
            raise FileNotFoundError(f"{self.embodiment_id}: URDF not found at {self.urdf_path}")

    @property
    def gripper_max_width_m(self) -> float:
        return self.gripper_travel_m[1]

    def with_urdf(self, path: Path) -> Self:
        """Attach a validated local URDF at an explicit runtime wiring site."""
        if not path.is_file():
            raise FileNotFoundError(f"{self.embodiment_id}: URDF not found at {path}")
        return dataclasses.replace(self, urdf_path=path)
