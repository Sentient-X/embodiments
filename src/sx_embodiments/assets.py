"""Content-addressed asset references and the canonical description-asset tree.

``AssetRef`` is the portable identity of a robotics asset anywhere (catalog, factory
uploads, R2). ``PackagedAsset`` is the repo-local case: a description file shipped under
this repository's ``assets/`` tree, content-pinned in the registry source and re-hashed by
``tests/test_asset_integrity.py`` (the file is the truth; the pin is the guard).

Assets ship in wheels and sdists so ``package://sx-embodiments/...`` remains resolvable
outside the glued checkout. Resolution is explicit and fail-closed: the
``SX_EMBODIMENTS_ASSETS`` environment variable wins (deployed workers), then the installed
asset tree, then the repo-relative ``assets/`` directory (editable/workspace installs), else
:class:`~sx_embodiments.errors.AssetsUnavailableError`. This module is the package's one
sanctioned environment read (see ``tests/test_purity.py``).
"""

import hashlib
import os
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path, PurePosixPath
from typing import Self
from urllib.parse import urlparse

from .errors import AssetIntegrityError, AssetsUnavailableError

_ASSETS_ENV = "SX_EMBODIMENTS_ASSETS"


def validate_logical_path(path: PurePosixPath) -> None:
    """Reject bundle-member paths that could escape or alias a materialization root."""
    text = str(path)
    if path.is_absolute():
        raise AssetIntegrityError(f"logical_path must be relative: {text!r}")
    if not path.parts:
        raise AssetIntegrityError("logical_path must not be empty")
    if any(part in (".", "..") for part in path.parts):
        raise AssetIntegrityError(f"logical_path must not contain '.' or '..' segments: {text!r}")
    if "\\" in text:
        raise AssetIntegrityError(f"logical_path must use forward slashes: {text!r}")


class AssetFormat(StrEnum):
    """Portable formats understood by the robotics data platform."""

    URDF = "urdf"
    SRDF = "srdf"
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
    LICENSE = "license"
    OTHER = "other"


@dataclass(frozen=True, slots=True)
class AssetRef:
    """Content-addressed robotics asset at a catalog-resolvable URI.

    ``logical_path`` is the member's path inside a multi-file bundle (URDF + SRDF + mesh
    closure), validated fail-closed: relative, forward-slash, no ``..``. Single-file
    producers keep ``None``; bundle ingest requires it. The same content blob may appear at
    multiple logical paths within one bundle.
    """

    uri: str
    sha256: str
    format: AssetFormat
    role: AssetRole
    media_type: str | None = None
    byte_size: int | None = None
    logical_path: PurePosixPath | None = None

    def __post_init__(self) -> None:
        parsed = urlparse(self.uri)
        if not parsed.scheme:
            raise AssetIntegrityError("asset uri must be absolute and include a scheme")
        digest = self.sha256.lower()
        if (
            self.sha256 != digest
            or len(digest) != 64
            or any(char not in "0123456789abcdef" for char in digest)
        ):
            raise AssetIntegrityError("asset sha256 must be 64 lowercase hexadecimal characters")
        if self.byte_size is not None and self.byte_size < 0:
            raise AssetIntegrityError("asset byte_size must be non-negative")
        if self.logical_path is not None:
            validate_logical_path(self.logical_path)

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


def asset_root() -> Path:
    """Locate the canonical ``assets/`` tree, or fail closed."""
    override = os.environ.get(_ASSETS_ENV)
    if override:
        root = Path(override)
        if not root.is_dir():
            raise AssetsUnavailableError(f"{_ASSETS_ENV}={override!r} is not a directory")
        return root
    installed = Path(__file__).resolve().parent / "_assets"
    if installed.is_dir():
        return installed
    repo_relative = Path(__file__).resolve().parents[2] / "assets"
    if repo_relative.is_dir():
        return repo_relative
    raise AssetsUnavailableError(
        f"description assets not found: set {_ASSETS_ENV} or reinstall sx-embodiments"
    )


@dataclass(frozen=True, slots=True)
class PackagedAsset:
    """A description file shipped under this repo's ``assets/`` tree, content-pinned."""

    relpath: str  # assets-root-relative, forward slashes ("so101/so101.urdf")
    sha256: str
    format: AssetFormat
    role: AssetRole
    media_type: str | None = None

    def __post_init__(self) -> None:
        if not self.relpath or self.relpath.startswith(("/", ".")):
            raise AssetIntegrityError(f"packaged asset relpath must be relative: {self.relpath!r}")
        digest = self.sha256.lower()
        if (
            self.sha256 != digest
            or len(digest) != 64
            or any(char not in "0123456789abcdef" for char in digest)
        ):
            raise AssetIntegrityError("asset sha256 must be 64 lowercase hexadecimal characters")

    def path(self) -> Path:
        """Resolve the on-disk file via :func:`asset_root`, checking existence."""
        resolved = asset_root() / self.relpath
        if not resolved.is_file():
            raise AssetsUnavailableError(f"packaged asset missing on disk: {self.relpath}")
        return resolved

    def ref(self) -> AssetRef:
        """Project to a portable :class:`AssetRef` at an explicit wiring site.

        The URI names the package-relative asset, not its checkout path. Consumers use
        :meth:`path` when they need local bytes; the stable URI keeps manifest content
        identity byte-equal across machines and deployment layouts.
        """
        resolved = self.path()
        return AssetRef(
            uri=f"package://sx-embodiments/{self.relpath}",
            sha256=self.sha256,
            format=self.format,
            role=self.role,
            media_type=self.media_type,
            byte_size=resolved.stat().st_size,
        )
