"""Package-local payload machinery over the shared asset vocabulary."""

import hashlib
import os
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

from sx_contracts.assets import (
    AssetFormat,
    AssetIntegrityError,
    AssetProvenance,
    AssetRef,
    AssetRole,
    ProvenancedAsset,
    validate_logical_path,
)
from sx_contracts.content import ContentBlob, Sha256Digest

from .errors import (
    AssetDigestMismatchError,
    AssetsUnavailableError,
)

_ASSETS_ENV = "SX_EMBODIMENTS_ASSETS"


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


_PACKAGE_URI_PREFIX = "package://sx-embodiments/"


def resolve_asset(ref: AssetRef) -> Path:
    """Resolve a ``package://sx-embodiments/...`` reference to its verified on-disk file.

    The inverse of :meth:`PackagedAsset.ref` for consumers that hold only the portable
    asset fact from an embodiment. Fail-closed on every step: a foreign
    URI scheme, a missing file, or bytes whose digest disagrees with the reference.
    """
    if not ref.uri.startswith(_PACKAGE_URI_PREFIX):
        raise AssetsUnavailableError(f"asset uri is not a packaged sx-embodiments asset: {ref.uri}")
    relpath = ref.uri.removeprefix(_PACKAGE_URI_PREFIX)
    validate_logical_path(PurePosixPath(relpath))
    resolved = asset_root() / relpath
    if not resolved.is_file():
        raise AssetsUnavailableError(f"packaged asset missing on disk: {relpath}")
    actual = hashlib.sha256(resolved.read_bytes()).hexdigest()
    if actual != ref.sha256:
        raise AssetDigestMismatchError(relpath, ref.sha256, actual)
    actual_size = resolved.stat().st_size
    if actual_size != ref.byte_size:
        raise AssetIntegrityError(f"{relpath}: expected {ref.byte_size} bytes, got {actual_size}")
    return resolved


@dataclass(frozen=True, slots=True)
class PackagedAsset:
    """A description file shipped under this repo's ``assets/`` tree, content-pinned."""

    relpath: str  # assets-root-relative, forward slashes ("so101/so101.urdf")
    content: ContentBlob
    format: AssetFormat
    role: AssetRole
    provenance: AssetProvenance
    media_type: str | None = None

    def __post_init__(self) -> None:
        if not self.relpath:
            raise AssetIntegrityError("packaged asset relpath must not be empty")
        # The same fail-closed law as bundle logical paths: relative, forward-slash,
        # no '.'/'..' segments anywhere (not just the first character).
        validate_logical_path(PurePosixPath(self.relpath))

    @property
    def sha256(self) -> str:
        return str(self.content.sha256)

    def path(self) -> Path:
        """Resolve the on-disk file via :func:`asset_root`, checking existence."""
        resolved = asset_root() / self.relpath
        if not resolved.is_file():
            raise AssetsUnavailableError(f"packaged asset missing on disk: {self.relpath}")
        return resolved

    def ref(self) -> AssetRef:
        """Project to a portable :class:`AssetRef` at an explicit wiring site.

        The URI names the package-relative asset, not its checkout path. Consumers use
        :meth:`path` when they need local bytes; the stable URI keeps embodiment content
        identity byte-equal across machines and deployment layouts.
        """
        resolved = self.path()
        actual = hashlib.sha256(resolved.read_bytes()).hexdigest()
        if actual != self.sha256:
            raise AssetDigestMismatchError(self.relpath, self.sha256, actual)
        actual_size = resolved.stat().st_size
        if actual_size != self.content.size_bytes:
            raise AssetIntegrityError(
                f"{self.relpath}: expected {self.content.size_bytes} bytes, got {actual_size}"
            )
        return AssetRef(
            location=f"package://sx-embodiments/{self.relpath}",
            content=self.content,
            format=self.format,
            role=self.role,
            media_type=self.media_type,
            logical_path=PurePosixPath(self.relpath),
        )

    def provenanced_asset(self) -> ProvenancedAsset:
        return ProvenancedAsset(self.ref(), self.provenance)


def packaged_asset(
    *,
    relpath: str,
    sha256: str,
    format: AssetFormat,
    role: AssetRole,
    provenance: AssetProvenance,
    media_type: str | None = None,
) -> PackagedAsset:
    """Author a packaged source from its expected digest and measured source size."""

    validate_logical_path(PurePosixPath(relpath))
    path = asset_root() / relpath
    if not path.is_file():
        raise AssetsUnavailableError(f"packaged asset missing on disk: {relpath}")
    return PackagedAsset(
        relpath=relpath,
        content=ContentBlob(Sha256Digest(sha256), path.stat().st_size),
        format=format,
        role=role,
        provenance=provenance,
        media_type=media_type,
    )
