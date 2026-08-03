"""Payload machinery over the shared asset vocabulary (sx_contracts.assets).

The vocabulary itself — ``AssetRef``/``AssetFormat``/``AssetRole``/``AssetProvenance``/
``validate_logical_path`` — moved to sx-contracts 2026-08-03; what lives here is what is
genuinely this package's: the resolver over its own shipped ``assets/`` tree
(``asset_root``/``resolve_asset``), the repo-payload record (:class:`PackagedAsset`),
and the provenance-mandatory refinement the ``Embodiment`` record carries
(:class:`EmbodiedAsset`).
"""

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
    validate_logical_path,
)

from .errors import (
    AssetDigestMismatchError,
    AssetsUnavailableError,
    EmbodimentSchemaError,
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
    return resolved


@dataclass(frozen=True, slots=True)
class EmbodiedAsset:
    """A governed embodiment bundle member: an asset that always names its origin.

    Identical in content facts to :class:`AssetRef`, but ``provenance`` is mandatory: every
    embodiment asset must state where its bytes came from. That invariant lives in the type,
    not in a runtime guard, so readers never re-narrow an optional. (Scene assets that
    legitimately lack provenance stay plain :class:`AssetRef`.) Adopt a verified reference
    with :meth:`from_ref`; project back with :attr:`ref` for asset-generic consumers.
    """

    uri: str
    sha256: str
    format: AssetFormat
    role: AssetRole
    provenance: AssetProvenance
    media_type: str | None = None
    byte_size: int | None = None
    logical_path: PurePosixPath | None = None

    def __post_init__(self) -> None:
        # Adopt AssetRef's fail-closed content invariants (URI, digest, size, logical path);
        # provenance is non-optional here by type, so a malformed field fails closed exactly
        # as AssetRef does.
        _ = self.ref

    @classmethod
    def from_ref(cls, ref: AssetRef) -> "EmbodiedAsset":
        """Adopt a verified reference, enforcing the embodiment provenance invariant."""
        if ref.provenance is None:
            raise EmbodimentSchemaError("every embodiment asset must carry provenance")
        return cls(
            uri=ref.uri,
            sha256=ref.sha256,
            format=ref.format,
            role=ref.role,
            provenance=ref.provenance,
            media_type=ref.media_type,
            byte_size=ref.byte_size,
            logical_path=ref.logical_path,
        )

    @property
    def ref(self) -> AssetRef:
        """Project to a plain :class:`AssetRef` for asset-generic consumers."""
        return AssetRef(
            uri=self.uri,
            sha256=self.sha256,
            format=self.format,
            role=self.role,
            media_type=self.media_type,
            byte_size=self.byte_size,
            logical_path=self.logical_path,
            provenance=self.provenance,
        )

    def local_path(self) -> Path:
        """Resolve and verify a packaged asset when its bytes are installed locally."""
        return resolve_asset(self.ref)

    def read_bytes(self) -> bytes:
        """Read verified local content for a packaged asset."""
        return self.local_path().read_bytes()


@dataclass(frozen=True, slots=True)
class PackagedAsset:
    """A description file shipped under this repo's ``assets/`` tree, content-pinned."""

    relpath: str  # assets-root-relative, forward slashes ("so101/so101.urdf")
    sha256: str
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
        :meth:`path` when they need local bytes; the stable URI keeps embodiment content
        identity byte-equal across machines and deployment layouts.
        """
        resolved = self.path()
        actual = hashlib.sha256(resolved.read_bytes()).hexdigest()
        if actual != self.sha256:
            raise AssetDigestMismatchError(self.relpath, self.sha256, actual)
        return AssetRef(
            uri=f"package://sx-embodiments/{self.relpath}",
            sha256=self.sha256,
            format=self.format,
            role=self.role,
            media_type=self.media_type,
            byte_size=resolved.stat().st_size,
            logical_path=PurePosixPath(self.relpath),
            provenance=self.provenance,
        )
