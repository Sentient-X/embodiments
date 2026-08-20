"""Package-local payload machinery over the shared asset vocabulary."""

import hashlib
import os
import urllib.request
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from urllib.parse import urlparse

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
_MIRROR_ENV = "SX_EMBODIMENTS_ASSET_MIRROR"
_CACHE_ENV = "SX_EMBODIMENTS_ASSET_CACHE"
_MIRROR_SCHEMES = frozenset({"https", "file"})
_FETCH_TIMEOUT_S = 60.0


def _local_root() -> Path | None:
    """The local ``assets/`` tree when one exists; ``None`` is a lawful cache miss."""
    override = os.environ.get(_ASSETS_ENV)
    if override:
        root = Path(override)
        if not root.is_dir():
            raise AssetsUnavailableError(f"{_ASSETS_ENV}={override!r} is not a directory")
        return root
    installed = Path(__file__).resolve().parent / "_assets"
    if installed.is_dir():
        return installed
    repo_relative = Path(__file__).resolve().parents[1] / "assets"
    if repo_relative.is_dir():
        return repo_relative
    return None


def asset_root() -> Path:
    """Locate the canonical ``assets/`` tree, or fail closed."""
    root = _local_root()
    if root is None:
        raise AssetsUnavailableError(
            f"description assets not found: set {_ASSETS_ENV} or reinstall sx-embodiments"
        )
    return root


def _cache_root() -> Path:
    override = os.environ.get(_CACHE_ENV)
    if override:
        return Path(override)
    xdg = os.environ.get("XDG_CACHE_HOME")
    base = Path(xdg) if xdg else Path.home() / ".cache"
    return base / "sx-embodiments" / "assets"


def _fetched(relpath: str, sha256: str, size_bytes: int) -> Path:
    """The digest-verified mirror fetch/cache in front of the local fail-closed miss.

    The mirror is explicit pre-execution configuration (``SX_EMBODIMENTS_ASSET_MIRROR``,
    the governed read-only rendering of the asset tree), never a guessed source: absent
    it, a local miss stays :class:`AssetsUnavailableError`. Bytes are verified against
    the declared content identity before they are cached or served — a corrupt or
    tampered mirror object is a typed refusal, and nothing unverified enters the cache.
    """

    mirror = os.environ.get(_MIRROR_ENV)
    if not mirror:
        raise AssetsUnavailableError(
            f"packaged asset missing on disk: {relpath} "
            f"(set {_ASSETS_ENV} to a local tree or {_MIRROR_ENV} to the governed mirror)"
        )
    scheme = urlparse(mirror).scheme
    if scheme not in _MIRROR_SCHEMES:
        raise AssetsUnavailableError(
            f"{_MIRROR_ENV} must use one of {sorted(_MIRROR_SCHEMES)}, got {scheme!r}"
        )
    cached = _cache_root() / relpath
    if cached.is_file():
        try:
            data = cached.read_bytes()
        except OSError:
            data = b""
        if hashlib.sha256(data).hexdigest() == sha256 and len(data) == size_bytes:
            return cached
        # An unreadable or corrupt cache entry is a cache miss; concurrent fetchers
        # may race to clear it, so a vanished entry is not an error.
        cached.unlink(missing_ok=True)
    url = mirror.rstrip("/") + "/" + relpath
    try:
        with urllib.request.urlopen(url, timeout=_FETCH_TIMEOUT_S) as response:
            data = response.read()
    except OSError as error:
        raise AssetsUnavailableError(f"asset mirror fetch failed for {relpath}: {error}") from error
    actual = hashlib.sha256(data).hexdigest()
    if actual != sha256:
        raise AssetDigestMismatchError(relpath, sha256, actual)
    if len(data) != size_bytes:
        raise AssetIntegrityError(f"{relpath}: expected {size_bytes} bytes, got {len(data)}")
    cached.parent.mkdir(parents=True, exist_ok=True)
    # Per-process partial name + atomic replace: concurrent fetchers never interleave
    # writes into one temp file, and a reader only ever sees a complete published file.
    partial = cached.with_name(f"{cached.name}.partial.{os.getpid()}")
    partial.write_bytes(data)
    os.replace(partial, cached)
    return cached


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
    root = _local_root()
    resolved = root / relpath if root is not None else None
    if resolved is None or not resolved.is_file():
        resolved = _fetched(relpath, ref.sha256, ref.byte_size)
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
        """Resolve the on-disk file: the local tree, else the digest-verified mirror cache."""
        root = _local_root()
        if root is not None:
            resolved = root / self.relpath
            if resolved.is_file():
                return resolved
        return _fetched(self.relpath, self.sha256, self.content.size_bytes)

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
    size_bytes: int,
    format: AssetFormat,
    role: AssetRole,
    provenance: AssetProvenance,
    media_type: str | None = None,
) -> PackagedAsset:
    """Author a packaged source from its declared content identity.

    Both halves of the identity are authored, and neither is read off the disk. The
    asymmetry this replaces — digest declared, size measured — made every module-scope
    declaration a filesystem probe, so `import sx_embodiments` required all 632 asset
    files to be present and raised `AssetsUnavailableError` from the import machinery
    when one was not. That cost was paid by every consumer, including the ones that
    never read a mesh: the GPU step runtime is an ASR/media node whose build context is
    capped at 32 MiB, so it cannot carry the 492 MB tree and could not import the
    registry at all.

    Declaring the size is not weaker than measuring it. `path()` and `ref()` still
    resolve through `asset_root()` and still fail closed, and the per-asset suite pins
    every declared digest *and* size against the bytes on disk, so a wrong number here
    is a failing test rather than a fact nobody checks.
    """

    validate_logical_path(PurePosixPath(relpath))
    return PackagedAsset(
        relpath=relpath,
        content=ContentBlob(Sha256Digest(sha256), size_bytes),
        format=format,
        role=role,
        provenance=provenance,
        media_type=media_type,
    )
