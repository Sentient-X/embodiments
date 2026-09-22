"""Package-local payload machinery over the shared asset vocabulary."""

import hashlib
import os
import tempfile
import urllib.request
from dataclasses import dataclass
from enum import StrEnum
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


class AssetAudience(StrEnum):
    """Who may receive an asset's bytes.

    Derived from the asset's own ``license_id``, never from a hand-kept path list: a
    list would have to be edited in a second place every time a robot is added, and the
    edit that is forgotten is the one that leaks. SPDX reserves the ``LicenseRef-``
    prefix for licences that are not public SPDX identifiers, which is exactly the
    "governed by a private agreement" case — so the prefix *is* the entitlement fact.
    """

    PUBLIC = "public"
    ENTITLED = "entitled"


def audience(license_id: str) -> AssetAudience:
    """Classify one declared licence into the audience allowed to receive its bytes."""
    if not license_id.strip():
        raise AssetIntegrityError("asset licence id must not be empty to classify audience")
    return AssetAudience.ENTITLED if "LicenseRef-" in license_id else AssetAudience.PUBLIC


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
    if (resolved is None or not resolved.is_file()) and relpath.startswith("generated/"):
        resolved = _cache_root() / relpath
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
        """Resolve and verify the declared bytes from the local tree or mirror cache."""

        return resolve_asset(self.ref())

    def ref(self) -> AssetRef:
        """Project to a portable :class:`AssetRef` at an explicit wiring site.

        The URI names the package-relative asset, not its checkout path. This projection
        uses only the authored content identity; consumers use :meth:`path` when they
        need verified bytes. The stable URI keeps embodiment identity byte-equal across
        machines and deployment layouts, including runtimes that carry no geometry.
        """
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

    Declaring the size is not weaker than measuring it. `ref()` is the pure projection
    of that declaration; `path()` resolves and verifies the declared digest and size,
    and the per-asset suite pins both against the bytes on disk. A wrong number here is
    therefore a failing test rather than a fact nobody checks.
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


def generated_description(urdf: bytes, sources: tuple[ProvenancedAsset, ...]) -> ProvenancedAsset:
    """Retain a composed URDF under its byte identity, with all source licences."""
    digest = Sha256Digest(hashlib.sha256(urdf).hexdigest())
    relpath = f"generated/{digest}/body.urdf"
    target = _cache_root() / relpath
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as temporary:
        temporary.write(urdf)
        temporary_path = Path(temporary.name)
    temporary_path.replace(target)
    return ProvenancedAsset(
        asset=AssetRef(
            location=_PACKAGE_URI_PREFIX + relpath,
            content=ContentBlob(digest, len(urdf)),
            format=AssetFormat.URDF,
            role=AssetRole.DESCRIPTION,
            media_type="application/xml",
            logical_path=PurePosixPath(relpath),
        ),
        provenance=AssetProvenance(
            repository="https://github.com/Sentient-X/embodiments",
            revision=str(digest),
            path=relpath,
            license_id=" AND ".join(sorted({asset.provenance.license_id for asset in sources})),
            generator="sx_embodiments.compose_embodiments/v1",
        ),
    )


def description_asset_uri(description: str, reference: str) -> str:
    """Resolve a description dependency without dropping its source package context.

    Relative paths belong to the description directory. Foreign ROS packages may
    be beside that description or at the asset root; two matches are ambiguous.
    Uninstalled foreign packages retain their URI for the runtime to reject.
    """
    if not description.startswith(_PACKAGE_URI_PREFIX):
        raise AssetsUnavailableError("description dependencies need a packaged description")
    source = PurePosixPath(description.removeprefix(_PACKAGE_URI_PREFIX)).parent
    if reference.startswith(_PACKAGE_URI_PREFIX):
        return reference
    if reference.startswith("package://"):
        package_path = PurePosixPath(reference.removeprefix("package://"))
        if package_path.is_absolute() or ".." in package_path.parts or len(package_path.parts) < 2:
            raise AssetsUnavailableError("invalid package dependency path")
        root = asset_root().resolve()
        candidates = {(source / package_path), package_path}
        present = [path for path in candidates if (root / path).is_file()]
        if len(present) > 1:
            raise AssetsUnavailableError(f"ambiguous package dependency: {reference}")
        return _PACKAGE_URI_PREFIX + str(present[0]) if present else reference
    if "://" in reference:
        return reference
    relative = PurePosixPath(reference)
    if relative.is_absolute():
        raise AssetsUnavailableError("description dependencies must not use absolute paths")
    pieces: list[str] = []
    for piece in (source / relative).parts:
        if piece == "..":
            if not pieces:
                raise AssetsUnavailableError("description dependency leaves its asset root")
            pieces.pop()
        elif piece not in (".", ""):
            pieces.append(piece)
    return _PACKAGE_URI_PREFIX + "/".join(pieces)
