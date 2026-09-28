"""Package-local payload machinery over the shared asset vocabulary."""

import hashlib
import http.client
import json
import os
import tempfile
import urllib.error
import urllib.request
from collections.abc import Mapping
from dataclasses import dataclass
from enum import StrEnum
from functools import cache
from pathlib import Path, PurePosixPath
from urllib.parse import urlparse

from sx_contracts import decode
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
from sx_contracts.identity import file_digest

from .errors import (
    AssetDigestMismatchError,
    AssetsUnavailableError,
)

_ASSETS_ENV = "SX_EMBODIMENTS_ASSETS"
_STORE_ENV = "SX_EMBODIMENTS_ASSET_STORE"
# The bearer credential for the store. It is read at the moment of a fetch, placed only in
# an unredirected ``Authorization`` header, and never enters a URL, a message or a log.
_STORE_TOKEN_ENV = "SX_EMBODIMENTS_ASSET_STORE_TOKEN"
_RETIRED_MIRROR_ENV = "SX_EMBODIMENTS_ASSET_MIRROR"
_CACHE_ENV = "SX_EMBODIMENTS_ASSET_CACHE"
_STORE_SCHEMES = frozenset({"https", "file"})
_REFUSED_STATUSES = frozenset({401, 403, 404})
_FETCH_TIMEOUT_S = 60.0
MANIFEST_PATH = Path(__file__).resolve().parent / "asset-manifest.json"
_MANIFEST_KEYS = {"path", "sha256", "size", "license_id"}
# The tree's own provenance index for restored upstream files; an input to the manifest,
# not an asset any description references.
MANIFEST_TREE_INDEX = "dependency-assets.json"


def local_asset_root() -> Path | None:
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
    root = local_asset_root()
    if root is None:
        raise AssetsUnavailableError(
            f"description assets not found: set {_ASSETS_ENV} or reinstall sx-embodiments"
        )
    return root


def cache_root() -> Path:
    """This host's asset cache: ``sha256/`` blobs and the views hardlinked from them."""
    override = os.environ.get(_CACHE_ENV)
    if override:
        return Path(override)
    xdg = os.environ.get("XDG_CACHE_HOME")
    base = Path(xdg) if xdg else Path.home() / ".cache"
    return base / "sx-embodiments"


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


@dataclass(frozen=True, slots=True)
class ManifestEntry:
    """One file of the asset tree: where it sits, its exact bytes, and who may have them."""

    relpath: str
    content: ContentBlob
    license_id: str

    def __post_init__(self) -> None:
        validate_logical_path(PurePosixPath(self.relpath))
        audience(self.license_id)

    @property
    def audience(self) -> AssetAudience:
        return audience(self.license_id)


def parse_manifest(text: str) -> tuple[ManifestEntry, ...]:
    """Parse the manifest's one rendering, refusing any row that is not exactly a file fact."""
    try:
        rows: object = json.loads(text)
    except json.JSONDecodeError as error:
        raise AssetIntegrityError(f"asset manifest is not JSON: {error}") from error
    entries = tuple(
        ManifestEntry(
            decode.text(row, "path"),
            ContentBlob(Sha256Digest(decode.text(row, "sha256")), decode.integer(row, "size")),
            decode.text(row, "license_id"),
        )
        for row in (
            decode.exactly(item, _MANIFEST_KEYS, error=AssetIntegrityError)
            for item in decode.documents({"manifest": rows}, "manifest", error=AssetIntegrityError)
        )
    )
    relpaths = [entry.relpath for entry in entries]
    if relpaths != sorted(set(relpaths)):
        raise AssetIntegrityError("asset manifest rows must be unique and sorted by path")
    return entries


def render_manifest_json(entries: tuple[ManifestEntry, ...]) -> str:
    """The manifest's one rendering: a sorted array, one file per line."""
    rows = [
        json.dumps(
            {
                "path": entry.relpath,
                "sha256": str(entry.content.sha256),
                "size": entry.content.size_bytes,
                "license_id": entry.license_id,
            },
            ensure_ascii=False,
        )
        for entry in sorted(entries, key=lambda entry: entry.relpath)
    ]
    return "[\n" + ",\n".join(rows) + "\n]\n"


@cache
def asset_manifest() -> Mapping[str, ManifestEntry]:
    """Every file of the asset tree by relpath, read from the committed manifest."""
    try:
        text = MANIFEST_PATH.read_text(encoding="utf-8")
    except OSError as error:
        raise AssetsUnavailableError(f"asset manifest unreadable: {MANIFEST_PATH.name}") from error
    return {entry.relpath: entry for entry in parse_manifest(text)}


def refuse_retired_configuration() -> None:
    """Refuse the relpath-keyed mirror's variable whenever it is set, not only on a miss.

    A host still configured for the mirror would otherwise work from its local tree and
    fail only on the first miss, far from the stale setting that caused it.
    """
    if os.environ.get(_RETIRED_MIRROR_ENV) is not None:
        raise AssetsUnavailableError(
            f"{_RETIRED_MIRROR_ENV} is retired: the store is digest-keyed; unset it and set "
            f"{_STORE_ENV}"
        )


def _store_token() -> str | None:
    """The configured bearer, parsed once: surrounding whitespace dropped, visible ASCII only.

    A credential read from a file or a secret mount often ends in a newline, and
    ``http.client`` rejects such a header value with an exception whose text is the whole
    header. Parsing here means no malformed credential reaches the transport, and the
    refusal names the variable, never the value.
    """
    raw = os.environ.get(_STORE_TOKEN_ENV)
    if raw is None:
        return None
    token = raw.strip()
    if not token or not all("!" <= character <= "~" for character in token):
        raise AssetsUnavailableError(
            f"{_STORE_TOKEN_ENV} must be a non-empty token of visible ASCII characters"
        )
    return token


def _store_request(sha256: Sha256Digest) -> urllib.request.Request:
    """The store read for one digest, carrying the bearer credential when one is configured.

    The credential goes in an *unredirected* header: when the store answers with a
    redirect to a signed object URL, the bearer stays with the store and never reaches the
    host the redirect names.
    """
    refuse_retired_configuration()
    store = os.environ.get(_STORE_ENV)
    if not store:
        raise AssetsUnavailableError(
            f"asset sha256 {sha256} is not on this host "
            f"(set {_ASSETS_ENV} to a local tree or {_STORE_ENV} to the asset store)"
        )
    scheme = urlparse(store).scheme
    if scheme not in _STORE_SCHEMES:
        raise AssetsUnavailableError(
            f"{_STORE_ENV} must use one of {sorted(_STORE_SCHEMES)}, got {scheme!r}"
        )
    request = urllib.request.Request(f"{store.rstrip('/')}/sha256/{sha256[:2]}/{sha256}")
    token = _store_token()
    if token is not None:
        request.add_unredirected_header("Authorization", f"Bearer {token}")
    return request


def _store_fetch(sha256: Sha256Digest, *, first_byte: bool = False) -> bytes:
    """Read one object from the store; every refusal is :class:`AssetsUnavailableError`.

    The raised message names the digest and the status or exception type only. Neither
    the request (which holds the credential) nor the transport exception is chained, so
    no message or traceback carries either. ``first_byte`` asks for one byte in an
    ordinary (redirected) header, so the signed object read honours it too.
    """
    request = _store_request(sha256)
    if first_byte:
        request.add_header("Range", "bytes=0-0")
    status: int | None = None
    try:
        with urllib.request.urlopen(request, timeout=_FETCH_TIMEOUT_S) as response:
            return response.read()
    except urllib.error.HTTPError as error:
        status = error.code
        error.close()
    except (OSError, http.client.HTTPException, ValueError) as error:
        # ValueError covers UnicodeError and http.client's invalid-header refusal, whose
        # text would otherwise carry the header value.
        reason = type(error).__name__
        raise AssetsUnavailableError(
            f"asset store read failed for sha256 {sha256}: {reason}"
        ) from None
    if status in _REFUSED_STATUSES:
        raise AssetsUnavailableError(f"asset store refused sha256 {sha256}: HTTP {status}")
    raise AssetsUnavailableError(f"asset store failed for sha256 {sha256}: HTTP {status}")


def probe_store(content: ContentBlob) -> None:
    """Prove the store serves ``content`` without downloading it, or raise the fetch's refusal.

    One request per digest reading its first byte (an empty object has none to read, so it
    is read whole): what a check over every manifest digest costs, and the same bearer,
    redirect and :class:`AssetsUnavailableError` a real fetch meets.
    """
    _store_fetch(content.sha256, first_byte=content.size_bytes > 0)


def _verified(label: str, content: ContentBlob, data: bytes) -> bytes:
    actual = hashlib.sha256(data).hexdigest()
    if actual != content.sha256:
        raise AssetDigestMismatchError(label, content.sha256, actual)
    if len(data) != content.size_bytes:
        raise AssetIntegrityError(f"{label}: expected {content.size_bytes} bytes, got {len(data)}")
    return data


def intact(path: Path, content: ContentBlob) -> bool:
    """Whether ``path`` holds exactly ``content``; an unreadable file does not."""
    try:
        return path.stat().st_size == content.size_bytes and file_digest(path) == content.sha256
    except OSError:
        return False


def _publish(target: Path, data: bytes) -> None:
    """Write complete bytes under ``target`` atomically, read-only, so no view can edit them."""
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as temporary:
        temporary.write(data)
        partial = Path(temporary.name)
    partial.chmod(0o444)
    os.replace(partial, target)


def cached_blob(content: ContentBlob, *, relpath: str, local: Path | None = None) -> Path:
    """The verified blob for ``content`` in the digest-keyed cache, filling a miss once.

    A miss is filled from ``local`` when those bytes verify, and otherwise from the store.
    Every source is checked against the declared sha256 and size before anything is
    written, and a corrupt entry is a miss: nothing unverified is ever cached or served.
    """
    digest = content.sha256
    blob = cache_root() / "sha256" / digest[:2] / digest
    if intact(blob, content):
        return blob
    blob.unlink(missing_ok=True)
    data: bytes | None = None
    if local is not None and local.is_file():
        candidate = local.read_bytes()
        if hashlib.sha256(candidate).hexdigest() == digest and len(candidate) == content.size_bytes:
            data = candidate
    if data is None:
        data = _verified(relpath, content, _store_fetch(digest))
    _publish(blob, data)
    return blob


def link_view(blob: Path, target: Path) -> None:
    """Place ``blob`` at ``target`` as a hardlink (a copy only where links are refused)."""
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(blob, target)
    except FileExistsError:
        target.unlink()
        os.link(blob, target)
    except OSError:
        _publish(target, blob.read_bytes())


def _fetched(relpath: str, content: ContentBlob) -> Path:
    """One verified file from the store, under its own name so its format still reads.

    ``files/<sha256>/<filename>``: keyed by content like the blob it links, so every
    revision of a path keeps its own view.
    """
    blob = cached_blob(content, relpath=relpath)
    view = cache_root() / "files" / content.sha256 / PurePosixPath(relpath).name
    if not (view.is_file() and os.path.samefile(view, blob)):
        link_view(blob, view)
    return view


PACKAGE_URI_PREFIX = "package://sx-embodiments/"
_SUPERSEDED_DIR = "_by_digest"


def superseded_relpath(relpath: str, sha256: str) -> str:
    """Where a package keeps an earlier published revision of one of its files.

    ``<package>/_by_digest/<sha256>/<filename>``: inside the owning package, so it keeps
    that package's audience, and under its original name, so its format still reads.
    Recordings name their embodiment by content, and a document an earlier schema
    published must keep resolving after the canonical file at the same path changes.
    """
    path = PurePosixPath(relpath)
    return f"{path.parts[0]}/{_SUPERSEDED_DIR}/{sha256}/{path.name}"


def resolve_asset(ref: AssetRef) -> Path:
    """Resolve a ``package://sx-embodiments/...`` reference to its verified on-disk file.

    The inverse of :meth:`PackagedAsset.ref` for consumers that hold only the portable
    asset fact from an embodiment. The local tree serves the file when its bytes match,
    then a kept revision beside it, then the store by digest; a foreign URI scheme, a
    missing file, or bytes whose digest disagrees with the reference all fail closed.
    """
    if not ref.uri.startswith(PACKAGE_URI_PREFIX):
        raise AssetsUnavailableError(f"asset uri is not a packaged sx-embodiments asset: {ref.uri}")
    refuse_retired_configuration()
    relpath = ref.uri.removeprefix(PACKAGE_URI_PREFIX)
    validate_logical_path(PurePosixPath(relpath))
    root = local_asset_root()
    candidates: list[Path] = []
    if root is not None:
        candidates += [root / relpath, root / superseded_relpath(relpath, ref.sha256)]
    if relpath.startswith("generated/"):
        candidates.append(cache_root() / relpath)
    present = [candidate for candidate in candidates if candidate.is_file()]
    for candidate in present:
        if intact(candidate, ref.content):
            return candidate
    if present and not os.environ.get(_STORE_ENV):
        # The package has since changed this file and keeps no revision of it here; the
        # refusal says which bytes were found, not merely that the right ones were not.
        found = present[0]
        actual = file_digest(found)
        if actual != ref.sha256:
            raise AssetDigestMismatchError(relpath, ref.sha256, actual)
        raise AssetIntegrityError(
            f"{relpath}: expected {ref.byte_size} bytes, got {found.stat().st_size}"
        )
    try:
        return _fetched(relpath, ref.content)
    except AssetsUnavailableError as error:
        if present:
            raise
        raise AssetsUnavailableError(
            f"packaged asset missing on disk: {relpath}; {error}"
        ) from None


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
        """Resolve and verify the declared bytes from the local tree or the asset store."""

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
    target = cache_root() / relpath
    target.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as temporary:
        temporary.write(urdf)
        temporary_path = Path(temporary.name)
    temporary_path.replace(target)
    return ProvenancedAsset(
        asset=AssetRef(
            location=PACKAGE_URI_PREFIX + relpath,
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
    Uninstalled foreign packages retain their URI for the runtime to reject. Presence is
    read from the asset manifest, never probed on disk, so it answers the same with or
    without a local tree.
    """
    if not description.startswith(PACKAGE_URI_PREFIX):
        raise AssetsUnavailableError("description dependencies need a packaged description")
    source = PurePosixPath(description.removeprefix(PACKAGE_URI_PREFIX)).parent
    if reference.startswith(PACKAGE_URI_PREFIX):
        return reference
    if reference.startswith("package://"):
        package_path = PurePosixPath(reference.removeprefix("package://"))
        if package_path.is_absolute() or ".." in package_path.parts or len(package_path.parts) < 2:
            raise AssetsUnavailableError("invalid package dependency path")
        manifest = asset_manifest()
        candidates = {(source / package_path), package_path}
        present = [path for path in candidates if str(path) in manifest]
        if len(present) > 1:
            raise AssetsUnavailableError(f"ambiguous package dependency: {reference}")
        return PACKAGE_URI_PREFIX + str(present[0]) if present else reference
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
    return PACKAGE_URI_PREFIX + "/".join(pieces)
