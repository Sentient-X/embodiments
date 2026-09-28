"""``materialize(embodiment)``: a directory holding exactly that embodiment's asset closure.

```python
root = materialize(embodiments["so101"])
model = mujoco.MjModel.from_xml_path(str(root / "so101/so101.urdf"))
```

The closure is every declared asset plus every file its descriptions name (URDF meshes
and textures; MJCF meshes, textures, height fields, skins and includes), resolved to
relpaths the manifest knows. Each file is taken from the digest-keyed cache, filled once
from the local tree or the store and verified before it is cached, and hardlinked into
``closures/<closure digest>/<relpath>``. Relpaths are the tree's own, so every
``package://sx-embodiments/<relpath>`` name and every relative reference inside a
description reads unchanged.
"""

import os
import shutil
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath

from sx_contracts.assets import AssetFormat, AssetRef, validate_logical_path
from sx_contracts.content import ContentBlob
from sx_contracts.identity import content_digest

from .assets import (
    PACKAGE_URI_PREFIX,
    asset_manifest,
    cache_root,
    cached_blob,
    description_asset_uri,
    intact,
    link_view,
    local_asset_root,
    refuse_retired_configuration,
)
from .embodiment import Embodiment
from .errors import AssetsUnavailableError

# A build or discard directory older than this was left by an interrupted process.
_ABANDONED_AFTER_S = 3600.0
_URDF_REFERENCES = (("mesh", "filename"), ("texture", "filename"))
_MJCF_MESH_TAGS = frozenset({"mesh", "skin"})
_MJCF_TEXTURE_TAGS = frozenset({"texture", "hfield"})
_MJCF_FILE_ATTRIBUTES = (
    "file",
    "fileright",
    "fileleft",
    "fileup",
    "filedown",
    "filefront",
    "fileback",
)


def _normalized(path: PurePosixPath) -> str:
    """A relpath with ``.`` and ``..`` folded, refusing one that leaves the tree."""
    pieces: list[str] = []
    for piece in path.parts:
        if piece == "..":
            if not pieces:
                raise AssetsUnavailableError(f"description reference leaves the asset tree: {path}")
            pieces.pop()
        elif piece not in (".", ""):
            pieces.append(piece)
    relpath = "/".join(pieces)
    validate_logical_path(PurePosixPath(relpath))
    return relpath


def _urdf_references(relpath: str, data: bytes) -> list[str]:
    uri = PACKAGE_URI_PREFIX + relpath
    references: list[str] = []
    for element in ET.fromstring(data).iter():
        for tag, attribute in _URDF_REFERENCES:
            value = element.get(attribute) if element.tag == tag else None
            if value is None:
                continue
            resolved = description_asset_uri(uri, value)
            if not resolved.startswith(PACKAGE_URI_PREFIX):
                raise AssetsUnavailableError(f"{relpath}: unresolved reference {value!r}")
            references.append(resolved.removeprefix(PACKAGE_URI_PREFIX))
    return references


def _mjcf_references(relpath: str, data: bytes) -> list[str]:
    root = ET.fromstring(data)
    model_dir = PurePosixPath(relpath).parent
    compiler = root.find("compiler")
    assetdir = compiler.get("assetdir", "") if compiler is not None else ""
    meshdir = compiler.get("meshdir", assetdir) if compiler is not None else ""
    texturedir = compiler.get("texturedir", assetdir) if compiler is not None else ""
    references: list[str] = []
    for element in root.iter():
        if element.tag == "include":
            base = model_dir
        elif element.tag in _MJCF_MESH_TAGS:
            base = model_dir / meshdir
        elif element.tag in _MJCF_TEXTURE_TAGS:
            base = model_dir / texturedir
        else:
            continue
        for attribute in _MJCF_FILE_ATTRIBUTES:
            value = element.get(attribute)
            if value is not None:
                references.append(_normalized(base / value))
    return references


def closure(embodiment: Embodiment) -> dict[str, ContentBlob]:
    """Every file ``embodiment`` needs, by relpath, with the bytes it must have.

    A declared asset keeps its declared identity (a recorded body may name an earlier
    revision than the tree's); every file a description names takes the manifest's.
    """
    refuse_retired_configuration()
    manifest = asset_manifest()
    local = local_asset_root()
    files: dict[str, ContentBlob] = {}
    pending: list[tuple[str, AssetFormat | None]] = []
    for provenanced in embodiment.assets:
        ref = provenanced.asset
        if not ref.uri.startswith(PACKAGE_URI_PREFIX):
            raise AssetsUnavailableError(
                f"asset uri is not a packaged sx-embodiments asset: {ref.uri}"
            )
        relpath = ref.uri.removeprefix(PACKAGE_URI_PREFIX)
        files[relpath] = ref.content
        pending.append((relpath, ref.format))
    missing: list[str] = []
    seen: set[str] = set()
    while pending:
        relpath, format = pending.pop()
        if relpath in seen:
            continue
        seen.add(relpath)
        is_mjcf = format is AssetFormat.MJCF or (format is None and relpath.endswith(".xml"))
        if format is not AssetFormat.URDF and not is_mjcf:
            continue
        data = _readable(relpath, files[relpath], local).read_bytes()
        references = (
            _urdf_references(relpath, data) if not is_mjcf else _mjcf_references(relpath, data)
        )
        for reference in references:
            if reference not in files:
                row = manifest.get(reference)
                if row is None:
                    missing.append(f"{relpath} -> {reference}")
                    continue
                files[reference] = row.content
            # An MJCF include is itself a model whose references join the closure.
            pending.append((reference, None if is_mjcf else AssetFormat.MESH))
    if missing:
        raise AssetsUnavailableError(
            f"{embodiment.name}: references outside the asset manifest: {', '.join(missing)}"
        )
    return files


def _source(relpath: str, content: ContentBlob, local: Path | None) -> Path:
    """The verified blob for one closure file, filled from the tree, this host or the store."""
    if relpath.startswith("generated/"):
        # A composed description is retained in this host's cache, not in the tree.
        return cached_blob(content, relpath=relpath, local=cache_root() / relpath)
    return cached_blob(content, relpath=relpath, local=local / relpath if local else None)


def _readable(relpath: str, content: ContentBlob, local: Path | None) -> Path:
    """A description's verified bytes; the tree serves them in place when it holds them."""
    if local is not None and intact(local / relpath, content):
        return local / relpath
    return _source(relpath, content, local)


def _complete(directory: Path, files: dict[str, ContentBlob]) -> bool:
    """``directory`` holds exactly ``files``, each a link of its blob or an intact copy."""
    try:
        present = {
            path.relative_to(directory).as_posix()
            for path in directory.rglob("*")
            if not path.is_dir()
        }
    except OSError:
        return False
    if present != files.keys():
        return False
    for relpath, content in files.items():
        blob = cache_root() / "sha256" / content.sha256[:2] / content.sha256
        target = directory / relpath
        try:
            linked = blob.is_file() and os.path.samefile(blob, target)
        except OSError:
            return False
        if not (linked or intact(target, content)):
            return False
    return True


def _discard(directory: Path) -> None:
    """Remove a directory no reader can be holding: renamed aside first, then deleted."""
    aside = directory.with_name(f".discard.{directory.name}.{uuid.uuid4().hex}")
    try:
        directory.rename(aside)
    except FileNotFoundError:
        return
    shutil.rmtree(aside, ignore_errors=True)


def _sweep(closures: Path) -> None:
    """Delete build and discard directories an interrupted materialize left behind."""
    horizon = time.time() - _ABANDONED_AFTER_S
    for entry in closures.glob(".*"):
        try:
            abandoned = entry.is_dir() and entry.stat().st_mtime < horizon
        except OSError:
            continue
        if abandoned:
            shutil.rmtree(entry, ignore_errors=True)


def materialize(embodiment: Embodiment) -> Path:
    """A directory holding exactly ``embodiment``'s closure, hardlinked from the cache.

    The directory is named by the closure's content digest, built beside its final name
    and renamed into place, so a reader only ever sees a complete closure. Every blob is
    verified as it is linked. A complete closure is never deleted; one left incomplete
    or altered is renamed aside before it is removed, so a concurrent reader holding a
    complete one is never disturbed.
    """
    files = closure(embodiment)
    local = local_asset_root()
    key = content_digest(
        [
            [relpath, str(content.sha256), content.size_bytes]
            for relpath, content in sorted(files.items())
        ]
    )
    closures = cache_root() / "closures"
    target = closures / key
    blobs = {relpath: _source(relpath, content, local) for relpath, content in files.items()}
    if target.is_dir() and _complete(target, files):
        return target
    closures.mkdir(parents=True, exist_ok=True)
    _sweep(closures)
    partial = Path(tempfile.mkdtemp(prefix=f".{key}.", dir=closures))
    try:
        for relpath, blob in blobs.items():
            link_view(blob, partial / relpath)
        for _ in range(3):
            if target.is_dir() and _complete(target, files):
                return target
            if target.exists():
                _discard(target)
            try:
                partial.rename(target)
                return target
            except OSError:
                # Another process published or replaced it first; check it again.
                continue
        if _complete(target, files):
            return target
        raise AssetsUnavailableError(f"{embodiment.name}: could not publish closure {key}")
    finally:
        if partial.exists():
            shutil.rmtree(partial, ignore_errors=True)


def materialized(embodiment: Embodiment, ref: AssetRef | None = None) -> Path:
    """Where ``ref`` sits in ``embodiment``'s materialized closure, beside every file it names.

    ``ref`` defaults to the authoritative description. A consumer that follows a
    description's own references (a URDF loader, MuJoCo, a mesh preview) reads it here
    rather than from :func:`resolve_asset`, whose single verified file has no siblings on
    a host without the tree.
    """
    target = embodiment.urdf.asset if ref is None else ref
    if not target.uri.startswith(PACKAGE_URI_PREFIX):
        raise AssetsUnavailableError(
            f"asset uri is not a packaged sx-embodiments asset: {target.uri}"
        )
    relpath = target.uri.removeprefix(PACKAGE_URI_PREFIX)
    root = materialize(embodiment)
    path = root / relpath
    if not path.is_file():
        raise AssetsUnavailableError(f"{embodiment.name}: {relpath} is not in its closure")
    return path
