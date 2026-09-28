"""The asset manifest: every file of the tree, rendered from the tree and the declarations.

``asset-manifest.json`` names each file by relpath with its sha256, size and ``license_id``.
It is the one place a file's bytes are known without the file, so it is what lets a host
without the tree resolve a description's meshes and fetch them from the store by digest.

A declared asset (a :class:`PackagedAsset`, a preview, a row of the tree's
``dependency-assets.json``) carries its own licence. The thousands of meshes a description
names carry none, so an undeclared file takes the licence declared in its nearest
ancestor directory that declares any, and only when every declaration beneath that
directory names the same licence: a directory whose subtrees disagree cannot license a file
in a subtree none of them covers, so that file is a disagreement until it is declared. A
vendored file no embodiment references is declared in ``known.sources.VENDORED_LICENCES``.
The audience rule stays whole: a file under an entitled directory can only inherit an
entitled licence.

    python tools/render_asset_manifest.py           # rewrite the manifest from the tree
    python tools/render_asset_manifest.py --check   # fail when it disagrees with either
"""

import json
from collections.abc import Iterator, Mapping
from pathlib import Path, PurePosixPath

from sx_contracts import decode
from sx_contracts.assets import AssetIntegrityError
from sx_contracts.content import ContentBlob, Sha256Digest
from sx_contracts.identity import file_digest

from .assets import (
    MANIFEST_TREE_INDEX,
    ManifestEntry,
    asset_manifest,
    audience,
)
from .embodiment import packaged_assets
from .known import asset_audiences, definitions
from .known._previews import PREVIEWS
from .known.sources import VENDORED_LICENCES


class ManifestDisagreementError(AssetIntegrityError):
    """The manifest, the tree and the declarations do not describe the same files."""

    def __init__(self, disagreements: tuple[str, ...]) -> None:
        shown = "\n  ".join(disagreements[:20])
        super().__init__(f"{len(disagreements)} asset manifest disagreements:\n  {shown}")
        self.disagreements = disagreements


def _declarations(root: Path | None) -> Iterator[tuple[str, ContentBlob, str]]:
    for definition in definitions():
        for packaged in packaged_assets(definition):
            yield packaged.relpath, packaged.content, packaged.provenance.license_id
    for preview in PREVIEWS.values():
        yield preview.relpath, preview.content, preview.provenance.license_id
    if root is not None and (root / MANIFEST_TREE_INDEX).is_file():
        rows: object = json.loads((root / MANIFEST_TREE_INDEX).read_text(encoding="utf-8"))
        where = MANIFEST_TREE_INDEX
        for row in decode.documents({where: rows}, where, error=AssetIntegrityError):
            content = ContentBlob(
                Sha256Digest(decode.text(row, "sha256")), decode.integer(row, "byte_size")
            )
            yield decode.text(row, "path"), content, decode.text(row, "license")


def _declared(root: Path | None) -> tuple[dict[str, ContentBlob], dict[str, set[str]], list[str]]:
    contents: dict[str, ContentBlob] = {}
    licences: dict[str, set[str]] = {}
    disagreements: list[str] = []
    for relpath, content, license_id in _declarations(root):
        established = contents.setdefault(relpath, content)
        if established != content:
            disagreements.append(f"{relpath}: declared as both {established} and {content}")
        licences.setdefault(relpath, set()).add(license_id)
    return contents, licences, disagreements


def _joined(licences: set[str]) -> str:
    return " AND ".join(sorted(licences))


def _directory_licences(licences: Mapping[str, set[str]]) -> dict[PurePosixPath, set[str]]:
    """Each directory's declared licences, over every declaration beneath it."""
    by_directory: dict[PurePosixPath, set[str]] = {}
    for declared, values in licences.items():
        for parent in PurePosixPath(declared).parents:
            if parent != PurePosixPath("."):
                by_directory.setdefault(parent, set()).update(values)
    return by_directory


def _inherited(relpath: str, by_directory: Mapping[PurePosixPath, set[str]]) -> str | None:
    """The one licence declared beneath the nearest declaring ancestor, or ``None``.

    A vendored declaration (``VENDORED_LICENCES``, a file or a directory) wins over
    inheritance. An ancestor whose declarations name several licences is refused rather
    than joined: the union of unrelated siblings is nobody's licence.
    """
    path = PurePosixPath(relpath)
    for candidate in (path, *path.parents):
        vendored = VENDORED_LICENCES.get(str(candidate))
        if vendored is not None:
            return vendored
    for parent in path.parents:
        if parent in by_directory:
            values = by_directory[parent]
            return next(iter(values)) if len(values) == 1 else None
    return None


def render_manifest(root: Path) -> tuple[ManifestEntry, ...]:
    """Every file under ``root`` with its measured bytes and its declared or inherited licence.

    Fails with every disagreement at once: a declaration whose bytes differ from the file,
    a declared file absent from the tree, a file no directory's declarations can license,
    and a file whose licence disagrees with its top-level directory's audience.
    """
    contents, licences, disagreements = _declared(root)
    audiences = asset_audiences()
    by_directory = _directory_licences(licences)
    entries: list[ManifestEntry] = []
    on_disk: set[str] = set()
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        relpath = path.relative_to(root).as_posix()
        if relpath == MANIFEST_TREE_INDEX:
            continue
        on_disk.add(relpath)
        measured = ContentBlob(file_digest(path), path.stat().st_size)
        declared = contents.get(relpath)
        if declared is not None and declared != measured:
            disagreements.append(f"{relpath}: declared {declared}, the tree holds {measured}")
        license_id = (
            _joined(licences[relpath]) if relpath in licences else _inherited(relpath, by_directory)
        )
        if license_id is None:
            disagreements.append(
                f"{relpath}: no declaration licenses this file, and its nearest declaring "
                "ancestor holds no single licence"
            )
            continue
        directory = PurePosixPath(relpath).parts[0]
        if audience(license_id) is not audiences.get(directory):
            disagreements.append(
                f"{relpath}: licence {license_id!r} disagrees with {directory}'s audience"
            )
        entries.append(ManifestEntry(relpath, measured, license_id))
    for relpath in sorted(contents.keys() - on_disk):
        disagreements.append(f"{relpath}: declared but absent from the tree")
    if disagreements:
        raise ManifestDisagreementError(tuple(disagreements))
    return tuple(entries)


def manifest_disagreements(root: Path | None) -> tuple[str, ...]:
    """Where the committed manifest disagrees with the declarations, and with ``root``.

    Without a tree (a host that carries none) only the declarations are compared: each
    declared file must be a row with the same bytes and a licence naming its own.
    """
    committed = asset_manifest()
    contents, licences, found = _declared(root)
    disagreements = list(found)
    for relpath, content in sorted(contents.items()):
        row = committed.get(relpath)
        if row is None:
            disagreements.append(f"{relpath}: declared but not in the manifest")
        elif row.content != content:
            disagreements.append(f"{relpath}: declared {content}, the manifest says {row.content}")
        elif row.license_id != _joined(licences[relpath]):
            disagreements.append(
                f"{relpath}: declared {_joined(licences[relpath])!r}, "
                f"the manifest says {row.license_id!r}"
            )
    if root is not None:
        try:
            rendered = {entry.relpath: entry for entry in render_manifest(root)}
        except ManifestDisagreementError as error:
            return (*disagreements, *error.disagreements)
        for relpath in sorted(rendered.keys() | committed.keys()):
            if rendered.get(relpath) != committed.get(relpath):
                disagreements.append(
                    f"{relpath}: the tree holds {rendered.get(relpath)}, "
                    f"the manifest says {committed.get(relpath)}"
                )
    return tuple(disagreements)
