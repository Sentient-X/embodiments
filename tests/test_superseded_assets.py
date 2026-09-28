"""A file an earlier published document names keeps resolving after the canonical one changes."""

from pathlib import Path

import pytest

from sx_embodiments import AssetDigestMismatchError, embodiments
from sx_embodiments.assets import resolve_asset, superseded_relpath

_URDF_RELPATH = "so101/so101.urdf"


@pytest.fixture
def published() -> bytes:
    return embodiments["so101"].urdf_bytes


def _local_tree(monkeypatch: pytest.MonkeyPatch, tmp_path: Path, files: dict[str, bytes]) -> Path:
    local = tmp_path / "assets"
    for relpath, data in files.items():
        target = local / relpath
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    local.mkdir(exist_ok=True)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSETS", str(local))
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_CACHE", str(tmp_path / "cache"))
    monkeypatch.delenv("SX_EMBODIMENTS_ASSET_STORE", raising=False)
    return local


def test_a_kept_revision_serves_a_reference_the_canonical_file_no_longer_matches(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, published: bytes
) -> None:
    ref = embodiments["so101"].urdf.asset
    kept = superseded_relpath(_URDF_RELPATH, ref.sha256)
    assert kept == f"so101/_by_digest/{ref.sha256}/so101.urdf"
    _local_tree(
        monkeypatch,
        tmp_path,
        {_URDF_RELPATH: published + b"<!-- revised -->", kept: published},
    )
    resolved = resolve_asset(ref)
    assert resolved.read_bytes() == published
    assert resolved.name == "so101.urdf"


def test_a_changed_file_without_a_kept_revision_still_refuses(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, published: bytes
) -> None:
    _local_tree(monkeypatch, tmp_path, {_URDF_RELPATH: published + b"<!-- revised -->"})
    with pytest.raises(AssetDigestMismatchError):
        resolve_asset(embodiments["so101"].urdf.asset)


def test_a_wrong_kept_revision_is_refused(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, published: bytes
) -> None:
    ref = embodiments["so101"].urdf.asset
    _local_tree(
        monkeypatch,
        tmp_path,
        {
            _URDF_RELPATH: published + b"<!-- revised -->",
            superseded_relpath(_URDF_RELPATH, ref.sha256): published + b"!",
        },
    )
    with pytest.raises(AssetDigestMismatchError):
        resolve_asset(ref)


def test_the_store_serves_an_earlier_revision_by_its_digest(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, published: bytes
) -> None:
    """The store is keyed by digest, so a revision the tree no longer holds still resolves."""
    ref = embodiments["so101"].urdf.asset
    _local_tree(monkeypatch, tmp_path, {_URDF_RELPATH: published + b"<!-- revised -->"})
    store = tmp_path / "store"
    target = store / "sha256" / ref.sha256[:2] / ref.sha256
    target.parent.mkdir(parents=True)
    target.write_bytes(published)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_STORE", store.as_uri())
    resolved = resolve_asset(ref)
    assert resolved.read_bytes() == published
    assert resolved.name == "so101.urdf"
