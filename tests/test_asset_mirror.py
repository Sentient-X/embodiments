"""The digest-verified mirror fetch/cache in front of the local fail-closed miss."""

import hashlib
from pathlib import Path

import pytest

from sx_embodiments import AssetDigestMismatchError, AssetsUnavailableError, embodiments
from sx_embodiments.assets import resolve_asset

_URDF_RELPATH = "so101/so101.urdf"


@pytest.fixture
def urdf_bytes() -> bytes:
    return embodiments["so101"].urdf_bytes


def _empty_local_tree(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Path:
    local = tmp_path / "empty-assets"
    local.mkdir()
    monkeypatch.setenv("SX_EMBODIMENTS_ASSETS", str(local))
    cache = tmp_path / "cache"
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_CACHE", str(cache))
    return cache


def _mirror(tmp_path: Path, relpath: str, data: bytes) -> str:
    tree = tmp_path / "mirror"
    target = tree / relpath
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(data)
    return tree.as_uri()


def test_a_local_miss_without_a_mirror_stays_fail_closed(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, urdf_bytes: bytes
) -> None:
    del urdf_bytes
    _empty_local_tree(monkeypatch, tmp_path)
    monkeypatch.delenv("SX_EMBODIMENTS_ASSET_MIRROR", raising=False)
    with pytest.raises(AssetsUnavailableError, match="SX_EMBODIMENTS_ASSET_MIRROR"):
        resolve_asset(embodiments["so101"].urdf.asset)


def test_the_mirror_serves_verified_bytes_and_the_cache_survives_the_mirror(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, urdf_bytes: bytes
) -> None:
    cache = _empty_local_tree(monkeypatch, tmp_path)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_MIRROR", _mirror(tmp_path, _URDF_RELPATH, urdf_bytes))
    ref = embodiments["so101"].urdf.asset
    resolved = resolve_asset(ref)
    assert resolved == cache / _URDF_RELPATH
    assert hashlib.sha256(resolved.read_bytes()).hexdigest() == ref.sha256
    # A second resolve is served from the verified cache: emptying the mirror proves it.
    (tmp_path / "mirror" / _URDF_RELPATH).unlink()
    assert resolve_asset(ref) == resolved


def test_tampered_mirror_bytes_refuse_and_never_enter_the_cache(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, urdf_bytes: bytes
) -> None:
    cache = _empty_local_tree(monkeypatch, tmp_path)
    monkeypatch.setenv(
        "SX_EMBODIMENTS_ASSET_MIRROR",
        _mirror(tmp_path, _URDF_RELPATH, urdf_bytes + b"<!-- tampered -->"),
    )
    with pytest.raises(AssetDigestMismatchError):
        resolve_asset(embodiments["so101"].urdf.asset)
    assert not (cache / _URDF_RELPATH).exists()


def test_a_corrupt_cache_entry_is_refetched_and_reverified(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, urdf_bytes: bytes
) -> None:
    cache = _empty_local_tree(monkeypatch, tmp_path)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_MIRROR", _mirror(tmp_path, _URDF_RELPATH, urdf_bytes))
    stale = cache / _URDF_RELPATH
    stale.parent.mkdir(parents=True)
    stale.write_bytes(b"rotted")
    resolved = resolve_asset(embodiments["so101"].urdf.asset)
    assert resolved.read_bytes() == urdf_bytes


def test_a_non_https_mirror_scheme_is_refused(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, urdf_bytes: bytes
) -> None:
    del urdf_bytes
    _empty_local_tree(monkeypatch, tmp_path)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_MIRROR", "http://mirror.example/assets")
    with pytest.raises(AssetsUnavailableError, match="must use one of"):
        resolve_asset(embodiments["so101"].urdf.asset)


def test_packaged_asset_path_serves_and_refuses_through_the_mirror(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, urdf_bytes: bytes
) -> None:
    # The PackagedAsset.path() route has no outer digest re-check, so it must be
    # proven against the mirror directly — clean bytes served, tampered refused.
    from sx_embodiments.known.so101 import SO101_URDF

    _empty_local_tree(monkeypatch, tmp_path)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_MIRROR", _mirror(tmp_path, _URDF_RELPATH, urdf_bytes))
    assert SO101_URDF.path().read_bytes() == urdf_bytes
    tampered_tmp = tmp_path / "second"
    tampered_tmp.mkdir()
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_CACHE", str(tmp_path / "cache2"))
    monkeypatch.setenv(
        "SX_EMBODIMENTS_ASSET_MIRROR",
        _mirror(tampered_tmp, _URDF_RELPATH, urdf_bytes + b"!"),
    )
    with pytest.raises(AssetDigestMismatchError):
        SO101_URDF.path()
