"""Materialize a closure from an empty cache through the digest-keyed store, fail-closed."""

import os
import traceback
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from email.message import Message
from pathlib import Path

import pytest
from sx_contracts.content import ContentBlob

from sx_embodiments import (
    AssetDigestMismatchError,
    AssetsUnavailableError,
    embodiments,
    materialize,
)
from sx_embodiments import assets as assets_module
from sx_embodiments.assets import asset_manifest, asset_root, description_asset_uri, resolve_asset
from sx_embodiments.materialize import closure

_TOKEN = "sx-test-bearer-7f3a9c"


def _store(tmp_path: Path, files: dict[str, ContentBlob], tamper: str | None = None) -> str:
    """A ``file://`` store holding ``files`` under ``sha256/<xx>/<digest>``, read from the tree."""
    root = asset_root()
    store = tmp_path / "store"
    for relpath, content in files.items():
        data = (root / relpath).read_bytes()
        if relpath == tamper:
            data = bytes([data[0] ^ 0xFF]) + data[1:]
        target = store / "sha256" / content.sha256[:2] / content.sha256
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    return store.as_uri()


def _no_local_tree(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Path:
    empty = tmp_path / "no-assets"
    empty.mkdir()
    monkeypatch.setenv("SX_EMBODIMENTS_ASSETS", str(empty))
    cache = tmp_path / "cache"
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_CACHE", str(cache))
    monkeypatch.delenv("SX_EMBODIMENTS_ASSET_STORE_TOKEN", raising=False)
    return cache


def test_so101_materializes_from_an_empty_cache_through_the_store(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    so101 = embodiments["so101"]
    files = closure(so101)
    store = _store(tmp_path, files)
    cache = _no_local_tree(monkeypatch, tmp_path)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_STORE", store)

    root = materialize(so101)

    present = {p.relative_to(root).as_posix() for p in root.rglob("*") if p.is_file()}
    assert present == set(files)
    for relpath, content in files.items():
        blob = cache / "sha256" / content.sha256[:2] / content.sha256
        assert os.path.samefile(root / relpath, blob), relpath
        assert (root / relpath).stat().st_size == content.size_bytes
    # Every mesh the description names reads beside it, under its unchanged package name.
    urdf = so101.urdf.asset.uri.removeprefix("package://sx-embodiments/")
    for mesh in ET.parse(root / urdf).getroot().iter("mesh"):
        name = description_asset_uri(so101.urdf.asset.uri, mesh.attrib["filename"])
        assert (root / name.removeprefix("package://sx-embodiments/")).is_file(), name

    # A second call is served from the cache alone: an emptied store proves it.
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_STORE", (tmp_path / "gone").as_uri())
    assert materialize(so101) == root


def test_the_closure_is_exactly_the_description_and_what_it_names() -> None:
    so101 = embodiments["so101"]
    files = closure(so101)
    urdf = so101.urdf.asset.uri.removeprefix("package://sx-embodiments/")
    meshes = {
        description_asset_uri(so101.urdf.asset.uri, mesh.attrib["filename"]).removeprefix(
            "package://sx-embodiments/"
        )
        for mesh in ET.fromstring(so101.urdf_bytes).iter("mesh")
    }
    assert set(files) == {urdf, *meshes}
    assert all(files[mesh] == asset_manifest()[mesh].content for mesh in meshes)


@pytest.mark.parametrize("name", sorted(embodiments))
def test_every_registered_closure_is_in_the_manifest(name: str) -> None:
    assert closure(embodiments[name])


def test_a_tampered_store_byte_is_refused_and_never_cached(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    so101 = embodiments["so101"]
    files = closure(so101)
    mesh = next(relpath for relpath in sorted(files) if relpath.endswith(".stl"))
    store = _store(tmp_path, files, tamper=mesh)
    cache = _no_local_tree(monkeypatch, tmp_path)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_STORE", store)

    with pytest.raises(AssetDigestMismatchError):
        materialize(so101)

    digest = files[mesh].sha256
    assert not (cache / "sha256" / digest[:2] / digest).exists()
    closures = cache / "closures"
    assert not closures.exists() or not any(closures.iterdir())


def test_a_corrupt_cached_blob_is_a_miss_that_is_refetched(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    so101 = embodiments["so101"]
    files = closure(so101)
    store = _store(tmp_path, files)
    cache = _no_local_tree(monkeypatch, tmp_path)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_STORE", store)
    root = materialize(so101)
    mesh = next(relpath for relpath in sorted(files) if relpath.endswith(".stl"))
    blob = cache / "sha256" / files[mesh].sha256[:2] / files[mesh].sha256
    blob.chmod(0o644)
    blob.write_bytes(b"rotted")
    again = materialize(so101)
    assert again == root
    assert (again / mesh).stat().st_size == files[mesh].size_bytes


def test_a_miss_without_a_store_is_fail_closed(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _no_local_tree(monkeypatch, tmp_path)
    monkeypatch.delenv("SX_EMBODIMENTS_ASSET_STORE", raising=False)
    monkeypatch.delenv("SX_EMBODIMENTS_ASSET_MIRROR", raising=False)
    with pytest.raises(AssetsUnavailableError, match="SX_EMBODIMENTS_ASSET_STORE"):
        materialize(embodiments["so101"])


def test_the_retired_mirror_variable_is_a_typed_refusal(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _no_local_tree(monkeypatch, tmp_path)
    monkeypatch.delenv("SX_EMBODIMENTS_ASSET_STORE", raising=False)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_MIRROR", "https://mirror.example/assets")
    with pytest.raises(AssetsUnavailableError, match="retired"):
        resolve_asset(embodiments["so101"].urdf.asset)


def test_a_plain_http_store_is_refused(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    _no_local_tree(monkeypatch, tmp_path)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_STORE", "http://store.example/assets")
    with pytest.raises(AssetsUnavailableError, match="must use one of"):
        resolve_asset(embodiments["so101"].urdf.asset)


def _https_store(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> list[urllib.request.Request]:
    _no_local_tree(monkeypatch, tmp_path)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_STORE", "https://store.example/assets")
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_STORE_TOKEN", _TOKEN)
    return []


@pytest.mark.parametrize("status", [401, 403, 404, 500])
def test_a_refusing_store_is_unavailable_and_never_shows_the_credential(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, status: int
) -> None:
    seen = _https_store(monkeypatch, tmp_path)

    def refuse(request: urllib.request.Request, timeout: float) -> object:
        del timeout
        seen.append(request)
        raise urllib.error.HTTPError(request.full_url, status, "refused", Message(), None)

    monkeypatch.setattr(assets_module.urllib.request, "urlopen", refuse)
    ref = embodiments["so101"].urdf.asset
    with pytest.raises(AssetsUnavailableError) as refused:
        resolve_asset(ref)

    (request,) = seen
    assert request.full_url == f"https://store.example/assets/sha256/{ref.sha256[:2]}/{ref.sha256}"
    # The bearer rides an unredirected header: never forwarded to a redirect's host.
    assert request.unredirected_hdrs == {"Authorization": f"Bearer {_TOKEN}"}
    assert "Authorization" not in request.headers
    assert _TOKEN not in request.full_url
    rendered = "".join(traceback.format_exception(refused.value))
    assert _TOKEN not in rendered
    assert str(status) in str(refused.value)


def test_a_transport_failure_is_unavailable_and_never_shows_the_credential(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    _https_store(monkeypatch, tmp_path)

    def unreachable(request: urllib.request.Request, timeout: float) -> object:
        del request, timeout
        raise urllib.error.URLError("connection refused")

    monkeypatch.setattr(assets_module.urllib.request, "urlopen", unreachable)
    with pytest.raises(AssetsUnavailableError) as refused:
        resolve_asset(embodiments["so101"].urdf.asset)
    assert _TOKEN not in "".join(traceback.format_exception(refused.value))
