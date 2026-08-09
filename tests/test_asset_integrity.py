"""Every pinned digest matches the file on disk (the file is the truth; the pin guards)."""

import hashlib
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import pytest
from sx_contracts.assets import AssetFormat

from sx_embodiments import AssetDigestMismatchError, AssetsUnavailableError, resolve_asset
from sx_embodiments.assets import PackagedAsset, asset_root
from sx_embodiments.known.aloha import ALOHA_MJCF, ALOHA_URDF
from sx_embodiments.known.b601 import B601_DM_STATION_URDF, B601_DM_URDF, BIMANUAL_B601_DM_URDF
from sx_embodiments.known.das import DAS_GRIPPER_URDF, DAS_UMI_V4_URDF, QUEST_EGO_URDF
from sx_embodiments.known.g1 import UNITREE_G1_MJCF, UNITREE_G1_URDF
from sx_embodiments.known.humanoid import SENTIENT_HUMANOID_MJCF, SENTIENT_HUMANOID_URDF
from sx_embodiments.known.panda import PANDA_MJCF, PANDA_URDF
from sx_embodiments.known.piper import PIPER_MJCF, PIPER_URDF
from sx_embodiments.known.rby1 import RBY1_MJCF, RBY1_URDF
from sx_embodiments.known.so101 import BIMANUAL_SO101_URDF, SO101_URDF
from sx_embodiments.known.stations import PIPER_STATION_URDF
from sx_embodiments.known.universal_robots import (
    UR5E_MJCF,
    UR5E_URDF,
    UR10E_MJCF,
    UR10E_URDF,
)
from sx_embodiments.known.yor import YOR_MJCF, YOR_URDF
from sx_embodiments.known.yubi import YUBI_HANDS_URDF

PINNED: tuple[PackagedAsset, ...] = (
    SO101_URDF,
    BIMANUAL_SO101_URDF,
    DAS_GRIPPER_URDF,
    DAS_UMI_V4_URDF,
    QUEST_EGO_URDF,
    YUBI_HANDS_URDF,
    PIPER_URDF,
    PIPER_MJCF,
    PANDA_URDF,
    PANDA_MJCF,
    ALOHA_URDF,
    ALOHA_MJCF,
    RBY1_URDF,
    RBY1_MJCF,
    UNITREE_G1_URDF,
    UNITREE_G1_MJCF,
    UR10E_URDF,
    UR10E_MJCF,
    UR5E_URDF,
    UR5E_MJCF,
    YOR_URDF,
    YOR_MJCF,
    SENTIENT_HUMANOID_URDF,
    SENTIENT_HUMANOID_MJCF,
    B601_DM_URDF,
    BIMANUAL_B601_DM_URDF,
    B601_DM_STATION_URDF,
    PIPER_STATION_URDF,
)


@pytest.mark.parametrize("asset", PINNED, ids=lambda a: a.relpath)
def test_pinned_digest_matches_file(asset: PackagedAsset) -> None:
    on_disk = hashlib.sha256(asset.path().read_bytes()).hexdigest()
    assert on_disk == asset.sha256


@pytest.mark.parametrize("asset", PINNED, ids=lambda a: a.relpath)
def test_description_parses_and_ref_projects(asset: PackagedAsset) -> None:
    ET.parse(asset.path())  # well-formed XML
    ref = asset.ref()
    assert ref.sha256 == asset.sha256
    assert ref.uri == f"package://sx-embodiments/{asset.relpath}"
    assert ref.byte_size == asset.path().stat().st_size


def test_urdf_mesh_references_exist() -> None:
    """Every mesh the pinned URDFs reference resolves inside the assets tree."""
    root = asset_root()
    for asset, mesh_base in (
        (SO101_URDF, root / "so101"),
        (BIMANUAL_SO101_URDF, root / "so101"),
        (DAS_GRIPPER_URDF, root / "das_gripper_with_vr"),
        (SENTIENT_HUMANOID_URDF, root / "humanoid_pkg"),
        (B601_DM_URDF, root / "b601_dm"),
        (BIMANUAL_B601_DM_URDF, root / "b601_dm"),
        (B601_DM_STATION_URDF, root / "b601_dm"),
    ):
        tree = ET.parse(asset.path())
        for mesh in tree.getroot().iter("mesh"):
            filename = mesh.get("filename")
            assert filename is not None
            if filename.startswith("package://"):
                tail = filename.removeprefix("package://").split("/", 1)[-1]
            else:
                tail = filename
            candidates = [mesh_base / tail, asset.path().parent / tail]
            assert any(c.is_file() for c in candidates), f"{asset.relpath}: missing {filename}"


@pytest.mark.parametrize(
    "asset",
    tuple(asset for asset in PINNED if asset.format is AssetFormat.MJCF),
    ids=lambda asset: asset.relpath,
)
def test_mjcf_dependencies_exist(asset: PackagedAsset) -> None:
    tree = ET.parse(asset.path())
    root = tree.getroot()
    compiler = root.find("compiler")
    meshdir = compiler.get("meshdir", "") if compiler is not None else ""
    for mesh in root.iter("mesh"):
        filename = mesh.get("file")
        if filename is not None:
            assert (asset.path().parent / meshdir / filename).resolve().is_file(), filename
    for include in root.iter("include"):
        filename = include.get("file")
        assert filename is not None
        assert (asset.path().parent / filename).is_file(), filename


def test_env_override_fails_closed_on_bad_path(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("SX_EMBODIMENTS_ASSETS", "/nonexistent/assets")
    with pytest.raises(AssetsUnavailableError):
        asset_root()


def test_installed_asset_tree_resolves(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    from sx_embodiments import assets

    package = tmp_path / "sx_embodiments"
    installed = package / "_assets"
    installed.mkdir(parents=True)
    monkeypatch.setattr(assets, "__file__", str(package / "assets.py"))
    assert assets.asset_root() == installed


def test_missing_packaged_asset_fails_closed() -> None:
    ghost = replace(SO101_URDF, relpath="so101/does_not_exist.urdf")
    with pytest.raises(AssetsUnavailableError):
        ghost.path()


def test_resolve_asset_inverts_ref_and_verifies_bytes() -> None:
    ref = SO101_URDF.ref()
    resolved = resolve_asset(ref)
    assert resolved == SO101_URDF.path()
    assert hashlib.sha256(resolved.read_bytes()).hexdigest() == ref.sha256


def test_resolve_asset_fails_closed() -> None:
    ref = SO101_URDF.ref()
    with pytest.raises(AssetsUnavailableError, match="not a packaged"):
        resolve_asset(replace(ref, location="https://example.com/so101.urdf"))
    with pytest.raises(AssetsUnavailableError, match="missing on disk"):
        resolve_asset(replace(ref, location="package://sx-embodiments/so101/ghost.urdf"))
    with pytest.raises(AssetDigestMismatchError):
        resolve_asset(
            replace(ref, content=replace(ref.content, sha256=type(ref.content.sha256)("0" * 64)))
        )
