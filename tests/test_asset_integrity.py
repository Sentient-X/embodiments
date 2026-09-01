"""Every pinned digest matches the file on disk (the file is the truth; the pin guards)."""

import hashlib
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import pytest
from sx_contracts.assets import AssetFormat, AssetIntegrityError

from sx_embodiments import AssetDigestMismatchError, AssetsUnavailableError, resolve_asset
from sx_embodiments.assets import PackagedAsset, asset_root
from sx_embodiments.known.aloha import ALOHA_MJCF, ALOHA_URDF
from sx_embodiments.known.b601 import B601_DM_STATION_URDF, B601_DM_URDF, BIMANUAL_B601_DM_URDF
from sx_embodiments.known.das import DAS_GRIPPER_URDF, DAS_UMI_V4_URDF, QUEST_EGO_URDF
from sx_embodiments.known.ffw import FFW_BG2_URDF
from sx_embodiments.known.g1 import UNITREE_G1_MJCF, UNITREE_G1_URDF
from sx_embodiments.known.humanoid import SENTIENT_HUMANOID_MJCF, SENTIENT_HUMANOID_URDF
from sx_embodiments.known.panda import PANDA_MJCF, PANDA_URDF
from sx_embodiments.known.piper import PIPER_MJCF, PIPER_URDF
from sx_embodiments.known.rby1 import RBY1_MJCF, RBY1_URDF
from sx_embodiments.known.sentient_rwh import SENTIENT_RWH_URDF
from sx_embodiments.known.so101 import BIMANUAL_SO101_URDF, SO101_URDF
from sx_embodiments.known.stations import PIPER_STATION_URDF
from sx_embodiments.known.universal_robots import (
    UR5E_MJCF,
    UR5E_URDF,
    UR10E_MJCF,
    UR10E_URDF,
)
from sx_embodiments.known.yor import YOR_MJCF, YOR_URDF
from sx_embodiments.known.yubi import YUBI_HANDS_URDF, YUBI_MESHES

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
    FFW_BG2_URDF,
    SENTIENT_RWH_URDF,
)


@pytest.mark.parametrize("asset", (*PINNED, *YUBI_MESHES), ids=lambda a: a.relpath)
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


def test_ref_projects_declared_identity_without_asset_bytes(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    empty = tmp_path / "no-assets"
    empty.mkdir()
    monkeypatch.setenv("SX_EMBODIMENTS_ASSETS", str(empty))
    monkeypatch.delenv("SX_EMBODIMENTS_ASSET_MIRROR", raising=False)

    ref = SO101_URDF.ref()

    assert ref.sha256 == SO101_URDF.sha256
    assert ref.byte_size == SO101_URDF.content.size_bytes
    assert ref.uri == f"package://sx-embodiments/{SO101_URDF.relpath}"


def test_packaged_asset_path_verifies_local_digest_and_size(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    original = (asset_root() / SO101_URDF.relpath).read_bytes()
    local = tmp_path / "assets"
    candidate = local / SO101_URDF.relpath
    candidate.parent.mkdir(parents=True)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSETS", str(local))
    monkeypatch.delenv("SX_EMBODIMENTS_ASSET_MIRROR", raising=False)

    candidate.write_bytes(original + b"tampered")
    with pytest.raises(AssetDigestMismatchError):
        SO101_URDF.path()

    candidate.write_bytes(original)
    wrong_size = replace(
        SO101_URDF,
        content=replace(SO101_URDF.content, size_bytes=SO101_URDF.content.size_bytes + 1),
    )
    with pytest.raises(AssetIntegrityError, match="expected"):
        wrong_size.path()


def test_urdf_mesh_references_exist() -> None:
    """Every mesh the pinned URDFs reference resolves inside the assets tree."""
    root = asset_root()
    for asset, mesh_base in (
        (SO101_URDF, root / "so101"),
        (BIMANUAL_SO101_URDF, root / "so101"),
        (DAS_GRIPPER_URDF, root / "das_gripper_with_vr"),
        (YUBI_HANDS_URDF, root / "yubi_description"),
        (SENTIENT_HUMANOID_URDF, root / "humanoid_pkg"),
        (B601_DM_URDF, root / "b601_dm"),
        (BIMANUAL_B601_DM_URDF, root / "b601_dm"),
        (B601_DM_STATION_URDF, root / "b601_dm"),
        (FFW_BG2_URDF, root / "ai_worker/ffw_bg2_rev4"),
        (SENTIENT_RWH_URDF, root / "sentient_rwh"),
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


def test_materializing_the_registry_does_not_require_the_asset_tree() -> None:
    """Declarations and refs are authored, so naming a robot touches no asset file.

    This package is imported by four pillars, and its `known/` modules build every
    PackagedAsset at module scope. Materializing one robot projects those declarations
    into AssetRefs. Neither operation may make the registry a hard dependency on all
    632 asset files: an image that legitimately ships none of them, like the GPU step
    runtime whose build context is capped at 32 MiB, must still be able to name the
    robot and carry its content identity. Absence surfaces where bytes are requested.

    A fresh interpreter is the only honest check: the registry is a module-level
    singleton, so an in-process import would already be cached and prove nothing.
    """
    env = {**os.environ, "SX_EMBODIMENTS_ASSETS": str(Path("/nonexistent-asset-root"))}
    probe = (
        "import sx_embodiments;"
        "e = sx_embodiments.embodiments['franka'];"
        "from sx_embodiments.known.das import QUEST3_HEADSET_MESH as m;"
        "print(len(list(sx_embodiments.embodiments)), len(e.assets), e.id, m.content.size_bytes)"
    )
    done = subprocess.run(
        [sys.executable, "-c", probe], env=env, capture_output=True, text=True, check=False
    )
    assert done.returncode == 0, done.stderr
    count, asset_count, embodiment_id, size = done.stdout.split()
    assert int(count) > 0 and int(asset_count) > 0 and len(embodiment_id) == 64 and int(size) > 0

    # ...and the same interpreter still refuses to hand out bytes it cannot verify.
    denied = subprocess.run(
        [
            sys.executable,
            "-c",
            "from sx_embodiments import embodiments; embodiments['franka'].urdf_path",
        ],
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )
    assert denied.returncode != 0
    assert "AssetsUnavailableError" in denied.stderr
