"""Every pinned digest matches the file on disk (the file is the truth; the pin guards)."""

import hashlib
import xml.etree.ElementTree as ET

import pytest

from sx_embodiments import AssetsUnavailableError, PackagedAsset, asset_root
from sx_embodiments.known.das import DAS_GRIPPER_URDF
from sx_embodiments.known.piper import PIPER_MJCF
from sx_embodiments.known.so101 import SO101_URDF
from sx_embodiments.known.sxd_arm import SXD_ARM_URDF

PINNED: tuple[PackagedAsset, ...] = (SO101_URDF, DAS_GRIPPER_URDF, PIPER_MJCF, SXD_ARM_URDF)


@pytest.mark.parametrize("asset", PINNED, ids=lambda a: a.relpath)
def test_pinned_digest_matches_file(asset: PackagedAsset) -> None:
    on_disk = hashlib.sha256(asset.path().read_bytes()).hexdigest()
    assert on_disk == asset.sha256


@pytest.mark.parametrize("asset", PINNED, ids=lambda a: a.relpath)
def test_description_parses_and_ref_projects(asset: PackagedAsset) -> None:
    ET.parse(asset.path())  # well-formed XML
    ref = asset.ref()
    assert ref.sha256 == asset.sha256
    assert ref.uri.startswith("file://")
    assert ref.byte_size == asset.path().stat().st_size


def test_urdf_mesh_references_exist() -> None:
    """Every mesh the pinned URDFs reference resolves inside the assets tree."""
    root = asset_root()
    for asset, mesh_base in (
        (SO101_URDF, root / "so101"),
        (DAS_GRIPPER_URDF, root / "das_gripper_with_vr"),
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


def test_env_override_fails_closed_on_bad_path(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("SX_EMBODIMENTS_ASSETS", "/nonexistent/assets")
    with pytest.raises(AssetsUnavailableError):
        asset_root()


def test_missing_packaged_asset_fails_closed() -> None:
    ghost = PackagedAsset(
        relpath="so101/does_not_exist.urdf",
        sha256=SO101_URDF.sha256,
        format=SO101_URDF.format,
        role=SO101_URDF.role,
    )
    with pytest.raises(AssetsUnavailableError):
        ghost.path()
