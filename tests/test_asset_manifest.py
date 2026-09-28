"""The committed manifest is exactly what the tree and the declarations render."""

import shutil
from pathlib import Path

import pytest

from sx_embodiments.assets import (
    MANIFEST_PATH,
    AssetAudience,
    asset_manifest,
    asset_root,
    parse_manifest,
    render_manifest_json,
)
from sx_embodiments.known import asset_audiences
from sx_embodiments.known.so101 import SO101_URDF
from sx_embodiments.manifest import (
    ManifestDisagreementError,
    manifest_disagreements,
    render_manifest,
)


def test_the_manifest_agrees_with_the_tree_and_the_declarations() -> None:
    """Regenerate with `python tools/render_asset_manifest.py` when this fails."""
    assert manifest_disagreements(asset_root()) == ()


def test_the_manifest_agrees_with_the_declarations_without_a_tree() -> None:
    assert manifest_disagreements(None) == ()


def test_the_manifest_is_its_own_one_rendering() -> None:
    text = MANIFEST_PATH.read_text(encoding="utf-8")
    assert render_manifest_json(parse_manifest(text)) == text


def test_every_row_keeps_its_directory_audience() -> None:
    audiences = asset_audiences()
    for entry in asset_manifest().values():
        directory = entry.relpath.split("/", 1)[0]
        assert entry.audience is audiences[directory], entry.relpath
    entitled = {
        entry.relpath.split("/", 1)[0]
        for entry in asset_manifest().values()
        if entry.audience is AssetAudience.ENTITLED
    }
    assert entitled == {"sentient_rwh", "yubi_description", "piper_umi_description"}


def test_a_tree_that_disagrees_with_a_declaration_refuses(tmp_path: Path) -> None:
    tree = tmp_path / "assets"
    shutil.copytree(asset_root() / "so101", tree / "so101")
    urdf = tree / SO101_URDF.relpath
    urdf.write_bytes(urdf.read_bytes() + b"<!-- edited -->")
    (tree / "so101" / "undeclared.bin").write_bytes(b"x")
    (tree / "stray.bin").write_bytes(b"x")
    with pytest.raises(ManifestDisagreementError) as refused:
        render_manifest(tree)
    found = "\n".join(refused.value.disagreements)
    assert f"{SO101_URDF.relpath}: declared" in found
    assert "stray.bin: no declaration licenses this file" in found
    assert "declared but absent from the tree" in found
    # An undeclared file inside a declared directory is licensed, not refused.
    assert "so101/undeclared.bin" not in found


def test_a_file_under_disagreeing_siblings_is_not_licensed_by_their_union(tmp_path: Path) -> None:
    """menagerie/ holds robots under three licences; a new undeclared robot inherits none."""
    tree = tmp_path / "assets"
    source = asset_root() / "menagerie" / "franka_emika_panda"
    shutil.copytree(source, tree / "menagerie" / "franka_emika_panda")
    shutil.copytree(
        asset_root() / "menagerie" / "agilex_piper", tree / "menagerie" / "agilex_piper"
    )
    (tree / "menagerie" / "unknown_robot").mkdir()
    (tree / "menagerie" / "unknown_robot" / "arm.stl").write_bytes(b"solid")
    with pytest.raises(ManifestDisagreementError) as refused:
        render_manifest(tree)
    assert any(
        line.startswith("menagerie/unknown_robot/arm.stl: no declaration licenses this file")
        for line in refused.value.disagreements
    )


def test_no_row_carries_a_union_of_sibling_licences() -> None:
    joined = [e.relpath for e in asset_manifest().values() if " AND " in e.license_id]
    assert joined == []
    assert all(
        e.license_id == "MIT"
        for e in asset_manifest().values()
        if e.relpath.startswith("menagerie/i2rt_yam/")
    )
