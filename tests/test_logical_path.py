"""Bundle-member paths: relative, forward-slash, no escape — validated fail-closed."""

from dataclasses import replace
from pathlib import PurePosixPath
from typing import cast

import pytest

from sx_embodiments import (
    AssetFormat,
    AssetIntegrityError,
    AssetProvenance,
    AssetRef,
    AssetRole,
    Embodiment,
    EmbodimentSchemaError,
    embodiments,
    validate_logical_path,
)

_SHA = "a" * 64


def _ref(logical_path: str | None) -> AssetRef:
    return AssetRef(
        uri="https://example.invalid/bundle",
        sha256=_SHA,
        format=AssetFormat.MESH,
        role=AssetRole.GEOMETRY,
        logical_path=PurePosixPath(logical_path) if logical_path is not None else None,
        provenance=AssetProvenance(
            repository="https://example.invalid/source",
            revision="fixture",
            path="mesh.dae",
            license_id="Apache-2.0",
        ),
    )


def test_valid_logical_paths() -> None:
    assert _ref("urdf/panda.urdf").logical_path == PurePosixPath("urdf/panda.urdf")
    assert _ref(None).logical_path is None
    validate_logical_path(PurePosixPath("meshes/visual/link0.dae"))


@pytest.mark.parametrize("bad", ["/etc/passwd", "../escape", "a/../b", ".", "", "a\\b"])
def test_escaping_paths_reject_typed(bad: str) -> None:
    with pytest.raises(AssetIntegrityError):
        _ref(bad)


def test_srdf_is_a_first_class_format() -> None:
    assert AssetFormat("srdf") is AssetFormat.SRDF
    assert AssetRole("license") is AssetRole.LICENSE


def test_same_blob_at_two_logical_paths_is_legal() -> None:
    base = embodiments["franka"]
    embodiment = replace(base, assets=(*base.assets, _ref("meshes/a.dae"), _ref("meshes/b.dae")))
    assert len(embodiment.assets) == len(base.assets) + 2
    with pytest.raises(EmbodimentSchemaError):
        replace(
            base,
            assets=(*base.assets, _ref("meshes/a.dae"), _ref("meshes/a.dae")),
        )


def test_wire_round_trip_carries_logical_path() -> None:
    base = embodiments["franka"]
    embodiment = replace(base, assets=(*base.assets, _ref(None)))
    parsed = Embodiment.from_dict(embodiment.to_dict())
    assert parsed.assets[0].logical_path is not None
    assert parsed.assets[-1].logical_path is None


def test_wire_rejects_malformed_logical_path() -> None:
    document = embodiments["franka"].to_dict()
    assets = document["assets"]
    assert isinstance(assets, list)
    entry = cast("dict[str, object]", assets[0])
    assert isinstance(entry, dict)
    entry["logical_path"] = "../escape"
    with pytest.raises(EmbodimentSchemaError):
        Embodiment.from_dict(document)
    entry["logical_path"] = 7
    with pytest.raises(EmbodimentSchemaError):
        Embodiment.from_dict(document)
