"""The object-first embodiment API, wire law, assets, and runtime projections."""

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
    EmbodiedAsset,
    Embodiment,
    EmbodimentId,
    EmbodimentName,
    EmbodimentSchemaError,
    LayoutError,
    PartValidationError,
    embodiments,
)
from sx_embodiments.curves import Curve1D

_DIGEST = "a" * 64


def _ref(
    uri: str = "https://assets.example/arm.urdf",
    sha256: str = _DIGEST,
) -> EmbodiedAsset:
    return EmbodiedAsset.from_ref(
        AssetRef(
            uri=uri,
            sha256=sha256,
            format=AssetFormat.URDF,
            role=AssetRole.DESCRIPTION,
            provenance=AssetProvenance(
                repository="https://assets.example/source",
                revision="fixture",
                path="arm.urdf",
                license_id="Apache-2.0",
            ),
        )
    )


@pytest.mark.parametrize("digest", ["", "A" * 64, "z" * 64, "a" * 63])
def test_non_canonical_asset_digest_rejected(digest: str) -> None:
    with pytest.raises(AssetIntegrityError):
        _ref(sha256=digest)


def test_relative_uri_rejected() -> None:
    with pytest.raises(AssetIntegrityError):
        _ref(uri="arm.urdf")


def test_from_bytes_hashes_content() -> None:
    asset = AssetRef.from_bytes(
        b"<robot/>",
        uri="urn:sentientx:embodiment:test:urdf",
        format=AssetFormat.URDF,
        role=AssetRole.DESCRIPTION,
    )
    assert asset.byte_size == 8
    assert len(asset.sha256) == 64


def test_embodiment_round_trips_identity_and_assets() -> None:
    embodiment = embodiments["piper"]
    wire = embodiment.to_dict()
    assert wire["schema_version"] == 8
    assert wire["id"] == str(embodiment.id)
    assert wire["name"] == "piper"
    assert wire["kind"] == "robot"
    assert Embodiment.from_dict(wire) == embodiment
    assert Embodiment.from_json(embodiment.to_json()) == embodiment
    assert embodiment.state.width == 7


def test_authoritative_urdf_is_an_asset_property() -> None:
    embodiment = embodiments["piper"]
    assert embodiment.urdf in embodiment.assets
    assert embodiment.urdf_bytes.lstrip().startswith(b"<?xml")


def test_external_assets_preserve_all_embodiment_facts() -> None:
    canonical = embodiments["piper"]
    external_ref = replace(
        canonical.urdf.ref,
        uri="bundle://piper/urdf/piper.urdf",
        logical_path=PurePosixPath("urdf/piper.urdf"),
    )
    external = canonical.with_assets((external_ref,), urdf=canonical.urdf_bytes)
    assert external.state == canonical.state
    assert external.components == canonical.components
    assert external.capabilities == canonical.capabilities
    assert external.cameras == canonical.cameras
    assert external.rates == canonical.rates
    assert external.assets == (EmbodiedAsset.from_ref(external_ref),)
    assert external.id != canonical.id


def test_external_assets_rehash_the_authoritative_urdf() -> None:
    canonical = embodiments["piper"]
    with pytest.raises(AssetIntegrityError, match="authoritative URDF bytes"):
        canonical.with_assets((canonical.urdf.ref,), urdf=b"<robot name='tampered'/>")


def test_id_is_the_content_identity() -> None:
    embodiment = embodiments["piper"]
    assert len(embodiment.id) == 64
    changed = replace(embodiment, name=EmbodimentName("piper-revision"))
    assert changed.id != embodiment.id
    assert embodiments[embodiment.id] == embodiment


@pytest.mark.parametrize("digest", ["", "A" * 64, "z" * 64, "a" * 63])
def test_id_type_fails_closed(digest: str) -> None:
    with pytest.raises(EmbodimentSchemaError):
        EmbodimentId(digest)


def test_from_dict_fails_closed() -> None:
    good = embodiments["piper"].to_dict()
    with pytest.raises(EmbodimentSchemaError):
        Embodiment.from_dict({**good, "schema_version": 6})
    with pytest.raises(EmbodimentSchemaError):
        Embodiment.from_dict({**good, "schema_version": 8.0})
    with pytest.raises(EmbodimentSchemaError):
        Embodiment.from_dict({key: value for key, value in good.items() if key != "assets"})
    raw_assets = cast(list[dict[str, object]], good["assets"])
    bad_assets = [{**entry, "format": "hologram"} for entry in raw_assets]
    with pytest.raises(EmbodimentSchemaError):
        Embodiment.from_dict({**good, "assets": bad_assets})
    with pytest.raises(EmbodimentSchemaError, match="unknown"):
        Embodiment.from_dict({**good, "future_semantic": True})


def test_embodiment_requires_assets() -> None:
    with pytest.raises(EmbodimentSchemaError):
        replace(embodiments["piper"], assets=())


def test_embodiment_rejects_duplicate_assets() -> None:
    asset = _ref()
    with pytest.raises(EmbodimentSchemaError):
        replace(embodiments["piper"], assets=(asset, asset))


def test_capture_rig_carries_cameras() -> None:
    embodiment = embodiments["das-umi-v4"]
    cameras = [
        component
        for component in cast(list[dict[str, object]], embodiment.to_dict()["components"])
        if component["type"] == "camera"
    ]
    assert [camera["name"] for camera in cameras] == ["left_wrist", "right_wrist", "base"]
    assert embodiment.state.width == 2


def test_non_default_camera_optics_round_trip() -> None:
    embodiment = embodiments["yubi-depth"]
    wire = embodiment.to_dict()
    cameras = [
        component
        for component in cast(list[dict[str, object]], wire["components"])
        if component["type"] == "camera"
    ]
    assert any(camera["resolution"] == [1280, 720] for camera in cameras)
    parsed = Embodiment.from_dict(wire)
    assert parsed == embodiment
    assert parsed.id == embodiment.id
    cameras[0]["projection"] = "orthographic"
    with pytest.raises(EmbodimentSchemaError):
        Embodiment.from_dict(wire)


def test_franka_has_deployable_description_assets() -> None:
    embodiment = embodiments["franka"]
    assert embodiment.name == "franka"
    assert [(asset.format, asset.role) for asset in embodiment.assets] == [
        (AssetFormat.URDF, AssetRole.DESCRIPTION),
        (AssetFormat.MJCF, AssetRole.DESCRIPTION),
    ]


def test_arm_kinematics_reject_multi_arm_bodies() -> None:
    with pytest.raises(LayoutError):
        _ = embodiments["bimanual-so101"].single_arm


def test_curve_requires_monotonic_axes() -> None:
    with pytest.raises(PartValidationError):
        Curve1D(x=(0.0, 1.0, 1.0), y=(0.0, 1.0, 2.0))
    with pytest.raises(PartValidationError):
        Curve1D(x=(0.0, 1.0, 2.0), y=(0.0, 1.0, 0.5))


def test_curve_interpolates_and_clamps() -> None:
    curve = Curve1D(x=(0.0, 1.0, 2.0), y=(10.0, 8.0, 4.0))
    assert curve.at(0.5) == 9.0
    assert curve.at(-1.0) == 10.0
    assert curve.at(5.0) == 4.0
    assert curve.inverse_at(9.0) == 0.5
    assert curve.inverse_at(11.0) == 0.0
