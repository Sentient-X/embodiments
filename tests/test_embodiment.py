"""The object-first embodiment API, wire law, assets, and runtime projections."""

from dataclasses import replace
from pathlib import PurePosixPath
from typing import cast

import pytest
from sx_contracts.assets import (
    AssetFormat,
    AssetIntegrityError,
    AssetProvenance,
    AssetRef,
    AssetRole,
    ProvenancedAsset,
)
from sx_contracts.content import ContentBlob, Sha256Digest

from sx_embodiments import (
    AssetDigestMismatchError,
    Embodiment,
    EmbodimentId,
    EmbodimentName,
    EmbodimentSchemaError,
    LayoutError,
    PartValidationError,
    embodiments,
)
from sx_embodiments.curves import Curve1D, Knot

_DIGEST = "a" * 64


def _ref(
    uri: str = "https://assets.example/arm.urdf",
    sha256: str = _DIGEST,
) -> ProvenancedAsset:
    return ProvenancedAsset(
        asset=AssetRef(
            location=uri,
            content=ContentBlob(Sha256Digest(sha256), 0),
            format=AssetFormat.URDF,
            role=AssetRole.DESCRIPTION,
        ),
        provenance=AssetProvenance(
            repository="https://assets.example/source",
            revision="fixture",
            path="arm.urdf",
            license_id="Apache-2.0",
        ),
    )


@pytest.mark.parametrize("digest", ["", "A" * 64, "z" * 64, "a" * 63])
def test_non_canonical_asset_digest_rejected(digest: str) -> None:
    with pytest.raises(ValueError):
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
    assert wire["schema_version"] == 13
    assert wire["id"] == str(embodiment.id)
    assert wire["name"] == "piper"
    assert wire["kind"] == "robot"
    assert Embodiment.from_dict(wire) == embodiment
    assert Embodiment.from_json(embodiment.to_json()) == embodiment
    assert embodiment.state.width == 7
    mount = embodiment.base_mount
    assert mount is not None and mount.kind.value == "bolt_down" and mount.frame == "base_link"
    assert Embodiment.from_dict(wire).base_mount == mount


def test_tool_frame_facts_round_trip_and_fail_closed() -> None:
    piper = embodiments["piper"]
    assert piper.ready_joints == (0.0, 1.4547, -0.9060, 0.0, 1.1093, 0.0)
    assert piper.grasp_centre_m == (0.0, 0.0, 0.10503)
    round_tripped = Embodiment.from_json(piper.to_json())
    assert round_tripped.ready_joints == piper.ready_joints
    assert round_tripped.grasp_centre_m == piper.grasp_centre_m
    undeclared = embodiments["so101"]
    with pytest.raises(LayoutError):
        _ = undeclared.ready_joints
    with pytest.raises(LayoutError):
        _ = undeclared.grasp_centre_m


def test_authoritative_urdf_is_an_asset_property() -> None:
    embodiment = embodiments["piper"]
    assert embodiment.urdf in embodiment.assets
    assert embodiment.urdf_bytes.lstrip().startswith(b"<?xml")


def test_external_assets_preserve_all_embodiment_facts() -> None:
    canonical = embodiments["piper"]
    external_ref = replace(
        canonical.urdf.asset,
        location="bundle://piper/urdf/piper.urdf",
        logical_path=PurePosixPath("urdf/piper.urdf"),
    )
    external_asset = ProvenancedAsset(external_ref, canonical.urdf.provenance)
    external = canonical.with_assets((external_asset,), urdf=canonical.urdf_bytes)
    assert external.state == canonical.state
    assert external.components == canonical.components
    assert external.capabilities == canonical.capabilities
    assert external.cameras == canonical.cameras
    assert external.rates == canonical.rates
    assert external.assets == (external_asset,)
    assert external.id != canonical.id


def test_external_assets_rehash_the_authoritative_urdf() -> None:
    canonical = embodiments["piper"]
    with pytest.raises(AssetDigestMismatchError, match="expected sha256"):
        canonical.with_assets((canonical.urdf,), urdf=b"<robot name='tampered'/>")


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
        Embodiment.from_dict({**good, "schema_version": 9.0})
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


def test_supported_capture_rig_carries_complete_cameras() -> None:
    embodiment = embodiments["quest-ego"]
    cameras = [
        component
        for component in cast(list[dict[str, object]], embodiment.to_dict()["components"])
        if cast(dict[str, object], component["attachment"])["kind"] == "sensor"
    ]
    assert [camera["instance"] for camera in cameras] == ["head_left", "head_right"]
    for camera in cameras:
        part = cast(dict[str, object], cast(dict[str, object], camera["attachment"])["part"])
        optics = cast(dict[str, object], part["optics"])
        assert optics["width"] == 1280
        assert optics["height"] == 960
        assert len(cast(list[float], optics["image_from_camera"])) == 9
    assert embodiment.state.width == 0


def test_quest_ego_is_a_controller_free_stereo_headset() -> None:
    embodiment = embodiments["quest-ego"]
    assert tuple(camera.name for camera in embodiment.cameras) == ("head_left", "head_right")
    urdf = embodiment.urdf_bytes.decode()
    assert "quest3mesh.obj" in urdf
    assert "controller" not in urdf

    assert tuple(mount.site.value for mount in embodiment.operator_mounts) == ("head",)


def test_camera_optics_round_trip_and_fail_closed() -> None:
    embodiment = embodiments["quest-ego"]
    wire = embodiment.to_dict()
    cameras = [
        component
        for component in cast(list[dict[str, object]], wire["components"])
        if cast(dict[str, object], component["attachment"])["kind"] == "sensor"
    ]
    parts = [
        cast(dict[str, object], cast(dict[str, object], camera["attachment"])["part"])
        for camera in cameras
    ]
    optics = cast(dict[str, object], parts[0]["optics"])
    assert optics["image_from_camera"] == [
        868.31,
        0.0,
        640.18,
        0.0,
        868.31,
        482.07,
        0.0,
        0.0,
        1.0,
    ]
    parsed = Embodiment.from_dict(wire)
    assert parsed == embodiment
    assert parsed.id == embodiment.id
    optics["image_from_camera"] = [0.0] * 9
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
        Curve1D((Knot(0.0, 0.0), Knot(1.0, 1.0), Knot(1.0, 2.0)))
    with pytest.raises(PartValidationError):
        Curve1D((Knot(0.0, 0.0), Knot(1.0, 1.0), Knot(2.0, 0.5)))


def test_curve_interpolates_and_clamps() -> None:
    curve = Curve1D((Knot(0.0, 10.0), Knot(1.0, 8.0), Knot(2.0, 4.0)))
    assert curve.at(0.5) == 9.0
    assert curve.at(-1.0) == 10.0
    assert curve.at(5.0) == 4.0
    assert curve.inverse_at(9.0) == 0.5
    assert curve.inverse_at(11.0) == 0.0
