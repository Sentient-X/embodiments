"""Asset references, the v3 wire manifest, and the kinematic record's invariants."""

from dataclasses import replace
from typing import cast

import pytest

from sx_embodiments import (
    AssetFormat,
    AssetIntegrityError,
    AssetProvenance,
    AssetRef,
    AssetRole,
    Curve1D,
    Embodiment,
    EmbodimentId,
    EmbodimentManifest,
    EmbodimentManifestDigest,
    LayoutError,
    ManifestSchemaError,
    PartValidationError,
    authoritative_urdf,
    kinematic_view,
    manifest_for,
    manifest_from_dict,
)
from sx_embodiments.known.das import DAS_UMI_V4_SPEC
from sx_embodiments.known.panda import LIBERO_PANDA_SPEC
from sx_embodiments.known.piper import PIPER_SPEC
from sx_embodiments.known.so101 import BIMANUAL_SO101_SPEC

_DIGEST = "a" * 64


def _ref(
    uri: str = "https://assets.example/arm.urdf",
    sha256: str = _DIGEST,
) -> AssetRef:
    return AssetRef(
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


@pytest.mark.parametrize("digest", ["", "A" * 64, "z" * 64, "a" * 63])
def test_non_canonical_digest_rejected(digest: str) -> None:
    with pytest.raises(AssetIntegrityError):
        _ref(sha256=digest)


def test_relative_uri_rejected() -> None:
    with pytest.raises(AssetIntegrityError):
        _ref(uri="arm.urdf")


def test_from_bytes_hashes_content() -> None:
    ref = AssetRef.from_bytes(
        b"<robot/>",
        uri="urn:sentientx:embodiment:test:urdf",
        format=AssetFormat.URDF,
        role=AssetRole.DESCRIPTION,
    )
    assert ref.byte_size == 8
    assert len(ref.sha256) == 64


def test_manifest_v3_round_trips_identity_and_assets() -> None:
    manifest = manifest_for(PIPER_SPEC)
    wire = manifest.to_dict()
    assert wire["schema_version"] == 3
    assert wire["embodiment_id"] == "piper"
    assert wire["kind"] == "robot"
    layout = cast(list[dict[str, object]], wire["layout"])
    assert len(layout) == 7
    parsed = manifest_from_dict(wire)
    assert parsed == manifest
    assert parsed.dof == 7


def test_authoritative_urdf_is_the_manifest_urdf() -> None:
    urdf = authoritative_urdf(PIPER_SPEC)
    manifest = manifest_for(PIPER_SPEC)
    assert urdf.ref() in manifest.assets
    assert urdf.path().read_bytes().lstrip().startswith(b"<?xml")


def test_manifest_digest_is_canonical_and_revision_sensitive() -> None:
    manifest = manifest_for(PIPER_SPEC)
    assert len(manifest.digest()) == 64
    assert manifest.canonical_json() == manifest.canonical_json()
    changed = replace(manifest, name=f"{manifest.name} revision")
    assert changed.digest() != manifest.digest()
    assert manifest.ref().embodiment_id == manifest.embodiment_id
    assert manifest.ref().manifest_sha256 == manifest.digest()


@pytest.mark.parametrize("digest", ["", "A" * 64, "z" * 64, "a" * 63])
def test_manifest_digest_type_fails_closed(digest: str) -> None:
    with pytest.raises(ManifestSchemaError):
        EmbodimentManifestDigest(digest)


def test_manifest_from_dict_fails_closed() -> None:
    good = manifest_for(PIPER_SPEC).to_dict()
    with pytest.raises(ManifestSchemaError):
        manifest_from_dict({**good, "schema_version": 1})
    with pytest.raises(ManifestSchemaError):
        manifest_from_dict({**good, "schema_version": 2.0})
    with pytest.raises(ManifestSchemaError):
        manifest_from_dict({**good, "policy_hz": float("nan")})
    with pytest.raises(ManifestSchemaError):
        manifest_from_dict({key: value for key, value in good.items() if key != "assets"})
    raw_assets = cast(list[dict[str, object]], good["assets"])
    bad_assets: list[dict[str, object]] = [{**entry, "format": "hologram"} for entry in raw_assets]
    with pytest.raises(ManifestSchemaError):
        manifest_from_dict({**good, "assets": bad_assets})


def test_manifest_requires_assets() -> None:
    with pytest.raises(ManifestSchemaError):
        EmbodimentManifest(embodiment_id=EmbodimentId("x"), name="X", assets=())


def test_manifest_rejects_duplicate_assets() -> None:
    ref = _ref()
    with pytest.raises(ManifestSchemaError):
        EmbodimentManifest(embodiment_id=EmbodimentId("x"), name="X", assets=(ref, ref))


def test_capture_rig_manifest_carries_cameras() -> None:
    manifest = manifest_for(DAS_UMI_V4_SPEC)
    wire = manifest.to_dict()
    cameras = cast(list[dict[str, object]], wire["cameras"])
    assert [camera["name"] for camera in cameras] == ["left_wrist", "right_wrist", "base"]
    assert manifest_from_dict(wire) == manifest
    assert wire["dof"] == 2  # two jaw channels


def test_libero_panda_has_a_deployable_asset_manifest() -> None:
    manifest = manifest_for(LIBERO_PANDA_SPEC)
    assert manifest.embodiment_id == EmbodimentId("libero_panda")
    assert [(asset.format, asset.role) for asset in manifest.assets] == [
        (AssetFormat.URDF, AssetRole.DESCRIPTION),
        (AssetFormat.MJCF, AssetRole.DESCRIPTION),
    ]


def test_kinematic_view_rejects_multi_arm_bodies() -> None:
    with pytest.raises(LayoutError):
        kinematic_view(BIMANUAL_SO101_SPEC)


def test_embodiment_joint_length_validation() -> None:
    with pytest.raises(PartValidationError):
        Embodiment(
            embodiment_id=EmbodimentId("bad"),
            dof=2,
            joint_lower=(-1.0,),
            joint_upper=(1.0, 1.0),
            home_joints=(0.0, 0.0),
            gripper_travel_m=(0.0, 0.1),
            policy_hz=10.0,
            mobile_base=False,
        )


def test_curve_requires_monotonic_axes() -> None:
    with pytest.raises(PartValidationError):
        Curve1D(x=(0.0, 1.0, 1.0), y=(0.0, 1.0, 2.0))
    with pytest.raises(PartValidationError):
        Curve1D(x=(0.0, 1.0, 2.0), y=(0.0, 1.0, 0.5))


def test_curve_interpolates_and_clamps() -> None:
    curve = Curve1D(x=(0.0, 1.0, 2.0), y=(10.0, 8.0, 4.0))
    assert curve.at(0.5) == 9.0
    assert curve.at(-1.0) == 10.0  # clamped
    assert curve.at(5.0) == 4.0  # clamped
    assert curve.inverse_at(9.0) == 0.5
    assert curve.inverse_at(11.0) == 0.0  # clamped at the open end
