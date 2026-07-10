import pytest

from sx_embodiments import (
    PANDA_OMRON,
    PIPER,
    AssetFormat,
    AssetRef,
    AssetRole,
    Embodiment,
    EmbodimentId,
    EmbodimentManifest,
)

_DIGEST = "a" * 64


def test_manifest_accepts_content_addressed_urdf() -> None:
    asset = AssetRef(
        uri="s3://robotics-assets/sha256/aa/model.urdf",
        sha256=_DIGEST,
        format=AssetFormat.URDF,
        role=AssetRole.DESCRIPTION,
        byte_size=123,
    )

    manifest = EmbodimentManifest(
        embodiment_id=EmbodimentId("piper@sha256:aaaa"),
        name="Piper",
        assets=(asset,),
        dof=6,
        link_count=8,
    )

    assert manifest.assets == (asset,)


@pytest.mark.parametrize("digest", ["", "A" * 64, "z" * 64, "a" * 63])
def test_asset_rejects_noncanonical_digest(digest: str) -> None:
    with pytest.raises(ValueError, match="sha256"):
        AssetRef(
            uri="https://example.test/model.urdf",
            sha256=digest,
            format=AssetFormat.URDF,
            role=AssetRole.DESCRIPTION,
        )


def test_asset_requires_absolute_uri() -> None:
    with pytest.raises(ValueError, match="absolute"):
        AssetRef(
            uri="models/model.urdf",
            sha256=_DIGEST,
            format=AssetFormat.URDF,
            role=AssetRole.DESCRIPTION,
        )


def test_manifest_rejects_duplicate_asset_identity() -> None:
    asset = AssetRef(
        uri="r2://assets/model.usd",
        sha256=_DIGEST,
        format=AssetFormat.USD,
        role=AssetRole.DESCRIPTION,
    )
    with pytest.raises(ValueError, match="duplicate"):
        EmbodimentManifest(
            embodiment_id=EmbodimentId("robot"),
            name="Robot",
            assets=(asset, asset),
        )


def test_known_embodiments_share_validated_runtime_facts() -> None:
    assert PIPER.dof == 6
    assert PANDA_OMRON.mobile_base
    assert PIPER.gripper_max_width_m == 0.07


def test_runtime_embodiment_validates_joint_lengths() -> None:
    with pytest.raises(ValueError, match="joint_lower"):
        Embodiment(
            embodiment_id=EmbodimentId("bad"),
            dof=2,
            joint_lower=(-1.0,),
            joint_upper=(1.0, 1.0),
            home_joints=(0.0, 0.0),
            gripper_travel_m=(0.0, 0.05),
            policy_hz=10.0,
            mobile_base=False,
        )
