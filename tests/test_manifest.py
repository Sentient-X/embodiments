import pytest

from sx_embodiments import AssetFormat, AssetRef, AssetRole, EmbodimentId, EmbodimentManifest

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
