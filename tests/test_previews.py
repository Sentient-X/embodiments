"""Display assets stay tied to exact bodies and retain publication restrictions."""

from sx_embodiments import PlacedEmbodiment, compose_embodiments, embodiments, preview_asset
from sx_embodiments.known import PREVIEWS, asset_audiences


def test_published_previews_are_pinned_small_webp_assets():
    assert PREVIEWS
    audiences = asset_audiences()
    for asset in PREVIEWS.values():
        data = asset.path().read_bytes()  # verifies size and content hash
        assert data[:4] == b"RIFF" and data[8:12] == b"WEBP"
        assert asset.media_type == "image/webp"
        assert 0 < len(data) <= 40_000
        assert asset.relpath.split("/")[0] in audiences


def test_preview_does_not_misrepresent_a_new_composition():
    body = embodiments["so101"]
    assert preview_asset(body) is not None
    pair = compose_embodiments(
        "test-pair",
        {"left": PlacedEmbodiment(body), "right": PlacedEmbodiment(body, (1.0, 0.0, 0.0))},
        label="Two arms",
    )
    assert preview_asset(pair) is None


def test_preview_preserves_source_repository_revision():
    from sx_embodiments import EmbodimentKind
    from sx_embodiments.embodiment import embodiment_from_definition
    from sx_embodiments.known import definitions

    for definition in definitions():
        if definition.kind is EmbodimentKind.CAPTURE_RIG:
            continue
        body = embodiment_from_definition(definition)
        preview = preview_asset(body)
        if preview is None:
            continue
        source = next(a for a in body.assets if a.asset.uri == body.urdf.uri)
        assert preview.provenance.repository == source.provenance.repository
        assert preview.provenance.revision == source.provenance.revision
        assert preview.provenance.path == source.provenance.path
