"""A recording's embodiment keeps the identity it was recorded under."""

import json
from importlib import resources

import pytest
from sx_contracts.identity import content_id

from sx_embodiments import (
    EmbodimentId,
    EmbodimentName,
    EmbodimentSchemaError,
    development_embodiments,
    historical_identities,
    read_recorded,
    resolve_asset,
)

# The canonical YUBI package published under schema 13 from bf11315 (2026-08-20) until
# the schema-14 bump, as the field pods recorded it. The schema-13 commit itself
# (7e0c968, pinned for hours that day) minted 9b537cea…, whose parts predate `grasp`:
# the converter cannot read it, and no field recording carries it.
YUBI_SCHEMA_13 = {
    EmbodimentId("a645c2bc67e91aa2ebd49039368f85bcd205e90bdd5e81404adcabd6487f7afd"),
}


def _historical_document(identity: EmbodimentId) -> dict[str, object]:
    path = resources.files("sx_embodiments") / "historical" / f"yubi-{identity}.json"
    return json.loads(path.read_text())


def test_historical_yubi_identities_are_the_published_schema_13_packages() -> None:
    assert historical_identities(EmbodimentName("yubi")) == YUBI_SCHEMA_13
    assert historical_identities(EmbodimentName("so101")) == frozenset()
    assert historical_identities(EmbodimentName("yubi")).isdisjoint(
        {development_embodiments["yubi"].id}
    )


def test_historical_identity_survives_a_canonical_change() -> None:
    """Admission rests on the published documents, never on today's canonical package."""
    for identity in YUBI_SCHEMA_13:
        assert identity != development_embodiments["yubi"].id
        recorded = read_recorded(_historical_document(identity))
        assert recorded.recorded_id == identity
        assert recorded.embodiment.name == "yubi"


def test_a_current_document_reads_exactly() -> None:
    current = development_embodiments["yubi"]
    recorded = read_recorded(current.to_dict())
    assert recorded.embodiment == current
    assert recorded.recorded_id == current.id


def test_a_schema_13_document_whose_content_moved_is_refused() -> None:
    document = _historical_document(min(YUBI_SCHEMA_13))
    document["label"] = "relabelled"
    with pytest.raises(EmbodimentSchemaError, match="id does not match"):
        read_recorded(document)


def test_a_re_identified_foreign_schema_13_package_is_not_historical() -> None:
    document = _historical_document(min(YUBI_SCHEMA_13))
    document["label"] = "relabelled"
    content = {key: value for key, value in document.items() if key != "id"}
    document["id"] = str(content_id(EmbodimentId, content))
    recorded = read_recorded(document)
    assert recorded.recorded_id not in historical_identities(EmbodimentName("yubi"))


def test_every_asset_a_historical_document_names_still_resolves() -> None:
    """A later change to a canonical file must keep the published revision by digest
    (``assets.superseded_relpath``); this is where forgetting to fails."""
    for entry in (resources.files("sx_embodiments") / "historical").iterdir():
        if entry.name.endswith(".json"):
            recorded = read_recorded(json.loads(entry.read_text()))
            for asset in recorded.embodiment.assets:
                resolve_asset(asset.asset)
