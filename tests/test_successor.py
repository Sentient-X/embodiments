"""An earlier id of a body that drives as it does stands for the current body."""

import pytest

from sx_embodiments import EmbodimentId, EmbodimentName, historical_document, read_recorded
from sx_embodiments.successor import current, successor, superseded_ids, wiring

STATIC = "sentient-static-upper-body"
# Schema 15: the body was wired otherwise then.
REWIRED = "35e41a6209ee550da7ef857e8fef4f5011a388c68f9bc47834f8ca925a248c6f"


@pytest.mark.parametrize(
    "identity",
    [
        # Before the 10-27 head mount and neck limit corrections: the Spark's assignment.
        "d3b5f72b9a397358ba9eede23af77976c03481013cfe342ff4e9141de2ce0834",
        # Before square-pixel optics.
        "b33e0d04ef8bdb8450d2d373a739bb134772dbe764eee32801b4939713781e1a",
        # Before the work-ready posture: a station sealed episodes against this revision.
        "4b394c4ba404577fee7089ef1e2365afa3adee543c4e7822caf6bd94a69869b4",
        # Before the arms and grippers became reusable modules on fixed shoulder mounts.
        "51db79c80289f76c9f20f2043c114836dfcb80baca88e8f5094762c925a37402",
    ],
)
def test_an_earlier_revision_wired_the_same_way_is_the_same_robot(identity: str) -> None:
    """The modules moved no coordinate: both arms on the CAN bus, then each gripper's bus, then
    the neck's, in the order a station composes its chains, so the robot assigned an earlier
    id opens as the current body."""
    body = current(STATIC)
    old = read_recorded(
        historical_document(EmbodimentName(STATIC), EmbodimentId(identity))
    ).embodiment
    assert identity in superseded_ids(STATIC)
    found = successor(identity)
    assert found is not None and found.id == body.id
    assert wiring(old) == wiring(body)
    assert old.joint_names == body.joint_names


def test_a_rewired_or_unknown_id_has_no_successor() -> None:
    assert REWIRED not in superseded_ids(STATIC)
    assert successor(REWIRED) is None
    assert successor("0" * 64) is None
    assert successor(str(current(STATIC).id)) is None  # the current id is no earlier one
