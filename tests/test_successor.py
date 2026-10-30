"""An earlier id of a body that drives as it does stands for the current body."""

from sx_embodiments.successor import current, successor, superseded_ids, wiring

STATIC = "sentient-static-upper-body"
# Before the 10-27 head mount and neck limit corrections: the id the Spark was assigned.
BEFORE = "d3b5f72b9a397358ba9eede23af77976c03481013cfe342ff4e9141de2ce0834"
# Schema 15: the body was wired otherwise then.
REWIRED = "35e41a6209ee550da7ef857e8fef4f5011a388c68f9bc47834f8ca925a248c6f"


def test_a_compatible_earlier_id_resolves_to_the_current_body() -> None:
    body = current(STATIC)
    assert BEFORE in superseded_ids(STATIC)
    found = successor(BEFORE)
    assert found is not None and found.id == body.id
    assert wiring(found) == wiring(body)


def test_a_rewired_or_unknown_id_has_no_successor() -> None:
    assert REWIRED not in superseded_ids(STATIC)
    assert successor(REWIRED) is None
    assert successor("0" * 64) is None
    assert successor(str(current(STATIC).id)) is None  # the current id is no earlier one


def test_the_body_before_its_square_pixel_optics_is_the_same_robot() -> None:
    """The ZED's corrected nominal optics changed the id, not how the body drives."""
    before_optics = "b33e0d04ef8bdb8450d2d373a739bb134772dbe764eee32801b4939713781e1a"
    assert before_optics in superseded_ids(STATIC)
    found = successor(before_optics)
    assert found is not None and found.id == current(STATIC).id


def test_the_body_before_its_work_ready_posture_is_the_same_robot() -> None:
    """The square-pixel revision (embodiments #180), superseded by the work-ready posture
    (#177): a station sealed episodes on it, so it stays readable and drawable."""
    before_posture = "4b394c4ba404577fee7089ef1e2365afa3adee543c4e7822caf6bd94a69869b4"
    assert before_posture in superseded_ids(STATIC)
    found = successor(before_posture)
    assert found is not None and found.id == current(STATIC).id
