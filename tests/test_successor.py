"""A successor preserves ordered drive semantics; old recordings retain their own bodies."""

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
    ],
)
def test_pre_module_revisions_remain_readable_without_a_positional_successor(identity: str) -> None:
    """These revisions once succeeded one another; fitted arms now interleave their grippers.
    An unchanged actuator set does not make those ordered coordinate arrays interchangeable.
    """
    body = current(STATIC)
    recorded = read_recorded(historical_document(EmbodimentName(STATIC), EmbodimentId(identity)))
    old = recorded.embodiment
    assert str(recorded.recorded_id) == identity
    assert identity not in superseded_ids(STATIC)
    assert successor(identity) is None
    assert wiring(old) != wiring(body)
    assert set(wiring(old)) == set(wiring(body))
    assert set(old.joint_names) == set(body.joint_names)
    assert old.joint_names.index("right_shoulder_pitch_joint") < old.joint_names.index(
        "left_gripper_left_finger_joint"
    )
    assert body.joint_names.index("left_gripper_left_finger_joint") < body.joint_names.index(
        "right_shoulder_pitch_joint"
    )
    # The historical description resolves with its own content identity.
    assert old.urdf_bytes


def test_a_rewired_or_unknown_id_has_no_successor() -> None:
    assert REWIRED not in superseded_ids(STATIC)
    assert successor(REWIRED) is None
    assert successor("0" * 64) is None
    assert successor(str(current(STATIC).id)) is None  # the current id is no earlier one
