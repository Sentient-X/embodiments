"""Each body's viewing facts are its registry module's, and a body that declares none gets the
default, so a render pack can be built for any body by name."""

import pytest

from sx_embodiments import development_embodiments, embodiments, viewing_of
from sx_embodiments.errors import UnknownEmbodimentError
from sx_embodiments.known.static_upper_body import (
    STATIC_UPPER_BODY_VIRTUAL_GRIPPER,
    STATIC_UPPER_BODY_WORKCELL,
)
from sx_embodiments.known.yubi import YUBI_TRACKING
from sx_embodiments.viewing import DEFAULT_TRIANGLE_BUDGET, Viewing


def test_a_body_declares_its_own_viewing() -> None:
    static = viewing_of("sentient-static-upper-body")
    assert static.mount == STATIC_UPPER_BODY_WORKCELL
    assert static.virtual_gripper == STATIC_UPPER_BODY_VIRTUAL_GRIPPER
    assert static.triangle_budget == DEFAULT_TRIANGLE_BUDGET
    yubi = viewing_of("yubi")
    assert (yubi.tracking, yubi.triangle_budget) == (YUBI_TRACKING, 60_000)


def test_every_body_has_viewing_facts_and_the_undeclared_get_the_default() -> None:
    names = [*embodiments, *development_embodiments]
    assert all(isinstance(viewing_of(name), Viewing) for name in names)
    assert viewing_of("so101") == Viewing()


def test_a_name_no_registry_holds_is_refused() -> None:
    with pytest.raises(UnknownEmbodimentError):
        viewing_of("no-such-body")
