"""A body's earlier revisions under the current schema read as the revision that superseded
it, so a recording that names one still finds its body."""

from sx_embodiments import embodiments
from sx_embodiments.identity import EmbodimentId
from sx_embodiments.known import definitions

#: panda_omron before its Omron LD base was drawn; RoboCasa episodes name this identity.
PANDA_ALONE = EmbodimentId("05db30ab443a7bd5fb9ecef055fbca2a12fcd3f244adfca6f7712c1f8a847140")


def test_the_panda_alone_resolves_to_the_panda_on_its_omron_base() -> None:
    body = embodiments[PANDA_ALONE]
    assert body is embodiments["panda_omron"]
    assert body.id != PANDA_ALONE
    assert any(link == "omron_ld" for link in body.urdf_bytes.decode().split('"'))


def test_an_earlier_revision_is_no_current_body_and_belongs_to_one_body() -> None:
    current = {str(body.id) for body in embodiments.values()}
    claimed: dict[str, str] = {}
    for definition in definitions():
        for earlier in definition.earlier_revisions:
            assert str(earlier) not in current, definition.name
            assert claimed.setdefault(str(earlier), str(definition.name)) == definition.name
