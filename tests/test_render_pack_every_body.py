"""Every body the registry can describe draws: any robot operated shows in a viewer.

No hand list: the bodies come from both registries, the budget and viewing from each body's
own declaration. A held-back definition with no description at all is not a body (nothing
exists to draw), and that list is pinned so a new one is a decision, not a skip.
"""

import json
import xml.etree.ElementTree as ET

import pytest

from sx_embodiments import development_embodiments, embodiments, viewing_of
from sx_embodiments.known import DEVELOPMENT_EMBODIMENTS

pytest.importorskip("numpy")
pytest.importorskip("scipy")

BODIES = {
    **dict.fromkeys(embodiments, embodiments),
    **dict.fromkeys(development_embodiments, development_embodiments),
}


def test_only_definitions_without_a_description_are_not_bodies() -> None:
    """A held-back definition with no description (a rig whose parts are unmeasured) is
    listed, not built, so it is the one kind of body a viewer cannot draw."""
    assert set(DEVELOPMENT_EMBODIMENTS) - set(development_embodiments) == {"insta360-umi"}


@pytest.mark.parametrize("name", list(BODIES))
def test_every_body_draws_every_link_with_geometry_within_its_budget(name: str) -> None:
    from sx_embodiments.render_pack import build

    body = BODIES[name][name]
    viewing = viewing_of(name)
    pack = build(body, name=name, viewing=viewing)
    document = json.loads(pack.document)
    assert 0 < document["triangles"] <= viewing.triangle_budget

    # Links with a visual are drawn; with no visual anywhere, links with a collision are.
    root = ET.fromstring(pack.urdf)
    tag = "visual" if root.find("link/visual") is not None else "collision"
    shaped = {link.get("name", "") for link in root.findall("link") if link.find(tag) is not None}
    drawn = {link["name"] for link in document["links"] if link["triangles"] > 0}
    assert shaped and shaped <= drawn, sorted(shaped - drawn)
