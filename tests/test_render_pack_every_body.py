"""Every body the registry can describe draws: any robot operated shows in a viewer.

No hand list: the bodies come from both registries, the budget and viewing from each body's
own declaration. A body is left out only when its definition carries no description at all
(nothing exists to draw), and that list is pinned so a new one is a decision, not a skip.
"""

import json
import xml.etree.ElementTree as ET

import pytest

from sx_embodiments import development_embodiments, embodiments, viewing_of
from sx_embodiments.identity import EmbodimentName
from sx_embodiments.known import DEVELOPMENT_EMBODIMENTS

pytest.importorskip("numpy")
pytest.importorskip("scipy")

#: Definitions with no description of any module: a capture rig whose jaw is unmeasured.
UNDESCRIBED = {"insta360-umi"}


def _described(name: str) -> bool:
    entry = DEVELOPMENT_EMBODIMENTS.get(EmbodimentName(name))
    if entry is None:
        return True  # a registered body always resolves its description
    return any(placed.module.description is not None for placed in entry.spec.build.placed)


BODIES = {
    **dict.fromkeys(embodiments, embodiments),
    **dict.fromkeys(development_embodiments, development_embodiments),
}


def test_only_bodies_without_a_description_are_not_drawn() -> None:
    assert {name for name in BODIES if not _described(name)} == UNDESCRIBED


@pytest.mark.parametrize("name", [name for name in BODIES if _described(name)])
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
