"""Nothing drives a body's tool points until a person checked them on its hardware.

Every registered body: its render pack's ``tcp_frames`` (what teleoperation drives) is empty
unless its definition carries a ``tool_points_checked`` record, and its derived tool points are
always offered as ``tcp_candidates`` (drawn, not driven). The record moves no embodiment id.
"""

import dataclasses
import datetime as dt

import pytest

from sx_embodiments import (
    ToolPointCheck,
    development_embodiments,
    embodiments,
    tool_points_checked_of,
)
from sx_embodiments.embodiment import embodiment_from_definition
from sx_embodiments.known import _definition_of  # pyright: ignore[reportPrivateUsage]
from sx_embodiments.tool_points import ToolPointCheckError

pytest.importorskip("numpy")

from sx_embodiments.render_pack import tcp_frames, tool_point_fields
from sx_embodiments.tool_frames import tool_frames
from sx_embodiments.tool_points import tool_chain

BODIES = sorted({*map(str, embodiments), *map(str, development_embodiments)})
CHECKED = {"sentient-static-upper-body"}


def _body(name: str):
    return embodiments[name] if name in embodiments else development_embodiments[name]


@pytest.mark.parametrize("name", BODIES)
def test_only_checked_tool_points_are_driven(name: str) -> None:
    body = _body(name)
    tools = tool_frames(body)
    fields = tool_point_fields(body, tools)
    assert fields["tcp_candidates"] == tcp_frames(body, tools)
    if tool_points_checked_of(name) is None:
        assert fields["tcp_frames"] == {}
        assert "tool_points_checked" not in fields
    else:
        # A record is of this revision and these links; a stale one fails here, asking for a
        # new check (robot-bench), never a silent pass.
        record = tool_points_checked_of(name)
        assert record is not None
        assert record.chain == tool_chain(body.urdf_bytes, record.points)
        assert dict(record.points) == fields["tcp_candidates"]
        assert fields["tcp_frames"] == fields["tcp_candidates"]
        assert fields["tool_points_checked"] == tool_points_checked_of(name).to_json()  # type: ignore[union-attr]


def test_the_checked_bodies_are_the_ones_with_evidence() -> None:
    assert {name for name in BODIES if tool_points_checked_of(name) is not None} == CHECKED


@pytest.mark.parametrize("name", ["b601-dm-station", "bimanual-b601-dm"])
def test_b601_tool_points_are_candidates_until_checked(name: str) -> None:
    body = _body(name)
    fields = tool_point_fields(body, tool_frames(body))
    assert fields["tcp_candidates"] == {"left": "left_gripper_link", "right": "right_gripper_link"}
    assert fields["tcp_frames"] == {}


@pytest.mark.parametrize("name", sorted(CHECKED))
def test_a_check_moves_no_embodiment_id(name: str) -> None:
    definition = _definition_of(name)
    unchecked = dataclasses.replace(definition, tool_points_checked=None)
    assert embodiment_from_definition(unchecked).id == _body(name).id


def test_a_check_says_who_how_and_where() -> None:
    with pytest.raises(ToolPointCheckError):
        ToolPointCheck(
            chain="c",
            points={"left": "l"},
            checked_by="",
            checked_on=dt.date(2026, 11, 5),
            method="m",
            evidence="e",
        )


def test_a_check_of_another_revision_or_other_links_drives_nothing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import sx_embodiments.render_pack as render_pack

    body = _body("sentient-static-upper-body")
    tools = tool_frames(body)
    real = tool_points_checked_of("sentient-static-upper-body")
    assert real is not None
    for stale in (
        dataclasses.replace(real, chain="0" * 64),
        dataclasses.replace(
            real, points={"left": "left_gripper_base_link", "right": "right_gripper_tcp"}
        ),
    ):
        monkeypatch.setattr(render_pack, "tool_points_checked_of", lambda _name, stale=stale: stale)
        fields = render_pack.tool_point_fields(body, tools)
        assert fields["tcp_frames"] == {}
        assert "tool_points_checked" not in fields


def _moved(urdf: bytes, joint: str) -> bytes:
    """``urdf`` with ``joint``'s origin moved a millimetre along x."""
    import xml.etree.ElementTree as ET

    root = ET.fromstring(urdf)  # nosec B314 - the registry's own description
    element = next(j for j in root.findall("joint") if j.get("name") == joint)
    origin = element.find("origin")
    assert origin is not None
    x, y, z = (float(v) for v in origin.get("xyz", "0 0 0").split())
    origin.set("xyz", f"{x + 0.001} {y} {z}")
    return ET.tostring(root)


def test_the_chain_is_what_was_checked_and_nothing_else() -> None:
    from sx_embodiments.kinematics import Kinematics

    body = _body("sentient-static-upper-body")
    record = tool_points_checked_of("sentient-static-upper-body")
    assert record is not None
    urdf = body.urdf_bytes
    kinematics = Kinematics.of(urdf)
    on_path = kinematics.joints[kinematics.parent_joint["left_gripper_tcp"]].name
    # A camera's mount is on no tool point's chain.
    camera = kinematics.joints[kinematics.parent_joint["ZED_camera"]].name
    assert tool_chain(_moved(urdf, on_path), record.points) != record.chain
    assert tool_chain(_moved(urdf, camera), record.points) == record.chain


def test_every_id_a_checked_body_may_be_assigned_names_its_chain() -> None:
    import json
    from pathlib import Path

    registry = json.loads(
        (
            Path(__file__).resolve().parents[1]
            / "drivers/sx-embodiment-bodies/bindings/registry.json"
        ).read_text(encoding="utf-8")
    )
    for document in registry:
        record = tool_points_checked_of(document["embodiment"])
        if record is None:
            assert "tool_chains" not in document, document["embodiment"]
            continue
        ids = [document["id"], *document.get("supersedes", [])]
        assert sorted(document["tool_chains"]) == sorted(ids)
        assert document["tool_chains"][document["id"]] == record.chain


def test_no_two_registered_bodies_share_an_id() -> None:
    """A drive's id resolves to exactly one body (the station opens it and checks its tool
    points by it): no id is two bodies', current or superseded."""
    import json
    from collections import Counter
    from pathlib import Path

    registry = json.loads(
        (
            Path(__file__).resolve().parents[1]
            / "drivers/sx-embodiment-bodies/bindings/registry.json"
        ).read_text(encoding="utf-8")
    )
    counts = Counter(
        identity
        for document in registry
        for identity in (document["id"], *document.get("supersedes", []))
    )
    assert [identity for identity, n in counts.items() if n > 1] == []
