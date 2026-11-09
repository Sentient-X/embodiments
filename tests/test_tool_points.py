"""Nothing drives a body's tool points until a person checked them on its hardware.

Every registered body: its render pack's ``tcp_frames`` (what teleoperation drives) is empty
unless its definition carries a ``tool_points_checked`` record, and its derived tool points are
always offered as ``tcp_candidates`` (drawn, not driven). The record moves no embodiment id.
"""

import dataclasses
import datetime as dt

import pytest

from sx_embodiments import (
    EmbodimentId,
    EmbodimentName,
    ToolPointCheck,
    development_embodiments,
    embodiments,
    historical_document,
    read_recorded,
    tool_points_checked_of,
)
from sx_embodiments.embodiment import embodiment_from_definition
from sx_embodiments.known import _definition_of  # pyright: ignore[reportPrivateUsage]
from sx_embodiments.tool_points import ToolPointCheckError

pytest.importorskip("numpy")

from sx_embodiments.render_pack import tcp_frames, tool_point_fields
from sx_embodiments.tool_frames import tool_frames, tool_grips
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
        grips = tool_grips(body, record.points, tools)
        assert record.chain == tool_chain(body.urdf_bytes, record.points, grips)
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


def _edited(urdf: bytes, joint: str, **change: str) -> bytes:
    """``urdf`` with ``joint`` changed: ``xyz``/``rpy`` set its origin, ``axis`` its axis,
    ``type`` its type, ``upper`` its limit's upper bound."""
    import xml.etree.ElementTree as ET

    root = ET.fromstring(urdf)  # nosec B314 - the registry's own description
    element = next(j for j in root.findall("joint") if j.get("name") == joint)
    for key, value in change.items():
        if key == "type":
            element.set("type", value)
        elif key in ("xyz", "rpy"):
            origin = element.find("origin")
            if origin is None:
                origin = ET.SubElement(element, "origin")
            origin.set(key, value)
        elif key == "axis":
            axis = element.find("axis")
            if axis is None:
                axis = ET.SubElement(element, "axis")
            axis.set("xyz", value)
        else:
            limit = element.find("limit")
            assert limit is not None
            limit.set(key, value)
    return ET.tostring(root)


def test_the_chain_is_what_was_checked_and_nothing_else() -> None:
    from sx_embodiments.kinematics import Kinematics

    body = _body("sentient-static-upper-body")
    record = tool_points_checked_of("sentient-static-upper-body")
    assert record is not None
    urdf = body.urdf_bytes
    kinematics = Kinematics.of(urdf)
    grips = tool_grips(body, record.points)
    assert tool_chain(urdf, record.points, grips) == record.chain

    def above(link: str) -> str:
        return kinematics.joints[kinematics.parent_joint[link]].name

    tcp = above("left_gripper_tcp")
    mount = kinematics.joints[tcp].parent
    finger = next(j.name for j in kinematics.joints.values() if j.parent == mount and j.name != tcp)
    revolute = next(
        j.name
        for j in kinematics.joints.values()
        if j.type.value == "revolute" and j.name.startswith("left_") and "finger" not in j.name
    )
    moved = {
        "the tool point, along x": _edited(urdf, tcp, xyz="0.2 0 0"),
        "the tool point, turned": _edited(urdf, tcp, rpy="0 0.1 0"),
        "a joint up the arm, turned": _edited(urdf, revolute, rpy="0.1 0 0"),
        "a joint up the arm, its axis": _edited(urdf, revolute, axis="0 0 -1"),
        "a joint up the arm, its type": _edited(urdf, revolute, type="continuous"),
        "the right tool point": _edited(urdf, above("right_gripper_tcp"), xyz="0.2 0 0"),
        "a finger of the tool": _edited(urdf, finger, xyz="0.03 0 0"),
        "a finger's axis": _edited(urdf, finger, axis="0 1 0"),
    }
    for what, edited in moved.items():
        assert tool_chain(edited, record.points, grips) != record.chain, what
    # Off the tool path: the head's camera mount, and a joint's range.
    unchanged = {
        "the camera mount": _edited(urdf, above("ZED_camera"), xyz="0.5 0 0"),
        "a joint's limit": _edited(urdf, revolute, upper="0.5"),
    }
    for what, edited in unchanged.items():
        assert tool_chain(edited, record.points, grips) == record.chain, what


def test_where_the_jaws_close_is_part_of_the_chain() -> None:
    """A revision whose fingers' collision geometry moves where its jaws close (the grasp point,
    the approach or the closing line) is unchecked, though no joint moved: the digest binds
    each side's grip as the render pack states it."""
    import copy

    body = _body("sentient-static-upper-body")
    record = tool_points_checked_of("sentient-static-upper-body")
    assert record is not None
    grips = tool_grips(body, record.points)
    assert set(grips) == set(record.points)
    urdf = body.urdf_bytes
    for field, index, delta in (
        ("grasp_point_m", 0, 0.01),
        ("grasp_point_m", 2, 1e-6),
        ("approach_axis", 1, 0.01),
        ("closing_axis", 2, 0.01),
    ):
        moved = copy.deepcopy(grips)
        moved["left"]["grip"][field][index] += delta  # type: ignore[index]
        assert tool_chain(urdf, record.points, moved) != record.chain, (field, index)
    with pytest.raises(ToolPointCheckError, match="rides with no jawed tool"):
        tool_chain(urdf, record.points, {"left": grips["left"]})


def test_an_earlier_revisions_chain_is_read_from_its_own_description(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The bindings' chain for a superseded id comes from that revision's kept description:
    an earlier revision whose tool point sat elsewhere names its own chain, never today's."""
    import importlib.util
    import types
    from pathlib import Path

    from sx_embodiments.kinematics import Kinematics

    path = Path(__file__).resolve().parents[1] / "tools" / "render_driver_bindings.py"
    spec = importlib.util.spec_from_file_location("render_driver_bindings", path)
    assert spec is not None and spec.loader is not None
    bindings = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bindings)
    body = _body("sentient-static-upper-body")
    record = tool_points_checked_of("sentient-static-upper-body")
    assert record is not None
    kinematics = Kinematics.of(body.urdf_bytes)
    tcp = kinematics.joints[kinematics.parent_joint["left_gripper_tcp"]].name
    elsewhere = _edited(body.urdf_bytes, tcp, xyz="0.2 0 0")
    monkeypatch.setattr(
        bindings,
        "read_recorded",
        lambda _document: types.SimpleNamespace(
            embodiment=types.SimpleNamespace(urdf_bytes=elsewhere)
        ),
    )
    grips = tool_grips(body, record.points)

    def same_grips(_revision: object, _points: object) -> dict[str, object]:
        return dict(grips)

    monkeypatch.setattr(bindings, "tool_grips", same_grips)
    document = bindings.document("sentient-static-upper-body")
    chains = document["tool_chains"]
    assert chains[document["id"]] == record.chain
    earlier = [identity for identity in chains if identity != document["id"]]
    assert earlier, "the static body supersedes earlier ids"
    for identity in earlier:
        assert chains[identity] != record.chain, identity


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


def test_a_fixed_mount_that_leaves_the_path_in_place_keeps_the_chain() -> None:
    """The fixed shoulder mounts (2026-11-08) put a link between the chest and each shoulder
    pitch joint at the same place: the accepted tool points are where they were. A mount that
    moves the shoulder asks for a new check."""
    from sx_embodiments.kinematics import Kinematics

    previous = read_recorded(
        historical_document(
            EmbodimentName("sentient-static-upper-body"),
            EmbodimentId("51db79c80289f76c9f20f2043c114836dfcb80baca88e8f5094762c925a37402"),
        )
    ).embodiment
    body = _body("sentient-static-upper-body")
    record = tool_points_checked_of("sentient-static-upper-body")
    assert record is not None
    kinematics = Kinematics.of(body.urdf_bytes)
    assert kinematics.joints["left_shoulder_pitch_joint"].parent == "left_shoulder_mount_link"
    assert Kinematics.of(previous.urdf_bytes).joints["left_shoulder_pitch_joint"].parent == (
        "chest_link"
    )
    grips = tool_grips(body, record.points)
    assert tool_chain(previous.urdf_bytes, record.points, grips) == record.chain
    assert tool_chain(body.urdf_bytes, record.points, grips) == record.chain
    moved = _edited(body.urdf_bytes, "left_shoulder_mount_joint", xyz="-0.059 0.231 0.0131")
    assert tool_chain(moved, record.points, grips) != record.chain
