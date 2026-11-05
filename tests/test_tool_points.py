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

BODIES = sorted({*map(str, embodiments), *map(str, development_embodiments)})
CHECKED = {"sentient-static-upper-body", "yubi"}


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
        ToolPointCheck(checked_by="", checked_on=dt.date(2026, 11, 5), method="m", evidence="e")
