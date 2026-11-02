"""Every tool of every body in the registry says how it grips, by one rule from its description.

No hand list: the bodies come from both registries and the tools from each body's own
components (every gripper, and every arm carrying none). A tool with jaws has a unit closing
line normal to its approach, signed by the one convention, and parallel jaws' line is the line
between their fingertips with the jaws open; a tool without says why.
"""

import math

import pytest

from sx_embodiments import development_embodiments, embodiments
from sx_embodiments.parts import GripperSpec, Limb

pytest.importorskip("numpy")
pytest.importorskip("scipy")

from sx_embodiments.tool_frames import (  # needs the render extra
    JawKind,
    Jaws,
    NoJaws,
    NoJawsReason,
    fingertip_line,
    tool_frames,
)

BODIES = {
    **dict.fromkeys(embodiments, embodiments),
    **dict.fromkeys(development_embodiments, development_embodiments),
}
#: How far, in degrees, parallel jaws' closing line may stand from their fingertip line.
FINGERTIP_DEGREES = 1.0


def _dot(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    return sum(x * y for x, y in zip(a, b, strict=True))


@pytest.mark.parametrize("name", list(BODIES))
def test_every_tool_of_every_body_says_how_it_grips(name: str) -> None:
    body = BODIES[name][name]
    tools = tool_frames(body)
    grippers = [c.instance for c in body.components if isinstance(c.part, GripperSpec)]
    assert set(grippers) <= {tool.component for tool in tools}
    arms = [c.instance for c in body.components if c.limb is Limb.ARM]
    carried = {getattr(c.mount, "parent", None) for c in body.components if c.instance in grippers}
    assert {arm for arm in arms if arm not in carried} <= {tool.component for tool in tools}
    for tool in tools:
        match tool.grip:
            case Jaws(kind, closing, approach, point):
                assert math.isclose(math.sqrt(_dot(closing, closing)), 1.0, abs_tol=1e-9)
                assert math.isclose(math.sqrt(_dot(approach, approach)), 1.0, abs_tol=1e-9)
                assert abs(_dot(closing, approach)) < 1e-9, tool
                assert _dot(point, approach) > 0, "the grasp point lies ahead of the mount"
                if kind is JawKind.PARALLEL:
                    largest = max(closing, key=abs)
                    assert largest > 0, "parallel jaws' line has its largest component positive"
                    line = fingertip_line(body, tool)
                    assert line is not None
                    angle = math.degrees(math.acos(min(1.0, abs(_dot(line, closing)))))
                    assert angle <= FINGERTIP_DEGREES, (tool, line, angle)
            case NoJaws(reason, detail):
                assert isinstance(reason, NoJawsReason) and detail.strip(), tool


def test_the_rule_is_deterministic() -> None:
    body = development_embodiments["sentient-static-upper-body"]
    assert tool_frames(body) == tool_frames(body)


def test_the_static_upper_body_grips_across_its_fingers() -> None:
    """Its revolute fingers hinge about their -z: the line runs across them, +y, and the
    approach out along +x, the solve's own approach to within a few degrees."""
    body = development_embodiments["sentient-static-upper-body"]
    tools = {tool.component: tool for tool in tool_frames(body)}
    for side in ("left", "right"):
        tool = tools[f"{side}_gripper"]
        assert tool.frame == f"{side}_gripper_base_link"
        assert isinstance(tool.grip, Jaws) and tool.grip.kind is JawKind.PARALLEL
        assert _dot(tool.grip.closing_axis, (0.0, 1.0, 0.0)) > math.cos(math.radians(1.0))
        assert _dot(tool.grip.approach_axis, (1.0, 0.0, 0.0)) > math.cos(math.radians(5.0))


def test_a_single_moving_jaw_runs_from_the_fixed_jaw() -> None:
    """The SO-101's jaw swings against the fixed one: the line points the way it opens."""
    import numpy as np

    from sx_embodiments.tool_frames import _jaw  # pyright: ignore[reportPrivateUsage]

    body = embodiments["so101"]
    tool = next(t for t in tool_frames(body) if t.component == "jaw")
    assert isinstance(tool.grip, Jaws) and tool.grip.kind is JawKind.SINGLE_MOVING
    component = next(c for c in body.components if c.instance == "jaw")
    assert isinstance(component.part, GripperSpec)
    from sx_embodiments.collision_proxies import (
        _collision_meshes,  # pyright: ignore[reportPrivateUsage]
    )
    from sx_embodiments.kinematics import Kinematics

    resolved = dict(zip(body.state.coordinates, body.joint_names, strict=True))
    jaw = _jaw(
        component,
        component.part,
        Kinematics.of(body.urdf_bytes),
        {(c.instance, c.joint_name): name for c, name in resolved.items()},
        {c.instance: c.mount.frame for c in body.components},
        _collision_meshes(body, body.urdf_bytes),
    )
    assert not isinstance(jaw, NoJaws)
    (link, closed), (_, opened) = (
        jaw.tip(jaw.sides[0], jaw.closed),
        jaw.tip(jaw.sides[0], jaw.opened),
    )
    assert link and float(np.dot(opened - closed, tool.grip.closing_axis)) > 0
