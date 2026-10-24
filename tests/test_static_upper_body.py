"""The static upper body's description is its generator's rendering of the received file."""

import importlib.util
import math
from pathlib import Path

import pytest

from sx_embodiments.compose import MountKind
from sx_embodiments.embodiment import embodiment_from_definition
from sx_embodiments.known import DEVELOPMENT_EMBODIMENTS
from sx_embodiments.known.static_upper_body import (
    STATIC_UPPER_BODY_LEFT_ARM,
    STATIC_UPPER_BODY_RIGHT_ARM,
    STATIC_UPPER_BODY_SPEC,
)
from sx_embodiments.layout import Bounds

ROOT = Path(__file__).resolve().parents[1]


def test_the_description_is_the_generators_rendering() -> None:
    pytest.importorskip("scipy")
    spec = importlib.util.spec_from_file_location(
        "compose_static_upper_body_urdf", ROOT / "tools/compose_static_upper_body_urdf.py"
    )
    assert spec is not None and spec.loader is not None
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    for path, data in tool.render().items():
        assert path.read_bytes() == data, path.relative_to(ROOT)


def test_the_body_is_a_table_clamped_development_body() -> None:
    entry = DEVELOPMENT_EMBODIMENTS[STATIC_UPPER_BODY_SPEC.name]
    assert STATIC_UPPER_BODY_SPEC.base_mount is not None
    assert STATIC_UPPER_BODY_SPEC.base_mount.kind is MountKind.CLAMP_EDGE
    robot = embodiment_from_definition(entry.spec)
    assert len(robot.joint_names) == 25
    # Each elbow stops at the straight arm on one side, never past it.
    for arm, straight in (
        (STATIC_UPPER_BODY_LEFT_ARM, -math.pi / 2),
        (STATIC_UPPER_BODY_RIGHT_ARM, math.pi / 2),
    ):
        (elbow,) = (axis for axis in arm.layout.axes if axis.name.endswith("_elbow_pitch_joint"))
        assert isinstance(elbow.bounds, Bounds)
        ends = (elbow.bounds.lower, elbow.bounds.upper)
        assert min(abs(end - straight) for end in ends) < 1e-9


def test_the_description_materializes_with_its_meshes() -> None:
    """Every attached URDF's references resolve, so MuJoCo and URDF loaders can read it."""
    from sx_embodiments.materialize import materialized

    robot = embodiment_from_definition(DEVELOPMENT_EMBODIMENTS[STATIC_UPPER_BODY_SPEC.name].spec)
    path = materialized(robot)
    assert path.name == "sentient_static_upper_body.urdf"
    assert (path.parent / "meshes/collision/base_link.stl").is_file()
