"""The static upper body is its generator's rendering of the received file and its motor table."""

import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from sx_embodiments.compose import MountKind
from sx_embodiments.embodiment import embodiment_from_definition
from sx_embodiments.known import DEVELOPMENT_EMBODIMENTS
from sx_embodiments.known.static_upper_body import (
    STATIC_UPPER_BODY_LEFT_ARM,
    STATIC_UPPER_BODY_MASTER_ID_OFFSET,
    STATIC_UPPER_BODY_RIGHT_ARM,
    STATIC_UPPER_BODY_SPEC,
    STATIC_UPPER_BODY_TORSO,
)
from sx_embodiments.layout import Bounds, DirectDrive

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


def test_the_bindings_are_the_received_motor_table() -> None:
    source = ET.parse(ROOT / "assets/sentient_static_upper_body/source/received.urdf").getroot()
    table = {
        values["name"]: values
        for values in (
            {p.attrib["name"]: p.attrib["value"] for p in motor}
            for motor in source.iter("motor_parameters")
        )
    }
    axes = [
        axis
        for chain in (
            STATIC_UPPER_BODY_TORSO,
            STATIC_UPPER_BODY_LEFT_ARM,
            STATIC_UPPER_BODY_RIGHT_ARM,
        )
        for axis in chain.layout.axes
    ]
    assert sorted(axis.name for axis in axes) == sorted(table)
    for axis in axes:
        motor = table[axis.name]
        assert isinstance(axis.actuation, DirectDrive) and isinstance(axis.bounds, Bounds)
        binding = axis.actuation.actuator
        assert binding.model.value == f"damiao_{motor['model'].lower()}", axis.name
        assert binding.bus_id == int(motor["send_id"], 16), axis.name
        assert int(motor["recv_id"], 16) == binding.bus_id + STATIC_UPPER_BODY_MASTER_ID_OFFSET
        assert (binding.sign, motor["encoder_turns"]) == (1, "0"), axis.name
        assert motor["direction_confirmed"] == motor["limits_confirmed"] == "true", axis.name
        assert axis.bounds == Bounds(float(motor["lower"]), float(motor["upper"])), axis.name


def test_the_neck_is_bound_provisionally_as_an_so101_on_its_own_port() -> None:
    from sx_embodiments.known.static_upper_body import (
        STATIC_UPPER_BODY_NECK,
        STATIC_UPPER_BODY_NECK_PORT,
    )
    from sx_embodiments.layout import ActuatorBus, ActuatorModel

    bindings = [axis.actuator for axis in STATIC_UPPER_BODY_NECK.layout.axes]
    assert [binding.bus_id for binding in bindings if binding is not None] == [1, 2, 3, 4, 5, 6]
    for binding in bindings:
        assert binding is not None
        assert binding.model is ActuatorModel.FEETECH_STS3215
        assert binding.bus is ActuatorBus.FEETECH_SERIAL
        assert binding.bus_port == STATIC_UPPER_BODY_NECK_PORT == "neck"
    robot = embodiment_from_definition(DEVELOPMENT_EMBODIMENTS[STATIC_UPPER_BODY_SPEC.name].spec)
    assert all(c.axis.actuator is not None for c in robot.state.coordinates)


def test_each_gripper_is_two_feetech_servos_on_a_bus_of_its_own() -> None:
    from sx_embodiments.known.static_upper_body import (
        STATIC_UPPER_BODY_LEFT_GRIPPER,
        STATIC_UPPER_BODY_RIGHT_GRIPPER,
    )
    from sx_embodiments.layout import ActuatorModel

    for gripper, port, ids in (
        (STATIC_UPPER_BODY_RIGHT_GRIPPER, "right_gripper", [1, 2]),
        (STATIC_UPPER_BODY_LEFT_GRIPPER, "left_gripper", [3, 4]),
    ):
        bindings = [axis.actuator for axis in gripper.layout.axes]
        assert [b.bus_id for b in bindings if b is not None] == ids
        assert all(
            b is not None and b.model is ActuatorModel.FEETECH_STS3215 and b.bus_port == port
            for b in bindings
        )
