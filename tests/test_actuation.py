"""Actuation facts: the qualified-motor vocabulary and per-axis bindings (schema 13)."""

import copy

import pytest

from sx_embodiments import (
    ActuatorBinding,
    ActuatorBus,
    ActuatorModel,
    CompositionError,
    Embodiment,
    EmbodimentSchemaError,
    LayoutError,
    embodiments,
)
from sx_embodiments.layout import Bounds, CoordinateUnit, JointAxis, JointLayout


def _sts(bus_id: int) -> ActuatorBinding:
    return ActuatorBinding(ActuatorModel.FEETECH_STS3215, ActuatorBus.FEETECH_SERIAL, bus_id)


def test_so101_axes_carry_qualified_feetech_bindings() -> None:
    robot = embodiments["so101"]
    bindings = tuple(coordinate.axis.actuator for coordinate in robot.state.coordinates)
    assert all(binding is not None for binding in bindings)
    assert tuple(binding.bus_id for binding in bindings if binding is not None) == (
        1,
        2,
        3,
        4,
        5,
        6,
    )
    for binding in bindings:
        assert binding is not None
        assert binding.model is ActuatorModel.FEETECH_STS3215
        assert binding.bus is ActuatorBus.FEETECH_SERIAL
        assert binding.sign == 1
        assert binding.zero_offset == 0.0
        assert binding.reduction == 1.0


def test_bimanual_sides_reuse_per_chain_addresses() -> None:
    robot = embodiments["bimanual-so101"]
    addresses = [
        coordinate.axis.actuator.bus_id
        for coordinate in robot.state.coordinates
        if coordinate.axis.actuator is not None
    ]
    # Each side is its own serial adapter, so the id space restarts per chain.
    assert addresses == [1, 2, 3, 4, 5, 6, 1, 2, 3, 4, 5, 6]


def test_binding_round_trips_through_the_wire_document() -> None:
    robot = embodiments["so101"]
    assert Embodiment.from_json(robot.to_json()) == robot
    wire = robot.to_dict()
    components = wire["components"]
    assert isinstance(components, list)
    part = components[0]["attachment"]["part"]
    axis = part["layout"][0]
    assert axis["actuator"] == {
        "model": "feetech_sts3215",
        "bus": "feetech_serial",
        "bus_id": 1,
        "sign": 1,
        "zero_offset": 0.0,
        "reduction": 1.0,
    }


def test_unqualified_actuator_model_fails_closed_naming_the_vocabulary() -> None:
    wire = copy.deepcopy(embodiments["so101"].to_dict())
    components = wire["components"]
    assert isinstance(components, list)
    components[0]["attachment"]["part"]["layout"][0]["actuator"]["model"] = "acme_servo_9000"
    with pytest.raises(EmbodimentSchemaError, match=r"unqualified actuator.*feetech_sts3215"):
        Embodiment.from_dict(wire)


def test_unknown_bus_fails_closed() -> None:
    wire = copy.deepcopy(embodiments["so101"].to_dict())
    components = wire["components"]
    assert isinstance(components, list)
    components[0]["attachment"]["part"]["layout"][0]["actuator"]["bus"] = "acme_bus"
    with pytest.raises(EmbodimentSchemaError, match="bus is unknown"):
        Embodiment.from_dict(wire)


def test_duplicate_bus_addresses_within_one_layout_are_rejected() -> None:
    axis = JointAxis("a", CoordinateUnit.RADIAN, Bounds(-1.0, 1.0), _sts(1))
    other = JointAxis("b", CoordinateUnit.RADIAN, Bounds(-1.0, 1.0), _sts(1))
    with pytest.raises(LayoutError, match="bus addresses must be unique"):
        JointLayout((axis, other))


def test_duplicate_bus_addresses_within_one_mounted_chain_are_an_authoring_error() -> None:
    # An arm and the jaw mounted on it share one physical daisy chain, so a jaw
    # reusing an arm address must fail at authoring — not at connect time.
    from sx_embodiments.compose import MountedOn, RootMount, body_component, validate_components
    from sx_embodiments.identity import EmbodimentKind, PartId
    from sx_embodiments.parts import ArmSpec, GripperSpec

    arm = ArmSpec(
        part_id=PartId("test-arm"),
        layout=JointLayout(
            (
                JointAxis("a", CoordinateUnit.RADIAN, Bounds(-1.0, 1.0), _sts(1)),
                JointAxis("b", CoordinateUnit.RADIAN, Bounds(-1.0, 1.0), _sts(2)),
            )
        ),
        home=(0.0, 0.0),
    )
    jaw = GripperSpec(
        part_id=PartId("test-jaw"),
        layout=JointLayout((JointAxis("g", CoordinateUnit.RADIAN, Bounds(-1.0, 1.0), _sts(2)),)),
    )
    colliding = (
        body_component("arm", arm, RootMount("base")),
        body_component("jaw", jaw, MountedOn("arm", "tool")),
    )
    with pytest.raises(CompositionError, match="drives both 'arm/b' and 'jaw/g'"):
        validate_components("test-body", EmbodimentKind.ROBOT, colliding)
    # The same addresses on separately rooted assemblies are two chains and legal.
    separate = (
        body_component("left", arm, RootMount("left_base")),
        body_component("right", arm, RootMount("right_base")),
    )
    validate_components("test-body", EmbodimentKind.ROBOT, separate)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("bus_id", 0, "bus_id must be a positive integer"),
        ("sign", 2, r"sign must be \+1 or -1"),
        ("zero_offset", float("nan"), "zero_offset must be finite"),
        ("reduction", 0.0, "reduction must be a positive finite ratio"),
    ],
)
def test_malformed_bindings_are_typed_authoring_errors(
    field: str, value: float, message: str
) -> None:
    values: dict[str, object] = {
        "model": ActuatorModel.FEETECH_STS3215,
        "bus": ActuatorBus.FEETECH_SERIAL,
        "bus_id": 1,
        field: value,
    }
    with pytest.raises(LayoutError, match=message):
        ActuatorBinding(**values)  # type: ignore[arg-type]
