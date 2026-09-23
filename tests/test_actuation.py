"""Independent observation and actuation facts in schema 14."""

import copy

import pytest
from sx_contracts.identity import content_id

from sx_embodiments import (
    ActuatorBinding,
    ActuatorBus,
    ActuatorFeedback,
    ActuatorModel,
    CompositionError,
    Embodiment,
    EmbodimentSchemaError,
    EncoderReadout,
    IntegratedDrive,
    LayoutError,
    Passive,
    UndocumentedDrive,
    Unobserved,
    convert_v13_to_v14,
    development_embodiments,
    embodiments,
)
from sx_embodiments.identity import EmbodimentId
from sx_embodiments.layout import (
    UNDOCUMENTED_DRIVE_REASON,
    UNDOCUMENTED_OBSERVATION_REASON,
    Bounds,
    CoordinateUnit,
    JointAxis,
    JointLayout,
)


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
    assert axis["observation"] == {"kind": "actuator_feedback"}
    assert axis["actuation"] == {
        "kind": "direct",
        "actuator": {
            "model": "feetech_sts3215",
            "bus": "feetech_serial",
            "bus_id": 1,
            "sign": 1,
            "zero_offset": 0.0,
            "reduction": 1.0,
        },
    }


def test_unqualified_actuator_model_fails_closed_naming_the_vocabulary() -> None:
    wire = copy.deepcopy(embodiments["so101"].to_dict())
    components = wire["components"]
    assert isinstance(components, list)
    components[0]["attachment"]["part"]["layout"][0]["actuation"]["actuator"]["model"] = (
        "acme_servo_9000"
    )
    with pytest.raises(EmbodimentSchemaError, match=r"unqualified actuator.*feetech_sts3215"):
        Embodiment.from_dict(wire)


def test_unknown_bus_fails_closed() -> None:
    wire = copy.deepcopy(embodiments["so101"].to_dict())
    components = wire["components"]
    assert isinstance(components, list)
    components[0]["attachment"]["part"]["layout"][0]["actuation"]["actuator"]["bus"] = "acme_bus"
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


def test_known_hardware_access_facts_are_explicit_without_promotion() -> None:
    so101 = embodiments["so101"]
    assert all(isinstance(c.axis.observation, ActuatorFeedback) for c in so101.state.coordinates)

    b601 = embodiments["b601-dm"]
    assert all(
        isinstance(c.axis.actuation, IntegratedDrive)
        for c in b601.state.coordinates
        if c.instance == "arm"
    )
    jaw = next(c.axis for c in b601.state.coordinates if c.instance == "gripper")
    assert isinstance(jaw.observation, Unobserved)
    assert isinstance(jaw.actuation, UndocumentedDrive)

    yubi = development_embodiments["yubi"]
    assert all(isinstance(c.axis.observation, EncoderReadout) for c in yubi.state.coordinates)
    assert all(isinstance(c.axis.actuation, Passive) for c in yubi.state.coordinates)


def test_schema_13_converter_preserves_vectors_and_records_ambiguity() -> None:
    current = embodiments["so101"].to_dict()
    legacy = copy.deepcopy(current)
    legacy["schema_version"] = 13
    components = legacy["components"]
    assert isinstance(components, list)
    for component in components:
        for axis in component["attachment"]["part"].get("layout", []):
            actuation = axis.pop("actuation")
            axis.pop("observation")
            axis["actuator"] = actuation.get("actuator")
    legacy["id"] = str(content_id(EmbodimentId, {k: v for k, v in legacy.items() if k != "id"}))

    migration = convert_v13_to_v14(legacy)
    converted = migration.embodiment
    assert str(migration.source_id) == legacy["id"]
    assert migration.target_id == converted.id
    assert converted.state.names == embodiments["so101"].state.names
    assert converted.state.width == embodiments["so101"].state.width
    assert all(
        isinstance(item.axis.observation, Unobserved) for item in converted.state.coordinates
    )
    assert all(item.axis.actuator is not None for item in converted.state.coordinates)


def test_schema_13_converter_rejects_a_forged_source_identity() -> None:
    legacy = copy.deepcopy(embodiments["so101"].to_dict())
    legacy["schema_version"] = 13
    for component in legacy["components"]:
        for axis in component["attachment"]["part"].get("layout", []):
            actuation = axis.pop("actuation")
            axis.pop("observation")
            axis["actuator"] = actuation.get("actuator")
    legacy["id"] = "0" * 64

    with pytest.raises(EmbodimentSchemaError, match="id does not match"):
        convert_v13_to_v14(legacy)


def test_schema_13_converter_wraps_malformed_nested_layout_as_schema_error() -> None:
    legacy = copy.deepcopy(embodiments["so101"].to_dict())
    legacy["schema_version"] = 13
    for component in legacy["components"]:
        for axis in component["attachment"]["part"].get("layout", []):
            actuation = axis.pop("actuation")
            axis.pop("observation")
            axis["actuator"] = actuation.get("actuator")
    legacy["components"][0]["attachment"]["part"]["layout"] = "not-an-array"
    legacy["id"] = str(content_id(EmbodimentId, {k: v for k, v in legacy.items() if k != "id"}))
    with pytest.raises(EmbodimentSchemaError, match="expected an array"):
        convert_v13_to_v14(legacy)


def test_schema_13_converter_wraps_malformed_actuator_as_schema_error() -> None:
    legacy = copy.deepcopy(embodiments["so101"].to_dict())
    legacy["schema_version"] = 13
    for component in legacy["components"]:
        for axis in component["attachment"]["part"].get("layout", []):
            actuation = axis.pop("actuation")
            axis.pop("observation")
            axis["actuator"] = actuation.get("actuator")
    legacy["components"][0]["attachment"]["part"]["layout"][0]["actuator"] = {
        "model": "feetech_sts3215"
    }
    legacy["id"] = str(content_id(EmbodimentId, {k: v for k, v in legacy.items() if k != "id"}))

    with pytest.raises(EmbodimentSchemaError, match="expected exactly these fields"):
        convert_v13_to_v14(legacy)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("bus_id", 0, "bus_id must be a positive integer"),
        ("sign", 2, r"sign must be \+1 or -1"),
        ("reduction", 0.0, "reduction must be a positive finite ratio"),
    ],
)
def test_schema_13_converter_wraps_semantically_invalid_actuator_as_schema_error(
    field: str, value: float, message: str
) -> None:
    legacy = copy.deepcopy(embodiments["so101"].to_dict())
    legacy["schema_version"] = 13
    for component in legacy["components"]:
        for axis in component["attachment"]["part"].get("layout", []):
            actuation = axis.pop("actuation")
            axis.pop("observation")
            axis["actuator"] = actuation.get("actuator")
    legacy["components"][0]["attachment"]["part"]["layout"][0]["actuator"][field] = value
    legacy["id"] = str(content_id(EmbodimentId, {k: v for k, v in legacy.items() if k != "id"}))

    with pytest.raises(EmbodimentSchemaError, match=message) as caught:
        convert_v13_to_v14(legacy)

    assert isinstance(caught.value.__cause__, LayoutError)


def _schema_13(current: dict[str, object], *, keep_actuators: bool) -> dict[str, object]:
    legacy = copy.deepcopy(current)
    legacy["schema_version"] = 13
    components = legacy["components"]
    assert isinstance(components, list)
    for component in components:
        for axis in component["attachment"]["part"].get("layout", []):
            actuation = axis.pop("actuation")
            axis.pop("observation")
            axis["actuator"] = actuation.get("actuator") if keep_actuators else None
    legacy["id"] = str(content_id(EmbodimentId, {k: v for k, v in legacy.items() if k != "id"}))
    return legacy


def test_schema_13_unactuated_body_converts_to_the_body_authored_today() -> None:
    """A schema-13 axis with no actuator is the absence today's layout records by default.

    Recorded history and catalog rows minted at schema 13 stay readable only if the
    converter says that absence in the same words: otherwise the converted body is a
    different identity from the registry's, and v14 validation refuses its actions.
    """
    body = embodiments["piper"]
    assert all(
        isinstance(item.axis.actuation, UndocumentedDrive)
        and item.axis.actuation.reason == UNDOCUMENTED_DRIVE_REASON
        and isinstance(item.axis.observation, Unobserved)
        and item.axis.observation.reason == UNDOCUMENTED_OBSERVATION_REASON
        for item in body.state.coordinates
    ), "the fixture must be a body with no documented drive or observation facts"
    legacy = _schema_13(body.to_dict(), keep_actuators=False)

    migration = convert_v13_to_v14(legacy)

    assert str(migration.source_id) == legacy["id"]
    assert migration.embodiment.id == body.id
    assert migration.embodiment.to_dict() == body.to_dict()
