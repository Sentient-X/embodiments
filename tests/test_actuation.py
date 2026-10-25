"""Independent observation and actuation facts, and the converters from earlier schemas."""

import copy

import pytest
from sx_contracts.identity import content_id

from sx_embodiments import (
    ActuatorBinding,
    ActuatorBus,
    ActuatorFeedback,
    ActuatorModel,
    ChannelKind,
    CompositionError,
    Embodiment,
    EmbodimentSchemaError,
    EncoderReadout,
    LayoutError,
    Limb,
    Passive,
    UndocumentedDrive,
    Unobserved,
    VendorReadout,
    development_embodiments,
    embodiments,
    migrate,
)
from sx_embodiments.identity import EmbodimentId
from sx_embodiments.layout import (
    UNDOCUMENTED_DRIVE_REASON,
    UNDOCUMENTED_OBSERVATION_REASON,
    Bounds,
    BusPort,
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
        assert binding.reduction == 1.0


def test_b601_axes_carry_the_vendor_damiao_bindings() -> None:
    robot = embodiments["b601-dm"]
    bindings = tuple(coordinate.axis.actuator for coordinate in robot.state.coordinates)
    expected = (
        *((ActuatorModel.DAMIAO_DM4340, bus_id) for bus_id in (1, 2, 3)),
        *((ActuatorModel.DAMIAO_DM4310, bus_id) for bus_id in (4, 5, 6, 7)),
    )
    assert (
        tuple((binding.model, binding.bus_id) for binding in bindings if binding is not None)
        == expected
    )
    for binding in bindings[:6]:
        assert binding is not None
        assert binding.bus is ActuatorBus.DAMIAO_CAN
        assert (binding.sign, binding.reduction) == (1, 1.0)
    jaw = bindings[6]
    assert jaw is not None
    assert jaw.bus is ActuatorBus.DAMIAO_CAN
    # Seeed's controller: finger = motor / -5.0 rad * 0.045 m, so motor = -joint * 5.0 / 0.045.
    assert jaw.sign == -1
    assert jaw.reduction == pytest.approx(5.0 / 0.045)


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


def test_a_named_port_is_written_and_read_and_the_default_is_never_spelled() -> None:
    robot = development_embodiments["sentient-static-upper-body"]
    assert Embodiment.from_json(robot.to_json()) == robot
    ports = {
        coordinate.axis.name: binding.bus_port
        for coordinate in robot.state.coordinates
        if (binding := coordinate.axis.actuator) is not None
    }
    assert ports["torso_joint"] == "can"
    assert ports["so101_camera_tilt_joint"] == "neck"
    # An unnamed port is not on the wire, so a body naming none keeps its identity.
    assert '"port"' not in embodiments["so101"].to_json()
    assert '"neck"' in robot.to_json()
    with pytest.raises(LayoutError, match="default for its bus"):
        ActuatorBinding(ActuatorModel.DAMIAO_DM4310, ActuatorBus.DAMIAO_CAN, 1, port=BusPort("can"))
    for spelled in ("", "Neck", "neck bus", "1neck"):
        with pytest.raises(LayoutError, match="port must be"):
            ActuatorBinding(
                ActuatorModel.FEETECH_STS3215, ActuatorBus.FEETECH_SERIAL, 1, port=BusPort(spelled)
            )


def test_one_address_may_recur_on_two_ports_of_one_layout() -> None:
    def neck(bus_id: int, port: str) -> ActuatorBinding:
        return ActuatorBinding(
            ActuatorModel.FEETECH_STS3215, ActuatorBus.FEETECH_SERIAL, bus_id, port=BusPort(port)
        )

    a = JointAxis("a", CoordinateUnit.RADIAN, Bounds(-1.0, 1.0), neck(1, "left"))
    b = JointAxis("b", CoordinateUnit.RADIAN, Bounds(-1.0, 1.0), neck(1, "right"))
    assert JointLayout((a, b)).width == 2


def test_duplicate_bus_addresses_within_one_mounted_chain_are_an_authoring_error() -> None:
    # An arm and the jaw mounted on it share one physical daisy chain, so a jaw
    # reusing an arm address must fail at authoring — not at connect time.
    from sx_embodiments.compose import MountedOn, RootMount, body_component, validate_components
    from sx_embodiments.identity import EmbodimentKind, PartId
    from sx_embodiments.parts import ChainSpec, GripperSpec, Limb

    arm = ChainSpec(
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
        body_component("arm", arm, RootMount("base"), limb=Limb.ARM),
        body_component("jaw", jaw, MountedOn("arm", "tool")),
    )
    with pytest.raises(CompositionError, match="drives both 'arm/b' and 'jaw/g'"):
        validate_components("test-body", EmbodimentKind.ROBOT, colliding)
    # The same addresses on separately rooted assemblies are two chains and legal.
    separate = (
        body_component("left", arm, RootMount("left_base"), limb=Limb.ARM),
        body_component("right", arm, RootMount("right_base"), limb=Limb.ARM),
    )
    validate_components("test-body", EmbodimentKind.ROBOT, separate)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("bus_id", 0, "bus_id must be a positive integer"),
        ("sign", 2, r"sign must be \+1 or -1"),
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
    assert all(isinstance(c.axis.observation, ActuatorFeedback) for c in b601.state.coordinates)

    yubi = development_embodiments["yubi"]
    assert all(isinstance(c.axis.observation, EncoderReadout) for c in yubi.state.coordinates)
    assert all(isinstance(c.axis.actuation, Passive) for c in yubi.state.coordinates)


def _historical(name: str) -> dict[str, object]:
    """The exact schema-14 document the registry published for ``name``."""
    import json
    from importlib import resources

    documents = [
        json.loads(e.read_text())
        for e in (resources.files("sx_embodiments") / "historical").iterdir()
        if e.name.endswith(".json")
    ]
    (document,) = (d for d in documents if d["name"] == name and d["schema_version"] == 14)
    return document


def _reminted(document: dict[str, object]) -> dict[str, object]:
    document["id"] = str(
        content_id(EmbodimentId, {k: v for k, v in document.items() if k != "id"})  # type: ignore[arg-type]
    )
    return document


def _schema_13(v14: dict[str, object], *, keep_actuators: bool) -> dict[str, object]:
    legacy = copy.deepcopy(v14)
    legacy["schema_version"] = 13
    for component in legacy["components"]:  # type: ignore[union-attr]
        for axis in component["attachment"]["part"].get("layout", []):
            actuation = axis.pop("actuation")
            axis.pop("observation")
            axis["actuator"] = actuation.get("actuator") if keep_actuators else None
    return _reminted(legacy)


def test_an_unchanged_body_converts_to_the_body_authored_today() -> None:
    """Conversion re-mints nothing it did not change: SO-101 at 14 reads as SO-101 at 15."""
    v14 = _historical("so101")
    migration = migrate(v14)
    assert str(migration.source_id) == v14["id"]
    assert migration.source_version == 14
    assert migration.embodiment == embodiments["so101"]
    assert migration.target_id == embodiments["so101"].id


def test_schema_14_joint_groups_become_chains_with_an_undocumented_limb() -> None:
    migration = migrate(_historical("rby1"))
    limbs = {c.instance: c.limb for c in migration.embodiment.components}
    assert limbs["right_arm"] is Limb.ARM
    assert limbs["torso"] is Limb.UNDOCUMENTED and limbs["head"] is Limb.UNDOCUMENTED
    # What the limb derives is unchanged: a torso or head channel is a body joint, as before.
    kinds = {c.instance: c.kind for c in migration.embodiment.state.coordinates}
    assert kinds["torso"] is ChannelKind.BODY_JOINT and kinds["head"] is ChannelKind.BODY_JOINT
    assert kinds["right_arm"] is ChannelKind.ARM_JOINT


def test_a_design_zero_is_refused_not_dropped() -> None:
    v14 = copy.deepcopy(_historical("so101"))
    v14["components"][0]["attachment"]["part"]["layout"][0]["actuation"]["actuator"][  # type: ignore[index]
        "zero_offset"
    ] = 0.25
    with pytest.raises(EmbodimentSchemaError, match="joint calibration"):
        migrate(_reminted(v14))


def test_schema_13_reads_through_both_steps() -> None:
    v13 = _schema_13(_historical("so101"), keep_actuators=True)
    migration = migrate(v13)
    assert migration.source_version == 13
    assert str(migration.source_id) == v13["id"]
    converted = migration.embodiment
    assert converted.state.names == embodiments["so101"].state.names
    assert all(isinstance(c.axis.observation, Unobserved) for c in converted.state.coordinates)
    assert all(c.axis.actuator is not None for c in converted.state.coordinates)


def test_schema_13_unactuated_body_converts_to_the_body_authored_today() -> None:
    """A schema-13 axis with no actuator is the absence today's layout records by default."""
    body = embodiments["piper"]
    assert all(
        isinstance(item.axis.actuation, UndocumentedDrive)
        and item.axis.actuation.reason == UNDOCUMENTED_DRIVE_REASON
        and isinstance(item.axis.observation, Unobserved)
        and item.axis.observation.reason == UNDOCUMENTED_OBSERVATION_REASON
        for item in body.state.coordinates
    ), "the fixture must be a body with no documented drive or observation facts"
    migration = migrate(_schema_13(_historical("piper"), keep_actuators=False))
    assert migration.embodiment == body


def test_a_forged_source_identity_is_refused() -> None:
    v14 = copy.deepcopy(_historical("so101"))
    v14["id"] = "0" * 64
    with pytest.raises(EmbodimentSchemaError, match="id does not match"):
        migrate(v14)
    v13 = _schema_13(_historical("so101"), keep_actuators=True)
    v13["id"] = "0" * 64
    with pytest.raises(EmbodimentSchemaError, match="id does not match"):
        migrate(v13)


def test_the_current_schema_and_unknown_schemas_have_no_converter() -> None:
    with pytest.raises(EmbodimentSchemaError, match="no converter"):
        migrate(embodiments["so101"].to_dict())
    with pytest.raises(EmbodimentSchemaError, match="no converter"):
        migrate(_reminted({**_historical("so101"), "schema_version": 12}))


def test_malformed_legacy_layout_and_actuators_are_schema_errors() -> None:
    v13 = _schema_13(_historical("so101"), keep_actuators=True)
    v13["components"][0]["attachment"]["part"]["layout"] = "not-an-array"  # type: ignore[index]
    with pytest.raises(EmbodimentSchemaError, match="expected an array"):
        migrate(_reminted(v13))
    v13 = _schema_13(_historical("so101"), keep_actuators=True)
    v13["components"][0]["attachment"]["part"]["layout"][0]["actuator"] = {  # type: ignore[index]
        "model": "feetech_sts3215"
    }
    with pytest.raises(EmbodimentSchemaError, match="expected exactly these fields"):
        migrate(_reminted(v13))


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("bus_id", 0, "bus_id must be a positive integer"),
        ("sign", 2, r"sign must be \+1 or -1"),
        ("reduction", 0.0, "reduction must be a positive finite ratio"),
    ],
)
def test_semantically_invalid_legacy_actuators_are_schema_errors(
    field: str, value: float, message: str
) -> None:
    v13 = _schema_13(_historical("so101"), keep_actuators=True)
    v13["components"][0]["attachment"]["part"]["layout"][0]["actuator"][field] = value  # type: ignore[index]
    with pytest.raises(EmbodimentSchemaError, match=message) as caught:
        migrate(_reminted(v13))
    assert isinstance(caught.value.__cause__, LayoutError)


def test_stararm102_readout_does_not_claim_qualified_actuation() -> None:
    body = development_embodiments["stararm102-ld"]
    restored = Embodiment.from_json(body.to_json())
    for candidate in (body, restored):
        for address, coordinate in enumerate(candidate.state.coordinates):
            axis = coordinate.axis
            assert axis.observation == VendorReadout(
                f"fashionstar.uart.servo_monitor[{address}].angle_monitor"
            )
            assert isinstance(axis.actuation, UndocumentedDrive)
            assert "LD SDK unlocks" in axis.actuation.reason
            assert axis.actuator is None
