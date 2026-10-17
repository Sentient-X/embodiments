"""Three calibration kinds, one robot instance, and every body resolving through them."""

import json
from collections.abc import Callable
from datetime import UTC, datetime
from pathlib import Path
from typing import cast

import pytest
from sx_contracts import Sha256Digest

from sx_embodiments.calibration import (
    IDENTITY_TRANSFORM,
    UNCALIBRATED_ZERO,
    Calibration,
    CalibrationError,
    CalibrationMethod,
    CameraUnitCalibration,
    Evidence,
    EvidenceKind,
    JointUnitCalibration,
    Measurement,
    Residual,
    ResidualUnit,
    RigidCalibration,
    RigidReference,
    UnitSerial,
    calibration_from_dict,
    calibration_id,
    calibration_to_dict,
    matrix_from_pose,
)
from sx_embodiments.embodiment import Embodiment, embodiment_from_definition
from sx_embodiments.errors import EmbodimentSchemaError
from sx_embodiments.instance import (
    InstanceError,
    RobotInstance,
    Slot,
    SlotKind,
    description_links,
    nominal_instance,
    resolve,
    slots,
)
from sx_embodiments.known import definitions, embodiments

DRIVERS = Path(__file__).resolve().parents[1] / "drivers"

MEASURED = Measurement(
    CalibrationMethod.CHECKERBOARD,
    datetime(2026, 10, 17, 4, 0, tzinfo=UTC),
    Residual(0.4, ResidualUnit.PIXEL),
    (Evidence(EvidenceKind.IMAGE, "a" * 64),),
)


def _bodies() -> list[Embodiment]:
    bodies: list[Embodiment] = []
    for definition in definitions():
        try:
            bodies.append(embodiment_from_definition(definition))
        except EmbodimentSchemaError:
            continue
    return bodies


BODIES = _bodies()


def _camera(unit: str = "cam-1", width: int = 1280, height: int = 960) -> CameraUnitCalibration:
    return CameraUnitCalibration(
        unit=UnitSerial(unit),
        width=width,
        height=height,
        image_from_camera=(870.0, 0.0, 640.0, 0.0, 870.0, 480.0, 0.0, 0.0, 1.0),
        distortion_model="plumb_bob",
        distortion_coefficients=(0.01, -0.02, 0.0, 0.0, 0.001),
        measurement=MEASURED,
    )


def _joint(unit: str = "motor-1", zero: float = 0.031) -> JointUnitCalibration:
    return JointUnitCalibration(
        unit=UnitSerial(unit),
        zero_offset=zero,
        direction=1,
        range=(-1.9, 1.9),
        measurement=Measurement(CalibrationMethod.HOMING, None),
    )


def _rigid(parent: str, child: str, body: Embodiment) -> RigidCalibration:
    return RigidCalibration(
        parent_frame=parent,
        child_frame=child,
        parent_from_child=matrix_from_pose((0.001, 0.0, -0.002), (0.0, 0.0, 0.0, 1.0)),
        reference=RigidReference.DESCRIPTION,
        description_sha256=Sha256Digest(body.urdf.sha256),
        units=(),
        measurement=MEASURED,
    )


@pytest.mark.parametrize("body", BODIES, ids=lambda body: str(body.name))
def test_every_body_resolves_to_its_nominal_design_labelled_nominal(body: Embodiment) -> None:
    """With nothing measured, every slot falls back to the design and says so."""
    resolved = resolve(body, nominal_instance(body), ())
    assert [camera.name for camera in resolved.cameras] == [c.name for c in body.cameras]
    for camera in resolved.cameras:
        assert camera.measured is None and camera.calibration is None
    for joint in resolved.joints:
        assert joint.measured is None
        assert (joint.zero_offset, joint.direction) == (UNCALIBRATED_ZERO, 1)
    assert {slot for slot in slots(body) if slot.kind is SlotKind.COMPONENT} == {
        Slot(SlotKind.COMPONENT, component.instance)
        for component in body.components
        if component.instance not in {camera.name for camera in body.cameras}
    }


@pytest.mark.parametrize("body", BODIES, ids=lambda body: str(body.name))
def test_every_driven_or_encoder_read_coordinate_is_a_slot(body: Embodiment) -> None:
    names = {slot.name for slot in slots(body) if slot.kind is not SlotKind.CAMERA}
    for coordinate in body.state.coordinates:
        if coordinate.axis.actuator is not None:
            assert f"{coordinate.instance}/{coordinate.axis.name}" in names


def test_the_nominal_zero_is_the_zero_every_driver_transcript_was_minted_with() -> None:
    """The Rust drivers and their transcripts use the uncalibrated zero this module names."""
    minters = sorted(DRIVERS.glob("*/tests/fixtures/*/mint.py"))
    assert minters
    for minter in minters:
        assert f"UNCALIBRATED_ZERO = {UNCALIBRATED_ZERO!r}" in minter.read_text(), minter
    binding = (DRIVERS / "sx-embodiment-drivers/src/binding.rs").read_text()
    assert f"pub const UNCALIBRATED_ZERO: f64 = {UNCALIBRATED_ZERO!r};" in binding
    assert "pub const UNCALIBRATED_DIRECTION: i8 = 1;" in binding
    assert "pub fn calibrated(self, zero: f64, direction: i8)" in binding


@pytest.mark.parametrize(
    "calibration",
    [
        _camera(),
        _joint(),
        RigidCalibration(
            "base_link",
            "camera_link",
            IDENTITY_TRANSFORM,
            RigidReference.ABSOLUTE,
            None,
            (UnitSerial("arm-7"), UnitSerial("cam-1")),
            MEASURED,
        ),
    ],
    ids=["camera_unit", "joint_unit", "rigid"],
)
def test_each_kind_round_trips_through_its_one_document(calibration: Calibration) -> None:
    document = json.loads(json.dumps(calibration_to_dict(calibration)))
    assert calibration_from_dict(document) == calibration
    assert calibration_id(calibration_from_dict(document)) == calibration_id(calibration)


def test_an_unknown_field_or_kind_is_refused() -> None:
    """A field the reader would drop is one the id does not cover, so it is refused."""
    document = calibration_to_dict(_joint())
    with pytest.raises(CalibrationError):
        calibration_from_dict({**document, "sign": 1})
    measurement = cast("dict[str, object]", document["measurement"])
    with pytest.raises(CalibrationError):
        calibration_from_dict({**document, "measurement": {**measurement, "operator": "x"}})
    camera = calibration_to_dict(_camera())
    measured = cast("dict[str, object]", camera["measurement"])
    residual = {**cast("dict[str, object]", measured["residual"]), "std": 0.1}
    with pytest.raises(CalibrationError):
        calibration_from_dict({**camera, "measurement": {**measured, "residual": residual}})
    evidence = [{"kind": "image", "reference": "a" * 64, "note": "x"}]
    with pytest.raises(CalibrationError):
        calibration_from_dict({**camera, "measurement": {**measured, "evidence": evidence}})
    with pytest.raises(CalibrationError):
        calibration_from_dict({**document, "kind": "hand_eye"})
    with pytest.raises(CalibrationError):
        calibration_from_dict({**document, "schema": "sx.calibration/v0"})


@pytest.mark.parametrize(
    "build",
    [
        lambda: _camera(width=0),
        lambda: CameraUnitCalibration(UnitSerial("c"), 640, 480, (1.0,) * 9, "none", (), MEASURED),
        lambda: CameraUnitCalibration(
            UnitSerial("c"),
            640,
            480,
            (500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0),
            "equidistant",
            (0.1,),
            MEASURED,
        ),
        lambda: RigidCalibration(
            "a", "b", (2.0, *IDENTITY_TRANSFORM[1:]), RigidReference.ABSOLUTE, None, (), MEASURED
        ),
        lambda: RigidCalibration(
            "a", "b", IDENTITY_TRANSFORM, RigidReference.DESCRIPTION, None, (), MEASURED
        ),
        lambda: RigidCalibration(
            "a",
            "a",
            matrix_from_pose((0.1, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            RigidReference.ABSOLUTE,
            None,
            (),
            MEASURED,
        ),
        lambda: RigidCalibration(
            "a",
            "a",
            matrix_from_pose((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            RigidReference.DESCRIPTION,
            Sha256Digest("c" * 64),
            (),
            MEASURED,
        ),
        lambda: JointUnitCalibration(UnitSerial("m"), float("nan"), 1, None, MEASURED),
        lambda: JointUnitCalibration(UnitSerial("m"), 0.0, 1, (1.0, -1.0), MEASURED),
        lambda: Measurement(CalibrationMethod.HOMING, datetime(2026, 1, 1)),
    ],
)
def test_a_malformed_measurement_is_refused(build: Callable[[], object]) -> None:
    with pytest.raises(CalibrationError):
        build()


def _so101() -> Embodiment:
    return embodiments["so101"]


def _quest() -> Embodiment:
    return next(body for body in BODIES if body.name == "quest-ego")


def test_a_measured_joint_and_camera_replace_the_nominal_ones() -> None:
    arm = _so101()
    slot = next(slot for slot in slots(arm) if slot.kind is SlotKind.ACTUATOR)
    joint = _joint()
    instance = RobotInstance.of(arm.id, {slot: UnitSerial("motor-1")}, (joint,))
    resolved = resolve(arm, instance, (joint,))
    assert resolved.joint(slot).zero_offset == joint.zero_offset
    assert resolved.joint(slot).measured == calibration_id(joint)
    assert resolved.joint(slot).unit == "motor-1"

    head = _quest()
    camera_name = head.cameras[0].name
    optics = head.cameras[0].camera.optics
    camera = _camera(width=optics.width, height=optics.height)
    camera_slot = Slot(SlotKind.CAMERA, camera_name)
    instance = RobotInstance.of(head.id, {camera_slot: UnitSerial("cam-1")}, (camera,))
    assert resolve(head, instance, (camera,)).camera(camera_name).calibration == camera


def test_an_instance_document_round_trips_and_its_id_is_its_content() -> None:
    arm = _so101()
    slot = next(slot for slot in slots(arm) if slot.kind is SlotKind.ACTUATOR)
    instance = RobotInstance.of(arm.id, {slot: UnitSerial("motor-1")}, (_joint(),))
    document = json.loads(json.dumps(instance.to_dict()))
    assert RobotInstance.from_dict(document) == instance
    assert RobotInstance.from_dict(document).id == instance.id
    assert RobotInstance.of(arm.id, {slot: UnitSerial("motor-2")}, (_joint(),)).id != instance.id


def test_an_instance_binds_only_to_its_body_its_slots_and_its_selection() -> None:
    arm = _so101()
    actuator = next(slot for slot in slots(arm) if slot.kind is SlotKind.ACTUATOR)
    component = next(slot for slot in slots(arm) if slot.kind is SlotKind.COMPONENT)
    joint = _joint()
    placed = {actuator: UnitSerial("motor-1")}
    with pytest.raises(InstanceError, match="different embodiment"):
        resolve(_quest(), RobotInstance.of(arm.id, placed, (joint,)), (joint,))
    with pytest.raises(InstanceError, match="no camera slot"):
        bad = {Slot(SlotKind.CAMERA, "nowhere"): UnitSerial("x")}
        resolve(arm, RobotInstance.of(arm.id, bad, ()), ())
    with pytest.raises(InstanceError, match="not the ones"):
        resolve(arm, RobotInstance.of(arm.id, placed, (joint,)), ())
    with pytest.raises(InstanceError, match="not placed"):
        resolve(arm, RobotInstance.of(arm.id, {}, (joint,)), (joint,))
    with pytest.raises(InstanceError, match="cannot describe"):
        wrong = {component: UnitSerial("motor-1")}
        resolve(arm, RobotInstance.of(arm.id, wrong, (joint,)), (joint,))
    unnamed = JointUnitCalibration(None, 0.0, 1, None, MEASURED)
    with pytest.raises(InstanceError, match="named units"):
        resolve(arm, RobotInstance.of(arm.id, placed, (unnamed,)), (unnamed,))
    second = _joint(zero=0.5)
    with pytest.raises(InstanceError, match="two calibrations"):
        resolve(arm, RobotInstance.of(arm.id, placed, (joint, second)), (joint, second))
    with pytest.raises(InstanceError, match="two slots"):
        RobotInstance.of(arm.id, {actuator: UnitSerial("m"), component: UnitSerial("m")}, ())


def test_a_rigid_calibration_joins_true_links_of_the_exact_description() -> None:
    arm = _so101()
    links = sorted(description_links(arm.urdf_bytes))
    good = _rigid(links[0], links[1], arm)
    resolved = resolve(arm, RobotInstance.of(arm.id, {}, (good,)), (good,))
    assert resolved.frames[0].calibration == good
    invented = _rigid(links[0], "not_a_link", arm)
    with pytest.raises(InstanceError, match="not a link"):
        resolve(arm, RobotInstance.of(arm.id, {}, (invented,)), (invented,))
    other = RigidCalibration(
        links[0],
        links[1],
        IDENTITY_TRANSFORM,
        RigidReference.DESCRIPTION,
        Sha256Digest("b" * 64),
        (),
        MEASURED,
    )
    with pytest.raises(InstanceError, match="different description"):
        resolve(arm, RobotInstance.of(arm.id, {}, (other,)), (other,))


def test_a_camera_measured_at_another_raster_than_the_body_declares_is_refused() -> None:
    head = _quest()
    name = head.cameras[0].name
    camera = _camera(width=1920, height=1080)
    instance = RobotInstance.of(
        head.id, {Slot(SlotKind.CAMERA, name): UnitSerial("cam-1")}, (camera,)
    )
    with pytest.raises(InstanceError, match="declares"):
        resolve(head, instance, (camera,))
