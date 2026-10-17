"""One physical robot: a content-addressed body, the unit in each slot, its calibrations.

An embodiment is a design. A robot instance places serial units into the design's slots
(the actuator driving each joint, the encoder reading it, each declared camera) and selects
the calibrations measured on those units. :func:`resolve` binds the two and returns what
the hardware actually is: measured where a calibration was selected, the nominal design,
labelled nominal, everywhere else. Neither moves the embodiment's id.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from enum import StrEnum
from typing import Final, Literal, cast

from sx_contracts import Sha256Digest
from sx_contracts.identity import JsonValue, content_digest

from .calibration import (
    UNCALIBRATED_ZERO,
    Calibration,
    CalibrationError,
    CalibrationId,
    CameraUnitCalibration,
    JointUnitCalibration,
    RigidCalibration,
    RigidReference,
    UnitSerial,
    calibration_id,
)
from .embodiment import Embodiment
from .identity import EmbodimentId
from .layout import EncoderReadout
from .parts import CameraOptics, CameraSpec

INSTANCE_SCHEMA: Final = "sx.robot-instance/v1"


class InstanceError(CalibrationError):
    """A robot instance does not bind to its body or to the calibrations it selects."""


class SlotKind(StrEnum):
    COMPONENT = "component"  # the unit one non-camera component is: an arm, a gripper, a base
    ACTUATOR = "actuator"  # the unit driving one joint
    ENCODER = "encoder"  # a separate encoder reading one joint
    CAMERA = "camera"  # one declared camera


@dataclass(frozen=True, slots=True, order=True)
class Slot:
    """A place in a body one physical unit fills.

    ``name`` is a component instance, a state name (``instance/axis``) or a camera name.
    """

    kind: SlotKind
    name: str


def slots(embodiment: Embodiment) -> tuple[Slot, ...]:
    """Every slot of a body, derived from its state and cameras, in one sorted order."""
    found = [
        Slot(SlotKind.COMPONENT, component.instance)
        for component in embodiment.components
        if not isinstance(component.part, CameraSpec)
    ]
    for coordinate in embodiment.state.coordinates:
        name = f"{coordinate.instance}/{coordinate.axis.name}"
        if coordinate.axis.actuator is not None:
            found.append(Slot(SlotKind.ACTUATOR, name))
        if isinstance(coordinate.axis.observation, EncoderReadout):
            found.append(Slot(SlotKind.ENCODER, name))
    found.extend(Slot(SlotKind.CAMERA, camera.name) for camera in embodiment.cameras)
    return tuple(sorted(found))


@dataclass(frozen=True, slots=True)
class PlacedUnit:
    slot: Slot
    unit: UnitSerial


@dataclass(frozen=True, slots=True)
class RobotInstance:
    """A body, the unit in each filled slot and the calibrations selected for them."""

    embodiment: EmbodimentId
    units: tuple[PlacedUnit, ...]
    calibrations: tuple[CalibrationId, ...]

    def __post_init__(self) -> None:
        placed = [entry.slot for entry in self.units]
        if placed != sorted(placed) or len(set(placed)) != len(placed):
            raise InstanceError("an instance fills each slot once, in slot order")
        serials = [entry.unit for entry in self.units]
        if len(set(serials)) != len(serials):
            raise InstanceError("one unit cannot fill two slots")
        if list(self.calibrations) != sorted(set(self.calibrations)):
            raise InstanceError("selected calibrations are unique and sorted")

    @classmethod
    def of(
        cls,
        embodiment: EmbodimentId,
        units: Mapping[Slot, UnitSerial],
        calibrations: Iterable[Calibration],
    ) -> RobotInstance:
        return cls(
            embodiment,
            tuple(PlacedUnit(slot, units[slot]) for slot in sorted(units)),
            tuple(sorted({calibration_id(item) for item in calibrations})),
        )

    def unit_in(self, slot: Slot) -> UnitSerial | None:
        return next((entry.unit for entry in self.units if entry.slot == slot), None)

    def to_dict(self) -> dict[str, JsonValue]:
        return {
            "schema": INSTANCE_SCHEMA,
            "embodiment": str(self.embodiment),
            "units": [
                {"kind": entry.slot.kind.value, "slot": entry.slot.name, "unit": entry.unit}
                for entry in self.units
            ],
            "calibrations": list(self.calibrations),
        }

    @property
    def id(self) -> Sha256Digest:
        return content_digest(self.to_dict())

    @classmethod
    def from_dict(cls, document: Mapping[str, object]) -> RobotInstance:
        if set(document) != {"schema", "embodiment", "units", "calibrations"}:
            raise InstanceError("a robot instance has exactly its four fields")
        if document["schema"] != INSTANCE_SCHEMA:
            raise InstanceError(f"not a {INSTANCE_SCHEMA} document")
        units = document["units"]
        selected = document["calibrations"]
        if not isinstance(units, list) or not isinstance(selected, list):
            raise InstanceError("instance units and calibrations are lists")
        placed: list[PlacedUnit] = []
        for raw in cast("list[object]", units):
            if not isinstance(raw, Mapping):
                raise InstanceError("a placed unit is an object")
            entry = cast("Mapping[str, object]", raw)
            if set(entry) != {"kind", "slot", "unit"} or not all(
                isinstance(entry[key], str) for key in entry
            ):
                raise InstanceError("a placed unit names its slot kind, slot and unit")
            placed.append(
                PlacedUnit(
                    Slot(SlotKind(cast("str", entry["kind"])), cast("str", entry["slot"])),
                    UnitSerial(cast("str", entry["unit"])),
                )
            )
        if not all(isinstance(item, str) for item in cast("list[object]", selected)):
            raise InstanceError("selected calibrations are ids")
        return cls(
            EmbodimentId(cast("str", document["embodiment"])),
            tuple(placed),
            tuple(
                CalibrationId(Sha256Digest(cast("str", item)))
                for item in cast("list[object]", selected)
            ),
        )


@dataclass(frozen=True, slots=True)
class ResolvedCamera:
    """One camera as the instance has it: measured optics, or the body's nominal ones."""

    name: str
    unit: UnitSerial | None
    measured: CalibrationId | None
    calibration: CameraUnitCalibration | None
    nominal: CameraOptics


@dataclass(frozen=True, slots=True)
class ResolvedJoint:
    """One actuator or encoder slot: its unit's zero and direction, nominal when unmeasured."""

    slot: Slot
    unit: UnitSerial | None
    measured: CalibrationId | None
    zero_offset: float
    direction: Literal[1, -1]
    range: tuple[float, float] | None


@dataclass(frozen=True, slots=True)
class ResolvedFrame:
    measured: CalibrationId
    calibration: RigidCalibration


@dataclass(frozen=True, slots=True)
class ResolvedInstance:
    instance: RobotInstance
    cameras: tuple[ResolvedCamera, ...]
    joints: tuple[ResolvedJoint, ...]
    frames: tuple[ResolvedFrame, ...]

    def joint(self, slot: Slot) -> ResolvedJoint:
        found = next((joint for joint in self.joints if joint.slot == slot), None)
        if found is None:
            raise InstanceError(f"the body has no {slot.kind} slot {slot.name!r}")
        return found

    def camera(self, name: str) -> ResolvedCamera:
        found = next((camera for camera in self.cameras if camera.name == name), None)
        if found is None:
            raise InstanceError(f"the body declares no camera {name!r}")
        return found


def nominal_instance(embodiment: Embodiment) -> RobotInstance:
    """The body with no unit recorded and nothing measured: the design as built."""
    return RobotInstance(embodiment.id, (), ())


def resolve(
    embodiment: Embodiment,
    instance: RobotInstance,
    calibrations: Iterable[Calibration],
) -> ResolvedInstance:
    """Bind an instance to its body and to exactly the calibrations it selects."""
    if instance.embodiment != embodiment.id:
        raise InstanceError("the instance is of a different embodiment")
    body_slots = set(slots(embodiment))
    for entry in instance.units:
        if entry.slot not in body_slots:
            raise InstanceError(
                f"the body has no {entry.slot.kind} slot {entry.slot.name!r} for {entry.unit}"
            )
    supplied = {calibration_id(item): item for item in calibrations}
    if set(supplied) != set(instance.calibrations):
        raise InstanceError("the calibrations supplied are not the ones the instance selects")
    slot_of = {entry.unit: entry.slot for entry in instance.units}

    by_slot: dict[Slot, tuple[CalibrationId, CameraUnitCalibration | JointUnitCalibration]] = {}
    frames: list[ResolvedFrame] = []
    for identity, calibration in sorted(supplied.items()):
        match calibration:
            case CameraUnitCalibration() | JointUnitCalibration():
                if calibration.unit is None:
                    raise InstanceError("an instance selects only calibrations of named units")
                slot = slot_of.get(calibration.unit)
                if slot is None:
                    raise InstanceError(f"unit {calibration.unit} is not placed in this instance")
                wanted = (
                    (SlotKind.CAMERA,)
                    if isinstance(calibration, CameraUnitCalibration)
                    else (SlotKind.ACTUATOR, SlotKind.ENCODER)
                )
                if slot.kind not in wanted:
                    raise InstanceError(
                        f"unit {calibration.unit} fills a {slot.kind} slot, "
                        "which this calibration cannot describe"
                    )
                if slot in by_slot:
                    raise InstanceError(f"two calibrations select slot {slot.name!r}")
                by_slot[slot] = (identity, calibration)
            case RigidCalibration():
                for unit in calibration.units:
                    if unit not in slot_of:
                        raise InstanceError(f"unit {unit} is not placed in this instance")
                frames.append(ResolvedFrame(identity, calibration))
    if frames:
        require_frames(
            embodiment, tuple(frame.calibration for frame in frames), embodiment.urdf_bytes
        )

    cameras: list[ResolvedCamera] = []
    for camera in embodiment.cameras:
        slot = Slot(SlotKind.CAMERA, camera.name)
        chosen = by_slot.get(slot)
        measured = cast("CameraUnitCalibration | None", chosen[1] if chosen else None)
        nominal = camera.camera.optics
        if (
            measured is not None
            and (measured.width, measured.height) != (nominal.width, nominal.height)
        ):
            raise InstanceError(
                f"{camera.name}: measured {measured.width}x{measured.height}, "
                f"the body declares {nominal.width}x{nominal.height}"
            )
        cameras.append(
            ResolvedCamera(
                camera.name,
                instance.unit_in(slot),
                chosen[0] if chosen else None,
                measured,
                nominal,
            )
        )
    joints: list[ResolvedJoint] = []
    for slot in sorted(body_slots):
        if slot.kind not in (SlotKind.ACTUATOR, SlotKind.ENCODER):
            continue
        chosen = by_slot.get(slot)
        if chosen is None:
            joints.append(
                ResolvedJoint(slot, instance.unit_in(slot), None, UNCALIBRATED_ZERO, 1, None)
            )
            continue
        joint = cast("JointUnitCalibration", chosen[1])
        joints.append(
            ResolvedJoint(
                slot,
                joint.unit,
                chosen[0],
                joint.zero_offset,
                joint.direction,
                joint.range,
            )
        )
    return ResolvedInstance(instance, tuple(cameras), tuple(joints), tuple(frames))


def description_links(urdf: bytes) -> frozenset[str]:
    """Every link of a description."""
    root = ET.fromstring(urdf)
    return frozenset(name for link in root.iter("link") if (name := link.get("name")) is not None)


def require_frames(
    embodiment: Embodiment, frames: tuple[RigidCalibration, ...], urdf: bytes
) -> None:
    """Every frame is a link of the body's description ``urdf``, and every correction
    corrects exactly that description."""
    links = description_links(urdf)
    pairs: set[tuple[str, str]] = set()
    for frame in frames:
        for name in (frame.parent_frame, frame.child_frame):
            if name not in links:
                raise InstanceError(f"{name!r} is not a link of the body's description")
        if (
            frame.reference is RigidReference.DESCRIPTION
            and frame.description_sha256 != embodiment.urdf.sha256
        ):
            raise InstanceError(
                f"{frame.parent_frame} -> {frame.child_frame} corrects a different description"
            )
        pair = (frame.parent_frame, frame.child_frame)
        if pair in pairs:
            raise InstanceError(f"two calibrations measure {pair[0]} -> {pair[1]}")
        pairs.add(pair)
