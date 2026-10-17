"""What was measured about one physical unit, kept apart from the design it was built to.

Three kinds cover every calibration the platform keeps:

- :class:`CameraUnitCalibration`: one camera unit's intrinsics at one image profile.
- :class:`RigidCalibration`: one rigid transform between two frames of a body, either
  absolute or a correction of the description's own edge (mount corrections, hand-eye,
  stereo pairs and depth-to-colour alignment are all this).
- :class:`JointUnitCalibration`: one actuator or encoder unit's zero, direction and range.

Each carries a :class:`Measurement` (method, time, residual, evidence) and is identified by
the digest of its document. Calibration never moves an embodiment id: a robot instance
(:mod:`sx_embodiments.instance`) selects calibrations for the units placed in its body.
Sealed forms recorded before these kinds existed keep their bytes; their readers parse them
into these kinds, recording the sealed document as evidence.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import UTC, datetime
from enum import StrEnum
from typing import Final, Literal, NewType, cast

from sx_contracts import Sha256Digest
from sx_contracts.identity import JsonValue, content_digest

from .errors import EmbodimentError

CALIBRATION_SCHEMA: Final = "sx.calibration/v1"

#: The joint value at an actuator's or encoder's zero when no unit was measured. The drive
#: map every driver applies is ``actuator = sign * (joint - zero_offset) * reduction``.
UNCALIBRATED_ZERO: Final = 0.0

UnitSerial = NewType("UnitSerial", str)
"""The identity a physical unit reports or carries: a serial, a flash UID, a bus serial."""

CalibrationId = NewType("CalibrationId", Sha256Digest)


class CalibrationError(EmbodimentError):
    """A calibration document or value is not a usable measurement."""


class CalibrationMethod(StrEnum):
    """How a calibration was measured. Solvers name theirs; sealed forms map onto these."""

    VENDOR_FACTORY = "vendor_factory"  # read from the unit's own factory record
    CHECKERBOARD = "checkerboard"
    CHARUCO = "charuco"
    APRILTAG = "apriltag"
    HAND_EYE = "hand_eye"
    BOARD_EPISODE_FIT = (
        "board_episode_fit"  # a board seen in a recorded episode, solved over the chain
    )
    HOMING = "homing"  # the unit was set at a known pose and its reading taken as the zero
    UNRECORDED = "unrecorded"  # a sealed form that never said how it was measured


class ResidualUnit(StrEnum):
    PIXEL = "pixel"
    METRE = "metre"
    RADIAN = "radian"


class EvidenceKind(StrEnum):
    IMAGE = "image"  # sha256 of one calibration view
    EPISODE = "episode"  # an episode id the solve or its check used
    SEALED_DOCUMENT = "sealed_document"  # sha256 of the sealed form this was parsed from


@dataclass(frozen=True, slots=True)
class Residual:
    value: float
    unit: ResidualUnit

    def __post_init__(self) -> None:
        if not math.isfinite(self.value) or self.value < 0.0:
            raise CalibrationError("a residual is a finite non-negative number")


@dataclass(frozen=True, slots=True)
class Evidence:
    kind: EvidenceKind
    reference: str

    def __post_init__(self) -> None:
        if not self.reference.strip() or self.reference != self.reference.strip():
            raise CalibrationError("evidence names what it references")


@dataclass(frozen=True, slots=True)
class Measurement:
    """How, when and how well one calibration was measured, and from what."""

    method: CalibrationMethod
    measured_at: datetime | None
    residual: Residual | None = None
    evidence: tuple[Evidence, ...] = ()

    def __post_init__(self) -> None:
        if self.measured_at is not None and self.measured_at.utcoffset() is None:
            raise CalibrationError("a measurement time carries its timezone")
        if len(set(self.evidence)) != len(self.evidence):
            raise CalibrationError("evidence is listed once")

    def to_dict(self) -> dict[str, JsonValue]:
        return {
            "method": self.method.value,
            "measured_at": (
                self.measured_at.astimezone(UTC).isoformat().replace("+00:00", "Z")
                if self.measured_at is not None
                else None
            ),
            "residual": (
                {"value": self.residual.value, "unit": self.residual.unit.value}
                if self.residual is not None
                else None
            ),
            "evidence": [
                {"kind": item.kind.value, "reference": item.reference} for item in self.evidence
            ],
        }

    @classmethod
    def from_dict(cls, document: Mapping[str, object]) -> Measurement:
        measured_at = document.get("measured_at")
        residual = document.get("residual")
        evidence = document.get("evidence")
        if not isinstance(evidence, list):
            raise CalibrationError("measurement evidence is a list")
        residual_object = _object(residual, "residual") if residual is not None else None
        return cls(
            method=CalibrationMethod(_text(document, "method")),
            measured_at=(
                utc_time(_string(measured_at, "measured_at")) if measured_at is not None else None
            ),
            residual=(
                Residual(
                    _number(residual_object, "value"),
                    ResidualUnit(_text(residual_object, "unit")),
                )
                if residual_object is not None
                else None
            ),
            evidence=tuple(
                Evidence(
                    EvidenceKind(_text(entry, "kind")),
                    _text(entry, "reference"),
                )
                for entry in (_object(item, "evidence") for item in cast("list[object]", evidence))
            ),
        )


def utc_time(text: str) -> datetime:
    """Read an ISO-8601 UTC time written with ``Z``; every other spelling is refused."""
    if not text.endswith("Z") or "T" not in text:
        raise CalibrationError(f"measurement time {text!r} is not ISO-8601 UTC with Z")
    try:
        value = datetime.fromisoformat(text.removesuffix("Z") + "+00:00")
    except ValueError as error:
        raise CalibrationError(f"measurement time {text!r} is not ISO-8601") from error
    return value


#: Coefficients each distortion model takes, in the camera wire vocabulary.
DISTORTION_COEFFICIENTS: Final[Mapping[str, int]] = {
    "none": 0,
    "plumb_bob": 5,
    "rational_polynomial": 8,
    "equidistant": 4,
    "kannala_brandt": 4,
}


@dataclass(frozen=True, slots=True)
class CameraUnitCalibration:
    """One camera unit's intrinsics, measured for exactly ``width`` x ``height``.

    ``unit`` is ``None`` only for a sealed form that never recorded which unit it measured;
    a robot instance refuses to select such a calibration.
    """

    unit: UnitSerial | None
    width: int
    height: int
    image_from_camera: tuple[float, ...]
    distortion_model: str
    distortion_coefficients: tuple[float, ...]
    measurement: Measurement

    def __post_init__(self) -> None:
        _require_unit(self.unit)
        if type(self.width) is not int or type(self.height) is not int:
            raise CalibrationError("camera image dimensions are integers")
        if self.width <= 0 or self.height <= 0:
            raise CalibrationError("camera image dimensions are positive")
        if len(self.image_from_camera) != 9 or not all(
            math.isfinite(value) for value in self.image_from_camera
        ):
            raise CalibrationError("camera intrinsics are a finite 3x3 matrix")
        fx, _skew, cx, row_1_x, fy, cy, row_2_x, row_2_y, row_2_z = self.image_from_camera
        if fx <= 0.0 or fy <= 0.0:
            raise CalibrationError("camera focal lengths are positive")
        if (
            any(abs(value) > 1e-9 for value in (row_1_x, row_2_x, row_2_y))
            or abs(row_2_z - 1.0) > 1e-9
        ):
            raise CalibrationError("camera intrinsics use canonical homogeneous rows")
        if not (0.0 <= cx < self.width and 0.0 <= cy < self.height):
            raise CalibrationError("the principal point lies inside the raster")
        expected = DISTORTION_COEFFICIENTS.get(self.distortion_model)
        if expected is None:
            raise CalibrationError(f"unknown distortion model {self.distortion_model!r}")
        if len(self.distortion_coefficients) != expected or not all(
            math.isfinite(value) for value in self.distortion_coefficients
        ):
            raise CalibrationError(
                f"{self.distortion_model} takes exactly {expected} finite coefficients"
            )


class RigidReference(StrEnum):
    """What a rigid transform is measured against."""

    #: ``parent_from_child`` is the whole transform between the two frames.
    ABSOLUTE = "absolute"
    #: ``parent_from_child`` corrects the description's own transform between the frames:
    #: ``T_actual = T_description · parent_from_child``, valid only for one exact URDF.
    DESCRIPTION = "description"


@dataclass(frozen=True, slots=True)
class RigidCalibration:
    """One rigid transform between two frames, row-major 4x4 in metres.

    ``units`` are the physical units whose placement was measured (a camera and the arm it
    was solved against, a gripper on its controller); empty when the transform belongs to
    the robot as a whole. A ``DESCRIPTION`` reference names the URDF it corrects.
    """

    parent_frame: str
    child_frame: str
    parent_from_child: tuple[float, ...]
    reference: RigidReference
    description_sha256: Sha256Digest | None
    units: tuple[UnitSerial, ...]
    measurement: Measurement

    def __post_init__(self) -> None:
        for frame in (self.parent_frame, self.child_frame):
            if not frame.strip() or frame != frame.strip():
                raise CalibrationError("a rigid calibration names both of its frames")
        if (
            self.parent_frame == self.child_frame
            and self.reference is RigidReference.ABSOLUTE
            and any(
                abs(a - b) > 1e-9
                for a, b in zip(self.parent_from_child, IDENTITY_TRANSFORM, strict=False)
            )
        ):
            raise CalibrationError("a frame's absolute transform to itself is the identity")
        if (self.reference is RigidReference.DESCRIPTION) != (self.description_sha256 is not None):
            raise CalibrationError(
                "a correction names the description it corrects, and only a correction does"
            )
        if self.description_sha256 is not None:
            object.__setattr__(self, "description_sha256", Sha256Digest(self.description_sha256))
        for unit in self.units:
            _require_unit(unit)
        if len(set(self.units)) != len(self.units) or list(self.units) != sorted(self.units):
            raise CalibrationError("rigid calibration units are unique and sorted")
        _require_rigid(self.parent_from_child)


@dataclass(frozen=True, slots=True)
class JointUnitCalibration:
    """One actuator or encoder unit's measured zero, direction and range.

    ``zero_offset`` is the joint value, in the joint's own unit, at which the unit reads
    zero; it is the ``zero_offset`` of the drive map. ``direction`` is ``-1`` when the unit
    turns against its design sign. ``range`` is the measured travel, when one was taken.
    """

    unit: UnitSerial | None
    zero_offset: float
    direction: Literal[1, -1]
    range: tuple[float, float] | None
    measurement: Measurement

    def __post_init__(self) -> None:
        _require_unit(self.unit)
        if not math.isfinite(self.zero_offset):
            raise CalibrationError("a joint zero is finite")
        if self.direction not in (1, -1):
            raise CalibrationError("a joint direction is 1 or -1")
        if self.range is not None:
            lower, upper = self.range
            if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
                raise CalibrationError("a measured joint range is finite and increasing")


Calibration = CameraUnitCalibration | RigidCalibration | JointUnitCalibration


def calibration_to_dict(calibration: Calibration) -> dict[str, JsonValue]:
    """The one document form; its digest is the calibration's id."""
    body: dict[str, JsonValue]
    match calibration:
        case CameraUnitCalibration():
            body = {
                "kind": "camera_unit",
                "unit": calibration.unit,
                "width": calibration.width,
                "height": calibration.height,
                "image_from_camera": list(calibration.image_from_camera),
                "distortion_model": calibration.distortion_model,
                "distortion_coefficients": list(calibration.distortion_coefficients),
            }
        case RigidCalibration():
            body = {
                "kind": "rigid",
                "parent_frame": calibration.parent_frame,
                "child_frame": calibration.child_frame,
                "parent_from_child": list(calibration.parent_from_child),
                "reference": calibration.reference.value,
                "description_sha256": calibration.description_sha256,
                "units": list(calibration.units),
            }
        case JointUnitCalibration():
            body = {
                "kind": "joint_unit",
                "unit": calibration.unit,
                "zero_offset": calibration.zero_offset,
                "direction": calibration.direction,
                "range": list(calibration.range) if calibration.range is not None else None,
            }
    return {
        "schema": CALIBRATION_SCHEMA,
        **body,
        "measurement": calibration.measurement.to_dict(),
    }


def calibration_id(calibration: Calibration) -> CalibrationId:
    return CalibrationId(content_digest(calibration_to_dict(calibration)))


def calibration_from_dict(document: Mapping[str, object]) -> Calibration:
    """Read one calibration document; an unknown schema or kind is refused."""
    if document.get("schema") != CALIBRATION_SCHEMA:
        raise CalibrationError(f"not a {CALIBRATION_SCHEMA} document")
    measurement = Measurement.from_dict(_object(document.get("measurement"), "measurement"))
    kind = document.get("kind")
    if kind == "camera_unit":
        expected = {"width", "height", "image_from_camera", "distortion_model"}
        expected |= {"distortion_coefficients", "unit"}
        _exact(document, expected)
        return CameraUnitCalibration(
            unit=_unit(document.get("unit")),
            width=_integer(document, "width"),
            height=_integer(document, "height"),
            image_from_camera=_numbers(document, "image_from_camera"),
            distortion_model=_text(document, "distortion_model"),
            distortion_coefficients=_numbers(document, "distortion_coefficients"),
            measurement=measurement,
        )
    if kind == "rigid":
        expected = {"parent_frame", "child_frame", "parent_from_child", "reference"}
        expected |= {"description_sha256", "units"}
        _exact(document, expected)
        description = document.get("description_sha256")
        units = document.get("units")
        if not isinstance(units, list):
            raise CalibrationError("rigid calibration units are a list")
        return RigidCalibration(
            parent_frame=_text(document, "parent_frame"),
            child_frame=_text(document, "child_frame"),
            parent_from_child=_numbers(document, "parent_from_child"),
            reference=RigidReference(_text(document, "reference")),
            description_sha256=(
                Sha256Digest(_string(description, "description_sha256"))
                if description is not None
                else None
            ),
            units=tuple(UnitSerial(_string(unit, "unit")) for unit in cast("list[object]", units)),
            measurement=measurement,
        )
    if kind == "joint_unit":
        _exact(document, {"unit", "zero_offset", "direction", "range"})
        direction = document.get("direction")
        if direction not in (1, -1) or type(direction) is not int:
            raise CalibrationError("a joint direction is 1 or -1")
        span = document.get("range")
        bounds = _number_list(span, "range") if span is not None else None
        if bounds is not None and len(bounds) != 2:
            raise CalibrationError("a joint range has two ends")
        return JointUnitCalibration(
            unit=_unit(document.get("unit")),
            zero_offset=_number(document, "zero_offset"),
            direction=cast("Literal[1, -1]", direction),
            range=(bounds[0], bounds[1]) if bounds is not None else None,
            measurement=measurement,
        )
    raise CalibrationError(f"unknown calibration kind {kind!r}")


def matrix_from_pose(
    translation_m: tuple[float, float, float],
    rotation_xyzw: tuple[float, float, float, float],
) -> tuple[float, ...]:
    """The row-major 4x4 of a translation and a unit quaternion."""
    x, y, z, w = rotation_xyzw
    return (
        1 - 2 * (y * y + z * z),
        2 * (x * y - z * w),
        2 * (x * z + y * w),
        translation_m[0],
        2 * (x * y + z * w),
        1 - 2 * (x * x + z * z),
        2 * (y * z - x * w),
        translation_m[1],
        2 * (x * z - y * w),
        2 * (y * z + x * w),
        1 - 2 * (x * x + y * y),
        translation_m[2],
        0.0,
        0.0,
        0.0,
        1.0,
    )


IDENTITY_TRANSFORM: Final = matrix_from_pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def _require_rigid(matrix: tuple[float, ...]) -> None:
    if len(matrix) != 16 or not all(math.isfinite(value) for value in matrix):
        raise CalibrationError("a rigid transform is a finite 4x4 matrix")
    if any(abs(a - b) > 1e-6 for a, b in zip(matrix[12:], (0.0, 0.0, 0.0, 1.0), strict=True)):
        raise CalibrationError("a rigid transform's bottom row is 0 0 0 1")
    rows = (matrix[0:3], matrix[4:7], matrix[8:11])
    for i in range(3):
        for j in range(3):
            dot = sum(rows[i][k] * rows[j][k] for k in range(3))
            if abs(dot - (1.0 if i == j else 0.0)) > 1e-5:
                raise CalibrationError("a rigid transform's rotation is orthonormal")
    (a, b, c), (d, e, f), (g, h, i) = rows
    determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(determinant - 1.0) > 1e-5:
        raise CalibrationError("a rigid transform's rotation keeps handedness")


def _require_unit(unit: UnitSerial | None) -> None:
    if unit is not None and (not unit.strip() or unit != unit.strip()):
        raise CalibrationError("a unit identity is a non-empty trimmed string")


def _exact(document: Mapping[str, object], fields: set[str]) -> None:
    extra = set(document) - fields - {"schema", "kind", "measurement"}
    missing = fields - set(document)
    if extra or missing:
        raise CalibrationError(
            f"calibration fields disagree: missing {sorted(missing)}, unexpected {sorted(extra)}"
        )


def _object(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise CalibrationError(f"{name} is an object")
    return cast("Mapping[str, object]", value)


def _string(value: object, name: str) -> str:
    if not isinstance(value, str):
        raise CalibrationError(f"{name} is a string")
    return value


def _text(document: Mapping[str, object], name: str) -> str:
    return _string(document.get(name), name)


def _unit(value: object) -> UnitSerial | None:
    return UnitSerial(_string(value, "unit")) if value is not None else None


def _integer(document: Mapping[str, object], name: str) -> int:
    value = document.get(name)
    if type(value) is not int:
        raise CalibrationError(f"{name} is an integer")
    return value


def _number(document: Mapping[str, object], name: str) -> float:
    value = document.get(name)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise CalibrationError(f"{name} is a number")
    return float(value)


def _number_list(value: object, name: str) -> tuple[float, ...]:
    if not isinstance(value, list):
        raise CalibrationError(f"{name} is a list of numbers")
    items = cast("list[object]", value)
    if any(isinstance(item, bool) or not isinstance(item, int | float) for item in items):
        raise CalibrationError(f"{name} is a list of numbers")
    return tuple(float(cast("float", item)) for item in items)


def _numbers(document: Mapping[str, object], name: str) -> tuple[float, ...]:
    return _number_list(document.get(name), name)
