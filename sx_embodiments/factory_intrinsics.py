"""A camera unit's intrinsics as its vendor's factory calibration reports them.

``sx_realsense::FactoryIntrinsics`` (``drivers/sx-realsense``) renders, for each stream profile
a station opens, what librealsense reads from the calibration Intel wrote into the device:
``sx.factory-intrinsics/v1``, in the SDK's own terms. :func:`factory_calibration` is its one
reader and the one place those terms become the platform's distortion vocabulary. The result is
a :class:`~sx_embodiments.calibration.CameraUnitCalibration` measured by
:attr:`~sx_embodiments.calibration.CalibrationMethod.VENDOR_FACTORY` with no measurement time
(the record does not carry when Intel calibrated the unit) and the record's digest as its
evidence, so a factory value is never mistaken for one measured on the robot.

A RealSense holds one camera unit per imager, each with its own intrinsics, so a unit is spelled
``<device serial>/<stream>``.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Final, cast

from sx_contracts.identity import JsonObject, content_digest

from .calibration import (
    CalibrationError,
    CalibrationMethod,
    CameraUnitCalibration,
    Evidence,
    EvidenceKind,
    Measurement,
    UnitSerial,
)

FACTORY_SCHEMA: Final = "sx.factory-intrinsics/v1"
_FIELDS: Final = frozenset(
    {
        "schema",
        "vendor",
        "product",
        "firmware",
        "unit",
        "serial",
        "stream",
        "width",
        "height",
        "fx",
        "fy",
        "ppx",
        "ppy",
        "distortion",
        "coefficients",
    }
)
_STREAMS: Final = frozenset({"color", "infrared_1", "infrared_2"})


def _distortion(model: str, coefficients: tuple[float, ...]) -> tuple[str, tuple[float, ...]]:
    """librealsense's model and five coefficients in the platform's distortion vocabulary.

    Brown-Conrady is the plumb-bob model (``k1, k2, p1, p2, k3``, the SDK's order) and
    Kannala-Brandt keeps its first four. An inverse Brown-Conrady model maps image to ideal
    points rather than ideal to image, so it is ``none`` only when it does nothing; the modified
    and F-theta models have no counterpart and are refused.
    """
    match model:
        case "none":
            return "none", ()
        case "brown_conrady":
            return "plumb_bob", coefficients
        case "kannala_brandt4":
            return "kannala_brandt", coefficients[:4]
        case "inverse_brown_conrady" if not any(coefficients):
            return "none", ()
        case _:
            raise CalibrationError(
                f"a factory {model} model with coefficients {list(coefficients)} has no "
                "counterpart in the platform's distortion models"
            )


def _finite(value: object, name: str) -> float:
    if type(value) is not float and type(value) is not int:
        raise CalibrationError(f"factory {name} is a number")
    number = float(cast("float", value))
    if not math.isfinite(number):
        raise CalibrationError(f"factory {name} is finite")
    return number


def _number(document: Mapping[str, object], name: str) -> float:
    return _finite(document[name], name)


def _text(document: Mapping[str, object], name: str) -> str:
    value = document[name]
    if not isinstance(value, str) or not value.strip():
        raise CalibrationError(f"factory {name} is text")
    return value


def _integer(document: Mapping[str, object], name: str) -> int:
    value = document[name]
    if type(value) is not int:
        raise CalibrationError(f"factory {name} is an integer")
    return value


def factory_calibration(document: Mapping[str, object]) -> CameraUnitCalibration:
    """The camera-unit calibration one ``sx.factory-intrinsics/v1`` record states.

    Raises:
        CalibrationError: for anything but such a record of one RealSense imager, or a model the
            platform's distortion vocabulary cannot state.
    """
    if frozenset(document) != _FIELDS:
        raise CalibrationError(f"a {FACTORY_SCHEMA} record has exactly its fields")
    if document["schema"] != FACTORY_SCHEMA or document["vendor"] != "realsense":
        raise CalibrationError(f"not a RealSense {FACTORY_SCHEMA} record")
    serial = _text(document, "serial")
    stream = _text(document, "stream")
    unit = _text(document, "unit")
    if stream not in _STREAMS or not serial.isalnum() or unit != f"{serial}/{stream}":
        raise CalibrationError(f"{unit!r} is not the {stream!r} imager of device {serial!r}")
    _text(document, "product")
    _text(document, "firmware")
    raw = document["coefficients"]
    if not isinstance(raw, list) or len(cast("list[object]", raw)) != 5:
        raise CalibrationError("a factory record carries the SDK's five coefficients")
    coefficients = tuple(_finite(value, "coefficient") for value in cast("list[object]", raw))
    model, kept = _distortion(_text(document, "distortion"), coefficients)
    fx, fy = _number(document, "fx"), _number(document, "fy")
    ppx, ppy = _number(document, "ppx"), _number(document, "ppy")
    return CameraUnitCalibration(
        unit=UnitSerial(unit),
        width=_integer(document, "width"),
        height=_integer(document, "height"),
        image_from_camera=(fx, 0.0, ppx, 0.0, fy, ppy, 0.0, 0.0, 1.0),
        distortion_model=model,
        distortion_coefficients=kept,
        measurement=Measurement(
            method=CalibrationMethod.VENDOR_FACTORY,
            measured_at=None,
            evidence=(
                Evidence(
                    EvidenceKind.SEALED_DOCUMENT, content_digest(cast("JsonObject", dict(document)))
                ),
            ),
        ),
    )
