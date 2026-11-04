"""A camera unit's intrinsics as its vendor's factory calibration reports them.

Two vendors' records are read, each into a
:class:`~sx_embodiments.calibration.CameraUnitCalibration` measured by
:attr:`~sx_embodiments.calibration.CalibrationMethod.VENDOR_FACTORY` with no measurement time
and the record's digest as its evidence, so a factory value is never mistaken for one measured
on the robot.

**RealSense.** ``sx_realsense::FactoryIntrinsics`` (``drivers/sx-realsense``) renders, for each
stream profile a station opens, what librealsense reads from the calibration Intel wrote into the
device: ``sx.factory-intrinsics/v1``, in the SDK's own terms. :func:`factory_calibration` is its
one reader and the one place those terms become the platform's distortion vocabulary. The record
does not carry when Intel calibrated the unit.

A RealSense holds one camera unit per imager, each with its own intrinsics, so a unit is spelled
``<device serial>/<stream>``.

**ZED.** Stereolabs publishes each ZED's factory calibration by serial, at
``https://calib.stereolabs.com/?SN=<serial>``: an ini file with, per eye and per resolution
(``LEFT_CAM_HD``), the pinhole and the first five distortion terms, per eye a resolution-free
``LEFT_DISTO`` with eight, and a ``STEREO`` section placing the right eye from the left.
:func:`zed_factory_calibration` reads one resolution of it into the two eyes' calibrations, units
``<serial>/left`` and ``<serial>/right``, and the right eye's pose in the left's optical frame.
The ``DISTO`` terms are the rational model, OpenCV's ``k1, k2, p1, p2, k3, k4, k5, k6`` (the
ZED SDK's ``disto`` order); the per-resolution five are that model's numerator alone, digit for
digit, so they are read only from a file that has no ``DISTO``, as the plumb-bob model it then
is. The ``STEREO`` translation is ``Baseline, TY, TZ`` in millimetres, the right eye's position
in the left eye's frame, and ``RX, CV, RZ`` of the resolution is the Rodrigues vector of OpenCV's
stereo ``R``, which takes a point from the left eye's frame into the right's, as Stereolabs'
``zed-open-capture`` hands it to ``cv::stereoRectify`` (``examples/include/calibration.hpp`` at
``93739154``). The right eye's orientation in the left's frame is therefore that rotation's
inverse. Measured on SN 30371653's own pair (2026-11-04, 169 SIFT matches that survive a
fundamental-matrix RANSAC): rectified by this reading the matches' rows differ by 0.57 px at the
median; read as the right eye's own orientation they differ by 4.97 px, and unrectified by
33 px (its eyes' principal points sit 36 px apart vertically).

**Commissioning a stereo camera.** Reading a unit's factory record is a commissioning step, run
once per unit and never at robot run time::

    python -m sx_embodiments.factory_intrinsics zed --serial 30371653 \
        --body sentient-static-upper-body --left head_left --right head_right --out ./calibration

fetches the record by serial (or reads ``--record``), keeps it as ``<serial>.conf`` and writes
the left eye, the right eye and the right eye's pose in the left's, each an
``sx.calibration/v1`` document named by its content digest (``<calibration id>.json``), at the
raster the body declares for the two cameras. Fleet carries those documents in the robot's
directory (``AssignStationRobot.camera_calibrations``); the instance selects them, so the
station's media plan serves the unit's lens and the pair's measured geometry.
"""

from __future__ import annotations

import argparse
import configparser
import json
import math
import sys
import urllib.request
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Final, Literal, cast

from sx_contracts.identity import JsonObject, content_digest, stream_blob

from .calibration import (
    CalibrationError,
    CalibrationMethod,
    CameraUnitCalibration,
    Evidence,
    EvidenceKind,
    Measurement,
    RigidCalibration,
    RigidReference,
    UnitSerial,
    calibration_id,
    calibration_to_dict,
    matrix_from_pose,
)
from .embodiment import Embodiment
from .errors import EmbodimentError

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


#: A ZED eye's raster and the ini's name for it.
ZED_RESOLUTIONS: Final[Mapping[tuple[int, int], str]] = {
    (2208, 1242): "2K",
    (1920, 1080): "FHD",
    (1280, 720): "HD",
    (672, 376): "VGA",
}


@dataclass(frozen=True, slots=True)
class StereoFactoryCalibration:
    """A stereo camera's factory record at one resolution: each eye, and where the right sits."""

    left: CameraUnitCalibration
    right: CameraUnitCalibration
    #: The right eye's optical frame in the left's.
    left_from_right: RigidCalibration


def _ini_number(section: configparser.SectionProxy, key: str) -> float:
    try:
        text = section[key]
    except KeyError:
        raise CalibrationError(f"the ZED record's [{section.name}] has no {key}") from None
    try:
        return _finite(float(text), f"[{section.name}] {key}")
    except ValueError:
        raise CalibrationError(f"the ZED record's [{section.name}] {key} is not a number") from None


def _ini_section(ini: configparser.ConfigParser, name: str) -> configparser.SectionProxy:
    if not ini.has_section(name):
        raise CalibrationError(f"the ZED record has no [{name}]")
    return ini[name]


def _rotation(rodrigues: tuple[float, float, float]) -> tuple[float, float, float, float]:
    """The unit quaternion (x, y, z, w) of a Rodrigues vector."""
    angle = math.sqrt(sum(value * value for value in rodrigues))
    if angle == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    scale = math.sin(angle / 2) / angle
    x, y, z = (value * scale for value in rodrigues)
    return (x, y, z, math.cos(angle / 2))


def zed_factory_calibration(
    document: bytes,
    serial: str,
    width: int,
    height: int,
    frames: tuple[str, str],
) -> StereoFactoryCalibration:
    """The two eyes of ZED ``serial`` at ``width`` x ``height`` from its factory ini.

    ``frames`` are the left and right eyes' optical frames in the body that carries the ZED.

    Raises:
        CalibrationError: for a record that is not such an ini, a raster the ZED has no mode for,
            or a section, key or number it lacks.
    """
    if not serial.isdecimal():
        raise CalibrationError(f"{serial!r} is not a ZED serial")
    resolution = ZED_RESOLUTIONS.get((width, height))
    if resolution is None:
        raise CalibrationError(f"a ZED eye has no {width}x{height} mode")
    ini = configparser.ConfigParser(interpolation=None)
    ini.optionxform = str  # type: ignore[assignment,method-assign]  # keys keep their case
    try:
        ini.read_string(document.decode("utf-8"))
    except (UnicodeDecodeError, configparser.Error) as error:
        raise CalibrationError(f"the ZED record is not an ini file: {error}") from error
    measurement = Measurement(
        method=CalibrationMethod.VENDOR_FACTORY,
        measured_at=None,
        evidence=(Evidence(EvidenceKind.SEALED_DOCUMENT, stream_blob([document]).sha256),),
    )

    def eye(side: Literal["LEFT", "RIGHT"]) -> CameraUnitCalibration:
        camera = _ini_section(ini, f"{side}_CAM_{resolution}")
        if ini.has_section(f"{side}_DISTO"):
            disto = ini[f"{side}_DISTO"]
            model = "rational_polynomial"
            coefficients = tuple(
                _ini_number(disto, key) for key in ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6")
            )
        else:
            model = "plumb_bob"
            coefficients = tuple(_ini_number(camera, key) for key in ("k1", "k2", "p1", "p2", "k3"))
        fx, fy = _ini_number(camera, "fx"), _ini_number(camera, "fy")
        cx, cy = _ini_number(camera, "cx"), _ini_number(camera, "cy")
        return CameraUnitCalibration(
            unit=UnitSerial(f"{serial}/{side.lower()}"),
            width=width,
            height=height,
            image_from_camera=(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0),
            distortion_model=model,
            distortion_coefficients=coefficients,
            measurement=measurement,
        )

    left, right = eye("LEFT"), eye("RIGHT")
    stereo = _ini_section(ini, "STEREO")
    translation = tuple(_ini_number(stereo, key) / 1000.0 for key in ("Baseline", "TY", "TZ"))
    # OpenCV's R takes the left eye's points into the right's: the right eye's orientation in
    # the left's frame is its inverse, the same axis turned the other way.
    rodrigues = tuple(-_ini_number(stereo, f"{key}_{resolution}") for key in ("RX", "CV", "RZ"))
    return StereoFactoryCalibration(
        left=left,
        right=right,
        left_from_right=RigidCalibration(
            parent_frame=frames[0],
            child_frame=frames[1],
            parent_from_child=matrix_from_pose(
                cast("tuple[float, float, float]", translation),
                _rotation(cast("tuple[float, float, float]", rodrigues)),
            ),
            reference=RigidReference.ABSOLUTE,
            description_sha256=None,
            units=tuple(sorted((cast("UnitSerial", left.unit), cast("UnitSerial", right.unit)))),
            measurement=measurement,
        ),
    )


#: Where Stereolabs publishes a ZED's factory record, by serial.
ZED_RECORD_URL: Final = "https://calib.stereolabs.com/?SN={serial}"


def fetch_zed_record(serial: str) -> bytes:
    """The factory record Stereolabs publishes for ZED ``serial``: a commissioning step's read,
    never a robot's at run time."""
    if not serial.isdecimal():
        raise CalibrationError(f"{serial!r} is not a ZED serial")
    with urllib.request.urlopen(ZED_RECORD_URL.format(serial=serial), timeout=30) as answer:
        return cast("bytes", answer.read())


def zed_pair_calibration(
    body: Embodiment, left: str, right: str, record: bytes, serial: str
) -> StereoFactoryCalibration:
    """A ZED's factory record for ``body``'s stereo pair ``left``, ``right``: each eye at the
    raster the body declares for it, and the right eye's optical frame in the left's."""
    bindings = {binding.name: binding for binding in body.cameras}
    missing = sorted({left, right} - set(bindings))
    if missing:
        raise CalibrationError(f"{body.name} declares no camera {missing}")
    optics = bindings[left].camera.optics
    factory = zed_factory_calibration(
        record,
        serial,
        optics.width,
        optics.height,
        (bindings[left].frame, bindings[right].frame),
    )
    return factory


def _body(key: str) -> Embodiment:
    from . import development_embodiments, embodiments

    for registry in (embodiments, development_embodiments):
        try:
            return registry[key]
        except EmbodimentError:
            continue
    raise CalibrationError(f"no registered or development body {key!r}")


def main(argv: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="python -m sx_embodiments.factory_intrinsics",
        description="Read a stereo camera unit's factory calibration into sx.calibration/v1.",
    )
    commands = parser.add_subparsers(dest="vendor", required=True)
    zed = commands.add_parser("zed", help="a Stereolabs ZED, by serial")
    zed.add_argument("--serial", required=True)
    zed.add_argument("--body", required=True, help="the body the unit is placed in, by name")
    zed.add_argument("--left", default="head_left", help="the body's left-eye camera")
    zed.add_argument("--right", default="head_right", help="the body's right-eye camera")
    zed.add_argument("--record", type=Path, help="a saved record instead of fetching it")
    zed.add_argument("--out", type=Path, required=True, help="where the documents are written")
    arguments = parser.parse_args(argv)
    record = (
        arguments.record.read_bytes()
        if arguments.record is not None
        else fetch_zed_record(arguments.serial)
    )
    try:
        factory = zed_pair_calibration(
            _body(arguments.body), arguments.left, arguments.right, record, arguments.serial
        )
    except CalibrationError as error:
        raise SystemExit(str(error)) from error
    arguments.out.mkdir(parents=True, exist_ok=True)
    (arguments.out / f"{arguments.serial}.conf").write_bytes(record)
    written: dict[str, str] = {}
    for name, calibration in (
        (arguments.left, factory.left),
        (arguments.right, factory.right),
        (f"{arguments.left}->{arguments.right}", factory.left_from_right),
    ):
        identity = str(calibration_id(calibration))
        (arguments.out / f"{identity}.json").write_text(
            json.dumps(calibration_to_dict(calibration), indent=2, sort_keys=True) + "\n"
        )
        written[name] = f"{identity}.json"
    json.dump({"record": f"{arguments.serial}.conf", "calibrations": written}, sys.stdout)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
