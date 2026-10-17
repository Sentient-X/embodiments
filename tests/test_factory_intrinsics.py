"""A RealSense unit's factory record is a camera-unit calibration labelled as the vendor's."""

import json
from pathlib import Path
from typing import cast

import pytest
from sx_contracts.identity import JsonObject, content_digest

from sx_embodiments.calibration import (
    CalibrationError,
    CalibrationMethod,
    EvidenceKind,
    calibration_from_dict,
    calibration_to_dict,
)
from sx_embodiments.composition import compose_build
from sx_embodiments.factory_intrinsics import factory_calibration
from sx_embodiments.instance import RobotInstance, resolve
from sx_embodiments.known.humanoid import (
    HEAD_INFRARED_RIG,
    SENTIENT_HUMANOID_UPPER_BODY_INFRARED_BUILD,
)
from sx_embodiments.media import media_document
from sx_embodiments.parts import CameraModality

#: ``sx_realsense::FactoryIntrinsics`` renders this from LuViRA's published D435i colour dump
#: (``ilaydayaman/LuViRA_Dataset`` ``db30d579``, ``vision/camera_factory_cal.txt``, CC-BY-4.0).
FIXTURE = (
    Path(__file__).resolve().parents[1]
    / "drivers/sx-realsense/tests/fixtures/factory-luvira-d435i-color.json"
)


def _record() -> dict[str, object]:
    return cast("dict[str, object]", json.loads(FIXTURE.read_text()))


def test_a_real_d435_factory_record_is_a_vendor_measured_unit_calibration() -> None:
    record = _record()
    calibration = factory_calibration(record)
    assert calibration.unit == "000000000000/color"
    assert (calibration.width, calibration.height) == (640, 480)
    fx, _, cx, _, fy, cy, *_ = calibration.image_from_camera
    assert (fx, fy, cx, cy) == (
        606.9658813476562,
        605.6773071289062,
        310.7396545410156,
        242.00755310058594,
    )
    # An inverse Brown-Conrady model with no coefficients does nothing.
    assert (calibration.distortion_model, calibration.distortion_coefficients) == ("none", ())
    measurement = calibration.measurement
    assert measurement.method is CalibrationMethod.VENDOR_FACTORY
    assert measurement.measured_at is None
    assert [(item.kind, item.reference) for item in measurement.evidence] == [
        (EvidenceKind.SEALED_DOCUMENT, content_digest(cast("JsonObject", record)))
    ]
    assert calibration_from_dict(calibration_to_dict(calibration)) == calibration


def test_the_sdk_models_become_the_platform_vocabulary_or_are_refused() -> None:
    coefficients = [0.1, -0.2, 0.001, 0.002, 0.03]
    brown = factory_calibration(
        _record() | {"distortion": "brown_conrady", "coefficients": coefficients}
    )
    assert brown.distortion_model == "plumb_bob"
    assert list(brown.distortion_coefficients) == coefficients
    kannala = factory_calibration(
        _record() | {"distortion": "kannala_brandt4", "coefficients": [*coefficients[:4], 0.0]}
    )
    assert kannala.distortion_model == "kannala_brandt"
    assert list(kannala.distortion_coefficients) == coefficients[:4]
    for bad in (
        {"distortion": "inverse_brown_conrady", "coefficients": coefficients},
        {"distortion": "modified_brown_conrady"},
        {"distortion": "ftheta"},
        {"unit": "000000000000/infrared_1"},
        {"stream": "depth", "unit": "000000000000/depth"},
        {"serial": "00-00", "unit": "00-00/color"},
        {"coefficients": [0.0] * 4},
        {"fx": float("nan")},
        {"vendor": "luxonis"},
        {"extra": 1},
    ):
        with pytest.raises(CalibrationError):
            factory_calibration(_record() | bad)


def test_the_head_infrared_pair_is_one_track_of_two_infrared_imagers() -> None:
    body = compose_build(
        SENTIENT_HUMANOID_UPPER_BODY_INFRARED_BUILD,
        name="sentient-humanoid-upper-body-infrared",
        label="Sentient humanoid upper body, head infrared pair",
    )
    assert {camera.name: camera.camera.modality for camera in body.cameras} == {
        "head_left": CameraModality.INFRARED,
        "head_right": CameraModality.INFRARED,
    }
    instance = RobotInstance.of(body.id, {}, ())
    plan = media_document(body, resolve(body, instance, ()), HEAD_INFRARED_RIG)
    tracks = cast("list[dict[str, object]]", plan["tracks"])
    assert len(tracks) == 1
    views = cast("list[dict[str, object]]", tracks[0]["views"])
    assert [view["mount_frame"] for view in views] == [
        "head_camera_infrared_1_optical",
        "head_camera_infrared_2_optical",
    ]
