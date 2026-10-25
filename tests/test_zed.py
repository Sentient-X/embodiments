"""The static upper body's ZED 2i: two eyes on the neck, calibrated by Stereolabs' own record."""

import configparser
import io
import math
from pathlib import Path
from typing import cast

import pytest
from sx_contracts.identity import stream_blob

from sx_embodiments import development_embodiments
from sx_embodiments.calibration import (
    CalibrationError,
    CalibrationMethod,
    EvidenceKind,
    UnitSerial,
    calibration_from_dict,
    calibration_to_dict,
)
from sx_embodiments.cameras import camera_label
from sx_embodiments.embodiment import Embodiment
from sx_embodiments.factory_intrinsics import zed_factory_calibration
from sx_embodiments.instance import RobotInstance, Slot, SlotKind, resolve
from sx_embodiments.kinematics import JointType, Kinematics
from sx_embodiments.known.static_upper_body import STATIC_UPPER_BODY_RIG
from sx_embodiments.known.zed import ZED_2I, optical_frame
from sx_embodiments.media import media_document

#: ``https://calib.stereolabs.com/?SN=30371653``, fetched 2026-10-25: the factory record of the
#: ZED 2i on the static upper body's neck.
FIXTURE = (
    Path(__file__).resolve().parents[1]
    / "drivers/sx-uvc-stereo/tests/fixtures/zed-2i-30371653.conf"
)
FRAMES = (optical_frame("left"), optical_frame("right"))


def _body() -> Embodiment:
    return development_embodiments["sentient-static-upper-body"]


def test_the_factory_record_is_each_eyes_rational_lens_and_the_right_eyes_pose() -> None:
    document = FIXTURE.read_bytes()
    factory = zed_factory_calibration(document, "30371653", 1280, 720, FRAMES)
    left, right = factory.left, factory.right
    assert (left.unit, right.unit) == ("30371653/left", "30371653/right")
    fx, _, cx, _, fy, cy, *_ = left.image_from_camera
    assert (fx, fy, cx, cy) == (531.47121315431, 531.54108728775, 636.32816079591, 338.999312596)
    assert left.distortion_model == right.distortion_model == "rational_polynomial"
    # OpenCV's order: k1 k2 p1 p2 k3 k4 k5 k6, from LEFT_DISTO.
    assert left.distortion_coefficients[:2] == (0.08267291685620068, 0.17224084040476442)
    assert left.distortion_coefficients[5] == 0.16984737229680713
    for calibration in (left, right):
        assert calibration.measurement.method is CalibrationMethod.VENDOR_FACTORY
        assert calibration.measurement.measured_at is None
        assert [(item.kind, item.reference) for item in calibration.measurement.evidence] == [
            (EvidenceKind.SEALED_DOCUMENT, stream_blob([document]).sha256)
        ]
        assert calibration_from_dict(calibration_to_dict(calibration)) == calibration
    stereo = factory.left_from_right
    assert (stereo.parent_frame, stereo.child_frame) == FRAMES
    translation = stereo.parent_from_child[3], stereo.parent_from_child[7]
    assert translation == pytest.approx((0.11978192582188, 7.648505802632155e-06))
    # The eyes are parallel to a quarter degree.
    assert math.degrees(math.acos(stereo.parent_from_child[0])) < 0.3
    vga = zed_factory_calibration(document, "30371653", 672, 376, FRAMES).left
    assert vga.image_from_camera[0] == 265.73560657716


def test_a_record_without_the_rational_terms_is_the_plumb_bob_it_states() -> None:
    ini = configparser.ConfigParser(interpolation=None)
    ini.optionxform = str  # type: ignore[assignment,method-assign]  # keys keep their case
    ini.read_string(FIXTURE.read_text())
    ini.remove_section("LEFT_DISTO")
    ini.remove_section("RIGHT_DISTO")
    out = io.StringIO()
    ini.write(out)
    legacy = out.getvalue().encode()
    left = zed_factory_calibration(legacy, "30371653", 1280, 720, FRAMES).left
    assert left.distortion_model == "plumb_bob"
    assert left.distortion_coefficients == (
        0.08267291685620068,
        0.17224084040476442,
        0.00015351052352872443,
        -0.00013096497163858409,
        0.17646420279149394,
    )


def test_what_is_not_one_zed_resolution_of_such_a_record_is_refused() -> None:
    document = FIXTURE.read_bytes()
    for bad in (
        lambda: zed_factory_calibration(document, "30371653", 1344, 376, FRAMES),
        lambda: zed_factory_calibration(document, "SN-1", 1280, 720, FRAMES),
        lambda: zed_factory_calibration(b"\xff", "30371653", 1280, 720, FRAMES),
        lambda: zed_factory_calibration(b"[STEREO]\nBaseline=1\n", "30371653", 1280, 720, FRAMES),
        lambda: zed_factory_calibration(
            document.replace(b"fx=531.47121315431", b"fx=nan"), "30371653", 1280, 720, FRAMES
        ),
    ):
        with pytest.raises(CalibrationError):
            bad()


def test_the_eyes_ride_the_neck_120_mm_apart_looking_out_of_the_lenses() -> None:
    body = _body()
    assert camera_label(ZED_2I) == "Stereolabs ZED 2i"
    assert {camera.name: camera.frame for camera in body.cameras} == {
        "head_left": "zed_left_optical",
        "head_right": "zed_right_optical",
    }
    kinematics = Kinematics.of(body.urdf_bytes)
    zero = {
        name: 0.0 for name, joint in kinematics.joints.items() if joint.type is not JointType.FIXED
    }
    left = kinematics.pose("ZED_camera", FRAMES[0], zero)
    right = kinematics.pose("ZED_camera", FRAMES[1], zero)
    assert (left[7], right[7]) == pytest.approx((0.06, -0.06))
    # The optical z axis is the ZED frame's x, out of the lenses; optical x is towards the right
    # eye.
    assert (left[2], left[6], left[10]) == pytest.approx((1.0, 0.0, 0.0), abs=1e-12)
    assert (left[0], left[4], left[8]) == pytest.approx((0.0, -1.0, 0.0), abs=1e-12)


def test_the_placed_zed_is_one_head_track_of_its_factory_eyes() -> None:
    body = _body()
    factory = zed_factory_calibration(FIXTURE.read_bytes(), "30371653", 1280, 720, FRAMES)
    calibrations = (factory.left, factory.right)
    units = {
        Slot(SlotKind.CAMERA, f"head_{eye}"): UnitSerial(f"30371653/{eye}")
        for eye in ("left", "right")
    }
    instance = RobotInstance.of(body.id, units, calibrations)
    plan = media_document(body, resolve(body, instance, calibrations), STATIC_UPPER_BODY_RIG)
    tracks = cast("list[dict[str, object]]", plan["tracks"])
    assert [track["track"] for track in tracks] == ["head"]
    views = cast("list[dict[str, object]]", tracks[0]["views"])
    assert [view["unit"] for view in views] == ["30371653/left", "30371653/right"]
    assert all("so101_camera_tilt_joint" in cast("list[str]", view["moves_with"]) for view in views)
    # A VGA calibration does not describe the HD720 eyes the body declares.
    vga = zed_factory_calibration(FIXTURE.read_bytes(), "30371653", 672, 376, FRAMES)
    with pytest.raises(CalibrationError):
        resolve(
            body, RobotInstance.of(body.id, units, (vga.left, vga.right)), (vga.left, vga.right)
        )
