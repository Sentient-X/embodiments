"""The calibration solvers recover known intrinsics and mounts, and refuse what cannot.

Every case builds its evidence from a known truth: board corners projected through a known
lens from known poses (and, for ChArUco, rendered into images and detected), so each solve
must give the truth back within its tolerance.
"""

from __future__ import annotations

import math
from datetime import UTC, datetime

import cv2
import numpy as np
import pytest
from numpy.typing import NDArray

from sx_embodiments import embodiments
from sx_embodiments.calibrate import (
    CalibrationSolveError,
    CameraViews,
    CharucoBoard,
    Checkerboard,
    Detection,
    JointView,
    Lens,
    MountView,
    View,
    _rigid,
    carrier_of,
    measure_intrinsics,
    measure_mount,
    solve_intrinsics,
    solve_mounts,
)
from sx_embodiments.calibrate.intrinsics import IntrinsicsLimits
from sx_embodiments.calibrate.targets import Target
from sx_embodiments.calibration import (
    Calibration,
    CalibrationMethod,
    RigidReference,
    UnitSerial,
    calibration_from_dict,
    calibration_to_dict,
)
from sx_embodiments.instance import InstanceError, RobotInstance, Slot, SlotKind, resolve
from sx_embodiments.kinematics import Kinematics

AT = datetime(2026, 10, 17, 9, 0, tzinfo=UTC)
CHECKERBOARD = Checkerboard(columns=9, rows=6, square_m=0.025)
CHARUCO = CharucoBoard(columns=7, rows=5, square_m=0.04, marker_m=0.03, dictionary="DICT_5X5_100")


def _pose(rotvec: tuple[float, float, float], xyz: tuple[float, float, float]) -> _rigid.Matrix:
    return _rigid.from_vector(np.array((*rotvec, *xyz), dtype=np.float64))


def _sha(index: int) -> str:
    return f"{index:064x}"


def _observe(
    lens: Lens, target: Target, camera_from_target: _rigid.Matrix, margin: float = 2.0
) -> Detection | None:
    points = target.points()
    camera = (camera_from_target[:3, :3] @ points.T).T + camera_from_target[:3, 3]
    if np.any(camera[:, 2] < 0.05):
        return None
    pixels = lens.project(camera)
    inside = (
        (pixels[:, 0] > margin)
        & (pixels[:, 0] < lens.width - margin)
        & (pixels[:, 1] > margin)
        & (pixels[:, 1] < lens.height - margin)
    )
    return Detection.of(pixels) if bool(np.all(inside)) else None


def _board_views(
    lens: Lens, target: Target, count: int, seed: int, noise_px: float = 0.0
) -> list[View]:
    """Boards facing the camera from varied distances, tilts and image positions."""
    rng = np.random.default_rng(seed)
    centre = target.points().mean(axis=0)
    views: list[View] = []
    while len(views) < count:
        tilt = rng.uniform(-0.6, 0.6, size=2)
        roll = rng.uniform(-math.pi, math.pi)
        distance = rng.uniform(0.35, 0.9)
        offset = rng.uniform(-0.18, 0.18, size=2) * distance
        rotation = _rigid.rotation_from_vector(np.array((tilt[0], tilt[1], roll)))
        pose = np.eye(4)
        pose[:3, :3] = rotation
        pose[:3, 3] = np.array((offset[0], offset[1], distance)) - rotation @ centre
        detection = _observe(lens, target, pose)
        if detection is None:
            continue
        noisy = detection.pixels + rng.normal(0.0, noise_px, detection.pixels.shape)
        views.append(View(Detection(detection.ids, noisy), _sha(len(views))))
    return views


PINHOLE = Lens(
    1280,
    720,
    np.array(((910.0, 0.0, 641.5), (0.0, 908.0, 362.0), (0.0, 0.0, 1.0))),
    "plumb_bob",
    np.array((0.11, -0.24, 0.0008, -0.0005, 0.09)),
)
#: A wide pinhole, so a board spun on the wrist stays in view.
CHARUCO_LENS = Lens(
    1280, 720, np.array(((500.0, 0.0, 640.0), (0.0, 500.0, 360.0), (0.0, 0.0, 1.0))), "none",
    np.zeros(0),
)  # fmt: skip
FISHEYE = Lens(
    1280,
    800,
    np.array(((420.0, 0.0, 643.0), (0.0, 421.0, 398.0), (0.0, 0.0, 1.0))),
    "kannala_brandt",
    np.array((0.02, -0.01, 0.004, -0.001)),
)


@pytest.mark.parametrize("truth", [PINHOLE, FISHEYE], ids=["plumb_bob", "kannala_brandt"])
@pytest.mark.parametrize("noise_px", [0.0, 0.05])
def test_intrinsics_are_recovered_from_board_views(truth: Lens, noise_px: float) -> None:
    views = _board_views(truth, CHECKERBOARD, 24, seed=3, noise_px=noise_px)
    solve = solve_intrinsics(
        CHECKERBOARD,
        views,
        width=truth.width,
        height=truth.height,
        model=truth.model,
        unit=UnitSerial("cam-1"),
        measured_at=AT,
    )
    found = Lens.of(solve.calibration)
    probe = np.array(((0.3, -0.2, 1.0), (-0.5, 0.25, 1.0), (0.0, 0.0, 1.0)))
    if noise_px == 0.0:  # exact evidence: the truth itself
        assert found.k == pytest.approx(truth.k, abs=1e-3)
        assert found.coefficients == pytest.approx(truth.coefficients, abs=1e-4)
    else:  # noisy evidence: the same lens to within a pixel, residual at the noise
        assert found.project(probe) == pytest.approx(truth.project(probe), abs=1.0)
    measurement = solve.calibration.measurement
    assert measurement.method is CalibrationMethod.CHECKERBOARD
    assert measurement.residual is not None
    assert measurement.residual.value < 2.0 * noise_px + 1e-3
    assert len(measurement.evidence) == len(solve.kept) == 24
    assert calibration_from_dict(calibration_to_dict(solve.calibration)) == solve.calibration


def _render(
    board: CharucoBoard, lens: Lens, camera_from_target: _rigid.Matrix
) -> NDArray[np.uint8]:
    """The printed board as a distortion-free pinhole camera sees it."""
    scale = 20
    width_px, height_px = board.columns * board.square_m, board.rows * board.square_m
    printed = board.image(round(width_px * 1000 * scale / 10), round(height_px * 1000 * scale / 10))
    metres_per_px = width_px / printed.shape[1]
    board_from_px = np.array(
        ((metres_per_px, 0.0, 0.0), (0.0, metres_per_px, 0.0), (0.0, 0.0, 1.0))
    )
    rt = np.column_stack((camera_from_target[:3, :2], camera_from_target[:3, 3]))
    homography = lens.k @ rt @ board_from_px
    return np.asarray(
        cv2.warpPerspective(
            printed, homography, (lens.width, lens.height), borderValue=(255,), flags=cv2.INTER_AREA
        ),
        dtype=np.uint8,
    )


def test_charuco_corners_detected_in_rendered_images_give_the_lens_back() -> None:
    truth = Lens(1280, 720, PINHOLE.k, "none", np.zeros(0))
    rng = np.random.default_rng(11)
    views: list[View] = []
    while len(views) < 14:
        tilt = rng.uniform(-0.5, 0.5, size=2)
        rotation = _rigid.rotation_from_vector(np.array((tilt[0], tilt[1], rng.uniform(-0.4, 0.4))))
        pose = np.eye(4)
        pose[:3, :3] = rotation
        centre = CHARUCO.points().mean(axis=0)
        pose[:3, 3] = np.array((*rng.uniform(-0.08, 0.08, size=2), rng.uniform(0.45, 0.8))) - (
            rotation @ centre
        )
        detection = CHARUCO.detect(_render(CHARUCO, truth, pose))
        if detection is not None and detection.ids.shape[0] >= 12:
            views.append(View(detection, _sha(len(views))))
    solve = solve_intrinsics(
        CHARUCO,
        views,
        width=1280,
        height=720,
        model="plumb_bob",
        unit=UnitSerial("cam-2"),
        measured_at=AT,
        limits=IntrinsicsLimits(minimum_views=10, max_view_rms_px=1.0),
    )
    assert solve.calibration.measurement.method is CalibrationMethod.CHARUCO
    assert Lens.of(solve.calibration).k == pytest.approx(truth.k, rel=1e-2)


def test_too_few_views_or_an_unknown_model_are_refused() -> None:
    views = _board_views(PINHOLE, CHECKERBOARD, 5, seed=1)
    with pytest.raises(CalibrationSolveError, match="at least 20"):
        solve_intrinsics(
            CHECKERBOARD, views, width=1280, height=720, model="plumb_bob",
            unit=UnitSerial("c"), measured_at=AT,
        )  # fmt: skip
    with pytest.raises(CalibrationSolveError, match="unknown distortion model"):
        solve_intrinsics(
            CHECKERBOARD, views * 5, width=1280, height=720, model="brown",
            unit=UnitSerial("c"), measured_at=AT,
        )  # fmt: skip


# --- mounts -----------------------------------------------------------------------------------


def _mount_views(
    lens: Lens,
    target: Target,
    carrier_from_camera: _rigid.Matrix,
    holder_from_target: _rigid.Matrix,
    carriers_from_holder: list[_rigid.Matrix],
) -> tuple[MountView, ...]:
    views: list[MountView] = []
    for carrier_from_holder in carriers_from_holder:
        camera_from_target = (
            _rigid.inverse(carrier_from_camera) @ carrier_from_holder @ holder_from_target
        )
        detection = _observe(lens, target, camera_from_target)
        if detection is not None:
            views.append(MountView(carrier_from_holder, detection))
    return tuple(views)


def _flange_poses(seed: int, count: int) -> list[_rigid.Matrix]:
    """Base-from-flange poses of an arm waving above a table."""
    rng = np.random.default_rng(seed)
    poses: list[_rigid.Matrix] = []
    for _ in range(count):
        rotvec = np.array((math.pi, 0.0, 0.0)) + rng.uniform(-0.35, 0.35, size=3)
        poses.append(_pose(tuple(rotvec), (rng.uniform(0.3, 0.5), rng.uniform(-0.15, 0.15), 0.55)))
    return poses


def _mismatch(first: _rigid.Matrix, second: _rigid.Matrix) -> tuple[float, float]:
    return float(np.linalg.norm(first[:3, 3] - second[:3, 3])), _rigid.rotation_angle(first, second)


@pytest.mark.parametrize("target", [CHECKERBOARD, CHARUCO], ids=["checkerboard", "charuco"])
def test_an_eye_in_hand_mount_is_recovered(target: Target) -> None:
    """A wrist camera over a board fixed to the table: carrier flange, holder base."""
    flange_from_camera = _pose((0.05, -0.08, 1.6), (0.03, -0.01, 0.06))
    base_from_board = _pose((0.0, 0.0, 0.3), (0.32, -0.12, 0.0))
    flanges = _flange_poses(5, 30)
    views = _mount_views(
        PINHOLE, target, flange_from_camera, base_from_board, [_rigid.inverse(p) for p in flanges]
    )
    assert len(views) >= 12
    camera = CameraViews("wrist", PINHOLE, views)
    nominal = flange_from_camera @ _pose((0.02, -0.03, 0.04), (0.004, 0.006, -0.005))
    initial = {"wrist": nominal} if target.ambiguous else None
    if target.ambiguous:  # a checkerboard arrives in either order
        camera = camera.with_views(
            [view.reversed() if index % 3 == 0 else view for index, view in enumerate(views)]
        )
    solve = solve_mounts(target, (camera,), initial=initial)
    translation, rotation = _mismatch(solve.carrier_from_camera["wrist"], flange_from_camera)
    assert translation < 1e-6 and rotation < 1e-6
    assert float(np.max(solve.per_view_rms_px(target)["wrist"])) < 1e-4


def test_an_eye_to_base_mount_is_recovered_from_hand_eye_alone() -> None:
    """A fixed camera watching a board in the gripper: carrier base, holder flange."""
    base_from_camera = _pose((math.pi, 0.0, 0.05), (0.4, 0.02, 1.35))
    # The board below the flange, facing back up at the camera.
    flange_from_board = _pose((math.pi, 0.0, 0.2), (-0.08, -0.06, 0.05))
    flanges = _flange_poses(9, 40)
    views = _mount_views(PINHOLE, CHARUCO, base_from_camera, flange_from_board, flanges)
    assert len(views) >= 12
    solve = solve_mounts(CHARUCO, (CameraViews("overhead", PINHOLE, views),))
    translation, rotation = _mismatch(solve.carrier_from_camera["overhead"], base_from_camera)
    assert translation < 1e-6 and rotation < 1e-6


def test_views_that_never_turn_the_target_are_refused() -> None:
    flange_from_camera = _pose((0.05, -0.08, 1.6), (0.03, -0.01, 0.06))
    base_from_board = _pose((0.0, 0.0, 0.3), (0.32, -0.12, 0.0))
    still = [_rigid.inverse(_flange_poses(5, 1)[0]) @ _pose((0, 0, 0), (dx, 0, 0)) for dx in
             (0.0, 0.01, 0.02, 0.03)]  # fmt: skip
    views = _mount_views(PINHOLE, CHARUCO, flange_from_camera, base_from_board, still)
    with pytest.raises(CalibrationSolveError, match="relative to the camera's carrier"):
        solve_mounts(CHARUCO, (CameraViews("wrist", PINHOLE, views),))


def test_a_checkerboard_without_a_nominal_mount_is_refused() -> None:
    flange_from_camera = _pose((0.05, -0.08, 1.6), (0.03, -0.01, 0.06))
    base_from_board = _pose((0.0, 0.0, 0.3), (0.32, -0.12, 0.0))
    views = _mount_views(
        PINHOLE,
        CHECKERBOARD,
        flange_from_camera,
        base_from_board,
        [_rigid.inverse(p) for p in _flange_poses(5, 30)],
    )
    with pytest.raises(CalibrationSolveError, match="ambiguous"):
        solve_mounts(CHECKERBOARD, (CameraViews("wrist", PINHOLE, views),))


# --- the humanoid through its robot instance ----------------------------------------------------

HUMANOID = embodiments["sentient-humanoid-upper-body"]
HEAD = Slot(SlotKind.CAMERA, "head")


def _instance(*calibrations: Calibration) -> RobotInstance:
    return RobotInstance.of(HUMANOID.id, {HEAD: UnitSerial("d435-0042")}, calibrations)


def test_the_humanoid_head_camera_rides_its_head() -> None:
    kinematics = Kinematics.of(HUMANOID.urdf_bytes)
    (camera,) = HUMANOID.cameras
    assert camera.camera.model == "realsense_d435"
    assert carrier_of(kinematics, camera.frame) == "head_link"


def test_the_humanoid_head_cameras_mount_is_measured_through_its_instance() -> None:
    """A board on a stand before the robot; the torso and neck turn the head across it."""
    kinematics = Kinematics.of(HUMANOID.urdf_bytes)
    (camera,) = HUMANOID.cameras
    nominal = _rigid.matrix(kinematics.pose("head_link", camera.frame))
    correction = _pose((0.01, -0.02, 0.015), (0.004, -0.003, 0.002))
    head_from_camera = nominal @ correction
    lens = Lens.of(camera.camera.optics)
    zero = {"torso_joint": 0.0, "waist_roll_joint": 0.0, "waist_pitch_joint": 0.0,
            "neck_yaw_joint": 0.0, "head_joint": 0.0}  # fmt: skip
    base_from_camera = _rigid.matrix(kinematics.pose("base_link", "head_link", zero)) @ (
        head_from_camera
    )
    # The board 0.9 m along the camera's view at the zero pose, facing it.
    base_from_board = base_from_camera @ _pose((0.0, math.pi, 0.0), (0.1, 0.075, 0.9))
    rng = np.random.default_rng(21)
    views: list[JointView] = []
    while len(views) < 16:
        positions = {
            **zero,
            "torso_joint": rng.uniform(-0.3, 0.3),
            "neck_yaw_joint": rng.uniform(-0.25, 0.25),
            "head_joint": rng.uniform(-0.2, 0.2),
        }
        head_from_base = _rigid.matrix(kinematics.pose("head_link", "base_link", positions))
        camera_from_board = _rigid.inverse(head_from_camera) @ head_from_base @ base_from_board
        detection = _observe(lens, CHECKERBOARD, camera_from_board)
        if detection is not None:
            views.append(JointView(positions, detection, _sha(len(views))))
    measurement = measure_mount(
        HUMANOID,
        _instance(),
        (),
        "head",
        holder="base_link",
        target=CHECKERBOARD,
        views=views,
        measured_at=AT,
    )
    calibration = measurement.calibration
    assert (calibration.parent_frame, calibration.child_frame) == ("head_link", camera.frame)
    assert calibration.reference is RigidReference.DESCRIPTION
    assert calibration.description_sha256 == HUMANOID.urdf.sha256
    translation, rotation = _mismatch(_rigid.matrix(calibration.parent_from_child), correction)
    assert translation < 1e-6 and rotation < 1e-6
    assert calibration.units == (UnitSerial("d435-0042"),)
    # The instance selects it, and resolving binds it to the body's own frames.
    resolved = resolve(HUMANOID, _instance(calibration), (calibration,))
    assert resolved.frames[0].calibration == calibration


def test_the_humanoid_head_cameras_intrinsics_are_its_units() -> None:
    lens = Lens(1280, 720, PINHOLE.k, "plumb_bob", PINHOLE.coefficients)
    views = _board_views(lens, CHECKERBOARD, 20, seed=8)
    solve = measure_intrinsics(HUMANOID, _instance(), "head", CHECKERBOARD, views, measured_at=AT)
    calibration = solve.calibration
    assert (calibration.unit, calibration.width, calibration.height) == ("d435-0042", 1280, 720)
    resolved = resolve(HUMANOID, _instance(calibration), (calibration,))
    assert resolved.camera("head").calibration == calibration


def test_a_camera_slot_without_a_unit_or_a_body_without_the_camera_is_refused() -> None:
    views = _board_views(PINHOLE, CHECKERBOARD, 20, seed=8)
    empty = RobotInstance.of(HUMANOID.id, {}, ())
    with pytest.raises(InstanceError, match="no unit in camera slot"):
        measure_intrinsics(HUMANOID, empty, "head", CHECKERBOARD, views, measured_at=AT)
    with pytest.raises(InstanceError, match="declares no camera"):
        measure_intrinsics(HUMANOID, _instance(), "wrist", CHECKERBOARD, views, measured_at=AT)


def test_a_mount_against_a_target_on_the_same_rigid_body_is_refused() -> None:
    with pytest.raises(CalibrationSolveError, match="cannot see the mount"):
        measure_mount(
            HUMANOID, _instance(), (), "head", holder="head_link", target=CHECKERBOARD,
            views=(), measured_at=AT,
        )  # fmt: skip


# --- every body ---------------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(embodiments))
def test_every_declared_camera_can_be_calibrated(name: str) -> None:
    """Each camera's nominal optics make a lens and its frame rides a rigid carrier link."""
    embodiment = embodiments[name]
    kinematics = Kinematics.of(embodiment.urdf_bytes)
    for camera in embodiment.cameras:
        lens = Lens.of(camera.camera.optics)
        centre = lens.project(np.array(((0.0, 0.0, 1.0),)))[0]
        assert centre == pytest.approx(lens.k[:2, 2])
        carrier = carrier_of(kinematics, camera.frame)
        assert kinematics.is_rigid(carrier, camera.frame), (name, camera.name)


def test_views_turning_about_one_axis_only_are_refused() -> None:
    """A turn about one axis commutes with a shift along it: the mount cannot be seen."""
    flange_from_camera = _pose((0.05, -0.08, 1.6), (0.03, -0.01, 0.06))
    base_from_board = _pose((0.0, 0.0, 0.3), (0.32, -0.12, 0.0))
    start = _flange_poses(5, 1)[0]
    spun = [
        _rigid.inverse(start @ _pose((0.0, 0.0, angle), (0.0, 0.0, 0.0)))
        for angle in (0.0, 0.2, 0.4, 0.6)
    ]
    views = _mount_views(CHARUCO_LENS, CHARUCO, flange_from_camera, base_from_board, spun)
    assert len(views) == 4
    with pytest.raises(CalibrationSolveError, match="about one axis"):
        solve_mounts(CHARUCO, (CameraViews("wrist", CHARUCO_LENS, views),))
