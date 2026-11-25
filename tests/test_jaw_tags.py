"""The YUBI jaw angle from rendered finger markers that turn oppositely about their hinges."""

from __future__ import annotations

import math
from pathlib import Path
from typing import cast

import cv2
import numpy as np
import pytest
from numpy.typing import NDArray

from sx_embodiments.calibrate.lens import Lens
from sx_embodiments.jaw_tags import (
    JawReference,
    JawSampleSource,
    JawTagLayout,
    ReferenceOrigin,
    TagPose,
    closed_stop_reference,
    fit_reference,
    has_jaw_tags,
    sight_tags,
    solve_track,
    solved_spool_name,
    write_solved_spool,
)

LENS = Lens(
    640,
    480,
    np.array([[300.0, 0.0, 320.0], [0.0, 300.0, 240.0], [0.0, 0.0, 1.0]]),
    "none",
    np.zeros(0),
)
LAYOUT = JawTagLayout.yubi("right")
# Each finger's hinge, and the marker's lever from it, in the camera frame (metres). As on
# yubi-0006, the markers sit low in the view, tilted well away from the camera, and turn
# about their own normal.
HINGES = (np.array([-0.025, 0.040, 0.10]), np.array([0.025, 0.040, 0.10]))
LEVER = 0.03
GLUE = (math.radians(4.0), math.radians(-7.0))  # each marker glued a little askew


def _render(theta: float, *, hide: int | None = None) -> NDArray[np.uint8]:
    frame = np.full((480, 640), 255, np.uint8)
    tilt = math.radians(40.0)  # about the camera x axis, away from the lens
    facing = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(tilt), -math.sin(tilt)],
            [0.0, math.sin(tilt), math.cos(tilt)],
        ]
    ) @ np.diag([1.0, -1.0, -1.0])
    for index, (hinge, sign) in enumerate(zip(HINGES, (-1.0, 1.0), strict=True)):
        if index == hide:
            continue
        turn = -sign * theta  # the fingers swing apart, as on the rig
        normal = facing[:, 2]
        hinge_turn = np.asarray(cv2.Rodrigues(normal * turn)[0], np.float64)
        glue = np.asarray(cv2.Rodrigues(normal * GLUE[index])[0], np.float64)
        lever = hinge_turn @ (facing @ np.array([sign * LEVER * 0.3, LEVER, 0.0], np.float64))
        rotation = hinge_turn @ glue @ facing
        corners = np.asarray((rotation @ LAYOUT.corners_m().T).T + hinge + lever, np.float64)
        marker = cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), LAYOUT.ids[index], 120
        )
        source = np.array([[-0.5, -0.5], [119.5, -0.5], [119.5, 119.5], [-0.5, 119.5]], np.float32)
        target = LENS.project(corners).astype(np.float32)
        warp = cv2.getPerspectiveTransform(source, target)
        mask = cv2.warpPerspective(np.full_like(marker, 255), warp, (640, 480))
        warped = cv2.warpPerspective(marker, warp, (640, 480))
        frame[mask > 0] = warped[mask > 0]
    return frame


def _sweep() -> tuple[NDArray[np.float64], list[dict[int, TagPose]]]:
    theta = np.concatenate([np.zeros(8), np.linspace(0.0, 0.6, 150), np.linspace(0.6, 0.0, 150)])
    return np.asarray(theta, np.float64), [
        sight_tags(_render(float(a)), LENS, LAYOUT) for a in theta.tolist()
    ]


@pytest.fixture(scope="module")
def sweep() -> tuple[NDArray[np.float64], list[dict[int, TagPose]]]:
    return _sweep()


def _within(reference: JawReference, tolerance_rad: float) -> None:
    # Read a fresh sweep as the ingest node does: a whole track, frame to frame consistent.
    theta = np.concatenate([np.linspace(0.0, 0.57, 150), np.linspace(0.57, 0.0, 150)])
    for hide in (None, 0, 1):
        frames = [sight_tags(_render(float(a), hide=hide), LENS, LAYOUT) for a in theta.tolist()]
        track = solve_track(reference, [i * 33_333_333 for i in range(len(theta))], frames)
        error = np.abs(track.angle_rad - theta)
        assert float(np.percentile(error, 95)) < tolerance_rad, hide


def test_a_reference_learned_from_a_working_encoder_reads_the_joint(
    sweep: tuple[NDArray[np.float64], list[dict[int, TagPose]]],
) -> None:
    theta, frames = sweep
    reference = fit_reference(LAYOUT, frames, theta.tolist())
    assert reference is not None and reference.origin is ReferenceOrigin.ENCODER
    _within(reference, math.radians(1.0))


def test_without_an_encoder_the_closed_stop_fixes_the_joint(
    sweep: tuple[NDArray[np.float64], list[dict[int, TagPose]]],
) -> None:
    _, frames = sweep
    reference = closed_stop_reference(LAYOUT, frames)
    assert reference is not None and reference.origin is ReferenceOrigin.CLOSED_STOP
    _within(reference, math.radians(1.0))


def test_every_sample_says_where_it_came_from_and_the_spool_is_the_seals(
    sweep: tuple[NDArray[np.float64], list[dict[int, TagPose]]], tmp_path: Path
) -> None:
    theta, frames = sweep
    reference = cast("JawReference", fit_reference(LAYOUT, frames, theta.tolist()))
    shown = [dict(frame) for frame in frames]
    for i in (0, 1, 120, 121, 122):
        shown[i] = {}  # hidden at the start (held) and mid-sweep (interpolated)
    shown[180] = {k: v for k, v in frames[180].items() if k == LAYOUT.first_id}
    t_ns = [1_000_000_000 + i * 33_333_333 for i in range(len(frames))]
    track = solve_track(reference, t_ns, shown)
    assert track.source[0] is JawSampleSource.HELD
    assert track.source[121] is JawSampleSource.INTERPOLATED
    assert track.source[180] is JawSampleSource.ONE_TAG
    assert track.source[20] is JawSampleSource.BOTH_TAGS
    assert float(np.max(np.abs(track.angle_rad[2:] - theta[2:]))) < math.radians(1.5)

    spool = tmp_path / solved_spool_name("right")
    clamped = write_solved_spool(track, spool, (0.0, 0.9599))
    lines = spool.read_text().splitlines()
    assert lines[0] == "t_monotonic_ns,angle_rad,source"
    assert len(lines) == len(frames) + 1 and clamped >= 0
    assert lines[122].endswith(",interpolated") and lines[1].endswith(",held")


def test_a_recording_without_the_markers_stops_early() -> None:
    blank = np.full((480, 640), 200, np.uint8)
    assert not has_jaw_tags([blank] * 5, LENS, LAYOUT)
    assert has_jaw_tags([_render(0.2)], LENS, LAYOUT)
