"""The YUBI jaw angle from the ArUco tags its own wrist camera sees on the two fingers.

A second measurement of the joint the jaw encoder reads, for when the encoder is dead, folded,
drifting or relative. Its output is the encoder's own quantity, the finger joint angle in
radians, so the jaw law that turns an encoder reading into a gap applies unchanged.

Each finger carries one 15 mm ``DICT_4X4_50`` marker that turns with the finger. Each finger
link pivots on its own hinge, and the two fingers are a mimic pair, so the two markers turn
by equal and opposite amounts. Measured on yubi-0006 (2026-11-25), each marker's rotation
about its hinge axis was 1.02 and 0.98 times the encoder angle over 43 degrees.

1. Pose: undistort a marker's corners through the lens, then run IPPE-square
   perspective-n-point with the printed side. The rotation is precise; the depth is not used.
2. Reference: a marker's pose at a known joint angle and its hinge axis are fixed by where it
   is glued. :func:`fit_reference` learns them per rig and side from any stretch where the
   encoder worked. :func:`closed_stop_reference` takes the narrowest pose seen as the joint's
   lower limit, for a rig with no working-encoder history.
3. Reading: each visible marker's rotation from its reference pose about its hinge gives the
   joint angle. The two readings must agree (the mirror check), and they are averaged.
4. Track, offline: :mod:`sx_embodiments.jaw_fusion` fuses the frames' readings with the
   encoder's reads, whichever of the two saw the jaw, into one angle with its deviation.

Held out on yubi-0006 (borrowed intrinsics), smoothed two-tag readings matched the encoder
within 1.34 mm of gap at the 95th percentile; fused with it, 1.44 mm over a grasp take.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from itertools import pairwise
from typing import cast

import cv2
import numpy as np
from numpy.typing import NDArray

from .calibrate.lens import Lens

type Vector = NDArray[np.float64]
type Matrix = NDArray[np.float64]

#: OpenCV's 4x4 dictionaries share their first fifty codes; YUBI's markers are IDs 0 to 3.
YUBI_JAW_DICTIONARY = "DICT_4X4_50"
#: The printed side of a YUBI finger marker (the outer black square), in metres.
YUBI_MARKER_SIDE_M = 0.015
#: The marker pair each YUBI wrist camera sees, one per finger.
YUBI_JAW_MARKERS: Mapping[str, tuple[int, int]] = {"right": (0, 1), "left": (2, 3)}

#: A marker whose corners reproject worse than this is not a reliable pose.
MAX_REPROJECTION_PX = 1.5
#: A face-on marker's two IPPE tilts fit almost equally; the right one may fit a little worse.
MAX_ALTERNATE_REPROJECTION_PX = 4.0
#: Two markers whose joint angles differ by more than this disagree: a bad frame or a broken
#: gripper. Measured spread on a sound unit: 2.76 degrees at p95.
MAX_MIRROR_DISAGREEMENT_RAD = math.radians(6.0)
#: A reference needs this many working-encoder sightings of a marker over this much travel.
MIN_REFERENCE_SIGHTINGS = 60
MIN_REFERENCE_TRAVEL_RAD = math.radians(15.0)
#: Frames to look at before deciding a recording has no jaw markers at all.
NO_TAGS_PROBE_FRAMES = 300


class JawTagError(Exception):
    """A layout, frame or recording cannot give a jaw angle."""


@dataclass(frozen=True, slots=True)
class JawTagLayout:
    """Which markers are on the two fingers, and how big they are printed."""

    first_id: int
    second_id: int
    marker_side_m: float = YUBI_MARKER_SIDE_M
    dictionary: str = YUBI_JAW_DICTIONARY

    def __post_init__(self) -> None:
        if self.first_id == self.second_id or min(self.first_id, self.second_id) < 0:
            raise JawTagError("the two finger markers need distinct non-negative IDs")
        if not (math.isfinite(self.marker_side_m) and 0.002 <= self.marker_side_m <= 0.1):
            raise JawTagError(f"marker side {self.marker_side_m!r} m is not a printed tag size")
        if not hasattr(cv2.aruco, self.dictionary):
            raise JawTagError(f"OpenCV has no ArUco dictionary {self.dictionary!r}")

    @classmethod
    def yubi(cls, side: str) -> JawTagLayout:
        try:
            first, second = YUBI_JAW_MARKERS[side]
        except KeyError as error:
            raise JawTagError(f"YUBI has no {side!r} hand") from error
        return cls(first, second)

    @property
    def ids(self) -> tuple[int, int]:
        return (self.first_id, self.second_id)

    def detector(self) -> cv2.aruco.ArucoDetector:
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        dictionary = cv2.aruco.getPredefinedDictionary(int(getattr(cv2.aruco, self.dictionary)))
        return cv2.aruco.ArucoDetector(dictionary, params)

    def corners_m(self) -> Vector:
        """The marker's corners in its own frame, in OpenCV's detection order."""
        h = self.marker_side_m / 2.0
        return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


@dataclass(frozen=True, slots=True)
class TagPose:
    """One finger marker's pose in the camera frame."""

    marker_id: int
    rotation: Matrix
    center_m: Vector
    reprojection_px: float
    #: IPPE's other solution for a planar marker (a mirrored tilt), when it has one. A marker
    #: seen face-on can take either; the one whose face points like the reference is kept.
    alternate: Matrix | None = None

    def candidates(self) -> tuple[Matrix, ...]:
        return (self.rotation,) if self.alternate is None else (self.rotation, self.alternate)

    def facing(self, normal: Vector) -> Matrix:
        """The rotation (of the two) whose face normal is closest to ``normal``."""
        if self.alternate is None:
            return self.rotation
        if float(self.alternate[:, 2] @ normal) > float(self.rotation[:, 2] @ normal):
            return self.alternate
        return self.rotation


def locate_tag(marker_id: int, pixels: Vector, lens: Lens, layout: JawTagLayout) -> TagPose | None:
    """One marker from its four detected corners (pixels, OpenCV order), or None if poor."""
    count, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        layout.corners_m(),
        lens.normalized(pixels),
        np.eye(3, dtype=np.float64),
        np.zeros(5, dtype=np.float64),
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    solutions: list[tuple[float, Matrix, Vector]] = []
    for rvec, tvec in zip(list(rvecs)[:count], list(tvecs)[:count], strict=True):
        rotation = cast("Matrix", cv2.Rodrigues(rvec)[0])
        center = np.asarray(tvec, np.float64).reshape(3)
        in_camera: Matrix = (rotation @ layout.corners_m().T).T + center
        offsets = lens.project(in_camera) - pixels
        error = math.sqrt(float(np.sum(offsets * offsets)) / len(offsets))
        if math.isfinite(error) and center[2] > 0:
            solutions.append((error, rotation, center))
    solutions.sort(key=lambda solution: solution[0])
    if not solutions or solutions[0][0] > MAX_REPROJECTION_PX:
        return None
    error, rotation, center = solutions[0]
    alternate = (
        solutions[1][1]
        if len(solutions) > 1 and solutions[1][0] <= MAX_ALTERNATE_REPROJECTION_PX
        else None
    )
    return TagPose(marker_id, rotation, center, error, alternate)


def sight_tags(gray: NDArray[np.uint8], lens: Lens, layout: JawTagLayout) -> dict[int, TagPose]:
    """The finger markers in one frame, each located once; a duplicated or poor one is dropped."""
    if gray.ndim != 2 or gray.shape != (lens.height, lens.width):
        raise JawTagError(
            f"frame {gray.shape} does not match the {lens.width}x{lens.height} calibration"
        )
    corners, ids, _ = layout.detector().detectMarkers(gray)
    if len(corners) == 0:  # OpenCV returns no ids (None) when it finds nothing
        return {}
    found: dict[int, list[Vector]] = {}
    for marker_id, quad in zip(np.asarray(ids).ravel().tolist(), corners, strict=True):
        if marker_id in layout.ids:
            found.setdefault(int(marker_id), []).append(np.asarray(quad, np.float64).reshape(4, 2))
    out: dict[int, TagPose] = {}
    for marker_id, quads in found.items():
        if len(quads) == 1 and (pose := locate_tag(marker_id, quads[0], lens, layout)):
            out[marker_id] = pose
    return out


def _rotation_vector(matrix: Matrix) -> Vector:
    return np.asarray(cv2.Rodrigues(matrix)[0], np.float64).reshape(3)


@dataclass(frozen=True, slots=True)
class MarkerReference:
    """Where one marker sits on its finger: its pose at ``angle_rad`` and its hinge axis.

    The joint angle of a later pose ``R`` is ``angle_rad + axis . log(reference^T R)``.
    """

    marker_id: int
    reference: tuple[float, ...]  # row-major 3x3
    axis: tuple[float, float, float]
    angle_rad: float

    def joint_angle(self, pose: TagPose) -> float:
        reference = np.asarray(self.reference, np.float64).reshape(3, 3)
        turn = _hinge_turn(reference, pose, np.asarray(self.axis))
        return self.angle_rad + float(turn @ np.asarray(self.axis))


class ReferenceOrigin(StrEnum):
    ENCODER = "encoder"
    CLOSED_STOP = "closed_stop"


@dataclass(frozen=True, slots=True)
class JawReference:
    """One rig side's two marker references, and how they were fixed."""

    layout: JawTagLayout
    first: MarkerReference
    second: MarkerReference
    origin: ReferenceOrigin

    def markers(self) -> tuple[MarkerReference, MarkerReference]:
        return (self.first, self.second)


def _geodesic(a: Matrix, b: Matrix) -> float:
    return float(np.linalg.norm(_rotation_vector(a.T @ b)))


def _continuous_family(options: Sequence[tuple[Matrix, ...]], start: int) -> list[int]:
    """The candidate per sighting that starts at ``start`` and turns least frame to frame."""
    inf = math.inf
    cost = [0.0 if j == start else inf for j in range(len(options[0]))]
    back: list[list[int]] = []
    for prev, cur in pairwise(options):
        step = [
            min(range(len(prev)), key=lambda j, r=r: cost[j] + _geodesic(prev[j], r)) for r in cur
        ]
        cost = [cost[j] + _geodesic(prev[j], r) for j, r in zip(step, cur, strict=True)]
        back.append(step)
    choice = [int(np.argmin(cost))]
    for step in reversed(back):
        choice.append(step[choice[-1]])
    choice.reverse()
    return choice


def _off_hinge(rotations: Sequence[Matrix]) -> float:
    """How far a sequence of orientations is from turning about one fixed axis (median, rad)."""
    turns = np.array([_rotation_vector(rotations[0].T @ r) for r in rotations], np.float64)
    if len(turns) < 3:
        return 0.0
    axis = np.asarray(np.linalg.svd(turns, full_matrices=False)[2][0], np.float64)
    off = np.asarray(np.linalg.norm(turns - np.outer(turns @ axis, axis), axis=1), np.float64)
    return float(np.median(off))


def consistent_poses(poses: Sequence[TagPose | None]) -> list[TagPose | None]:
    """Each sighting with IPPE's tilt chosen consistently across the sequence.

    A planar marker has two IPPE solutions (mirrored tilts), and a frame-by-frame choice
    flips between them. Each family of choices that turns least from frame to frame is
    smooth, and each turns about a fixed axis, so neither smoothness nor the hinge picks one.
    Keep the smooth family that agrees most often with IPPE's better-fitting solution.
    """
    seen = [i for i, pose in enumerate(poses) if pose is not None]
    if not seen:
        return list(poses)
    options = [cast("TagPose", poses[i]).candidates() for i in seen]
    families = [_continuous_family(options, start) for start in range(len(options[0]))]
    # Both families turn about a fixed axis (the mirror of one hinge is another), so the
    # image decides: the true family is IPPE's better-fitting solution in most frames.
    choice = min(
        families,
        key=lambda family: (
            -sum(pick == 0 for pick in family),
            _off_hinge([options[k][pick] for k, pick in enumerate(family)]),
        ),
    )
    out = list(poses)
    for i, pick in zip(seen, choice, strict=True):
        pose = cast("TagPose", poses[i])
        rotation = pose.candidates()[pick]
        out[i] = TagPose(pose.marker_id, rotation, pose.center_m, pose.reprojection_px, None)
    return out


def consistent_frames(
    layout: JawTagLayout, frames: Sequence[Mapping[int, TagPose]]
) -> list[dict[int, TagPose]]:
    """:func:`consistent_poses` for both finger markers across a recording's frames."""
    out: list[dict[int, TagPose]] = [{} for _ in frames]
    for marker in layout.ids:
        for i, pose in enumerate(consistent_poses([frame.get(marker) for frame in frames])):
            if pose is not None:
                out[i][marker] = pose
    return out


def _hinge_turn(reference: Matrix, pose: TagPose, axis: Vector) -> Vector:
    """The pose's turn from ``reference``, taking the IPPE solution a hinge turn best explains."""
    turns = [_rotation_vector(reference.T @ rotation) for rotation in pose.candidates()]
    return min(turns, key=lambda turn: float(np.linalg.norm(turn - (turn @ axis) * axis)))


#: Rounds of choosing each pose's IPPE solution and refitting the hinge axis.
_AXIS_ROUNDS = 4
#: A turn whose off-axis part exceeds this is not a hinge turn and does not fit the axis.
MAX_OFF_AXIS_RAD = math.radians(3.0)


def _hinge_fit(
    reference: Matrix, poses: Sequence[TagPose]
) -> tuple[float, Vector, Vector, NDArray[np.bool_]]:
    """One hinge axis for the poses' turns from ``reference``.

    Returns the median off-axis turn, the axis, the turns and which poses fit it.
    """
    normal = reference[:, 2]
    turns = np.array(
        [_rotation_vector(reference.T @ pose.facing(normal)) for pose in poses], np.float64
    )
    axis = np.asarray(np.linalg.svd(turns, full_matrices=False)[2][0], np.float64)
    keep = np.ones(len(poses), bool)
    off = np.zeros(len(poses))
    for _ in range(_AXIS_ROUNDS):
        turns = np.array([_hinge_turn(reference, pose, axis) for pose in poses], np.float64)
        off = np.asarray(np.linalg.norm(turns - np.outer(turns @ axis, axis), axis=1), np.float64)
        keep = off <= max(MAX_OFF_AXIS_RAD, float(np.percentile(off, 80)))
        axis = np.asarray(np.linalg.svd(turns[keep], full_matrices=False)[2][0], np.float64)
    return float(np.median(off)), axis, turns, keep


def _marker_reference(
    marker_id: int, poses: Sequence[TagPose], angles: Vector, zero_index: int
) -> MarkerReference | None:
    # The reference pose may itself be IPPE's mirrored tilt; keep whichever of its two
    # solutions lets one hinge explain the most poses.
    fits = [(_hinge_fit(r, poses), r) for r in poses[zero_index].candidates()]
    (_, axis, turns, keep), reference = min(fits, key=lambda fit: fit[0][0])
    along = turns @ axis
    if float(np.ptp(along[keep])) < MIN_REFERENCE_TRAVEL_RAD:
        return None
    if np.corrcoef(along[keep], angles[keep])[0, 1] < 0:
        axis, along = -axis, -along
    return MarkerReference(
        marker_id,
        tuple(float(v) for v in reference.ravel()),
        (float(axis[0]), float(axis[1]), float(axis[2])),
        float(np.median((angles - along)[keep])),
    )


def fit_reference(
    layout: JawTagLayout,
    frames: Sequence[Mapping[int, TagPose]],
    encoder_angles_rad: Sequence[float],
) -> JawReference | None:
    """Learn a side's marker references from frames whose encoder angle is known and sound.

    ``encoder_angles_rad[i]`` is the encoder's joint angle at frame ``i`` (NaN where it is not
    trusted). None when either marker was seen too little or over too little travel.
    """
    angles = np.asarray(encoder_angles_rad, np.float64)
    frames = consistent_frames(layout, frames)
    refs: list[MarkerReference] = []
    for marker in layout.ids:
        rows = [i for i, frame in enumerate(frames) if marker in frame and math.isfinite(angles[i])]
        if len(rows) < MIN_REFERENCE_SIGHTINGS:
            return None
        known = angles[rows]
        poses = [frames[i][marker] for i in rows]
        made = _marker_reference(marker, poses, known, int(np.argmin(known)))
        if made is None:
            return None
        refs.append(made)
    return JawReference(layout, refs[0], refs[1], ReferenceOrigin.ENCODER)


def closed_stop_reference(
    layout: JawTagLayout, frames: Sequence[Mapping[int, TagPose]], closed_angle_rad: float = 0.0
) -> JawReference | None:
    """Fix the references without an encoder: the most-closed pose seen is ``closed_angle_rad``.

    The two markers turn oppositely, so their relative rotation grows as the jaw opens; the
    frame where it is smallest is the closed stop. Valid only for a recording that closes the
    jaw fully at least once; a worn pad moves that stop, which this cannot see.
    """
    first, second = layout.ids
    frames = consistent_frames(layout, frames)
    both = [i for i, frame in enumerate(frames) if first in frame and second in frame]
    if len(both) < MIN_REFERENCE_SIGHTINGS:
        return None
    # Each marker is glued at its own small angle, so the size of the markers' relative
    # rotation is not monotonic near the stop. Its turn about its own axis from any one frame
    # is (twice the joint angle, up to sign), and the markers are closer when the jaw is shut.
    normals = (frames[both[0]][first].rotation[:, 2], frames[both[0]][second].rotation[:, 2])
    relative = [
        frames[i][first].facing(normals[0]).T @ frames[i][second].facing(normals[1]) for i in both
    ]
    turns = np.array([_rotation_vector(relative[0].T @ r) for r in relative], np.float64)
    axis = np.asarray(
        np.linalg.svd(turns - turns.mean(axis=0), full_matrices=False)[2][0], np.float64
    )
    spread = np.asarray(turns @ axis, np.float64)
    distance = np.array(
        [np.linalg.norm(frames[i][first].center_m - frames[i][second].center_m) for i in both]
    )
    if np.corrcoef(spread, distance)[0, 1] < 0:
        spread = -spread
    # The stop is the 1st percentile, so one flipped (IPPE-ambiguous) pose cannot set it.
    floor = float(np.percentile(spread, 1.0))
    closed = int(np.argmin(np.abs(spread - floor)))
    refs: list[MarkerReference] = []
    for marker in layout.ids:
        poses = [frames[i][marker] for i in both]
        made = _marker_reference(marker, poses, spread, closed)
        if made is None:
            return None
        refs.append(MarkerReference(made.marker_id, made.reference, made.axis, closed_angle_rad))
    return JawReference(layout, refs[0], refs[1], ReferenceOrigin.CLOSED_STOP)


@dataclass(frozen=True, slots=True)
class JawReading:
    angle_rad: float
    #: How many of the two finger markers the reading came from.
    tags: int
    #: The two markers' disagreement (NaN with one marker).
    mirror_disagreement_rad: float


def read_frame(reference: JawReference, frame: Mapping[int, TagPose]) -> JawReading | None:
    """The joint angle one frame shows, or None when no marker (or no agreeing pair) is seen."""
    angles = [
        ref.joint_angle(frame[ref.marker_id])
        for ref in reference.markers()
        if ref.marker_id in frame
    ]
    if not angles:
        return None
    if len(angles) == 1:
        return JawReading(angles[0], 1, math.nan)
    disagreement = abs(angles[0] - angles[1])
    if disagreement > MAX_MIRROR_DISAGREEMENT_RAD:
        return None
    return JawReading((angles[0] + angles[1]) / 2.0, 2, disagreement)


def has_jaw_tags(
    gray_frames: Sequence[NDArray[np.uint8]], lens: Lens, layout: JawTagLayout
) -> bool:
    """Whether a recording's opening frames show either finger marker at all; stop early if not."""
    return any(sight_tags(gray, lens, layout) for gray in gray_frames[:NO_TAGS_PROBE_FRAMES])
