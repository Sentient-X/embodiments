"""Distal joint bend and absolute fingertip objectives from calibrated landmarks."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise

import numpy as np
from numpy.typing import NDArray

from ._ego_temporal import native_gaps, smooth_acceleration

type Floats = NDArray[np.float64]
type Indices = NDArray[np.int64]
type Booleans = NDArray[np.bool_]


@dataclass(frozen=True)
class ArticulationTargets:
    landmarks: Indices
    sites: Indices
    angles: Floats
    controls: Booleans

    @classmethod
    def from_landmarks(
        cls,
        mapped: Indices,
        side_count: int,
        points: Floats,
        times: Indices,
        fps: float | None,
        ancestor: Booleans,
        camera: int | None,
        coupling: Floats | None,
    ) -> ArticulationTargets:
        index_of = {int(landmark): i for i, landmark in enumerate(mapped)}
        # OpenPose indexes each finger in order from knuckle to tip after wrist 0.
        triplets: list[tuple[int, int, int]] = []
        for side in range(side_count):
            for finger in range(5):
                base = side * 21 + 4 * finger
                triplet = (base + 2, base + 3, base + 4)
                if all(i in index_of for i in triplet):
                    triplets.append(triplet)
        landmarks = np.array(triplets, dtype=np.int64).reshape(-1, 3)
        sites = np.array(
            [[index_of[i] for i in triple] for triple in landmarks], dtype=np.int64
        ).reshape(-1, 3)
        segments = np.diff(points.reshape(len(times), -1, 3)[:, landmarks], axis=2)
        first, second = segments[:, :, 0], segments[:, :, 1]
        angles = np.arctan2(
            np.linalg.norm(np.cross(first, second), axis=-1), np.sum(first * second, axis=-1)
        )
        angles[
            (np.linalg.norm(first, axis=-1) < 1e-6) | (np.linalg.norm(second, axis=-1) < 1e-6)
        ] = np.nan
        intervals = np.diff(times).astype(np.float64) * 1e-9
        gaps = native_gaps(intervals, fps)
        for digit in range(len(landmarks)):
            valid = np.isfinite(angles[:, digit])
            bounds = np.r_[0, np.flatnonzero(gaps | (valid[1:] != valid[:-1])) + 1, len(times)]
            for start, stop in pairwise(bounds):
                if valid[start] and stop - start >= 3:
                    angles[start:stop, digit] = smooth_acceleration(
                        angles[start:stop, digit, None], intervals[start : stop - 1]
                    )[:, 0]

        wrist_sites = [index_of[21 * side] for side in range(side_count)]
        protected_sites = wrist_sites if camera is None else [*wrist_sites, camera]

        def independent(ancestry: Booleans) -> Booleans:
            return ancestry if coupling is None else np.any(coupling[ancestry] != 0, axis=0)

        finger_ancestry = np.asarray(ancestor[sites.ravel()].any(axis=0))
        wrist_and_camera = np.asarray(ancestor[protected_sites].any(axis=0))
        controls = independent(finger_ancestry) & ~independent(wrist_and_camera)
        return cls(landmarks, sites, angles, controls)

    def residuals(
        self,
        row: int,
        points: Floats,
        poses: Floats,
        jacobian: Floats,
        coupling: Floats | None,
    ) -> tuple[Floats, Floats]:
        observed = np.isfinite(self.angles[row])
        sites = self.sites[observed]
        bones = np.diff(poses[sites], axis=1)
        lengths = np.linalg.norm(bones, axis=2)
        keep = (lengths > 1e-6).all(1)
        u, v = (bones[keep] / lengths[keep, :, None]).transpose(1, 0, 2)
        dot = (u * v).sum(1)
        sine = np.linalg.norm(np.cross(u, v), axis=1)
        # Angle gradient of atan2(norm(cross(u, v)), dot(u, v)) on the two unit segments.
        denominator = np.maximum(sine, 1e-5)
        proximal = -(v - dot[:, None] * u) / (lengths[keep, 0] * denominator)[:, None]
        distal = -(u - dot[:, None] * v) / (lengths[keep, 1] * denominator)[:, None]
        blocks = np.diff(jacobian[sites[keep]], axis=1)
        bend_jacobian = np.einsum("ni,nik->nk", proximal, blocks[:, 0]) + np.einsum(
            "ni,nik->nk", distal, blocks[:, 1]
        )
        tips = sites[keep, 2]
        tip_jacobian = jacobian[tips].reshape(-1, jacobian.shape[-1])
        if coupling is not None:
            tip_jacobian = tip_jacobian @ coupling
            bend_jacobian = bend_jacobian @ coupling
        # Convert the angular residual to tip-equivalent metres using this
        # robot's actual segment lengths. Position carries four times that weight.
        lever = 4.0 * lengths[keep].sum(1)
        angular_error = self.angles[row, observed][keep] - np.arctan2(sine, dot)
        tip_error = points[self.landmarks[observed][keep, 2]] - poses[tips]
        return (
            np.r_[16.0 * tip_error.reshape(-1), lever * angular_error],
            np.vstack((16.0 * tip_jacobian, lever[:, None] * bend_jacobian)),
        )
