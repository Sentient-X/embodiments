"""Rigid transforms inside the solvers: 4x4 float arrays, and the 6-vectors a solve varies."""

from __future__ import annotations

from collections.abc import Sequence
from typing import cast

import numpy as np
from numpy.typing import NDArray

Matrix = NDArray[np.float64]


def matrix(transform: Sequence[float]) -> Matrix:
    """A row-major 16-tuple as a 4x4 array."""
    return np.asarray(transform, dtype=np.float64).reshape(4, 4)


def flat(value: Matrix) -> tuple[float, ...]:
    """A 4x4 array as the row-major 16-tuple the calibration kinds store, bottom row exact."""
    out = value.copy()
    out[3] = (0.0, 0.0, 0.0, 1.0)
    return tuple(float(entry) for entry in out.reshape(-1))


def inverse(value: Matrix) -> Matrix:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = value[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ value[:3, 3]
    return out


def rotation_from_vector(rotvec: NDArray[np.float64]) -> Matrix:
    """Rodrigues: the rotation of angle ``|rotvec|`` about ``rotvec``."""
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-12:
        x, y, z = (float(value) for value in rotvec)
        return np.eye(3, dtype=np.float64) + np.array(
            ((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)), dtype=np.float64
        )
    x, y, z = (float(value) / angle for value in rotvec)
    skew = np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)), dtype=np.float64)
    return (
        np.eye(3, dtype=np.float64) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)
    )


def vector_from_rotation(rotation: Matrix) -> NDArray[np.float64]:
    """The rotation vector of a rotation matrix, stable near 0 and pi."""
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    axis = np.array(
        (
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ),
        dtype=np.float64,
    )
    if angle < 1e-8:
        return axis / 2.0
    if np.pi - angle < 1e-4:
        # Near a half turn the antisymmetric part vanishes; read the axis off R + I.
        symmetric = (rotation + np.eye(3, dtype=np.float64)) / 2.0
        column = int(np.argmax(np.diag(symmetric)))
        direction = symmetric[:, column] / np.sqrt(max(float(symmetric[column, column]), 1e-12))
        if float(np.dot(direction, axis)) < 0.0:
            direction = -direction
        return direction * angle
    return axis * (angle / (2.0 * np.sin(angle)))


def to_vector(value: Matrix) -> NDArray[np.float64]:
    return np.concatenate((vector_from_rotation(value[:3, :3]), value[:3, 3]))


def from_vector(vector: NDArray[np.float64]) -> Matrix:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rotation_from_vector(vector[:3])
    out[:3, 3] = vector[3:6]
    return out


def rotation_angle(first: Matrix, second: Matrix) -> float:
    """The angle, in radians, of the rotation taking ``first`` to ``second``."""
    relative = first[:3, :3].T @ second[:3, :3]
    cosine = (float(np.trace(relative)) - 1.0) / 2.0
    return float(np.arccos(np.clip(cosine, -1.0, 1.0)))


def average(values: Sequence[Matrix]) -> Matrix:
    """The chordal mean rotation and the median translation of ``values``."""
    stacked = np.stack([value[:3, :3] for value in values])
    u, _singular, vt = cast("tuple[Matrix, Matrix, Matrix]", np.linalg.svd(stacked.sum(axis=0)))
    u = u.copy()
    rotation = u @ vt
    if float(np.linalg.det(rotation)) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rotation
    out[:3, 3] = np.median(np.stack([value[:3, 3] for value in values]), axis=0)
    return out
