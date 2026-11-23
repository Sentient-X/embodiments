"""Fit both interior finger bends and the absolute tip using the mapped robot linkage."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

from ._ego_temporal import (
    _relieve_penetration,  # pyright: ignore[reportPrivateUsage] -- same solver CAD projection
    native_gaps,
    smooth_acceleration,
)

if TYPE_CHECKING:
    from ._ego_collision import CollisionModel
    from .ego_retarget import (
        _Sites,  # pyright: ignore[reportPrivateUsage] -- native solver FK interface
    )

Floats = NDArray[np.float64]
Indices = NDArray[np.int64]
Bools = NDArray[np.bool_]


def bends(points: Floats) -> Floats:
    segments = np.diff(points, axis=-2)
    first, second = segments[..., :-1, :], segments[..., 1:, :]
    angle = np.arctan2(np.linalg.norm(np.cross(first, second), axis=-1), (first * second).sum(-1))
    lengths = np.linalg.norm(segments, axis=-1)
    return np.where((lengths[..., :-1] > 1e-6) & (lengths[..., 1:] > 1e-6), angle, np.nan)


@dataclass(frozen=True)
class Digits:
    """Complete digit chains and exclusive controls, including affine mimic leaders."""

    landmarks: Indices
    sites: Indices
    columns: Indices
    active: Bools
    controls: Indices
    native_units: Floats

    @classmethod
    def mapped(cls, model: _Sites, mapped: Indices) -> Digits | None:
        lookup = {int(point): i for i, point in enumerate(mapped)}
        ancestry = model.ancestor
        if model.robot.coupling is not None:
            ancestry = (ancestry @ np.abs(model.robot.coupling)) > 0
        landmarks: list[list[int]] = []
        sites: list[list[int]] = []
        controls: list[Indices] = []
        for first in mapped:
            slot = int(first) % 21
            if slot not in (1, 5, 9, 13, 17):
                continue
            digit = list(range(int(first), int(first) + 4))
            if any(landmark not in lookup for landmark in digit):
                continue
            chain = [lookup[landmark] for landmark in digit]
            # Any control moving another mapped site, wrist or optical frame stays fixed.
            remaining = np.ones(len(ancestry), dtype=np.bool_)
            remaining[chain] = False
            private = ancestry[chain].any(0) & ~ancestry[remaining].any(0)
            if model.robot.coupling is not None:
                # A control can also drive an unscored sibling through URDF mimic.
                # Exclude it even if none of that sibling's bodies is a mapped site.
                outside = ~np.asarray(model.ancestor[chain].any(0))
                private &= ~(model.robot.coupling[outside] != 0).any(0)
            columns = np.flatnonzero(private)
            if len(columns):
                landmarks.append(digit)
                sites.append(chain)
                controls.append(columns)
        if not controls:
            return None
        widths: Indices = np.asarray([len(c) for c in controls], dtype=np.int64)
        active: Bools = np.arange(int(widths.max()))[None, :] < widths[:, None]
        cols = np.zeros(active.shape, dtype=np.int64)
        cols[active] = np.concatenate(controls)
        # Express translation in finger lengths and rotation in radians so linear
        # and rotary drivers contribute comparable step and smoothness costs.
        neutral = model.positions(model.robot.neutral)[np.asarray(sites)]
        scale = np.linalg.norm(np.diff(neutral, axis=1), axis=-1).mean(axis=1).clip(1e-6)
        native_units = np.ones(len(model.robot.names))
        for digit, columns in enumerate(controls):
            for axis in columns:
                if model.slide[model.model.joint(model.robot.names[int(axis)]).id]:
                    native_units[axis] = scale[digit]
        return cls(
            np.asarray(landmarks), np.asarray(sites), cols, active, cols[active], native_units
        )

    def subset(self, indices: list[int]) -> Digits:
        columns = self.columns[indices]
        active = self.active[indices]
        return Digits(
            self.landmarks[indices],
            self.sites[indices],
            columns,
            active,
            columns[active],
            self.native_units,
        )

    def converge(
        self, model: _Sites, starting: Floats, reference: Floats, target_angles: Floats
    ) -> Floats:
        """Bounded damped Gauss-Newton on independent digit blocks for one exposure."""
        q = starting.copy()
        robot = model.robot
        columns = self.columns
        digit_ids = np.arange(len(columns))[:, None]
        identity = np.eye(columns.shape[1])
        for _ in range(18):
            world = model.positions(q)
            points = world[self.sites]
            jac = model.jacobian(world)
            if robot.coupling is not None:
                jac = jac @ robot.coupling
            per_digit = jac[self.sites].transpose(0, 3, 1, 2)[digit_ids, columns]
            jac = (
                per_digit.transpose(0, 2, 3, 1)
                * (self.active * self.native_units[columns])[:, None, None, :]
            )
            bones = np.diff(points, axis=1)
            lengths = np.linalg.norm(bones, axis=-1)
            valid = (lengths > 1e-6).all(axis=1) & np.isfinite(target_angles).all(axis=1)
            if not valid.any():
                break
            axes = bones / np.maximum(lengths[:, :, None], 1e-6)
            u, v = axes[:, :-1], axes[:, 1:]
            cosine = (u * v).sum(-1)
            # Differentiate atan2(|u x v|, u.v) with respect to each segment.
            sine = np.maximum(1e-5, np.linalg.norm(np.cross(u, v), axis=-1))
            du = -(v - cosine[..., None] * u) / np.maximum(lengths[:, :-1] * sine, 1e-10)[..., None]
            dv = -(u - cosine[..., None] * v) / np.maximum(lengths[:, 1:] * sine, 1e-10)[..., None]
            segment_jac = np.diff(jac, axis=1)
            grad = np.einsum("ndi,ndik->ndk", du, segment_jac[:, :-1])
            grad += np.einsum("ndi,ndik->ndk", dv, segment_jac[:, 1:])
            scale = np.maximum(1e-6, 1.5 * lengths.mean(axis=1))
            objective = np.concatenate((jac[:, -1] / scale[:, None, None], grad), axis=1)
            residual = np.concatenate(
                (
                    (reference[:, -1] - points[:, -1]) / scale[:, None],
                    target_angles - bends(points),
                ),
                axis=1,
            )
            residual[~valid] = 0
            transposed = objective.swapaxes(1, 2)
            rhs = transposed @ residual[..., None]
            hessian = transposed @ objective + 0.012 * identity
            direction = np.linalg.solve(hessian, rhs)[..., 0]
            outward = ((q[columns] <= robot.lower[columns] + 1e-7) & (direction < 0)) | (
                (q[columns] >= robot.upper[columns] - 1e-7) & (direction > 0)
            )
            free = self.active & ~outward
            system = hessian * free[:, :, None] * free[:, None, :] + identity * (~free)[:, :, None]
            delta = np.linalg.solve(system, rhs * free[:, :, None])[..., 0]
            delta *= np.minimum(1, 0.24 / np.maximum(1e-9, np.max(np.abs(delta), axis=1)))[:, None]
            q[self.controls] = np.clip(
                q[self.controls] + delta[self.active] * self.native_units[self.controls],
                robot.lower[self.controls],
                robot.upper[self.controls],
            )
            if np.max(np.abs(delta)) < 1e-6:
                break
        return q


def _candidate_geometry(
    model: _Sites, digits: Digits, source: Floats, target_angles: Floats, candidates: Floats
) -> tuple[Floats, Floats]:
    n, count, nq = candidates.shape
    points = np.stack([model.positions(q)[digits.sites] for q in candidates.reshape(-1, nq)])
    achieved = points.reshape(n, count, *points.shape[1:])
    angles = bends(achieved) - target_angles[:, None]
    coverage = np.isfinite(angles).sum(axis=(2, 3)).clip(1)
    angular_loss = np.nansum(angles**2, axis=(2, 3)) / coverage
    tips = np.linalg.norm(achieved[:, :, :, -1] - source[:, None, :, -1], axis=-1)
    return angular_loss, tips


def _transitions(candidates: Floats, controls: Indices, dt: Floats) -> Floats:
    q = candidates[:, :, controls]
    return (q[1:, None, :, :] - q[:-1, :, None, :]) / dt[:, None, None, None]


def _path(
    score: Floats, velocities: Floats, bounds: Floats, dt: Floats, *, polish: bool
) -> Indices:
    """Choose a continuous feasible curve. Retain the motion history needed for each derivative."""
    n = len(score)
    speed_ok = np.asarray((np.abs(velocities) <= bounds + 1e-6).all(-1), dtype=np.bool_)
    pairs = score[0, :, None] + score[1, None, :]
    if not polish:
        pairs += 0.004 * np.mean(velocities[0] ** 2, axis=-1)
    value = np.where(speed_ok[0], pairs, np.inf)
    history = 2
    parents: list[Indices] = []
    previous: Floats | None = None
    for row in range(2, n):
        dv = velocities[row - 1][None, ...] - velocities[row - 2][:, :, None, :]
        if polish:
            acceleration = dv / ((dt[row - 1] + dt[row - 2]) / 2)
            motion = 0.00005 * np.mean(acceleration**2, axis=-1)
            if row == 2:
                value = value[:, :, None] + motion
                history = 3
            else:
                assert previous is not None
                jerk = (acceleration[None, ...] - previous[:, :, :, None, :]) / (
                    dt[row - 3 : row].mean()
                )
                trial = value[:, :, :, None] + 0.0000005 * np.mean(jerk**2, axis=-1)
                parents.append(np.argmin(trial, axis=0))
                value = trial.min(axis=0) + motion
            previous = acceleration
        else:
            trial = value[:, :, None] + 0.03 * np.mean(dv**2, axis=-1)
            parents.append(np.argmin(trial, axis=0))
            value = trial.min(axis=0) + 0.004 * np.mean(velocities[row - 1] ** 2, axis=-1)
        value = np.where(speed_ok[row - 1], value + score[row], np.inf)
    if not np.isfinite(value).any():
        raise ValueError("digit shape refinement found no continuous velocity-bounded seed path")
    state = np.zeros(n, dtype=np.int64)
    state[-history:] = np.unravel_index(value.argmin(), value.shape)
    for row in reversed(range(len(parents))):
        state[row] = parents[row][tuple(state[row + 1 : row + history + 1])]
    return state


def _guarded_path(
    model: _Sites,
    collision: CollisionModel,
    digits: Digits,
    reference: Floats,
    target_angles: Floats,
    original_tips: Floats,
    seed: Floats,
    desired: Floats,
    dt: Floats,
    limit_m: float,
    fixed: Bools,
    *,
    polish: bool,
) -> Floats:
    robot = model.robot
    fractions = np.linspace(0, 1, 3 if polish else 9)
    candidates = seed[:, None] + fractions[None, :, None] * (desired - seed)[:, None]
    permitted = np.ones(candidates.shape[:2], dtype=np.bool_)
    for row, states in enumerate(candidates):
        # These equal lower and upper bounds freeze wrists, head and unrelated links
        # during contact relief, including for a robot with affine mimic coordinates.
        lower = seed[row].copy()
        upper = lower.copy()
        lower[digits.controls] = robot.lower[digits.controls]
        upper[digits.controls] = robot.upper[digits.controls]
        for k in range(1, len(states)):
            pose, contact = _relieve_penetration(
                collision, states[k], lower, upper, robot.velocity, model.full_q, robot.coupling
            )
            states[k] = pose
            permitted[row, k] = contact.unallowed <= 1e-6
    angles, tips = _candidate_geometry(model, digits, reference, target_angles, candidates)
    originally_within = np.all(~np.isfinite(original_tips) | (original_tips <= limit_m), axis=-1)
    within = np.all(~np.isfinite(tips) | (tips <= limit_m), axis=-1)
    permitted &= ~originally_within[:, None] | within
    # The caller's original pose remains a clearance-checked fallback. Its measured
    # contacts, including any pre-existing violations, are audited on final output.
    score = (
        0.02 * angles
        if polish
        else (
            angles
            + 0.12 * np.nansum((tips / limit_m) ** 2, axis=-1) / np.isfinite(tips).sum(-1).clip(1)
        )
    )
    permitted[fixed, 1:] = False
    score[~permitted] = np.inf
    if len(seed) == 1:
        return candidates[:, np.argmin(score[0])]
    units = digits.native_units[digits.controls]
    speed = _transitions(candidates, digits.controls, dt) / units
    states = _path(score, speed, robot.velocity[digits.controls] / units, dt, polish=polish)
    return candidates[np.arange(len(seed)), states]


def refine_digits(
    model: _Sites,
    collision: CollisionModel | None,
    mapped: Indices,
    points: Floats,
    times: Indices,
    seed: Floats,
    fps: float | None,
    limit_m: float,
) -> Floats:
    """Fit measured digit bend and tip trajectories, preserving unrelated robot controls.

    Complete four-site fingers are optimized in their exclusive independent coordinates.
    Partial fingers, shared controls and wrist/head motion remain in the original fit.
    Derivative and displacement costs regularize the review trajectory. Actuator
    qualification requires separate hardware evidence.
    """
    if not np.isfinite(limit_m) or limit_m <= 0:
        raise ValueError(
            "digit_shape_limit_m must be a positive finite fingertip distance in metres"
        )
    if collision is None or not collision.checked:
        raise ValueError("digit shape fitting requires checked URDF collision geometry")
    robot = model.robot
    if seed.shape != (len(times), len(robot.names)) or not np.isfinite(seed).all():
        raise ValueError(
            "digit shape seed must carry finite independent coordinates on the native clock"
        )
    if (seed < robot.lower - 1e-9).any() or (seed > robot.upper + 1e-9).any():
        raise ValueError("digit shape seed exceeds independent joint position limits")
    intervals = np.diff(times).astype(np.float64) * 1e-9
    if np.any(np.abs(np.diff(seed, axis=0)) > intervals[:, None] * robot.velocity + 1e-8):
        raise ValueError("digit shape seed exceeds the native URDF velocity limits")
    digits = Digits.mapped(model, mapped)
    result = seed.copy()
    if digits is None:
        return result
    gaps = native_gaps(intervals, fps)
    points = points.reshape(len(times), -1, 3)
    availability = np.asarray(
        np.isfinite(bends(points[:, digits.landmarks])).all(-1), dtype=np.bool_
    )
    by_visibility: dict[bytes, list[int]] = {}
    for digit in range(len(digits.sites)):
        by_visibility.setdefault(availability[:, digit].tobytes(), []).append(digit)
    for indices in by_visibility.values():
        group = digits.subset(indices)
        present = availability[:, indices[0]]
        breaks = gaps | (present[1:] != present[:-1])
        boundaries = np.r_[0, np.flatnonzero(breaks) + 1, len(times)]
        for begin, end in pairwise(boundaries):
            if not present[begin]:
                continue
            # Include only the neighbouring *seed* pose on each side for the
            # velocity transition. Its target is absent and its q stays exact.
            # Only a digit's own coverage boundaries limit its observations.
            cut_before = bool(begin > 0 and gaps[begin - 1])
            cut_after = bool(end < len(times) and gaps[end - 1])
            left = int(begin) if cut_before else max(0, int(begin) - 1)
            right = int(end) if cut_after else min(len(times), int(end) + 1)
            saved = result[left:right].copy()
            core = slice(int(begin) - left, int(end) - left)
            reference = np.full((len(saved), len(group.sites), 4, 3), np.nan)
            reference[core] = points[begin:end, group.landmarks]
            target_angles = bends(reference)
            desired = saved.copy()
            for row in range(core.start, core.stop):
                desired[row] = group.converge(model, saved[row], reference[row], target_angles[row])
            controls = group.controls
            observed_dt = intervals[begin : end - 1]
            desired[core, controls] = smooth_acceleration(desired[core, controls], observed_dt)
            desired = np.clip(desired, robot.lower, robot.upper)
            original = np.stack([model.positions(q)[group.sites[:, -1]] for q in saved])
            original_tips = np.linalg.norm(original - reference[:, :, -1], axis=-1)
            fixed = np.ones(len(saved), dtype=np.bool_)
            fixed[core] = False
            # Both measured endpoints of a discontinuous camera clock retain the
            # original robot pose. Neither independent island sees its neighbour.
            if cut_before:
                fixed[core.start] = True
            if cut_after:
                fixed[core.stop - 1] = True
            dt = intervals[left : right - 1]
            path = _guarded_path(
                model,
                collision,
                group,
                reference,
                target_angles,
                original_tips,
                saved,
                desired,
                dt,
                limit_m,
                fixed,
                polish=False,
            )
            for _ in range(3):
                desired = path.copy()
                desired[core, controls] = smooth_acceleration(path[core, controls], observed_dt)
                desired = np.clip(desired, robot.lower, robot.upper)
                path = _guarded_path(
                    model,
                    collision,
                    group,
                    reference,
                    target_angles,
                    original_tips,
                    path,
                    desired,
                    dt,
                    limit_m,
                    fixed,
                    polish=True,
                )
            result[begin:end, controls] = path[core, controls]
    return result
