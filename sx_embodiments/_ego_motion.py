"""Offline trajectory optimization on native exposures of a fixed-base robot.

Each Gauss-Newton step couples every exposure in a continuous clock segment.
The full nonlinear objective and sampled CAD are evaluated on the accepted step.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise
from types import ModuleType
from typing import TYPE_CHECKING, cast

import numpy as np
import scipy.linalg as linalg  # pyright: ignore[reportMissingTypeStubs] -- SciPy numerical extension.
from numpy.typing import NDArray
from sx_contracts.identity import JsonValue

from ._ego_governor import governed_start, select_path
from ._ego_temporal import native_gaps

if TYPE_CHECKING:
    from collections.abc import Callable, Mapping, Sequence

    from ._ego_collision import CollisionModel
    from ._ego_dynamics import EffortModel
    from .ego_retarget import (
        MappingProfile,
        _Sites,  # pyright: ignore[reportPrivateUsage] -- shared native FK kernel
    )

type Floats = NDArray[np.float64]
type Indices = NDArray[np.int64]
type Bools = NDArray[np.bool_]


@dataclass(frozen=True)
class MotionCheckpoint:
    """One accepted iterate on a source-bound clock segment; the caller binds its inputs."""

    first_index: int
    stop_index: int
    phase: str
    q: Floats
    objective: tuple[float, ...]
    linear_residuals: tuple[float, ...]
    stop: str | None


@dataclass(frozen=True)
class MotionReport:
    torque_before: Floats
    torque: Floats
    effort_limits: Floats
    speed_limits: Floats
    speed_edge_ok: Bools
    training_speed_ok: Bools
    training_effort_ok: Bools
    effort_model_available: bool
    effort_evaluated: bool
    details: dict[str, JsonValue]

    def document(self) -> dict[str, JsonValue]:
        return self.details.copy()


def _derivatives(t: Floats) -> tuple[Floats, Floats, Floats, Floats]:
    """Native divided differences. Row i describes the stencil starting at i."""
    h = np.diff(t)
    velocity = np.column_stack((-1 / h, 1 / h))
    h0, h1 = h[:-1], h[1:]
    middle_speed = np.column_stack(
        (-h1 / (h0 * (h0 + h1)), (h1 - h0) / (h0 * h1), h0 / (h1 * (h0 + h1)))
    )
    acceleration = np.column_stack((2 / (h0 * (h0 + h1)), -2 / (h0 * h1), 2 / (h1 * (h0 + h1))))
    jerk = np.pad(acceleration[1:], ((0, 0), (1, 0))) - np.pad(acceleration[:-1], ((0, 0), (0, 1)))
    jerk *= (3 / (t[3:] - t[:-3]))[:, None]
    return velocity, middle_speed, acceleration, jerk


def _apply(q: Floats, coefficients: Floats) -> Floats:
    n = len(coefficients)
    result = np.zeros((n, q.shape[1]))
    for k, column in enumerate(coefficients.T):
        result += column[:, None] * q[k : k + n]
    return result


def _unit(v: Floats) -> tuple[Floats, Floats]:
    size = np.linalg.norm(v, axis=-1, keepdims=True)
    return v / np.maximum(size, 1e-8), size


class _Normal:
    """Lower block-banded normal equations; no dense square trajectory matrix."""

    def __init__(self, n: int, m: int) -> None:
        self.gradient = np.zeros((n, m))
        self.lower = [np.zeros((max(0, n - k), m, m)) for k in range(4)]

    def spatial(self, r: Floats, j: Floats) -> None:
        jt = j.swapaxes(1, 2)
        self.gradient += (jt @ r[:, :, None])[:, :, 0]
        self.lower[0] += jt @ j

    def temporal(self, r: Floats, coefficients: Floats, weight: Floats) -> None:
        n = len(r)
        for i, a in enumerate(coefficients.T):
            self.gradient[i : i + n] += a[:, None] * weight * r
            for j in range(i + 1):
                diagonal = self.lower[i - j][j : j + n]
                idx = np.arange(diagonal.shape[1])
                diagonal[:, idx, idx] += (a * coefficients[:, j])[:, None] * weight**2

    def step(self, damping: Floats) -> tuple[Floats, float]:
        n, m = self.gradient.shape
        idx = np.arange(m)
        self.lower[0][:, idx, idx] += damping**2
        band = np.zeros((min(4 * m, n * m), n * m))
        row, col = np.indices((m, m))
        for lag, block in enumerate(self.lower):
            if not len(block):
                continue
            use = row >= col if lag == 0 else np.ones_like(row, dtype=bool)
            band[lag * m + row[use] - col[use], np.arange(len(block))[:, None] * m + col[use]] = (
                block[:, use]
            )
        linear = cast("ModuleType", linalg)
        direction = np.asarray(
            linear.solveh_banded(
                band, -self.gradient.ravel(), lower=True, overwrite_ab=True, check_finite=False
            ),
            dtype=np.float64,
        ).reshape(n, m)
        residual = self.gradient.copy()
        for lag, block in enumerate(self.lower):
            if not len(block):
                continue
            residual[lag:] += (block @ direction[: n - lag, :, None])[:, :, 0]
            if lag:
                residual[: n - lag] += (block.swapaxes(1, 2) @ direction[lag:, :, None])[:, :, 0]
        relative_residual = np.linalg.norm(residual) / max(np.linalg.norm(self.gradient), 1e-12)
        return direction, float(relative_residual)


@dataclass(frozen=True)
class _Direction:
    first: int
    second: int
    reference: Floats
    weight: float
    posture: bool


class _Tracking:
    def __init__(
        self,
        sites: _Sites,
        profile: MappingProfile,
        links: Sequence[str],
        points: Floats,
        camera: Floats,
        head: Floats,
        arms: Mapping[str, tuple[str, str, str]],
        body: Mapping[str, Floats],
    ) -> None:
        self.sites = sites
        where = {name: i for i, name in enumerate(links)}
        self.camera = where[profile.camera_optical_link] if profile.camera_optical_link else None
        self.rotations = camera
        self.positions: list[tuple[int, Floats, float]] = []
        self.directions: list[_Direction] = []
        wrists: list[int] = []
        hand_tolerance: list[float] = []
        for s, (side, mapping) in enumerate(profile.hands.items()):
            wrist = mapping[0]
            assert wrist is not None
            wrists.append(where[wrist])
            self.positions.append((where[wrist], points[:, s, 0], 220.0))
            hand_tolerance.append(0.025)
            for k in range(4, 21, 4):
                name = mapping[k]
                if name is not None:
                    self.positions.append((where[name], points[:, s, k], 85.0))
                    hand_tolerance.append(0.04)
            a, b = mapping[5], mapping[17]
            if a is not None and b is not None:
                self.directions.append(
                    _Direction(
                        where[a],
                        where[b],
                        self._reference(points[:, s, 17] - points[:, s, 5]),
                        2.0,
                        False,
                    )
                )
            if side in arms and side in body:
                a, b, c = (where[x] for x in arms[side])
                for seg, first, second in ((0, a, b), (1, b, c)):
                    target = body[side][:, seg + 1] - body[side][:, seg]
                    self.directions.append(
                        _Direction(first, second, self._reference(target), 2.2, True)
                    )
        self.hand_sites = np.asarray([site for site, _, _ in self.positions], dtype=np.int64)
        self.hand_targets = np.stack([target for _, target, _ in self.positions], axis=1)
        self.hand_tolerance = np.asarray(hand_tolerance, dtype=np.float64)
        if self.camera is not None:
            wrists.append(self.camera)
            self.positions.append((self.camera, head, 10.0))
        ancestry = sites.ancestor[wrists].any(axis=0)
        self.controls = np.flatnonzero(
            ancestry
            if sites.robot.coupling is None
            else np.any(sites.robot.coupling[ancestry] != 0, axis=0)
        )
        # Protect finger actuators even when a mimic follower lies on an unrelated limb.
        if sites.robot.coupling is not None:
            independent_outside = np.any(sites.robot.coupling[~ancestry] != 0, axis=0)
            self.controls = self.controls[~independent_outside[self.controls]]
        influence = sites.ancestor[self.hand_sites]
        if sites.robot.coupling is not None:
            influence = influence @ (sites.robot.coupling != 0)
        self.hand_influence = influence[:, self.controls]
        self.camera_influence = np.zeros(len(self.controls), dtype=bool)
        if self.camera is not None:
            ancestors = sites.ancestor[self.camera]
            if sites.robot.coupling is not None:
                ancestors = ancestors @ (sites.robot.coupling != 0)
            self.camera_influence = ancestors[self.controls]

    def camera_angle(self, i: int) -> float:
        assert self.camera is not None
        actual = self.sites.data.xmat[self.sites.ids[self.camera]].reshape(3, 3)
        return float(np.arccos(np.clip((np.sum(actual * self.rotations[i]) - 1) / 2, -1, 1)))

    @staticmethod
    def _reference(v: Floats) -> Floats:
        unit, length = _unit(v)
        return np.where(length > 1e-5, unit, np.nan)

    def cost(
        self,
        q: Floats,
        span: slice,
        mass: Floats,
        interval: Floats,
        normal: _Normal | None,
        metrics: dict[str, list[Floats]] | None = None,
    ) -> float:
        n, k, a = len(q), len(self.sites.ids), len(self.controls)
        poses = np.empty((n, k, 3))
        rotation = np.empty((n, 3, 3))
        jac = np.empty((n, k, 3, a)) if normal is not None else None
        axes = np.empty((n, 3, 3, a)) if normal is not None and self.camera is not None else None
        for t, pose in enumerate(q):
            sites = self.sites
            poses[t] = sites.positions(pose)
            if jac is not None:
                physical = sites.jacobian(poses[t])
                if sites.robot.coupling is not None:
                    physical = physical @ sites.robot.coupling
                jac[t] = physical[..., self.controls]
            if self.camera is not None:
                rotation[t] = sites.data.xmat[sites.ids[self.camera]].reshape(3, 3)
                if axes is not None:
                    active = sites.ancestor[self.camera] & ~sites.slide
                    axis = sites.data.xaxis * active[:, None]
                    for col in (1, 2):
                        angular = np.asarray(
                            np.cross(axis, rotation[t, :, col]), dtype=np.float64
                        ).T
                        if sites.robot.coupling is not None:
                            angular = angular @ sites.robot.coupling
                        axes[t, :, col] = angular[:, self.controls]
        cost = 0.0

        def term(delta: Floats, j: Floats | None, w: float, robust: float | None = None) -> None:
            nonlocal cost
            valid = np.asarray(np.isfinite(delta).all(axis=1))
            weight = np.where(valid, w * np.sqrt(mass), 0.0)
            finite = np.where(valid[:, None], delta, 0.0)
            squared = np.sum(finite**2, axis=1)
            if robust is not None:
                scale = np.sqrt(1 + squared / robust**2)
                cost += float(np.sum(weight**2 * 2 * squared / (1 + scale)))
                weight /= np.sqrt(scale)
            else:
                cost += float(np.sum(weight**2 * squared))
            r = finite * weight[:, None]
            if normal is not None and j is not None:
                normal.spatial(r, j * weight[:, None, None])

        for i, target, weight in self.positions:
            term(
                poses[:, i] - target[span],
                None if jac is None else jac[:, i],
                weight,
                0.035 if i != self.camera else None,
            )
        for d in self.directions:
            v, length = _unit(poses[:, d.second] - poses[:, d.first])
            j = None
            if jac is not None:
                diff = jac[:, d.second] - jac[:, d.first]
                j = (
                    diff - v[:, :, None] * np.einsum("ni,nik->nk", v, diff)[:, None, :]
                ) / np.maximum(length[:, :, None], 1e-8)
            target = d.reference[span]
            if d.posture and metrics is not None:
                angle = np.arccos(np.clip(np.sum(v * target, axis=1), -1, 1))
                metrics["arm_direction_error_deg"].append(np.rad2deg(angle[np.isfinite(angle)]))
            term(np.where(length > 1e-8, v - target, np.nan), j, d.weight)
        if self.camera is not None:
            for col in (1, 2):
                term(
                    rotation[:, :, col] - self.rotations[span, :, col],
                    None if axes is None else axes[:, :, col],
                    72.0,
                )
                # Native optical-axis rate from consecutive SLAM orientations.
                factor = 7.0 / np.sqrt(interval)
                error = rotation[:, :, col] - self.rotations[span, :, col]
                residual = np.diff(error, axis=0) * factor[:, None]
                cost += float(np.sum(residual**2))
                if axes is not None and normal is not None and len(interval):
                    prev = -axes[:-1, :, col] * factor[:, None, None]
                    nex = axes[1:, :, col] * factor[:, None, None]
                    pt, nt = prev.swapaxes(1, 2), nex.swapaxes(1, 2)
                    normal.gradient[:-1] += (pt @ residual[:, :, None])[:, :, 0]
                    normal.gradient[1:] += (nt @ residual[:, :, None])[:, :, 0]
                    normal.lower[0][:-1] += pt @ prev
                    normal.lower[0][1:] += nt @ nex
                    normal.lower[1] += nt @ prev
        return cost


class _Span:
    def __init__(
        self,
        q: Floats,
        span: slice,
        time: Floats,
        tracking: _Tracking,
        collision: CollisionModel | None,
        effort: EffortModel | None,
        pinned: Bools,
    ) -> None:
        self.seed, self.span, self.tracking = q, span, tracking
        self.effort, self.collision, self.pinned = effort, collision, pinned
        self.ids = tracking.controls
        self.rate = tracking.sites.robot.velocity[self.ids]
        self.operating_rate = (
            np.full(len(self.ids), np.inf)
            if effort is None
            else np.where(
                np.isfinite(effort.operating_velocity_limits[self.ids]),
                effort.operating_velocity_limits[self.ids],
                np.inf,
            )
        )
        self.h = np.diff(time)
        self.mass = (
            np.r_[self.h[0] / 2, (self.h[:-1] + self.h[1:]) / 2, self.h[-1] / 2]
            if len(self.h)
            else np.ones(1)
        )
        self.v, self.mid, self.acc, self.jerk = _derivatives(time)
        self.hand_target = tracking.hand_targets[span]
        original_sites = np.empty_like(self.hand_target)
        self.camera_budget = np.full(len(q), np.inf)
        for i, row in enumerate(q):
            original_sites[i] = tracking.sites.positions(row)[tracking.hand_sites]
            if tracking.camera is not None:
                angle = tracking.camera_angle(span.start + i)
                threshold = np.deg2rad(7)
                self.camera_budget[i] = threshold if angle <= threshold else angle + np.deg2rad(1)
        original_error = np.linalg.norm(original_sites - self.hand_target, axis=2)
        self.hand_budget = np.where(
            original_error <= tracking.hand_tolerance,
            tracking.hand_tolerance,
            original_error + np.maximum(0.004, 0.5 * (original_error - tracking.hand_tolerance)),
        )
        self.clearance = (
            np.zeros(len(q))
            if collision is None
            else np.array(
                [
                    max(1e-6, c.unallowed)
                    for c in collision.batch_at([tracking.sites.full_q(row) for row in q])
                ]
            )
        )

    def cost(self, q: Floats, normal: _Normal | None = None) -> float:
        total = self.tracking.cost(q, self.span, self.mass, self.h, normal)
        active = q[:, self.ids]
        ref_weight = np.sqrt(self.mass)[:, None] / (0.45 * self.rate)
        reference = ref_weight * (active - self.seed[:, self.ids])
        total += float(np.sum(reference**2))
        if normal is not None:
            m = len(self.ids)
            idx = np.arange(m)
            normal.lower[0][:, idx, idx] += ref_weight**2
            normal.gradient += reference * ref_weight
        for coefficients, timescale in ((self.v, 0.9), (self.acc, 0.28), (self.jerk, 0.034)):
            k = coefficients.shape[1] - 1
            # Integral of each native divided difference at its own support.
            duration = (
                np.convolve(self.h, np.ones(k) / k, mode="valid")
                if len(coefficients)
                else np.empty(0)
            )
            weight = np.sqrt(duration)[:, None] * timescale / self.rate
            residual = _apply(active, coefficients) * weight
            total += float(np.sum(residual**2))
            if normal is not None:
                normal.temporal(residual, coefficients, weight)
        if np.any(np.isfinite(self.operating_rate)):
            speed = _apply(active, self.v) / self.operating_rate
            hinge = np.maximum(np.abs(speed) - 0.85, 0.0)
            weight = 9.0 * np.sqrt(self.h)[:, None]
            residual = weight * np.sign(speed) * hinge
            total += float(np.sum(residual**2))
            if normal is not None:
                normal.temporal(residual, self.v, weight * (hinge > 0) / self.operating_rate)
        if self.effort is None or len(q) < 3:
            return total
        rated = np.isfinite(self.effort.effort_limits)
        if not np.any(rated):
            return total
        velocity, accel = _apply(q, self.mid), _apply(q, self.acc)
        # Blocking the assembly of the exact inverse dynamics Jacobians bounds
        # memory independently of recording length. Temporal rows still couple
        # the whole span in one normal system.
        for start in range(0, len(accel), 64):
            end = min(start + 64, len(accel))
            index = np.arange(start, end)
            torque = np.empty((len(index), int(rated.sum())))
            derivatives = (
                [np.empty((*torque.shape, len(self.ids))) for _ in range(3)]
                if normal is not None
                else []
            )
            for k, i in enumerate(index):
                if normal is None:
                    tor = self.effort.inverse(q[i + 1], velocity[i], accel[i])
                else:
                    tor, jq, jv, ja = self.effort.linearize(
                        q[i + 1], velocity[i], accel[i], columns=self.ids
                    )
                    for p in range(3):
                        derivatives[p][k] = self.mid[i, p] * jv[rated] + self.acc[i, p] * ja[rated]
                    derivatives[1][k] += jq[rated]
                torque[k] = tor[rated]
            limits = self.effort.effort_limits[rated]
            ratio = torque / limits
            magnitude = np.abs(ratio)
            # The rated effort is a ceiling, not an invitation to sit at it.
            # Keep a smooth quadratic load cost and increase the slope near the rating.
            excess = np.sign(ratio) * np.maximum(0, magnitude - 0.8)
            weight = 1.8 * np.sqrt(self.mass[index + 1, None])
            r = weight * (ratio + 5.0 * excess)
            total += float(np.sum(r * r))
            if normal is not None:
                factor = weight * (1.0 + 5.0 * (magnitude > 0.8)) / limits
                for jac in derivatives:
                    jac *= factor[:, :, None]
                # These rows start at 'start', unlike divided differences of the full span.
                for i, jac in enumerate(derivatives):
                    jt = jac.swapaxes(1, 2)
                    normal.gradient[start + i : end + i] += (jt @ r[:, :, None])[:, :, 0]
                    for j in range(i + 1):
                        normal.lower[i - j][start + j : end + j] += jt @ derivatives[j]
        return total

    def candidate(self, q: Floats, delta: Floats) -> Floats:
        robot = self.tracking.sites.robot
        # A trust region bounds the local FK/effort linearization. A separate
        # nonlinear line search decides whether the resulting motion helps.
        ids = self.ids
        delta = np.clip(delta, -0.12 * self.rate, 0.12 * self.rate)
        delta = np.clip(q[:, ids] + delta, robot.lower[ids], robot.upper[ids]) - q[:, ids]
        delta[self.pinned] = 0
        scales = np.ones_like(delta)
        previous = q.copy()
        threshold = self.clearance + 1e-12
        for iteration in range(12):
            for i in range(1, len(q)):
                scales[i] = np.minimum(scales[i], scales[i - 1] + self.h[i - 1] / 0.15)
            for i in range(len(q) - 2, -1, -1):
                scales[i] = np.minimum(scales[i], scales[i + 1] + self.h[i] / 0.15)
            proposal = q[:, ids] + scales * delta
            # The boxes between the feasible iterate and requested position
            # are mutually reachable because the current iterate belongs to
            # every box. Tighten reachable intervals and clip along the chain.
            lo, hi = np.minimum(q[:, ids], proposal), np.maximum(q[:, ids], proposal)
            travel = self.h[:, None] * self.rate
            for i in range(1, len(q)):
                lo[i] = np.maximum(lo[i], lo[i - 1] - travel[i - 1])
                hi[i] = np.minimum(hi[i], hi[i - 1] + travel[i - 1])
            for i in range(len(q) - 2, -1, -1):
                lo[i] = np.maximum(lo[i], lo[i + 1] - travel[i])
                hi[i] = np.minimum(hi[i], hi[i + 1] + travel[i])
            first = np.clip(proposal, lo, hi)
            second = first.copy()
            for i in range(1, len(q)):
                first[i] = np.clip(
                    first[i], first[i - 1] - travel[i - 1], first[i - 1] + travel[i - 1]
                )
            for i in range(len(q) - 2, -1, -1):
                second[i] = np.clip(second[i], second[i + 1] - travel[i], second[i + 1] + travel[i])
            candidate = q.copy()
            candidate[:, ids] = (first + second) * 0.5
            candidate[self.pinned] = q[self.pinned]
            invalid = False
            changed = np.flatnonzero(np.any(candidate != previous, axis=1))
            contacts = (
                []
                if self.collision is None
                else self.collision.batch_at(
                    [self.tracking.sites.full_q(candidate[i]) for i in changed]
                )
            )
            for k, i in enumerate(changed):
                tracking = self.tracking
                positions = tracking.sites.positions(candidate[i])[tracking.hand_sites]
                error = np.linalg.norm(positions - self.hand_target[i], axis=1)
                outside = error > self.hand_budget[i] + 1e-10
                active = np.asarray(tracking.hand_influence[outside].any(axis=0))
                unsafe = bool(np.any(outside))
                if (
                    tracking.camera is not None
                    and tracking.camera_angle(self.span.start + i) > self.camera_budget[i] + 1e-10
                ):
                    unsafe = True
                    active |= tracking.camera_influence
                if self.collision is not None:
                    contact = contacts[k]
                    if contact.unallowed > threshold[i]:
                        unsafe = True
                        blocked = contact.blocked_joints
                        if robot.coupling is not None:
                            blocked = np.asarray(np.any(robot.coupling[blocked] != 0, axis=0))
                        active |= blocked[ids]
                if unsafe:
                    invalid = True
                    # Restrict ancestors of the affected geometry so the unaffected
                    # arm and head can continue improving in the same exposure.
                    active &= candidate[i, ids] != q[i, ids]
                    if not np.any(active) or iteration >= 6:
                        active = np.ones(len(ids), dtype=np.bool_)
                    scales[i, active] = 0.0 if iteration >= 2 else 0.5 * scales[i, active]
                    # Keep this row in the next verification even if the new
                    # speed projection places it at the same offending pose.
                    candidate[i, 0] = np.nan
            if not invalid:
                return candidate
            previous = candidate
        return q.copy()

    def pose_feasible(self, values: Floats) -> Bools:
        tracking = self.tracking
        robot = tracking.sites.robot
        eligible = np.asarray(
            np.all((values >= robot.lower - 1e-9) & (values <= robot.upper + 1e-9), axis=1)
        )
        for i, row in enumerate(values):
            sites = tracking.sites.positions(row)[tracking.hand_sites]
            error = np.linalg.norm(sites - self.hand_target[i], axis=1)
            if np.any(error > self.hand_budget[i] + 1e-10):
                eligible[i] = False
            if tracking.camera is not None:
                eligible[i] &= (
                    tracking.camera_angle(self.span.start + i) <= self.camera_budget[i] + 1e-10
                )
        if self.collision is not None:
            rows = np.flatnonzero(eligible)
            contacts = self.collision.batch_at([tracking.sites.full_q(values[i]) for i in rows])
            eligible[rows] &= (
                np.array([c.unallowed for c in contacts]) <= self.clearance[rows] + 1e-12
            )
        return eligible

    def optimize(
        self,
        phase: str,
        initial: Floats,
        checkpoints: Mapping[tuple[int, int, str], MotionCheckpoint],
        on_checkpoint: Callable[[MotionCheckpoint], None] | None,
    ) -> tuple[Floats, list[float], list[float], str]:
        start, end = int(self.span.start), int(self.span.stop)
        q = initial.copy()
        history = [self.cost(q)]
        residuals: list[float] = []
        cached = checkpoints.get((start, end, phase))
        if cached is not None:
            if (
                (cached.first_index, cached.stop_index, cached.phase) != (start, end, phase)
                or cached.q.shape != q.shape
                or not np.isfinite(cached.q).all()
                or not 1 <= len(cached.objective) <= 37
                or not np.all(np.isfinite(cached.objective))
                or not np.all(np.diff(cached.objective) < 0)
                or len(cached.linear_residuals)
                != len(cached.objective) - 1 + (cached.stop == "no feasible decrease")
                or not np.all(np.isfinite(cached.linear_residuals))
                or cached.stop
                not in (
                    None,
                    "no feasible decrease",
                    "relative objective tolerance",
                    "iteration limit",
                    "no adjustable tracking coordinates",
                )
            ):
                raise ValueError("invalid motion checkpoint clock segment or solver history")
            other = np.ones(q.shape[1], dtype=bool)
            other[self.ids] = False
            robot = self.tracking.sites.robot
            if (
                not np.array_equal(cached.q[:, other], initial[:, other])
                or not np.array_equal(cached.q[self.pinned], initial[self.pinned])
                or np.any(
                    np.abs(np.diff(cached.q, axis=0)) > self.h[:, None] * robot.velocity + 1e-8
                )
                or not self.pose_feasible(cached.q).all()
                or not np.isclose(cached.objective[0], history[0], rtol=1e-9, atol=1e-10)
                or not np.isclose(cached.objective[-1], self.cost(cached.q), rtol=1e-9, atol=1e-10)
            ):
                raise ValueError(
                    "motion checkpoint violates the source objective or feasible trajectory"
                )
            q = cached.q.copy()
            history, residuals = list(cached.objective), list(cached.linear_residuals)
            if cached.stop is not None:
                return q, history, residuals, cached.stop

        def publish(stop: str | None) -> None:
            if on_checkpoint is not None:
                on_checkpoint(
                    MotionCheckpoint(
                        start, end, phase, q.copy(), tuple(history), tuple(residuals), stop
                    )
                )

        if not len(self.ids):
            publish("no adjustable tracking coordinates")
            return q, history, residuals, "no adjustable tracking coordinates"
        for _ in range(len(history) - 1, 36):
            normal = _Normal(len(q), len(self.ids))
            self.cost(q, normal)
            step, residual = normal.step(1.0 / (0.35 * self.rate))
            residuals.append(residual)
            diagonal = normal.lower[0].diagonal(axis1=1, axis2=2)
            descent = -normal.gradient / diagonal
            del normal
            decreased = False
            # A Newton step can point uphill once dependent coordinates are
            # stopped by a sampled contact or upper bound. Each coordinate
            # of the diagonal-gradient fallback independently points downhill.
            for direction in (step, descent):
                for scale in (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125):
                    trial = self.candidate(q, scale * direction)
                    if np.array_equal(q, trial):
                        continue
                    cost = self.cost(trial)
                    if cost < history[-1] - 1e-10 * max(1.0, history[-1]):
                        decreased = True
                        break
                if decreased:
                    break
            else:
                publish("no feasible decrease")
                return q, history, residuals, "no feasible decrease"
            q = trial
            history.append(cost)
            if history[-2] - cost < 1e-4 * max(1.0, history[-2]):
                publish("relative objective tolerance")
                return q, history, residuals, "relative objective tolerance"
            publish(None)
        publish("iteration limit")
        return q, history, residuals, "iteration limit"


def _summary(x: Floats) -> dict[str, JsonValue]:
    v = x[np.isfinite(x)]
    return {
        "count": len(v),
        "median": float(np.median(v)) if len(v) else None,
        "p90": float(np.quantile(v, 0.9)) if len(v) else None,
        "p99": float(np.quantile(v, 0.99)) if len(v) else None,
        "max": float(v.max()) if len(v) else None,
    }


def optimize(
    sites: _Sites,
    collision: CollisionModel | None,
    profile: MappingProfile,
    links: Sequence[str],
    seed: Floats,
    times: Indices,
    fps: float | None,
    points: Floats,
    rotations: Floats,
    head: Floats,
    arms: Mapping[str, tuple[str, str, str]],
    body: Mapping[str, Floats],
    effort: EffortModel | None,
    unavailable: str | None,
    checkpoints: Mapping[tuple[int, int, str], MotionCheckpoint] | None,
    on_checkpoint: Callable[[MotionCheckpoint], None] | None,
) -> tuple[Floats, MotionReport]:
    intervals = np.diff(times).astype(np.float64) * 1e-9
    gaps = native_gaps(intervals, fps)
    # Near-coincident timestamps cannot resolve numerical derivatives. Their
    # poses remain pinned, including both sides of an unresolved interval.
    tiny = intervals < 1e-4
    pinned = np.r_[False, tiny] | np.r_[tiny, False]
    edges = np.r_[0, np.flatnonzero(gaps | tiny) + 1, len(times)]
    robot = sites.robot
    if seed.shape != (len(times), len(robot.names)) or not np.isfinite(seed).all():
        raise ValueError("seed requires a finite position for every joint and exposure")
    if np.any(seed < robot.lower - 1e-9) or np.any(seed > robot.upper + 1e-9):
        raise ValueError("seed violates joint position bounds")
    if np.any(
        (np.abs(np.diff(seed, axis=0)) > intervals[:, None] * robot.velocity + 1e-8)
        & ~gaps[:, None]
    ):
        raise ValueError("seed violates a native joint velocity limit")
    tracking = _Tracking(sites, profile, links, points, rotations, head, arms, body)
    q = seed.copy()
    initial_torque, torque = np.full_like(q, np.nan), np.full_like(q, np.nan)
    stats: list[dict[str, list[Floats]]] = [
        {"arm_direction_error_deg": [], "worst_jerk_over_joint_speed_per_s2": []} for _ in range(2)
    ]
    segments: list[JsonValue] = []
    for start, end in pairwise(edges):
        span = slice(start, end)
        relative = (times[span] - times[start]).astype(np.float64) * 1e-9
        model = _Span(seed[span], span, relative, tracking, collision, effort, pinned[span])
        q[span], history, residuals, status = model.optimize(
            "whole", seed[span], checkpoints or {}, on_checkpoint
        )
        segment: dict[str, JsonValue] = {
            "first_index": int(start),
            "stop_index": int(end),
            "objective": list(history),
            "accepted_steps": len(history) - 1,
            "linear_system_relative_residual": list(residuals),
            "stop": status,
        }
        second = governed_start(model)
        if second is not None:
            reduced, initial, control = second
            alt, alternate_history, alternate_residuals, alternate_status = reduced.optimize(
                "governed_start", initial, checkpoints or {}, on_checkpoint
            )
            q[span], lattice = select_path(model, q[span], alt)
            segment["governed_start"] = {
                "projected_joint": robot.names[control],
                "objective": list(alternate_history),
                "accepted_steps": len(alternate_history) - 1,
                "stop": alternate_status,
                "linear_system_relative_residual": list(alternate_residuals),
            }
            segment["feasible_lattice"] = lattice
        segment["final_objective"] = model.cost(q[span])
        segments.append(segment)
        for values, tau, measures in zip(
            [seed[span], q[span]], [initial_torque, torque], stats, strict=True
        ):
            tracking.cost(values, span, model.mass, model.h, None, measures)
            if len(values) >= 4 and len(tracking.controls):
                jerk = _apply(values[:, tracking.controls], model.jerk)
                measures["worst_jerk_over_joint_speed_per_s2"].append(
                    np.max(np.abs(jerk) / model.rate, axis=1)
                )
            if effort is not None:
                v, a = _apply(values, model.mid), _apply(values, model.acc)
                for i in range(len(v)):
                    tau[start + i + 1] = effort.inverse(values[i + 1], v[i], a[i])
    limits = np.full(len(robot.names), np.nan) if effort is None else effort.effort_limits.copy()
    ratios = np.abs(torque) / limits
    ok = np.asarray((np.isfinite(ratios) & (ratios <= 1.0 + 1e-9)).all(axis=1))

    def metrics(data: dict[str, list[Floats]], tau: Floats) -> dict[str, JsonValue]:
        result: dict[str, JsonValue] = {
            k: _summary(np.concatenate(x) if x else np.empty(0)) for k, x in data.items()
        }
        rated = np.isfinite(limits)
        peaks = (
            np.max(np.abs(tau[:, rated]) / limits[rated], axis=1) if rated.any() else np.empty(0)
        )
        result["peak_rated_effort_fraction"] = _summary(peaks)
        result["over_rated_frames"] = int(np.count_nonzero(peaks > 1.0 + 1e-9))
        return result

    operating = (
        np.full(len(robot.names), np.nan) if effort is None else effort.operating_velocity_limits
    )
    speed_limits = np.fmin(robot.velocity, operating)
    native = ~(gaps | tiny)

    def speed_edges(values: Floats) -> Bools:
        speed = np.abs(np.diff(values, axis=0)) / intervals[:, None]
        return np.asarray(native & np.all(speed <= speed_limits * (1.0 + 1e-9), axis=1))

    initial_edge_ok, edge_ok = speed_edges(seed), speed_edges(q)
    failed_edge = native & ~edge_ok
    has_edge = np.r_[native, False] | np.r_[False, native]
    training_speed_ok = has_edge & ~(np.r_[failed_edge, False] | np.r_[False, failed_edge])
    available = effort is not None and bool(np.all(np.isfinite(limits)))
    info: dict[str, JsonValue] = {
        "method": "whole-segment damped Gauss-Newton on native exposures with feasible line search",
        "optimality": "local numerical solution of a nonconvex trajectory objective",
        "effort_model_available": available,
        "effort_evaluated": effort is not None,
        "effort_model_issue": unavailable,
        "effort_basis": (
            "Unloaded inverse dynamics at three-exposure interior stencils, "
            "using CAD inertias, declared gravity and any supplied reflected inertia."
        ),
        "objective": {
            "aggregation": "sum of squared residuals integrated on native clock",
            "wrist_per_m": 220.0,
            "fingertip_per_m": 85.0,
            "head_per_m": 10.0,
            "camera_axis_direction": 72.0,
            "camera_axis_turn_rate_s": 7.0,
            "hand_position_pseudo_huber_scale_m": 0.035,
            "declared_operating_speed_knee_fraction": 0.85,
            "declared_operating_speed_excess_weight": 9.0,
            "palm_transverse_direction": 2.0,
            "upper_arm_and_forearm_direction": 2.2,
            "seed_displacement_over_speed_s": 0.45,
            "joint_derivative_weights_over_speed": [0.9, 0.28, 0.034],
            "effort_over_limit_weight": 1.8,
            "effort_knee_fraction": 0.8,
            "additional_effort_weight_above_knee": 5.0,
        },
        "sampled_hand_tracking_envelope_m": {
            "wrist": 0.025,
            "fingertip": 0.04,
            "outside_budget_min_allowance": 0.004,
            "outside_budget_allowance_fraction_of_excess": 0.5,
        },
        "sampled_camera_orientation_envelope_deg": {"angle": 7.0, "seed_error_allowance": 1.0},
        "continuous_segments": len(segments),
        "solver": {
            "max_iterations": 36,
            "relative_objective_tolerance": 1e-4,
            "line_search_scales": [1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125],
            "directions": ["damped Gauss-Newton", "diagonal gradient"],
        },
        "training_effort_ok_frames": int(np.count_nonzero(ok)),
        "speed_qualification": {
            "criterion": "Each adjoining resolved native edge meets every effective speed ceiling",
            "declared_operating_limits": {
                robot.names[i]: float(operating[i]) for i in np.flatnonzero(np.isfinite(operating))
            },
            "native_edges": int(native.sum()),
            "over_limit_edges_before": int(np.count_nonzero(native & ~initial_edge_ok)),
            "over_limit_edges_after": int(failed_edge.sum()),
            "supported_exposures": int(training_speed_ok.sum()),
        },
        "optimized_joints": [robot.names[i] for i in tracking.controls],
        "segments": segments,
        "before": metrics(stats[0], initial_torque),
        "after": metrics(stats[1], torque),
    }
    return q, MotionReport(
        initial_torque,
        torque,
        limits,
        speed_limits,
        edge_ok,
        training_speed_ok,
        ok,
        available,
        effort is not None,
        info,
    )
