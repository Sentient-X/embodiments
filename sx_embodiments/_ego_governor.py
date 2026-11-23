"""A governed-axis alternative and an exact finite-horizon selection of feasible poses."""

from __future__ import annotations

from copy import copy
from types import ModuleType
from typing import TYPE_CHECKING, cast

import numpy as np
import scipy.optimize as scipy_optimize  # pyright: ignore[reportMissingTypeStubs] -- numerical extension.
from numpy.typing import NDArray
from sx_contracts.identity import JsonValue

if TYPE_CHECKING:
    from ._ego_motion import (
        _Span,  # pyright: ignore[reportPrivateUsage] -- internal numerical kernel
    )

type Floats = NDArray[np.float64]


def governed_start(model: _Span) -> tuple[_Span, Floats, int] | None:
    """Project the most overspeed coordinate on its native clock and re-fit the other axes."""
    if len(model.seed) < 4 or len(model.ids) < 2:
        return None
    delta = np.abs(np.diff(model.seed[:, model.ids], axis=0)) / model.h[:, None]
    excess = np.maximum(delta / model.operating_rate - 1, 0)
    severity = np.sum(model.h[:, None] * excess**2, axis=0)
    axis = int(np.argmax(severity))
    if severity[axis] <= 1e-12:
        return None
    control = int(model.ids[axis])
    t = np.r_[0.0, np.cumsum(model.h)] * model.operating_rate[axis]
    projected = model.seed[:, control].copy()
    rising, falling = np.zeros_like(t), np.zeros_like(t)
    numerical = cast("ModuleType", scipy_optimize)
    # Dykstra projection on the two one-sided Lipschitz cones. The final
    # forward clamp enforces the bound even if the iteration ceiling is reached.
    for _ in range(500):
        previous = projected.copy()
        upper = (
            np.asarray(numerical.isotonic_regression(projected + rising - t, increasing=False).x)
            + t
        )
        rising += projected - upper
        projected = (
            np.asarray(numerical.isotonic_regression(upper + falling + t, increasing=True).x) - t
        )
        falling += upper - projected
        if np.max(np.abs(projected - previous)) < 1e-11 and np.all(
            np.abs(np.diff(projected)) <= np.diff(t) + 1e-10
        ):
            break
    travel = np.diff(t)
    for i in range(1, len(projected)):
        projected[i] = np.clip(
            projected[i], projected[i - 1] - travel[i - 1], projected[i - 1] + travel[i - 1]
        )
    prior = model.seed.copy()
    prior[:, control] = projected
    if not np.array_equal(prior[model.pinned], model.seed[model.pinned]):
        return None
    selected = model.ids != control
    tracking = copy(model.tracking)
    tracking.controls = model.ids[selected]
    tracking.hand_influence = tracking.hand_influence[:, selected]
    tracking.camera_influence = tracking.camera_influence[selected]
    alternative = copy(model)
    alternative.tracking, alternative.ids = tracking, tracking.controls
    alternative.rate, alternative.operating_rate = (
        model.rate[selected],
        model.operating_rate[selected],
    )
    # Re-IK from a changed root can initially miss a hand or optical target.
    # Only this exploratory arm/head solve receives these relaxed budgets.
    # Every candidate used below is checked against the ORIGINAL model.
    alternative.hand_budget = model.hand_budget.copy()
    alternative.camera_budget = model.camera_budget.copy()
    for i, row in enumerate(prior):
        hand = tracking.sites.positions(row)[tracking.hand_sites]
        err = np.linalg.norm(hand - model.hand_target[i], axis=1)
        alternative.hand_budget[i] = np.fmax(model.hand_budget[i], err)
        if tracking.camera is not None:
            alternative.camera_budget[i] = max(
                model.camera_budget[i], tracking.camera_angle(model.span.start + i)
            )
    if model.collision is not None:
        contacts = model.collision.batch_at([tracking.sites.full_q(row) for row in prior])
        alternative.clearance = np.maximum(model.clearance, [c.unallowed for c in contacts])
    return alternative, prior, control


def select_path(
    model: _Span, reference: Floats, alternative: Floats
) -> tuple[Floats, dict[str, JsonValue]]:
    """Minimize the full objective on five interpolants per exposure by dynamic programming.

    A state remembers the previous three choices. The fourth pose determines
    the native jerk and the third pose determines inverse dynamics. Work buffers
    for these transitions have constant size independent of recording duration.
    """
    levels = np.linspace(0, 1, 5)
    poses = reference[:, None] + levels[None, :, None] * (alternative - reference)[:, None]
    poses[:, 0] = reference
    poses[:, -1] = alternative
    n, choices, joints = poses.shape
    tr, ids, effort = model.tracking, model.ids, model.effort
    assert n >= 4 and effort is not None
    rated = np.isfinite(effort.effort_limits)
    unary = np.full((n, choices), np.inf)
    eyes = np.zeros((n, choices, 3, 2))
    empty = np.empty(0)
    for k in range(choices):
        ok = model.pose_feasible(poses[:, k])
        for i in np.flatnonzero(ok):
            row = poses[i : i + 1, k]
            source = slice(model.span.start + i, model.span.start + i + 1)
            unary[i, k] = tr.cost(row, source, model.mass[i : i + 1], empty, None)
            deviation = (row[0, ids] - model.seed[i, ids]) / (0.45 * model.rate)
            unary[i, k] += model.mass[i] * np.sum(deviation**2)
            if tr.camera is not None:
                current = tr.sites.data.xmat[tr.sites.ids[tr.camera]].reshape(3, 3)
                eyes[i, k] = current[:, 1:] - tr.rotations[source.start, :, 1:]
    if not np.isfinite(unary[:, 0]).all():
        raise ValueError("the accepted continuous pose leaves its sampled tracking or CAD envelope")
    parents = np.zeros((n, choices, choices, choices), dtype=np.uint8)
    velocity_before = np.empty((choices, choices, joints))
    accel_before = np.empty((choices, choices, choices, len(ids)))
    edge_before = np.empty((choices, choices))
    history = unary[0]
    for i in range(1, n):
        dt = model.h[i - 1]
        velocity = (poses[i, None] - poses[i - 1, :, None]) / dt
        active = velocity[:, :, ids]
        edge = dt * np.sum((0.9 * active / model.rate) ** 2, axis=-1)
        hinge = np.maximum(np.abs(active) / model.operating_rate - 0.85, 0)
        edge += dt * 9.0**2 * np.sum(hinge**2, axis=-1)
        eye_rate = (eyes[i, None] - eyes[i - 1, :, None]) / dt
        edge += dt * 7.0**2 * np.sum(eye_rate**2, axis=(2, 3))
        # Same numerical travel tolerance used to accept the continuous seed.
        edge[np.any(np.abs(velocity) * dt > tr.sites.robot.velocity * dt + 1e-8, axis=-1)] = np.inf
        if i == 1:
            history = history[:, None] + edge + unary[i]
        else:
            centre_dt = (model.h[i - 2] + dt) / 2
            acceleration = (velocity[None] - velocity_before[:, :, None]) / centre_dt
            accel_active = np.take(acceleration, ids, axis=-1)
            triple = centre_dt * np.sum((0.28 * accel_active / model.rate) ** 2, axis=-1)
            feasible = np.isfinite(
                unary[i - 2, :, None, None]
                + unary[i - 1, None, :, None]
                + unary[i, None, None, :]
                + edge_before[:, :, None]
                + edge[None]
            )
            if rated.any():
                weights = model.mid[i - 2]
                for a, b, c in np.argwhere(feasible):
                    qd = (
                        weights[0] * poses[i - 2, a]
                        + weights[1] * poses[i - 1, b]
                        + weights[2] * poses[i, c]
                    )
                    tau = effort.inverse(poses[i - 1, b], qd, acceleration[a, b, c])
                    ratio = tau[rated] / effort.effort_limits[rated]
                    peak = ratio + 5 * np.sign(ratio) * np.maximum(np.abs(ratio) - 0.8, 0)
                    triple[a, b, c] += model.mass[i - 1] * np.sum((1.8 * peak) ** 2)
            scores = history[..., None] + unary[i] + edge + triple
            if i >= 3:
                jerk_dt = float(np.mean(model.h[i - 3 : i]))
                jerk = (accel_active[None] - accel_before[:, :, :, None]) / jerk_dt
                scores += jerk_dt * np.sum((0.034 * jerk / model.rate) ** 2, axis=-1)
                parents[i] = np.argmin(scores, axis=0)
                history = np.min(scores, axis=0)
            else:
                history = scores
            accel_before = accel_active
        velocity_before, edge_before = velocity, edge
    path = np.zeros(n, dtype=np.int64)
    path[-3:] = np.unravel_index(np.argmin(history), history.shape)
    for i in range(n - 1, 2, -1):
        path[i - 3] = parents[i, path[i - 2], path[i - 1], path[i]]
    result = poses[np.arange(n), path]
    optimum = float(np.min(history))
    checked = model.cost(result)
    baseline = model.cost(reference)
    if not np.isclose(optimum, checked, rtol=5e-8, atol=1e-8):
        raise ValueError("finite trajectory objective differs from direct nonlinear evaluation")
    info: dict[str, JsonValue] = {
        "blend_fractions": levels.tolist(),
        "feasible_poses_per_fraction": np.isfinite(unary).sum(axis=0).tolist(),
        "selected_poses_per_fraction": np.bincount(path, minlength=choices).tolist(),
        "continuous_objective": baseline,
        "finite_global_objective": optimum,
        "direct_objective": checked,
    }
    return (result if checked < baseline else reference), info
