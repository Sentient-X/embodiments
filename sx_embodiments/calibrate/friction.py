"""In-place joint friction: BAM's model family, fitted from sweeps the mounted motors run.

The model is Rhoban's "Better Actuator Models" (BAM, Duclusaud and Passault,
https://github.com/Rhoban/bam at e9a619d, Apache-2.0; ``bam/model.py``, models M1 to M5),
reimplemented here as a closed form rather than BAM's simulation-rollout fit, because a mounted
joint's sweep gives friction directly. ``sx_embodiment_drivers::JointFriction`` is the one
real-time implementation; :func:`budget` and :func:`moving_friction` mirror it for fitting and
for the simulator's motor model, and the crate's parity fixture holds the two equal.

A sweep moves one joint at constant speed through a sub-range while every other joint holds.
In a constant-velocity segment the motor delivers ``tau = g(q) + f(q̇, tau, -g(q))``: gravity
at that joint, which for one joint turning with the rest still is ``A sin q + B cos q`` (plus a
sensor offset ``C``), and friction. Passing the same ``q`` forwards and backwards separates
them without any mass: their mean is ``g`` and half their difference is ``f``. Gravity's load
changes across the sweep, which is what identifies BAM's load-dependent terms. A breakaway
ramp, a very slow move from rest, adds the torque at which motion starts: the full budget at
standstill.

:func:`fit_joint` fits M1 (Coulomb, viscous), M2 (+ Stribeck), M3 (+ load), M4 (M3 + Stribeck
load) and M5 (directional load) by robust nonlinear least squares and keeps the model with the
lowest error on a held-out pass (a more complex model must beat a simpler one by 5 %).
:func:`segment_checks` rejects segments a contact or a stall corrupted before anything is fitted.
"""

from __future__ import annotations

import importlib
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any, Final, cast

import numpy as np
from numpy.typing import NDArray

from ..calibration import JointFriction

Array = NDArray[np.float64]

#: The models, simplest first, with the terms each frees (BAM's M1 to M5).
MODELS: Final[dict[str, tuple[str, ...]]] = {
    "m1": ("coulomb", "viscous"),
    "m2": ("coulomb", "viscous", "stribeck", "stribeck_velocity", "stribeck_exponent"),
    "m3": ("coulomb", "viscous", "load"),
    "m4": (
        "coulomb",
        "viscous",
        "load",
        "stribeck",
        "stribeck_velocity",
        "stribeck_exponent",
        "load_stribeck",
    ),
    "m5": (
        "coulomb",
        "viscous",
        "load_motor",
        "load_external",
        "stribeck",
        "stribeck_velocity",
        "stribeck_exponent",
        "load_motor_stribeck",
        "load_external_stribeck",
    ),
}
#: A more complex model is kept only when its held-out error is this much lower.
COMPLEXITY_MARGIN: Final = 0.05


def budget(friction: JointFriction, speed: Array, motor: Array, external: Array) -> Array:
    """The friction budget, as ``JointFriction::budget``."""
    moving = friction.coulomb + np.abs(
        friction.load_external * external - friction.load_motor * motor
    )
    if friction.stribeck_velocity <= 0.0:
        return moving
    fade = np.exp(-(np.abs(speed / friction.stribeck_velocity) ** friction.stribeck_exponent))
    return moving + fade * (
        friction.stribeck
        + np.abs(friction.load_external_stribeck * external - friction.load_motor_stribeck * motor)
    )


def moving_friction(
    friction: JointFriction,
    speed: Array,
    acceleration: Array,
    motor: Array,
    external: Array,
    stiction_velocity: float,
) -> Array:
    """``JointFriction::torque``: armature, viscous, and the budget with a smooth sign."""
    return (
        friction.armature * acceleration
        + friction.viscous * speed
        + np.tanh(speed / stiction_velocity) * budget(friction, speed, motor, external)
    )


def _friction(joint: str, model: str, values: Sequence[float], armature: float) -> JointFriction:
    named = dict(zip(MODELS[model], values, strict=True))
    load = named.pop("load", 0.0)
    load_stribeck = named.pop("load_stribeck", 0.0)
    return JointFriction(
        joint=joint,
        coulomb=named.get("coulomb", 0.0),
        viscous=named.get("viscous", 0.0),
        armature=armature,
        stribeck=named.get("stribeck", 0.0),
        stribeck_velocity=named.get("stribeck_velocity", 0.0),
        stribeck_exponent=named.get("stribeck_exponent", 0.0),
        load_motor=named.get("load_motor", load),
        load_external=named.get("load_external", load),
        load_motor_stribeck=named.get("load_motor_stribeck", load_stribeck),
        load_external_stribeck=named.get("load_external_stribeck", load_stribeck),
    )


_BOUNDS: Final[dict[str, tuple[float, float, float]]] = {
    # start, lower, upper (BAM's ranges, widened for these larger drives)
    "coulomb": (0.5, 0.0, 10.0),
    "viscous": (0.2, 0.0, 10.0),
    "stribeck": (0.3, 0.0, 10.0),
    "stribeck_velocity": (0.05, 0.005, 1.0),
    "stribeck_exponent": (1.5, 1.0, 10.0),
    "load": (0.02, 0.0, 0.5),
    "load_stribeck": (0.02, 0.0, 1.0),
    "load_motor": (0.02, 0.0, 0.5),
    "load_external": (0.02, 0.0, 0.5),
    "load_motor_stribeck": (0.02, 0.0, 1.0),
    "load_external_stribeck": (0.02, 0.0, 1.0),
}


@dataclass(frozen=True, slots=True, eq=False)
class Segment:
    """One constant-velocity pass (or a breakaway onset) of one joint."""

    label: str
    #: Which pass it belongs to; the last pass is held out.
    pass_index: int
    position: Array
    velocity: Array
    torque: Array
    #: Commanded position, for the tracking checks.
    command: Array
    #: A breakaway onset: one sample, velocity zero, torque the onset torque, sign the
    #: direction of motion.
    breakaway: int = 0


@dataclass(frozen=True, slots=True)
class Rejection:
    label: str
    reason: str


@dataclass(frozen=True, slots=True)
class SegmentLimits:
    peak_torque: float
    #: Relative spread of velocity allowed inside a constant-velocity segment.
    velocity_spread: float = 0.25
    #: The largest step in torque between consecutive samples a smooth sweep makes, N·m.
    torque_step: float = 1.5
    #: The largest step in tracking error between consecutive samples, rad.
    tracking_step: float = 0.03
    #: The share of the commanded range a segment must cover.
    coverage: float = 0.9
    #: The share of peak torque a segment may approach.
    torque_share: float = 0.8


def segment_checks(segment: Segment, limits: SegmentLimits) -> str | None:
    """Why a sweep segment cannot be fitted, or ``None``: velocity not constant, a torque
    step that does not follow ``q``, a tracking-error jump (contact), torque near the peak,
    or motion stalling short of the commanded range."""
    if segment.breakaway:
        return None
    if segment.position.size < 5:
        return "too short"
    speed = float(np.median(segment.velocity))
    if abs(speed) < 1e-3:
        return "not moving"
    if float(np.std(segment.velocity)) > limits.velocity_spread * abs(speed):
        return f"velocity not constant ({np.std(segment.velocity):.3f} about {speed:.3f} rad/s)"
    steps = np.abs(np.diff(segment.torque))
    if float(np.max(steps)) > limits.torque_step:
        return f"torque step {np.max(steps):.2f} N·m (suspected contact)"
    error = segment.command - segment.position
    if float(np.max(np.abs(np.diff(error)))) > limits.tracking_step:
        return f"tracking error jumped {np.max(np.abs(np.diff(error))):.3f} rad (suspected contact)"
    if float(np.max(np.abs(segment.torque))) > limits.torque_share * limits.peak_torque:
        return f"torque {np.max(np.abs(segment.torque)):.1f} N·m near the peak"
    commanded = float(np.ptp(segment.command))
    if commanded > 0 and float(np.ptp(segment.position)) < limits.coverage * commanded:
        return f"stalled: covered {np.ptp(segment.position):.3f} of {commanded:.3f} rad"
    return None


@dataclass(frozen=True, slots=True, eq=False)
class JointFit:
    joint: str
    model: str
    friction: JointFriction
    #: ``g(q) = A sin q + B cos q + C``.
    gravity: tuple[float, float, float]
    #: RMS torque error per model on the held-out pass, N·m.
    held_out: dict[str, float]
    #: RMS on the fitted passes for the chosen model.
    fitted: float
    rejected: tuple[Rejection, ...] = field(default=())
    #: Gravity disagreement between forward and backward at matched q, N·m (RMS).
    direction_gap: float = 0.0


def _gravity(q: Array, a: float, b: float, c: float) -> Array:
    return a * np.sin(q) + b * np.cos(q) + c


def _least_squares() -> Callable[..., Any]:
    return cast(Callable[..., Any], importlib.import_module("scipy.optimize").least_squares)


def _stack(segments: Sequence[Segment]) -> tuple[Array, Array, Array, Array]:
    q = np.concatenate([s.position for s in segments])
    v = np.concatenate(
        [s.velocity if not s.breakaway else np.zeros_like(s.velocity) for s in segments]
    )
    tau = np.concatenate([s.torque for s in segments])
    direction = np.concatenate(
        [
            np.sign(s.velocity) if not s.breakaway else np.full_like(s.torque, float(s.breakaway))
            for s in segments
        ]
    )
    return q, v, tau, direction


def _predict(
    model: str, values: Array, q: Array, v: Array, tau: Array, direction: Array, joint: str
) -> Array:
    a, b, c = values[:3]
    friction = _friction(joint, model, values[3:].tolist(), 0.0)
    g = _gravity(q, a, b, c)
    # Moving or breaking away: the drive overcomes the whole budget in the direction of motion.
    return g + direction * budget(friction, v, tau, -g) + friction.viscous * v


def fit_joint(
    joint: str,
    segments: Sequence[Segment],
    limits: SegmentLimits,
    armature: float = 0.0,
) -> JointFit:
    """Fit every model to the passes but the last, choose by the last pass's error, refit the
    choice on every pass. Segments failing :func:`segment_checks` are rejected first."""
    rejected: list[Rejection] = []
    kept: list[Segment] = []
    for segment in segments:
        reason = segment_checks(segment, limits)
        if reason is None:
            kept.append(segment)
        else:
            rejected.append(Rejection(segment.label, reason))
    passes = sorted({s.pass_index for s in kept})
    if len(passes) < 2:
        raise ValueError(f"{joint}: fewer than two usable passes")
    held = [s for s in kept if s.pass_index == passes[-1]]
    train = [s for s in kept if s.pass_index != passes[-1]]
    least_squares = _least_squares()

    def solve(model: str, data: Sequence[Segment]) -> Array:
        q, v, tau, direction = _stack(data)
        start = [0.0, 0.0, 0.0] + [_BOUNDS[name][0] for name in MODELS[model]]
        lower = [-np.inf] * 3 + [_BOUNDS[name][1] for name in MODELS[model]]
        upper = [np.inf] * 3 + [_BOUNDS[name][2] for name in MODELS[model]]
        def residual(x: Array) -> Array:
            return _predict(model, x, q, v, tau, direction, joint) - tau

        result = least_squares(
            residual,
            np.array(start),
            bounds=(np.array(lower), np.array(upper)),
            loss="soft_l1",
            f_scale=0.3,
        )
        return np.asarray(result.x, dtype=np.float64)

    def rms(model: str, values: Array, data: Sequence[Segment]) -> float:
        q, v, tau, direction = _stack(data)
        error = _predict(model, values, q, v, tau, direction, joint) - tau
        return math.sqrt(float(np.mean(error**2)))

    held_out: dict[str, float] = {}
    for model in MODELS:
        held_out[model] = rms(model, solve(model, train), held)
    chosen = "m1"
    for model in MODELS:
        if held_out[model] < held_out[chosen] * (1.0 - COMPLEXITY_MARGIN):
            chosen = model
    values = solve(chosen, kept)
    a, b, c = (float(x) for x in values[:3])
    gap = _direction_gap(kept, (a, b, c), _friction(joint, chosen, values[3:].tolist(), 0.0))
    return JointFit(
        joint=joint,
        model=chosen,
        friction=_friction(joint, chosen, values[3:].tolist(), armature),
        gravity=(a, b, c),
        held_out=held_out,
        fitted=rms(chosen, values, kept),
        rejected=tuple(rejected),
        direction_gap=gap,
    )


def _direction_gap(
    segments: Sequence[Segment], gravity: tuple[float, float, float], friction: JointFriction
) -> float:
    """Forward and backward passes each imply gravity once their friction is taken off; the
    RMS of the two estimates' difference on a shared grid of ``q``."""
    estimates: dict[int, list[tuple[Array, Array]]] = {1: [], -1: []}
    for segment in segments:
        if segment.breakaway:
            continue
        direction = 1 if float(np.median(segment.velocity)) > 0 else -1
        g = _gravity(segment.position, *gravity)
        implied = (
            segment.torque
            - direction * budget(friction, segment.velocity, segment.torque, -g)
            - friction.viscous * segment.velocity
        )
        estimates[direction].append((segment.position, implied))
    if not estimates[1] or not estimates[-1]:
        return math.nan
    q_f = np.concatenate([p for p, _ in estimates[1]])
    g_f = np.concatenate([g for _, g in estimates[1]])
    q_b = np.concatenate([p for p, _ in estimates[-1]])
    g_b = np.concatenate([g for _, g in estimates[-1]])
    lo, hi = max(q_f.min(), q_b.min()), min(q_f.max(), q_b.max())
    if hi <= lo:
        return math.nan
    grid = np.linspace(lo, hi, 20)
    order_f, order_b = np.argsort(q_f), np.argsort(q_b)
    forward = np.interp(grid, q_f[order_f], g_f[order_f])
    backward = np.interp(grid, q_b[order_b], g_b[order_b])
    return math.sqrt(float(np.mean((forward - backward) ** 2)))
