"""How fast each driven joint may move: the motor's rating and the arm it carries.

One source for every motion path a station runs (an operator's teleoperation, a program goal,
a Ready or Park return, a policy's chunk): ``tools/render_driver_bindings.py`` renders each
driven joint's :class:`JointMotion` into the driver bindings, and the governor
(``sx_embodiment_drivers::AxisLimits::rated``) reads it there.

* **Speed** is the actuator model's rated output speed
  (:data:`~sx_embodiments.layout.ACTUATOR_RATINGS`: the vendor's speed at rated torque and
  rated supply, not the no-load speed) through the binding's reduction. A serial servo port
  that declares a :class:`~sx_embodiments.layout.ServoApproach` moves no faster than the
  servo's own firmware limit, so the governor's step never outruns the servo.
* **Acceleration** is what the motor's torque leaves after holding the arm: its rated
  (continuous) torque, which it gives across its rated speed (where none is recorded,
  :data:`TORQUE_SHARE` of its peak), less the most gravity the joint ever holds, over the
  most inertia it ever moves (the joint-space mass matrix's diagonal), both over
  :data:`POSE_SAMPLES` poses spread over every joint's range, with every link as many times
  the description's mass as the joint's port declares (a body whose description is an
  estimate the robot was measured against: the static upper body's arms, 2.61). It is never
  above the rate that reaches full speed in :data:`SHORTEST_RAMP_S` (a joint whose description
  carries almost no inertia would otherwise be asked for a step change of speed), nor above a
  serial port's servo approach.

The acceleration (and the inertia rendered beside it) is an estimate, not a measurement: the
description's masses times the port's margin (identified on the static upper body's forearm
alone), the mass matrix's diagonal (no coupling, Coriolis or centripetal torque) and no rotor
inertia (the vendors publish none). A joint identified in place should replace it.

These are what the motors can do. A vendor chain may hold an axis lower still, where its own
gains cannot follow that fast (``ActuatorChain::tracking_ceilings``); how much of the rest a
session uses is the station's runtime share, never a change here.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Final

import numpy as np

from ..buses import driven_axes
from ..embodiment import Embodiment
from ..layout import ACTUATOR_RATINGS, BusPort, LayoutError, ServoApproach
from .dynamics import RigidBodyTree

#: The share of a motor's peak torque a planned acceleration may spend: the bench tools' torque
#: guard (``sx-identify`` stops over 80 % of peak), so the planned motion never reaches it.
TORQUE_SHARE: Final = 0.8
#: Poses the most gravity and inertia are searched over, seeded so the rendering is stable.
POSE_SAMPLES: Final = 32
POSE_SEED: Final = 20261030
#: The quickest a joint is brought from rest to full speed: five steps of the governor's 50 Hz.
SHORTEST_RAMP_S: Final = 0.1
#: The farthest a joint may lie from the position just sent while it is not moving: what PD
#: alone leaves a heavy joint short of its target against gravity the description lacks (the
#: left elbow, 2026-10-26). The governor adds the speed term (:func:`tracking_bound`).
SETTLED_TRACKING_RAD: Final = 0.2
#: How many of the governor's longest steps (``MAX_STEP_INTERVAL``, 20 ms) of travel the
#: tracking bound allows on top of the settled error: the step just sent leads the joint by
#: one, the motor's own PD lags it by about another, and one more covers a late answer.
TRACKING_PERIODS: Final = 3.0
#: ``sx_embodiment_drivers::MAX_STEP_INTERVAL``: the longest step the governor ever takes.
STEP_INTERVAL_S: Final = 0.02
#: ``sx_embodiment_drivers::governor::TRACKING_MEMORY``: how long a speed widens the bound.
TRACKING_MEMORY_S: Final = 0.1


@dataclass(frozen=True, slots=True)
class JointMotion:
    """The fastest one joint moves, in its own units: rad/s and rad/s² (m for a prismatic),
    and the most inertia it moves (kg m², kg for a prismatic), so a vendor chain can bound the
    acceleration its own gains can make it follow (``sx_damiao_can`` does)."""

    speed: float
    acceleration: float
    inertia: float = 0.0

    def __post_init__(self) -> None:
        if not all(math.isfinite(v) and v > 0.0 for v in (self.speed, self.acceleration)):
            raise LayoutError("actuator", "a joint's motion must be positive finite")
        if not (math.isfinite(self.inertia) and self.inertia >= 0.0):
            raise LayoutError("actuator", "a joint's inertia must be finite and not negative")


def tracking_bound(speed: float) -> float:
    """The governor's tracking bound for a joint recently stepped at ``speed`` (the step's
    speed, remembered over :data:`TRACKING_MEMORY_S`), whatever the station's loop rate: the
    at-rest bound ``AxisLimits::rated`` gives, widened as ``Governor`` widens it."""
    return SETTLED_TRACKING_RAD + TRACKING_PERIODS * speed * STEP_INTERVAL_S


def _loads(embodiment: Embodiment, joints: list[str]) -> dict[str, tuple[float, float]]:
    """Per joint, the most gravity torque it holds and the most inertia it moves, at the
    description's mass: both are linear in it."""
    tree = RigidBodyTree.of(embodiment.urdf_bytes)
    theta = tree.parameters()
    count = len(tree.axes)
    lower = np.array([(tree.limits[a] or (-math.pi, math.pi))[0] for a in tree.axes])
    upper = np.array([(tree.limits[a] or (-math.pi, math.pi))[1] for a in tree.axes])
    indices = [tree.axes.index(joint) for joint in joints if joint in tree.axes]
    gravity = np.zeros(count)
    inertia = np.zeros(count)
    rng = np.random.default_rng(POSE_SEED)
    zero = np.zeros(count)
    for sample in range(POSE_SAMPLES):
        # The zero pose first: every arm straight out, where the shoulders carry the most.
        pose = zero if sample == 0 else rng.uniform(lower, upper)
        held = tree.inverse(theta, pose, zero, zero)
        gravity = np.maximum(gravity, np.abs(held))
        for index in indices:
            unit = np.zeros(count)
            unit[index] = 1.0
            inertia[index] = max(
                inertia[index], float(tree.inverse(theta, pose, zero, unit)[index] - held[index])
            )
    return {tree.axes[index]: (float(gravity[index]), float(inertia[index])) for index in indices}


def joint_motion(
    embodiment: Embodiment,
    approach: Mapping[BusPort, ServoApproach] | None = None,
    mass_margin: Mapping[BusPort, float] | None = None,
    speed_cap: Mapping[str, float] | None = None,
) -> dict[str, JointMotion]:
    """Every driven joint's :class:`JointMotion`, by joint name; ``approach`` is the body's
    serial servo approach per port and ``mass_margin`` how many times the description's mass
    each port's joints are planned to carry (1 where none is given). ``speed_cap`` holds a
    joint below what its motor can do, by name, in its own unit per second (its acceleration
    scaled alike), where the body's own evidence says its controller cannot yet carry more
    (the static upper body's torso).

    Raises :class:`LayoutError` for a driven joint whose model has no rated speed and whose
    port declares no servo approach: a joint with no known speed is not moved.
    """
    approach = approach or {}
    mass_margin = mass_margin or {}
    speed_cap = speed_cap or {}
    driven = [(axis.joint, axis.binding, axis.port) for axis in driven_axes(embodiment)]
    loads = _loads(embodiment, [joint for joint, _, _ in driven])
    motions: dict[str, JointMotion] = {}
    for joint, binding, port in driven:
        rating = ACTUATOR_RATINGS[binding.model]
        servo = approach.get(port)
        speeds = [
            s
            for s in (
                None
                if rating.rated_speed_rad_s is None
                else rating.rated_speed_rad_s / binding.reduction,
                None if servo is None else servo.speed,
            )
            if s is not None
        ]
        if not speeds:
            raise LayoutError("actuator", f"{joint}'s {binding.model.value} has no rated speed")
        speed = min(speeds)
        margin = mass_margin.get(port, 1.0)
        gravity, inertia = (margin * load for load in loads.get(joint, (0.0, 0.0)))
        available = (
            TORQUE_SHARE * rating.peak_torque_nm
            if rating.rated_torque_nm is None
            else rating.rated_torque_nm
        )
        torque = available * binding.reduction - gravity
        if torque <= 0.0:
            raise LayoutError("actuator", f"{joint} cannot hold its own arm on its rated torque")
        accelerations = [speed / SHORTEST_RAMP_S]
        if inertia > 0.0:
            accelerations.append(torque / inertia)
        if servo is not None:
            accelerations.append(servo.acceleration)
        cap = speed_cap.get(joint, speed)
        if not cap > 0.0:
            raise LayoutError("actuator", f"{joint}'s speed cap must be positive")
        # A capped joint keeps its acceleration, never a ramp slower than SHORTEST_RAMP_S to
        # its cap: a cap slows it, it does not make it sluggish to start and stop.
        capped = min(cap, speed)
        accelerations.append(capped / SHORTEST_RAMP_S)
        motions[joint] = JointMotion(capped, min(accelerations), inertia)
    return motions
