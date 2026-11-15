#!/usr/bin/env python3
"""Render the humanoid's rated source description: the vendor URDF with real drive limits.

The source is ``assets/humanoid_pkg/urdf/urdf_robot_acc.urdf`` (``humanoid_pkg`` at
``666bc6b5``), whose torso, waist and leg joints all declare ``effort="10" velocity="1.0"``
(the left hip yaw 36), placeholders no motor is rated at. This writes the same bytes with
each of those joints' ``<limit effort velocity>`` replaced by the joint rating of the drive the
registry binds it to (``sx_embodiments.layout.joint_rating``, from ``ACTUATOR_RATINGS``):

* a directly driven joint (torso yaw, hip pitch, roll and yaw, knee) declares its motor's peak
  torque and rated speed at its rated 24 V supply;
* a joint two motors drive through a linkage (waist roll and pitch, both ankle joints)
  declares both motors' summed peak torque and their rated speed at a unit lever ratio, an
  upper bound until the linkage is measured.

Nothing else changes: bounds, geometry and inertials stay the CAD's. The full body's core and
legs are cut from this file (``known/humanoid.py``); the legacy upper-body variants keep the
vendor file. Run from the package root after changing a rating or a binding, then update the
asset's digest:

    python tools/compose_humanoid_rated_urdf.py

``tests/test_humanoid.py`` re-renders it and compares bytes.
"""

from __future__ import annotations

import re
from pathlib import Path

from sx_embodiments.known.humanoid import (
    SENTIENT_HUMANOID_LEFT_LEG,
    SENTIENT_HUMANOID_RATED_URDF,
    SENTIENT_HUMANOID_RIGHT_LEG,
    SENTIENT_HUMANOID_TORSO,
    SENTIENT_HUMANOID_URDF,
)
from sx_embodiments.layout import joint_rating

ROOT = Path(__file__).resolve().parents[1]

_JOINT = re.compile(r'<joint\s+name="(?P<name>[^"]+)".*?</joint>', re.DOTALL)
_EFFORT = re.compile(r'effort="[^"]*"')
_VELOCITY = re.compile(r'velocity="[^"]*"')


def _number(value: float) -> str:
    return format(value, ".9g")


def rated_limits() -> dict[str, tuple[float, float]]:
    """Each rated joint's effort (N·m) and velocity (rad/s), from its drive's rating."""
    limits: dict[str, tuple[float, float]] = {}
    for part in (SENTIENT_HUMANOID_TORSO, SENTIENT_HUMANOID_RIGHT_LEG, SENTIENT_HUMANOID_LEFT_LEG):
        for axis in part.layout.axes:
            rating = joint_rating(axis.actuation)
            if rating is None:
                raise ValueError(f"{axis.name} has no rated drive")
            limits[axis.name] = (rating.effort, rating.velocity)
    return limits


def render(source: str) -> str:
    limits = rated_limits()
    seen: set[str] = set()

    def joint(match: re.Match[str]) -> str:
        name = match["name"]
        if name not in limits:
            return match[0]
        effort, velocity = limits[name]
        block, efforts = _EFFORT.subn(f'effort="{_number(effort)}"', match[0])
        block, velocities = _VELOCITY.subn(f'velocity="{_number(velocity)}"', block)
        if efforts != 1 or velocities != 1:
            raise ValueError(f"{name} has no single <limit effort velocity>")
        seen.add(name)
        return block

    rendered = _JOINT.sub(joint, source)
    if missing := sorted(set(limits) - seen):
        raise ValueError(f"the source has no joint {missing}")
    return rendered


def main() -> None:
    source = (ROOT / "assets" / SENTIENT_HUMANOID_URDF.relpath).read_bytes().decode("utf-8")
    (ROOT / "assets" / SENTIENT_HUMANOID_RATED_URDF.relpath).write_bytes(
        render(source).encode("utf-8")
    )


if __name__ == "__main__":
    main()
