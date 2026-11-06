"""Whether a body's tool points were checked on its hardware before anything drives them.

A body's tool point on each side (:func:`sx_embodiments.render_pack.tcp_frames`) is derived
from its description: the link nearest a jawed tool's grasp point. Derived is not checked. The
console's teleoperation (headset and desk, which read the render pack) drives a tool point only
after a person checked on the real robot, through the embodiment layer (``robot-bench``), that
the derived link is where the jaws close and points the way the tool approaches, and recorded it
here. Until then the render pack carries the derived points as ``tcp_candidates`` (drawn, never
driven) and no ``tcp_frames``. The station reads it too: it composes a follower only when the
pack's check names the chain of the revision the robot is assigned (the bindings'
``tool_chains``), so no client's tool target reaches an unchecked body (sx #3457).

A check is of the body design's tool points as they hang off its root: it names the links it
checked and the digest of the kinematic chain from the description's root to each of them
(:func:`tool_chain`: every joint on the path, its type, origin, axis and mimic). A render pack
fills ``tcp_frames`` only while both still match, so a revision that moves a tool point or any
joint on its chain asks for a new check, and one that changes only what is off the tool path (a
camera, a mesh) keeps it. The check is per body design, never per unit: a unit's own mount
offsets are its measured calibration (a correction edge), not this record. Like ``front`` and
``viewing``, the record lives on the definition outside the revision's identity: recording it
moves no embodiment id.
"""

from __future__ import annotations

import datetime as dt
from collections.abc import Mapping
from dataclasses import dataclass

from sx_contracts.identity import JsonValue, content_digest

from .errors import EmbodimentError


class ToolPointCheckError(EmbodimentError):
    """A tool-point check record that says nothing checkable."""


@dataclass(frozen=True, slots=True)
class ToolPointCheck:
    """Who checked a body's tool points on hardware, when, how, and where the evidence is."""

    #: The digest of the kinematic chain from the root to each checked tool point
    #: (:func:`tool_chain`) on the description that was checked.
    chain: str
    #: The tool point checked on each side: ``{"left": link, "right": link}``.
    points: Mapping[str, str]
    #: The person (or agent and the person present) who checked them.
    checked_by: str
    checked_on: dt.date
    #: What was done, in a sentence: the procedure ``robot-bench`` names, or what stood for it.
    method: str
    #: Where the evidence is: a robot-bench session, a station session, PRs, a skill section.
    evidence: str

    def __post_init__(self) -> None:
        if not self.points:
            raise ToolPointCheckError("a tool-point check names the tool points it checked")
        for name in ("chain", "checked_by", "method", "evidence"):
            if not getattr(self, name).strip():
                raise ToolPointCheckError(f"a tool-point check needs its {name}")

    def to_json(self) -> dict[str, JsonValue]:
        return {
            "chain": self.chain,
            "points": dict(self.points),
            "checked_by": self.checked_by,
            "checked_on": self.checked_on.isoformat(),
            "method": self.method,
            "evidence": self.evidence,
        }


def _rounded(values: tuple[float, ...]) -> list[float]:
    # Nine decimals: far below a micrometre or a microradian, and above any platform's last-bit
    # difference in the trigonometry an origin's rpy goes through; +0.0 folds -0.0.
    return [round(float(value), 9) + 0.0 for value in values]


def tool_chain(urdf: bytes, points: Mapping[str, str]) -> str:
    """The digest of the kinematic chain from ``urdf``'s root to each tool point in ``points``.

    For each side (sorted), every joint from the root link down to the tool point's link: its
    name, type, parent and child links, origin transform, axis and mimic. Anything off those
    paths (another limb, a camera, a mesh, a collision proxy, a joint limit) is not in it.

    Raises:
        KinematicsError: a tool point that is not a link of the description.
    """
    from .kinematics import Kinematics

    kinematics = Kinematics.of(urdf)
    sides: list[JsonValue] = []
    for side, link in sorted(points.items()):
        root = link
        while (above := kinematics.parent_joint.get(root)) is not None:
            root = kinematics.joints[above].parent
        _, down = kinematics.path(root, link)
        joints: list[JsonValue] = []
        for joint in down:
            mimic: JsonValue = None
            if joint.mimic is not None:
                mimic = {
                    "leader": joint.mimic.leader,
                    "multiplier": round(joint.mimic.multiplier, 9) + 0.0,
                    "offset": round(joint.mimic.offset, 9) + 0.0,
                }
            origin: list[JsonValue] = [*_rounded(joint.origin)]
            axis: list[JsonValue] = [*_rounded(joint.axis)]
            joints.append(
                {
                    "name": joint.name,
                    "type": joint.type.value,
                    "parent": joint.parent,
                    "child": joint.child,
                    "origin": origin,
                    "axis": axis,
                    "mimic": mimic,
                }
            )
        sides.append({"side": side, "root": root, "link": link, "joints": joints})
    return str(content_digest(sides))
