"""Whether a body's tool points were checked on its hardware before anything drives them.

A body's tool point on each side (:func:`sx_embodiments.render_pack.tcp_frames`) is derived
from its description: the link nearest a jawed tool's grasp point. Derived is not checked. A
teleoperation or program drives a tool point only after a person checked on the real robot,
through the embodiment layer (``robot-bench``), that the derived link is where the jaws close and
points the way the tool approaches, and recorded it here. Until then the render pack carries the
derived points as ``tcp_candidates`` (drawn, never driven) and no ``tcp_frames``.

Like ``front`` and ``viewing``, the record lives on the definition outside the revision's
identity: recording or correcting a check moves no embodiment id.
"""

from __future__ import annotations

import datetime as dt
from dataclasses import dataclass

from sx_contracts.identity import JsonValue

from .errors import EmbodimentError


class ToolPointCheckError(EmbodimentError):
    """A tool-point check record that says nothing checkable."""


@dataclass(frozen=True, slots=True)
class ToolPointCheck:
    """Who checked a body's tool points on hardware, when, how, and where the evidence is."""

    #: The person (or agent and the person present) who checked them.
    checked_by: str
    checked_on: dt.date
    #: What was done, in a sentence: the procedure ``robot-bench`` names, or what stood for it.
    method: str
    #: Where the evidence is: a robot-bench session, a station session, PRs, a skill section.
    evidence: str

    def __post_init__(self) -> None:
        for name in ("checked_by", "method", "evidence"):
            if not getattr(self, name).strip():
                raise ToolPointCheckError(f"a tool-point check needs its {name}")

    def to_json(self) -> dict[str, JsonValue]:
        return {
            "checked_by": self.checked_by,
            "checked_on": self.checked_on.isoformat(),
            "method": self.method,
            "evidence": self.evidence,
        }
