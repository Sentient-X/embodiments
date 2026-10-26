"""What a viewer needs beside a body's description, declared once and carried in its render pack.

These are facts about a body's surroundings and how an operator's XR controllers meet it,
not its morphology: they are not part of :class:`~sx_embodiments.embodiment.Embodiment`'s
content identity, so declaring or correcting one never changes an embodiment id, an
episode seal or a description digest. Each body's module in ``known/`` declares its own;
``tools/render_packs.py`` hands them to :func:`sx_embodiments.render_pack.build`, which
writes them into ``pack.json`` (``tracking``, ``mount``, ``virtual_gripper``).
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Literal

from sx_contracts.identity import JsonValue

from .errors import EmbodimentError

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Side = Literal["left", "right"]
Space = Literal["openxr-grip"]


class ViewingError(EmbodimentError):
    """A viewing fact is malformed."""


def _finite(values: tuple[float, ...], what: str) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ViewingError(f"{what} must be finite")


@dataclass(frozen=True, slots=True)
class TrackedDevice:
    """Where an XR controller's grip space sits on a body: ``frame_from_grip``.

    ``side`` is the operator hand holding it. ``device`` is the controller's WebXR
    input-profile id; ``space`` names the pose the headset reports for it, ``openxr-grip``
    being the WebXR ``gripSpace`` / OpenXR ``/input/grip/pose``. ``frame`` is a description
    link; the pose places that space in it (translation in metres, rotation as a unit
    quaternion x, y, z, w). ``provenance`` says how it was measured.
    """

    side: Side
    device: str
    space: Space
    frame: str
    translation_m: Vector3
    rotation_xyzw: Quaternion
    provenance: str

    def __post_init__(self) -> None:
        if not (self.device.strip() and self.frame.strip() and self.provenance.strip()):
            raise ViewingError("a tracked device needs a device, a frame and a provenance")
        _finite(self.translation_m + self.rotation_xyzw, "tracked device pose")
        if abs(math.sqrt(sum(c * c for c in self.rotation_xyzw)) - 1.0) > 1e-5:
            raise ViewingError(f"{self.frame}: rotation_xyzw is not a unit quaternion")

    def to_json(self) -> JsonValue:
        return {
            "device": self.device,
            "space": self.space,
            "frame": self.frame,
            "translation_m": list(self.translation_m),
            "rotation_xyzw": list(self.rotation_xyzw),
        }


@dataclass(frozen=True, slots=True)
class Workcell:
    """How high a fixed body's mount surface stands above the floor.

    The surface itself is the body's :class:`~sx_embodiments.compose.BaseMount`: its frame's
    ``z = 0`` plane, solid below (``collision.json``'s ``support``, which the IK keeps clear
    of). This adds only what a viewer needs to stand the operator on the floor beside it.
    """

    floor_to_frame_m: float
    provenance: str

    def __post_init__(self) -> None:
        if not self.provenance.strip():
            raise ViewingError("a workcell needs a provenance")
        if not math.isfinite(self.floor_to_frame_m) or self.floor_to_frame_m < 0.0:
            raise ViewingError("floor_to_frame_m must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class VirtualGripper:
    """The body's own gripper, drawn at an operator's empty XR controller.

    When the operator holds only controllers, each hand drives a gripper whose tool frame
    sits ``ahead_m`` ahead of the controller's grip origin along its pointing ray (WebXR
    ``targetRaySpace`` -z), with x (the fingers) along that ray and the jaw opening horizontal.
    It is drawn with every link that moves rigidly with the body's tool frame (``tcp_frames``):
    the subtree below the last actuated joint above it, so the wrist plates, motor mount and
    camera that carry the jaw are drawn with it. That tool frame lands on the operator's.
    """

    ahead_m: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.ahead_m) or self.ahead_m < 0.0:
            raise ViewingError("ahead_m must be finite and non-negative")

    def to_json(self, links: Mapping[str, list[str]]) -> JsonValue:
        """``links``: by side, the links drawn as that side's gripper."""
        return {
            "anchor": "grip",
            "along": "target_ray",
            "ahead_m": self.ahead_m,
            "x_axis": "target_ray",
            "jaw_opening": "horizontal",
            "links": {side: list(drawn) for side, drawn in links.items()},
        }


@dataclass(frozen=True, slots=True)
class Viewing:
    """One body's viewing facts; every one optional."""

    tracking: tuple[TrackedDevice, ...] = ()
    mount: Workcell | None = None
    virtual_gripper: VirtualGripper | None = None

    def __post_init__(self) -> None:
        sides = [device.side for device in self.tracking]
        if len(set(sides)) != len(sides):
            raise ViewingError("one tracked device per side")
