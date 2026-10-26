"""Named rigid sites on physical components, independent of controller semantics."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field

from sx_contracts import ComponentId, decode

from .assembly import Pose
from .errors import EmbodimentSchemaError
from .kinematics import Kinematics, Transform, compose, from_xyz_rpy


@dataclass(frozen=True, slots=True)
class BodySite:
    """A named point and orientation fixed to a component's description link.

    Names are open: a body may have any number of arms, tools or other sites. The pose
    is ``link_from_site`` in metres/radians. A site declares physical geometry, not an
    observed trajectory, an anatomical correspondence or an executable controller.
    """

    name: str
    component_id: ComponentId
    link: str
    pose: Pose = field(default_factory=Pose)

    def __post_init__(self) -> None:
        if any(not value.strip() for value in (self.name, self.component_id, self.link)):
            raise EmbodimentSchemaError("a body site needs a name, component and description link")

    def transform(
        self, kinematics: Kinematics, frame: str, positions: Mapping[str, float] | None = None
    ) -> Transform:
        """``frame_from_site``; every joint on the path needs its measured position."""
        return compose(
            kinematics.pose(frame, self.link, positions),
            from_xyz_rpy(self.pose.xyz, self.pose.rpy),
        )

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "component_id": str(self.component_id),
            "link": self.link,
            "xyz": list(self.pose.xyz),
            "rpy": list(self.pose.rpy),
        }

    @classmethod
    def from_dict(cls, document: Mapping[str, object]) -> BodySite:
        entry = decode.exactly(
            document,
            {"name", "component_id", "link", "xyz", "rpy"},
            where="body site",
            error=EmbodimentSchemaError,
        )
        xyz, rpy = decode.numbers(entry, "xyz"), decode.numbers(entry, "rpy")
        if len(xyz) != 3 or len(rpy) != 3:
            raise EmbodimentSchemaError("a body site's xyz and rpy each contain three numbers")
        return cls(
            decode.text(entry, "name"),
            ComponentId(decode.text(entry, "component_id")),
            decode.text(entry, "link"),
            Pose((xyz[0], xyz[1], xyz[2]), (rpy[0], rpy[1], rpy[2])),
        )
