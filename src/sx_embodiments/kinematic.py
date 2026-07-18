"""The flat single-arm kinematic view consumed by runtimes (enpire safety, drivers, sim).

This is the record real-robot code binds (``ArmDriver.embodiment``); for registry bodies it
is DERIVED from the compositional :class:`~sx_embodiments.compose.EmbodimentSpec` via
``kinematic_view`` and pinned by test, never re-declared.
"""

import dataclasses
from dataclasses import dataclass
from pathlib import Path
from typing import Self

from .errors import PartValidationError
from .identity import EmbodimentId


@dataclass(frozen=True, slots=True)
class Embodiment:
    """Validated robot-body invariants shared by real and simulated runtimes."""

    embodiment_id: EmbodimentId
    dof: int
    joint_lower: tuple[float, ...]
    joint_upper: tuple[float, ...]
    home_joints: tuple[float, ...]
    gripper_travel_m: tuple[float, float]
    policy_hz: float
    mobile_base: bool
    urdf_path: Path | None = None

    def __post_init__(self) -> None:
        if not self.embodiment_id.strip():
            raise PartValidationError(str(self.embodiment_id), "embodiment_id must not be empty")
        if self.dof <= 0:
            raise PartValidationError(str(self.embodiment_id), "dof must be positive")
        for name, values in (
            ("joint_lower", self.joint_lower),
            ("joint_upper", self.joint_upper),
            ("home_joints", self.home_joints),
        ):
            if len(values) != self.dof:
                raise PartValidationError(
                    str(self.embodiment_id),
                    f"{name} must have length {self.dof}, got {len(values)}",
                )
        if any(
            lower >= upper for lower, upper in zip(self.joint_lower, self.joint_upper, strict=True)
        ):
            raise PartValidationError(str(self.embodiment_id), "joint_lower must be < joint_upper")
        if any(
            home < lower or home > upper
            for home, lower, upper in zip(
                self.home_joints, self.joint_lower, self.joint_upper, strict=True
            )
        ):
            raise PartValidationError(
                str(self.embodiment_id), "home_joints must lie within the limits"
            )
        lo, hi = self.gripper_travel_m
        if not 0.0 <= lo < hi:
            raise PartValidationError(
                str(self.embodiment_id), "gripper travel must satisfy 0 <= lo < hi"
            )
        if self.policy_hz <= 0.0:
            raise PartValidationError(str(self.embodiment_id), "policy_hz must be positive")
        if self.urdf_path is not None and not self.urdf_path.is_file():
            raise FileNotFoundError(f"{self.embodiment_id}: URDF not found at {self.urdf_path}")

    @property
    def gripper_max_width_m(self) -> float:
        return self.gripper_travel_m[1]

    def with_urdf(self, path: Path) -> Self:
        """Attach a validated local URDF at an explicit runtime wiring site."""
        if not path.is_file():
            raise FileNotFoundError(f"{self.embodiment_id}: URDF not found at {path}")
        return dataclasses.replace(self, urdf_path=path)
