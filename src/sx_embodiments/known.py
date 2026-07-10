"""Canonical robot-body facts used by multiple Sentient-X runtimes."""

from collections.abc import Mapping
from typing import Final

from .manifest import Embodiment, EmbodimentId

PIPER: Final = Embodiment(
    embodiment_id=EmbodimentId("piper"),
    dof=6,
    joint_lower=(-2.6179, 0.0, -2.967, -1.745, -1.22, -2.09439),
    joint_upper=(2.6179, 3.14, 0.0, 1.745, 1.22, 2.09439),
    home_joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    gripper_travel_m=(0.0, 0.07),
    policy_hz=30.0,
    mobile_base=False,
)

PANDA_OMRON: Final = Embodiment(
    embodiment_id=EmbodimentId("panda_omron"),
    dof=7,
    joint_lower=(-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973),
    joint_upper=(2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973),
    home_joints=(0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785),
    gripper_travel_m=(0.0, 0.08),
    policy_hz=20.0,
    mobile_base=True,
)

EMBODIMENTS: Final[Mapping[EmbodimentId, Embodiment]] = {
    PIPER.embodiment_id: PIPER,
    PANDA_OMRON.embodiment_id: PANDA_OMRON,
}
