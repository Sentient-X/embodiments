import pytest
from sx_capabilities import (
    Capability,
    TaskCapabilityRequirements,
    TaskRoleId,
    TaskRoleRequirement,
    UnsatisfiedCapabilityRequirementsError,
    match_capabilities,
)

from sx_embodiments import EmbodimentSpec, capability_profile
from sx_embodiments.known.panda import FRANKA_SPEC, PANDA_OMRON_SPEC
from sx_embodiments.known.piper import PIPER_SPEC
from sx_embodiments.known.so101 import BIMANUAL_SO101_SPEC


def _manipulation_requirements(count: int) -> TaskCapabilityRequirements:
    return TaskCapabilityRequirements(
        tuple(
            TaskRoleRequirement(
                TaskRoleId(f"manipulation.{index}"),
                (Capability.SPATIAL_MOTION_SE3, Capability.GRASP),
            )
            for index in range(count)
        )
    )


@pytest.mark.parametrize("spec", [PIPER_SPEC, FRANKA_SPEC])
def test_single_arm_robots_bind_the_same_semantic_task_role(spec: EmbodimentSpec) -> None:
    binding = match_capabilities(_manipulation_requirements(1), capability_profile(spec))
    assert binding.assignments[0].component_id in {"gripper", "jaw"}


def test_bimanual_profile_binds_two_distinct_effectors() -> None:
    binding = match_capabilities(
        _manipulation_requirements(2), capability_profile(BIMANUAL_SO101_SPEC)
    )
    assert tuple(item.component_id for item in binding.assignments) == (
        "left_jaw",
        "right_jaw",
    )


def test_fixed_and_mobile_panda_differ_by_derived_locomotion_capability() -> None:
    mobile = capability_profile(PANDA_OMRON_SPEC)
    fixed = capability_profile(FRANKA_SPEC)
    assert any(Capability.LOCOMOTION_PLANAR in item.capabilities for item in mobile.components)
    assert not any(Capability.LOCOMOTION_PLANAR in item.capabilities for item in fixed.components)


def test_single_arm_profile_cannot_fake_bimanuality() -> None:
    with pytest.raises(UnsatisfiedCapabilityRequirementsError):
        match_capabilities(_manipulation_requirements(2), capability_profile(PIPER_SPEC))
