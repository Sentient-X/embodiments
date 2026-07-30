import pytest
from sx_contracts import (
    Capability,
    TaskCapabilityRequirements,
    TaskRoleId,
    TaskRoleRequirement,
    UnsatisfiedCapabilityRequirementsError,
    match_capabilities,
)

from sx_embodiments import Embodiment, embodiments


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


@pytest.mark.parametrize("embodiment", [embodiments["piper"], embodiments["franka"]])
def test_single_arm_robots_bind_the_same_semantic_task_role(embodiment: Embodiment) -> None:
    binding = match_capabilities(_manipulation_requirements(1), embodiment.capabilities)
    assert binding.assignments[0].component_id in {"gripper", "jaw"}


def test_bimanual_profile_binds_two_distinct_effectors() -> None:
    binding = match_capabilities(
        _manipulation_requirements(2), embodiments["bimanual-so101"].capabilities
    )
    assert tuple(item.component_id for item in binding.assignments) == (
        "left_jaw",
        "right_jaw",
    )


def test_fixed_and_mobile_panda_differ_by_derived_locomotion_capability() -> None:
    mobile = embodiments["panda_omron"].capabilities
    fixed = embodiments["franka"].capabilities
    assert any(Capability.LOCOMOTION_PLANAR in item.capabilities for item in mobile.components)
    assert not any(Capability.LOCOMOTION_PLANAR in item.capabilities for item in fixed.components)


def test_single_arm_profile_cannot_fake_bimanuality() -> None:
    with pytest.raises(UnsatisfiedCapabilityRequirementsError):
        match_capabilities(_manipulation_requirements(2), embodiments["piper"].capabilities)
