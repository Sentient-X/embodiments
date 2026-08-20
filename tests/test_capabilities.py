import pytest
from sx_contracts import (
    Capability,
    CapabilitySet,
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
                CapabilitySet((Capability.SPATIAL_MOTION_SE3, Capability.GRASP)),
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


def test_a_dexterous_gripper_is_the_grasp_dexterous_producer() -> None:
    from sx_contracts import Capability

    from sx_embodiments.compose import capabilities_for_part
    from sx_embodiments.identity import PartId
    from sx_embodiments.layout import Bounds, CoordinateUnit, JointAxis, JointLayout
    from sx_embodiments.parts import GraspKind, GripperSpec

    hand = GripperSpec(
        part_id=PartId("test-hand"),
        layout=JointLayout(
            tuple(
                JointAxis(name, CoordinateUnit.RADIAN, Bounds(-1.0, 1.0))
                for name in ("thumb", "index", "middle")
            )
        ),
        grasp=GraspKind.DEXTEROUS,
    )
    assert Capability.GRASP_DEXTEROUS in capabilities_for_part(hand)
    assert Capability.GRASP_PARALLEL not in capabilities_for_part(hand)
