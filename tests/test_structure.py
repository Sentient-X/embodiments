from sx_contracts import Capability, ComponentId

from sx_embodiments import ComponentKind, ComponentRole, embodiments


def test_franka_component_graph_is_topological_and_capability_bearing() -> None:
    graph = embodiments["franka"].components
    arm, gripper = graph
    assert arm.component_id == ComponentId("arm")
    assert arm.kind is ComponentKind.MANIPULATOR
    assert arm.capabilities == (Capability.SPATIAL_MOTION_SE3,)
    assert arm.role is ComponentRole.BODY
    assert not arm.mount.parent_instance
    assert gripper.component_id == ComponentId("gripper")
    assert gripper.kind is ComponentKind.EFFECTOR
    assert gripper.capabilities == (
        Capability.SPATIAL_MOTION_SE3,
        Capability.GRASP,
        Capability.GRASP_PARALLEL,
    )
    assert gripper.mount.parent_instance == "arm"
    assert gripper.mount.frame == "panda_link8"


def test_mobile_base_is_a_component_not_a_robot_category() -> None:
    base = embodiments["panda_omron"].components[0]
    assert base.kind is ComponentKind.MOBILE_BASE
    assert Capability.LOCOMOTION_PLANAR in base.capabilities


def test_bimanuality_is_two_effector_components() -> None:
    graph = embodiments["bimanual-so101"].components
    effectors = tuple(item for item in graph if item.kind is ComponentKind.EFFECTOR)
    assert tuple(item.component_id for item in effectors) == ("left_jaw", "right_jaw")
