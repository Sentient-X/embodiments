from sx_capabilities import Capability, ComponentId

from sx_embodiments import ComponentKind, FrameId, MountedComponent, RootComponent, component_graph
from sx_embodiments.known.panda import FRANKA_SPEC, PANDA_OMRON_SPEC
from sx_embodiments.known.so101 import BIMANUAL_SO101_SPEC


def test_franka_component_graph_is_topological_and_capability_bearing() -> None:
    graph = component_graph(FRANKA_SPEC)
    assert graph == (
        RootComponent(
            component_id=ComponentId("arm"),
            part_id=FRANKA_SPEC.attachments[0].part.part_id,
            kind=ComponentKind.MANIPULATOR,
            frame_id=FrameId("arm"),
            capabilities=(Capability.SPATIAL_MOTION_SE3,),
        ),
        MountedComponent(
            component_id=ComponentId("gripper"),
            part_id=FRANKA_SPEC.attachments[1].part.part_id,
            kind=ComponentKind.EFFECTOR,
            frame_id=FrameId("panda_link8"),
            capabilities=(
                Capability.SPATIAL_MOTION_SE3,
                Capability.GRASP,
                Capability.GRASP_PARALLEL,
            ),
            parent_component_id=ComponentId("arm"),
        ),
    )


def test_mobile_base_is_a_component_not_a_robot_category() -> None:
    base = component_graph(PANDA_OMRON_SPEC)[0]
    assert base.kind is ComponentKind.MOBILE_BASE
    assert Capability.LOCOMOTION_PLANAR in base.capabilities


def test_bimanuality_is_two_effector_components() -> None:
    graph = component_graph(BIMANUAL_SO101_SPEC)
    effectors = tuple(item for item in graph if item.kind is ComponentKind.EFFECTOR)
    assert tuple(item.component_id for item in effectors) == ("left_jaw", "right_jaw")
