import dataclasses
import types

from sx_contracts import Capability, ComponentId

import sx_embodiments
from sx_embodiments import embodiments
from sx_embodiments.compose import ComponentKind, ComponentRole, MountedOn, RootMount

# The whole read-only public surface. An embodiment is obtained from the registry, never
# assembled: the arguments ``Embodiment(...)`` requires — components, attachments, parts,
# joint layouts, lineage, kinds — are absent here on purpose, so hardware facts keep one
# construction site. Adding a name is a deliberate widening; adding construction material
# hands a second source of truth to every consumer.
PUBLIC_SURFACE = frozenset(
    {
        "AssetDigestMismatchError",
        "AssetsUnavailableError",
        "BaseMount",
        "Bounds",
        "ChannelKind",
        "ComponentGraphError",
        "CompositionError",
        "CoordinateBounds",
        "CoordinateUnit",
        "Embodiment",
        "EmbodimentError",
        "EmbodimentId",
        "EmbodimentName",
        "EmbodimentSchemaError",
        "GripperKinematicsError",
        "InvalidCameraMountError",
        "LayoutError",
        "LensProjection",
        "MissingUrdfError",
        "MountKind",
        "PartValidationError",
        "Unbounded",
        "UnknownEmbodimentError",
        "embodiments",
        "resolve_asset",
    }
)


def test_public_surface_offers_no_way_to_assemble_an_embodiment() -> None:
    exported = frozenset(sx_embodiments.__all__)
    assert exported == PUBLIC_SURFACE
    leaked = {
        name
        for name, value in vars(sx_embodiments).items()
        if not name.startswith("_")
        and name not in exported
        and not isinstance(value, types.ModuleType)
    }
    assert not leaked, f"names reachable from the package root but not declared: {sorted(leaked)}"
    required = {
        field.name
        for field in dataclasses.fields(sx_embodiments.Embodiment)
        if field.default is dataclasses.MISSING
    }
    assert required == {"name", "label", "kind", "lineage", "components", "assets"}
    # Three of the six required arguments have no public spelling at all, so the constructor
    # is unreachable from outside without deliberately importing a private module.
    assert not exported & {"Component", "EmbodimentKind", "Lineage"}


def test_franka_component_graph_is_topological_and_capability_bearing() -> None:
    graph = embodiments["franka"].components
    arm, gripper = graph
    assert arm.component_id == ComponentId("arm")
    assert arm.kind is ComponentKind.MANIPULATOR
    assert arm.capabilities == (Capability.SPATIAL_MOTION_SE3,)
    assert arm.role is ComponentRole.BODY
    assert isinstance(arm.mount, RootMount)
    assert gripper.component_id == ComponentId("gripper")
    assert gripper.kind is ComponentKind.EFFECTOR
    assert gripper.capabilities == (
        Capability.SPATIAL_MOTION_SE3,
        Capability.GRASP,
        Capability.GRASP_PARALLEL,
    )
    assert isinstance(gripper.mount, MountedOn)
    assert gripper.mount.parent == "arm"
    assert gripper.mount.frame == "panda_link8"


def test_mobile_base_is_a_component_not_a_robot_category() -> None:
    base = embodiments["panda_omron"].components[0]
    assert base.kind is ComponentKind.MOBILE_BASE
    assert Capability.LOCOMOTION_PLANAR in base.capabilities


def test_bimanuality_is_two_effector_components() -> None:
    graph = embodiments["bimanual-so101"].components
    effectors = tuple(item for item in graph if item.kind is ComponentKind.EFFECTOR)
    assert tuple(item.component_id for item in effectors) == ("left_jaw", "right_jaw")
