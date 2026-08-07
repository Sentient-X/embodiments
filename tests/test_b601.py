"""The reBot B601-DM family: one arm block, composed three ways, one description.

``b601-dm`` (single follower), ``bimanual-b601-dm`` (the pair the pod runs), and
``b601-dm-station`` (that pair with its two leaders and three cameras) all resolve from the
public registry over the same vendored URDF, so the derived wire layout is pinned here for
each of them.
"""

import xml.etree.ElementTree as ET

import pytest
from sx_contracts import Capability
from sx_contracts.assets import AssetFormat, AssetRole

from sx_embodiments import (
    ChannelKind,
    ComponentRole,
    CoordinateUnit,
    Embodiment,
    EmbodimentKind,
    EmbodimentName,
    MountedOn,
    RootMount,
    embodiments,
)
from sx_embodiments.compose import EmbodimentDefinition
from sx_embodiments.errors import CompositionError
from sx_embodiments.known import DEVELOPMENT_EMBODIMENTS
from sx_embodiments.known.b601 import (
    B601_ARM,
    B601_DM_SPEC,
    B601_DM_STATION_SPEC,
    B601_DM_URDF,
    B601_GRIPPER,
    BIMANUAL_B601_DM_SPEC,
    D435I_30,
)
from sx_embodiments.parts import CameraModality, SensorModel

_ARM_BLOCK = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_joint1")


def _block(instance_prefix: str) -> tuple[str, ...]:
    arm, gripper = f"{instance_prefix}arm", f"{instance_prefix}gripper"
    return (*(f"{arm}/{name}" for name in _ARM_BLOCK[:6]), f"{gripper}/{_ARM_BLOCK[6]}")


def test_all_three_resolve_by_name_from_the_public_registry() -> None:
    """Promotion: the family is registered, so nothing is held back any more."""
    for name in ("b601-dm", "bimanual-b601-dm", "b601-dm-station"):
        embodiment = embodiments[name]
        assert isinstance(embodiment, Embodiment)
        assert str(embodiment.name) == name
        assert embodiment.lineage.family == "rebot-b601"
        assert embodiments[embodiment.id] is embodiment  # content id resolves too
        assert EmbodimentName(name) not in DEVELOPMENT_EMBODIMENTS


def test_the_three_share_one_authoritative_description() -> None:
    """One URDF, attached on the arm block, so the three cannot drift apart."""
    urdfs = {embodiments[name].urdf for name in ("b601-dm", "bimanual-b601-dm", "b601-dm-station")}
    assert len(urdfs) == 1
    urdf = urdfs.pop()
    assert urdf.uri == "package://sx-embodiments/b601_dm/reBot_B601_DM_with_gripper.urdf"
    assert urdf.sha256 == B601_DM_URDF.sha256
    assert urdf.format is AssetFormat.URDF and urdf.role is AssetRole.DESCRIPTION
    provenance = urdf.provenance
    assert provenance.repository == "https://github.com/Seeed-Projects/reBotArmController_ROS2"
    assert provenance.revision == "a61efe4fa223ca50cd721ef8ebe4a60e90f28bfd"
    assert provenance.license_id == "Apache-2.0"
    # The digest is of the vendored file, so the rewrite must be declared, not implied.
    assert provenance.generator is not None and "meshes/" in provenance.generator


def test_the_vendored_mesh_closure_resolves_from_the_embodiment() -> None:
    """All ten STLs the description references exist beside it on disk."""
    urdf_path = embodiments["b601-dm"].urdf_path
    root = ET.fromstring(embodiments["b601-dm"].urdf_bytes)
    referenced = {mesh.get("filename") for mesh in root.iter("mesh")}
    assert referenced == {
        f"meshes/{name}.STL"
        for name in (
            "base_link",
            "link1",
            "link2",
            "link3",
            "link4",
            "link5",
            "link6",
            "gripper_link",
            "gripper_left",
            "gripper_right",
        )
    }
    for filename in referenced:
        assert filename is not None
        assert (urdf_path.parent / filename).is_file(), filename
    # The license evidence upstream ships (there is no LICENSE file) travels with them.
    assert (urdf_path.parent / "package.xml").is_file()


def test_single_arm_state_is_the_seven_channel_follower_block() -> None:
    state = embodiments["b601-dm"].state
    assert state.names == _block("")
    assert state.width == 7
    assert state.arm_joint_count == 6
    assert state.gripper_count == 1
    assert state.indices(ChannelKind.ARM_JOINT) == (0, 1, 2, 3, 4, 5)
    assert state.indices(ChannelKind.GRIPPER) == (6,)
    assert embodiments["b601-dm"].kind is EmbodimentKind.ROBOT
    assert embodiments["b601-dm"].single_arm.dof == 6
    assert embodiments["b601-dm"].gripper_travel_m == (0.0, 0.143)
    assert embodiments["b601-dm"].cameras == ()


def test_bimanual_state_is_left_block_then_right_block() -> None:
    """``[left 0..6 | right 7..13]`` — exactly the declared attachment order."""
    state = embodiments["bimanual-b601-dm"].state
    assert state.names == (*_block("left_"), *_block("right_"))
    assert state.width == 14
    assert state.arm_joint_count == 12
    assert state.gripper_count == 2
    assert state.indices(ChannelKind.ARM_JOINT) == (0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12)
    assert state.indices(ChannelKind.GRIPPER) == (6, 13)
    state.validate_widths(joint_dim=12, gripper_dim=2)


def test_the_station_state_is_the_two_followers_and_nothing_else() -> None:
    """Leaders contribute identity and assets, never channels."""
    station = embodiments["b601-dm-station"]
    assert station.kind is EmbodimentKind.TELEOP_STATION
    assert station.state.names == embodiments["bimanual-b601-dm"].state.names
    assert station.state.width == 14
    assert not any("leader" in name for name in station.state.names)
    assert tuple(
        component.instance
        for component in station.components
        if component.role is ComponentRole.LEADER
    ) == ("left_leader", "right_leader")


def test_units_and_limits_are_the_descriptions_own() -> None:
    """Six revolute radians plus one prismatic meter channel, straight from the URDF."""
    state = embodiments["bimanual-b601-dm"].state
    described = {
        "joint1": (-2.8, 2.8, CoordinateUnit.RADIAN),
        "joint2": (-3.14, 0.0, CoordinateUnit.RADIAN),
        "joint3": (-3.14, 0.0, CoordinateUnit.RADIAN),
        "joint4": (-1.87, 1.57, CoordinateUnit.RADIAN),
        "joint5": (-1.57, 1.57, CoordinateUnit.RADIAN),
        "joint6": (-3.14, 3.14, CoordinateUnit.RADIAN),
        "gripper_joint1": (0.0, 0.0715, CoordinateUnit.METER),
    }
    for coordinate in state.coordinates:
        lower, upper, unit = described[coordinate.joint_name]
        assert (coordinate.lower, coordinate.upper, coordinate.unit) == (lower, upper, unit)
    assert B601_ARM.joint_units == (CoordinateUnit.RADIAN,) * 6
    assert B601_GRIPPER.joint_units == (CoordinateUnit.METER,)


def test_station_camera_names_are_the_pod_agents_dataset_keys() -> None:
    """``teleop_camera_ports.CAMERA_ROLES``: two D405 wrists and one overhead D435i."""
    bindings = embodiments["b601-dm-station"].cameras
    assert tuple(binding.name for binding in bindings) == (
        "left_wrist_camera",
        "right_wrist_camera",
        "top_camera",
    )
    wrists = bindings[:2]
    assert all(binding.camera.model is SensorModel.REALSENSE_D405 for binding in wrists)
    assert all(
        isinstance(binding.mount, MountedOn) and binding.mount.parent.endswith("_arm")
        for binding in wrists
    )
    top = bindings[2]
    assert top.camera == D435I_30
    assert top.camera.model is SensorModel.REALSENSE_D435  # the deployed unit is a D435i
    assert top.camera.modality is CameraModality.RGBD
    assert top.camera.resolution is None  # D435 depth and RGB natives differ
    assert isinstance(top.mount, RootMount)


def test_mount_frames_name_real_links_in_the_authoritative_description() -> None:
    links = {
        link.get("name") for link in ET.fromstring(embodiments["b601-dm"].urdf_bytes).iter("link")
    }
    for name in ("b601-dm", "bimanual-b601-dm", "b601-dm-station"):
        for component in embodiments[name].components:
            if isinstance(component.mount, MountedOn):
                assert component.mount.frame in links, f"{name}: {component.instance}"


def test_capabilities_are_a_grasping_pair_plus_three_rgbd_sensors() -> None:
    profile = {
        str(entry.component_id): entry.capabilities
        for entry in embodiments["b601-dm-station"].capabilities.components
    }
    # An undescribed leader device offers nothing derivable, so it is absent entirely.
    assert "left_leader" not in profile and "right_leader" not in profile
    assert profile["left_arm"].values == frozenset({Capability.SPATIAL_MOTION_SE3})
    assert profile["right_arm"].values == frozenset({Capability.SPATIAL_MOTION_SE3})
    grasp = frozenset(
        {Capability.SPATIAL_MOTION_SE3, Capability.GRASP, Capability.GRASP_PARALLEL}
    )
    assert profile["left_gripper"].values == grasp
    assert profile["right_gripper"].values == grasp
    sensing = frozenset({Capability.SENSING_RGB, Capability.SENSING_DEPTH})
    assert profile["left_wrist_camera"].values == sensing
    assert profile["right_wrist_camera"].values == sensing
    assert profile["top_camera"].values == sensing


def test_content_ids_are_distinct_and_round_trip() -> None:
    """Three compositions of one part set are three revisions, not one."""
    names = ("b601-dm", "bimanual-b601-dm", "b601-dm-station")
    ids = {name: embodiments[name].id for name in names}
    assert len(set(ids.values())) == 3
    for name, identity in ids.items():
        embodiment = embodiments[name]
        assert len(identity) == 64
        assert Embodiment.from_json(embodiment.to_json()) == embodiment


def test_a_station_without_a_leader_is_rejected() -> None:
    """The leaders are load-bearing, not decoration."""
    with pytest.raises(CompositionError):
        EmbodimentDefinition(
            name=EmbodimentName("b601-dm-followers-only"),
            label="B601-DM followers with no leader",
            kind=EmbodimentKind.TELEOP_STATION,
            lineage=B601_DM_STATION_SPEC.lineage,
            attachments=B601_DM_STATION_SPEC.body_attachments(),
        )


def test_the_declared_graph_is_topologically_ordered() -> None:
    for spec in (B601_DM_SPEC, BIMANUAL_B601_DM_SPEC, B601_DM_STATION_SPEC):
        declared: set[str] = set()
        for component in spec.attachments:
            if isinstance(component.mount, MountedOn):
                assert component.mount.parent in declared, component.instance
            declared.add(component.instance)
        assert spec.layout_declared()
