"""The reBot B601-DM family: one arm block, composed into complete descriptions.

``b601-dm`` (single follower), ``bimanual-b601-dm`` (the pair the pod runs), and
The single and bimanual robot bodies resolve publicly. The station description remains a
development record until its installed camera extrinsics are captured.
"""

import xml.etree.ElementTree as ET

import pytest
from sx_contracts import Capability
from sx_contracts.assets import AssetFormat, AssetRole

from sx_embodiments import (
    ChannelKind,
    CoordinateUnit,
    Embodiment,
    EmbodimentName,
    embodiments,
)
from sx_embodiments.compose import ComponentRole, EmbodimentDefinition, MountedOn
from sx_embodiments.embodiment import embodiment_from_definition
from sx_embodiments.errors import CompositionError
from sx_embodiments.identity import EmbodimentKind
from sx_embodiments.known import DEVELOPMENT_EMBODIMENTS
from sx_embodiments.known.b601 import (
    B601_ARM,
    B601_DM_SPEC,
    B601_DM_STATION_SPEC,
    B601_DM_STATION_URDF,
    B601_DM_URDF,
    B601_GRIPPER,
    BIMANUAL_B601_DM_SPEC,
    BIMANUAL_B601_DM_URDF,
)

_ARM_BLOCK = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_joint1")


def _block(instance_prefix: str) -> tuple[str, ...]:
    arm, gripper = f"{instance_prefix}arm", f"{instance_prefix}gripper"
    return (*(f"{arm}/{name}" for name in _ARM_BLOCK[:6]), f"{gripper}/{_ARM_BLOCK[6]}")


def _station() -> Embodiment:
    return embodiment_from_definition(B601_DM_STATION_SPEC)


def test_complete_robot_bodies_resolve_from_the_public_registry() -> None:
    for name in ("b601-dm", "bimanual-b601-dm"):
        embodiment = embodiments[name]
        assert isinstance(embodiment, Embodiment)
        assert str(embodiment.name) == name
        assert embodiment.lineage.family == "rebot-b601"
        assert embodiments[embodiment.id] is embodiment  # content id resolves too
        assert EmbodimentName(name) not in DEVELOPMENT_EMBODIMENTS
    assert EmbodimentName("b601-dm-station") in DEVELOPMENT_EMBODIMENTS
    assert "b601-dm-station" not in set(embodiments)


def test_each_composition_has_a_complete_authoritative_description() -> None:
    expected = {
        "b601-dm": (B601_DM_URDF, 10),
        "bimanual-b601-dm": (BIMANUAL_B601_DM_URDF, 21),
    }
    for name, (asset, link_count) in expected.items():
        embodiment = embodiments[name]
        assert embodiment.urdf.sha256 == asset.sha256
        assert embodiment.urdf.format is AssetFormat.URDF
        assert embodiment.urdf.role is AssetRole.DESCRIPTION
        root = ET.fromstring(embodiment.urdf_bytes)
        assert len(root.findall("link")) == link_count
        assert embodiment.urdf.provenance.repository == (
            "https://github.com/Seeed-Projects/reBotArmController_ROS2"
        )
    assert embodiments["b601-dm"].urdf.provenance.generator is not None
    assert "meshes/" in embodiments["b601-dm"].urdf.provenance.generator
    for name in ("bimanual-b601-dm",):
        assert embodiments[name].urdf.provenance.generator == (
            "sx-embodiments/tools/compose_registered_urdfs.py"
        )
    assert len(ET.parse(B601_DM_STATION_URDF.path()).getroot().findall("link")) == 26


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
    station = _station()
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


def test_station_is_not_promoted_without_installed_camera_extrinsics() -> None:
    entry = DEVELOPMENT_EMBODIMENTS[EmbodimentName("b601-dm-station")]
    assert entry.spec is B601_DM_STATION_SPEC
    assert entry.reason.value == "missing_camera_installation"
    assert _station().cameras == ()


def test_mount_frames_name_real_links_in_the_authoritative_description() -> None:
    values = {
        "b601-dm": embodiments["b601-dm"],
        "bimanual-b601-dm": embodiments["bimanual-b601-dm"],
        "b601-dm-station": _station(),
    }
    for name, embodiment in values.items():
        links = {link.get("name") for link in ET.fromstring(embodiment.urdf_bytes).iter("link")}
        for component in embodiment.components:
            assert component.mount.frame in links, f"{name}: {component.instance}"


def test_station_marks_only_unsurfaced_devices_for_honest_preview() -> None:
    root = ET.fromstring(_station().urdf_bytes)
    anchors = {
        link.get("name"): (
            link.get("data-preview-kind"),
            link.get("data-preview-label"),
        )
        for link in root.findall("link")
        if link.get("data-preview-kind") is not None
    }
    assert anchors == {
        "left_leader": ("leader", "Left leader · surface unavailable"),
        "right_leader": ("leader", "Right leader · surface unavailable"),
        "left_wrist_camera": ("camera", "Left wrist camera"),
        "right_wrist_camera": ("camera", "Right wrist camera"),
        "top_camera": ("camera", "Top camera"),
    }


def test_development_station_capabilities_are_only_the_proven_follower_pair() -> None:
    profile = {
        str(entry.component_id): entry.capabilities for entry in _station().capabilities.components
    }
    # An undescribed leader device offers nothing derivable, so it is absent entirely.
    assert "left_leader" not in profile and "right_leader" not in profile
    assert profile["left_arm"].values == frozenset({Capability.SPATIAL_MOTION_SE3})
    assert profile["right_arm"].values == frozenset({Capability.SPATIAL_MOTION_SE3})
    grasp = frozenset({Capability.SPATIAL_MOTION_SE3, Capability.GRASP, Capability.GRASP_PARALLEL})
    assert profile["left_gripper"].values == grasp
    assert profile["right_gripper"].values == grasp


def test_content_ids_are_distinct_and_round_trip() -> None:
    """Three compositions of one part set are three revisions, not one."""
    values = {
        "b601-dm": embodiments["b601-dm"],
        "bimanual-b601-dm": embodiments["bimanual-b601-dm"],
        "b601-dm-station": _station(),
    }
    ids = {name: embodiment.id for name, embodiment in values.items()}
    assert len(set(ids.values())) == 3
    for name, identity in ids.items():
        embodiment = values[name]
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
