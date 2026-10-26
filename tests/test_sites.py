"""Physical site facts survive stored documents, kinematics and arbitrary composition."""

import json
import xml.etree.ElementTree as ET
from dataclasses import replace
from importlib import resources

import pytest
from sx_contracts import ComponentId

from sx_embodiments import (
    BodySite,
    Embodiment,
    PlacedEmbodiment,
    compose_embodiments,
    development_embodiments,
    embodiments,
    migrate,
)
from sx_embodiments.assembly import Pose
from sx_embodiments.composition import compose_build
from sx_embodiments.embodiment import embodiment_from_definition
from sx_embodiments.errors import EmbodimentSchemaError
from sx_embodiments.kinematics import Kinematics, compose, from_xyz_rpy
from sx_embodiments.known import definitions
from sx_embodiments.known.b601 import B601_DM_MODULE, B601_GRIPPER_MODULE
from sx_embodiments.mechanical import Build, Mate, Placed


def test_site_offset_follows_the_declared_link_and_round_trips() -> None:
    body = embodiments["rby1"]
    tree = Kinematics.of(body.urdf_bytes)
    positions = dict.fromkeys(body.joint_names, 0.0)
    positions["left_arm_3"] = -0.6
    site = replace(body.site("left_wrist"), pose=Pose((0.01, 0.02, 0.03), (0.2, 0.1, -0.1)))
    assert site.transform(tree, "base", positions) == pytest.approx(
        compose(
            tree.pose("base", "link_left_arm_6", positions),
            from_xyz_rpy(site.pose.xyz, site.pose.rpy),
        )
    )
    assert BodySite.from_dict(site.to_dict()) == site
    revised = replace(body, sites=(site,))
    assert revised.id != body.id
    assert Embodiment.from_dict(revised.to_dict()) == revised


def test_any_number_of_sites_and_bodies_preserves_names_and_geometry() -> None:
    body = embodiments["rby1"]
    # The names are mechanical, not a fixed anatomical enum or a two-arm maximum.
    third = replace(body.site("left_wrist"), name="inspection-probe")
    body = replace(body, sites=(*body.sites, third))
    composed = compose_embodiments(
        "three",
        {
            name: PlacedEmbodiment(body, xyz=(float(index), 0.0, 0.0))
            for index, name in enumerate(("alpha", "beta", "gamma"))
        },
        label="Three bodies",
    )
    tree = Kinematics.of(composed.urdf_bytes)
    positions = dict.fromkeys(composed.joint_names, 0.0)
    source_tree = Kinematics.of(body.urdf_bytes)
    source_positions = dict.fromkeys(body.joint_names, 0.0)
    for index, name in enumerate(("alpha", "beta", "gamma")):
        for site in body.sites:
            result = composed.site(f"{name}/{site.name}")
            assert result.component_id == f"{name}.{site.component_id}"
            assert result.link == f"{name}/{site.link}"
            assert result.transform(tree, "assembly", positions) == pytest.approx(
                compose(
                    from_xyz_rpy((float(index), 0.0, 0.0), (0.0, 0.0, 0.0)),
                    site.transform(source_tree, "base", source_positions),
                )
            )


def test_fitted_sites_follow_removal_and_rejoining() -> None:
    gripper = replace(
        B601_GRIPPER_MODULE,
        sites=(BodySite("jaw_mount", ComponentId("gripper"), "gripper_joint_mount"),),
    )
    arm = replace(B601_DM_MODULE, sockets=(replace(B601_DM_MODULE.sockets[0], fitted=gripper),))
    fitted = Build(placed=(Placed("tool", arm),), robot="fitted", root="world")
    body = compose_build(fitted, name="fitted", label="Fitted")
    assert body.site("tool_jaw_mount").link == "tool_link6"
    reopened = replace(fitted, mates=(Mate("jaw", gripper, "tool", "flange"),))
    detached = compose_build(reopened, name="separate", label="Separate")
    assert "tool_jaw_mount" not in {site.name for site in detached.sites}
    assert detached.site("jaw_jaw_mount").link == "jaw_gripper_joint_mount"
    assert detached.site("tool_wrist").component_id == "tool_arm"
    positions = dict.fromkeys(body.joint_names, 0.0)
    detached_positions = dict.fromkeys(detached.joint_names, 0.0)
    assert body.site("tool_jaw_mount").transform(
        Kinematics.of(body.urdf_bytes), "world", positions
    ) == pytest.approx(
        detached.site("jaw_jaw_mount").transform(
            Kinematics.of(detached.urdf_bytes), "world", detached_positions
        )
    )
    rejoined = replace(fitted, mates=(Mate("jaw", gripper, "tool", "flange", namespaced=False),))
    unprefixed = compose_build(rejoined, name="rejoined", label="Rejoined")
    assert unprefixed.site("jaw_mount").transform(
        Kinematics.of(unprefixed.urdf_bytes), "world", dict.fromkeys(unprefixed.joint_names, 0.0)
    ) == pytest.approx(
        body.site("tool_jaw_mount").transform(Kinematics.of(body.urdf_bytes), "world", positions)
    )


def test_site_boundaries_refuse_missing_owners_links_names_and_bad_poses() -> None:
    body = embodiments["rby1"]
    with pytest.raises(EmbodimentSchemaError, match="no declared"):
        body.site("imagined")
    with pytest.raises(EmbodimentSchemaError, match="unique"):
        replace(body, sites=(body.sites[0], body.sites[0]))
    with pytest.raises(EmbodimentSchemaError, match="absent component"):
        replace(body, sites=(replace(body.sites[0], component_id=ComponentId("absent")),))
    invalid = replace(body, sites=(replace(body.sites[0], link="absent"),))
    with pytest.raises(EmbodimentSchemaError, match="absent from"):
        invalid.with_assets(body.assets, urdf=body.urdf_bytes)
    wrong_branch = replace(body, sites=(replace(body.site("left_wrist"), link="link_right_arm_6"),))
    with pytest.raises(EmbodimentSchemaError, match="not owned by"):
        wrong_branch.with_assets(body.assets, urdf=body.urdf_bytes)
    document = body.sites[0].to_dict()
    document["xyz"] = [1.0, 2.0]
    with pytest.raises(EmbodimentSchemaError):
        BodySite.from_dict(document)


def test_registered_sites_are_real_and_capture_sites_do_not_imply_anatomy() -> None:
    for definition in definitions():
        try:
            body = embodiment_from_definition(definition)
        except EmbodimentSchemaError:
            continue
        tree = Kinematics.of(body.urdf_bytes)
        assert all(site.link in tree.links for site in body.sites)
    assert {site.name for site in embodiments["quest-ego"].sites} == {"head"}
    assert {site.name for site in development_embodiments["yubi"].sites} == {
        "left_hand",
        "right_hand",
    }
    assert "footprint" not in {site.name for site in embodiments["unitree-g1"].sites}


def test_schema15_migration_preserves_identity_without_inventing_sites() -> None:
    documents = [
        json.loads(entry.read_text())
        for entry in (resources.files("sx_embodiments") / "historical").iterdir()
        if entry.name.endswith(".json")
    ]
    previous = [document for document in documents if document["schema_version"] == 15]
    assert len(previous) >= 25
    for document in previous:
        migration = migrate(document)
        assert str(migration.source_id) == document["id"]
        assert migration.embodiment.sites == ()
        with pytest.raises(EmbodimentSchemaError, match="no declared"):
            migration.embodiment.site("left_wrist")


def test_authored_module_keeps_sites_and_old_documents_invent_none() -> None:
    from sx_embodiments import part_to_dict
    from sx_embodiments.assemble import admit_module, module_from_dict, module_to_dict
    from sx_embodiments.known.b601 import B601_GRIPPER
    from sx_embodiments.mechanical import InterfaceId

    description = B601_GRIPPER_MODULE.description
    assert description is not None
    urdf = description.urdf()
    site = BodySite("jaw", ComponentId("gripper"), "gripper_joint_mount")
    meshes = frozenset(element.attrib["filename"] for element in ET.fromstring(urdf).iter("mesh"))
    module = admit_module(
        "jaw",
        "Jaw",
        urdf=urdf,
        plug=InterfaceId("seeed-b601-flange"),
        sockets=(),
        part=part_to_dict(B601_GRIPPER),
        sites=(site,),
        mesh_paths=meshes,
    )
    document = module_to_dict(module)
    restored = module_from_dict(document, urdf=urdf, mesh_paths=meshes)
    assert restored.sites == (site,)
    old = {key: value for key, value in document.items() if key != "sites"}
    old["schema"] = "sx.authored-module/v1"
    assert module_from_dict(old, urdf=urdf, mesh_paths=meshes).sites == ()


def test_rby1_native_gripper_units_bounds_and_opposed_fingers_match_its_urdf() -> None:
    from sx_embodiments import CoordinateUnit, historical_document
    from sx_embodiments.identity import EmbodimentId, EmbodimentName
    from sx_embodiments.parts import GripperSpec

    body = embodiments["rby1"]
    document = ET.fromstring(body.urdf_bytes)
    joints = {joint.attrib["name"]: joint for joint in document.findall("joint")}
    for coordinate, name in zip(body.state.coordinates, body.joint_names, strict=True):
        joint = joints[name]
        if joint.attrib["type"] == "prismatic":
            assert coordinate.unit is CoordinateUnit.METER
        else:
            assert coordinate.unit is CoordinateUnit.RADIAN
        limit = joint.find("limit")
        if limit is not None and "lower" in limit.attrib:
            assert coordinate.lower == pytest.approx(float(limit.attrib["lower"]))
            assert coordinate.upper == pytest.approx(float(limit.attrib["upper"]))
    tree = Kinematics.of(body.urdf_bytes)
    for side, suffix in (("right", "r"), ("left", "l")):
        component = next(c for c in body.components if c.instance == f"{side}_gripper")
        assert isinstance(component.part, GripperSpec)
        mimic = component.part.mimic_joints[0]
        assert mimic.multiplier == -1.0
        positions = {f"gripper_finger_{suffix}1": 0.025, f"gripper_finger_{suffix}2": -0.025}
        first = tree.pose(f"ee_{side}", f"ee_finger_{suffix}1", positions)
        second = tree.pose(f"ee_{side}", f"ee_finger_{suffix}2", positions)
        assert first[7] == pytest.approx(0.025)
        assert second[7] == pytest.approx(-0.025)
    old = migrate(
        historical_document(
            EmbodimentName("rby1"),
            EmbodimentId("7c6fbed4dcc92f907754910a1c59ece4ed52f50462802d2276823decd6d4fe9d"),
        )
    ).embodiment
    # Schema migration preserves recorded semantics, including this old incorrect unit.
    original = next(c for c in old.state.coordinates if c.joint_name == "gripper_finger_l1")
    assert original.unit is CoordinateUnit.RADIAN and original.lower == -0.05
