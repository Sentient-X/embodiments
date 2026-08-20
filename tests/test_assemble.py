"""The assembly door: untrusted definitions mint validated bodies, or refuse by name."""

from collections.abc import Mapping

import pytest

from sx_embodiments import (
    AssemblyError,
    EmbodimentSchemaError,
    assemble,
    composable_parts,
    embodiments,
)
from sx_embodiments.parts import GraspKind, GripperSpec


def _so101_definition() -> dict[str, object]:
    source = embodiments["so101"].to_dict()
    return {
        "name": "so101",
        "label": "SO-101 (single arm)",
        "kind": "robot",
        "lineage": {"family": "so101", "variant": "", "revision": ""},
        "attachments": [
            {
                "instance": "arm",
                "part_id": "so101-arm",
                "mount": {"kind": "root", "frame": "base_link"},
            },
            {
                "instance": "jaw",
                "part_id": "so101-jaw",
                "mount": {"kind": "mounted_on", "parent": "arm", "frame": "gripper_link"},
            },
        ],
        "assets": source["assets"],
        "rates": None,
        "base_mount": None,
    }


def test_the_parts_catalog_is_derived_from_the_registries() -> None:
    parts = composable_parts()
    assert "so101-arm" in parts
    assert "so101-jaw" in parts
    jaw = parts["so101-jaw"]
    assert isinstance(jaw, GripperSpec)
    assert jaw.grasp is GraspKind.PARALLEL


def test_assembling_the_so101_definition_reproduces_the_governed_identity() -> None:
    robot = embodiments["so101"]
    assembled = assemble(_so101_definition(), urdf=robot.urdf_bytes)
    assert assembled.id == robot.id
    assert assembled == robot


def test_an_unknown_part_refuses_naming_the_catalog() -> None:
    definition = _so101_definition()
    attachments = definition["attachments"]
    assert isinstance(attachments, list)
    attachments[0]["part_id"] = "acme-arm"
    with pytest.raises(AssemblyError, match="not in the composable catalog"):
        assemble(definition, urdf=embodiments["so101"].urdf_bytes)


def test_an_unbound_part_refuses_because_assembled_bodies_are_drivable() -> None:
    definition = _so101_definition()
    attachments = definition["attachments"]
    assert isinstance(attachments, list)
    attachments[0]["part_id"] = "panda-arm"
    with pytest.raises(AssemblyError, match="carries no actuator binding"):
        assemble(definition, urdf=embodiments["so101"].urdf_bytes)


def test_only_robots_are_assembled() -> None:
    definition = _so101_definition()
    definition["kind"] = "teleop_station"
    with pytest.raises(AssemblyError, match="assembles robots"):
        assemble(definition, urdf=embodiments["so101"].urdf_bytes)


def test_description_bytes_must_match_the_declared_asset() -> None:
    with pytest.raises(ValueError, match="sha256"):
        assemble(_so101_definition(), urdf=b"<robot name='forged'/>")


def test_joint_parity_refuses_a_description_that_disagrees() -> None:
    # The bimanual description is a real, sha-verifiable URDF whose joints are all
    # side-prefixed, so the single-arm coordinates cannot resolve in it.
    bimanual = embodiments["bimanual-so101"]
    description = bimanual.to_dict()["assets"]
    assert isinstance(description, list)
    definition = _so101_definition()
    definition["assets"] = [
        asset
        for asset in description
        if asset["asset"]["role"] == "description" or asset["asset"]["role"] == "other"
    ]
    with pytest.raises(EmbodimentSchemaError, match="not a movable joint"):
        assemble(definition, urdf=bimanual.urdf_bytes)


def test_the_definition_is_decoded_strictly() -> None:
    definition: Mapping[str, object] = {**_so101_definition(), "surprise": True}
    with pytest.raises(EmbodimentSchemaError, match="expected exactly these fields"):
        assemble(definition, urdf=embodiments["so101"].urdf_bytes)
