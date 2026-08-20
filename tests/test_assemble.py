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


def _hand_document() -> dict[str, object]:
    from sx_embodiments import part_to_dict
    from sx_embodiments.identity import PartId
    from sx_embodiments.layout import (
        ActuatorBinding,
        ActuatorBus,
        ActuatorModel,
        Bounds,
        CoordinateUnit,
        JointAxis,
        JointLayout,
    )

    jaw = GripperSpec(
        part_id=PartId("acme-jaw"),
        layout=JointLayout(
            (
                JointAxis(
                    "jaw",
                    CoordinateUnit.RADIAN,
                    Bounds(0.0, 1.0),
                    ActuatorBinding(ActuatorModel.FEETECH_STS3215, ActuatorBus.FEETECH_SERIAL, 6),
                ),
            )
        ),
    )
    return part_to_dict(jaw)


_HAND_URDF = b"""<robot name="acme-jaw">
  <link name="base"/>
  <link name="finger">
    <inertial><mass value="0.05"/>
      <inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><geometry><mesh filename="package://acme/finger.stl"/></geometry></visual>
  </link>
  <joint name="jaw" type="revolute">
    <parent link="base"/><child link="finger"/>
    <limit lower="0.0" upper="1.0" effort="1" velocity="1"/>
  </joint>
</robot>"""


def test_a_qualified_part_fragment_is_admitted() -> None:
    from sx_embodiments import admit_part

    part = admit_part(_hand_document(), urdf=_HAND_URDF, mesh_paths=frozenset({"acme/finger.stl"}))
    assert str(part.part_id) == "acme-jaw"


def test_admission_refuses_an_unbound_axis() -> None:
    from sx_embodiments import admit_part

    document = _hand_document()
    document["layout"][0]["actuator"] = None
    with pytest.raises(AssemblyError, match="carries no actuator binding"):
        admit_part(document, urdf=_HAND_URDF, mesh_paths=frozenset({"acme/finger.stl"}))


def test_admission_refuses_a_massless_moving_link() -> None:
    from sx_embodiments import admit_part

    massless = _HAND_URDF.replace(b'<mass value="0.05"/>', b'<mass value="0"/>')
    with pytest.raises(EmbodimentSchemaError, match="positive-mass inertial"):
        admit_part(_hand_document(), urdf=massless, mesh_paths=frozenset({"acme/finger.stl"}))


def test_admission_refuses_an_escaping_mesh_reference() -> None:
    from sx_embodiments import admit_part

    with pytest.raises(EmbodimentSchemaError, match="closure law"):
        admit_part(_hand_document(), urdf=_HAND_URDF, mesh_paths=frozenset({"other.stl"}))


def test_admission_refuses_limits_that_disagree() -> None:
    from sx_embodiments import admit_part

    widened = _HAND_URDF.replace(b'upper="1.0"', b'upper="2.0"')
    with pytest.raises(EmbodimentSchemaError, match="limits disagree"):
        admit_part(_hand_document(), urdf=widened, mesh_paths=frozenset({"acme/finger.stl"}))


def test_an_admitted_part_composes_through_assemble() -> None:
    from sx_embodiments import admit_part, composable_parts
    from sx_embodiments.identity import PartId

    hand = admit_part(_hand_document(), urdf=_HAND_URDF, mesh_paths=frozenset({"acme/finger.stl"}))
    robot = embodiments["so101"]
    definition = _so101_definition()
    attachments = definition["attachments"]
    assert isinstance(attachments, list)
    attachments[1] = {
        "instance": "jaw",
        "part_id": "acme-jaw",
        "mount": {"kind": "mounted_on", "parent": "arm", "frame": "gripper_link"},
    }
    merged = {**composable_parts(), PartId("acme-jaw"): hand}
    # The so101 description does not carry the acme jaw, so parity refuses — which is
    # the correct refusal: the composed body needs its own composed description. The
    # part RESOLVES (no unknown-part refusal), which is what this pins.
    with pytest.raises(EmbodimentSchemaError, match="not a movable joint"):
        assemble(definition, urdf=robot.urdf_bytes, parts=merged)


def test_admission_refuses_a_basename_that_stands_in_for_a_path() -> None:
    from sx_embodiments import admit_part

    # The closure law's whole point: a traversal that ends in a staged basename is not
    # a staged asset. This admitted before the exact-match fix.
    escaping = _HAND_URDF.replace(
        b"package://acme/finger.stl", b"package://x/../../../etc/finger.stl"
    )
    with pytest.raises(EmbodimentSchemaError, match="closure law"):
        admit_part(_hand_document(), urdf=escaping, mesh_paths=frozenset({"finger.stl"}))


def test_admission_refuses_a_non_finite_mass() -> None:
    from sx_embodiments import admit_part

    # `nan <= 0.0` is False, so the positive-mass law admitted the one mass every
    # solver dies on. NaN and infinity are both refused, by name.
    for spelling in (b'<mass value="nan"/>', b'<mass value="inf"/>'):
        urdf = _HAND_URDF.replace(b'<mass value="0.05"/>', spelling)
        with pytest.raises(EmbodimentSchemaError, match="must be finite"):
            admit_part(_hand_document(), urdf=urdf, mesh_paths=frozenset({"acme/finger.stl"}))


def test_admission_refuses_non_finite_limits() -> None:
    from sx_embodiments import admit_part

    # `abs(nan - x) > tolerance` is False, so NaN limits agreed with every bound.
    urdf = _HAND_URDF.replace(b'upper="1.0"', b'upper="nan"')
    with pytest.raises(EmbodimentSchemaError, match="must be finite"):
        admit_part(_hand_document(), urdf=urdf, mesh_paths=frozenset({"acme/finger.stl"}))


def test_admission_refuses_a_number_that_is_not_a_number() -> None:
    from sx_embodiments import admit_part

    # A bare ValueError escaped the door's typed handler as a 500; it is an authoring
    # error and says so.
    urdf = _HAND_URDF.replace(b'<mass value="0.05"/>', b'<mass value="heavy"/>')
    with pytest.raises(EmbodimentSchemaError, match="is not a number"):
        admit_part(_hand_document(), urdf=urdf, mesh_paths=frozenset({"acme/finger.stl"}))


def test_admission_refuses_a_hidden_degree_of_freedom() -> None:
    from sx_embodiments import admit_part

    # An undeclared movable joint never reaches the composed body's state vector, so
    # the part would drive fewer axes than the description moves.
    hidden = _HAND_URDF.replace(
        b"</robot>",
        b'<link name="secret"><inertial><mass value="0.01"/>'
        b'<inertia ixx="1e-5" iyy="1e-5" izz="1e-5" ixy="0" ixz="0" iyz="0"/></inertial></link>'
        b'<joint name="secret_axis" type="revolute"><parent link="base"/>'
        b'<child link="secret"/><limit lower="0" upper="1" effort="1" velocity="1"/>'
        b"</joint></robot>",
    )
    with pytest.raises(EmbodimentSchemaError, match="hidden degree of freedom"):
        admit_part(_hand_document(), urdf=hidden, mesh_paths=frozenset({"acme/finger.stl"}))
