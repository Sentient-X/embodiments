"""An assembly is the one source of a built description and of its component graph."""

import hashlib
from xml.etree import ElementTree as ET

import pytest

from sx_embodiments import PlacedEmbodiment, compose_embodiments, embodiments
from sx_embodiments.assembly import Assembly, Frame, Member, Pose
from sx_embodiments.assets import AssetRole
from sx_embodiments.compose import MountedOn, RootMount
from sx_embodiments.errors import CompositionError
from sx_embodiments.known import definitions
from sx_embodiments.known.so101 import SO101_SPEC, SO101_URDF

ASSEMBLED = tuple(d for d in definitions() if d.assembly is not None)


def test_every_multi_device_description_is_rendered_from_its_declaration() -> None:
    assert {str(d.name) for d in ASSEMBLED} == {
        "bimanual-so101",
        "bimanual-b601-dm",
        "b601-dm-station",
        "piperx-station",
    }
    for definition in ASSEMBLED:
        assert definition.assembly is not None
        (description,) = (a for a in definition.extra_assets if a.role is AssetRole.DESCRIPTION)
        assert definition.assembly.urdf() == description.path().read_bytes(), (
            f"{definition.name}: run tools/compose_registered_urdfs.py and repin the digest"
        )


def test_no_body_is_built_from_other_descriptions_without_its_assembly() -> None:
    """Adoption is total: a description made by composing others has a declaration."""
    for definition in definitions():
        description = next(
            (a for a in definition.extra_assets if a.role is AssetRole.DESCRIPTION), None
        )
        composed = description is not None and description.provenance.generator == (
            "sx-embodiments/tools/compose_registered_urdfs.py"
        )
        assert composed == (definition.assembly is not None), definition.name


def test_every_component_frame_is_a_link_of_the_rendered_description() -> None:
    for definition in ASSEMBLED:
        assert definition.assembly is not None
        root = ET.fromstring(definition.assembly.urdf())
        links = {link.attrib["name"] for link in root.findall("link")}
        for component in definition.attachments:
            assert component.mount.frame in links, (definition.name, component.instance)


def test_member_components_are_namespaced_like_their_links() -> None:
    pair = Assembly(
        "pair", "origin", (Member("left", SO101_URDF), Member("right", SO101_URDF, Pose((0, 1, 0))))
    )
    arm, jaw = pair.components_of("left", SO101_SPEC.attachments)
    assert (arm.instance, arm.mount.frame) == ("left_arm", "left_base_link")
    assert jaw.mount == MountedOn("left_arm", "left_gripper_link")
    assert jaw.part is SO101_SPEC.attachments[1].part
    with pytest.raises(CompositionError):
        pair.components_of("middle", SO101_SPEC.attachments)


def test_rendering_is_deterministic_and_refuses_dangling_mounts() -> None:
    one = Assembly("one", "origin", (Member("arm", SO101_URDF),))
    assert one.urdf() == one.urdf()
    with pytest.raises(CompositionError):
        Assembly("bad", "origin", (Member("arm", SO101_URDF, parent="nowhere"),)).urdf()
    with pytest.raises(CompositionError):
        Assembly("bad", "origin", (Member("arm", SO101_URDF),), (Frame("cam", "nowhere"),)).urdf()
    with pytest.raises(CompositionError):
        Assembly("bad", "origin", (Member("arm", SO101_URDF), Member("arm", SO101_URDF)))
    with pytest.raises(CompositionError):
        Assembly("bad", "origin", ())
    with pytest.raises(CompositionError):
        Pose((float("nan"), 0.0, 0.0))


def test_a_member_mounts_on_a_link_of_an_earlier_member() -> None:
    """An SO-101 carried on another robot's wrist: no new code, only a declaration."""
    carrier = embodiments["bimanual-b601-dm"]
    body = compose_embodiments(
        "carrier-with-arm",
        {
            "body": PlacedEmbodiment(carrier),
            "head": PlacedEmbodiment(
                embodiments["so101"], (0.0, 0.0, 0.05), parent="body/left_link6"
            ),
        },
        label="A pair carrying an SO-101",
    )
    assert body.state.width == carrier.state.width + 6
    assert body.joint_names[-6:] == tuple("head/" + n for n in embodiments["so101"].joint_names)
    root = ET.fromstring(body.urdf_bytes)
    (mount,) = (j for j in root.findall("joint") if j.attrib["name"] == "body/left_link6_to_head")
    parent, child = mount.find("parent"), mount.find("child")
    assert parent is not None and parent.attrib["link"] == "body/left_link6"
    assert child is not None and child.attrib["link"] == "head/base_link"
    # The graph says what the description says: the carried arm rides on the arm moving link6.
    mounts = {c.instance: c.mount for c in body.components}
    assert mounts["head.arm"] == MountedOn("body.left_arm", "head/base_link")
    assert mounts["head.jaw"] == MountedOn("head.arm", "head/gripper_link")
    with pytest.raises(CompositionError):
        compose_embodiments(
            "backwards",
            {
                "head": PlacedEmbodiment(embodiments["so101"], parent="body/left_link6"),
                "body": PlacedEmbodiment(carrier),
            },
            label="Mounted before its parent exists",
        )


def test_a_member_on_a_fixed_root_link_stays_root_mounted() -> None:
    pair = compose_embodiments("pair", {"a": PlacedEmbodiment(embodiments["so101"])}, label="Pair")
    body = compose_embodiments(
        "nested",
        {
            "pair": PlacedEmbodiment(pair),
            "extra": PlacedEmbodiment(embodiments["so101"], parent="pair/assembly"),
        },
        label="Nested with an extra arm on the inner root",
    )
    mounts = {c.instance: c.mount for c in body.components}
    assert mounts["extra.arm"] == RootMount("extra/base_link")


def test_colliding_names_are_refused_not_minted() -> None:
    with pytest.raises(CompositionError, match="link names collide"):
        Assembly(
            "bad", "origin", (Member("left", SO101_URDF),), (Frame("left_base_link", "origin"),)
        ).urdf()
    with pytest.raises(CompositionError, match="collide"):
        Assembly("bad", "left_base_link", (Member("left", SO101_URDF),)).urdf()
    # Through composition: the inner body already has a joint "b/assembly_to_left".
    inner = compose_embodiments(
        "inner", {"left": PlacedEmbodiment(embodiments["so101"])}, label="Inner"
    )
    with pytest.raises(CompositionError, match="joint names collide"):
        compose_embodiments(
            "outer",
            {
                "b": PlacedEmbodiment(inner),
                "left": PlacedEmbodiment(embodiments["so101"], parent="b/assembly"),
            },
            label="Outer",
        )


def test_a_composed_description_is_pinned() -> None:
    """A change to the renderer re-mints every composed id; it must be deliberate."""
    body = compose_embodiments(
        "pair",
        {
            "a": PlacedEmbodiment(embodiments["so101"]),
            "b": PlacedEmbodiment(embodiments["so101"], (0.5, 0.0, 0.0)),
        },
        label="Pair",
    )
    assert hashlib.sha256(body.urdf_bytes).hexdigest() == (
        "ac9d5a59a446970b2af178270846f6db96f0077b9f21b0a462354e01f370320e"
    )
