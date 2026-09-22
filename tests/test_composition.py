from xml.etree import ElementTree as ET

import pytest

from sx_embodiments import (
    Embodiment,
    EmbodimentKind,
    PlacedEmbodiment,
    compose_embodiments,
    embodiments,
)
from sx_embodiments.errors import CompositionError


@pytest.mark.parametrize("count", (1, 2, 5))
def test_arbitrary_robot_count_preserves_coordinates_and_graph(count, monkeypatch, tmp_path):
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_CACHE", str(tmp_path))
    body = compose_embodiments(
        "assembly",
        {
            f"robot{i}": PlacedEmbodiment(embodiments["so101"], (float(i), 0.0, 0.0))
            for i in range(count)
        },
        label="Assembly",
    )
    assert body.state.width == 6 * count
    assert len(set(body.joint_names)) == 6 * count
    assert Embodiment.from_json(body.to_json()) == body
    assert Embodiment.from_json(body.to_json()).urdf_bytes == body.urdf_bytes
    root = ET.fromstring(body.urdf_bytes)
    assert {j.attrib["name"] for j in root.findall("joint") if j.attrib["type"] != "fixed"} == set(
        body.joint_names
    )
    assert len(body.capabilities.components) == 2 * count


def test_nested_composition_and_repeated_parts(monkeypatch, tmp_path):
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_CACHE", str(tmp_path))
    pair = compose_embodiments(
        "pair",
        {
            "a": PlacedEmbodiment(embodiments["so101"]),
            "b": PlacedEmbodiment(embodiments["so101"], (0.5, 0.0, 0.0)),
        },
        label="Pair",
    )
    nested = compose_embodiments("nested", {"pair": PlacedEmbodiment(pair)}, label="Nested")
    assert nested.state.width == 12
    assert nested.joint_names == tuple("pair/" + name for name in pair.joint_names)
    assert len({c.instance for c in nested.components}) == 4


def test_existing_registry_joint_bindings_are_unique():
    for body in embodiments.values():
        assert len(set(body.joint_names)) == body.state.width, body.name


def test_cameras_keep_their_own_intrinsics_and_named_streams(monkeypatch, tmp_path):
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_CACHE", str(tmp_path))
    source = embodiments["quest-ego"]
    body = compose_embodiments(
        "robot-camera",
        {
            "robot": PlacedEmbodiment(embodiments["so101"]),
            "vision": PlacedEmbodiment(source),
        },
        label="Robot and cameras",
    )
    assert len(body.cameras) == len(source.cameras)
    for camera, original in zip(body.cameras, source.cameras, strict=True):
        assert camera.camera.optics == original.camera.optics
        assert camera.mount.frame == "vision/" + original.mount.frame
        assert camera.name == "vision." + original.name


def test_empty_invalid_and_nonfinite_compositions_refuse():
    with pytest.raises(CompositionError):
        compose_embodiments("empty", {}, label="Empty")
    with pytest.raises(CompositionError):
        compose_embodiments(
            "invalid", {"../robot": PlacedEmbodiment(embodiments["so101"])}, label="Invalid"
        )
    with pytest.raises(CompositionError):
        PlacedEmbodiment(embodiments["so101"], (float("nan"), 0.0, 0.0))


def test_capture_rig_kind_remains_explicit(monkeypatch, tmp_path):
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_CACHE", str(tmp_path))
    body = compose_embodiments(
        "capture",
        {"rig": PlacedEmbodiment(embodiments["quest-ego"])},
        label="Capture",
        kind=EmbodimentKind.CAPTURE_RIG,
    )
    assert body.kind is EmbodimentKind.CAPTURE_RIG
    assert body.operator_mounts[0].root_frame.startswith("rig/")


def test_part_identity_is_preserved_when_instances_are_composed(monkeypatch, tmp_path):
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_CACHE", str(tmp_path))
    original = embodiments["bimanual-b601-dm"]
    body = compose_embodiments("mounted", {"robot": PlacedEmbodiment(original)}, label="Mounted")
    assert tuple(c.part for c in body.components) == tuple(c.part for c in original.components)
    assert body.joint_names == tuple("robot/" + name for name in original.joint_names)


def test_assembly_grammar_accepts_calibrated_camera_parts():
    from sx_embodiments import assemble, composable_parts

    body = embodiments["quest-ego"]
    document = body.to_dict()
    definition = {
        key: document[key]
        for key in ("name", "label", "kind", "lineage", "assets", "rates", "base_mount")
    }
    definition["attachments"] = [
        {
            "instance": component["instance"],
            "part_id": component["attachment"]["part"]["part_id"],
            "mount": component["mount"],
        }
        for component in document["components"]
    ]
    result = assemble(definition, urdf=body.urdf_bytes)
    assert result.cameras == body.cameras
    assert all(camera.camera.part_id in composable_parts() for camera in result.cameras)
