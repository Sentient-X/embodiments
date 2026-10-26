"""The committed render packs are what the registry renders, and fit a headset."""

import hashlib
import importlib.util
import json
import struct
from pathlib import Path

import pytest

pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools/render_packs.py"


def _tool():
    spec = importlib.util.spec_from_file_location("render_packs", TOOL)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _gltf(glb: bytes) -> dict[str, object]:
    magic, version, total = struct.unpack_from("<III", glb, 0)
    assert (magic, version, total) == (0x46546C67, 2, len(glb))
    length, kind = struct.unpack_from("<II", glb, 12)
    assert kind == 0x4E4F534A
    return json.loads(glb[20 : 20 + length])


def test_committed_packs_and_poses_are_rendered_bytes() -> None:
    """Regenerate with `python tools/render_packs.py` when this fails."""
    stale = [
        str(path.relative_to(ROOT))
        for path, data in _tool().rendered_files().items()
        if not path.is_file() or path.read_bytes() != data
    ]
    assert not stale, f"stale render pack files: {stale}"


@pytest.mark.parametrize("name", ["sentient-static-upper-body", "yubi"])
def test_a_pack_draws_every_link_within_its_budget(name: str) -> None:
    from sx_embodiments.kinematics import Kinematics
    from sx_embodiments.render_pack import SCHEMA, URDF_TO_GLTF

    budget = _tool().BODIES[name]
    directory = ROOT / "render_packs" / name
    urdf = (directory / "robot.urdf").read_bytes()
    glb = (directory / "pack.glb").read_bytes()
    pack = json.loads((directory / "pack.json").read_bytes())
    assert pack["schema"] == SCHEMA and pack["embodiment_id"] == name
    assert pack["urdf_sha256"] == hashlib.sha256(urdf).hexdigest()
    assert pack["glb_sha256"] == hashlib.sha256(glb).hexdigest()
    assert urdf == _tool().body(name).urdf_bytes

    links = [link["name"] for link in pack["links"]]
    assert set(links) == set(Kinematics.of(urdf).links)
    total = sum(link["triangles"] for link in pack["links"])
    assert total == pack["triangles"] and 0 < total <= budget

    gltf = _gltf(glb)
    nodes = gltf["nodes"]
    assert isinstance(nodes, list)
    root, *children = nodes
    assert root["name"] == "urdf_root" and root["rotation"] == list(URDF_TO_GLTF)
    assert [node["name"] for node in children] == links
    assert all(set(node) <= {"name", "mesh"} for node in children), "link nodes at identity"
    meshes, accessors = gltf["meshes"], gltf["accessors"]
    assert isinstance(meshes, list) and isinstance(accessors, list)
    for node, link in zip(children, pack["links"], strict=True):
        primitives = meshes[node["mesh"]]["primitives"] if "mesh" in node else []
        drawn = sum(accessors[p["attributes"]["POSITION"]]["count"] // 3 for p in primitives)
        assert drawn == link["triangles"], link["name"]


def test_tool_frames_are_the_grippers() -> None:
    def frames(name: str) -> dict[str, str]:
        return json.loads((ROOT / "render_packs" / name / "pack.json").read_bytes())["tcp_frames"]

    assert frames("sentient-static-upper-body") == {
        "left": "left_gripper_tcp",
        "right": "right_gripper_tcp",
    }
    assert frames("yubi") == {"left": "left_gripper_tip", "right": "right_gripper_tip"}


@pytest.mark.parametrize("name", ["sentient-static-upper-body", "yubi"])
def test_collision_proxies_contain_every_collision_vertex(name: str) -> None:
    import numpy as np
    from sx_embodiments.collision_proxies import SCHEMA, _collision_points

    directory = ROOT / "render_packs" / name
    document = json.loads((directory / "collision.json").read_bytes())
    urdf = (directory / "robot.urdf").read_bytes()
    assert document["schema"] == SCHEMA
    assert document["urdf_sha256"] == hashlib.sha256(urdf).hexdigest()
    mount = _tool().body(name).base_mount
    assert document["support"] == (
        None if mount is None else {"frame": mount.frame, "kind": str(mount.kind)}
    )
    points = _collision_points(_tool().body(name), urdf)
    assert {c["link"] for c in document["capsules"]} == set(points)
    for link, vertices in points.items():
        gaps = []
        for capsule in (c for c in document["capsules"] if c["link"] == link):
            a, b = np.asarray(capsule["a"]), np.asarray(capsule["b"])
            d = b - a
            t = np.clip((vertices - a) @ d / max(float(d @ d), 1e-18), 0.0, 1.0)
            gaps.append(np.linalg.norm(vertices - (a + t[:, None] * d), axis=1) - capsule["radius"])
        assert float(np.min(gaps, axis=0).max()) <= 0.0, link
