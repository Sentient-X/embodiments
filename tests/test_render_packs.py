"""The committed render packs are what the registry renders, and fit a headset."""

import hashlib
import importlib.util
import json
import struct
from pathlib import Path

import pytest

from sx_embodiments import viewing_of
from sx_embodiments.rendered import same_rendering

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
        if not same_rendering(path, data)
    ]
    assert not stale, f"stale render pack files: {stale}"


@pytest.mark.parametrize("name", ["sentient-static-upper-body", "yubi"])
def test_a_pack_draws_every_link_within_its_budget(name: str) -> None:
    from sx_embodiments.kinematics import Kinematics
    from sx_embodiments.render_pack import SCHEMA, URDF_TO_GLTF

    budget = viewing_of(name).triangle_budget
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

    from sx_embodiments.collision_proxies import SCHEMA, _collision_points, support_document

    directory = ROOT / "render_packs" / name
    document = json.loads((directory / "collision.json").read_bytes())
    urdf = (directory / "robot.urdf").read_bytes()
    assert document["schema"] == SCHEMA
    assert document["urdf_sha256"] == hashlib.sha256(urdf).hexdigest()
    mount = _tool().body(name).base_mount
    assert document["support"] == (None if mount is None else support_document(mount))
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


def test_viewing_facts_are_carried_from_the_registry() -> None:
    from sx_embodiments.known.static_upper_body import STATIC_UPPER_BODY_WORKCELL
    from sx_embodiments.known.yubi import YUBI_TRACKING

    def pack(name: str) -> dict[str, object]:
        return json.loads((ROOT / "render_packs" / name / "pack.json").read_bytes())

    yubi, body = pack("yubi"), pack("sentient-static-upper-body")
    assert yubi["tracking"] == {d.side: d.to_json() for d in YUBI_TRACKING}
    assert yubi["tracking"]["left"]["frame"] == "left_controller_link"
    assert "mount" not in yubi and "tracking" not in body
    support = json.loads(
        (ROOT / "render_packs/sentient-static-upper-body/collision.json").read_bytes()
    )["support"]
    assert body["mount"] == {
        **support,
        "floor_to_frame_m": STATIC_UPPER_BODY_WORKCELL.floor_to_frame_m,
    }
    gripper = body["virtual_gripper"]
    assert gripper["ahead_m"] == 0.09
    drawn = gripper["links"]["left"] + gripper["links"]["right"]
    assert set(body["tcp_frames"].values()) <= set(drawn)


def test_a_json_rendering_is_current_through_another_platforms_last_bits(tmp_path: Path) -> None:
    """macOS and Linux disagree in a float's last bits; that is the same rendering, a real
    change is not, and anything that is not JSON is compared byte for byte."""
    committed = tmp_path / "poses.json"

    def document(pose: list[float], name: str = "a") -> bytes:
        return json.dumps({"pose": pose, "name": name}).encode()

    committed.write_bytes(document([0.575051208457572, 1.0, 0.0]))
    assert same_rendering(committed, document([0.5750512084575721, 1.0, 0.0]))
    assert not same_rendering(committed, document([0.57505121, 1.0, 0.0]))
    assert not same_rendering(committed, document([0.575051208457572, 1.0]))
    assert not same_rendering(committed, document([0.575051208457572, 1.0, 0.0], "b"))
    urdf = tmp_path / "body.urdf"
    urdf.write_bytes(b'<origin xyz="0.1"/>')
    assert not same_rendering(urdf, b'<origin xyz="0.1000000000000001"/>')
    assert not same_rendering(tmp_path / "missing.json", b"{}")


def test_pieces_follow_a_concave_part_and_cover_its_surface() -> None:
    """An L of two 4 cm by 1 cm bars: its pieces keep the inside corner empty (a hull of the
    whole would fill it), and every vertex lies in a piece."""
    import numpy as np
    from scipy.spatial import Delaunay, QhullError

    from sx_embodiments.collision_proxies import pieces

    def bar(lo: tuple[float, float, float], hi: tuple[float, float, float]) -> np.ndarray:
        corners = np.array(
            [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        )
        faces = [
            (0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
            (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3),
        ]  # fmt: skip
        return corners[np.array(faces)]

    shape = np.vstack([bar((0, 0, 0), (0.04, 0.01, 0.01)), bar((0, 0, 0), (0.01, 0.04, 0.01))])
    found = pieces(shape)
    corner = np.array([0.03, 0.03, 0.005])  # inside the L's bend, outside the part

    def inside(piece: np.ndarray) -> bool:
        try:
            return bool(Delaunay(piece).find_simplex(corner) >= 0)
        except QhullError:  # a flat piece (one face) holds no volume
            return False

    assert not any(inside(piece) for piece in found)

    def covers(piece: np.ndarray, point: np.ndarray) -> bool:
        """Whether `point` is a convex combination of the piece's corners (to a micrometre)."""
        from scipy.optimize import linprog

        n = len(piece)
        equalities = np.vstack([piece.T, np.ones(n)])
        target = np.append(point, 1.0)
        found = linprog(np.zeros(n), A_eq=equalities, b_eq=target, bounds=(0, None))
        return bool(found.status == 0)

    for vertex in shape.reshape(-1, 3):
        assert any(covers(piece, vertex) for piece in found), vertex
