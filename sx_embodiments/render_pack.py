"""The render pack: one artifact every viewer draws a registered body from.

A pack is a directory of three files, generated from the body's composed description:

* ``robot.urdf``, the description's exact bytes, so a viewer runs forward kinematics on the
  same tree the meshes were cut from (``sx_embodiment_drivers::kinematics::Tree`` on a headset).
* ``pack.glb``, one glTF 2.0 scene. Its one root node, ``urdf_root``, turns URDF Z-up into
  glTF Y-up (-90 deg about x). Its children are one node per URDF link, named exactly the link,
  in description order, each at identity. A link node's mesh is that link's visual geometry with
  every visual origin baked in, in the link frame, in metres. A viewer places a link by setting
  its node's local transform to the link's pose in the root link's frame, in URDF coordinates.
  Primitives carry positions and flat normals (not indexed), one primitive per material; a
  material is the visual's URDF colour, or one neutral grey. Box, cylinder and sphere visuals
  are tessellated. Meshes are decimated so the pack fits a headset's triangle budget.
* ``pack.json``, ``sx.render-pack/v1``: the body, both files' digests, every link's triangle
  count, and the body's gripper tool frames by side; and, when the body declares them
  (:mod:`sx_embodiments.viewing`), ``tracking`` (each XR controller's grip frame on a link, by
  side), ``mount`` (the base mount's frame and kind, as ``collision.json``'s ``support``, and
  that surface's height above the floor) and ``virtual_gripper`` (the body's gripper drawn at an
  empty controller).
* ``collision.json``, when a link has collision geometry: capsules containing it and the link
  pairs a self-collision check skips (:mod:`sx_embodiments.collision_proxies`), which the
  ``sx-embodiment-ik`` solver keeps apart.

Decimation is vertex clustering on a per-mesh grid, its cell sized by bisection to the mesh's
share of the budget (shares are proportional to source triangles). Each cluster keeps the
source vertex nearest its centroid, so every output position is a source position moved only
by the visual origin, and the bytes are reproducible: no BLAS, no randomness, sorted keys.

NumPy is this module's only dependency beyond the registry (the ``render`` extra).
"""

from __future__ import annotations

import hashlib
import math
import struct
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Final

import numpy as np
import numpy.typing as npt
from sx_contracts.identity import JsonValue, canonical_bytes

from .assets import PACKAGE_URI_PREFIX, description_asset_uri
from .embodiment import Embodiment
from .errors import EmbodimentError
from .kinematics import Transform, from_xyz_rpy
from .materialize import materialize
from .viewing import Viewing

SCHEMA: Final = "sx.render-pack/v1"
GENERATOR: Final = "sx_embodiments.render_pack/v1"
DEFAULT_TRIANGLE_BUDGET: Final = 150_000
URDF_FILE: Final = "robot.urdf"
GLB_FILE: Final = "pack.glb"
JSON_FILE: Final = "pack.json"
COLLISION_FILE: Final = "collision.json"

#: URDF Z-up to glTF Y-up: -90 deg about x, as glTF's (x, y, z, w).
URDF_TO_GLTF: Final = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
NEUTRAL_RGBA: Final = (0.7, 0.7, 0.7, 1.0)
_SIDES: Final = ("left", "right")
_TCP_SUFFIXES: Final = ("gripper_tcp", "gripper_tip")
_CYLINDER_SEGMENTS: Final = 32
_SPHERE_RINGS: Final = 12
_SPHERE_SEGMENTS: Final = 24
_SMALLEST_SHARE: Final = 12

Floats = npt.NDArray[np.float64]
Faces = npt.NDArray[np.int64]


class RenderPackError(EmbodimentError):
    """A description cannot be drawn as asked."""


@dataclass(frozen=True, slots=True)
class _Mesh:
    """An indexed triangle mesh: unique vertices and faces into them."""

    vertices: Floats  # (n, 3)
    faces: Faces  # (m, 3)


@dataclass(frozen=True, slots=True)
class _Visual:
    link: str
    origin: Transform
    source: _Mesh
    key: str  # what decimates identically: the mesh file and scale, or the primitive
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class RenderPack:
    """A generated pack, in memory: write it with :meth:`write`."""

    embodiment_id: str
    urdf: bytes
    glb: bytes
    document: bytes  # pack.json
    collision: bytes | None = None  # collision.json

    @property
    def files(self) -> dict[str, bytes]:
        files = {URDF_FILE: self.urdf, GLB_FILE: self.glb, JSON_FILE: self.document}
        if self.collision is not None:
            files[COLLISION_FILE] = self.collision
        return files

    def write(self, root: Path) -> Path:
        """Write the pack under ``root/<embodiment_id>/``; returns that directory."""
        directory = root / self.embodiment_id
        directory.mkdir(parents=True, exist_ok=True)
        for name, data in self.files.items():
            (directory / name).write_bytes(data)
        return directory


# --- reading the description -------------------------------------------------------------


def _floats(text: str | None, count: int, default: str, what: str) -> tuple[float, ...]:
    raw = default if text is None else text
    try:
        values = tuple(float(value) for value in raw.split())
    except ValueError as error:
        raise RenderPackError(f"{what}={raw!r} is not {count} numbers") from error
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise RenderPackError(f"{what}={raw!r} is not {count} finite numbers")
    return values


def _rgba(
    element: ET.Element | None, named: Mapping[str, tuple[float, float, float, float]]
) -> tuple[float, float, float, float]:
    if element is None:
        return NEUTRAL_RGBA
    color = element.find("color")
    if color is not None and color.get("rgba") is not None:
        r, g, b, a = _floats(color.get("rgba"), 4, "", "rgba")
        return (r, g, b, a)
    return named.get(element.get("name", ""), NEUTRAL_RGBA)


def _read_stl(data: bytes, what: str) -> Floats:
    """An STL's triangle corners, (3m, 3)."""
    if len(data) >= 84:
        (count,) = struct.unpack_from("<I", data, 80)
        if len(data) == 84 + 50 * count:
            records = np.frombuffer(data, dtype=np.uint8, offset=84).reshape(count, 50)
            corners = np.ascontiguousarray(records[:, 12:48]).view("<f4")
            return corners.reshape(-1, 3).astype(np.float64)
    corners = [
        [float(value) for value in line.split()[1:4]]
        for line in data.decode("ascii", "replace").splitlines()
        if line.strip().startswith("vertex")
    ]
    if not corners or len(corners) % 3:
        raise RenderPackError(f"{what} is not an STL")
    return np.asarray(corners, dtype=np.float64)


def _read_obj(data: bytes, what: str) -> Floats:
    """An OBJ's triangle corners (polygons fanned), (3m, 3)."""
    vertices: list[list[float]] = []
    corners: list[list[float]] = []
    for line in data.decode("utf-8", "replace").splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "v":
            vertices.append([float(value) for value in parts[1:4]])
        elif parts[0] == "f":
            indices = [int(part.split("/")[0]) for part in parts[1:]]
            resolved = [index - 1 if index > 0 else len(vertices) + index for index in indices]
            for k in range(1, len(resolved) - 1):
                corners += [
                    vertices[resolved[0]],
                    vertices[resolved[k]],
                    vertices[resolved[k + 1]],
                ]
    if not corners:
        raise RenderPackError(f"{what} has no faces")
    return np.asarray(corners, dtype=np.float64)


def _weld(corners: Floats) -> _Mesh:
    """A triangle soup as an indexed mesh, joining corners with equal coordinates."""
    vertices, inverse = np.unique(corners, axis=0, return_inverse=True)
    return _clean(vertices, inverse.reshape(-1, 3).astype(np.int64))


def _clean(vertices: Floats, faces: Faces) -> _Mesh:
    """Drop degenerate and repeated faces (first winding kept) and unused vertices."""
    keep = (
        (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    )
    faces = faces[keep]
    if len(faces):
        _, first = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
        faces = faces[np.sort(first)]
    used, remapped = np.unique(faces, return_inverse=True)
    return _Mesh(vertices[used], remapped.reshape(-1, 3).astype(np.int64))


def _box(size: tuple[float, ...]) -> _Mesh:
    hx, hy, hz = (value / 2 for value in size)
    vertices = np.array(
        [[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)],
        dtype=np.float64,
    )
    faces = np.array(
        [
            [0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5],  # -x, +x
            [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6],  # -y, +y
            [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3],  # -z, +z
        ],
        dtype=np.int64,
    )  # fmt: skip
    return _Mesh(vertices, faces)


def _cylinder(radius: float, length: float) -> _Mesh:
    n = _CYLINDER_SEGMENTS
    ring = [
        (radius * math.cos(2 * math.pi * k / n), radius * math.sin(2 * math.pi * k / n))
        for k in range(n)
    ]
    vertices = [[x, y, -length / 2] for x, y in ring] + [[x, y, length / 2] for x, y in ring]
    vertices += [[0.0, 0.0, -length / 2], [0.0, 0.0, length / 2]]
    faces: list[list[int]] = []
    for k in range(n):
        j = (k + 1) % n
        faces += [[k, j, n + j], [k, n + j, n + k], [2 * n, j, k], [2 * n + 1, n + k, n + j]]
    return _Mesh(np.asarray(vertices, dtype=np.float64), np.asarray(faces, dtype=np.int64))


def _sphere(radius: float) -> _Mesh:
    rings, segments = _SPHERE_RINGS, _SPHERE_SEGMENTS
    vertices = [[0.0, 0.0, radius]]
    for i in range(1, rings):
        theta = math.pi * i / rings
        for k in range(segments):
            phi = 2 * math.pi * k / segments
            vertices.append(
                [
                    radius * math.sin(theta) * math.cos(phi),
                    radius * math.sin(theta) * math.sin(phi),
                    radius * math.cos(theta),
                ]
            )
    vertices.append([0.0, 0.0, -radius])
    bottom = len(vertices) - 1

    def at(i: int, k: int) -> int:
        return 1 + (i - 1) * segments + k % segments

    faces: list[list[int]] = []
    for k in range(segments):
        faces.append([0, at(1, k), at(1, k + 1)])
        faces.append([bottom, at(rings - 1, k + 1), at(rings - 1, k)])
        for i in range(1, rings - 1):
            faces += [
                [at(i, k), at(i + 1, k), at(i + 1, k + 1)],
                [at(i, k), at(i + 1, k + 1), at(i, k + 1)],
            ]
    return _Mesh(np.asarray(vertices, dtype=np.float64), np.asarray(faces, dtype=np.int64))


def _visuals(embodiment: Embodiment, urdf: bytes) -> tuple[list[str], list[_Visual]]:
    root = ET.fromstring(urdf)  # nosec B314 - registry-owned description bytes
    directory = materialize(embodiment)
    description = embodiment.urdf.asset.uri
    named = {
        name: _rgba(material, {})
        for material in root.findall("material")
        if (name := material.get("name")) is not None
    }
    links: list[str] = []
    visuals: list[_Visual] = []
    meshes: dict[tuple[str, tuple[float, ...]], _Mesh] = {}
    for link in root.findall("link"):
        name = link.get("name")
        if name is None:
            raise RenderPackError("a link has no name")
        links.append(name)
        for visual in link.findall("visual"):
            origin = visual.find("origin")
            transform = from_xyz_rpy(
                _xyz(_floats(None if origin is None else origin.get("xyz"), 3, "0 0 0", "xyz")),
                _xyz(_floats(None if origin is None else origin.get("rpy"), 3, "0 0 0", "rpy")),
            )
            geometry = visual.find("geometry")
            shape = geometry[0] if geometry is not None and len(geometry) else None
            if shape is None:
                raise RenderPackError(f"link {name!r} has a visual with no geometry")
            rgba = _rgba(visual.find("material"), named)
            match shape.tag:
                case "mesh":
                    filename = shape.get("filename", "")
                    scale = _floats(shape.get("scale"), 3, "1 1 1", "scale")
                    uri = description_asset_uri(description, filename)
                    if not uri.startswith(PACKAGE_URI_PREFIX):
                        raise RenderPackError(f"link {name!r}: {filename!r} is not packaged")
                    relpath = uri.removeprefix(PACKAGE_URI_PREFIX)
                    cached = meshes.get((relpath, scale))
                    if cached is None:
                        cached = _load(directory / relpath, relpath, scale)
                        meshes[(relpath, scale)] = cached
                    visuals.append(_Visual(name, transform, cached, f"{relpath}@{scale}", rgba))
                case "box":
                    size = _floats(shape.get("size"), 3, "", "box size")
                    visuals.append(_Visual(name, transform, _box(size), f"box{size}", rgba))
                case "cylinder":
                    radius = _floats(shape.get("radius"), 1, "", "radius")[0]
                    length = _floats(shape.get("length"), 1, "", "length")[0]
                    mesh = _cylinder(radius, length)
                    visuals.append(_Visual(name, transform, mesh, f"cyl{radius},{length}", rgba))
                case "sphere":
                    radius = _floats(shape.get("radius"), 1, "", "radius")[0]
                    visuals.append(_Visual(name, transform, _sphere(radius), f"sph{radius}", rgba))
                case other:
                    raise RenderPackError(f"link {name!r} has a {other!r} visual")
    return links, visuals


def _xyz(values: tuple[float, ...]) -> tuple[float, float, float]:
    return (values[0], values[1], values[2])


def _load(path: Path, relpath: str, scale: tuple[float, ...]) -> _Mesh:
    data = path.read_bytes()
    match PurePosixPath(relpath).suffix.lower():
        case ".stl":
            corners = _read_stl(data, relpath)
        case ".obj":
            corners = _read_obj(data, relpath)
        case suffix:
            raise RenderPackError(f"{relpath}: {suffix} meshes are not drawn yet")
    return _weld(corners * np.asarray(scale, dtype=np.float64))


# --- decimation ---------------------------------------------------------------------------


def _cluster(mesh: _Mesh, cell: float) -> _Mesh:
    """Vertex clustering at ``cell`` metres: each cluster keeps its vertex nearest the centroid."""
    v = mesh.vertices
    grid = np.floor((v - v.min(axis=0)) / cell).astype(np.int64)
    span = grid.max(axis=0) + 1
    keys = (grid[:, 0] * span[1] + grid[:, 1]) * span[2] + grid[:, 2]
    _, cluster = np.unique(keys, return_inverse=True)
    count = np.bincount(cluster).astype(np.float64)
    centroid = np.stack(
        [np.bincount(cluster, weights=v[:, axis]) / count for axis in range(3)], axis=1
    )
    distance = np.sum((v - centroid[cluster]) ** 2, axis=1)
    # Per cluster the nearest vertex, ties to the lowest index: sort by (cluster, distance, index).
    order = np.lexsort((np.arange(len(v)), distance, cluster))
    first = np.ones(len(order), dtype=bool)
    first[1:] = cluster[order][1:] != cluster[order][:-1]
    representative = order[first]
    return _clean(v[representative], cluster[mesh.faces].astype(np.int64))


def _decimate(mesh: _Mesh, target: int) -> _Mesh:
    """The finest clustering with at most ``target`` faces (the mesh itself when it fits)."""
    if len(mesh.faces) <= target:
        return mesh
    extent = float(np.max(mesh.vertices.max(axis=0) - mesh.vertices.min(axis=0)))
    if extent <= 0.0:
        return _Mesh(mesh.vertices[:0], mesh.faces[:0])
    low, high = extent * 1e-5, extent  # high: one cell, nothing survives
    best = _Mesh(mesh.vertices[:0], mesh.faces[:0])
    for _ in range(28):
        middle = math.sqrt(low * high)
        candidate = _cluster(mesh, middle)
        if len(candidate.faces) <= target:
            high = middle
            if len(candidate.faces) > len(best.faces):
                best = candidate
        else:
            low = middle
    return best


# --- the glTF ------------------------------------------------------------------------------


def _flat(mesh: _Mesh, origin: Transform) -> tuple[Floats, Floats]:
    """Positions with the visual origin baked in, and each corner's face normal, (3m, 3) each."""
    r = np.asarray(origin, dtype=np.float64).reshape(4, 4)
    v = mesh.vertices
    # Elementwise, not a BLAS product, so the bytes do not depend on the machine.
    moved = np.stack(
        [
            r[row, 0] * v[:, 0] + r[row, 1] * v[:, 1] + r[row, 2] * v[:, 2] + r[row, 3]
            for row in range(3)
        ],
        axis=1,
    )
    corners = moved[mesh.faces].astype(np.float32).astype(np.float64)  # (m, 3, 3)
    a = corners[:, 1] - corners[:, 0]
    b = corners[:, 2] - corners[:, 0]
    normal = np.stack(
        [
            a[:, 1] * b[:, 2] - a[:, 2] * b[:, 1],
            a[:, 2] * b[:, 0] - a[:, 0] * b[:, 2],
            a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0],
        ],
        axis=1,
    )
    length = np.sqrt(np.sum(normal * normal, axis=1))
    normal = np.where(
        length[:, None] > 0, normal / np.where(length > 0, length, 1.0)[:, None], [0.0, 0.0, 1.0]
    )
    return corners.reshape(-1, 3), np.repeat(normal, 3, axis=0)


def _glb(
    links: list[str], parts: Mapping[str, list[tuple[tuple[float, ...], Floats, Floats]]]
) -> bytes:
    """One scene: ``urdf_root`` over one identity node per link, meshes grouped by material."""
    buffer = bytearray()
    views: list[JsonValue] = []
    accessors: list[JsonValue] = []
    materials: list[tuple[float, ...]] = []
    meshes: list[JsonValue] = []
    nodes: list[JsonValue] = [
        {
            "name": "urdf_root",
            "rotation": list(URDF_TO_GLTF),
            "children": list(range(1, len(links) + 1)),
        }
    ]

    def accessor(values: Floats, with_bounds: bool) -> int:
        data = values.astype("<f4")
        views.append(
            {"buffer": 0, "byteOffset": len(buffer), "byteLength": data.nbytes, "target": 34962}
        )
        buffer.extend(data.tobytes())
        entry: dict[str, JsonValue] = {
            "bufferView": len(views) - 1,
            "componentType": 5126,
            "count": len(data),
            "type": "VEC3",
        }
        if with_bounds:
            entry["min"] = [float(x) for x in data.min(axis=0)]
            entry["max"] = [float(x) for x in data.max(axis=0)]
        accessors.append(entry)
        return len(accessors) - 1

    for link in links:
        node: dict[str, JsonValue] = {"name": link}
        grouped: dict[tuple[float, ...], tuple[list[Floats], list[Floats]]] = {}
        for rgba, positions, normals in parts.get(link, []):
            if len(positions):
                bucket = grouped.setdefault(rgba, ([], []))
                bucket[0].append(positions)
                bucket[1].append(normals)
        if grouped:
            primitives: list[JsonValue] = []
            for rgba, (positions, normals) in grouped.items():
                if rgba not in materials:
                    materials.append(rgba)
                primitives.append(
                    {
                        "attributes": {
                            "POSITION": accessor(np.concatenate(positions), True),
                            "NORMAL": accessor(np.concatenate(normals), False),
                        },
                        "material": materials.index(rgba),
                        "mode": 4,
                    }
                )
            meshes.append({"name": link, "primitives": primitives})
            node["mesh"] = len(meshes) - 1
        nodes.append(node)
    document: dict[str, JsonValue] = {
        "asset": {"version": "2.0", "generator": GENERATOR},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": nodes,
        "meshes": meshes,
        "materials": [
            {
                "name": f"rgba({','.join(repr(c) for c in rgba)})",
                "doubleSided": True,
                "pbrMetallicRoughness": {
                    "baseColorFactor": list(rgba),
                    "metallicFactor": 0.0,
                    "roughnessFactor": 0.8,
                },
                **({"alphaMode": "BLEND"} if rgba[3] < 1.0 else {}),
            }
            for rgba in materials
        ],
        "accessors": accessors,
        "bufferViews": views,
        "buffers": [{"byteLength": len(buffer)}],
    }
    text = canonical_bytes(document)
    text += b" " * (-len(text) % 4)
    buffer.extend(b"\0" * (-len(buffer) % 4))
    total = 12 + 8 + len(text) + 8 + len(buffer)
    return (
        struct.pack("<III", 0x46546C67, 2, total)
        + struct.pack("<II", len(text), 0x4E4F534A)
        + text
        + struct.pack("<II", len(buffer), 0x004E4942)
        + bytes(buffer)
    )


# --- the pack -------------------------------------------------------------------------------


def tcp_frames(links: list[str]) -> dict[str, str]:
    """The body's gripper tool frames by side: ``<side>_gripper_tcp``, else ``_gripper_tip``."""
    frames: dict[str, str] = {}
    for side in _SIDES:
        for suffix in _TCP_SUFFIXES:
            if f"{side}_{suffix}" in links:
                frames[side] = f"{side}_{suffix}"
                break
    return frames


def _rigid_with(urdf: bytes, tcp: str) -> set[str]:
    """Every link that moves with ``tcp`` but for its own joints: up from ``tcp`` through fixed
    joints to the last actuated joint's child, and that link's whole subtree (the jaw's fingers)."""
    root = ET.fromstring(urdf)  # nosec B314 - registry-owned description bytes
    parent: dict[str, tuple[str, str]] = {}
    children: dict[str, list[str]] = {}
    for joint in root.findall("joint"):
        p, c = joint.find("parent"), joint.find("child")
        if p is None or c is None:
            raise RenderPackError(f"joint {joint.get('name')!r} lacks a parent or child")
        parent[c.get("link", "")] = (p.get("link", ""), joint.get("type", ""))
        children.setdefault(p.get("link", ""), []).append(c.get("link", ""))
    top = tcp
    while top in parent and parent[top][1] == "fixed":
        top = parent[top][0]
    drawn: set[str] = set()
    stack = [top]
    while stack:
        link = stack.pop()
        drawn.add(link)
        stack.extend(children.get(link, []))
    return drawn


def _viewing(viewing: Viewing, links: list[str], embodiment: Embodiment) -> dict[str, JsonValue]:
    frames = [device.frame for device in viewing.tracking]
    if missing := sorted(set(frames) - set(links)):
        raise RenderPackError(f"{embodiment.name}: viewing names links it lacks: {missing}")
    fields: dict[str, JsonValue] = {}
    if viewing.tracking:
        fields["tracking"] = {device.side: device.to_json() for device in viewing.tracking}
    if viewing.mount is not None:
        base = embodiment.base_mount
        if base is None:
            raise RenderPackError(f"{embodiment.name}: a workcell needs a base mount")
        fields["mount"] = {
            "frame": base.frame,
            "kind": str(base.kind),
            "floor_to_frame_m": viewing.mount.floor_to_frame_m,
        }
    if viewing.virtual_gripper is not None:
        frames = tcp_frames(links)
        if set(frames) != {"left", "right"}:
            raise RenderPackError(f"{embodiment.name}: a virtual gripper needs both tool frames")
        drawn = {side: _rigid_with(embodiment.urdf_bytes, frame) for side, frame in frames.items()}
        fields["virtual_gripper"] = viewing.virtual_gripper.to_json(
            {side: [link for link in links if link in drawn[side]] for side in frames}
        )
    return fields


def build(
    embodiment: Embodiment,
    *,
    name: str | None = None,
    triangle_budget: int = DEFAULT_TRIANGLE_BUDGET,
    viewing: Viewing | None = None,
) -> RenderPack:
    """``embodiment``'s render pack, decimated to at most ``triangle_budget`` triangles.

    ``name`` is the registry key the pack is filed under (the body's name by default);
    ``viewing`` the body's declared viewing facts, written into ``pack.json``.
    """
    urdf = embodiment.urdf_bytes
    links, visuals = _visuals(embodiment, urdf)
    source_total = sum(len(visual.source.faces) for visual in visuals)
    if source_total == 0:
        raise RenderPackError(f"{embodiment.name} has no visual geometry")
    decimated: dict[tuple[str, int], _Mesh] = {}
    parts: dict[str, list[tuple[tuple[float, ...], Floats, Floats]]] = {}
    for visual in visuals:
        faces = len(visual.source.faces)
        share = max(_SMALLEST_SHARE, triangle_budget * faces // source_total)
        key = (visual.key, share)
        if key not in decimated:
            decimated[key] = _decimate(visual.source, share)
        positions, normals = _flat(decimated[key], visual.origin)
        parts.setdefault(visual.link, []).append((visual.rgba, positions, normals))
    glb = _glb(links, parts)
    triangles = {link: sum(len(p) // 3 for _, p, _ in parts.get(link, [])) for link in links}
    if sum(triangles.values()) > triangle_budget:
        raise RenderPackError(
            f"{embodiment.name}: {sum(triangles.values())} triangles exceed {triangle_budget}"
        )
    key = name or str(embodiment.name)
    document: dict[str, JsonValue] = {
        "schema": SCHEMA,
        "embodiment_id": key,
        "embodiment_digest": str(embodiment.id),
        "urdf_sha256": hashlib.sha256(urdf).hexdigest(),
        "glb_sha256": hashlib.sha256(glb).hexdigest(),
        "links": [{"name": link, "triangles": triangles[link]} for link in links],
        "triangles": sum(triangles.values()),
        "tcp_frames": tcp_frames(links),
        "generated_by": GENERATOR,
    }
    if viewing is not None:
        document.update(_viewing(viewing, links, embodiment))
    from .collision_proxies import proxies

    return RenderPack(key, urdf, glb, canonical_bytes(document), proxies(embodiment, name=key))
