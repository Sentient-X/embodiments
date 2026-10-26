"""Collision proxies: capsules fitted to a body's collision geometry, for a solver that must
keep links apart many times a second.

``collision.json`` (``sx.collision-proxies/v2``) sits in a body's render pack beside
``robot.urdf``. For every link with ``<collision>`` geometry it holds one to three capsules in
the link frame, each a segment ``a``-``b`` and a radius, that together **contain** every vertex
of the link's collision meshes (and so their hulls): a distance between proxies never
overstates the distance between the bodies. ``ignored_pairs`` lists the link pairs a
self-collision check skips, each for its reason:

* ``adjacent``: at most two moving joints apart along the tree. Links that close meet at a
  joint whose own design keeps them apart, and their fitted proxies overlap there.
* ``touching_at_zero``: proxies that overlap with every joint at zero, the description's
  as-built pose; a constraint there could never be satisfied.

``support`` is the surface the body stands on, from its declared
:class:`~sx_embodiments.compose.BaseMount`: the top is the ``z = 0`` plane of the mount's
``frame``, and the support is solid below it everywhere within reach. A body clamped at a table
edge has the table under and in front of where its arms hang, so a link below the top is in
collision unless the embodiment declares otherwise. ``null`` when the body declares no mount.

Fitting is deterministic: a capsule's axis is the points' principal direction (eigenvector
sign fixed), its line through the centre of the points' extent across that axis, its radius
the farthest point from the line, its ends as short as covering every point allows. A link is
split along its second principal direction into two or three slabs, each with its own capsule,
when that shrinks the total volume by more than :data:`_SPLIT_GAIN`. Numbers are rounded to
micrometres with radii rounded up, and the document is canonical JSON.
"""

from __future__ import annotations

import hashlib
import math
import xml.etree.ElementTree as ET
from typing import Final

import numpy as np
import numpy.typing as npt
from sx_contracts.identity import JsonValue, canonical_bytes

from .assets import PACKAGE_URI_PREFIX, description_asset_uri
from .embodiment import Embodiment
from .kinematics import JointType, Kinematics, Transform, from_xyz_rpy
from .materialize import materialize

SCHEMA: Final = "sx.collision-proxies/v2"
GENERATOR: Final = "sx_embodiments.collision_proxies/v2"
FILE: Final = "collision.json"
#: Links at most this many moving joints apart are not checked against each other.
ADJACENT_JOINTS: Final = 2
_MAX_CAPSULES: Final = 3
_SPLIT_GAIN: Final = 0.15
_MICRO: Final = 1e-6

Floats = npt.NDArray[np.float64]
Capsule = tuple[Floats, Floats, float]


def _matrix(transform: Transform) -> Floats:
    return np.asarray(transform, dtype=np.float64).reshape(4, 4)


def _collision_points(embodiment: Embodiment, urdf: bytes) -> dict[str, Floats]:
    """Each link's collision mesh vertices (and primitive corners) in the link frame."""
    from .render_pack import RenderPackError, _box, _cylinder, _floats, _load, _sphere, _xyz

    root = ET.fromstring(urdf)  # nosec B314 - registry-owned description bytes
    directory = materialize(embodiment)
    description = embodiment.urdf.asset.uri
    points: dict[str, Floats] = {}
    for link in root.findall("link"):
        name = link.get("name", "")
        parts: list[Floats] = []
        for collision in link.findall("collision"):
            origin = collision.find("origin")
            transform = _matrix(
                from_xyz_rpy(
                    _xyz(_floats(None if origin is None else origin.get("xyz"), 3, "0 0 0", "xyz")),
                    _xyz(_floats(None if origin is None else origin.get("rpy"), 3, "0 0 0", "rpy")),
                )
            )
            geometry = collision.find("geometry")
            shape = geometry[0] if geometry is not None and len(geometry) else None
            if shape is None:
                raise RenderPackError(f"link {name!r} has a collision with no geometry")
            match shape.tag:
                case "mesh":
                    filename = shape.get("filename", "")
                    scale = _floats(shape.get("scale"), 3, "1 1 1", "scale")
                    uri = description_asset_uri(description, filename)
                    if not uri.startswith(PACKAGE_URI_PREFIX):
                        raise RenderPackError(f"link {name!r}: {filename!r} is not packaged")
                    relpath = uri.removeprefix(PACKAGE_URI_PREFIX)
                    vertices = _load(directory / relpath, relpath, scale).vertices
                case "box":
                    vertices = _box(_floats(shape.get("size"), 3, "", "box size")).vertices
                case "cylinder":
                    radius = _floats(shape.get("radius"), 1, "", "radius")[0]
                    length = _floats(shape.get("length"), 1, "", "length")[0]
                    vertices = _cylinder(radius, length).vertices
                case "sphere":
                    vertices = _sphere(_floats(shape.get("radius"), 1, "", "radius")[0]).vertices
                case other:
                    raise RenderPackError(f"link {name!r} has a {other!r} collision")
            parts.append(vertices @ transform[:3, :3].T + transform[:3, 3])
        if parts:
            points[name] = np.vstack(parts)
    return points


def _principal(points: Floats) -> Floats:
    """The points' principal directions, longest first, each with its largest component > 0."""
    centred = points - points.mean(axis=0)
    _, vectors = np.linalg.eigh(centred.T @ centred)
    axes = vectors[:, ::-1].T.copy()
    for axis in axes:
        if axis[int(np.argmax(np.abs(axis)))] < 0:
            axis *= -1.0
    return axes


def _capsule(points: Floats) -> Capsule:
    """The capsule along the points' principal direction that contains every point."""
    axis, across, third = _principal(points)
    flat = np.stack([points @ across, points @ third], axis=1)
    middle = (flat.min(axis=0) + flat.max(axis=0)) / 2
    t = points @ axis
    base = middle[0] * across + middle[1] * third
    r = np.linalg.norm(flat - middle, axis=1)
    radius = float(r.max())
    slack = np.sqrt(np.maximum(radius**2 - r**2, 0.0))
    low, high = float((t + slack).min()), float((t - slack).max())
    if low > high:  # a sphere: centre it and cover every point from there
        centre = base + axis * (low + high) / 2
        radius = float(np.linalg.norm(points - centre, axis=1).max())
        return centre, centre.copy(), radius
    return base + axis * low, base + axis * high, radius


def _volume(capsule: Capsule) -> float:
    a, b, radius = capsule
    return math.pi * radius**2 * float(np.linalg.norm(b - a)) + 4 / 3 * math.pi * radius**3


def fit(points: Floats) -> list[Capsule]:
    """One to three capsules that together contain ``points``."""
    best = [_capsule(points)]
    across = _principal(points)[1]
    s = points @ across
    for count in range(2, _MAX_CAPSULES + 1):
        edges = np.linspace(s.min(), s.max(), count + 1)
        slabs = [
            points[(s >= low) & (s <= high)] for low, high in zip(edges[:-1], edges[1:], strict=True)
        ]
        if any(len(slab) < 4 for slab in slabs):
            break
        candidate = [_capsule(slab) for slab in slabs]
        if sum(map(_volume, candidate)) < (1 - _SPLIT_GAIN) * sum(map(_volume, best)):
            best = candidate
    return best


def _distance(p: Capsule, q: Capsule) -> float:
    """Signed distance between two capsules' surfaces (negative when they overlap)."""
    (a0, a1, ra), (b0, b1, rb) = p, q
    d1, d2, r = a1 - a0, b1 - b0, a0 - b0
    aa, ee, f = float(d1 @ d1), float(d2 @ d2), float(d2 @ r)
    if aa <= 1e-18 and ee <= 1e-18:
        s = t = 0.0
    elif aa <= 1e-18:
        s, t = 0.0, min(max(f / ee, 0.0), 1.0)
    else:
        c = float(d1 @ r)
        if ee <= 1e-18:
            t, s = 0.0, min(max(-c / aa, 0.0), 1.0)
        else:
            bb = float(d1 @ d2)
            denominator = aa * ee - bb * bb
            s = min(max((bb * f - c * ee) / denominator, 0.0), 1.0) if denominator > 1e-18 else 0.0
            t = (bb * s + f) / ee
            if t < 0.0:
                t, s = 0.0, min(max(-c / aa, 0.0), 1.0)
            elif t > 1.0:
                t, s = 1.0, min(max((bb - c) / aa, 0.0), 1.0)
    gap = (a0 + d1 * s) - (b0 + d2 * t)
    return float(np.linalg.norm(gap)) - ra - rb


def _moving_joints_between(kinematics: Kinematics, a: str, b: str) -> int:
    up, down = kinematics.path(a, b)
    return sum(1 for joint in (*up, *down) if joint.type is not JointType.FIXED)


def _at_zero(kinematics: Kinematics, root: str, link: str, at_zero: dict[str, float]) -> Transform:
    """``root_from_link`` at zero; a floating or planar joint holds its origin."""
    transform = _matrix(kinematics.pose(root, root))
    for joint in reversed(kinematics.path(root, link)[1]):
        held = joint.type in (JointType.FLOATING, JointType.PLANAR)
        local = joint.origin if held else kinematics.pose(joint.parent, joint.child, at_zero)
        transform = _matrix(local) @ transform
    return tuple(float(v) for v in transform.reshape(-1))


def _rounded(capsule: Capsule) -> Capsule:
    a, b, radius = capsule
    # Ends move by at most half a micrometre each; the radius grows by one to cover them.
    return (
        np.round(a / _MICRO) * _MICRO,
        np.round(b / _MICRO) * _MICRO,
        math.ceil(radius / _MICRO + 1) * _MICRO,
    )


def proxies(embodiment: Embodiment, *, name: str | None = None) -> bytes | None:
    """``embodiment``'s ``collision.json``, or ``None`` when no link has collision geometry."""
    urdf = embodiment.urdf_bytes
    points = _collision_points(embodiment, urdf)
    if not points:
        return None
    kinematics = Kinematics.of(urdf)
    capsules = {link: [_rounded(c) for c in fit(p)] for link, p in points.items()}
    (root,) = sorted(link for link in kinematics.links if link not in kinematics.parent_joint)
    at_zero = {
        joint: 0.0
        for joint in kinematics.independent
        if kinematics.joints[joint].type not in (JointType.FLOATING, JointType.PLANAR)
    }
    zero = {link: _at_zero(kinematics, root, link, at_zero) for link in capsules}
    placed = {
        link: [
            (m[:3, :3] @ a + m[:3, 3], m[:3, :3] @ b + m[:3, 3], r)
            for a, b, r in shapes
            for m in [_matrix(zero[link])]
        ]
        for link, shapes in capsules.items()
    }
    links = list(capsules)
    adjacent: list[JsonValue] = []
    touching: list[JsonValue] = []
    for i, first in enumerate(links):
        for second in links[i + 1 :]:
            pair: JsonValue = sorted([first, second])
            if _moving_joints_between(kinematics, first, second) <= ADJACENT_JOINTS:
                adjacent.append(pair)
            elif min(_distance(p, q) for p in placed[first] for q in placed[second]) < 0:
                touching.append(pair)
    document: dict[str, JsonValue] = {
        "schema": SCHEMA,
        "embodiment_id": name or str(embodiment.name),
        "urdf_sha256": hashlib.sha256(urdf).hexdigest(),
        "capsules": [
            {"link": link, "a": [float(v) for v in a], "b": [float(v) for v in b], "radius": r}
            for link, shapes in capsules.items()
            for a, b, r in shapes
        ],
        "ignored_pairs": {"adjacent": sorted(adjacent), "touching_at_zero": sorted(touching)},
        "adjacent_joints": ADJACENT_JOINTS,
        "support": None
        if embodiment.base_mount is None
        else {"frame": embodiment.base_mount.frame, "kind": str(embodiment.base_mount.kind)},
        "generated_by": GENERATOR,
    }
    return canonical_bytes(document)
