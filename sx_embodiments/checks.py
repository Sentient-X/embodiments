"""Physical checks of a composition: what bus ids and interface ids cannot see.

A composition that mates is not yet a robot that works. Four arms on one base can hit each
other, overload the deck, tip it when they reach out together, or never reach the table they
were put there for. :func:`check_build` answers those questions from the build and its
rendered description, each answer a :class:`Finding` on the member it is about:

- **collision**: the members' collision geometry, each link bounded by oriented boxes, with
  every joint at zero (a collision at rest is a refusal) and across :data:`SAMPLES` sampled
  joint configurations (a warning, with the share of samples that collide). Links of one
  member are the vendor's concern, and a link is never checked against the links it hangs
  from, except a mobile base's chassis, which an arm must not reach into;
- **workspace**: the space each arm's tool point sweeps over the samples, on a
  :data:`VOXEL_M` grid. Two arms that share some of it are warned (they can meet, and
  collide, there); a work surface reports the arms that reach none of it, and the
  composition when no part of it is reached by every arm;
- **payload**: what is mated on a base, the members' description masses and their arms'
  rated payloads, against the base's rating (over it is a refusal);
- **stability**: the centre of mass over the base's support polygon with every joint at zero
  (outside is a refusal), and with every arm reaching the same way at once, in each of
  :data:`DIRECTIONS` directions (outside is a warning).

Every check is sampled, deterministic for a seed, and bounded so a preview stays interactive:
``tests/test_checks.py`` holds the four-arm base under :data:`BUDGET_S` (0.14 s measured on an
idle Apple M-series laptop; the first check in a process also reads its meshes' bounds once,
0.4 s for the B601's).
"""

from __future__ import annotations

import math
import random
import struct
import xml.etree.ElementTree as ET
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from functools import cache
from pathlib import Path, PurePosixPath
from typing import Final

from .assembly import Assembly
from .assets import PACKAGE_URI_PREFIX, asset_manifest, cached_blob, local_asset_root
from .compose import Component
from .mechanical import Build, Module, Socket
from .mobile_base import CHASSIS, MobileBase
from .parts import ChainSpec, Limb

#: Joint configurations sampled per check; the budget below is measured with it.
SAMPLES: Final = 256
#: The workspace grid's cell edge, metres.
VOXEL_M: Final = 0.1
#: Horizontal directions the arms reach together in when stability is checked.
DIRECTIONS: Final = 16
#: What one check of a four-arm base may take, seconds, with its meshes' bounds cached.
BUDGET_S: Final = 0.5

Affine = tuple[float, ...]
"""A rigid transform as twelve floats, a 3x3 rotation's rows each followed by a translation."""

_IDENTITY: Final[Affine] = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)


class Check(StrEnum):
    COLLISION_AT_REST = "collision_at_rest"
    COLLISION_IN_MOTION = "collision_in_motion"
    WORKSPACE_OVERLAP = "workspace_overlap"
    WORK_SURFACE_REACH = "work_surface_reach"
    PAYLOAD = "payload"
    STABILITY_AT_REST = "stability_at_rest"
    STABILITY_EXTENDED = "stability_extended"
    BASE_NOT_QUALIFIED = "base_not_qualified"


class Severity(StrEnum):
    #: The composition cannot be registered while it stands.
    REFUSAL = "refusal"
    #: Registered as it is; what a person should know about it.
    WARNING = "warning"


@dataclass(frozen=True, slots=True)
class Collision:
    link: str
    other_link: str
    #: The share of sampled configurations in which the two overlap; 1 at rest.
    fraction: float


@dataclass(frozen=True, slots=True)
class WorkspaceOverlap:
    shared_m3: float
    fraction_of_smaller: float


@dataclass(frozen=True, slots=True)
class WorkSurfaceReach:
    reached_fraction: float


@dataclass(frozen=True, slots=True)
class Payload:
    load_kg: float
    capacity_kg: float


@dataclass(frozen=True, slots=True)
class Stability:
    #: Distance to the support polygon's nearest edge; negative outside it.
    margin_m: float
    centre_of_mass: tuple[float, float, float]


Detail = Collision | WorkspaceOverlap | WorkSurfaceReach | Payload | Stability


@dataclass(frozen=True, slots=True)
class Finding:
    """One check's result on ``entry`` (a placement or mate; empty for the whole build) and,
    for a pair, on ``other``."""

    entry: str
    check: Check
    severity: Severity
    message: str
    detail: Detail | None = None
    other: str = ""


@dataclass(frozen=True, slots=True)
class WorkSurface:
    """A horizontal table top in the build's root frame: its centre and its size, metres."""

    centre: tuple[float, float, float]
    size: tuple[float, float]


def refusals(findings: Iterable[Finding]) -> tuple[Finding, ...]:
    return tuple(finding for finding in findings if finding.severity is Severity.REFUSAL)


def check_build(
    build: Build,
    urdf: bytes,
    *,
    work_surface: WorkSurface | None = None,
    samples: int = SAMPLES,
    seed: int = 0,
) -> tuple[Finding, ...]:
    """Every physical check of ``build``, whose rendered description is ``urdf`` (the
    composed body's, meshes named by package URI), refusals first."""
    body = _Body.of(build, urdf)
    rest = body.forward(body.rest)
    configurations = [body.rest, *body.sample(samples, seed)]
    poses = [rest, *(body.forward(q) for q in configurations[1:])]
    findings = [
        *_collisions(body, poses),
        *_workspaces(body, poses, work_surface),
        *_bases(body, poses),
    ]
    order = {Severity.REFUSAL: 0, Severity.WARNING: 1}
    return tuple(sorted(findings, key=lambda finding: order[finding.severity]))


@dataclass(frozen=True, slots=True)
class _Joint:
    name: str
    parent: str
    child: str
    origin: Affine
    kind: str
    axis: tuple[float, float, float]
    mimic: tuple[str, float, float] | None
    bounds: tuple[float, float] | None


@dataclass(frozen=True, slots=True)
class _Box:
    """An oriented box in its link's frame: its centre and axes (``frame``) and half sizes."""

    frame: Affine
    half: tuple[float, float, float]

    @property
    def radius(self) -> float:
        return math.sqrt(sum(value * value for value in self.half))


@dataclass(frozen=True, slots=True)
class _Body:
    """The rendered description read once: its joint tree in order, every link's boxes and
    mass, which member each link belongs to, and the build's arms and bases."""

    root: str
    joints: tuple[_Joint, ...]
    owner: Mapping[str, str]
    boxes: Mapping[str, tuple[_Box, ...]]
    spheres: Mapping[str, tuple[tuple[float, float, float], float]]  # bounding, link frame
    masses: Mapping[str, tuple[float, tuple[float, float, float]]]
    parent: Mapping[str, _Joint]
    arms: Mapping[str, tuple[str, Affine]]  # member -> its tool link and the tool in it
    bases: Mapping[str, MobileBase]
    hosts: Mapping[str, str]  # mated member -> the member it is mated on
    payloads: Mapping[str, float]  # member -> its arms' rated payload
    rest: Mapping[str, float]

    @classmethod
    def of(cls, build: Build, urdf: bytes) -> _Body:
        robot = ET.fromstring(urdf)  # nosec B314 - a description this package rendered
        assembly = build.assembly()
        owner: dict[str, str] = {}
        for member in assembly.members:
            prefix = assembly.prefix(member.name)
            for link in member.source().findall("link"):
                owner.setdefault(prefix + link.attrib["name"], member.name)
        modules = {entry.name: entry.module for entry in (*build.placed, *build.mates)}
        joints = _ordered(robot, build.root or "")
        boxes: dict[str, tuple[_Box, ...]] = {}
        masses: dict[str, tuple[float, tuple[float, float, float]]] = {}
        for link in robot.findall("link"):
            name = link.attrib["name"]
            boxes[name] = tuple(
                box for element in link.findall("collision") for box in _boxes(element)
            )
            inertial = link.find("inertial")
            mass = inertial.find("mass") if inertial is not None else None
            if inertial is not None and mass is not None:
                masses[name] = (
                    float(mass.attrib["value"]),
                    _triple(inertial.find("origin"), "xyz"),
                )
        arms: dict[str, tuple[str, Affine]] = {}
        payloads: dict[str, float] = {}
        for name, module in modules.items():
            chains = [entry for entry in module.components if isinstance(entry, Component)]
            arm = next((entry for entry in chains if entry.limb is Limb.ARM), None)
            payloads[name] = sum(
                entry.part.physical.payload_kg or 0.0
                for entry in chains
                if entry.limb is Limb.ARM
                and isinstance(entry.part, ChainSpec)
                and entry.part.physical is not None
            )
            if arm is not None:
                arms[name] = _tool(assembly, name, module, arm.instance, owner, joints)
        return cls(
            root=build.root or "",
            joints=joints,
            owner=owner,
            boxes=boxes,
            spheres={link: _sphere(shapes) for link, shapes in boxes.items() if shapes},
            masses=masses,
            parent={joint.child: joint for joint in joints},
            arms=arms,
            bases={name: module.base for name, module in modules.items() if module.base},
            hosts={mate.name: mate.host for mate in build.mates},
            payloads=payloads,
            rest={
                joint.name: min(max(0.0, joint.bounds[0]), joint.bounds[1])
                if joint.bounds is not None
                else 0.0
                for joint in joints
                if joint.kind in _MOVING and joint.mimic is None
            },
        )

    def sample(self, count: int, seed: int) -> list[dict[str, float]]:
        """``count`` configurations, every bounded joint uniform in its bounds and every
        unbounded one (a wheel) at rest."""
        generator = random.Random(seed)
        return [
            {
                joint.name: generator.uniform(*joint.bounds) if joint.bounds is not None else 0.0
                for joint in self.joints
                if joint.kind in _MOVING and joint.mimic is None
            }
            for _ in range(count)
        ]

    def forward(self, positions: Mapping[str, float]) -> dict[str, Affine]:
        """Every link's pose in the root frame."""
        poses: dict[str, Affine] = {self.root: _IDENTITY}
        values = dict(positions)
        for joint in self.joints:
            local = joint.origin
            if joint.kind in _MOVING:
                if joint.mimic is not None:
                    leader, multiplier, offset = joint.mimic
                    value = multiplier * values.get(leader, 0.0) + offset
                    values[joint.name] = value
                else:
                    value = values.get(joint.name, 0.0)
                motion = (
                    _rotation(joint.axis, value)
                    if joint.kind != "prismatic"
                    else _translation(joint.axis, value)
                )
                local = _mul(local, motion)
            poses[joint.child] = _mul(poses[joint.parent], local)
        return poses

    def ancestors(self, link: str) -> list[tuple[str, int]]:
        """Each link above ``link`` and how many moving joints lie between them."""
        out: list[tuple[str, int]] = []
        moving = 0
        while (joint := self.parent.get(link)) is not None:
            moving += joint.kind in _MOVING
            link = joint.parent
            out.append((link, moving))
        return out

    def carried_by(self, base: str) -> list[str]:
        """The members mated on ``base``, directly or through another member."""

        def on(member: str) -> bool:
            while (host := self.hosts.get(member)) is not None:
                if host == base:
                    return True
                member = host
            return False

        return [member for member in self.hosts if on(member)]

    def mass_of(self, members: Iterable[str]) -> float:
        chosen = set(members)
        return sum(
            mass for link, (mass, _) in self.masses.items() if self.owner.get(link) in chosen
        )


_MOVING: Final = frozenset({"revolute", "continuous", "prismatic"})


def _ordered(robot: ET.Element, root: str) -> tuple[_Joint, ...]:
    """The description's joints, each after the joint above its parent link."""
    by_parent: dict[str, list[_Joint]] = {}
    for element in robot.findall("joint"):
        parent, child = element.find("parent"), element.find("child")
        assert parent is not None and child is not None
        limit = element.find("limit")
        kind = element.attrib["type"]
        mimic = element.find("mimic")
        bounds: tuple[float, float] | None = None
        if kind == "revolute" or kind == "prismatic":
            lower = float(limit.get("lower", "0")) if limit is not None else 0.0
            upper = float(limit.get("upper", "0")) if limit is not None else 0.0
            bounds = (min(lower, upper), max(lower, upper))
        axis = _triple(element.find("axis"), "xyz", "1 0 0")
        norm = math.sqrt(sum(value * value for value in axis)) or 1.0
        origin = element.find("origin")
        joint = _Joint(
            name=element.attrib["name"],
            parent=parent.attrib["link"],
            child=child.attrib["link"],
            origin=_affine(_triple(origin, "xyz"), _triple(origin, "rpy")),
            kind=kind,
            axis=(axis[0] / norm, axis[1] / norm, axis[2] / norm),
            mimic=(
                (
                    mimic.attrib["joint"],
                    float(mimic.get("multiplier", "1")),
                    float(mimic.get("offset", "0")),
                )
                if mimic is not None
                else None
            ),
            bounds=bounds,
        )
        by_parent.setdefault(joint.parent, []).append(joint)
    out: list[_Joint] = []
    stack = [root]
    while stack:
        link = stack.pop()
        for joint in by_parent.get(link, ()):
            out.append(joint)
            stack.append(joint.child)
    return tuple(out)


def _tool(
    assembly: Assembly,
    member: str,
    module: Module,
    arm: str,
    owner: Mapping[str, str],
    joints: Sequence[_Joint],
) -> tuple[str, Affine]:
    """An arm's tool point: the socket its arm carries (its flange), else its deepest link."""
    prefix = assembly.prefix(member)
    flange: Socket | None = next(
        (socket for socket in module.sockets if socket.carrier == arm), None
    )
    if flange is not None:
        return prefix + flange.link, _affine(flange.pose.xyz, flange.pose.rpy)
    depth: dict[str, int] = {}
    for joint in joints:
        depth[joint.child] = depth.get(joint.parent, 0) + (joint.kind in _MOVING)
    own = [link for link, name in owner.items() if name == member]
    return max(own, key=lambda link: depth.get(link, 0)), _IDENTITY


# --- geometry ---------------------------------------------------------------------------


def _triple(
    element: ET.Element | None, key: str, default: str = "0 0 0"
) -> tuple[float, float, float]:
    text = element.get(key, default) if element is not None else default
    x, y, z = (float(value) for value in text.split())
    return (x, y, z)


def _affine(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> Affine:
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    return (
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0],
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1],
        -sp, cp * sr, cp * cr, xyz[2],
    )  # fmt: skip


def _rotation(axis: tuple[float, float, float], angle: float) -> Affine:
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    k = 1.0 - c
    return (
        c + x * x * k, x * y * k - z * s, x * z * k + y * s, 0.0,
        y * x * k + z * s, c + y * y * k, y * z * k - x * s, 0.0,
        z * x * k - y * s, z * y * k + x * s, c + z * z * k, 0.0,
    )  # fmt: skip


def _translation(axis: tuple[float, float, float], distance: float) -> Affine:
    x, y, z = axis
    return (1.0, 0.0, 0.0, x * distance, 0.0, 1.0, 0.0, y * distance, 0.0, 0.0, 1.0, z * distance)


def _mul(a: Affine, b: Affine) -> Affine:
    a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11 = a
    b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11 = b
    return (
        a0 * b0 + a1 * b4 + a2 * b8,
        a0 * b1 + a1 * b5 + a2 * b9,
        a0 * b2 + a1 * b6 + a2 * b10,
        a0 * b3 + a1 * b7 + a2 * b11 + a3,
        a4 * b0 + a5 * b4 + a6 * b8,
        a4 * b1 + a5 * b5 + a6 * b9,
        a4 * b2 + a5 * b6 + a6 * b10,
        a4 * b3 + a5 * b7 + a6 * b11 + a7,
        a8 * b0 + a9 * b4 + a10 * b8,
        a8 * b1 + a9 * b5 + a10 * b9,
        a8 * b2 + a9 * b6 + a10 * b10,
        a8 * b3 + a9 * b7 + a10 * b11 + a11,
    )


def _point(transform: Affine, xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = xyz
    t = transform
    return (
        t[0] * x + t[1] * y + t[2] * z + t[3],
        t[4] * x + t[5] * y + t[6] * z + t[7],
        t[8] * x + t[9] * y + t[10] * z + t[11],
    )


def _boxes(collision: ET.Element) -> list[_Box]:
    """The oriented boxes bounding one collision element, in its link's frame."""
    origin = collision.find("origin")
    frame = _affine(_triple(origin, "xyz"), _triple(origin, "rpy"))
    geometry = collision.find("geometry")
    if geometry is None or len(geometry) == 0:
        return []
    shape = geometry[0]
    match shape.tag:
        case "box":
            x, y, z = _triple(shape, "size")
            return [_Box(frame, (x / 2, y / 2, z / 2))]
        case "cylinder":
            radius, length = float(shape.attrib["radius"]), float(shape.attrib["length"])
            return [_Box(frame, (radius, radius, length / 2))]
        case "sphere":
            radius = float(shape.attrib["radius"])
            return [_Box(frame, (radius, radius, radius))]
        case "mesh":
            path = _mesh(shape.attrib["filename"])
            if path is None:
                return []
            scale = _triple(shape, "scale", "1 1 1")
            low, high = _stl_bounds(path)
            centre = tuple((lo + hi) / 2 * s for lo, hi, s in zip(low, high, scale, strict=True))
            half = tuple(abs(hi - lo) / 2 * s for lo, hi, s in zip(low, high, scale, strict=True))
            cx, cy, cz = centre
            hx, hy, hz = half
            return [_Box(_mul(frame, (1, 0, 0, cx, 0, 1, 0, cy, 0, 0, 1, cz)), (hx, hy, hz))]
        case _:
            return []


def _sphere(boxes: Sequence[_Box]) -> tuple[tuple[float, float, float], float]:
    """A sphere around every box of a link: centred on their centres' mean."""
    count = len(boxes)
    centre = (
        sum(box.frame[3] for box in boxes) / count,
        sum(box.frame[7] for box in boxes) / count,
        sum(box.frame[11] for box in boxes) / count,
    )
    radius = max(
        math.dist(centre, (box.frame[3], box.frame[7], box.frame[11])) + box.radius for box in boxes
    )
    return centre, radius


@cache
def _mesh(uri: str) -> Path | None:
    """A packaged mesh's verified file, or none for one this package does not hold (its link
    then has no collision geometry in the checks)."""
    found: Path | None = None
    relpath = uri.removeprefix(PACKAGE_URI_PREFIX)
    entry = asset_manifest().get(relpath) if uri.startswith(PACKAGE_URI_PREFIX) else None
    if entry is not None and PurePosixPath(relpath).suffix.lower() == ".stl":
        root = local_asset_root()
        found = cached_blob(
            entry.content, relpath=relpath, local=root / relpath if root is not None else None
        )
    return found


@cache
def _stl_bounds(path: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """An STL's axis-aligned bounds; cached for the life of the process, by verified path."""
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if len(data) == 84 + 50 * count:
        records = struct.iter_unpack("<12fH", memoryview(data)[84:])
        xs: list[float] = []
        ys: list[float] = []
        zs: list[float] = []
        for record in records:
            xs += (record[3], record[6], record[9])
            ys += (record[4], record[7], record[10])
            zs += (record[5], record[8], record[11])
    else:
        vertices = [
            tuple(float(value) for value in line.split()[1:4])
            for line in data.decode("ascii", "replace").splitlines()
            if line.strip().startswith("vertex")
        ]
        xs = [vertex[0] for vertex in vertices]
        ys = [vertex[1] for vertex in vertices]
        zs = [vertex[2] for vertex in vertices]
    if not xs:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def _overlap(
    a: Affine, ha: tuple[float, float, float], b: Affine, hb: tuple[float, float, float]
) -> bool:
    """Whether two oriented boxes (frames in one space, half sizes) intersect: the separating
    axis test over their fifteen candidate axes."""
    # Axes as the columns of each rotation.
    ax = ((a[0], a[4], a[8]), (a[1], a[5], a[9]), (a[2], a[6], a[10]))
    bx = ((b[0], b[4], b[8]), (b[1], b[5], b[9]), (b[2], b[6], b[10]))
    d = (b[3] - a[3], b[7] - a[7], b[11] - a[11])
    r = [[sum(ax[i][k] * bx[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    absr = [[abs(value) + 1e-9 for value in row] for row in r]
    t = [sum(d[k] * ax[i][k] for k in range(3)) for i in range(3)]
    for i in range(3):
        if abs(t[i]) > ha[i] + sum(hb[j] * absr[i][j] for j in range(3)):
            return False
    for j in range(3):
        if abs(sum(t[i] * r[i][j] for i in range(3))) > hb[j] + sum(
            ha[i] * absr[i][j] for i in range(3)
        ):
            return False
    for i in range(3):
        i1, i2 = (i + 1) % 3, (i + 2) % 3
        for j in range(3):
            j1, j2 = (j + 1) % 3, (j + 2) % 3
            separation = abs(t[i2] * r[i1][j] - t[i1] * r[i2][j])
            reach = (
                ha[i1] * absr[i2][j]
                + ha[i2] * absr[i1][j]
                + hb[j1] * absr[i][j2]
                + hb[j2] * absr[i][j1]
            )
            if separation > reach:
                return False
    return True


# --- collision --------------------------------------------------------------------------


_Pair = tuple[str, str, float]
"""Two links and the square of their bounding spheres' summed radii."""


def _pairs(body: _Body) -> dict[tuple[str, str], list[_Pair]]:
    """The link pairs checked, by the two members they belong to: links of two members,
    neither hanging from the other, except a base's chassis against the links of other
    members two or more moving joints above it."""
    shaped = [link for link, boxes in body.boxes.items() if boxes and link in body.owner]
    above = {link: dict(body.ancestors(link)) for link in shaped}
    out: dict[tuple[str, str], list[_Pair]] = {}
    for index, link in enumerate(shaped):
        for other in shaped[index + 1 :]:
            if body.owner[link] == body.owner[other]:
                continue
            if other in above[link] or link in above[other]:
                low, high = (link, other) if other in above[link] else (other, link)
                chassis = body.owner[high] in body.bases and high.endswith(CHASSIS)
                if not (chassis and above[low][high] >= 2):
                    continue
            reach = body.spheres[link][1] + body.spheres[other][1]
            out.setdefault(_members(body, link, other), []).append((link, other, reach * reach))
    return out


def _colliding(
    body: _Body,
    pose: Mapping[str, Affine],
    pairs: Mapping[tuple[str, str], list[_Pair]],
    skip: Iterable[tuple[str, str]] = (),
) -> dict[tuple[str, str], tuple[str, str]]:
    """Each pair of members that overlaps at ``pose``, with the first two links that do."""
    passed = set(skip)
    centres = {link: _point(pose[link], centre) for link, (centre, _) in body.spheres.items()}
    boxes: dict[str, list[tuple[Affine, tuple[float, float, float]]]] = {}

    def placed(link: str) -> list[tuple[Affine, tuple[float, float, float]]]:
        known = boxes.get(link)
        if known is None:
            known = boxes[link] = [
                (_mul(pose[link], box.frame), box.half) for box in body.boxes[link]
            ]
        return known

    hits: dict[tuple[str, str], tuple[str, str]] = {}
    for members, links in pairs.items():
        if members in passed:
            continue
        for link, other, reach in links:
            a, b = centres[link], centres[other]
            if (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2 > reach:
                continue
            if any(
                _overlap(box, half, other_box, other_half)
                for box, half in placed(link)
                for other_box, other_half in placed(other)
            ):
                hits[members] = (link, other)
                break
    return hits


def _collisions(body: _Body, poses: Sequence[Mapping[str, Affine]]) -> list[Finding]:
    pairs = _pairs(body)
    at_rest = _colliding(body, poses[0], pairs)
    moving: dict[tuple[str, str], tuple[int, tuple[str, str]]] = {}
    for pose in poses[1:]:
        for members, links in _colliding(body, pose, pairs, at_rest).items():
            count, first = moving.get(members, (0, links))
            moving[members] = (count + 1, first)
    findings = [
        Finding(
            entry=entry,
            other=other_entry,
            check=Check.COLLISION_AT_REST,
            severity=Severity.REFUSAL,
            message=f"{entry} and {other_entry} overlap at rest ({link} meets {other})",
            detail=Collision(link, other, 1.0),
        )
        for (entry, other_entry), (link, other) in at_rest.items()
    ]
    sampled = max(len(poses) - 1, 1)
    for (entry, other_entry), (count, (link, other)) in moving.items():
        fraction = count / sampled
        findings.append(
            Finding(
                entry=entry,
                other=other_entry,
                check=Check.COLLISION_IN_MOTION,
                severity=Severity.WARNING,
                message=(
                    f"{entry} and {other_entry} can collide: they overlap in "
                    f"{fraction:.0%} of sampled poses"
                ),
                detail=Collision(link, other, fraction),
            )
        )
    return findings


def _members(body: _Body, link: str, other: str) -> tuple[str, str]:
    first, second = body.owner[link], body.owner[other]
    return (first, second) if first <= second else (second, first)


# --- workspace --------------------------------------------------------------------------


def _cell(point: tuple[float, float, float]) -> tuple[int, int, int]:
    return (
        math.floor(point[0] / VOXEL_M),
        math.floor(point[1] / VOXEL_M),
        math.floor(point[2] / VOXEL_M),
    )


def _workspaces(
    body: _Body, poses: Sequence[Mapping[str, Affine]], surface: WorkSurface | None
) -> list[Finding]:
    tools = {
        arm: [_point(_mul(pose[link], tool), (0.0, 0.0, 0.0)) for pose in poses]
        for arm, (link, tool) in body.arms.items()
    }
    cells = {arm: {_cell(point) for point in points} for arm, points in tools.items()}
    findings: list[Finding] = []
    arms = sorted(cells)
    cube = VOXEL_M**3
    for index, arm in enumerate(arms):
        for other in arms[index + 1 :]:
            shared = len(cells[arm] & cells[other])
            if shared == 0:
                continue
            fraction = shared / min(len(cells[arm]), len(cells[other]))
            findings.append(
                Finding(
                    entry=arm,
                    other=other,
                    check=Check.WORKSPACE_OVERLAP,
                    severity=Severity.WARNING,
                    message=(
                        f"{arm} and {other} share {shared * cube:.3f} m3 of workspace "
                        f"({fraction:.0%} of the smaller): they can meet, and collide, there"
                    ),
                    detail=WorkspaceOverlap(shared * cube, min(fraction, 1.0)),
                )
            )
    if surface is None or not arms:
        return findings
    table = _table(surface)
    reached: dict[str, set[tuple[int, int]]] = {}
    low, high = surface.centre[2] - VOXEL_M / 2, surface.centre[2] + 1.5 * VOXEL_M
    for arm, points in tools.items():
        reached[arm] = {
            cell
            for x, y, z in points
            if low <= z <= high
            and (cell := (math.floor(x / VOXEL_M), math.floor(y / VOXEL_M))) in table
        }
        if not reached[arm]:
            findings.append(
                Finding(
                    entry=arm,
                    check=Check.WORK_SURFACE_REACH,
                    severity=Severity.WARNING,
                    message=f"{arm} reaches none of the work surface",
                    detail=WorkSurfaceReach(0.0),
                )
            )
    common = set(table)
    for cells in reached.values():
        common &= cells
    if len(arms) > 1 and not common:
        findings.append(
            Finding(
                entry="",
                check=Check.WORK_SURFACE_REACH,
                severity=Severity.WARNING,
                message="no part of the work surface is reached by every arm",
                detail=WorkSurfaceReach(0.0),
            )
        )
    return findings


def _table(surface: WorkSurface) -> frozenset[tuple[int, int]]:
    (x, y, _), (length, width) = surface.centre, surface.size
    return frozenset(
        (i, j)
        for i in range(
            math.floor((x - length / 2) / VOXEL_M), math.floor((x + length / 2) / VOXEL_M) + 1
        )
        for j in range(
            math.floor((y - width / 2) / VOXEL_M), math.floor((y + width / 2) / VOXEL_M) + 1
        )
    )


# --- bases ------------------------------------------------------------------------------


def _bases(body: _Body, poses: Sequence[Mapping[str, Affine]]) -> list[Finding]:
    findings: list[Finding] = []
    for name, base in body.bases.items():
        findings.append(
            Finding(
                entry=name,
                check=Check.BASE_NOT_QUALIFIED,
                severity=Severity.WARNING,
                message=f"{base.label} has no qualified wheel motor: it drives in simulation only",
            )
        )
        carried = body.carried_by(name)
        load = body.mass_of(carried) + sum(body.payloads.get(member, 0.0) for member in carried)
        if load > base.payload_kg:
            findings.append(
                Finding(
                    entry=name,
                    check=Check.PAYLOAD,
                    severity=Severity.REFUSAL,
                    message=(
                        f"{base.label} carries {load:.1f} kg, over its {base.payload_kg:.1f} kg "
                        "rating"
                    ),
                    detail=Payload(load, base.payload_kg),
                )
            )
        findings.extend(_stability(body, name, base, [name, *carried], poses))
    return findings


def _moments(
    body: _Body, members: set[str], pose: Mapping[str, Affine]
) -> tuple[float, float, float, float]:
    """Mass and its first moments about the root's axes, for the links of ``members``."""
    mass = mx = my = mz = 0.0
    for link, (m, centre) in body.masses.items():
        if body.owner.get(link) in members and link in pose:
            x, y, z = _point(pose[link], centre)
            mass, mx, my, mz = mass + m, mx + m * x, my + m * y, mz + m * z
    return mass, mx, my, mz


def _margin(polygon: Sequence[tuple[float, float]], x: float, y: float) -> float:
    """Signed distance from (x, y) to a convex counter-clockwise polygon's edges; negative
    outside."""
    margin = math.inf
    for (x0, y0), (x1, y1) in zip(polygon, (*polygon[1:], polygon[0]), strict=True):
        ex, ey = x1 - x0, y1 - y0
        length = math.hypot(ex, ey)
        margin = min(margin, (ex * (y - y0) - ey * (x - x0)) / length)
    return margin


def _stability(
    body: _Body,
    name: str,
    base: MobileBase,
    members: list[str],
    poses: Sequence[Mapping[str, Affine]],
) -> list[Finding]:
    chassis = next(
        link for link, owner in body.owner.items() if owner == name and link.endswith(CHASSIS)
    )
    frame = poses[0][chassis]
    polygon = [_point(frame, (x, y, 0.0))[:2] for x, y in base.support]
    rest = poses[0]
    arms = [member for member in members if member in body.arms]
    still = set(members) - set(arms)
    mass, mx, my, mz = _moments(body, set(members), rest)
    if mass <= 0.0:
        return []
    findings: list[Finding] = []
    margin = _margin(polygon, mx / mass, my / mass)
    if margin < 0.0:
        findings.append(
            Finding(
                entry=name,
                check=Check.STABILITY_AT_REST,
                severity=Severity.REFUSAL,
                message=(
                    f"the centre of mass lies {-margin:.3f} m outside {base.label}'s support "
                    "at rest: it tips over"
                ),
                detail=Stability(margin, (mx / mass, my / mass, mz / mass)),
            )
        )
        return findings
    if not arms:
        return findings
    fixed = _moments(body, still, rest)
    swept = {arm: [_moments(body, {arm}, pose) for pose in poses] for arm in arms}
    worst: tuple[float, tuple[float, float, float]] | None = None
    for step in range(DIRECTIONS):
        angle = 2 * math.pi * step / DIRECTIONS
        dx, dy = math.cos(angle), math.sin(angle)
        total = list(fixed)
        for moments in swept.values():
            reach = max(moments, key=lambda item: dx * item[1] + dy * item[2])
            total = [a + b for a, b in zip(total, reach, strict=True)]
        m, sx, sy, sz = total
        centre = (sx / m, sy / m, sz / m)
        here = _margin(polygon, centre[0], centre[1])
        if worst is None or here < worst[0]:
            worst = (here, centre)
    assert worst is not None
    if worst[0] < 0.0:
        findings.append(
            Finding(
                entry=name,
                check=Check.STABILITY_EXTENDED,
                severity=Severity.WARNING,
                message=(
                    "with every arm reaching the same way, the centre of mass leaves "
                    f"{base.label}'s support by {-worst[0]:.3f} m: "
                    "limit how far they reach together"
                ),
                detail=Stability(worst[0], worst[1]),
            )
        )
    return findings
