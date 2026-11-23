"""MuJoCo contact evidence and contact-point gradients for offline ego IK."""

from __future__ import annotations

import csv
import hashlib
import os
import xml.etree.ElementTree as ET
from collections.abc import Callable, Sequence
from concurrent.futures import ThreadPoolExecutor
from copy import copy
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Protocol, cast

import mujoco  # pyright: ignore[reportMissingTypeStubs] -- the pybind11 wheel has no stubs
import numpy as np
from numpy.typing import NDArray
from sx_contracts.identity import content_digest

from .kinematics import JointType, Kinematics

type Floats = NDArray[np.float64]
type Pair = tuple[str, str]
type Bearing = tuple[str, str, float]
_mj = cast("ModuleType", mujoco)


def collision_assets(path: Path) -> tuple[ET.Element, dict[str, bytes], str]:
    source = path.read_bytes()
    root = ET.fromstring(source)
    assets: dict[str, bytes] = {}
    identities: dict[str, str] = {}
    for mesh in root.findall(".//collision/geometry/mesh"):
        ref = mesh.attrib["filename"]
        if "://" in ref:
            raise ValueError(f"materialize collision mesh URI in the supplied URDF: {ref}")
        if ref not in identities:
            data = (path.parent / ref).read_bytes()
            digest = hashlib.sha256(data).hexdigest()
            identities[ref] = digest
            assets[digest + Path(ref).suffix.lower()] = data
        mesh.set("filename", identities[ref] + Path(ref).suffix.lower())
    # The nominal bearing certificate is derived from the individual visual CAD
    # parts. Those bytes are therefore also input evidence whenever materialized.
    visuals: dict[str, str | None] = {}
    for mesh in root.findall(".//visual/geometry/mesh"):
        ref = mesh.attrib["filename"]
        if ref not in visuals:
            candidate = path.parent / ref
            visuals[ref] = (
                hashlib.sha256(candidate.read_bytes()).hexdigest()
                if "://" not in ref and candidate.is_file()
                else None
            )
    identity = str(
        content_digest(
            {"urdf": hashlib.sha256(source).hexdigest(), "meshes": identities, "visuals": visuals}
        )
    )
    return root, assets, identity


@dataclass(frozen=True)
class JointInterval:
    joint: str
    lower: float
    upper: float


@dataclass(frozen=True)
class BearingArc:
    pair: Pair
    axes: tuple[JointInterval, ...]
    depth: float


_CAD_SWEEPS = Path(__file__).with_name("sentient_wuji_cad_sweeps.csv")


def nominal_bearings() -> tuple[Bearing, ...]:
    table = Path(__file__).with_name("sentient_wuji_bearings.csv")
    with table.open(newline="") as stream:
        return tuple(
            (row["link_a"], row["link_b"], float(row["cap_m"])) for row in csv.DictReader(stream)
        )


def cad_sweep_identity() -> str:
    return hashlib.sha256(_CAD_SWEEPS.read_bytes()).hexdigest()


def certified_arcs(certificate: str | None, geometry_id: str) -> tuple[BearingArc, ...]:
    if certificate is None:
        return ()
    if certificate != cad_sweep_identity():
        raise ValueError("unknown CAD bearing certificate")
    with _CAD_SWEEPS.open(newline="") as stream:
        arcs: list[BearingArc] = []
        for row in csv.DictReader(stream):
            if row["geometry_sha256"] != geometry_id:
                continue
            axes = tuple(
                JointInterval(
                    row["joint" + suffix],
                    float(row["lower_rad" + suffix]),
                    float(row["upper_rad" + suffix]),
                )
                for suffix in ("", "_2")
                if row.get("joint" + suffix)
            )
            arcs.append(
                BearingArc((row["link_a"], row["link_b"]), axes, float(row["convex_cap_m"]))
            )
        return tuple(arcs)


class _Named(Protocol):
    id: int
    name: str


class _Options(Protocol):
    disableflags: int


class _Model(Protocol):
    opt: _Options
    nv: int
    nbody: int
    ngeom: int
    npair: int
    geom_contype: NDArray[np.int32]
    geom_conaffinity: NDArray[np.int32]
    body_contype: NDArray[np.int32]
    body_conaffinity: NDArray[np.int32]
    geom_bodyid: NDArray[np.int32]
    geom_margin: Floats
    jnt_type: NDArray[np.int32]
    body_jntadr: NDArray[np.int32]
    body_jntnum: NDArray[np.int32]
    body_parentid: NDArray[np.int32]

    def body(self, i: int) -> _Named: ...
    def joint(self, name: str) -> _Named: ...


class _Contact(Protocol):
    dist: float
    geom1: int
    geom2: int
    pos: Floats
    frame: Floats


class _Data(Protocol):
    qpos: Floats
    xanchor: Floats
    xaxis: Floats
    contact: tuple[_Contact, ...]


@dataclass(frozen=True)
class Contacts:
    maximum: float
    excess: float
    unallowed: float
    worst_pair: Pair
    violation: Floats
    jacobian: Floats
    blocked_joints: NDArray[np.bool_]


class CollisionModel:
    """Convex URDF collision geometry with every independently moving link pair checked."""

    margin_m = 0.003

    def __init__(
        self,
        model: _Model,
        ordered_joints: tuple[str, ...],
        neutral: Floats,
        allowed: frozenset[Pair],
        missing_geometry: tuple[str, ...],
        bearings: dict[Pair, float],
        arcs: tuple[BearingArc, ...],
    ) -> None:
        self.model = model
        self.data = cast("_Data", _mj.MjData(model))
        self._query_key: bytes | None = None
        self._previous_query: tuple[bytes, _Data] | None = None
        self.permutation = np.asarray(
            [model.joint(name).id for name in ordered_joints], dtype=np.int64
        )
        self.ancestor = np.zeros((model.nbody, model.nv), dtype=np.int8)
        for i in range(model.nbody):
            parent = i
            while parent > 0:
                start, count = int(model.body_jntadr[parent]), int(model.body_jntnum[parent])
                self.ancestor[i, start : start + count] = 1
                parent = int(model.body_parentid[parent])
        self.names = tuple(model.body(i).name for i in range(model.nbody))
        self.slide = model.jnt_type == int(_mj.mjtJoint.mjJNT_SLIDE)
        self.arcs = arcs
        self.arc_joints = {
            arc.pair: (
                tuple((model.joint(axis.joint).id, axis.lower, axis.upper) for axis in arc.axes),
                arc.depth,
            )
            for arc in arcs
        }
        self.allowed = allowed
        self.missing_geometry = missing_geometry
        self.checked = not missing_geometry and len(np.unique(model.geom_bodyid)) >= 2
        self.neutral: dict[Pair, float] = {}
        self.geom_neutral: dict[tuple[int, int], float] = {}
        self.geom_bearing_cap: dict[tuple[int, int], float] = {}
        self.geom_links = tuple(self.names[int(i)] for i in model.geom_bodyid)
        for geoms, contact in self._contacts(neutral).items():
            depth = -contact.dist
            if depth <= 0:
                continue
            pair = self._pair(geoms)
            self.geom_neutral[geoms] = depth
            self.neutral[pair] = max(self.neutral.get(pair, 0.0), depth)
        for geoms, depth in self.geom_neutral.items():
            pair = self._pair(geoms)
            if pair in bearings:
                # A cap grants at most the audited increase over the link pair's
                # neutral depth to each already-nested geometry pair.
                self.geom_bearing_cap[geoms] = min(
                    bearings[pair], depth + max(0.0, bearings[pair] - self.neutral[pair])
                )

    def _pair(self, geoms: tuple[int, int]) -> Pair:
        a, b = self.geom_links[geoms[0]], self.geom_links[geoms[1]]
        return (min(a, b), max(a, b))

    @classmethod
    def read(
        cls,
        geometry: tuple[ET.Element, dict[str, bytes], str],
        ordered_joints: tuple[str, ...],
        neutral: Floats,
        allowed: tuple[Pair, ...],
        bearings: tuple[Bearing, ...] = (),
        arcs: tuple[BearingArc, ...] = (),
    ) -> CollisionModel | None:
        root, assets, geometry_id = geometry

        # A pre-certificate saved nominal profile can still carry all old
        # depth allowances. Never transfer those grants onto changed geometry.
        def unordered(values: tuple[Bearing, ...]) -> frozenset[Bearing]:
            return frozenset((min(a, b), max(a, b), depth) for a, b, depth in values)

        if (
            bearings
            and unordered(bearings) == unordered(nominal_bearings())
            and not certified_arcs(cad_sweep_identity(), geometry_id)
        ):
            raise ValueError("nominal Sentient-Wuji bearing limits require the exact certified CAD")

        tree = Kinematics.of(ET.tostring(root))
        caps: dict[Pair, float] = {}
        for a, b, depth in bearings:
            if not np.isfinite(depth) or depth <= 0:
                raise ValueError("bearing penetration cap must be finite and positive")
            pair = (min(a, b), max(a, b))
            if pair in caps or pair in allowed or (pair[1], pair[0]) in allowed:
                raise ValueError(f"duplicate or unrestricted bearing allowance {pair}")
            caps[pair] = depth
            # A named allowance is solely for a nested rotary bearing or paired
            # rotary bearings. A slider can drive a link through its parent.
            joints = [j for j in sum(tree.path(a, b), ()) if j.type != JointType.FIXED]
            if not 1 <= len(joints) <= 2 or any(
                j.type not in {JointType.REVOLUTE, JointType.CONTINUOUS} for j in joints
            ):
                raise ValueError(f"depth cap requires one or two rotary bearing axes: {pair}")
        for a, b in (*allowed, *caps):
            if a not in tree.links or b not in tree.links or a == b:
                raise ValueError(f"invalid allowed collision pair {(a, b)}")
        bearing: set[str] = set()
        all_groups: set[str] = set()
        for link in root.findall("link"):
            name = link.attrib["name"]
            # Fixed child frames form one rigid piece; at least one collision shape must
            # describe every moving rigid piece and the base to claim complete coverage.
            group = name
            while group in tree.parent_joint:
                joint = tree.joints[tree.parent_joint[group]]
                if joint.type != JointType.FIXED:
                    break
                group = joint.parent
            all_groups.add(group)
            for element in list(link):
                if element.tag != "collision":
                    link.remove(element)
            if link.findall("collision/geometry/*"):
                bearing.add(group)
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "mass", value="1")
            ET.SubElement(inertial, "inertia", ixx="1", iyy="1", izz="1", ixy="0", ixz="0", iyz="0")
        if not bearing:
            return None
        for item in root.findall("mujoco"):
            root.remove(item)
        ET.SubElement(
            ET.SubElement(root, "mujoco"),
            "compiler",
            fusestatic="false",
            strippath="false",
            discardvisual="true",
            balanceinertia="true",
        )
        spec = _mj.MjSpec.from_string(ET.tostring(root, encoding="unicode"), assets=assets)
        for mesh_spec in spec.meshes:
            mesh_spec.inertia = _mj.mjtMeshInertia.mjMESH_INERTIA_SHELL
        model = cast("_Model", spec.compile())
        # The engine otherwise suppresses collisions between bodies separated by
        # a movable joint, including a slider driven straight into its parent.
        model.opt.disableflags |= int(_mj.mjtDisableBit.mjDSBL_FILTERPARENT)
        model.geom_margin[:] = cls.margin_m
        pairs = frozenset((min(a, b), max(a, b)) for a, b in allowed)
        return cls(
            model, ordered_joints, neutral, pairs, tuple(sorted(all_groups - bearing)), caps, arcs
        )

    def moving_pairs(self, physical: NDArray[np.int64]) -> CollisionModel:
        """Filter only the candidate screening model. The original checks every accepted pose."""
        model = self.model
        if (
            not self.checked
            or not len(physical)
            or model.npair
            or not model.ngeom
            or np.any(model.geom_contype != 1)
            or np.any(model.geom_conaffinity != 1)
        ):
            return self
        active = self.permutation[physical]
        # Identical moving ancestors imply a constant inter-geom transform at
        # each exposure while all other physical coordinates remain frozen.
        _, group = np.unique(self.ancestor[:, active], axis=0, return_inverse=True)
        labels = group[model.geom_bodyid] + 1
        if len(np.unique(labels)) == len(labels):
            return self
        if int(labels.max()).bit_length() > 30:
            return self
        for (left, right), (axes, _) in self.arc_joints.items():
            if left not in self.names or right not in self.names:
                return self
            if group[self.names.index(left)] == group[self.names.index(right)] and any(
                joint in active for joint, _, _ in axes
            ):
                return self
        subset = copy(self)
        subset.model = copy(model)
        subset.data = cast("_Data", _mj.MjData(subset.model))
        subset._query_key, subset._previous_query = None, None
        mutable = subset.model
        bits = (1 << int(labels.max()).bit_length()) - 1
        # MuJoCo accepts (A.type & B.affinity) | (B.type & A.affinity).
        # Label and its bitwise complement therefore admit exactly unequal labels.
        mutable.geom_contype[:] = labels
        mutable.geom_conaffinity[:] = bits ^ labels
        for i in range(model.nbody):
            geoms = model.geom_bodyid == i
            mutable.body_contype[i] = np.bitwise_or.reduce(mutable.geom_contype[geoms], initial=0)
            mutable.body_conaffinity[i] = np.bitwise_or.reduce(
                mutable.geom_conaffinity[geoms], initial=0
            )
        return subset

    def _contacts(self, q_full: Floats) -> dict[tuple[int, int], _Contact]:
        # A trial and its Jacobian query the identical pose. Rejected LM steps
        # also return to the preceding pose. Keep both native narrow-phase
        # workspaces, including contact frames and derivative data, verbatim.
        if q_full.shape != self.permutation.shape:
            raise ValueError("CAD pose must provide one scalar per physical joint")
        pose = np.asarray(q_full, dtype=np.float64)
        key = pose.tobytes()
        if key != self._query_key:
            previous = self._previous_query
            current = None if self._query_key is None else (self._query_key, self.data)
            data = self.data
            if previous is not None:
                data = previous[1]
            elif current is not None:
                data = cast("_Data", _mj.MjData(self.model))
            if previous is None or previous[0] != key:
                # A failed query may overwrite the recycled buffer. Publish
                # its key only after the native computation succeeds.
                self._previous_query = None
                data.qpos[self.permutation] = pose
                _mj.mj_kinematics(self.model, data)
                _mj.mj_comPos(self.model, data)
                _mj.mj_collision(self.model, data)
            self.data, self._query_key, self._previous_query = data, key, current
        result: dict[tuple[int, int], _Contact] = {}
        for contact in self.data.contact:
            pair = (min(contact.geom1, contact.geom2), max(contact.geom1, contact.geom2))
            old = result.get(pair)
            if old is None or contact.dist < old.dist:
                result[pair] = contact
        return result

    def map_queries[T](self, count: int, operation: Callable[[CollisionModel, int], T]) -> list[T]:
        """Keep independent CAD queries ordered, with private MuJoCo data per worker."""
        workers = min(8, os.cpu_count() or 1, (count + 3) // 4)
        if count < 16 or workers < 2:
            return [operation(self, i) for i in range(count)]
        width = (count + workers - 1) // workers

        def check_chunk(start: int) -> list[T]:
            worker = copy(self)
            worker.data = cast("_Data", _mj.MjData(self.model))
            worker._query_key, worker._previous_query = None, None
            return [operation(worker, i) for i in range(start, min(count, start + width))]

        with ThreadPoolExecutor(max_workers=workers) as pool:
            return [
                value for chunk in pool.map(check_chunk, range(0, count, width)) for value in chunk
            ]

    def batch_at(self, rows: Sequence[Floats]) -> list[Contacts]:
        """Use the full contact query for each row, including all neutral and bearing allowances."""
        return self.map_queries(len(rows), lambda model, i: model.at(rows[i]))

    def at(
        self, q_full: Floats, *, gradient: bool = False, penetration_only: bool = False
    ) -> Contacts:
        contacts = self._contacts(q_full)
        maximum, excess, unallowed = 0.0, 0.0, 0.0
        worst: Pair = ("", "")
        blocked = np.zeros(self.model.nv, dtype=np.bool_)
        residuals: list[float] = []
        gradients: list[Floats] = []
        for geoms, contact in contacts.items():
            pair = self._pair(geoms)
            depth = max(0.0, -contact.dist)
            maximum = max(maximum, depth)
            excess = max(excess, depth - self.geom_neutral.get(geoms, 0.0))
            if pair in self.allowed:
                continue
            cap = self.geom_bearing_cap.get(geoms, 0.0)
            arc_entry = self.arc_joints.get(pair)
            if arc_entry is not None:
                axes, certified_depth = arc_entry
                # This evidence covers only the original CAD, one named rotary pair,
                # and its certified angle interval. Keep absolute penetration unchanged.
                if all(lo <= self.data.qpos[index] <= hi for index, lo, hi in axes):
                    cap = max(cap, certified_depth)
            if depth > cap:
                if depth > unallowed:
                    unallowed, worst = depth, pair
                if depth > 1e-6:
                    first, second = (
                        self.model.geom_bodyid[contact.geom1],
                        self.model.geom_bodyid[contact.geom2],
                    )
                    blocked |= self.ancestor[first] != self.ancestor[second]
            padding = 0.0 if penetration_only else self.margin_m
            residual = (-cap if cap > 0 else padding) - contact.dist
            if residual <= 0:
                continue
            residuals.append(residual)
            if gradient:
                a, b = (
                    int(self.model.geom_bodyid[contact.geom1]),
                    int(self.model.geom_bodyid[contact.geom2]),
                )
                normal = contact.frame[:3]
                dot = np.cross(self.data.xaxis, contact.pos - self.data.xanchor) @ normal
                dot[self.slide] = self.data.xaxis[self.slide] @ normal
                gradients.append((dot * (self.ancestor[a] - self.ancestor[b]))[self.permutation])
        return Contacts(
            maximum,
            excess,
            unallowed,
            worst,
            np.asarray(residuals, dtype=np.float64),
            np.asarray(gradients, dtype=np.float64).reshape(-1, len(self.permutation)),
            blocked[self.permutation],
        )
