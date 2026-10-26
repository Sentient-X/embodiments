"""Identify one built robot's dynamics from its motors' torque: held poses, then excitation.

Joint torque is linear in each link's ten mass properties (mass, first moment ``m c``, the
inertia tensor about the link origin; :class:`~sx_embodiments.calibration.LinkInertia`) and in
each joint's friction (Coulomb with sign ``tanh(q̇ / v_s)``, viscous, reflected rotor
inertia; :class:`~sx_embodiments.calibration.JointFriction`):

    tau = Y(q, q̇, q̈) · theta

:meth:`RigidBodyTree.regressor` builds ``Y`` over the description's whole joint tree, read as
``sx_embodiment_drivers::Tree`` reads it (fixed joints carry their child, a mimic joint moves
with its leader), so ``Y · theta`` is the Rust ``Dynamics::inverse`` for the same parameters;
the crate's ``dynamics-parity.json`` fixture holds both equal to MuJoCo's ``mj_inverse``.

Identification has two experiments a person runs on the robot (the runbook is
``docs/DYNAMICS-IDENTIFICATION.md``):

1. **Held poses** (:func:`hold_poses`): about twenty poses inside the joint limits and free of
   collision, chosen greedily so gravity's regressor is as well conditioned as the poses
   allow. Each is approached from below and from above and held; the mean of the two
   steady-state torques cancels Coulomb friction, which at rest takes any value inside its band.
   These rows fix gravity: every link's mass and first moment as far as torque can see them.
2. **Excitation** (:func:`excitation`): a finite Fourier series per joint, starting and ending
   at rest, inside position, speed, acceleration and torque limits and free of collision,
   the best-conditioned of many random feasible candidates. These rows fix inertia and friction.

:func:`fit` solves both together as weighted least squares (each joint weighted by its own
residual spread), with a weak prior pulling every parameter torque cannot see back to the
description's estimate, bounds keeping masses and friction non-negative, and every link
checked for physical consistency; a link that fails is held harder to its prior and refitted.
Only combinations of parameters reach torque (the base parameters); the report gives their
count and the regressor's condition number. :meth:`DynamicsFit.calibration` seals the result
as a :class:`~sx_embodiments.calibration.DynamicsCalibration` for the robot instance to select.
"""

from __future__ import annotations

import importlib
import math
import xml.etree.ElementTree as ET
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Final, cast

import numpy as np
from numpy.typing import NDArray
from sx_contracts import Sha256Digest

from ..calibration import (
    CalibrationMethod,
    DynamicsCalibration,
    JointFriction,
    LinkInertia,
    Measurement,
    Residual,
    ResidualUnit,
    UnitSerial,
    is_physical_inertia,
)
from ..kinematics import JointType, Kinematics
from .targets import CalibrationSolveError

Array = NDArray[np.float64]
Index = NDArray[np.intp]
#: SciPy ships no stubs; the two entry points used are pinned by these shapes.
_Solver = Callable[..., Any]


def _scipy(module: str, name: str) -> _Solver:
    return cast(_Solver, getattr(importlib.import_module(module), name))


#: Standard gravity, m/s², and gravity in the root frame of a body standing root-z-up.
STANDARD_GRAVITY: Final = 9.80665
UPRIGHT: Final = (0.0, 0.0, -STANDARD_GRAVITY)
#: The speed at which Coulomb friction reaches tanh(1) of its magnitude.
STICTION_VELOCITY: Final = 0.02
#: Singular values below this share of the largest count as directions torque cannot see.
RANK_TOLERANCE: Final = 1e-6
PER_LINK: Final = 10
PER_JOINT: Final = 3
#: How far, in each parameter's own unit, the prior lets a parameter wander per unit of
#: weighted torque residual: masses by about a kilogram, first moments by 0.05 kg·m, inertia
#: by 0.005 kg·m², friction by 0.5 N·m (s/rad), armature by 0.05 kg·m².
LINK_SCALE: Final = (1.0, 0.05, 0.05, 0.05, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005)
JOINT_SCALE: Final = (0.5, 0.5, 0.05)


@dataclass(frozen=True, slots=True, eq=False)
class _Joint:
    kind: JointType
    parent: int
    child: int
    origin: Array
    axis: Array
    leader: int | None
    multiplier: float
    offset: float


def _skew(v: Array) -> Array:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def _tensor_map(v: Array) -> Array:
    """``L(v)`` with ``I v = L(v) [xx, xy, xz, yy, yz, zz]``."""
    x, y, z = v
    return np.array([[x, y, z, 0.0, 0.0, 0.0], [0.0, x, 0.0, y, z, 0.0], [0.0, 0.0, x, 0.0, y, z]])


def _about(axis: Array, angle: float) -> Array:
    x, y, z = axis
    s, c = math.sin(angle), math.cos(angle)
    t = 1.0 - c
    return np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ]
    )


def link_inertia_from_urdf(element: ET.Element | None) -> tuple[float, ...]:
    """A URDF ``<inertial>`` as the ten parameters about the link origin, on its axes."""
    if element is None:
        return (0.0,) * PER_LINK
    origin = element.find("origin")
    xyz = np.array(
        [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
    )
    rpy = [
        float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()
    ]
    mass_element = element.find("mass")
    mass = float(mass_element.get("value", "0")) if mass_element is not None else 0.0
    tensor = element.find("inertia")
    keys = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
    xx, xy, xz, yy, yz, zz = (
        (float(tensor.get(key, "0")) for key in keys) if tensor is not None else (0.0,) * 6
    )
    turn = (
        _about(np.array([0.0, 0.0, 1.0]), rpy[2])
        @ _about(np.array([0.0, 1.0, 0.0]), rpy[1])
        @ _about(np.array([1.0, 0.0, 0.0]), rpy[0])
    )
    at_centre = turn @ np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]) @ turn.T
    about_origin = at_centre + mass * (float(xyz @ xyz) * np.eye(3) - np.outer(xyz, xyz))
    first = mass * xyz
    return (
        mass,
        float(first[0]),
        float(first[1]),
        float(first[2]),
        float(about_origin[0, 0]),
        float(about_origin[0, 1]),
        float(about_origin[0, 2]),
        float(about_origin[1, 1]),
        float(about_origin[1, 2]),
        float(about_origin[2, 2]),
    )


@dataclass(frozen=True, slots=True, eq=False)
class RigidBodyTree:
    """A description's joint tree with its design inertials, ready to regress torque.

    ``axes`` are the independent moving joints in description order, the order
    ``sx_embodiment_drivers::Tree::joints`` lists them in; every vector here is indexed by them.
    """

    links: tuple[str, ...]
    axes: tuple[str, ...]
    limits: Mapping[str, tuple[float, float] | None]
    root: int
    joints: tuple[_Joint, ...]
    prior: Array

    @classmethod
    def of(cls, urdf: bytes) -> RigidBodyTree:
        kinematics = Kinematics.of(urdf)
        root_element = ET.fromstring(urdf)  # nosec B314 - registry-owned description bytes
        links = tuple(
            name for link in root_element.findall("link") if (name := link.get("name")) is not None
        )
        prior = np.array(
            [
                value
                for link in root_element.findall("link")
                for value in link_inertia_from_urdf(link.find("inertial"))
            ],
            dtype=np.float64,
        )
        axes = kinematics.independent
        limits: dict[str, tuple[float, float] | None] = {}
        for element in root_element.findall("joint"):
            name = element.get("name", "")
            if name in axes:
                limit = element.find("limit")
                kind = element.get("type")
                limits[name] = (
                    (float(limit.get("lower", "0")), float(limit.get("upper", "0")))
                    if kind in ("revolute", "prismatic") and limit is not None
                    else None
                )
        index = {name: i for i, name in enumerate(links)}
        children: dict[str, list[str]] = {}
        for joint in kinematics.joints.values():
            children.setdefault(joint.parent, []).append(joint.name)
        roots = [link for link in links if link not in kinematics.parent_joint]
        if len(roots) != 1:
            raise CalibrationSolveError("a description has one root link")
        ordered: list[_Joint] = []
        frontier = [roots[0]]
        while frontier:
            link = frontier.pop()
            for name in children.get(link, []):
                joint = kinematics.joints[name]
                frontier.append(joint.child)
                leader, multiplier, offset = name, 1.0, 0.0
                while (mimic := kinematics.joints[leader].mimic) is not None:
                    offset = multiplier * mimic.offset + offset
                    multiplier *= mimic.multiplier
                    leader = mimic.leader
                moving = joint.type in (
                    JointType.REVOLUTE,
                    JointType.CONTINUOUS,
                    JointType.PRISMATIC,
                )
                ordered.append(
                    _Joint(
                        kind=joint.type if moving else JointType.FIXED,
                        parent=index[joint.parent],
                        child=index[joint.child],
                        origin=np.array(joint.origin, dtype=np.float64).reshape(4, 4),
                        axis=np.array(joint.axis, dtype=np.float64),
                        leader=axes.index(leader) if moving and leader in axes else None,
                        multiplier=multiplier,
                        offset=offset,
                    )
                )
        return cls(links, axes, limits, index[roots[0]], tuple(ordered), prior)

    @property
    def parameter_count(self) -> int:
        return PER_LINK * len(self.links) + PER_JOINT * len(self.axes)

    def parameters(self, calibration: DynamicsCalibration | None = None) -> Array:
        """``theta``: the design inertials and no friction, with ``calibration`` applied."""
        theta = np.concatenate([self.prior, np.zeros(PER_JOINT * len(self.axes))])
        if calibration is None:
            return theta
        for item in calibration.links:
            at = PER_LINK * self.links.index(item.link)
            theta[at : at + PER_LINK] = (item.mass, *item.first_moment, *item.inertia)
        base = PER_LINK * len(self.links)
        for item in calibration.joints:
            at = base + PER_JOINT * self.axes.index(item.joint)
            theta[at : at + PER_JOINT] = (item.coulomb, item.viscous, item.armature)
        return theta

    def subtree_links(self, joints: Sequence[str]) -> tuple[int, ...]:
        """Every link with mass a torque on ``joints`` can feel: the subtrees below them. A
        link the description gives no mass is a frame (a tool point, a camera's optical
        frame), not a body, and is never identified."""
        below: set[int] = set()
        for joint in self.joints:
            if joint.leader is not None and self.axes[joint.leader] in joints:
                below.add(joint.child)
        grew = True
        while grew:
            grew = False
            for joint in self.joints:
                if joint.parent in below and joint.child not in below:
                    below.add(joint.child)
                    grew = True
        return tuple(sorted(link for link in below if self.prior[PER_LINK * link] > 0.0))

    def regressor(
        self,
        position: Sequence[float] | Array,
        velocity: Sequence[float] | Array,
        acceleration: Sequence[float] | Array,
        gravity: Sequence[float] = UPRIGHT,
        stiction_velocity: float = STICTION_VELOCITY,
    ) -> Array:
        """``Y`` with ``tau = Y theta``: one row per axis, :attr:`parameter_count` columns."""
        q = np.asarray(position, dtype=np.float64)
        qd = np.asarray(velocity, dtype=np.float64)
        qdd = np.asarray(acceleration, dtype=np.float64)
        n_links, n_axes = len(self.links), len(self.axes)
        if not (q.shape == qd.shape == qdd.shape == (n_axes,)):
            raise CalibrationSolveError(f"expected {n_axes} positions, velocities, accelerations")
        spin = np.zeros((n_links, 3))
        spin_rate = np.zeros((n_links, 3))
        accel = np.zeros((n_links, 3))
        accel[self.root] = -np.asarray(gravity, dtype=np.float64)
        placed: list[tuple[Array, Array]] = []
        for joint in self.joints:
            if joint.leader is None:
                x = xd = xdd = 0.0
            else:
                x = joint.multiplier * q[joint.leader] + joint.offset
                xd = joint.multiplier * qd[joint.leader]
                xdd = joint.multiplier * qdd[joint.leader]
            turn = joint.origin[:3, :3]
            offset = joint.origin[:3, 3].copy()
            if joint.kind in (JointType.REVOLUTE, JointType.CONTINUOUS):
                turn = turn @ _about(joint.axis, x)
            elif joint.kind is JointType.PRISMATIC:
                offset = offset + turn @ (joint.axis * x)
            w, dw, a = spin[joint.parent], spin_rate[joint.parent], accel[joint.parent]
            # np.cross of two 3-vectors is a 3-vector; this numpy's stubs leave it unknown.
            centripetal = cast(Array, np.cross(w, cast(Array, np.cross(w, offset))))
            carried: Array = turn.T @ (a + cast(Array, np.cross(dw, offset)) + centripetal)
            w_child, dw_child = turn.T @ w, turn.T @ dw
            z = joint.axis
            if joint.kind in (JointType.REVOLUTE, JointType.CONTINUOUS):
                spin[joint.child] = w_child + z * xd
                spin_rate[joint.child] = dw_child + z * xdd + np.cross(w_child, z * xd)
                accel[joint.child] = carried
            elif joint.kind is JointType.PRISMATIC:
                spin[joint.child], spin_rate[joint.child] = w_child, dw_child
                accel[joint.child] = carried + 2.0 * xd * np.cross(w_child, z) + z * xdd
            else:
                spin[joint.child], spin_rate[joint.child] = w_child, dw_child
                accel[joint.child] = carried
            placed.append((turn, offset))
        columns = self.parameter_count
        wrench = np.zeros((n_links, 6, columns))
        for link in range(n_links):
            w, dw, a = spin[link], spin_rate[link], accel[link]
            block = np.zeros((6, PER_LINK))
            block[0:3, 0] = a
            block[0:3, 1:4] = _skew(dw) + _skew(w) @ _skew(w)
            block[3:6, 1:4] = -_skew(a)
            block[3:6, 4:10] = _tensor_map(dw) + _skew(w) @ _tensor_map(w)
            wrench[link, :, PER_LINK * link : PER_LINK * (link + 1)] = block
        rows = np.zeros((n_axes, columns))
        for joint, (turn, offset) in zip(reversed(self.joints), reversed(placed), strict=True):
            child = wrench[joint.child]
            if joint.leader is not None:
                share = joint.axis @ (
                    child[0:3] if joint.kind is JointType.PRISMATIC else child[3:6]
                )
                rows[joint.leader] += joint.multiplier * share
            force = turn @ child[0:3]
            moment = turn @ child[3:6] + _skew(offset) @ force
            wrench[joint.parent, 0:3] += force
            wrench[joint.parent, 3:6] += moment
        base = PER_LINK * n_links
        for i in range(n_axes):
            rows[i, base + PER_JOINT * i] = math.tanh(qd[i] / stiction_velocity)
            rows[i, base + PER_JOINT * i + 1] = qd[i]
            rows[i, base + PER_JOINT * i + 2] = qdd[i]
        return rows

    def inverse(
        self,
        theta: Array,
        position: Sequence[float] | Array,
        velocity: Sequence[float] | Array,
        acceleration: Sequence[float] | Array,
        gravity: Sequence[float] = UPRIGHT,
        stiction_velocity: float = STICTION_VELOCITY,
    ) -> Array:
        """``Y theta``: the torque each axis needs."""
        return self.regressor(position, velocity, acceleration, gravity, stiction_velocity) @ theta

    def vector(self, positions: Mapping[str, float]) -> Array:
        """A per-axis vector from named values; an axis not named is zero."""
        return np.array([positions.get(axis, 0.0) for axis in self.axes], dtype=np.float64)


def _scales(tree: RigidBodyTree) -> Array:
    return np.concatenate(
        [
            np.tile(np.array(LINK_SCALE), len(tree.links)),
            np.tile(np.array(JOINT_SCALE), len(tree.axes)),
        ]
    )


def base_condition(rows: Array, scales: Array) -> tuple[int, float]:
    """The number of base parameters (independent column combinations) of a stacked
    regressor, and the condition number of its column-scaled base part."""
    scaled = rows * scales
    singular = np.linalg.svd(scaled, compute_uv=False)
    if singular.size == 0 or singular[0] == 0.0:
        return 0, math.inf
    rank = int(np.count_nonzero(singular > singular[0] * RANK_TOLERANCE))
    return rank, float(singular[0] / singular[rank - 1])


def _columns(tree: RigidBodyTree, joints: Sequence[str]) -> tuple[Index, Index]:
    """Rows (axis indices) of ``joints`` and the columns their torque depends on."""
    rows = np.array([tree.axes.index(joint) for joint in joints], dtype=np.intp)
    links = tree.subtree_links(joints)
    base = PER_LINK * len(tree.links)
    columns = [PER_LINK * link + k for link in links for k in range(PER_LINK)]
    columns += [base + PER_JOINT * int(i) + k for i in rows for k in range(PER_JOINT)]
    return rows, np.array(columns, dtype=np.intp)


Admissible = Callable[[Mapping[str, float]], bool]


def hold_poses(
    tree: RigidBodyTree,
    joints: Sequence[str],
    rest: Mapping[str, float],
    count: int,
    admissible: Admissible,
    *,
    seed: int = 0,
    margin: float = 0.15,
    candidates: int = 600,
    box: Mapping[str, tuple[float, float]] | None = None,
    gravity: Sequence[float] = UPRIGHT,
) -> tuple[dict[str, float], ...]:
    """``count`` poses of ``joints`` (every other axis at ``rest``) inside their limits less
    ``margin`` radians (and inside ``box`` where given), each ``admissible`` (collision-free),
    chosen greedily to maximise the smallest singular value of gravity's stacked regressor."""
    rng = np.random.default_rng(seed)
    rows, columns = _columns(tree, joints)
    scales = _scales(tree)[columns]
    pool: list[tuple[dict[str, float], Array]] = []
    for _ in range(candidates):
        pose = dict(rest)
        for joint in joints:
            lower, upper = _range(tree, joint, margin, box)
            pose[joint] = float(rng.uniform(lower, upper))
        if not admissible(pose):
            continue
        q = tree.vector(pose)
        y = tree.regressor(q, np.zeros_like(q), np.zeros_like(q), gravity)[np.ix_(rows, columns)]
        pool.append((pose, y * scales))
    if len(pool) < count:
        raise CalibrationSolveError(
            f"only {len(pool)} of {candidates} candidate poses are admissible; need {count}"
        )
    chosen: list[int] = []
    stacked = np.zeros((0, len(columns)))
    for _ in range(count):
        best, best_score = -1, -math.inf
        for i, (_, y) in enumerate(pool):
            if i in chosen:
                continue
            singular = np.linalg.svd(np.vstack([stacked, y]), compute_uv=False)
            nonzero = singular[singular > singular[0] * RANK_TOLERANCE]
            score = float(np.sum(np.log(nonzero)))
            if score > best_score:
                best, best_score = i, score
        chosen.append(best)
        stacked = np.vstack([stacked, pool[best][1]])
    return tuple(pool[i][0] for i in chosen)


def _range(
    tree: RigidBodyTree,
    joint: str,
    margin: float,
    box: Mapping[str, tuple[float, float]] | None,
) -> tuple[float, float]:
    limit = tree.limits.get(joint) or (-math.pi, math.pi)
    lower, upper = limit[0] + margin, limit[1] - margin
    if box is not None and joint in box:
        lower, upper = max(lower, box[joint][0]), min(upper, box[joint][1])
    if lower >= upper:
        raise CalibrationSolveError(f"{joint} has no room inside its limits less the margin")
    return lower, upper


@dataclass(frozen=True, slots=True, eq=False)
class Excitation:
    """A finite Fourier series per joint, periodic, at rest at its start and end:

    ``q(t) = q0 + sum_k a_k/(w k) sin(w k t) - b_k/(w k) cos(w k t)``, ``w = 2 pi / period``.
    """

    joints: tuple[str, ...]
    period: float
    centre: Array
    sine: Array
    cosine: Array

    def at(self, time: Array) -> tuple[Array, Array, Array]:
        """Positions, velocities and accelerations, ``(len(time), len(joints))``."""
        w = 2.0 * math.pi / self.period
        k = np.arange(1, self.sine.shape[1] + 1, dtype=np.float64)
        phase = w * np.outer(time, k)
        s, c = np.sin(phase), np.cos(phase)
        q = self.centre + s @ (self.sine / (w * k)).T - c @ (self.cosine / (w * k)).T
        qd = c @ self.sine.T + s @ self.cosine.T
        qdd = (-s @ (self.sine * w * k).T) + c @ (self.cosine * w * k).T
        return q, qd, qdd

    def to_dict(self) -> dict[str, object]:
        return {
            "joints": list(self.joints),
            "period_s": self.period,
            "centre_rad": self.centre.tolist(),
            "sine": self.sine.tolist(),
            "cosine": self.cosine.tolist(),
        }


@dataclass(frozen=True, slots=True)
class ExcitationLimits:
    """What the trajectory may ask of every excited joint."""

    speed: float  # rad/s
    acceleration: float  # rad/s²
    #: Per joint, the largest torque the trajectory may need at the prior parameters, N·m.
    torque: Mapping[str, float]
    margin: float = 0.15  # rad inside each joint limit


def excitation(
    tree: RigidBodyTree,
    joints: Sequence[str],
    rest: Mapping[str, float],
    limits: ExcitationLimits,
    admissible: Admissible,
    *,
    period: float = 10.0,
    harmonics: int = 5,
    candidates: int = 40,
    seed: int = 0,
    box: Mapping[str, tuple[float, float]] | None = None,
    gravity: Sequence[float] = UPRIGHT,
    rate_hz: float = 20.0,
) -> tuple[Excitation, float]:
    """The best-conditioned of ``candidates`` random feasible trajectories, and its condition
    number (column-scaled base regressor over one period sampled at ``rate_hz``)."""
    rng = np.random.default_rng(seed)
    rows, columns = _columns(tree, joints)
    scales = _scales(tree)[columns]
    theta = tree.parameters()
    time = np.arange(0.0, period, 1.0 / rate_hz, dtype=np.float64)
    fine = np.arange(0.0, period, 0.002, dtype=np.float64)
    k = np.arange(1, harmonics + 1)
    best: tuple[Excitation, float] | None = None
    for _ in range(candidates):
        sine = rng.normal(size=(len(joints), harmonics)) / k
        cosine = rng.normal(size=(len(joints), harmonics)) / k
        # At rest at t = 0: q̇(0) = sum a_k = 0 and q̈(0) = w sum k b_k = 0.
        sine -= sine.mean(axis=1, keepdims=True)
        cosine -= np.outer(cosine @ k, k) / float(k @ k)
        trial = Excitation(tuple(joints), period, np.zeros(len(joints)), sine, cosine)
        # Bounds are met on a fine grid; the regressor and collisions are sampled at rate_hz.
        q, qd, qdd = trial.at(fine)
        centre = np.zeros(len(joints))
        for j, joint in enumerate(joints):
            lower, upper = _range(tree, joint, limits.margin, box)
            span = float(np.ptp(q[:, j]))
            factor = min(
                (upper - lower) / span if span > 0 else math.inf,
                limits.speed / max(float(np.max(np.abs(qd[:, j]))), 1e-12),
                limits.acceleration / max(float(np.max(np.abs(qdd[:, j]))), 1e-12),
            )
            sine[j] *= factor
            cosine[j] *= factor
            middle = (float(np.max(q[:, j])) + float(np.min(q[:, j]))) / 2.0 * factor
            centre[j] = (lower + upper) / 2.0 - middle
        trial = Excitation(tuple(joints), period, centre, sine, cosine)
        q, qd, qdd = trial.at(time)
        stacked: list[Array] = []
        feasible = True
        for i in range(len(time)):
            pose = dict(rest)
            pose.update({joint: float(q[i, j]) for j, joint in enumerate(joints)})
            if not admissible(pose):
                feasible = False
                break
            y = tree.regressor(
                tree.vector(pose),
                tree.vector(dict(zip(joints, qd[i].tolist(), strict=True))),
                tree.vector(dict(zip(joints, qdd[i].tolist(), strict=True))),
                gravity,
            )
            torque = y @ theta
            if any(abs(torque[tree.axes.index(j)]) > limits.torque[j] for j in joints):
                feasible = False
                break
            stacked.append(y[np.ix_(rows, columns)])
        if not feasible:
            continue
        _, condition = base_condition(np.vstack(stacked), scales)
        if best is None or condition < best[1]:
            best = (trial, condition)
    if best is None:
        raise CalibrationSolveError(f"none of {candidates} candidate trajectories is feasible")
    return best


@dataclass(frozen=True, slots=True)
class Hold:
    """One held pose's steady state: mean position and mean motor torque per joint, the pose
    approached once from below and once from above and the two holds averaged."""

    position: Mapping[str, float]
    torque: Mapping[str, float]


@dataclass(frozen=True, slots=True, eq=False)
class Trajectory:
    """A recorded excitation: per-sample joint positions, velocities, accelerations (from the
    zero-phase filtered velocity) and motor torque, each ``(samples, len(joints))``."""

    joints: tuple[str, ...]
    position: Array
    velocity: Array
    acceleration: Array
    torque: Array


def differentiate(
    time: Array, position: Array, velocity: Array, torque: Array, cutoff_hz: float
) -> tuple[Array, Array, Array, Array]:
    """Zero-phase low-pass of position, velocity and torque (4th-order Butterworth) and the
    acceleration as the central difference of the filtered velocity."""
    butter, filtfilt = _scipy("scipy.signal", "butter"), _scipy("scipy.signal", "filtfilt")
    rate = 1.0 / float(np.median(np.diff(time)))
    b, a = cast(tuple[Array, Array], butter(4, cutoff_hz / (rate / 2.0)))

    def smooth(values: Array) -> Array:
        return np.asarray(filtfilt(b, a, values, axis=0), dtype=np.float64)

    q, qd, tau = smooth(position), smooth(velocity), smooth(torque)
    qdd = np.gradient(qd, time, axis=0)
    return q, qd, qdd, tau


@dataclass(frozen=True, slots=True, eq=False)
class DynamicsFit:
    """What :func:`fit` found, and how well."""

    tree: RigidBodyTree
    joints: tuple[str, ...]
    theta: Array
    #: Links whose parameters the fit moved, by index into ``tree.links``.
    links: tuple[int, ...]
    stiction_velocity: float
    #: Root-mean-square torque residual per joint, N·m, on the holds and the trajectory.
    hold_residual: Mapping[str, float]
    trajectory_residual: Mapping[str, float]
    base_parameters: int
    condition: float
    #: Links that stayed unphysical after refitting and were returned to the design estimate.
    reverted: tuple[str, ...]

    @property
    def residual(self) -> float:
        """The worst joint's RMS torque residual over both experiments."""
        values = [*self.hold_residual.values(), *self.trajectory_residual.values()]
        return max(values) if values else 0.0

    def calibration(
        self,
        description_sha256: str,
        units: Sequence[UnitSerial],
        measured_at: datetime,
        evidence: Sequence[object] = (),
    ) -> DynamicsCalibration:
        """The sealed record a robot instance selects."""
        from ..calibration import Evidence

        links = tuple(
            LinkInertia(
                link=self.tree.links[link],
                mass=float(self.theta[PER_LINK * link]),
                first_moment=_triple(self.theta[PER_LINK * link + 1 : PER_LINK * link + 4]),
                inertia=_six(self.theta[PER_LINK * link + 4 : PER_LINK * link + 10]),
            )
            for link in self.links
        )
        base = PER_LINK * len(self.tree.links)
        joints = tuple(
            JointFriction(
                joint=joint,
                coulomb=float(self.theta[base + PER_JOINT * self.tree.axes.index(joint)]),
                viscous=float(self.theta[base + PER_JOINT * self.tree.axes.index(joint) + 1]),
                armature=float(self.theta[base + PER_JOINT * self.tree.axes.index(joint) + 2]),
            )
            for joint in self.joints
        )
        return DynamicsCalibration(
            description_sha256=Sha256Digest(description_sha256),
            stiction_velocity=self.stiction_velocity,
            links=tuple(sorted(links, key=lambda item: item.link)),
            joints=tuple(sorted(joints, key=lambda item: item.joint)),
            units=tuple(sorted(set(units))),
            measurement=Measurement(
                method=CalibrationMethod.DYNAMICS_IDENTIFICATION,
                measured_at=measured_at,
                residual=Residual(self.residual, ResidualUnit.NEWTON_METRE),
                evidence=tuple(item for item in evidence if isinstance(item, Evidence)),
            ),
        )


def _triple(values: Array) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


def _six(values: Array) -> tuple[float, float, float, float, float, float]:
    return tuple(float(v) for v in values)  # type: ignore[return-value]  # six by slicing


def fit(
    tree: RigidBodyTree,
    joints: Sequence[str],
    holds: Sequence[Hold],
    trajectories: Sequence[Trajectory],
    rest: Mapping[str, float],
    *,
    gravity: Sequence[float] = UPRIGHT,
    stiction_velocity: float = STICTION_VELOCITY,
    prior_weight: float = 1.0,
    rounds: int = 6,
) -> DynamicsFit:
    """Weighted least squares over the holds' and trajectories' rows, with the description's
    estimate as a weak prior, masses and friction bounded at zero, and every moved link checked
    physical; a link that is not is held ten times harder to its prior and the fit repeated."""
    rows, columns = _columns(tree, joints)
    scales = _scales(tree)
    prior = tree.parameters()
    hold_rows: list[Array] = []
    hold_torque: list[Array] = []
    for hold in holds:
        pose = dict(rest)
        pose.update(hold.position)
        q = tree.vector(pose)
        y = tree.regressor(q, np.zeros_like(q), np.zeros_like(q), gravity, stiction_velocity)
        hold_rows.append(y[rows])
        hold_torque.append(np.array([hold.torque[joint] for joint in joints]))
    traj_rows: list[Array] = []
    traj_torque: list[Array] = []
    for trajectory in trajectories:
        for i in range(trajectory.position.shape[0]):
            pose = dict(rest)
            pose.update(zip(trajectory.joints, trajectory.position[i].tolist(), strict=True))
            speeds = dict(zip(trajectory.joints, trajectory.velocity[i].tolist(), strict=True))
            rates = dict(zip(trajectory.joints, trajectory.acceleration[i].tolist(), strict=True))
            y = tree.regressor(
                tree.vector(pose),
                tree.vector(speeds),
                tree.vector(rates),
                gravity,
                stiction_velocity,
            )
            measured = dict(zip(trajectory.joints, trajectory.torque[i].tolist(), strict=True))
            traj_rows.append(y[rows])
            traj_torque.append(np.array([measured.get(joint, math.nan) for joint in joints]))
    if not hold_rows and not traj_rows:
        raise CalibrationSolveError("nothing to fit: no holds and no trajectory")
    blocks = hold_rows + traj_rows
    torque = np.concatenate(hold_torque + traj_torque)
    regressor = np.vstack(blocks)[:, columns]
    known = np.isfinite(torque)
    regressor, torque = regressor[known], torque[known]
    joint_of_row = np.tile(np.arange(len(joints)), len(blocks))[known]
    rank, condition = base_condition(regressor, scales[columns])
    prior_c = prior[columns]
    scale_c = scales[columns]
    lower = np.full(len(columns), -np.inf)
    upper = np.full(len(columns), np.inf)
    base = PER_LINK * len(tree.links)
    for c, column in enumerate(columns):
        if column < base and column % PER_LINK == 0:
            lower[c] = 0.0
        if column >= base:
            lower[c] = 0.0
    strength = np.full(len(columns), prior_weight)
    weights = np.ones(len(joints))
    theta_c = prior_c.copy()
    links = tree.subtree_links(joints)
    reverted: list[str] = []
    for round_ in range(rounds + 2):
        w = weights[joint_of_row]
        a = np.vstack([regressor * w[:, None], np.diag(np.sqrt(strength) / scale_c)])
        b = np.concatenate([torque * w, np.sqrt(strength) / scale_c * prior_c])
        solved = _scipy("scipy.optimize", "lsq_linear")(
            a, b, bounds=(lower, upper), method="bvls" if len(columns) < 400 else "trf"
        )  # pyright: ignore[reportUnknownVariableType]
        theta_c = np.asarray(solved.x, dtype=np.float64)  # pyright: ignore[reportUnknownMemberType]
        residual = torque - regressor @ theta_c
        if round_ == 0:
            # Second pass weighs each joint by the spread of its own residual.
            spread = np.array(
                [
                    math.sqrt(float(np.mean(residual[joint_of_row == j] ** 2)) + 1e-6)
                    for j in range(len(joints))
                ]
            )
            weights = 1.0 / spread
            continue
        bad: list[int] = []
        for link in links:
            at = [list(columns).index(PER_LINK * link + k) for k in range(PER_LINK)]
            values = theta_c[at]
            if not is_physical_inertia(
                float(values[0]), tuple(values[1:4].tolist()), tuple(values[4:10].tolist())
            ):
                bad.append(link)
                for i in at:
                    strength[i] *= 10.0
        if not bad:
            break
        if round_ == rounds + 1:
            for link in bad:
                at = [list(columns).index(PER_LINK * link + k) for k in range(PER_LINK)]
                theta_c[at] = prior_c[at]
                reverted.append(tree.links[link])
    theta = prior.copy()
    theta[columns] = theta_c
    residual = torque - regressor @ theta_c
    hold_known = known[: len(hold_rows) * len(joints)] if hold_rows else np.zeros(0, dtype=bool)
    n_hold = int(np.sum(hold_known))

    def rms(values: Array, owners: NDArray[np.int_]) -> dict[str, float]:
        return {
            joint: float(math.sqrt(float(np.mean(values[owners == j] ** 2))))
            for j, joint in enumerate(joints)
            if np.any(owners == j)
        }

    return DynamicsFit(
        tree=tree,
        joints=tuple(joints),
        theta=theta,
        links=links,
        stiction_velocity=stiction_velocity,
        hold_residual=rms(residual[:n_hold], joint_of_row[:n_hold]),
        trajectory_residual=rms(residual[n_hold:], joint_of_row[n_hold:]),
        base_parameters=rank,
        condition=condition,
        reverted=tuple(reverted),
    )
