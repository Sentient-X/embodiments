"""Offline fixed-base pose fitting to declared URDF links and observed human landmarks.

The source convention is OpenPose wrist then four landmarks per digit, in metres.
All returned errors are recomputed with FK of the final joint positions. A source
observation and a kinematically feasible robot fit are distinct pieces of evidence.
"""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from importlib import import_module
from itertools import product
from pathlib import Path
from typing import Protocol, cast

import numpy as np
from numpy.typing import NDArray
from sx_contracts.identity import JsonValue

from .kinematics import JointType, Kinematics, from_xyz_rpy

type Floats = NDArray[np.float64]
type Indices = NDArray[np.int64]


# MuJoCo's C-extension wheels ship no Python type declarations. Keep the
# unchecked extension boundary here; the numerical solver uses typed FK arrays.
class _Named(Protocol):
    id: int
    name: str


class _Model(Protocol):
    nv: int
    nq: int
    njnt: int
    body_jntadr: Indices
    body_jntnum: Indices
    body_parentid: Indices
    jnt_type: Indices
    jnt_limited: NDArray[np.bool_]
    jnt_range: Floats
    qpos0: Floats

    def joint(self, name: str | int) -> _Named: ...
    def body(self, name: str) -> _Named: ...


class _Data(Protocol):
    qpos: Floats
    xpos: Floats
    xmat: Floats
    xanchor: Floats
    xaxis: Floats


_mj = import_module("mujoco")


@dataclass(frozen=True)
class MappingProfile:
    """A side maps 21 anatomical indices to URDF link origins. Null skips a landmark."""

    hands: Mapping[str, tuple[str | None, ...]]
    camera_optical_link: str | None = None

    def __post_init__(self) -> None:
        if not self.hands or set(self.hands) - {"left", "right"}:
            raise ValueError("hands must map one or both anatomical sides")
        links: list[str] = []
        for side, sites in self.hands.items():
            if len(sites) != 21 or sites[0] is None:
                raise ValueError(f"{side} needs a wrist and 20 digit slots")
            for name in sites:
                if name is not None:
                    if not name.strip():
                        raise ValueError("each mapped landmark needs a URDF link name")
                    links.append(name)
        if self.camera_optical_link is not None:
            if not self.camera_optical_link.strip():
                raise ValueError("camera_optical_link must be a URDF optical link or null")
            links.append(self.camera_optical_link)
        if len(set(links)) != len(links):
            raise ValueError("mapped landmarks and optical frame must be distinct links")

    @classmethod
    def sentient_wuji(cls) -> MappingProfile:
        hands: dict[str, tuple[str | None, ...]] = {}
        for side in ("left", "right"):
            links = [f"{side}_palm_link"]
            for finger in range(1, 6):
                # Wuji thumb starts at the carpometacarpal pivot; other digits' two
                # nearly coincident base axes form the single anatomical MCP joint.
                base = "link1" if finger == 1 else "link2"
                for part in (base, "link3", "link4", "tip_link"):
                    links.append(f"{side}_finger{finger}_{part}")
            hands[side] = tuple(links)
        return cls(hands, "zed_left_optical")

    @classmethod
    def read(cls, path: Path) -> MappingProfile:
        document = cast("JsonValue", json.loads(path.read_bytes()))
        if not isinstance(document, dict) or set(document) - {"hands", "camera_optical_link"}:
            raise ValueError("profile expects hands and optional camera_optical_link")
        hands = document.get("hands")
        if not isinstance(hands, dict):
            raise ValueError("profile hands must map each side to an array")
        mapped: dict[str, tuple[str | None, ...]] = {}
        for side, values in hands.items():
            if not isinstance(values, list):
                raise ValueError("expected an array of link names")
            links: list[str | None] = []
            for name in values:
                if name is not None and not isinstance(name, str):
                    raise ValueError("profile landmark must name a link or be null")
                links.append(name)
            mapped[side] = tuple(links)
        camera = document.get("camera_optical_link")
        if camera is not None and not isinstance(camera, str):
            raise ValueError("expected an optical link name or null")
        return cls(mapped, camera)


@dataclass(frozen=True)
class Fit:
    time_ns: Indices
    joint_names: tuple[str, ...]
    q: Floats
    side_names: tuple[str, ...]
    observed: NDArray[np.bool_]
    wrist_error_m: Floats
    fingertip_error_m: Floats
    gaze_error_rad: Floats
    camera_rotation_error_rad: Floats
    robot_from_world: Floats
    landmark_error_m: Floats
    landmark_positions_robot_m: Floats

    def save(self, path: Path) -> None:
        with path.open("wb") as handle:
            np.savez_compressed(handle, **vars(self))


def _rigid(matrices: Floats) -> None:
    rotation = matrices[..., :3, :3]
    if (
        matrices.shape[-2:] != (4, 4)
        or not np.isfinite(matrices).all()
        or not np.allclose(matrices[..., 3, :], (0, 0, 0, 1), atol=1e-6)
        or not np.allclose(rotation.swapaxes(-1, -2) @ rotation, np.eye(3), atol=1e-5)
        or not np.allclose(np.linalg.det(rotation), 1.0, atol=1e-5)
    ):
        raise ValueError("frame registrations must be proper rigid transforms")


@dataclass(frozen=True)
class _Robot:
    model: _Model
    names: tuple[str, ...]
    lower: Floats
    upper: Floats
    velocity: Floats
    coupling: Floats | None
    offset: Floats

    @property
    def neutral(self) -> Floats:
        return np.clip(np.zeros(len(self.names)), self.lower, self.upper)


def _model(path: Path) -> _Robot:
    """Compile URDF for pure kinematics. Mesh resolution and inertial data are unnecessary."""
    source = path.read_bytes()
    tree = Kinematics.of(source)
    supported = {JointType.FIXED, JointType.REVOLUTE, JointType.PRISMATIC, JointType.CONTINUOUS}
    if any(j.type not in supported for j in tree.joints.values()):
        raise ValueError(
            "fixed-base IK expects scalar joint coordinates; free and planar roots need a placement"
        )
    root = ET.fromstring(source)  # nosec B314 - local, supplied robot description
    for item in root.findall("mujoco"):
        root.remove(item)
    for link in root.findall("link"):
        for child in list(link):
            link.remove(child)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass", value="1")
        ET.SubElement(inertial, "inertia", ixx="1", iyy="1", izz="1", ixy="0", ixz="0", iyz="0")
    ET.SubElement(ET.SubElement(root, "mujoco"), "compiler", fusestatic="false")
    model = cast("_Model", _mj.MjModel.from_xml_string(ET.tostring(root, encoding="unicode")))
    if not model.nv or model.nq != model.nv or model.njnt != model.nv:
        raise ValueError("URDF must compile to independent scalar coordinates")
    for j in tree.joints.values():
        if j.type in {JointType.REVOLUTE, JointType.PRISMATIC}:
            index = model.joint(j.name).id
            if (
                not model.jnt_limited[index]
                or model.jnt_range[index, 1] <= model.jnt_range[index, 0]
            ):
                raise ValueError(f"{j.name} needs finite independent URDF limits")
    velocities = np.empty(model.nv)
    for joint in root.findall("joint"):
        if joint.get("type") == "fixed":
            continue
        limit = joint.find("limit")
        name = joint.attrib["name"]
        velocity = float("nan") if limit is None else float(limit.get("velocity", "nan"))
        if not np.isfinite(velocity) or velocity <= 0:
            raise ValueError(f"{name} needs a finite positive URDF velocity limit")
        velocities[model.joint(name).id] = velocity
    native = tuple(model.joint(i).name for i in range(model.njnt))
    lo = np.where(model.jnt_limited, model.jnt_range[:, 0], -np.inf)
    hi = np.where(model.jnt_limited, model.jnt_range[:, 1], np.inf)
    if all(tree.joints[name].mimic is None for name in native):
        return _Robot(model, native, lo, hi, velocities, None, np.zeros(model.nv))

    # Followers give one affine equation per URDF joint. Propagate their limits
    # and velocity bounds to the independent coordinate, including opposite signs.
    names = tuple(name for name in native if tree.joints[name].mimic is None)
    columns = {name: i for i, name in enumerate(names)}
    lower, upper = np.full(len(names), -np.inf), np.full(len(names), np.inf)
    speed = np.full(len(names), np.inf)
    matrix = np.zeros((model.nv, len(names)))
    offsets = np.zeros(model.nv)
    for i, name in enumerate(native):
        joint = tree.joints[name]
        multiplier = 1.0
        while joint.mimic is not None:
            offsets[i] += multiplier * joint.mimic.offset
            multiplier *= joint.mimic.multiplier
            joint = tree.joints[joint.mimic.leader]
        if joint.name not in columns or not np.isfinite(multiplier) or not np.isfinite(offsets[i]):
            raise ValueError(f"invalid affine leader for {name}")
        if multiplier == 0:
            if not lo[i] <= offsets[i] <= hi[i]:
                raise ValueError(f"constant mimic {name} violates its bounds")
            continue
        j = columns[joint.name]
        matrix[i, j] = multiplier
        ends = (lo[i] - offsets[i]) / multiplier, (hi[i] - offsets[i]) / multiplier
        lower[j] = max(lower[j], min(ends))
        upper[j] = min(upper[j], max(ends))
        speed[j] = min(speed[j], velocities[i] / abs(multiplier))
    if np.any(lower > upper):
        raise ValueError("mimic and independent joint limits have no intersection")
    return _Robot(model, names, lower, upper, speed, matrix, offsets)


class _Sites:
    """Link-origin geometric Jacobian, in robot space, on declared kinematic ancestors."""

    def __init__(self, robot: _Robot, links: tuple[str, ...]) -> None:
        self.robot = robot
        self.model = model = robot.model
        self.data = cast("_Data", _mj.MjData(model))
        self.ids = np.array([model.body(name).id for name in links], dtype=np.int64)
        self.ancestor = np.zeros((len(links), model.nv), dtype=np.bool_)
        for i, body_id in enumerate(self.ids):
            parent = int(body_id)
            while parent > 0:
                first = int(model.body_jntadr[parent])
                self.ancestor[i, first : first + int(model.body_jntnum[parent])] = True
                parent = int(model.body_parentid[parent])
        self.slide = model.jnt_type == int(_mj.mjtJoint.mjJNT_SLIDE)

    def positions(self, q: Floats) -> Floats:
        self.data.qpos[:] = (
            q if self.robot.coupling is None else self.robot.coupling @ q + self.robot.offset
        )
        _mj.mj_kinematics(self.model, self.data)
        return self.data.xpos[self.ids]

    def jacobian(self, positions: Floats) -> Floats:
        vectors = positions[:, None, :] - self.data.xanchor[None, :, :]
        jacobian = np.cross(self.data.xaxis, vectors)
        jacobian[:, self.slide] = self.data.xaxis[self.slide]
        return np.asarray(
            (jacobian * self.ancestor[..., None]).transpose(0, 2, 1), dtype=np.float64
        )


@dataclass(frozen=True)
class _Targets:
    sites: Indices
    origins: Indices
    landmarks: Indices
    weights: Floats
    camera: int | None

    def values(
        self, points: Floats, camera_rotation: Floats, poses: Floats, sites: _Sites
    ) -> Floats:
        # Match the absolute wrist and digit shape separately. A difficult-to-reach wrist
        # must not cause the optimizer to unfold the fingers to compensate for reach error.
        actual = poses[self.sites].copy()
        targets = points[self.landmarks].copy()
        relative = self.sites != self.origins
        actual[relative] -= poses[self.origins[relative]]
        targets[relative] -= points[(self.landmarks[relative] // 21) * 21]
        values = ((actual - targets) * self.weights[:, None]).reshape(-1)
        if self.camera is not None:
            rotation = sites.data.xmat[sites.ids[self.camera]].reshape(3, 3)
            values = np.r_[
                values,
                0.16 * (rotation[:, 2] - camera_rotation[:, 2]),
                0.08 * (rotation[:, 1] - camera_rotation[:, 1]),
            ]
        return values

    def jacobian(self, poses: Floats, sites: _Sites) -> Floats:
        full = sites.jacobian(poses)
        jac = full[self.sites].copy()
        relative = self.sites != self.origins
        jac[relative] -= full[self.origins[relative]]
        jac = (jac * self.weights[:, None, None]).reshape(-1, sites.model.nv)
        if self.camera is not None:
            rotation = sites.data.xmat[sites.ids[self.camera]].reshape(3, 3)
            active = sites.ancestor[self.camera] & ~sites.slide
            for index, weight in ((2, 0.16), (1, 0.08)):
                camera_jac = np.cross(sites.data.xaxis, rotation[:, index]) * active[:, None]
                jac = np.concatenate((jac, weight * camera_jac.T))
        return jac if sites.robot.coupling is None else jac @ sites.robot.coupling


def _fit_frame(
    sites: _Sites,
    terms: _Targets,
    points: Floats,
    gaze: Floats,
    previous: Floats,
    temporal: float,
    lo: Floats,
    hi: Floats,
    iterations: int,
) -> Floats:
    """Box-constrained damped Gauss-Newton, warm-started at the previous source timestamp."""
    q = previous.copy()
    damping = 0.008
    for _ in range(iterations):
        poses = sites.positions(q)
        residual = terms.values(points, gaze, poses, sites)
        jac = terms.jacobian(poses, sites)
        gradient = jac.T @ residual + temporal**2 * (q - previous)
        # A joint at a hard stop may still leave it if the gradient points inward.
        free = ~(((q <= lo + 1e-8) & (gradient > 0)) | ((q >= hi - 1e-8) & (gradient < 0)))
        if not np.any(free):
            break
        active = jac[:, free]
        hessian = active.T @ active
        hessian.flat[:: len(hessian) + 1] += temporal**2 + damping**2
        step = np.linalg.solve(hessian, -gradient[free])
        step *= min(1.0, 0.35 / max(float(np.max(np.abs(step))), 1e-12))
        candidate = q.copy()
        candidate[free] = np.clip(q[free] + step, lo[free], hi[free])
        trial = terms.values(points, gaze, sites.positions(candidate), sites)
        before = residual @ residual + temporal**2 * np.sum((q - previous) ** 2)
        after = trial @ trial + temporal**2 * np.sum((candidate - previous) ** 2)
        if after < before:
            q = candidate
            damping = max(0.002, damping * 0.6)
            if before - after < 1e-9:
                break
        else:
            damping *= 3.0
            if damping > 1.0:
                break
    return q


def solve(
    urdf: Path,
    profile: MappingProfile,
    *,
    time_ns: Indices,
    hands_world_m: Mapping[str, Floats],
    world_from_camera: Floats,
    robot_from_world: Floats | None,
) -> Fit:
    """Fit on the native clock. Registration explicitly maps the input world to the URDF root."""
    times = np.asarray(time_ns)
    if (
        times.dtype != np.int64
        or times.ndim != 1
        or not len(times)
        or np.any(times[1:] <= times[:-1])
    ):
        raise ValueError("source time_ns must be strictly increasing int64")
    if robot_from_world is None:
        return _virtual_fit(
            urdf,
            profile,
            time_ns=times,
            hands_world_m=hands_world_m,
            world_from_camera=world_from_camera,
        )
    placement = np.asarray(robot_from_world, dtype=np.float64)
    cameras = np.asarray(world_from_camera, dtype=np.float64)
    if placement.shape != (4, 4) or cameras.shape != (len(times), 4, 4):
        raise ValueError("one registration and one camera transform per timestamp required")
    _rigid(placement)
    _rigid(cameras)
    sides = tuple(profile.hands)
    source = np.stack([np.asarray(hands_world_m[side], dtype=np.float64) for side in sides], axis=1)
    if source.shape != (len(times), len(sides), 21, 3) or np.isinf(source).any():
        raise ValueError("source landmarks must be N x 21 x 3 metres for each side")
    present = np.asarray(np.isfinite(source).all(axis=(2, 3)))
    absent = np.isnan(source).all(axis=(2, 3))
    if not np.all(present | absent):
        raise ValueError("a hand observation must have 21 finite landmarks or be entirely missing")
    if np.any(np.ptp(source[present], axis=1).max(axis=1) <= 1e-8):
        raise ValueError("degenerate hand observation")
    points = source @ placement[:3, :3].T + placement[:3, 3]
    camera_rotations = placement[:3, :3] @ cameras[:, :3, :3]
    robot = _model(urdf)
    mapped = [link for side in sides for link in profile.hands[side]]
    links = [name for name in mapped if name is not None]
    if profile.camera_optical_link is not None:
        links.append(profile.camera_optical_link)
    sites = _Sites(robot, tuple(links))
    q = robot.neutral
    seq = np.empty((len(times), len(robot.names)))
    errors = np.full((len(times), len(sides), 21), np.nan)
    actual = np.full(points.shape, np.nan)
    gaze_error = np.full(len(times), np.nan)
    head_error = np.full(len(times), np.nan)
    map_indices = np.array([i for i, name in enumerate(mapped) if name is not None], dtype=np.int64)
    target_cache: dict[tuple[bool, ...], _Targets] = {}
    camera = len(links) - 1 if profile.camera_optical_link is not None else None
    for i, mask in enumerate(present):
        key = tuple(bool(v) for v in mask)
        if key not in target_cache:
            observed = mask[map_indices // 21]
            ids = np.flatnonzero(observed)
            indices = map_indices[observed]
            origins = np.searchsorted(map_indices, indices // 21 * 21)
            digit = indices % 21
            weights = np.where(digit == 0, 2.5, np.where(digit % 4 == 0, 1.0, 0.45))
            target_cache[key] = _Targets(ids, origins, indices, weights, camera)
        terms = target_cache[key]
        first = i == 0
        dt = 0.0 if first else (int(times[i]) - int(times[i - 1])) * 1e-9
        # Penalize squared joint velocity on the native sampling interval. This weak
        # prior resolves redundancy while leaving positions in the capture's original clock.
        temporal = 0.0 if first else 0.0005 / max(dt, 1e-6)
        q = _fit_frame(
            sites,
            terms,
            points[i].reshape(-1, 3),
            camera_rotations[i],
            q,
            temporal,
            robot.lower if first else np.maximum(robot.lower, q - dt * robot.velocity),
            robot.upper if first else np.minimum(robot.upper, q + dt * robot.velocity),
            65 if first else 12,
        )
        seq[i] = q
    # A symmetric pass damps alternating IK branches without adding a frame of lag.
    # At most half of each sample comes from its neighbours' time-weighted line.
    # This convex combination preserves box bounds; adjacent velocities are also
    # convex combinations of the original three velocities on an irregular clock.
    # Keep observed/occluded transitions and recording gaps at their solved values.
    if len(times) >= 3:
        intervals = np.diff(times).astype(np.float64) * 1e-9
        before, after = intervals[:-1], intervals[1:]
        supported = (
            np.all(present[:-2] == present[1:-1], axis=1)
            & np.all(present[1:-1] == present[2:], axis=1)
            & (np.maximum(before, after) <= 0.15)
        )
        fraction = (before / (before + after))[:, None]
        interpolated = (1 - fraction) * seq[:-2] + fraction * seq[2:]
        weight = (0.5 * supported)[:, None]
        seq[1:-1] += weight * (interpolated - seq[1:-1])

    for i, final_q in enumerate(seq):
        positions = sites.positions(final_q)
        actual[i].reshape(-1, 3)[map_indices] = positions[: len(map_indices)]
        errors[i] = np.linalg.norm(actual[i] - points[i], axis=-1)
        if camera is not None:
            rotation = sites.data.xmat[sites.ids[camera]].reshape(3, 3)
            delta = rotation.T @ camera_rotations[i]
            gaze_error[i] = np.arccos(np.clip(delta[2, 2], -1.0, 1.0))
            head_error[i] = np.arccos(np.clip((np.trace(delta) - 1) / 2, -1.0, 1.0))
    return Fit(
        times.copy(),
        robot.names,
        seq,
        sides,
        present,
        errors[..., 0],
        errors[..., 4::4],
        gaze_error,
        head_error,
        placement.copy(),
        errors,
        actual,
    )


def _placement_score(fit: Fit) -> float:
    # Score each observed side separately, so one easy hand cannot hide the other.
    scores: list[float] = []
    for i in range(len(fit.side_names)):
        wrist = fit.wrist_error_m[fit.observed[:, i], i]
        if not wrist.size:
            continue
        score = float(np.quantile(wrist, 0.9) + np.mean(wrist))
        tips = fit.fingertip_error_m[:, i]
        finite = tips[np.isfinite(tips)]
        if finite.size:
            score += float(np.quantile(finite, 0.9)) * 0.5
        scores.append(score)
    return max(scores) + float(np.mean(scores))


def _virtual_fit(
    urdf: Path,
    profile: MappingProfile,
    *,
    time_ns: Indices,
    hands_world_m: Mapping[str, Floats],
    world_from_camera: Floats,
) -> Fit:
    """Choose one virtual workspace registration using upright (+z) source and robot frames.

    Align optical image-right headings after projecting away gravity. Compare fixed,
    scale-preserving translations by achievable wrist and finger FK on samples across
    the native clock. This estimates a display/workspace placement, never a physical mount.
    """
    if profile.camera_optical_link is None:
        raise ValueError("virtual registration requires a declared robot optical link")
    cameras = np.asarray(world_from_camera, dtype=np.float64)
    times = np.asarray(time_ns)
    if times.ndim != 1 or not len(times) or cameras.shape != (len(times), 4, 4):
        raise ValueError("one camera pose is required for every source timestamp")
    _rigid(cameras)
    robot = _model(urdf)
    sides = tuple(profile.hands)
    wrists = tuple(cast("str", profile.hands[side][0]) for side in sides)
    sites = _Sites(robot, (*wrists, profile.camera_optical_link))
    positions = sites.positions(robot.neutral)
    source_lateral = np.median(cameras[:, :2, 0], axis=0)
    robot_lateral = sites.data.xmat[sites.ids[-1]].reshape(3, 3)[:2, 0]
    if min(np.linalg.norm(source_lateral), np.linalg.norm(robot_lateral)) < 0.25:
        raise ValueError("optical heading is ambiguous; supply a measured registration")
    yaw = np.arctan2(robot_lateral[1], robot_lateral[0]) - np.arctan2(
        source_lateral[1], source_lateral[0]
    )

    frame = np.array(from_xyz_rpy((0.0, 0.0, 0.0), (0.0, 0.0, float(yaw)))).reshape(4, 4)
    # Estimate scale only for search spacing. Every candidate retains metric unit scale.
    reach = float(np.linalg.norm(positions[:-1] - positions[-1], axis=1).mean())
    hands = {side: np.asarray(hands_world_m[side], dtype=np.float64) for side in sides}
    for side, sample in hands.items():
        if sample.shape != (len(times), 21, 3):
            raise ValueError(f"{side} must have one set of landmarks per timestamp")
    observations = {side: np.isfinite(sample).all(axis=(1, 2)) for side, sample in hands.items()}
    offsets: list[Floats] = []
    sample_ids: list[int] = []
    for i, side in enumerate(sides):
        indices = np.flatnonzero(observations[side])
        if not len(indices):
            continue
        wrists_robot = hands[side][indices, 0] @ frame[:3, :3].T
        offsets.append(positions[i] - np.median(wrists_robot, axis=0))
        # Select across time and also include the extremes of the task workspace.
        for row in np.linspace(0, len(indices) - 1, min(len(indices), 24)).astype(int):
            sample_ids.append(int(indices[row]))
        for axis in range(3):
            coordinate = wrists_robot[:, axis]
            for percentile in (0.05, 0.95):
                row = np.argmin(
                    np.abs(coordinate - np.sort(coordinate)[round((len(indices) - 1) * percentile)])
                )
                sample_ids.append(int(indices[row]))
    if not offsets:
        raise ValueError("virtual registration needs observed wrist landmarks")
    chosen = np.array(sorted(set(sample_ids)), dtype=np.int64)
    center = np.mean(offsets, axis=0)

    def fit_at(point: Floats, indices: Indices) -> Fit:
        frame[:3, 3] = point
        return solve(
            urdf,
            profile,
            time_ns=times[indices],
            hands_world_m={side: hands[side][indices] for side in sides},
            world_from_camera=cameras[indices],
            robot_from_world=frame,
        )

    # Coupled shoulders and the torso admit local IK branches, so greedy moves in
    # individual axes can miss a much better workspace placement.
    sampled: list[tuple[float, Floats]] = []
    best: Fit | None = None
    best_score = np.inf
    coarse = (
        np.array(delta) * (0.27 * reach) + center for delta in product((-1.0, 0.0, 1.0), repeat=3)
    )
    for point in coarse:
        fit = fit_at(point, chosen)
        score = _placement_score(fit)
        sampled.append((score, point))
        if score < best_score:
            best, best_score = fit, score
    assert best is not None
    fine_center = best.robot_from_world[:3, 3]
    for delta in np.concatenate((np.eye(3), -np.eye(3))):
        point = fine_center + delta * (0.11 * reach)
        fit = fit_at(point, chosen)
        score = _placement_score(fit)
        sampled.append((score, point))
        if score < best_score:
            best, best_score = fit, score
    if len(chosen) == len(times):
        return best

    # Sparse fits can choose a different IK branch from the native-clock fit.
    # Compare the best three candidates on the actual ordered sequence and reuse
    # the winning trajectory. No frames are dropped from final FK residuals.
    best_score = np.inf
    all_indices = np.arange(len(times), dtype=np.int64)
    for _, point in sorted(sampled, key=lambda item: item[0])[:3]:
        fit = fit_at(point, all_indices)
        score = _placement_score(fit)
        if score < best_score:
            best, best_score = fit, score
    return best


def estimate_registration(
    urdf: Path,
    profile: MappingProfile,
    *,
    time_ns: Indices,
    hands_world_m: Mapping[str, Floats],
    world_from_camera: Floats,
) -> Floats:
    """Return the selected virtual registration. The solve entry point also returns the FK."""
    return solve(
        urdf,
        profile,
        time_ns=time_ns,
        hands_world_m=hands_world_m,
        world_from_camera=world_from_camera,
        robot_from_world=None,
    ).robot_from_world
