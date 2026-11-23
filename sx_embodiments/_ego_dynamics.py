"""Unloaded inverse dynamics in the exact independent joint coordinates of an offline fit."""

from __future__ import annotations

import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path
from types import ModuleType
from typing import Protocol, cast

import mujoco  # pyright: ignore[reportMissingTypeStubs] -- the native wheel has no stubs
import numpy as np
from numpy.typing import NDArray

from ._ego_effort_limits import EffortLimits
from .calibrate.dynamics import UPRIGHT
from .kinematics import JointType, Kinematics

type Floats = NDArray[np.float64]
type Indices = NDArray[np.int64]
_mj = cast("ModuleType", mujoco)


class _Options(Protocol):
    gravity: Floats
    disableflags: int


class _Model(Protocol):
    nq: int
    nv: int
    njnt: int
    opt: _Options
    dof_armature: Floats


class _Data(Protocol):
    qpos: Floats
    qvel: Floats
    qacc: Floats
    qfrc_inverse: Floats


class EffortModel:
    """Fixed-base CAD rigid-body loads, without contact, transmissions, or joint friction.

    The root-frame gravity vector is in m/s². Returned efforts are Nm for revolute
    joints and N for prismatic joints. Dependent joints contribute by virtual work
    to their leader, assuming the follower is an ideal mechanical coupling. Supplied
    reflected rotational inertias augment the joint-space mass matrix.
    """

    def __init__(
        self,
        model: _Model,
        joint_names: tuple[str, ...],
        effort_limits: Floats,
        operating_velocity_limits: Floats,
        mapping: Floats,
        origin: Floats,
    ) -> None:
        self.joint_names = joint_names
        self.effort_limits = effort_limits
        self.operating_velocity_limits = operating_velocity_limits
        self._model = model
        self._data = cast("_Data", _mj.MjData(model))
        self._mapping = mapping
        self._origin = origin
        self._mass = np.empty((model.nv, model.nv))

    @classmethod
    def of(
        cls,
        urdf: Path,
        joint_names: tuple[str, ...],
        gravity: tuple[float, float, float] | Floats = UPRIGHT,
        effort_limits: EffortLimits | None = None,
    ) -> EffortModel:
        source = urdf.read_bytes()
        tree = Kinematics.of(source)
        if len(joint_names) != len(tree.independent) or set(joint_names) != set(tree.independent):
            raise ValueError("dynamics needs all independent URDF coordinates in a unique order")
        if np.shape(gravity) != (3,) or not np.all(np.isfinite(gravity)):
            raise ValueError("dynamics needs a finite gravity vector in the robot root frame")
        root = ET.fromstring(source)  # nosec B314 - supplied robot description
        for extension in root.findall("mujoco"):
            root.remove(extension)
        ET.SubElement(
            ET.SubElement(root, "mujoco"),
            "compiler",
            fusestatic="false",
            balanceinertia="false",
            inertiafromgeom="false",
        )
        massless: list[str] = []
        for link in root.findall("link"):
            name = link.attrib["name"]
            meshes = [*link.findall("visual"), *link.findall("collision")]
            parent = tree.parent_joint.get(name)
            moving = parent is not None and tree.joints[parent].type != JointType.FIXED
            inertia = link.find("inertial")
            # The base is bolted down. Fixed, geometry-free links with no declared
            # inertia are URDF coordinate frames (e.g. the optical axes).
            if parent is not None and (moving or meshes or inertia is not None):
                mass = link.find("inertial/mass")
                tensor = link.find("inertial/inertia")
                if (
                    mass is None
                    or not (0 < float(mass.get("value", "nan")) < np.inf)
                    or tensor is None
                    or not all(
                        np.isfinite(float(tensor.get(key, "nan")))
                        for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
                    )
                ):
                    massless.append(name)
            for mesh in meshes:
                link.remove(mesh)
        if massless:
            raise ValueError(f"missing physical link inertia for inverse dynamics: {massless}")
        limits: dict[str, float] = {}
        for joint in root.findall("joint"):
            # Deliberately strip passive URDF coefficients: this estimate reports
            # rigid-body demand only, with an identical definition on every model.
            for passive in joint.findall("dynamics"):
                joint.remove(passive)
            limit = joint.find("limit")
            if limit is not None:
                effort = float(limit.get("effort", "nan"))
                if np.isfinite(effort) and effort > 0:
                    limits[joint.attrib["name"]] = effort

        model = cast("_Model", _mj.MjModel.from_xml_string(ET.tostring(root, encoding="unicode")))
        if model.nq != model.nv or model.njnt != model.nv or not model.nv:
            raise ValueError("inverse dynamics expects fixed-base scalar URDF joints")
        model.opt.gravity[:] = gravity
        model.opt.disableflags |= int(_mj.mjtDisableBit.mjDSBL_CONSTRAINT)
        native = tuple(
            str(_mj.mj_id2name(model, _mj.mjtObj.mjOBJ_JOINT, i)) for i in range(model.nv)
        )
        operating_velocity = np.full(len(joint_names), np.nan)
        if effort_limits is not None:
            if effort_limits.urdf_sha256 != hashlib.sha256(source).hexdigest():
                raise ValueError("effort limits do not match the URDF SHA-256")
            for rating in effort_limits.joints:
                limits[rating.joint] = rating.maximum
                if rating.operating_velocity is not None:
                    operating_velocity[joint_names.index(rating.joint)] = (
                        rating.operating_velocity.maximum
                    )
                if rating.reflected_inertia_kg_m2 is not None:
                    model.dof_armature[native.index(rating.joint)] = rating.reflected_inertia_kg_m2
        independent = dict.fromkeys(joint_names, 0.0)

        def coordinates() -> Floats:
            expanded = tree.coordinates(independent)
            return np.array([expanded[name] for name in native])

        origin = coordinates()
        mapping = np.empty((model.nv, len(joint_names)))
        for col, name in enumerate(joint_names):
            independent[name] = 1.0
            mapping[:, col] = coordinates() - origin
            independent[name] = 0.0
        return cls(
            model,
            joint_names,
            np.array([limits.get(name, np.nan) for name in joint_names]),
            operating_velocity,
            mapping,
            origin,
        )

    def inverse(self, q: Floats, qd: Floats, qdd: Floats) -> Floats:
        if any(
            np.shape(value) != (len(self.joint_names),) or not np.all(np.isfinite(value))
            for value in (q, qd, qdd)
        ):
            raise ValueError("dynamics state must be finite and match the independent joint order")
        self._data.qpos[:] = self._origin + self._mapping @ q
        self._data.qvel[:] = self._mapping @ qd
        self._data.qacc[:] = self._mapping @ qdd
        _mj.mj_inverse(self._model, self._data)
        return self._mapping.T @ self._data.qfrc_inverse

    def linearize(
        self, q: Floats, qd: Floats, qdd: Floats, columns: Indices | None = None
    ) -> tuple[Floats, Floats, Floats, Floats]:
        """Return tau and d(tau)/d(position, velocity, acceleration) at one exposure.

        All rows retain the input joint order. Optional columns select the coordinates
        varied by the optimizer. Acceleration derivatives are the exact reduced mass
        matrix. Finite differences in position/velocity use MuJoCo's native recursion.
        """
        tau = self.inverse(q, qd, qdd)
        native = self._model.nv
        selected = self._mapping if columns is None else self._mapping[:, columns]
        if selected.ndim != 2:
            raise ValueError("dynamics derivative columns must be a vector of joint indices")
        reduce = self._mapping.T
        _mj.mj_fullM(self._model, self._data, self._mass)
        mass = reduce @ self._mass @ selected
        eps = 1e-7
        if 2 * selected.shape[1] <= native:
            # A whole-body fit may vary just the arms. Only re-evaluate the
            # requested columns, and reuse position quantities for velocity columns.
            data = self._data
            position, velocity, base = data.qpos.copy(), data.qvel.copy(), data.qfrc_inverse.copy()
            pos, vel = np.empty_like(selected), np.empty_like(selected)
            for col, step in enumerate(selected.T * eps):
                data.qpos[:] = position + step
                _mj.mj_inverse(self._model, data)
                pos[:, col] = (data.qfrc_inverse - base) / eps
            data.qpos[:] = position
            _mj.mj_inverse(self._model, data)
            for col, step in enumerate(selected.T * eps):
                data.qvel[:] = velocity + step
                _mj.mj_inverseSkip(self._model, data, _mj.mjtStage.mjSTAGE_POS, 1)
                vel[:, col] = (data.qfrc_inverse - base) / eps
            data.qvel[:] = velocity
            return tau, reduce @ pos, reduce @ vel, mass

        pos, vel = np.empty((native, native)), np.empty((native, native))
        _mj.mjd_inverseFD(
            self._model, self._data, eps, False, pos, vel, None, None, None, None, None
        )
        # mjd_inverseFD writes derivative inputs along rows, force outputs along columns.
        return tau, reduce @ pos.T @ selected, reduce @ vel.T @ selected, mass
