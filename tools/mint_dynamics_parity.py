"""Mint ``dynamics-parity.json``: MuJoCo's inverse dynamics of the static upper body.

The fixture holds ``sx_embodiment_drivers::Dynamics`` (Rust) and
``sx_embodiments.calibrate.dynamics.RigidBodyTree`` (Python) to one outside answer, MuJoCo's
``mj_inverse`` over the same URDF, at random positions, velocities and accelerations:

- ``design``: the description's own inertials, no friction.
- ``calibrated``: a dynamics calibration (every arm link's mass, first moment and inertia
  changed, friction and armature on every arm joint) written into MuJoCo's model as body mass,
  centre, principal inertia and frame, joint armature and damping; Coulomb friction, which
  MuJoCo models as a constraint rather than ``tanh``, is added analytically.

Pinocchio's RNEA, where installed, is a second check on ``design``; its worst disagreement is
printed. Run from the repository root with ``mujoco`` (and optionally ``pin``) installed:

    python tools/mint_dynamics_parity.py
"""

from __future__ import annotations

import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from datetime import UTC, datetime
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sx_embodiments.calibrate.dynamics import (  # noqa: E402
    PER_LINK,
    STANDARD_GRAVITY,
    RigidBodyTree,
)
from sx_embodiments.calibration import (  # noqa: E402
    CalibrationMethod,
    DynamicsCalibration,
    JointFriction,
    LinkInertia,
    Measurement,
    calibration_to_dict,
)

URDF = ROOT / "assets/sentient_static_upper_body/sentient_static_upper_body.urdf"
FIXTURE = ROOT / "drivers/sx-embodiment-drivers/tests/fixtures/dynamics-parity.json"
SAMPLES = 12
STICTION = 0.05


def mujoco_model(urdf: bytes) -> mujoco.MjModel:
    """The description with its meshes stripped (inertials only), no body fused, no
    constraint (a joint limit would add its force), MuJoCo's gravity set to ours."""
    root = ET.fromstring(urdf)
    for link in root.findall("link"):
        for tag in ("visual", "collision"):
            for element in link.findall(tag):
                link.remove(element)
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(
        extension,
        "compiler",
        fusestatic="false",
        discardvisual="true",
        balanceinertia="false",
        inertiafromgeom="false",
        angle="radian",
    )
    model = mujoco.MjModel.from_xml_string(ET.tostring(root).decode())
    model.opt.gravity[:] = (0.0, 0.0, -STANDARD_GRAVITY)
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
    return model


def write_inertia(model: mujoco.MjModel, body: int, values: np.ndarray) -> None:
    """Ten parameters about the link origin as MuJoCo's mass, centre, principal inertia, frame."""
    mass = float(values[0])
    centre = values[1:4] / mass
    xx, xy, xz, yy, yz, zz = values[4:10]
    about_origin = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    at_centre = about_origin - mass * (
        float(centre @ centre) * np.eye(3) - np.outer(centre, centre)
    )
    principal, axes = np.linalg.eigh(at_centre)
    if np.linalg.det(axes) < 0:
        axes[:, 0] *= -1
    quaternion = np.zeros(4)
    mujoco.mju_mat2Quat(quaternion, axes.flatten())
    model.body_mass[body] = mass
    model.body_ipos[body] = centre
    model.body_inertia[body] = principal
    model.body_iquat[body] = quaternion


def main() -> None:
    urdf = URDF.read_bytes()
    tree = RigidBodyTree.of(urdf)
    model = mujoco_model(urdf)
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(model.njnt)]
    if names != list(tree.axes) or model.nv != len(tree.axes):
        raise SystemExit("MuJoCo's joints are not the tree's independent axes in order")
    data = mujoco.MjData(model)
    rng = np.random.default_rng(20261026)

    arm_joints = [
        axis
        for axis in tree.axes
        if "_arm_" in axis
        or "shoulder" in axis
        or "elbow" in axis
        or "forearm" in axis
        or "wrist" in axis
    ]
    arm_joints = [axis for axis in arm_joints if not axis.startswith("so101")]
    arm_links = tree.subtree_links(arm_joints)
    theta = tree.parameters()
    links: list[LinkInertia] = []
    for link in arm_links:
        values = theta[PER_LINK * link : PER_LINK * (link + 1)].copy()
        if values[0] <= 0.0:
            continue
        mass = values[0] * float(rng.uniform(0.7, 1.3))
        centre = values[1:4] / values[0] + rng.uniform(-0.01, 0.01, 3)
        about_origin = values[4:10]
        tensor = np.array(
            [
                [about_origin[0], about_origin[1], about_origin[2]],
                [about_origin[1], about_origin[3], about_origin[4]],
                [about_origin[2], about_origin[4], about_origin[5]],
            ]
        )
        old_centre = values[1:4] / values[0]
        at_centre = tensor - values[0] * (
            float(old_centre @ old_centre) * np.eye(3) - np.outer(old_centre, old_centre)
        )
        at_centre = at_centre * mass / values[0]
        shifted = at_centre + mass * (float(centre @ centre) * np.eye(3) - np.outer(centre, centre))
        links.append(
            LinkInertia(
                link=tree.links[link],
                mass=float(mass),
                first_moment=(
                    float(mass * centre[0]),
                    float(mass * centre[1]),
                    float(mass * centre[2]),
                ),
                inertia=(
                    float(shifted[0, 0]),
                    float(shifted[0, 1]),
                    float(shifted[0, 2]),
                    float(shifted[1, 1]),
                    float(shifted[1, 2]),
                    float(shifted[2, 2]),
                ),
            )
        )
    friction = [
        JointFriction(
            joint=joint,
            coulomb=float(rng.uniform(0.1, 1.0)),
            viscous=float(rng.uniform(0.05, 0.5)),
            armature=float(rng.uniform(0.001, 0.05)),
        )
        for joint in arm_joints
    ]
    calibration = DynamicsCalibration(
        description_sha256=hashlib.sha256(urdf).hexdigest(),  # pyright: ignore[reportArgumentType]
        stiction_velocity=STICTION,
        links=tuple(sorted(links, key=lambda item: item.link)),
        joints=tuple(sorted(friction, key=lambda item: item.joint)),
        units=(),
        measurement=Measurement(
            CalibrationMethod.DYNAMICS_IDENTIFICATION, datetime(2026, 10, 26, tzinfo=UTC)
        ),
    )
    calibrated = mujoco_model(urdf)
    for item in calibration.links:
        body = mujoco.mj_name2id(calibrated, mujoco.mjtObj.mjOBJ_BODY, item.link)
        write_inertia(calibrated, body, np.array([item.mass, *item.first_moment, *item.inertia]))
    for item in calibration.joints:
        dof = calibrated.jnt_dofadr[tree.axes.index(item.joint)]
        calibrated.dof_armature[dof] = item.armature
        calibrated.dof_damping[dof] = item.viscous
    calibrated_data = mujoco.MjData(calibrated)
    coulomb = np.zeros(len(tree.axes))
    for item in calibration.joints:
        coulomb[tree.axes.index(item.joint)] = item.coulomb
    calibrated_theta = tree.parameters(calibration)

    try:
        import pinocchio

        pin_model = pinocchio.buildModelFromXML(urdf.decode())
        pin_model.gravity.linear = np.array([0.0, 0.0, -STANDARD_GRAVITY])
        pin_data = pin_model.createData()
        pin_order = [pin_model.getJointId(axis) - 1 for axis in tree.axes]
    except ImportError:
        pin_model = None
        pin_data = None
        pin_order = []

    samples: list[dict[str, object]] = []
    worst_python = worst_pin = 0.0
    for _ in range(SAMPLES):
        q = rng.uniform(-math.pi, math.pi, model.nv)
        qd = rng.uniform(-3.0, 3.0, model.nv)
        qdd = rng.uniform(-10.0, 10.0, model.nv)
        data.qpos[:], data.qvel[:], data.qacc[:] = q, qd, qdd
        mujoco.mj_inverse(model, data)
        design = data.qfrc_inverse.copy()
        calibrated_data.qpos[:], calibrated_data.qvel[:], calibrated_data.qacc[:] = q, qd, qdd
        mujoco.mj_inverse(calibrated, calibrated_data)
        rubbed = calibrated_data.qfrc_inverse + coulomb * np.tanh(qd / STICTION)
        worst_python = max(
            worst_python,
            float(np.max(np.abs(tree.inverse(tree.parameters(), q, qd, qdd) - design))),
            float(
                np.max(
                    np.abs(
                        tree.inverse(calibrated_theta, q, qd, qdd, stiction_velocity=STICTION)
                        - rubbed
                    )
                )
            ),
        )
        if pin_model is not None and pin_data is not None:
            order = np.argsort(pin_order)
            tau = pinocchio.rnea(pin_model, pin_data, q[order], qd[order], qdd[order])
            worst_pin = max(worst_pin, float(np.max(np.abs(np.asarray(tau)[pin_order] - design))))
        samples.append(
            {
                "position": q.tolist(),
                "velocity": qd.tolist(),
                "acceleration": qdd.tolist(),
                "design": design.tolist(),
                "calibrated": rubbed.tolist(),
            }
        )
    print(  # noqa: T201 - the mint reports its own cross-check
        f"python vs mujoco worst {worst_python:.3e} N·m; "
        f"pinocchio vs mujoco worst {worst_pin:.3e} N·m"
    )
    FIXTURE.write_text(
        json.dumps(
            {
                "source": "tools/mint_dynamics_parity.py; MuJoCo " + mujoco.__version__,
                "description": "assets/sentient_static_upper_body/sentient_static_upper_body.urdf",
                "gravity": [0.0, 0.0, -STANDARD_GRAVITY],
                "axes": list(tree.axes),
                "calibration": calibration_to_dict(calibration),
                "samples": samples,
            },
            indent=1,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
