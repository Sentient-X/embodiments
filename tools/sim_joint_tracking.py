"""Slice 3b's simulation evidence: does model feed-forward fix the static upper body's tracking?

The robot in MuJoCo is the static upper body's description with every link's mass
deliberately wrong: the truth is each URDF mass (and inertia) scaled by a factor drawn from
0.7..1.3 with its centre of mass moved up to 1 cm, plus friction and rotor inertia per motor
model (MuJoCo ``frictionloss``, which sticks at rest, ``damping`` and ``armature``). The
controller only ever knows the URDF, or what identification measured.

Each Damiao motor is the MIT law its firmware runs, at 1 kHz on its own encoder:
``tau = kp (q_des - q) + kd (v_des - v) + tau_ff``, saturated at the motor's peak torque
(DM8009 40, DM4340 27, DM4310 7 N·m). The station updates ``q_des``, ``v_des`` and ``tau_ff``
at 100 Hz. Gear backlash, torque ripple and current-loop dynamics are not modelled; BAM
(https://github.com/Rhoban/bam, "Better Actuator Models", Rhoban) identifies those per motor and
is the better model for a later pass.

The operator trajectory is real: the IK commands of the 2026-10-25 physical run of Grippansh's
``humanoid_pkg`` (20 s of following, both arms; ``docs/evidence/joint-tracking/
operator-2026-10-25.csv``, which also holds what the robot measured). Those commands jump (IK
steps up to 50 rad/s), so the reference every controller tracks is them rate-limited to the
declared 0.5 rad/s and low-passed at 2 Hz: the planned motion a governed station would send.

Four controllers track it:

1. the prototypes: per-motor kp/kd from Supraj's ``allisa_current_motors.json`` (the same
   table ``src/static_upper_body.rs`` carries), ``v_des = 0``, ``tau_ff = 0``;
2. the same gains with the planned velocity;
3. URDF-model feed-forward (gravity at the measured q, inertia and Coriolis at the plan) with
   the planned velocity and low gains;
4. the same with the identified model: identification run in this simulation exactly as the
   runbook runs it on the robot (held poses approached from both sides, then one excitation per
   arm), from motor torque quantised to the 12-bit MIT field with 0.5 % noise.

Run from the repository root with ``mujoco`` and the ``calibrate`` extra installed::

    python tools/sim_joint_tracking.py [--markdown docs/evidence/joint-tracking/README.md]
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
import sys
import time
import xml.etree.ElementTree as ET
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from scipy.signal import butter, filtfilt

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from identify_dynamics import fit_recordings, read_plan  # noqa: E402
from sx_embodiments.calibrate.friction import budget, moving_friction  # noqa: E402
from sx_embodiments.calibration import JointFriction  # noqa: E402
from sx_embodiments.calibrate.dynamics import (  # noqa: E402
    PER_LINK,
    STANDARD_GRAVITY,
    STICTION_VELOCITY,
    DynamicsFit,
    Excitation,
    Hold,
    RigidBodyTree,
    Trajectory,
    differentiate,
)

DESCRIPTION = ROOT / "assets/sentient_static_upper_body"
URDF = DESCRIPTION / "sentient_static_upper_body.urdf"
OPERATOR = ROOT / "docs/evidence/joint-tracking/operator-2026-10-25.csv"

#: Supraj's per-motor gains and models (``config/allisa_current_motors.json``), the same
#: gains ``src/static_upper_body.rs`` carries.
MOTORS: Mapping[str, tuple[str, float, float]] = {
    "torso_joint": ("DM8009", 40.0, 1.0),
    "left_shoulder_pitch_joint": ("DM8009", 60.984, 1.091),
    "left_shoulder_roll_joint": ("DM8009", 60.984, 1.091),
    "left_upper_arm_yaw_joint": ("DM4340", 24.0, 5.0),
    "left_elbow_pitch_joint": ("DM4340", 30.492, 4.655),
    "left_forearm_yaw_joint": ("DM4340", 29.04, 4.75),
    "left_wrist_pitch_joint": ("DM4310", 20.0, 1.0),
    "left_wrist_roll_joint": ("DM4310", 20.0, 1.0),
    "right_shoulder_pitch_joint": ("DM8009", 60.984, 1.091),
    "right_shoulder_roll_joint": ("DM8009", 42.0, 0.98),
    "right_upper_arm_yaw_joint": ("DM4340", 30.492, 4.655),
    "right_elbow_pitch_joint": ("DM4340", 30.492, 4.655),
    "right_forearm_yaw_joint": ("DM4340", 30.492, 4.655),
    "right_wrist_pitch_joint": ("DM4310", 20.0, 1.0),
    "right_wrist_roll_joint": ("DM4310", 20.0, 1.0),
}
JOINTS = tuple(MOTORS)
LEFT = tuple(j for j in JOINTS if j.startswith("left_"))
RIGHT = tuple(j for j in JOINTS if j.startswith("right_"))
#: Peak torque (N·m), and the 12-bit MIT torque field's range (TMAX) the motor reports in.
PEAK = {"DM8009": 40.0, "DM4340": 27.0, "DM4310": 7.0}
TMAX = {"DM8009": 54.0, "DM4340": 28.0, "DM4310": 10.0}
VMAX = {"DM8009": 45.0, "DM4340": 10.0, "DM4310": 30.0}
#: The truth's drive friction per motor model, BAM's M4 (Coulomb, Stribeck, load-dependent,
#: viscous, armature). The DM4340 elbows' breakaway, Coulomb + Stribeck plus load, is the
#: ±2.5 to 3 N·m band measured on the robot on 2026-10-26 (both elbows settling +5.4/-3.7 deg
#: and +5.9/-5.3 deg from below/above at kp 30.5); the DM4310 wrists settle within 0.2 to 1.4
#: deg at kp 20 (about 0.3 N·m). The DM8009 values are assumed in between.
DRIVE = {
    model: JointFriction(
        model,
        coulomb=c,
        viscous=v,
        armature=a,
        stribeck=s,
        stribeck_velocity=0.03,
        stribeck_exponent=1.5,
        load_motor=k,
        load_external=k,
    )
    for model, (c, v, a, s, k) in {
        "DM8009": (1.2, 0.4, 0.02, 0.8, 0.02),
        "DM4340": (1.4, 0.3, 0.01, 1.2, 0.02),
        "DM4310": (0.15, 0.05, 0.002, 0.1, 0.02),
    }.items()
}
#: Low gains for the feed-forward controllers: a quarter of the prototypes' stiffness.
LOW_KP, LOW_KD = 0.25, 0.5
SPEED_CAP = 0.5  # rad/s, the declared arm speed
SIM_DT = 0.001
TICK = 0.01  # the station's 100 Hz
TORQUE_NOISE = 0.005  # of TMAX
TABLE_MARGIN = 0.02  # m, every collision check's clearance


def stripped(urdf: bytes, meshes: bool, held: bool = False) -> str:
    """The description for MuJoCo; ``held`` welds every joint no Damiao motor drives (the
    Feetech grippers and neck, whose gram-scale links a stiff explicit hold would destabilise)."""
    root = ET.fromstring(urdf)
    if held:
        for joint in root.findall("joint"):
            if joint.get("type") != "fixed" and joint.get("name") not in MOTORS:
                joint.set("type", "fixed")
    for link in root.findall("link"):
        for element in link.findall("visual") + ([] if meshes else link.findall("collision")):
            link.remove(element)
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(
        extension,
        "compiler",
        meshdir=str(DESCRIPTION),
        strippath="false",
        fusestatic="false",
        discardvisual="true",
        balanceinertia="false",
        inertiafromgeom="false",
        angle="radian",
    )
    return ET.tostring(root).decode()


@dataclass
class Truth:
    model: mujoco.MjModel
    theta: np.ndarray


def truth_model(urdf: bytes, tree: RigidBodyTree, seed: int) -> Truth:
    """The robot as built: URDF masses scaled 0.7..1.3, centres moved up to 1 cm, friction."""
    rng = np.random.default_rng(seed)
    model = mujoco.MjModel.from_xml_string(stripped(urdf, meshes=False, held=True))
    model.opt.gravity[:] = (0.0, 0.0, -STANDARD_GRAVITY)
    model.opt.timestep = SIM_DT
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    # Friction loss as stiction: MuJoCo's default soft constraint (0.02 s) lets a held joint
    # creep through its friction band, which a geared motor does not.
    model.dof_solref[:, 0] = 0.002
    model.dof_solimp[:, :2] = (0.99, 0.999)
    theta = tree.parameters()
    for body in range(1, model.nbody):
        if model.body_mass[body] <= 0.0:
            continue
        factor = float(rng.uniform(0.7, 1.3))
        model.body_mass[body] *= factor
        model.body_inertia[body] *= factor
        model.body_ipos[body] += rng.uniform(-0.01, 0.01, 3)
        link = tree.links.index(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body))
        theta[PER_LINK * link : PER_LINK * (link + 1)] = body_parameters(model, body)
    base = PER_LINK * len(tree.links)
    for joint, (motor, _, _) in MOTORS.items():
        drive = DRIVE[motor]
        dof = model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)]
        # frictionloss is set every step from the BAM budget (Robot.run).
        model.dof_damping[dof] = drive.viscous
        model.dof_armature[dof] = drive.armature
        at = base + 3 * tree.axes.index(joint)
        theta[at : at + 3] = (drive.coulomb, drive.viscous, drive.armature)
    return Truth(model, theta)


def body_parameters(model: mujoco.MjModel, body: int) -> np.ndarray:
    """MuJoCo's mass, centre and principal inertia as the ten parameters about the origin."""
    mass = float(model.body_mass[body])
    centre = np.array(model.body_ipos[body])
    turn = np.zeros(9)
    mujoco.mju_quat2Mat(turn, model.body_iquat[body])
    turn = turn.reshape(3, 3)
    at_centre = turn @ np.diag(model.body_inertia[body]) @ turn.T
    about = at_centre + mass * (float(centre @ centre) * np.eye(3) - np.outer(centre, centre))
    return np.array(
        [
            mass,
            *(mass * centre),
            about[0, 0],
            about[0, 1],
            about[0, 2],
            about[1, 1],
            about[1, 2],
            about[2, 2],
        ]
    )


def reference(
    path: Path,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, np.ndarray]]:
    """The operator's commands at 100 Hz, rate-limited to the speed cap and low-passed at 2 Hz:
    time, positions, velocities, accelerations (``(n, 15)``), and the recording's raw columns."""
    with path.open() as handle:
        rows = list(csv.reader(handle))
    header, values = rows[0], np.array(rows[1:], dtype=float)
    raw = {name: values[:, i] for i, name in enumerate(header)}
    stamps = raw["time_s"]
    time_ = np.arange(0.0, stamps[-1], TICK)
    command = np.stack([np.interp(time_, stamps, raw[f"command:{j}"]) for j in JOINTS], axis=1)
    limited = command.copy()
    for i in range(1, len(time_)):
        step = np.clip(command[i] - limited[i - 1], -SPEED_CAP * TICK, SPEED_CAP * TICK)
        limited[i] = limited[i - 1] + step
    b, a = butter(2, 2.0 / (0.5 / TICK))
    q = filtfilt(b, a, limited, axis=0)
    qd = np.gradient(q, TICK, axis=0)
    qdd = np.gradient(qd, TICK, axis=0)
    return time_, q, qd, qdd, raw


@dataclass(frozen=True)
class Controller:
    name: str
    gains: float  # kp scale
    damping: float  # kd scale
    velocity: bool
    theta: np.ndarray | None  # the model feed-forward is computed from, or none
    #: BAM friction per joint for the feed-forward; none uses theta's Coulomb and viscous.
    friction: Mapping[str, JointFriction] | None = None


class Robot:
    """The truth model driven by fifteen simulated Damiao MIT loops; every other joint (the
    grippers and the neck, Feetech servos) is welded at zero."""

    def __init__(self, truth: Truth, tree: RigidBodyTree, seed: int) -> None:
        self.model = truth.model
        self.data = mujoco.MjData(self.model)
        self.tree = tree
        self.rng = np.random.default_rng(seed)
        self.names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
            for j in range(self.model.njnt)
        ]
        self.dofs = np.array([self.model.jnt_dofadr[self.names.index(j)] for j in JOINTS])
        self.peak = np.array([PEAK[MOTORS[j][0]] for j in JOINTS])
        self.tmax = np.array([TMAX[MOTORS[j][0]] for j in JOINTS])
        self.kp = np.array([MOTORS[j][1] for j in JOINTS])
        self.kd = np.array([MOTORS[j][2] for j in JOINTS])
        self.axis_of = np.array([tree.axes.index(j) for j in JOINTS])
        self.drives = [DRIVE[MOTORS[j][0]] for j in JOINTS]
        self.tcp = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, n)
            for n in ("left_gripper_tcp", "right_gripper_tcp")
        ]

    def place(self, q: np.ndarray) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.dofs] = q
        mujoco.mj_forward(self.model, self.data)

    def full(self, values: np.ndarray) -> np.ndarray:
        out = np.zeros(len(self.tree.axes))
        out[self.axis_of] = values
        return out

    def feedforward(
        self,
        controller: Controller,
        q_meas: np.ndarray,
        q: np.ndarray,
        qd: np.ndarray,
        qdd: np.ndarray,
    ) -> np.ndarray:
        """Gravity at the measured q; inertia, Coriolis and friction for the plan. With BAM
        friction, as ``Dynamics::inverse_into`` does: the rigid torque is the drive's and
        gravity's is the load."""
        tree = self.tree
        theta = controller.theta
        assert theta is not None
        if controller.friction is not None:
            theta = theta.copy()
            theta[PER_LINK * len(tree.links) :] = 0.0
        zero = np.zeros(len(tree.axes))
        planned = tree.inverse(theta, self.full(q), self.full(qd), self.full(qdd))
        held_plan = tree.inverse(theta, self.full(q), zero, zero)
        held_now = tree.inverse(theta, self.full(q_meas), zero, zero)
        torque = (planned - held_plan + held_now)[self.axis_of]
        if controller.friction is None:
            return torque
        held = held_now[self.axis_of]
        return torque + np.array(
            [
                float(
                    moving_friction(
                        controller.friction[j],
                        np.array([qd[i]]),
                        np.array([qdd[i]]),
                        np.array([torque[i]]),
                        np.array([-held[i]]),
                        STICTION_VELOCITY,
                    )[0]
                )
                for i, j in enumerate(JOINTS)
            ]
        )

    def run(
        self,
        controller: Controller,
        plan: Callable[[float], tuple[np.ndarray, np.ndarray, np.ndarray]],
        duration: float,
        settle: float = 2.0,
        record: float = 0.002,
    ) -> dict[str, np.ndarray]:
        """Track ``plan`` for ``duration`` seconds after ``settle`` seconds holding its start."""
        q0, _, _ = plan(0.0)
        self.place(q0)
        kp, kd = self.kp * controller.gains, self.kd * controller.damping
        steps_per_tick = round(TICK / SIM_DT)
        log: dict[str, list[np.ndarray]] = {
            k: [] for k in ("t", "q", "qd", "tau", "plan", "tip", "tip_plan")
        }
        t = -settle
        q_des = v_des = tau_ff = np.zeros(len(JOINTS))
        step = 0
        next_record = 0.0
        while t < duration:
            if step % steps_per_tick == 0:
                q_plan, v_plan, a_plan = plan(max(t, 0.0))
                if t < 0.0:
                    v_plan = a_plan = np.zeros_like(q_plan)
                q_meas = self.data.qpos[self.dofs].copy()
                q_des = q_plan
                v_des = v_plan if controller.velocity else np.zeros_like(v_plan)
                tau_ff = (
                    self.feedforward(controller, q_meas, q_plan, v_plan, a_plan)
                    if controller.theta is not None
                    else np.zeros_like(q_plan)
                )
            q = self.data.qpos[self.dofs]
            v = self.data.qvel[self.dofs]
            tau = np.clip(kp * (q_des - q) + kd * (v_des - v) + tau_ff, -self.peak, self.peak)
            self.data.qfrc_applied[:] = 0.0
            self.data.qfrc_applied[self.dofs] = tau
            # BAM's motor model: the friction budget at this speed and drive torque, which
            # MuJoCo applies as frictionloss (sticking at rest). The load is taken as the drive's
            # own torque, as it is at quasi-static motion.
            for i, drive in enumerate(self.drives):
                self.model.dof_frictionloss[self.dofs[i]] = float(
                    budget(drive, np.array([v[i]]), np.array([tau[i]]), np.array([-tau[i]]))[0]
                )
            if t >= next_record and t >= 0.0:
                q_plan, _, _ = plan(t)
                log["t"].append(np.array([t]))
                log["q"].append(q.copy())
                log["qd"].append(v.copy())
                log["tau"].append(tau.copy())
                log["plan"].append(q_plan)
                log["tip"].append(np.concatenate([self.data.xpos[b] for b in self.tcp]))
                next_record = t + record
            mujoco.mj_step(self.model, self.data)
            t += SIM_DT
            step += 1
        out = {k: np.array(v) for k, v in log.items()}
        out["t"] = out["t"][:, 0]
        # The planned tip: forward kinematics of the plan in the same model.
        scratch = mujoco.MjData(self.model)
        tips = []
        for q_plan in out["plan"]:
            scratch.qpos[:] = 0.0
            scratch.qpos[self.dofs] = q_plan
            mujoco.mj_kinematics(self.model, scratch)
            tips.append(np.concatenate([scratch.xpos[b] for b in self.tcp]))
        out["tip_plan"] = np.array(tips)
        return out

    def reported(self, tau: np.ndarray) -> np.ndarray:
        """What the motor's MIT feedback reports: 12-bit over +-TMAX, with noise."""
        noisy = tau + self.rng.normal(0.0, TORQUE_NOISE * self.tmax, tau.shape)
        step = 2.0 * self.tmax / 4095.0
        return np.round(noisy / step) * step


def interpolator(
    time_: np.ndarray, *series: np.ndarray
) -> Callable[[float], tuple[np.ndarray, ...]]:
    def at(t: float) -> tuple[np.ndarray, ...]:
        i = min(int(t / TICK), len(time_) - 1)
        return tuple(s[i] for s in series)

    return at


def identify(
    robot: Robot, tree: RigidBodyTree, urdf_theta: np.ndarray
) -> tuple[DynamicsFit, dict[str, object]]:
    """Run the runbook's plan (``docs/dynamics/static-upper-body-identification-plan.json``)
    in simulation as ``sx-identify`` runs it on the robot, then fit as ``identify_dynamics.py
    fit`` does. The torso is not driven, as on the bench."""
    plan = read_plan()
    started = time.time()
    holding = Controller("hold", 1.0, 1.0, False, None)
    holds_doc = plan["holds"]
    approach, settle, record = (
        holds_doc["approach_rad"],
        holds_doc["settle_s"],
        holds_doc["record_s"],
    )
    holds: list[Hold] = []
    arm = [j for j in JOINTS if j != "torso_joint"]
    index = [JOINTS.index(j) for j in arm]
    for pose in holds_doc["poses"]:
        target = np.array([pose.get(j, 0.0) for j in JOINTS])
        sides = []
        for sign in (-1.0, 1.0):
            begin = target + sign * approach
            duration = approach / 0.1 + settle + record

            def approach_plan(
                t: float, begin: np.ndarray = begin
            ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
                s = min(t * 0.1 / approach, 1.0)
                return begin + (target - begin) * s, np.zeros(len(JOINTS)), np.zeros(len(JOINTS))

            log = robot.run(holding, approach_plan, duration, settle=1.0)
            steady = log["t"] >= duration - record
            sides.append(
                (log["q"][steady].mean(axis=0), robot.reported(log["tau"][steady]).mean(axis=0))
            )
        q_mean = (sides[0][0] + sides[1][0]) / 2.0
        tau_mean = (sides[0][1] + sides[1][1]) / 2.0
        holds.append(
            Hold(
                dict(zip(arm, q_mean[index].tolist(), strict=True)),
                dict(zip(arm, tau_mean[index].tolist(), strict=True)),
            )
        )
    trajectories: list[Trajectory] = []
    for entry in plan["excitations"]:
        trajectory = Excitation(
            tuple(entry["joints"]),
            float(entry["period_s"]),
            np.array(entry["centre_rad"]),
            np.array(entry["sine"]),
            np.array(entry["cosine"]),
        )
        posture = np.array([entry["posture"].get(j, 0.0) for j in JOINTS])
        swung = [JOINTS.index(j) for j in trajectory.joints]

        def excite(
            t: float,
            trajectory: Excitation = trajectory,
            posture: np.ndarray = posture,
            swung: list[int] = swung,
        ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
            q, qd, qdd = trajectory.at(np.array([t]))
            full_q, full_v, full_a = posture.copy(), np.zeros(len(JOINTS)), np.zeros(len(JOINTS))
            full_q[swung], full_v[swung], full_a[swung] = q[0], qd[0], qdd[0]
            return full_q, full_v, full_a

        log = robot.run(
            Controller("excite", 1.0, 1.0, False, None),
            excite,
            entry["repeats"] * trajectory.period,
            settle=1.0,
            record=0.02,
        )
        quantum_q = 2 * 12.5 / 65535.0
        q_read = np.round(log["q"] / quantum_q) * quantum_q
        v_step = np.array([2 * VMAX[MOTORS[j][0]] / 4095.0 for j in JOINTS])
        v_read = np.round(log["qd"] / v_step) * v_step
        q_f, v_f, a_f, tau_f = differentiate(
            log["t"], q_read, v_read, robot.reported(log["tau"]), cutoff_hz=3.0
        )
        keep = slice(50, -50, 1)  # 20 Hz after dropping a second each end
        trajectories.append(
            Trajectory(
                tuple(arm),
                q_f[keep][:, index],
                v_f[keep][:, index],
                a_f[keep][:, index],
                tau_f[keep][:, index],
            )
        )
    found = fit_recordings(tree, holds, trajectories)
    return found, {
        "conditions": {entry["name"]: entry["condition"] for entry in plan["excitations"]},
        "seconds": time.time() - started,
    }


def errors(log: dict[str, np.ndarray], plan: np.ndarray) -> dict[str, float]:
    joint = log["q"] - plan
    tip = np.linalg.norm((log["tip"] - log["tip_plan"]).reshape(len(plan), 2, 3), axis=2)
    return {
        "joint_rms": float(np.sqrt(np.mean(joint**2))),
        "joint_max": float(np.max(np.abs(joint))),
        "tip_rms_mm": 1000 * float(np.sqrt(np.mean(tip**2))),
        "tip_max_mm": 1000 * float(np.max(tip)),
        "torque_peak_frac": 0.0,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--markdown", type=Path)
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()
    urdf = URDF.read_bytes()
    tree = RigidBodyTree.of(urdf)
    truth = truth_model(urdf, tree, args.seed)
    robot = Robot(truth, tree, args.seed)
    urdf_theta = tree.parameters()
    time_, q_ref, qd_ref, qdd_ref, raw = reference(OPERATOR)
    plan = interpolator(time_, q_ref, qd_ref, qdd_ref)
    duration = float(time_[-1])

    found, run = identify(robot, tree, urdf_theta)

    # How close is the identified model to the truth, on gravity at random admissible poses?
    rng = np.random.default_rng(5)
    gravity_error = {"urdf": [], "identified": []}
    for _ in range(50):
        pose = {a: 0.0 for a in tree.axes}
        for j in JOINTS:
            lo, hi = tree.limits[j] or (-1.0, 1.0)
            pose[j] = float(rng.uniform(lo + 0.2, hi - 0.2))
        q = tree.vector(pose)
        zero = np.zeros_like(q)
        truth_tau = tree.inverse(truth.theta, q, zero, zero)[robot.axis_of]
        gravity_error["urdf"].append(
            tree.inverse(urdf_theta, q, zero, zero)[robot.axis_of] - truth_tau
        )
        gravity_error["identified"].append(
            tree.inverse(found.theta, q, zero, zero)[robot.axis_of] - truth_tau
        )  # type: ignore[attr-defined]

    # The in-place sweeps recover BAM friction within a few percent (tests/test_friction.py,
    # and fit-friction on synthetic sweeps: Coulomb 1.50 for 1.5, Stribeck 1.19 for 1.2); the
    # truth stands in for that fit here.
    in_place = {j: DRIVE[MOTORS[j][0]] for j in JOINTS}
    band = jog_band(robot, urdf_theta, in_place)
    controllers = [
        Controller("(i) prototype PD, v_des=0, tau_ff=0", 1.0, 1.0, False, None),
        Controller("(ii) prototype PD + planned velocity", 1.0, 1.0, True, None),
        Controller("(iii) URDF feed-forward + low PD + velocity", LOW_KP, LOW_KD, True, urdf_theta),
        Controller(
            "(iv) identified feed-forward + low PD + velocity", LOW_KP, LOW_KD, True, found.theta
        ),  # type: ignore[attr-defined]
        Controller(
            "(v) identified + in-place BAM friction + low PD + velocity",
            LOW_KP,
            LOW_KD,
            True,
            found.theta,  # type: ignore[attr-defined]
            in_place,
        ),
    ]
    results = []
    for controller in controllers:
        log = robot.run(controller, plan, duration, record=TICK)
        sample = np.clip((log["t"] / TICK).round().astype(int), 0, len(time_) - 1)
        results.append((controller, errors(log, q_ref[sample]), log))

    # The prototype on hardware vs the same controller in this simulation, both on the raw
    # (stepping) command, to see whether the simulated robot is credible.
    raw_q = np.stack([np.interp(time_, raw["time_s"], raw[f"command:{j}"]) for j in JOINTS], axis=1)
    raw_measured = np.stack(
        [np.interp(time_, raw["time_s"], raw[f"measured:{j}"]) for j in JOINTS], axis=1
    )
    zero = np.zeros_like(raw_q)
    raw_log = robot.run(
        controllers[0], interpolator(time_, raw_q, zero, zero), duration, record=TICK
    )
    sample = np.clip((raw_log["t"] / TICK).round().astype(int), 0, len(time_) - 1)
    sim_raw = float(np.sqrt(np.mean((raw_log["q"] - raw_q[sample]) ** 2)))
    hardware_raw = float(np.sqrt(np.mean((raw_measured - raw_q) ** 2)))

    lines = report(results, found, run, gravity_error, sim_raw, hardware_raw, q_ref, qd_ref, urdf)
    lines += [
        "",
        "Settled error after a 0.5 rad elbow jog at 0.3 rad/s, prototype gains (target minus measured, degrees; robot 2026-10-26: right +5.4 / -3.7, left +5.9 / -5.3):",
        "",
        "| Joint | Controller | From below | From above |",
        "| --- | --- | --- | --- |",
    ]
    lines += [f"| {j} | {c} | {a:+.1f} | {b:+.1f} |" for j, c, a, b in band]
    text = "\n".join(lines) + "\n"
    print(text)
    if args.markdown is not None:
        args.markdown.write_text(text)


def jog_band(
    robot: Robot, urdf_theta: np.ndarray, friction: Mapping[str, JointFriction]
) -> list[tuple[str, str, float, float]]:
    """Today's bench jog in simulation: an elbow moved 0.5 rad at 0.3 rad/s by a linear
    target (as the governor slews it), held 2 s, and back. The settled error from each side,
    in degrees, for the prototype PD, and PD with the planned velocity and the URDF gravity
    plus in-place friction feed-forward."""
    out: list[tuple[str, str, float, float]] = []
    controllers = [
        Controller("PD only", 1.0, 1.0, False, None),
        Controller(
            "PD + velocity + gravity and friction feed-forward",
            1.0,
            1.0,
            True,
            urdf_theta,
            friction,
        ),
    ]
    for joint, start in (("right_elbow_pitch_joint", 0.6), ("left_elbow_pitch_joint", -0.6)):
        i = JOINTS.index(joint)
        for controller in controllers:
            errors_deg = []
            for sign in (1.0, -1.0):
                begin = np.zeros(len(JOINTS))
                begin[i] = start - sign * 0.25
                travel = 0.5 / 0.3

                def jog(
                    t: float, begin: np.ndarray = begin, sign: float = sign
                ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
                    q = begin.copy()
                    v = np.zeros(len(JOINTS))
                    s = min(t / travel, 1.0)
                    q[i] = begin[i] + sign * 0.5 * s
                    v[i] = sign * 0.3 if t < travel else 0.0
                    return q, v, np.zeros(len(JOINTS))

                log = robot.run(controller, jog, travel + 2.0, settle=1.0, record=TICK)
                q_end, _, _ = jog(travel + 2.0)
                errors_deg.append(math.degrees(float(q_end[i] - log["q"][-1, i])))
            out.append((joint, controller.name, errors_deg[0], errors_deg[1]))
    return out


def report(
    results, found, run, gravity_error, sim_raw, hardware_raw, q_ref, qd_ref, urdf
) -> list[str]:  # type: ignore[no-untyped-def]
    g_urdf = np.abs(np.array(gravity_error["urdf"]))
    g_id = np.abs(np.array(gravity_error["identified"]))
    lines = [
        "| Controller | Joint RMS (rad) | Joint max (rad) | Tip RMS (mm) | Tip max (mm) |",
        "| --- | --- | --- | --- | --- |",
    ]
    for controller, e, _ in results:
        lines.append(
            f"| {controller.name} | {e['joint_rms']:.4f} | {e['joint_max']:.3f} | {e['tip_rms_mm']:.1f} | {e['tip_max_mm']:.1f} |"
        )
    lines += [
        "",
        f"Reference: {len(q_ref) * TICK:.1f} s, peak planned speed {np.max(np.abs(qd_ref)):.2f} rad/s, both arms.",
        f"Prototype PD on the raw stepping command: simulated {sim_raw:.3f} rad RMS, the robot measured {hardware_raw:.3f} rad RMS.",
        "",
        "Identification in simulation, the runbook's plan as `sx-identify` plays it (PD only, torso held): 20 held poses, each approached from below and above; one 2 x 10 s excitation per arm at <= 0.25 rad/s and 1 rad/s²:",
        f"- base parameters seen: {found.base_parameters}; column-scaled condition number of the stacked regressor: {found.condition:.0f}",
        "- excitation condition numbers: "
        + ", ".join(f"{k} {v:.0f}" for k, v in run["conditions"].items()),
        f"- worst joint RMS torque residual: holds {max(found.hold_residual.values()):.3f} N·m, trajectory {max(found.trajectory_residual.values()):.3f} N·m",
        f"- gravity torque error vs the truth at 50 random poses (mean |error| / max): URDF {g_urdf.mean():.3f} / {g_urdf.max():.2f} N·m, identified {g_id.mean():.3f} / {g_id.max():.2f} N·m",
        f"- links returned to the design estimate as unphysical: {list(found.reverted) or 'none'}",
        f"- run time {run['seconds']:.0f} s",
        "",
        "Gains (kp N·m/rad, kd N·m·s/rad):",
        "",
        "| Joint | Motor | (i)/(ii) kp | kd | (iii)/(iv) kp | kd |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for joint, (motor, kp, kd) in MOTORS.items():
        lines.append(f"| {joint} | {motor} | {kp:g} | {kd:g} | {kp * LOW_KP:g} | {kd * LOW_KD:g} |")
    lines += ["", f"Description sha256 {hashlib.sha256(urdf).hexdigest()}."]
    return lines


if __name__ == "__main__":
    main()
