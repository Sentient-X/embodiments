"""Plan and fit the static upper body's dynamics identification (runbook:
``docs/DYNAMICS-IDENTIFICATION.md``).

``plan`` writes the poses and trajectories ``sx-identify`` plays on the robot: twenty held
poses of both arms inside the joint limits and free of collision (MuJoCo's convex hulls of the
description's collision meshes, grown 2 cm, and the table top as the base's z = 0 plane), and
one excitation per arm, a finite Fourier series inside the position, speed, acceleration and
torque bounds below. ``fit`` reads what ``sx-identify`` recorded and writes the sealed
``dynamics`` calibration for the robot instance to select::

    python tools/identify_dynamics.py plan
    python tools/identify_dynamics.py fit --holds holds.csv --excitation left.csv \\
        --excitation right.csv --unit <motor serial>... --out dynamics.json

``plan`` needs ``mujoco``; ``fit`` needs only the ``calibrate`` extra.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sx_embodiments.calibrate.dynamics import (  # noqa: E402
    DynamicsFit,
    ExcitationLimits,
    Hold,
    RigidBodyTree,
    Trajectory,
    differentiate,
    excitation,
    fit,
    hold_poses,
)
from sx_embodiments.calibrate.targets import CalibrationSolveError  # noqa: E402
from sx_embodiments.calibrate.friction import Segment, SegmentLimits, fit_joint  # noqa: E402
from sx_embodiments.calibrate.dynamics import STICTION_VELOCITY  # noqa: E402
from sx_embodiments.calibration import (  # noqa: E402
    FRICTION_TERMS,
    CalibrationMethod,
    DynamicsCalibration,
    JointFriction,
    Measurement,
    UnitSerial,
    calibration_id,
    calibration_to_dict,
)
from sx_contracts import Sha256Digest  # noqa: E402

DESCRIPTION = ROOT / "assets/sentient_static_upper_body"
URDF = DESCRIPTION / "sentient_static_upper_body.urdf"
PLAN = ROOT / "docs/dynamics/static-upper-body-identification-plan.json"
PLAN_SCHEMA = "sx.dynamics-identification-plan/v1"

TORSO = "torso_joint"
ARMS = {
    side: tuple(
        f"{side}_{name}_joint"
        for name in (
            "shoulder_pitch",
            "shoulder_roll",
            "upper_arm_yaw",
            "elbow_pitch",
            "forearm_yaw",
            "wrist_pitch",
            "wrist_roll",
        )
    )
    for side in ("left", "right")
}
#: Peak torque per joint (DM8009 40, DM4340 27, DM4310 7 N·m).
PEAK: Mapping[str, float] = {
    TORSO: 40.0,
    **{
        joint: peak
        for arm in ARMS.values()
        for joint, peak in zip(arm, (40.0, 40.0, 27.0, 27.0, 27.0, 7.0, 7.0), strict=True)
    },
}
#: Today's driver speed cap; the declared arm speed is 0.5 rad/s.
SPEED = 0.25
ACCELERATION = 1.0
#: Each excitation may ask at most this share of a joint's peak torque at the design masses.
TORQUE_SHARE = 0.5
POSES = 20
MARGIN = 0.15  # rad inside every joint limit
EXCITATION_REACH = 0.35  # rad either side of the held pose an arm swings about
APPROACH = 0.05  # rad: each pose is approached from this far below and above
SETTLE, RECORD = 2.0, 1.0  # s held before recording, s recorded
TABLE_MARGIN = 0.02  # m
#: What a hold's fit residual must stay within, per joint, for the fit to be accepted: 3 % of
#: the joint's peak torque.
ACCEPT_SHARE = 0.03


def collision_checker(
    urdf: bytes, tree: RigidBodyTree, margin: float = TABLE_MARGIN
) -> Callable[[Mapping[str, float]], bool]:
    """A conservative self- and table-collision check: MuJoCo's convex hulls of the collision
    meshes, grown by ``TABLE_MARGIN``, and the table top as the plane z = 0 of the base. Pairs
    already touching at the zero pose (the base on the table, its turntable, closed fingers)
    are mounting, not collision."""
    import xml.etree.ElementTree as ET

    import mujoco

    root = ET.fromstring(urdf)
    for link in root.findall("link"):
        for element in link.findall("visual"):
            link.remove(element)
    for mesh in root.iter("mesh"):
        name = mesh.get("filename", "")
        if name.startswith("package://sx-embodiments/"):
            mesh.set(
                "filename", str(ROOT / "assets" / name.removeprefix("package://sx-embodiments/"))
            )
    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(
        extension,
        "compiler",
        meshdir=str(DESCRIPTION),
        strippath="false",
        fusestatic="false",
        discardvisual="true",
        angle="radian",
    )
    spec = mujoco.MjSpec.from_string(ET.tostring(root).decode())
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1], name="table")
    model = spec.compile()
    model.geom_margin[:] = margin
    data = mujoco.MjData(model)
    order = [
        tree.axes.index(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j))
        for j in range(model.njnt)
    ]

    def touching() -> set[tuple[int, int]]:
        mujoco.mj_forward(model, data)
        pairs: set[tuple[int, int]] = set()
        for contact in data.contact[: data.ncon]:
            a, b = int(model.geom_bodyid[contact.geom1]), int(model.geom_bodyid[contact.geom2])
            pairs.add((min(a, b), max(a, b)))
        return pairs

    data.qpos[:] = 0.0
    mounted = touching()

    def admissible(pose: Mapping[str, float]) -> bool:
        data.qpos[:] = tree.vector(pose)[order]
        return touching() <= mounted

    return admissible


def make_plan(urdf: bytes) -> dict[str, object]:
    tree = RigidBodyTree.of(urdf)
    admissible = collision_checker(urdf, tree)
    rest = dict.fromkeys(tree.axes, 0.0)
    joints = (*ARMS["left"], *ARMS["right"])
    poses = hold_poses(tree, joints, rest, POSES, admissible, seed=7, margin=MARGIN)
    excitations: list[dict[str, object]] = []
    for side, arm in ARMS.items():
        limits = ExcitationLimits(
            speed=SPEED,
            acceleration=ACCELERATION,
            torque={joint: TORQUE_SHARE * PEAK[joint] for joint in arm},
            margin=MARGIN,
        )
        # Each arm swings about a held pose (the other arm held there too), inside a box of
        # +-EXCITATION_REACH: the collision check is convex hulls, conservative enough that
        # whole-range swings rarely clear the chest.
        found = None
        for posture in poses:
            box = {j: (posture[j] - EXCITATION_REACH, posture[j] + EXCITATION_REACH) for j in arm}
            try:
                found = (
                    excitation(
                        tree, arm, posture, limits, admissible, seed=11, candidates=12, box=box
                    ),
                    posture,
                )
            except CalibrationSolveError:
                continue
            break
        if found is None:
            raise SystemExit(f"no feasible {side} excitation about any held pose")
        (trajectory, condition), posture = found
        excitations.append(
            {
                "name": side,
                "repeats": 2,
                "condition": round(condition, 1),
                "posture": {joint: round(posture[joint], 4) for joint in joints},
                **trajectory.to_dict(),
            }
        )
    return {
        "schema": PLAN_SCHEMA,
        "description_sha256": hashlib.sha256(urdf).hexdigest(),
        "speed_rad_s": SPEED,
        "acceleration_rad_s2": ACCELERATION,
        "holds": {
            "approach_rad": APPROACH,
            "settle_s": SETTLE,
            "record_s": RECORD,
            "poses": [{joint: round(pose[joint], 4) for joint in joints} for pose in poses],
        },
        "excitations": excitations,
    }


#: In-place friction sweeps: the clearance kept along every sweep and path, the speeds swept
#: (each direction, every pass), and the breakaway ramp.
SWEEP_PLAN = ROOT / "docs/dynamics/static-upper-body-friction-sweeps.json"
SWEEP_SCHEMA = "sx.friction-sweep-plan/v1"
SWEEP_CLEARANCE = 0.05  # m
SWEEP_SPEEDS = (0.05, 0.1, 0.2, 0.3, 0.45)
SWEEP_PASSES = 3
SWEEP_REACH = 1.2  # rad, the longest sub-range swept
SWEEP_MIN = 0.4  # rad, the shortest sub-range worth sweeping
BREAKAWAY_RATE = 0.01  # rad/s
BREAKAWAY_TRAVEL = 0.08  # rad
#: Not swept: the torso motor is out of the bench chain, and the left wrist roll rests out of
#: its range until its range fix lands.
NOT_SWEPT = ("torso_joint", "left_wrist_roll_joint")


def _clear_interval(
    tree: RigidBodyTree,
    admissible: Callable[[Mapping[str, float]], bool],
    pose: Mapping[str, float],
    joint: str,
    step: float = 0.02,
) -> tuple[float, float]:
    """The contiguous range of ``joint`` around its value in ``pose`` that stays admissible,
    inside its limits less ``MARGIN`` and at most ``SWEEP_REACH`` long."""
    limit = tree.limits[joint] or (-math.pi, math.pi)
    low_limit, high_limit = limit[0] + MARGIN, limit[1] - MARGIN
    start = pose[joint]
    bounds = []
    for sign in (-1.0, 1.0):
        reach = 0.0
        while reach + step <= SWEEP_REACH:
            value = start + sign * (reach + step)
            if not (low_limit <= value <= high_limit):
                break
            if not admissible({**pose, joint: value}):
                break
            reach += step
        bounds.append(start + sign * reach)
    lower, upper = bounds
    if upper - lower > SWEEP_REACH:
        upper = lower + SWEEP_REACH
    return lower, upper


def make_sweep_plan(urdf: bytes, seed: int = 3, candidates: int = 120) -> dict[str, object]:
    """For each swept joint, a staging pose of both arms from which that joint's sub-range is
    clear of the body and the table by ``SWEEP_CLEARANCE`` all the way (MuJoCo's convex hulls
    of the collision meshes, which only overstate the body, and the table top taken as the
    whole plane z = 0 of the base, which only overstates the table), chosen for the longest
    clear sub-range weighted by how much gravity's load changes across it, the load BAM's
    load-dependent terms are identified from. The straight path from the ready pose (every arm
    joint at zero) to each staging pose is checked too."""
    rng = np.random.default_rng(seed)
    tree = RigidBodyTree.of(urdf)
    admissible = collision_checker(urdf, tree, SWEEP_CLEARANCE)
    ready = dict.fromkeys(tree.axes, 0.0)
    if not admissible(ready):
        raise SystemExit("the ready pose is not clear; choose another")
    arm = [*ARMS["left"], *ARMS["right"]]
    theta = tree.parameters()
    zero = np.zeros(len(tree.axes))
    sweeps: list[dict[str, object]] = []
    for joint in (j for j in arm if j not in NOT_SWEPT):
        best: tuple[float, dict[str, float], tuple[float, float]] | None = None
        for _ in range(candidates):
            pose = dict(ready)
            for other in arm:
                limit = tree.limits[other] or (-1.0, 1.0)
                pose[other] = float(
                    np.clip(rng.uniform(-0.8, 0.8), limit[0] + MARGIN, limit[1] - MARGIN)
                )
            if not admissible(pose):
                continue
            lower, upper = _clear_interval(tree, admissible, pose, joint)
            if upper - lower < SWEEP_MIN:
                continue
            staging = {**pose, joint: lower}
            path_clear = all(
                admissible({k: ready[k] + s * (staging[k] - ready[k]) for k in ready})
                for s in np.linspace(0.0, 1.0, 40)
            )
            if not path_clear:
                continue
            index = tree.axes.index(joint)
            load = [
                tree.inverse(theta, tree.vector({**pose, joint: float(q)}), zero, zero)[index]
                for q in np.linspace(lower, upper, 9)
            ]
            score = (upper - lower) * (1.0 + float(np.ptp(load)))
            if best is None or score > best[0]:
                best = (score, staging, (lower, upper))
        if best is None:
            raise SystemExit(f"no clear staging pose for {joint}")
        _, staging, (lower, upper) = best
        sweeps.append(
            {
                "joint": joint,
                "staging": {k: round(staging[k], 4) for k in arm},
                "lower": round(lower, 4),
                "upper": round(upper, 4),
            }
        )
    return {
        "schema": SWEEP_SCHEMA,
        "description_sha256": hashlib.sha256(urdf).hexdigest(),
        "clearance_m": SWEEP_CLEARANCE,
        "ready": {k: 0.0 for k in arm},
        "speeds_rad_s": list(SWEEP_SPEEDS),
        "passes": SWEEP_PASSES,
        "dwell_s": 0.5,
        "breakaway_rate_rad_s": BREAKAWAY_RATE,
        "breakaway_travel_rad": BREAKAWAY_TRAVEL,
        "sweeps": sweeps,
    }


def read_plan(path: Path = PLAN) -> dict[str, object]:
    plan = json.loads(path.read_text())
    if plan.get("schema") != PLAN_SCHEMA:
        raise SystemExit(f"{path} is not a {PLAN_SCHEMA} document")
    return plan


@dataclass(frozen=True, slots=True, eq=False)
class Recording:
    """What ``sx-identify`` wrote: per tick, time, segment, and per axis target, position,
    velocity and torque at the joint."""

    joints: tuple[str, ...]
    time: np.ndarray
    segments: tuple[str, ...]
    target: np.ndarray
    position: np.ndarray
    velocity: np.ndarray
    torque: np.ndarray

    @classmethod
    def read(cls, path: Path) -> Recording:
        with path.open() as handle:
            # A leading "# key=value" line is run metadata (the supply voltage).
            rows = [row for row in csv.reader(handle) if row and not row[0].startswith("#")]
        header = rows[0]
        joints = tuple(name.split(":", 1)[1] for name in header if name.startswith("position:"))
        body = rows[1:]

        def block(kind: str) -> np.ndarray:
            columns = [header.index(f"{kind}:{joint}") for joint in joints]
            return np.array([[float(row[c]) for c in columns] for row in body])

        return cls(
            joints,
            np.array([float(row[0]) for row in body]),
            tuple(row[1] for row in body),
            block("target"),
            block("position"),
            block("velocity"),
            block("torque"),
        )

    def rows(self, label: str) -> np.ndarray:
        return np.array([i for i, s in enumerate(self.segments) if s == label], dtype=np.intp)


def read_holds(path: Path, record_s: float) -> tuple[list[Hold], dict[str, float]]:
    """Each pose's two holds (from below and above), the last ``record_s`` of each averaged,
    then the two averaged; and the worst joint speed seen while recording."""
    recording = Recording.read(path)
    joints = recording.joints
    by_pose: dict[str, list[tuple[np.ndarray, np.ndarray]]] = {}
    still: dict[str, float] = {}
    for label in sorted({s for s in recording.segments if s.startswith("pose")}):
        rows = recording.rows(label)
        recorded = rows[recording.time[rows] >= recording.time[rows[-1]] - record_s]
        pose = label.split(":", 1)[0]
        by_pose.setdefault(pose, []).append(
            (recording.position[recorded].mean(axis=0), recording.torque[recorded].mean(axis=0))
        )
        for j, joint in enumerate(joints):
            speed = float(np.max(np.abs(recording.velocity[recorded, j])))
            still[joint] = max(still.get(joint, 0.0), speed)
    holds = [
        Hold(
            dict(zip(joints, np.mean([p for p, _ in sides], axis=0).tolist(), strict=True)),
            dict(zip(joints, np.mean([t for _, t in sides], axis=0).tolist(), strict=True)),
        )
        for _, sides in sorted(by_pose.items())
        if len(sides) == 2
    ]
    return holds, still


def read_excitation(path: Path, cutoff_hz: float = 3.0, rate_hz: float = 20.0) -> Trajectory:
    """The excitation rows, zero-phase filtered, differentiated, and sampled at ``rate_hz``."""
    recording = Recording.read(path)
    rows = recording.rows("excite")
    time_ = recording.time[rows]
    q, qd, qdd, tau = differentiate(
        time_, recording.position[rows], recording.velocity[rows], recording.torque[rows], cutoff_hz
    )
    period = float(np.median(np.diff(time_)))
    step = max(1, round(1.0 / (rate_hz * period)))
    edge = round(1.0 / period)  # drop a second at each end, where the filter rings
    keep = slice(edge, -edge, step)
    return Trajectory(recording.joints, q[keep], qd[keep], qdd[keep], tau[keep])


#: The share of a sweep's travel cut from each end, where the target accelerates.
SWEEP_TRIM = 0.15
#: How far a breakaway ramp's joint must move from where it started to count as moving, rad.
BREAKAWAY_ONSET = 0.005


def friction_segments(recording: Recording, joint: str) -> list[Segment]:
    """The constant-velocity middle of every ``sweep:`` segment of ``joint``, and the onset of
    every ``break:`` ramp: the torque at the first sample ``BREAKAWAY_ONSET`` from its start."""
    j = recording.joints.index(joint)
    segments: list[Segment] = []
    for label in dict.fromkeys(recording.segments):
        if label.startswith("sweep:"):
            rows = recording.rows(label)
            command = recording.target[rows, j]
            lo, hi = command.min(), command.max()
            span = hi - lo
            middle = rows[(command >= lo + SWEEP_TRIM * span) & (command <= hi - SWEEP_TRIM * span)]
            pass_index = int(label.split(":")[1])
            segments.append(
                Segment(
                    label,
                    pass_index,
                    recording.position[middle, j],
                    recording.velocity[middle, j],
                    recording.torque[middle, j],
                    recording.target[middle, j],
                )
            )
        elif label.startswith("break:"):
            rows = recording.rows(label)
            start = recording.position[rows[0], j]
            moved = np.flatnonzero(np.abs(recording.position[rows, j] - start) > BREAKAWAY_ONSET)
            if moved.size == 0:
                continue
            at = rows[moved[0]]
            direction = 1 if label.endswith("+") else -1
            segments.append(
                Segment(
                    label,
                    int(label.split(":")[1]),
                    recording.position[[at], j],
                    np.zeros(1),
                    recording.torque[[at], j],
                    recording.target[[at], j],
                    direction,
                )
            )
    return segments


def fit_recordings(
    tree: RigidBodyTree,
    holds: Sequence[Hold],
    trajectories: Sequence[Trajectory],
) -> DynamicsFit:
    joints = sorted(
        {j for hold in holds for j in hold.torque} | {j for t in trajectories for j in t.joints}
    )
    ordered = tuple(axis for axis in tree.axes if axis in joints)
    return fit(tree, ordered, holds, trajectories, dict.fromkeys(tree.axes, 0.0))


def report(found: DynamicsFit) -> list[str]:
    lines = [
        f"base parameters seen {found.base_parameters}, condition number {found.condition:.0f}",
        "joint                         hold RMS N·m  trajectory RMS N·m  accept <=  coulomb  viscous  armature",
    ]
    base = 10 * len(found.tree.links)
    for joint in found.joints:
        at = base + 3 * found.tree.axes.index(joint)
        coulomb, viscous, armature = found.theta[at : at + 3]
        accept = ACCEPT_SHARE * PEAK.get(joint, 7.0)
        hold = found.hold_residual.get(joint, float("nan"))
        lines.append(
            f"{joint:30} {hold:12.3f} {found.trajectory_residual.get(joint, float('nan')):19.3f}"
            f" {accept:9.2f} {coulomb:8.3f} {viscous:8.3f} {armature:9.4f}"
            + ("" if hold <= accept else "  OVER")
        )
    if found.reverted:
        lines.append(
            f"links kept at the design estimate (unphysical fit): {', '.join(found.reverted)}"
        )
    return lines


def fit_friction(urdf: bytes, paths: Sequence[Path], units: Sequence[str], out: Path) -> None:
    """Every joint swept in ``paths``: reject, fit, choose a model, report, and seal the joint
    friction as a ``dynamics`` calibration (links untouched)."""
    by_joint: dict[str, list[Segment]] = {}
    for path in paths:
        recording = Recording.read(path)
        swept = {label.split(":")[0] for label in recording.segments}
        if not swept & {"sweep", "break"}:
            raise SystemExit(f"{path} holds no sweep or breakaway")
        joint = _swept_joint(recording)
        by_joint.setdefault(joint, []).extend(friction_segments(recording, joint))
    frictions: list[JointFriction] = []
    for joint, segments in sorted(by_joint.items()):
        found = fit_joint(joint, segments, SegmentLimits(peak_torque=PEAK.get(joint, 7.0)))
        frictions.append(found.friction)
        print(
            f"{joint}: {found.model}, fitted RMS {found.fitted:.3f} N·m, "
            f"forward/backward gravity gap {found.direction_gap:.3f} N·m"
        )
        print("  held out: " + ", ".join(f"{m} {e:.3f}" for m, e in found.held_out.items()))
        print(
            "  "
            + ", ".join(f"{name} {getattr(found.friction, name):.4g}" for name in FRICTION_TERMS)
        )
        total = len(segments)
        print(f"  rejected {len(found.rejected)} of {total} segments")
        for rejection in found.rejected:
            print(f"    {rejection.label}: {rejection.reason}")
    calibration = DynamicsCalibration(
        description_sha256=Sha256Digest(hashlib.sha256(urdf).hexdigest()),
        stiction_velocity=STICTION_VELOCITY,
        links=(),
        joints=tuple(frictions),
        units=tuple(sorted({UnitSerial(unit) for unit in units})),
        measurement=Measurement(CalibrationMethod.DYNAMICS_IDENTIFICATION, datetime.now(UTC)),
    )
    out.write_text(json.dumps(calibration_to_dict(calibration), indent=1) + "\n")
    print(f"wrote {out}: calibration {calibration_id(calibration)}")


def _swept_joint(recording: Recording) -> str:
    """The joint a sweep or breakaway recording moved: the one whose target travelled most in
    its recorded segments."""
    rows = np.array(
        [i for i, s in enumerate(recording.segments) if s.startswith(("sweep:", "break:"))]
    )
    travel = np.ptp(recording.target[rows], axis=0)
    return recording.joints[int(np.argmax(travel))]


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    commands = parser.add_subparsers(dest="command", required=True)
    plan = commands.add_parser("plan", help="write the identification plan")
    plan.add_argument("--out", type=Path, default=PLAN)
    fitting = commands.add_parser("fit", help="fit recordings into a dynamics calibration")
    fitting.add_argument("--plan", type=Path, default=PLAN)
    fitting.add_argument("--holds", type=Path, required=True)
    fitting.add_argument("--excitation", type=Path, action="append", default=[])
    fitting.add_argument("--unit", action="append", default=[], help="a fitted motor's serial")
    fitting.add_argument("--out", type=Path, required=True)
    sweeping = commands.add_parser("sweep-plan", help="write the in-place friction sweep plan")
    sweeping.add_argument("--out", type=Path, default=SWEEP_PLAN)
    friction = commands.add_parser("fit-friction", help="fit sweeps into joint friction")
    friction.add_argument(
        "--recording",
        type=Path,
        action="append",
        required=True,
        help="sx-identify's CSV of a sweep:<joint> or breakaway:<joint> part",
    )
    friction.add_argument("--unit", action="append", default=[], help="a fitted motor's serial")
    friction.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    urdf = URDF.read_bytes()
    if args.command == "sweep-plan":
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(make_sweep_plan(urdf), indent=1) + "\n")
        print(f"wrote {args.out}")
        return
    if args.command == "fit-friction":
        fit_friction(urdf, args.recording, args.unit, args.out)
        return
    if args.command == "plan":
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(make_plan(urdf), indent=1) + "\n")
        print(f"wrote {args.out}")
        return
    plan_document = read_plan(args.plan)
    if plan_document["description_sha256"] != hashlib.sha256(urdf).hexdigest():
        raise SystemExit("the plan was made for another description; run plan again")
    tree = RigidBodyTree.of(urdf)
    holds, still = read_holds(args.holds, float(plan_document["holds"]["record_s"]))  # type: ignore[index]
    moving = {joint: speed for joint, speed in still.items() if speed > 0.02}
    if moving:
        print(f"warning: joints moved while held (rad/s): {moving}")
    trajectories = [read_excitation(path) for path in args.excitation]
    found = fit_recordings(tree, holds, trajectories)
    print("\n".join(report(found)))
    calibration = found.calibration(
        hashlib.sha256(urdf).hexdigest(),
        tuple(UnitSerial(unit) for unit in args.unit),
        datetime.now(UTC),
    )
    args.out.write_text(json.dumps(calibration_to_dict(calibration), indent=1) + "\n")
    print(f"wrote {args.out}: calibration {calibration_id(calibration)}")


if __name__ == "__main__":
    main()
