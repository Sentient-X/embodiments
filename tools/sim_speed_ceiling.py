"""How fast can the static upper body move, and what stops it first: motor, gains, loop or bus?

Each arm joint is moved alone, from the pose where it carries the most (every joint at zero:
both arms straight out ahead, the shoulders and elbows lifting the whole arm against gravity),
by the governor's own trajectory: each tick the position sent steps toward the target at most
the joint's speed limit, its speed changing by at most its acceleration limit and never past
the speed it can still brake from (``sx_embodiment_drivers::governor::slew``, ported line for
line below). The limits are the registry's (``sx_embodiments.calibrate.motion``): the motor's
rated speed and the acceleration its torque headroom gives over the arm's inertia.

The robot is ``sim_joint_tracking``'s truth model: the description's masses scaled 0.7..1.3 at
random, BAM friction per motor model, and each Damiao motor running its MIT law at 1 kHz,
saturated at peak torque, under the prototype gains ``src/static_upper_body.rs`` carries. On
top of that every link here is made :data:`HEAVY` times heavier: identification on the real
robot (2026-10-26) put the forearm subtree at 2.61 times the description's mass, and a heavier
arm is the conservative side for speed.

The governor's tracking check compares what a command's answer reports with the position that
command just sent, at every tick. This prints that error's peak per joint, beside the limit
``sx_embodiment_drivers::AxisLimits::rated`` would give it, the settled error half a second
after the move, and the peak torque as a share of the motor's peak, for:

1. ``v_des = 0``: the governor sending positions only (today's every path but teleop with a
   dynamics model);
2. ``v_des`` = the governor's step speed (what ``Governor`` now sends every Damiao axis);
3. 2. plus the description's gravity and inertia as feed-forward (teleop with dynamics).

at the station's 50 Hz and at 100 Hz. Run from the repository root with ``mujoco`` installed::

    python tools/sim_speed_ceiling.py [--markdown docs/evidence/speed-ceiling/README.md]
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import sim_joint_tracking as sim  # noqa: E402

from sx_embodiments import development_embodiments  # noqa: E402
from sx_embodiments.calibrate.dynamics import RigidBodyTree  # noqa: E402
from sx_embodiments.calibrate.friction import budget  # noqa: E402
from sx_embodiments.calibrate.motion import (  # noqa: E402
    TRACKING_MEMORY_S,
    joint_motion,
    tracking_bound,
)
from sx_embodiments.known.static_upper_body import (  # noqa: E402
    STATIC_UPPER_BODY_MASS_MARGIN,
    STATIC_UPPER_BODY_SERVO_APPROACH,
    STATIC_UPPER_BODY_SPEED_CAP,
)

#: The real forearm's measured mass over the description's (dynamics.md, 2026-10-26).
HEAVY = 2.61
#: ``sx_damiao_can::chain::{MAX_COAST, MAX_LAG}``: the planned velocity coasts at most
#: MAX_COAST past the last position if commands stop, and each axis is no faster than
#: ``(MAX_COAST + MAX_LAG) kp / kd``.
MAX_COAST = 0.1
MAX_LAG = 0.1
#: How far each joint is moved: far enough to reach full speed at full acceleration.
TRAVEL = 1.2


@dataclass(frozen=True)
class Limit:
    speed: float
    acceleration: float


def slew(value: float, target: float, speed: float, limit: Limit, dt: float) -> tuple[float, float]:
    """``sx_embodiment_drivers::governor::slew`` without the range: position and step speed."""
    gap = target - value
    wanted = max(-limit.speed, min(limit.speed, gap / dt))
    braking = math.sqrt(2.0 * limit.acceleration * abs(gap))
    change = limit.acceleration * dt
    wanted = max(-braking, min(braking, wanted))
    wanted = max(speed - change, min(speed + change, wanted))
    nxt = value + wanted * dt
    return nxt, (nxt - value) / dt


def heavy(truth: sim.Truth) -> sim.Truth:
    model = truth.model
    model.body_mass[:] *= HEAVY
    model.body_inertia[:] *= HEAVY
    return truth


def move(
    robot: sim.Robot,
    joint: str,
    limit: Limit,
    tick: float,
    velocity: bool,
    theta: np.ndarray | None,
    stall: bool = False,
) -> dict[str, float]:
    """One joint from zero by :data:`TRAVEL` (toward its wider side) and back, all others held
    at zero; the governor's tracking error at every tick, settle, peak torque."""
    index = sim.JOINTS.index(joint)
    lo, hi = robot.tree.limits[joint] or (-math.pi, math.pi)
    sign = 1.0 if hi >= -lo else -1.0
    reach = 0.9 * (hi if sign > 0 else -lo)
    # A stall is measured at the joint's top speed, so it gets its whole range to reach it.
    travel = reach if stall else min(TRAVEL, reach)
    q0 = np.zeros(len(sim.JOINTS))
    robot.place(q0)
    model, data = robot.model, robot.data
    kp, kd = robot.kp, robot.kd
    steps = round(tick / sim.SIM_DT)
    sent = q0.copy()
    speed = 0.0
    held = sent.copy()
    v_des = np.zeros_like(sent)
    tau_ff = np.zeros_like(sent)
    worst = 0.0
    ratio = 0.0
    bound_at = 0.0
    recent = 0.0
    peak = 0.0
    settled = 0.0
    t = 0.0
    plan_end = None
    step = 0
    # Settle 1 s at zero under the controller, then out, then back, then settle 0.5 s.
    schedule = [(1.0, 0.0), (None, sign * travel), (None, 0.0)]
    phase = 0
    phase_started = 0.0
    stalled_at: float | None = None
    coast = 0.0
    stall_speed = 0.0
    fastest = 0.0
    while True:
        if stalled_at is not None:
            # The host stopped commanding: the motor holds its last frame until its watchdog.
            past = sign * (data.qpos[robot.dofs][index] - sent[index])
            coast = max(coast, past)
            if t - stalled_at >= 0.75:
                break
        elif (
            stall
            and phase == 1
            and (abs(speed) >= limit.speed - 1e-9 or abs(speed) < fastest - 1e-9)
        ):
            # At its top speed, or the fastest its range lets it reach (as it starts to brake).
            stalled_at, stall_speed = t, abs(speed)
        fastest = max(fastest, abs(speed)) if phase == 1 else fastest
        if stalled_at is None and step % steps == 0:
            hold_for, target = schedule[phase]
            if hold_for is not None and t - phase_started >= hold_for:
                phase += 1
                phase_started = t
                hold_for, target = schedule[phase]
            q_now = data.qpos[robot.dofs].copy()
            nxt, speed = slew(sent[index], target, speed, limit, tick)
            # The governor's check: the answer to this command reports where the joint is now,
            # against the position it just sent, within a bound widened by the recent speed.
            recent = max(abs(speed), recent * math.exp(-tick / TRACKING_MEMORY_S))
            if phase > 0:
                error = abs(q_now[index] - nxt)
                bound = tracking_bound(recent)
                worst = max(worst, error)
                if error / bound > ratio:
                    ratio, bound_at = error / bound, bound
            sent = held.copy()
            sent[index] = nxt
            v_des = np.zeros_like(sent)
            if velocity:
                # DamiaoChain's cap: never a planned velocity that coasts past MAX_COAST.
                cap = MAX_COAST * kp[index] / kd[index]
                v_des[index] = max(-cap, min(cap, speed))
            if theta is not None:
                qdd = np.zeros_like(sent)
                tau_ff = robot.feedforward(
                    sim.Controller("ff", 1.0, 1.0, True, theta), q_now, sent, v_des, qdd
                )
            if hold_for is None and abs(sent[index] - target) < 1e-9 and speed == 0.0:
                if phase == len(schedule) - 1:
                    if plan_end is None:
                        plan_end = t
                    elif t - plan_end >= 0.5:
                        settled = abs(q_now[index] - target)
                        break
                else:
                    phase += 1
                    phase_started = t
        q = data.qpos[robot.dofs]
        v = data.qvel[robot.dofs]
        tau = np.clip(kp * (sent - q) + kd * (v_des - v) + tau_ff, -robot.peak, robot.peak)
        peak = max(peak, abs(tau[index]) / robot.peak[index])
        data.qfrc_applied[:] = 0.0
        data.qfrc_applied[robot.dofs] = tau
        for i, drive in enumerate(robot.drives):
            model.dof_frictionloss[robot.dofs[i]] = float(
                budget(drive, np.array([v[i]]), np.array([tau[i]]), np.array([-tau[i]]))[0]
            )
        mujoco.mj_step(model, data)
        t += sim.SIM_DT
        step += 1
        if t > 30.0:
            break
    return {
        "coast": coast,
        "stall_speed": stall_speed,
        "tracking": worst,
        "ratio": ratio,
        "bound": bound_at,
        "settled": settled,
        "peak": peak,
    }


def tcp_speed(robot: sim.Robot, limits: dict[str, Limit], side: str) -> tuple[float, float]:
    """The fastest straight-line TCP speed at the reach pose (every joint at zero), the
    slowest over the six axis directions, and the fastest single direction: each joint held
    to its speed limit, the joint velocities the least-squares IK of the direction scaled
    until the first joint meets its limit."""
    model, data = robot.model, robot.data
    robot.place(np.zeros(len(sim.JOINTS)))
    body = robot.tcp[0 if side == "left" else 1]
    jacp = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, None, body)
    arm = [j for j in sim.JOINTS if j.startswith(side + "_")]
    cols = [model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in arm]
    jac = jacp[:, cols]
    caps = np.array([limits[j].speed for j in arm])
    best = []
    for direction in np.vstack([np.eye(3), -np.eye(3)]):
        # Scale-weighted pseudo-inverse: the joint velocities for 1 m/s along the direction.
        weighted = jac * caps
        qd = caps * (np.linalg.pinv(weighted) @ direction)
        best.append(1.0 / np.max(np.abs(qd) / caps))
    return min(best), max(best)


def watchdog_window(speed: float, cap: float | None, window: float, gravity: float = 0.0) -> float:
    """The elbow (J 0.124 kg m^2, kp 30.492, kd 4.655, Coulomb 1.4 and viscous 0.3 N m s/rad,
    27 N m peak) tracking at ``speed`` when the host stalls: it holds its last frame for
    ``window`` (its TIMEOUT), then has no torque. Returns the farthest it goes past the last
    position sent within 2 s, with ``gravity`` N m pulling along the motion once limp."""
    inertia, kp, kd, coulomb, viscous, peak = 0.124, 30.492, 4.655, 1.4, 0.3, 27.0
    step, tick = 1e-4, 0.02
    v_des = speed if cap is None else min(speed, cap)
    q, w, t, due, p_des = 0.0, speed, 0.0, 0.0, 0.0
    while t < 1.0:
        if t >= due:
            p_des, due = speed * (t + tick), due + tick
        torque = max(-peak, min(peak, kp * (p_des - q) + kd * (v_des - w)))
        friction = -coulomb * math.copysign(1.0, w) if abs(w) > 1e-3 else 0.0
        w += (torque + friction - viscous * w) / inertia * step
        q += w * step
        t += step
    stalled, farthest = t, -math.inf
    while t < stalled + 2.0:
        held = t < stalled + window
        torque = max(-peak, min(peak, kp * (p_des - q) + kd * (v_des - w))) if held else 0.0
        load = 0.0 if held else gravity
        if not held and abs(w) <= 1e-3 and load <= coulomb:
            w = 0.0
        friction = -coulomb * math.copysign(1.0, w) if abs(w) > 1e-3 else 0.0
        w += (torque + load + friction - viscous * w) / inertia * step
        q += w * step
        t += step
        farthest = max(farthest, q - p_des)
    return farthest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--markdown", type=Path)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--fraction", type=float, default=1.0)
    parser.add_argument("--joints", default="left_,torso")
    args = parser.parse_args()
    urdf = sim.URDF.read_bytes()
    tree = RigidBodyTree.of(urdf)
    motions = joint_motion(
        development_embodiments["sentient-static-upper-body"],
        STATIC_UPPER_BODY_SERVO_APPROACH,
        STATIC_UPPER_BODY_MASS_MARGIN,
        STATIC_UPPER_BODY_SPEED_CAP,
    )

    def ceiling(joint: str) -> tuple[float, float]:
        # DamiaoChain::tracking_ceilings: the gains' speed and, over the joint's inertia, the
        # acceleration their position term follows within MAX_LAG.
        # The speed is also held to MAX_COAST sqrt(kp / J): the momentum a held position stops.
        _, kp, kd = sim.MOTORS[joint]
        inertia = motions[joint].inertia
        peak = sim.PEAK[sim.MOTORS[joint][0]]
        speed = min(
            (MAX_COAST + MAX_LAG) * kp / kd,
            MAX_COAST * math.sqrt(kp / inertia),
            math.sqrt(2.0 * MAX_COAST * peak / inertia),
        )
        return speed, kp * MAX_LAG / inertia

    limits = {
        j: Limit(
            min(motions[j].speed, ceiling(j)[0]) * args.fraction,
            min(motions[j].acceleration, ceiling(j)[1]) * args.fraction,
        )
        for j in sim.JOINTS
    }
    theta = tree.parameters()
    lines = [
        f"Limits at {args.fraction:.0%} of the registry's (sx_embodiments.calibrate.motion), truth "
        f"{HEAVY}x the description's mass, prototype gains.",
        "",
        "| Joint | Speed rad/s | Accel rad/s² | Hz | Controller | Peak tracking rad "
        "| Worst share of bound | Settled rad | Peak torque |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    wanted = tuple(args.joints.split(","))
    for hz in (50, 100):
        tick = 1.0 / hz
        sim.TICK = tick
        for joint in sim.JOINTS:
            if not joint.startswith(wanted):
                continue
            limit = limits[joint]
            for name, velocity, model_theta in (
                ("v_des 0", False, None),
                ("v_des", True, None),
                ("v_des + model", True, theta),
            ):
                truth = heavy(sim.truth_model(urdf, tree, args.seed))
                robot = sim.Robot(truth, tree, args.seed)
                result = move(robot, joint, limit, tick, velocity, model_theta)
                flag = "" if result["ratio"] <= 1.0 else " **trips**"
                lines.append(
                    f"| {joint} | {limit.speed:.2f} | {limit.acceleration:.1f} | {hz} | {name} | "
                    f"{result['tracking']:.3f} | {result['ratio']:.0%}{flag} | "
                    f"{result['settled']:.3f} | {result['peak']:.0%} |"
                )
                print(lines[-1], flush=True)  # noqa: T201
    lines += [
        "",
        "If the host stops commanding when the joint moves at its top governed speed (or the "
        "fastest its range lets it reach), the v_des controller at 50 Hz: how far past the last "
        "position it goes before its 0.75 s watchdog, momentum and gravity included:",
        "",
        "| Joint | Speed at the stall rad/s | Coast rad |",
        "| --- | --- | --- |",
    ]
    for joint in sim.JOINTS:
        if not joint.startswith(wanted):
            continue
        sim.TICK = 0.02
        truth = heavy(sim.truth_model(urdf, tree, args.seed))
        robot = sim.Robot(truth, tree, args.seed)
        result = move(robot, joint, limits[joint], 0.02, True, None, stall=True)
        lines.append(f"| {joint} | {result['stall_speed']:.2f} | {result['coast']:.3f} |")
        print(lines[-1], flush=True)  # noqa: T201
    lines += [
        "",
        "How far the elbow goes past its last position after a host stall, by how long its "
        "TIMEOUT lets it hold the last frame (single joint, `watchdog_window`), rad:",
        "",
        "| Speed rad/s | Planned velocity | 0.1 s | 0.3 s | 0.75 s | 0.1 s, 4.5 N m along |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for speed in (1.31, 3.77):
        for cap, name in ((None, "uncapped"), (MAX_COAST * 30.492 / 4.655, "capped")):
            cells = [f"{watchdog_window(speed, cap, window):.3f}" for window in (0.1, 0.3, 0.75)]
            limp = watchdog_window(speed, cap, 0.1, gravity=4.5)
            cells.append("falls to its stop" if limp > math.pi else f"{limp:.3f}")
            lines.append(f"| {speed:.2f} | {name} | " + " | ".join(cells) + " |")
    truth = heavy(sim.truth_model(urdf, tree, args.seed))
    robot = sim.Robot(truth, tree, args.seed)
    lines.append("")
    for side in ("left", "right"):
        slow, fast = tcp_speed(robot, limits, side)
        lines.append(
            f"{side} arm TCP at the reach pose: {slow:.2f} m/s in its slowest axis direction, "
            f"{fast:.2f} m/s in its fastest."
        )
    text = "\n".join(lines) + "\n"
    print(text)  # noqa: T201
    if args.markdown is not None:
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(text)


if __name__ == "__main__":
    main()
