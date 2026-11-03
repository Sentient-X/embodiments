//! Every registered Damiao axis, alone on a simulated bus running its MIT law, answers a small
//! step at its rated limits as a damped joint does: it never runs away, its swings shrink, and
//! it settles at the target (short of it only by what Coulomb friction holds against `kp`).
//! Three simulated joints: the station's (`Sim::with_model_dynamics`, the bindings' inertia
//! with the rotor, no friction), the bench's (`Sim::with_mit_dynamics`), and the station's with
//! each joint's inertia at the lightest link the registry carries, 1e-4 kg m^2 with no rotor.
//! That last is #3004: the humanoid's wrists, so light under kd 5, ran away within one tick
//! when explicit Euler at 1 ms (stable only for `kd dt / J` below 2) stepped them. Each joint
//! is sampled every millisecond, the integration step, so no flip-flop hides between ticks.

use std::cell::Cell;
use std::rc::Rc;
use std::time::Instant;

use serde_json::Value;
use sx_damiao_can::body::BODIES;
use sx_damiao_can::chain::{DamiaoAxis, DamiaoChain};
use sx_damiao_can::sim::Sim;
use sx_embodiment_drivers::{
    ActuatorChain, AxisLimits, Clock, Governor, InstanceDrive, MAX_STEP_INTERVAL,
};

fn drive(body: &str) -> InstanceDrive {
    let drives: Value = serde_json::from_str(include_str!("fixtures/drives.json")).expect("drives");
    let nominal = &drives[body]["nominal"];
    InstanceDrive::parse(
        &nominal.to_string(),
        nominal["instance"].as_str().expect("an instance"),
    )
    .expect("a drive")
}

/// Every Damiao axis the registry binds, by body.
fn every_axis() -> Vec<(String, DamiaoAxis)> {
    let mut axes: Vec<(String, DamiaoAxis)> = sx_damiao_can::b601::b601(&drive("b601-dm"))
        .expect("b601")
        .into_iter()
        .map(|axis| ("b601-dm".to_owned(), axis))
        .collect();
    for body in BODIES {
        for axis in body.axes(&drive(body.embodiment)).expect("axes") {
            axes.push((body.embodiment.to_owned(), axis));
        }
    }
    axes
}

#[derive(Clone)]
struct Manual(Rc<Cell<Instant>>);

impl Clock for Manual {
    fn now(&self) -> Instant {
        self.0.get()
    }
}

/// The step's size at the joint, rad (or m for a prismatic), toward the middle of its range:
/// this, or a twentieth of the range if that is less.
const STEP: f64 = 0.02;
/// How long the joint is watched after the step: 3 s at the control period.
const TICKS: usize = 150;
/// The simulation's step, at which each joint is sampled.
const SAMPLE: std::time::Duration = std::time::Duration::from_millis(1);
/// The lightest link inertia any registered joint declared before its rotor was counted: the
/// humanoid's wrist yaw, kg m^2.
const LIGHTEST: f64 = 1e-4;
/// Within this of where it should be, motor rad, the joint counts as there.
const DEADBAND: f64 = 1e-3;

/// What one axis did with a small step, in motor coordinates.
struct Answer {
    /// The motor's position every millisecond, and the MIT frame's position and planned
    /// velocity it was driven by over that millisecond.
    positions: Vec<f64>,
    frames: Vec<(f64, f64)>,
    start: f64,
    target: f64,
    /// How far short of the target the joint may rest: Coulomb friction, and the quantised
    /// planned velocity and feed-forward of the MIT frame, against `kp`, plus the frame's
    /// position quantisation.
    held: f64,
    /// The share of a step the continuous joint overshoots by, `exp(-pi zeta / sqrt(1 -
    /// zeta^2))` underdamped and none from critical damping on.
    overshoot: f64,
    /// The share of a step the continuous joint has still to go when it is last watched: its
    /// slowest mode, `exp(-rate t)`.
    residual: f64,
}

/// The simulated joint an axis is stepped on.
#[derive(Clone, Copy, Debug)]
enum Joint {
    /// The station's: the bindings' inertia, rotor included, no friction.
    Model,
    /// The bench's: each model's loaded joint, with friction.
    Bench,
    /// The station's at [`LIGHTEST`], no rotor: the light, firmly damped case.
    Light,
}

/// One axis on its own bus: command a small step toward the middle of its range and watch it.
fn step(axis: &DamiaoAxis, joint: Joint) -> Answer {
    let mut axis = axis.clone();
    if matches!(joint, Joint::Light) {
        axis.axis.motion.inertia = LIGHTEST;
    }
    let axis = &axis;
    let one = std::slice::from_ref(axis);
    let sim = Sim::for_axes(one);
    let bus = match joint {
        Joint::Bench => sim.with_mit_dynamics(),
        Joint::Model | Joint::Light => sim.with_model_dynamics(one),
    }
    .with_manual_time();
    let chain = DamiaoChain::open(bus.clone(), one.to_vec()).expect("opens");
    let limits = AxisLimits::rated_all(chain.axes(), 1.0).expect("rated");
    let clock = Manual(Rc::new(Cell::new(Instant::now())));
    let mut governor = Governor::with_clock(chain, limits, clock.clone()).expect("governed");
    let rest = governor.last_observed().expect("rest")[0];
    let middle = f64::midpoint(axis.axis.lower, axis.axis.upper);
    let size = STEP.min((axis.axis.upper - axis.axis.lower) / 20.0);
    let target = size.copysign(middle - rest) + rest;
    let id = axis.axis.binding.bus_id;
    let mut positions = Vec::with_capacity(TICKS * 20);
    let mut frames = Vec::with_capacity(TICKS * 20);
    for _ in 0..TICKS {
        governor
            .command(&[target])
            .unwrap_or_else(|error| panic!("{}: refused a {size} step: {error}", axis.axis.joint));
        clock.0.set(clock.0.get() + MAX_STEP_INTERVAL);
        let mut passed = std::time::Duration::ZERO;
        while passed < MAX_STEP_INTERVAL {
            let mit = bus.state().motors[&id].dynamics.expect("dynamics");
            frames.push((mit.p_des, mit.v_des));
            bus.advance_time(SAMPLE);
            passed += SAMPLE;
            positions.push(bus.state().motors[&id].q);
        }
    }
    let target = axis.axis.actuator_from_joint(target);
    let joint = bus.state().motors[&id].dynamics.expect("dynamics");
    let zeta = (joint.kd + joint.damping) / (2.0 * (joint.kp * joint.inertia).sqrt());
    let natural = (joint.kp / joint.inertia).sqrt();
    let rate = if zeta < 1.0 {
        zeta * natural
    } else {
        natural * (zeta - zeta.mul_add(zeta, -1.0).sqrt())
    };
    #[allow(clippy::cast_precision_loss)] // a tick count
    let watched = MAX_STEP_INTERVAL.as_secs_f64() * TICKS as f64;
    Answer {
        positions,
        frames,
        start: axis.axis.actuator_from_joint(rest),
        target,
        held: (joint.coulomb + (joint.kd * joint.v_des).abs() + joint.tau_ff.abs()) / joint.kp
            + (joint.p_des - target).abs(),
        overshoot: if zeta < 1.0 {
            (-std::f64::consts::PI * zeta / (1.0 - zeta * zeta).sqrt()).exp()
        } else {
            0.0
        },
        residual: (-rate * watched).exp(),
    }
}

fn check(joint: Joint) {
    let axes = every_axis();
    assert!(axes.len() > 30, "only {} axes", axes.len());
    for (body, axis) in &axes {
        let Answer {
            positions,
            frames: _,
            start,
            target,
            held,
            overshoot,
            residual,
        } = step(axis, joint);
        let span = (target - start).abs();
        let toward = (target - start).signum();
        let name = format!("{body}/{} ({joint:?})", axis.axis.joint);
        for q in &positions {
            // Bounded: never back past where it started, nor past the target by more than
            // the continuous joint overshoots a step plus half the step for the governor's
            // ramp and planned velocity (the B601-DM's joint1, damping ratio 0.4, overshoots
            // by 37 %, the humanoid's torso, 0.15, by 63 %). #3004's wrists, overdamped, went
            // 200 steps past.
            let past = (q - target) * toward;
            assert!(
                past <= (overshoot + 0.5) * span + held + DEADBAND
                    && (q - start) * toward >= -DEADBAND,
                "{name}: at {q}, a {span} step from {start} to {target} ran away: {positions:?}"
            );
        }
        // Not growing: no swing past the target is more than 5 % above the one before (the
        // governor's quantised commands nudge a lightly damped joint's late swings by about
        // that; a steady oscillation fails the settling bound below). Swings within twice the
        // deadband of the friction band are rest, not ringing.
        let mut swings: Vec<f64> = Vec::new();
        let mut side = None;
        for error in positions.iter().map(|q| q - target) {
            if error.abs() <= held + 2.0 * DEADBAND {
                continue;
            }
            if side == Some(error > 0.0) {
                let last = swings.last_mut().expect("a swing");
                *last = last.max(error.abs());
            } else {
                side = Some(error > 0.0);
                swings.push(error.abs());
            }
        }
        assert!(
            swings.windows(2).all(|pair| pair[1] <= pair[0] * 1.05),
            "{name}: its swings grow, {swings:?}: {positions:?}"
        );
        // Settled as the continuous joint has, short only by what friction and the frame hold.
        let last = positions.last().expect("a tick");
        assert!(
            (last - target).abs() <= (1.0 + overshoot) * residual * span + held + DEADBAND,
            "{name}: settled at {last}, not {target} (held {held}): {positions:?}"
        );
    }
}

#[test]
fn every_axis_steps_cleanly_on_the_model_the_station_drives() {
    check(Joint::Model);
}

#[test]
fn every_axis_steps_cleanly_on_the_bench_joint() {
    check(Joint::Bench);
}

/// #3004: the gains on a joint as light as the humanoid's wrist links alone.
#[test]
fn every_axis_steps_cleanly_on_the_lightest_link() {
    check(Joint::Light);
}

/// What a joint's step answer identifies it as: the MIT law fitted to its own readings and
/// frames, `q'' = omega^2 (p - q) + 2 zeta omega (v - q')`, by least squares over every
/// millisecond (the simulation's own semi-implicit step, so the fit is exact on it).
#[allow(clippy::many_single_char_names)] // the law's own names
fn identified(answer: &Answer) -> sx_embodiment_drivers::FollowerModel {
    let h = SAMPLE.as_secs_f64();
    let q = &answer.positions;
    let (mut saa, mut sab, mut sbb, mut say, mut sby) = (0.0, 0.0, 0.0, 0.0, 0.0);
    for k in 1..q.len() - 1 {
        let (v0, v1) = ((q[k] - q[k - 1]) / h, (q[k + 1] - q[k]) / h);
        let (p, v) = answer.frames[k];
        let (a, b, y) = (p - q[k + 1], v - v1, (v1 - v0) / h);
        saa += a * a;
        sab += a * b;
        sbb += b * b;
        say += a * y;
        sby += b * y;
    }
    let det = saa * sbb - sab * sab;
    let stiffness = (say * sbb - sby * sab) / det;
    let damping = (saa * sby - sab * say) / det;
    let omega = stiffness.sqrt();
    sx_embodiment_drivers::FollowerModel {
        omega,
        zeta: damping / (2.0 * omega),
        feedforward: true,
    }
}

/// Each Damiao axis's follower ([`DamiaoAxis::follower`]: its gains over the inertia its
/// bindings carry), identified from a small step on act-sim's motor model through the chain
/// and its governor: a ringing joint (damping ratio under 0.5) within 2 % in natural frequency
/// and 6 % in damping (the static upper body's shoulders and torso, 0.2-0.5 % and 3.4-4.8 %,
/// 2026-11-03); a well-damped one is printed (5-10 % slower, its wrists' damping 16 % lower).
/// `--nocapture` prints them.
#[test]
fn every_axis_identifies_as_its_follower_on_the_model_the_station_drives() {
    for (body, axis) in every_axis() {
        let Some(declared) = axis.follower() else {
            panic!("{body}/{}: no follower", axis.axis.joint);
        };
        let found = identified(&step(&axis, Joint::Model));
        println!(
            "{body}/{}: declared omega {:.2} zeta {:.3}, identified omega {:.2} zeta {:.3}",
            axis.axis.joint, declared.omega, declared.zeta, found.omega, found.zeta
        );
        // A ringing joint is where the prediction matters (a shoulder rings at zeta 0.1); a
        // well-damped one lands where it is sent whatever its exact rate, and a firmly damped
        // light wrist (kd dt / J near 1) is barely excited by a 1 ms sample.
        assert!(
            declared.zeta >= 0.5
                || ((found.omega / declared.omega - 1.0).abs() < 0.02
                    && (found.zeta / declared.zeta - 1.0).abs() < 0.06),
            "{body}/{}: identified {found:?}, declared {declared:?}",
            axis.axis.joint
        );
    }
}
