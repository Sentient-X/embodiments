//! Every registered Damiao axis, alone on a simulated bus running its MIT law, answers a small
//! step at its rated limits as a damped joint does: it never runs away, never rings, and
//! settles at the target (short of it only by what Coulomb friction holds against `kp`). Both
//! simulated joints: the station's (`Sim::with_model_dynamics`, the bindings' inertia, no
//! friction) and the bench's (`Sim::with_mit_dynamics`). The humanoid's wrists, 1.05e-4 kg m^2
//! under kd 5, once ran away here within one tick (#3004): explicit Euler at 1 ms is stable
//! only for `kd dt / J` below 2, and theirs is near 48.

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
/// Within this of where it should be, motor rad, the joint counts as there.
const DEADBAND: f64 = 1e-3;

/// What one axis did with a small step, in motor coordinates.
struct Answer {
    /// The motor's position after each tick.
    positions: Vec<f64>,
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

/// One axis on its own bus: command a small step toward the middle of its range and watch it.
fn step(axis: &DamiaoAxis, bench: bool) -> Answer {
    let one = std::slice::from_ref(axis);
    let sim = Sim::for_axes(one);
    let bus = if bench {
        sim.with_mit_dynamics()
    } else {
        sim.with_model_dynamics(one)
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
    let mut positions = Vec::with_capacity(TICKS);
    for _ in 0..TICKS {
        governor
            .command(&[target])
            .unwrap_or_else(|error| panic!("{}: refused a {size} step: {error}", axis.axis.joint));
        clock.0.set(clock.0.get() + MAX_STEP_INTERVAL);
        bus.advance_time(MAX_STEP_INTERVAL);
        positions.push(bus.state().motors[&id].q);
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

fn check(bench: bool) {
    let axes = every_axis();
    assert!(axes.len() > 30, "only {} axes", axes.len());
    for (body, axis) in &axes {
        let Answer {
            positions,
            start,
            target,
            held,
            overshoot,
            residual,
        } = step(axis, bench);
        let span = (target - start).abs();
        let toward = (target - start).signum();
        let kind = if bench { "bench" } else { "model" };
        let name = format!("{body}/{} ({kind})", axis.axis.joint);
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
        // Not growing: each swing past the target is smaller than the one before, to within
        // what sampling at the control period misses of a swing's peak.
        let mut swings: Vec<f64> = Vec::new();
        let mut side = None;
        for error in positions.iter().map(|q| q - target) {
            if error.abs() <= held + DEADBAND {
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
            swings.windows(2).all(|pair| pair[1] < pair[0] + DEADBAND),
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
    check(false);
}

#[test]
fn every_axis_steps_cleanly_on_the_bench_joint() {
    check(true);
}

/// The humanoid's wrists are why: light links under the strongest damping the MIT frame
/// carries (kd 5).
#[test]
fn the_humanoid_wrists_are_the_light_firmly_damped_case() {
    let axes = every_axis();
    let wrist = axes
        .iter()
        .find(|(_, axis)| axis.axis.joint == "right_wrist_yaw_joint")
        .map(|(_, axis)| axis)
        .expect("the humanoid's right wrist");
    let explicit = wrist.kd * 0.001 / wrist.axis.motion.inertia;
    assert!(
        explicit > 2.0,
        "kd dt / J is {explicit}: no longer the hard case"
    );
}
