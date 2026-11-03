//! The static upper body's proposed damped gains (`STATIC_UPPER_BODY_DAMPED`) against its
//! commissioned ones, on act-sim's motor model (`Sim::with_model_dynamics`: the bindings'
//! inertia with the rotor, no friction). Each torso and shoulder joint is stepped 0.02 rad
//! through the governor at 50 Hz, and `step_response::fit` reads its follower back. The
//! commissioned joints ring at zeta 0.06-0.12; the proposed ones follow at 0.5. On the real
//! arm the same step is `sx-identify --gains damped --part step:<joint>`, and it waits for a
//! person at the robot.

use std::cell::Cell;
use std::rc::Rc;
use std::time::Instant;

use serde_json::Value;
use sx_damiao_can::body::DamiaoBody;
use sx_damiao_can::chain::DamiaoChain;
use sx_damiao_can::sim::Sim;
use sx_damiao_can::static_upper_body::{STATIC_UPPER_BODY, STATIC_UPPER_BODY_DAMPED};
use sx_embodiment_commission::step_response::{Fit, Sample, fit};
use sx_embodiment_drivers::{
    ActuatorChain, AxisLimits, Clock, Governor, InstanceDrive, MAX_STEP_INTERVAL,
};

fn drive() -> InstanceDrive {
    let drives: Value = serde_json::from_str(include_str!(
        "../../sx-damiao-can/tests/fixtures/drives.json"
    ))
    .expect("drives");
    let nominal = &drives["sentient-static-upper-body"]["nominal"];
    InstanceDrive::parse(
        &nominal.to_string(),
        nominal["instance"].as_str().expect("an instance"),
    )
    .expect("a drive")
}

#[derive(Clone)]
struct Manual(Rc<Cell<Instant>>);

impl Clock for Manual {
    fn now(&self) -> Instant {
        self.0.get()
    }
}

/// One joint of `body`, alone on act-sim's motor model, stepped 0.02 rad toward the middle of
/// its range and watched 3 s at 50 Hz; its fit, and the follower its gains make over the
/// simulated inertia `(omega, zeta)`.
fn stepped(body: &DamiaoBody, joint: &str) -> (Fit, (f64, f64)) {
    let axis = body
        .axes(&drive())
        .expect("axes")
        .into_iter()
        .find(|a| a.axis.joint == joint)
        .expect(joint);
    let one = std::slice::from_ref(&axis);
    let bus = Sim::for_axes(one)
        .with_model_dynamics(one)
        .with_manual_time();
    let chain = DamiaoChain::open(bus.clone(), one.to_vec()).expect("opens");
    let limits = AxisLimits::rated_all(chain.axes(), 1.0).expect("rated");
    let clock = Manual(Rc::new(Cell::new(Instant::now())));
    let mut governor = Governor::with_clock(chain, limits, clock.clone()).expect("governed");
    let rest = governor.last_observed().expect("rest")[0];
    let middle = f64::midpoint(axis.axis.lower, axis.axis.upper);
    let target = 0.02_f64.copysign(middle - rest) + rest;
    let id = axis.axis.binding.bus_id;
    let mut samples = Vec::new();
    for tick in 0..150_u32 {
        let goal = if tick < 10 { rest } else { target };
        governor.command(&[goal]).expect("a step");
        let (p_des, v_des, q) = {
            let state = bus.state();
            let motor = &state.motors[&id];
            let dynamics = motor.dynamics.expect("dynamics");
            (dynamics.p_des, dynamics.v_des, motor.q)
        };
        samples.push(Sample {
            t: f64::from(tick) * MAX_STEP_INTERVAL.as_secs_f64(),
            target: p_des,
            velocity: v_des,
            position: q,
        });
        clock.0.set(clock.0.get() + MAX_STEP_INTERVAL);
        bus.advance_time(MAX_STEP_INTERVAL);
    }
    let joint = bus.state().motors[&id].dynamics.expect("dynamics");
    let omega = (joint.kp / joint.inertia).sqrt();
    let zeta = (joint.kd + joint.damping) / (2.0 * (joint.kp * joint.inertia).sqrt());
    (fit(&samples).expect("a fit"), (omega, zeta))
}

const RINGING: [&str; 5] = [
    "torso_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
];

/// The step reads each follower back within 10 % in omega and 20 % in zeta (sampled at 50 Hz), commissioned and
/// proposed alike; the proposed ones are damped at 0.45 or more, the commissioned at 0.15 or
/// less.
#[test]
fn the_proposed_gains_damp_the_torso_and_shoulders_as_a_step_reads_them() {
    for (name, body, damped) in [
        ("commissioned", &STATIC_UPPER_BODY, false),
        ("proposed", &STATIC_UPPER_BODY_DAMPED, true),
    ] {
        for joint in RINGING {
            let (got, (omega, zeta)) = stepped(body, joint);
            println!(
                "{name} {joint}: fitted omega {:.2} zeta {:.3} delay {:.3} s overshoot {:.0} %, \
                 model omega {omega:.2} zeta {zeta:.3}",
                got.omega,
                got.zeta,
                got.delay_s,
                got.overshoot * 100.0
            );
            assert!(
                (got.omega - omega).abs() <= 0.1 * omega,
                "{name} {joint}: {got:?}"
            );
            assert!(
                (got.zeta - zeta).abs() <= 0.2 * zeta + 0.01,
                "{name} {joint}: {got:?}"
            );
            if damped {
                assert!(got.zeta >= 0.45, "{name} {joint}: {got:?}");
            } else {
                assert!(got.zeta <= 0.15, "{name} {joint}: {got:?}");
            }
        }
    }
}
