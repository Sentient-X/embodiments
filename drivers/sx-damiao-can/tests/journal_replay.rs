//! A station's recorded teleop commands replayed through the governed static upper body on
//! the simulated MIT bus (`Sim::with_mit_dynamics`: each motor's law on its loaded joint, with
//! its model's Coulomb and viscous friction), so what the motors do with each command (its
//! planned velocity, its feed-forward torque, how many frames a tick) is measured on the very
//! commands the real arm was given. It writes `control_journal_lag.py export`'s format, for
//! `sx-embodiment-ik`'s `teleop_shake journal` to read the tool's tracking, wobble and lag
//! beside the real session's.
//!
//! ```text
//! JOURNAL=/tmp/shake/json/22ebee05_0193e6d5.json OUT=/tmp/sim.json FF=planned \
//!   cargo test --release -p sx-damiao-can --test journal_replay replay -- --ignored --nocapture
//! cargo run --release -p sx-embodiment-ik --example teleop_shake -- journal /tmp/sim.json
//! ```
//!
//! - `FF`: `planned` (what the station sends today: the planned velocity, no friction),
//!   `friction` (each simulated joint's own friction, `FRICTION_SHARE` of it, fed forward along
//!   the planned velocity), `inertia` (its inertia times the planned acceleration, low-passed by
//!   `ACCEL_FILTER` as the station's follower does), `all` (both).
//! - `GAINS=damped`: `STATIC_UPPER_BODY_DAMPED` in place of the commissioned gains.
//! - `SIM_FRICTION`: the simulated joints' friction, times (the feed-forward keeps the model's).
//! - `SUB`: frames a recorded command, each at its position carried on along its planned
//!   velocity, up to the next command's time.
//!
//! Measured 2026-11-06 on the Spark's four moving sessions (10-28 946454b7, 10-30 22ebee05,
//! ea603be8, 52209133), tool point, mm (wobble: measured motion above 1 Hz; track p50 / p95):
//!
//! | | wobble | track p50 | track p95 |
//! | --- | --- | --- | --- |
//! | real arm | 1.6-2.5 | 10.7-20.7 | 18.0-41.8 |
//! | sim, as sent today | 1.1-2.3 | 9.4-21.6 | 18.1-38.2 |
//! | sim, no friction at all | 1.0-4.5 | 0.2-1.8 | 2.6-13.5 |
//! | 2 or 4 frames per tick | unchanged (<= 0.1) | unchanged | unchanged |
//! | inertia feed-forward | 0.7-1.9 | unchanged | unchanged |
//! | friction feed-forward | 1.0-2.5 | 4.1-13.5 | 9.6-22.1 |
//! | damped gains | 0.5-1.7 | unchanged | unchanged |
//! | damped gains + friction + inertia | 0.5-1.7 | 3.0-18.1 | 9.7-21.3 |
//! | friction fed forward at twice the joint's | 1.1-3.4 (worse) | 4.5-9.3 | 7.1-15.4 |
//!
//! So on these motors the planned velocity already rides every frame (the governor sends its
//! step's speed), the staircase of one frame a tick costs nothing measurable, unmodelled
//! friction is the tracking error, and the shoulders' low damping is the wobble.
// A measurement harness over a recorded journal: times and counts arrive as JSON floats.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::too_many_lines
)]

use std::cell::Cell;
use std::rc::Rc;
use std::time::{Duration, Instant};

use serde_json::{Value, json};
use sx_damiao_can::chain::DamiaoChain;
use sx_damiao_can::sim::Sim;
use sx_damiao_can::static_upper_body::{STATIC_UPPER_BODY, STATIC_UPPER_BODY_DAMPED};
use sx_embodiment_drivers::{
    ActuatorChain, AxisLimits, Clock, Feedforward, FeedforwardLimit, Governor, InstanceDrive,
};

fn drive() -> InstanceDrive {
    let drives: Value = serde_json::from_str(include_str!("fixtures/drives.json")).expect("drives");
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

fn env_f64(name: &str, default: f64) -> f64 {
    std::env::var(name)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn floats(value: &Value) -> Vec<f64> {
    value
        .as_array()
        .expect("an array")
        .iter()
        .map(|v| v.as_f64().unwrap_or(f64::NAN))
        .collect()
}

/// The stiction velocity the friction feed-forward's `tanh` turns on over, rad/s at the joint.
const STICTION_VELOCITY: f64 = 0.02;

#[test]
#[ignore = "a replay of a recorded journal: JOURNAL=<export json> OUT=<json>"]
fn replay() {
    let path = std::env::var("JOURNAL").expect("JOURNAL=<control_journal_lag.py export json>");
    let out = std::env::var("OUT").expect("OUT=<json>");
    let mode = std::env::var("FF").unwrap_or_else(|_| "planned".into());
    let share = env_f64("FRICTION_SHARE", 1.0);
    let sim_friction = env_f64("SIM_FRICTION", 1.0);
    let sub = env_f64("SUB", 1.0).max(1.0) as usize;
    let doc: Value =
        serde_json::from_str(&std::fs::read_to_string(&path).expect("journal")).expect("json");
    let names: Vec<String> = doc["names"]
        .as_array()
        .expect("names")
        .iter()
        .map(|n| n.as_str().expect("a name").to_owned())
        .collect();
    let commands: Vec<(f64, Vec<f64>, Vec<f64>)> = doc["commands"]
        .as_array()
        .expect("commands")
        .iter()
        .map(|row| {
            (
                row[0].as_f64().expect("t"),
                floats(&row[1]),
                floats(&row[2]),
            )
        })
        .collect();
    let readings = doc["readings"].as_array().expect("readings");

    let body = if std::env::var("GAINS").as_deref() == Ok("damped") {
        STATIC_UPPER_BODY_DAMPED
    } else {
        STATIC_UPPER_BODY
    };
    let axes = body.axes(&drive()).expect("axes");
    // Each chain axis' column in the journal.
    let column: Vec<usize> = axes
        .iter()
        .map(|axis| {
            names
                .iter()
                .position(|n| *n == axis.axis.joint)
                .expect("a journal axis")
        })
        .collect();
    let bus = Sim::for_axes(&axes).with_mit_dynamics().with_manual_time();
    // Start where the arm first read, at rest.
    let first = readings
        .iter()
        .find(|r| r[0].as_f64().unwrap_or(0.0) >= commands[0].0 - 1e8)
        .unwrap_or(&readings[0]);
    let start = floats(&first[1]);
    {
        let mut state = bus.state();
        for (axis, col) in axes.iter().zip(&column) {
            let motor = state
                .motors
                .get_mut(&axis.axis.binding.bus_id)
                .expect("motor");
            motor.q = axis.axis.actuator_from_joint(start[*col]);
            if let Some(dynamics) = motor.dynamics.as_mut() {
                dynamics.coulomb *= sim_friction;
                dynamics.damping *= sim_friction;
            }
        }
    }
    // The friction the feed-forward models, at the joint: each motor's simulated joint's.
    let friction: Vec<(f64, f64, f64)> = {
        let state = bus.state();
        axes.iter()
            .map(|axis| {
                let gain = axis.axis.actuator_from_joint(axis.axis.zero + 1.0).abs();
                let d = state.motors[&axis.axis.binding.bus_id]
                    .dynamics
                    .expect("dynamics");
                // Motor torque to joint torque: times the gain; viscous at the joint: gain^2.
                (
                    d.coulomb / sim_friction * gain,
                    d.damping / sim_friction * gain * gain,
                    d.inertia * gain * gain,
                )
            })
            .collect()
    };
    let chain = DamiaoChain::open(bus.clone(), axes.clone()).expect("opens");
    let limits = AxisLimits::rated_all(chain.axes(), 1.0).expect("rated");
    let clock = Manual(Rc::new(Cell::new(Instant::now())));
    let mut governor = Governor::with_clock(chain, limits.clone(), clock.clone()).expect("gov");
    governor
        .set_feedforward_limits(
            &limits
                .iter()
                .map(|l| FeedforwardLimit {
                    velocity: Some(l.max_speed),
                    torque: Some(10.0),
                })
                .collect::<Vec<_>>(),
        )
        .expect("limits");

    let width = axes.len();
    let mut out_readings = Vec::new();
    let mut ff = vec![Feedforward::default(); width];
    let mut target = vec![0.0; width];
    let t0 = commands[0].0;
    // The planned acceleration, low-passed as the station's follower does (`ACCELERATION_FILTER`).
    let filter = env_f64("ACCEL_FILTER", 0.2);
    let mut accel = vec![0.0; width];
    let mut previous: Option<(f64, Vec<f64>)> = None;
    let mut now_ns = t0;
    for (k, (t, q, v)) in commands.iter().enumerate() {
        let next = commands.get(k + 1).map_or(*t + 1e7, |c| c.0);
        // A gap (no command for longer than 60 ms) is a hold: the motors keep the last frame.
        let span = (next - t).min(6e7);
        if let Some((tp, vp)) = &previous {
            let dt = (t - tp) * 1e-9;
            for (i, col) in column.iter().enumerate() {
                let raw = if dt > 0.0 && dt < 0.06 {
                    (v[*col] - vp[*col]) / dt
                } else {
                    0.0
                };
                accel[i] += filter * (raw - accel[i]);
            }
        }
        previous = Some((*t, v.clone()));
        for s in 0..sub {
            let at = t + span * s as f64 / sub as f64;
            let advance = (at - now_ns).max(0.0);
            bus.advance_time(Duration::from_nanos(advance as u64));
            clock
                .0
                .set(clock.0.get() + Duration::from_nanos(advance as u64));
            now_ns = at;
            let lead = (at - t) * 1e-9;
            for (i, col) in column.iter().enumerate() {
                let planned = v.get(*col).copied().unwrap_or(0.0);
                let velocity = if mode == "zero" { 0.0 } else { planned };
                target[i] = q[*col] + velocity * lead;
                let (coulomb, viscous, inertia) = friction[i];
                let rubbing =
                    share * (coulomb * (planned / STICTION_VELOCITY).tanh() + viscous * planned);
                let inertial = env_f64("INERTIA_SHARE", 1.0) * inertia * accel[i];
                let torque = match mode.as_str() {
                    "friction" => rubbing,
                    "inertia" => inertial,
                    "all" => rubbing + inertial,
                    _ => 0.0,
                };
                ff[i] = Feedforward {
                    velocity,
                    torque,
                    gravity: 0.0,
                };
            }
            let receipt = governor.command_with(&target, &ff);
            if s == 0 {
                // The reading the real chain journals: each motor's answer to this frame.
                let mut row = start.clone();
                let state = bus.state();
                for (axis, col) in axes.iter().zip(&column) {
                    let m = &state.motors[&axis.axis.binding.bus_id];
                    row[*col] = axis.axis.joint_from_actuator(m.q);
                }
                drop(state);
                let fresh = vec![true; names.len()];
                out_readings.push(json!([*t, row, fresh]));
            }
            if let Err(error) = receipt {
                println!("stopped at command {k} ({:.2} s): {error}", (t - t0) * 1e-9);
                break;
            }
        }
    }
    let doc = json!({
        "names": names,
        "commands": doc["commands"],
        "readings": out_readings,
    });
    std::fs::write(&out, doc.to_string()).expect("write");
    println!(
        "{} commands replayed ({mode}, sub {sub}, friction share {share}, sim friction {sim_friction}) -> {out}",
        commands.len()
    );
}
