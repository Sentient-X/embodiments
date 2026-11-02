//! The simulated motor's MIT law, one frame at a time: each term of the frame does what the law
//! says, the torque never exceeds the motor's peak, a hard stop holds, and a light joint under
//! firm damping settles (#3004). No governor: the frames are written here, as the firmware
//! receives them.

use std::time::Duration;

use sx_damiao_can::dm_can::{DmMotorType, float_to_uint};
use sx_damiao_can::sim::Sim;
use sx_embodiment_drivers::can::{CanFrame, CanPort};

const ID: u16 = 1;
const MODEL: DmMotorType = DmMotorType::DM4340;
const STEP: Duration = Duration::from_millis(1);

/// One DM4340 on a bus whose motor runs its MIT law, enabled, on manual time, its joint of
/// `inertia` with no friction or damping, and no hard stops unless given.
fn motor(inertia: f64, stops: Option<[f64; 2]>) -> Sim {
    let mut sim = Sim::of(&[(ID, ID + 0x10, MODEL)])
        .with_mit_dynamics()
        .with_manual_time();
    {
        let mut state = sim.state();
        let motor = state.motors.get_mut(&ID).expect("motor");
        motor.stops = stops;
        let joint = motor.dynamics.as_mut().expect("dynamics");
        joint.inertia = inertia;
        joint.coulomb = 0.0;
        joint.damping = 0.0;
    }
    sim.send(&CanFrame {
        id: u32::from(ID),
        data: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
    })
    .expect("enable");
    sim
}

/// Send one MIT frame, packed as `DM_CAN.py`'s `controlMIT` packs it.
fn mit(sim: &mut Sim, p: f64, v: f64, kp: f64, kd: f64, tau: f64) {
    let [q_max, v_max, t_max] = MODEL.limit_param();
    let q = float_to_uint(p, -q_max, q_max, 16).expect("p");
    let v = float_to_uint(v, -v_max, v_max, 12).expect("v");
    let kp = float_to_uint(kp, 0.0, 500.0, 12).expect("kp");
    let kd = float_to_uint(kd, 0.0, 5.0, 12).expect("kd");
    let tau = float_to_uint(tau, -t_max, t_max, 12).expect("tau");
    let byte = |x: u16| x.to_le_bytes()[0];
    sim.send(&CanFrame {
        id: u32::from(ID),
        data: [
            byte(q >> 8),
            byte(q),
            byte(v >> 4),
            byte(((v & 0xf) << 4) | (kp >> 8)),
            byte(kp),
            byte(kd >> 4),
            byte(((kd & 0xf) << 4) | (tau >> 8)),
            byte(tau),
        ],
    })
    .expect("mit");
    while sim.receive().expect("receive").is_some() {}
}

/// Position and velocity after each of `ms` milliseconds.
fn run(sim: &Sim, ms: usize) -> Vec<(f64, f64)> {
    (0..ms)
        .map(|_| {
            sim.advance_time(STEP);
            let state = sim.state();
            let motor = &state.motors[&ID];
            (motor.q, motor.dynamics.as_ref().expect("dynamics").velocity)
        })
        .collect()
}

fn peak() -> f64 {
    let sim = Sim::of(&[(ID, ID + 0x10, MODEL)]).with_mit_dynamics();
    let state = sim.state();
    state.motors[&ID].dynamics.as_ref().expect("dynamics").peak
}

/// A far target under the stiffest gains: the motor gives its peak and no more, so the joint
/// gains exactly the speed the peak gives it, however hard the law asks.
#[test]
fn a_far_target_is_reached_at_peak_torque_and_no_faster() {
    let inertia = 0.01;
    let mut sim = motor(inertia, None);
    mit(&mut sim, 3.0, 0.0, 500.0, 0.5, 0.0);
    let peak = peak();
    for (ms, (q, v)) in run(&sim, 20).into_iter().enumerate() {
        #[allow(clippy::cast_precision_loss)] // a millisecond count
        let t = (ms + 1) as f64 * STEP.as_secs_f64();
        // Saturated throughout (500 N m/rad over 3 rad dwarfs kd v): exactly peak / J.
        assert!(
            (v - peak / inertia * t).abs() <= 1e-9 * (1.0 + v),
            "{v} rad/s at {t} s, not {}",
            peak / inertia * t
        );
        assert!(
            q <= 0.5 * peak / inertia * (t + STEP.as_secs_f64()).powi(2),
            "{q} at {t} s"
        );
        assert!(v > 0.0 && q > 0.0, "it moves toward the target");
    }
}

/// A target past a hard stop: the joint reaches the stop and stays at it, never past.
#[test]
fn a_hard_stop_holds_the_joint() {
    let mut sim = motor(0.01, Some([-0.2, 0.2]));
    mit(&mut sim, 3.0, 0.0, 500.0, 2.0, 0.0);
    let path = run(&sim, 500);
    assert!(path.iter().all(|(q, _)| *q <= 0.2), "past the stop");
    let (q, v) = *path.last().expect("a step");
    assert!((q - 0.2).abs() < 1e-12 && v == 0.0, "at {q}, {v} rad/s");
}

/// `kd` with a planned velocity and no `kp`: the joint runs at the planned velocity.
#[test]
fn the_planned_velocity_is_what_kd_tracks() {
    let mut sim = motor(0.01, None);
    mit(&mut sim, 0.0, 1.0, 0.0, 2.0, 0.0);
    let (_, v) = *run(&sim, 200).last().expect("a step");
    assert!((v - 1.0).abs() < 0.01, "{v} rad/s, not 1");
}

/// Feed-forward torque alone accelerates the joint at `tau / J`.
#[test]
fn feed_forward_torque_accelerates_the_joint() {
    let inertia = 0.1;
    let mut sim = motor(inertia, None);
    mit(&mut sim, 0.0, 0.0, 0.0, 0.0, 1.0);
    let (_, v) = *run(&sim, 100).last().expect("a step");
    assert!((v - 1.0 / inertia * 0.1).abs() < 0.02, "{v} rad/s, not 1");
}

/// #3004 at the frame: the humanoid wrist's gains (kp 54.77, kd 5) on its links' 1.05e-4
/// kg m^2 with no rotor, a 0.004 rad step. Explicit Euler took it to 0.83 rad within 20 ms;
/// the overdamped joint creeps to the target and never past it.
#[test]
fn a_light_joint_under_firm_damping_settles_without_overshoot() {
    let mut sim = motor(1.05e-4, None);
    mit(&mut sim, 0.004, 0.0, 54.772, 5.0, 0.0);
    let path = run(&sim, 1000);
    // The 16-bit position field is 3.8e-4 rad coarse, and the 12-bit velocity field has no
    // exact zero: kd times its nearest value, against kp, holds the joint up to 2.2e-4 off.
    let target = path.last().expect("a step").0;
    assert!((target - 0.004).abs() < 1e-3, "settled at {target}");
    for pair in path.windows(2) {
        assert!(pair[1].0 >= pair[0].0 - 1e-12, "it turned back: {pair:?}");
        assert!(pair[1].0 <= target + 1e-9, "past the target: {pair:?}");
    }
}
