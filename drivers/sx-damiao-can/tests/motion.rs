//! The coast bound each Damiao axis opens with (`DamiaoAxis::coast`): Standard is the driver as
//! it was before the bound became a choice, frame for frame; Brisk doubles it, so the planned
//! velocity a frame carries and every speed ceiling scale with it, and nothing else moves.

use serde_json::Value;
use sx_damiao_can::chain::{BRISK_COAST, DamiaoAxis, DamiaoChain, MAX_COAST};
use sx_damiao_can::sim::Sim;
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_drivers::{ActuatorChain, Feedforward, FeedforwardLimit, InstanceDrive};

fn drive() -> InstanceDrive {
    let drives: Value = serde_json::from_str(include_str!("fixtures/drives.json")).expect("drives");
    let nominal = &drives["sentient-static-upper-body"]["nominal"];
    InstanceDrive::parse(
        &nominal.to_string(),
        nominal["instance"].as_str().expect("an instance"),
    )
    .expect("a drive")
}

fn axes(coast: Option<f64>) -> Vec<DamiaoAxis> {
    let mut axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    if let Some(coast) = coast {
        for axis in &mut axes {
            axis.coast = coast;
        }
    }
    axes
}

/// FNV-1a over every frame sent (id, then data) and every ceiling's bits: one number that moves
/// when any byte or ceiling does.
fn digest(frames: &[(u32, [u8; 8])], ceilings: &[(usize, [u64; 3])]) -> u64 {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut eat = |bytes: &[u8]| {
        for byte in bytes {
            hash ^= u64::from(*byte);
            hash = hash.wrapping_mul(0x0100_0000_01b3);
        }
    };
    for (id, data) in frames {
        eat(&id.to_le_bytes());
        eat(data);
    }
    for (index, bits) in ceilings {
        eat(&index.to_le_bytes());
        for bit in bits {
            eat(&bit.to_le_bytes());
        }
    }
    hash
}

/// A fixed run: 120 commands moving every joint by a few milliradians about where it rests, with
/// planned velocities from 0 to 4 rad/s either way (past every axis's planned-velocity cap) and
/// torques up to 6 N m, all under generous feed-forward bounds.
/// Every frame sent (id, data), and every axis's ceilings as bits (speed, acceleration, rise).
type Run = (Vec<(u32, [u8; 8])>, Vec<(usize, [u64; 3])>);

fn run(axes: Vec<DamiaoAxis>) -> Run {
    let bus = Sim::for_axes(&axes);
    let mut chain = DamiaoChain::open(bus.clone(), axes).expect("opens");
    let rest = chain.last_observed().expect("rest").to_vec();
    let width = rest.len();
    let lanes: Vec<f64> = (0..u32::try_from(width).expect("a few axes"))
        .map(f64::from)
        .collect();
    chain
        .set_feedforward_limits(&vec![
            FeedforwardLimit {
                velocity: Some(10.0),
                torque: Some(30.0),
            };
            width
        ])
        .expect("bounds");
    let ceilings: Vec<(usize, [u64; 3])> = chain
        .tracking_ceilings()
        .into_iter()
        .map(|(index, c)| {
            (
                index,
                [
                    c.speed.to_bits(),
                    c.acceleration.unwrap_or(f64::NAN).to_bits(),
                    c.rise_s.unwrap_or(f64::NAN).to_bits(),
                ],
            )
        })
        .collect();
    let start = bus.state().frames.len();
    for step in 0..120_u32 {
        let phase = f64::from(step) * 0.13;
        let targets: Vec<f64> = rest
            .iter()
            .enumerate()
            .map(|(i, q)| q + 0.004 * (phase + lanes[i]).sin())
            .collect();
        let feedforward: Vec<Feedforward> = lanes
            .iter()
            .map(|lane| Feedforward {
                velocity: 4.0 * (phase * 0.7 + lane).sin(),
                torque: 6.0 * (phase * 0.3 + lane).cos(),
                gravity: 2.0 * (phase * 0.3 + lane).cos(),
            })
            .collect();
        chain.command_with(&targets, &feedforward).expect("command");
    }
    let frames = bus.state().frames[start..]
        .iter()
        .map(|frame| (frame.id, frame.data))
        .collect();
    (frames, ceilings)
}

/// Recorded from sx-damiao-can before `DamiaoAxis::coast` existed (`MAX_COAST` a constant): what
/// every static upper body axis was sent, and its ceilings, over [`run`].
const BEFORE: u64 = 0xc55f_6a58_0d17_dbaf;

#[test]
fn standard_is_the_driver_as_it_was_frame_for_frame() {
    let (frames, ceilings) = run(axes(None));
    assert!(frames.len() > 120 * 15, "{} frames", frames.len());
    assert_eq!(digest(&frames, &ceilings), BEFORE);
    let (explicit, explicit_ceilings) = run(axes(Some(MAX_COAST)));
    assert_eq!((explicit, explicit_ceilings), (frames, ceilings));
}

#[test]
fn brisk_doubles_the_planned_velocity_cap_and_lifts_only_the_speed_ceilings() {
    let standard = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let open = |coast: f64| {
        let axes = axes(Some(coast));
        DamiaoChain::open(Sim::for_axes(&axes), axes).expect("opens")
    };
    let (slow, brisk) = (open(MAX_COAST), open(BRISK_COAST));
    for ((index, a), (other, b)) in slow
        .tracking_ceilings()
        .into_iter()
        .zip(brisk.tracking_ceilings())
    {
        assert_eq!(index, other);
        let motor = &standard[index];
        let name = &motor.axis.joint;
        // Acceleration and rise time are the gains' alone.
        assert_eq!(a.acceleration, b.acceleration, "{name}");
        assert_eq!(a.rise_s, b.rise_s, "{name}");
        // Every speed term grows with the coast bound, at most in proportion.
        assert!(b.speed > a.speed, "{name}: {} -> {}", a.speed, b.speed);
        assert!(
            b.speed <= a.speed * BRISK_COAST / MAX_COAST + 1e-9,
            "{name}"
        );
        // The lag term is (coast + MAX_LAG) kp / kd.
        let lag = (BRISK_COAST + sx_damiao_can::chain::MAX_LAG) * motor.kp / motor.kd;
        assert!(b.speed <= lag + 1e-9, "{name}");
    }
}

#[test]
fn a_coast_bound_outside_its_range_is_refused_at_open() {
    for coast in [0.0, -0.1, BRISK_COAST * 1.01, f64::NAN] {
        let axes = axes(Some(coast));
        assert!(
            DamiaoChain::open(Sim::for_axes(&axes), axes).is_err(),
            "coast {coast} opened"
        );
    }
}
