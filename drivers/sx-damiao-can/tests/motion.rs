//! The coast bound each Damiao axis opens with (`DamiaoAxis::coast`): Standard is the driver as
//! it was before the bound became a choice, frame for frame; Brisk doubles it, so a frame carries
//! up to twice the planned velocity, and every ceiling stays Standard's.

use serde_json::Value;
use sx_damiao_can::chain::{BRISK_COAST, DamiaoAxis, DamiaoChain, MAX_COAST, MAX_LAG};
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
fn brisk_carries_twice_the_planned_velocity_and_keeps_every_ceiling() {
    let open = |coast: f64| {
        let axes = axes(Some(coast));
        DamiaoChain::open(Sim::for_axes(&axes), axes).expect("opens")
    };
    let (mut slow, mut brisk) = (open(MAX_COAST), open(BRISK_COAST));
    assert_eq!(slow.tracking_ceilings(), brisk.tracking_ceilings());
    let standard = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let width = standard.len();
    let bound = vec![
        FeedforwardLimit {
            velocity: Some(10.0),
            torque: None,
        };
        width
    ];
    // Every axis asked for 1.2 rad/s: Standard clips the DM4340 joints at 0.1 kp/kd (0.48-0.66
    // rad/s); Brisk at 0.2 kp/kd (0.96-1.31).
    let wanted = vec![
        Feedforward {
            velocity: 1.2,
            torque: 0.0,
            gravity: 0.0,
        };
        width
    ];
    // Mid-range targets, so the range leaves the planned velocity whole.
    let targets: Vec<f64> = slow
        .commandable()
        .iter()
        .map(|range| f64::midpoint(range.lower, range.upper))
        .collect();
    // The joints are first brought to the targets at rest: Brisk's bound is for a joint
    // tracking its target, and gives way to one far behind it.
    let still = vec![Feedforward::default(); width];
    for chain in [&mut slow, &mut brisk] {
        chain.set_feedforward_limits(&bound).expect("bounds");
        chain
            .command_with(&targets, &still)
            .expect("to the targets");
        chain.command_with(&targets, &wanted).expect("command");
    }
    let (a, b) = (slow.applied_feedforward(), brisk.applied_feedforward());
    for (index, motor) in standard.iter().enumerate() {
        let name = &motor.axis.joint;
        let clip = |coast: f64| (coast * motor.kp / motor.kd).min(1.2);
        assert!(a[index].velocity.abs() <= clip(MAX_COAST) + 1e-6, "{name}");
        assert!(
            b[index].velocity.abs() <= clip(BRISK_COAST) + 1e-6,
            "{name}"
        );
        assert!(
            b[index].velocity.abs() >= a[index].velocity.abs() - 1e-9,
            "{name}"
        );
    }
    let elbow = standard
        .iter()
        .position(|m| m.axis.joint == "left_elbow_pitch_joint")
        .expect("an elbow");
    assert!(
        b[elbow].velocity.abs() > 1.5 * a[elbow].velocity.abs(),
        "the elbow: {} -> {}",
        a[elbow].velocity,
        b[elbow].velocity
    );
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

/// A joint blocked by what it met (a person, a table) pushes `kp e + kd v_des` into it, `e` its
/// error toward its target: at joint scale, `e + (kd / kp) v_des` times `kp`. Brisk's doubled
/// planned velocity gives way as `e` grows, so the push never exceeds the most Standard pushes
/// before the governor's at-rest tracking bound (`2 MAX_LAG`) trips it, and from that bound on
/// it is Standard's, error for error.
#[test]
fn a_blocked_joint_pushes_no_harder_at_brisk_than_at_standard() {
    let standard = axes(None);
    let width = standard.len();
    let push = |coast: f64, index: usize, error: f64| {
        let axes = axes(Some(coast));
        let bus = Sim::for_axes(&axes);
        let mut chain = DamiaoChain::open(bus.clone(), axes.clone()).expect("opens");
        chain
            .set_feedforward_limits(&vec![
                FeedforwardLimit {
                    velocity: Some(10.0),
                    torque: None,
                };
                width
            ])
            .expect("bounds");
        let middle: Vec<f64> = chain
            .commandable()
            .iter()
            .map(|range| f64::midpoint(range.lower, range.upper))
            .collect();
        chain
            .command_with(&middle, &vec![Feedforward::default(); width])
            .expect("to the middle");
        // Blocked where it stands; the target and the planned velocity run on past it.
        let motor = &axes[index];
        let blocked = motor.axis.actuator_from_joint(middle[index]);
        bus.state()
            .motors
            .get_mut(&motor.axis.binding.bus_id)
            .expect("the motor")
            .q = blocked;
        let _ = chain.read_positions();
        let mut targets = middle.clone();
        targets[index] += error;
        let mut wanted = vec![Feedforward::default(); width];
        wanted[index].velocity = 10.0;
        chain.command_with(&targets, &wanted).expect("command");
        let v = chain.applied_feedforward()[index].velocity;
        error + motor.kd / motor.kp * v
    };
    for (index, motor) in standard.iter().enumerate() {
        let name = &motor.axis.joint;
        let most = push(MAX_COAST, index, 2.0 * MAX_LAG);
        for step in 0..=30_u32 {
            let error = f64::from(step) * 0.01;
            let (slow, brisk) = (
                push(MAX_COAST, index, error),
                push(BRISK_COAST, index, error),
            );
            // Within one 12-bit velocity code of the frame (0.2 mrad of push at the upper-arm yaw).
            assert!(
                brisk <= slow.max(most) + 5e-4,
                "{name} at {error}: {brisk} > {slow}, {most}"
            );
            if error >= 2.0 * MAX_LAG {
                assert!(
                    (brisk - slow).abs() < 5e-4,
                    "{name} at {error}: {brisk} != {slow}"
                );
            }
        }
        // Tracking (no error), Brisk carries more of the planned velocity wherever the clip bound.
        assert!(
            push(BRISK_COAST, index, 0.0) >= push(MAX_COAST, index, 0.0) - 1e-9,
            "{name}"
        );
    }
}
