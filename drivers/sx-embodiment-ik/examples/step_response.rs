//! How fast the teleoperation solve closes a hand jump: the static upper body at the bench's
//! per-model speed caps, its left hand asked 10 cm away at once, stepped as a ghost (each step
//! from the last) at 10 ms. Prints the ticks to within 5 mm and 1 mm, and each joint's peak
//! speed against its cap and the time it first reached 90 % of it.
//!
//! ```text
//! cargo run --release -p sx-embodiment-ik --example step_response -- [ramp_s] [dx dy dz]
//! ```

#![allow(clippy::too_many_lines)] // One measurement, start to report.

use std::collections::BTreeMap;

use sx_embodiment_ik::teleop::{Pose, TELEOP_SCHEMA, Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{Body, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF};

const DT: f64 = 0.01;

fn cap(name: &str) -> f64 {
    let small = name.contains("wrist");
    let medium = ["upper_arm_yaw", "elbow_pitch", "forearm_yaw"]
        .iter()
        .any(|joint| name.contains(joint));
    if small {
        8.0
    } else if medium || name.starts_with("so101_") || name.contains("finger") {
        3.0
    } else {
        0.5
    }
}

fn main() {
    let args: Vec<f64> = std::env::args()
        .skip(1)
        .filter_map(|a| a.parse().ok())
        .collect();
    let ramp_s = args.first().copied().unwrap_or(0.25);
    let shift = [
        args.get(1).copied().unwrap_or(0.0),
        args.get(2).copied().unwrap_or(0.0),
        args.get(3).copied().unwrap_or(0.10),
    ];
    let body = Body::static_upper_body();
    let names: Vec<String> = body.model().axes().iter().map(|a| a.name.clone()).collect();
    let speed: BTreeMap<String, f64> = names.iter().map(|n| (n.clone(), cap(n))).collect();
    let config = TeleopConfig {
        schema: TELEOP_SCHEMA.into(),
        limbs: TeleopConfig::static_upper_body_limbs(),
        speed,
        accel: BTreeMap::new(),
        ramp_s,
        max_step_rad: 0.08,
        held: Vec::new(),
        obstacles: Vec::new(),
        posture: std::collections::BTreeMap::new(),
    };
    let mut solve =
        Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config).expect("solve");
    // The bench's working pose: the clearest of a grid of symmetric arm poses, hands in front.
    let index_of = |name: &str| {
        body.model()
            .axes()
            .iter()
            .position(|a| a.name == name)
            .expect(name)
    };
    let mut q: Vec<f64> = solve.ik().rest().to_vec();
    let mut best = (f64::NEG_INFINITY, q.clone());
    for pitch in [-1.2, -0.9, -0.6, -0.3, 0.0] {
        for elbow in [0.4, 0.8, 1.2, 1.6] {
            for roll in [0.0, 0.2, 0.4] {
                let mut c = q.clone();
                c[index_of("left_shoulder_pitch_joint")] = pitch;
                c[index_of("right_shoulder_pitch_joint")] = pitch;
                c[index_of("left_shoulder_roll_joint")] = roll;
                c[index_of("right_shoulder_roll_joint")] = -roll;
                c[index_of("left_elbow_pitch_joint")] = -elbow;
                c[index_of("right_elbow_pitch_joint")] = elbow;
                let clear = solve.ik().clearance(&c).0;
                if clear > best.0 {
                    best = (clear, c);
                }
            }
        }
    }
    q = best.1;
    println!("start clearance {:.3} m", solve.ik().clearance(&q).0);
    let frames = body.model().poses(&q);
    let tcp = |hand: usize| {
        let p =
            sx_embodiment_drivers::kinematics::pose_of(&body.frame(&frames, hand).to_transform());
        Pose {
            translation_m: p.translation_m,
            rotation_xyzw: p.rotation_xyzw,
        }
    };
    let (left, right) = (tcp(0), tcp(1));
    let mut target = left;
    for (axis, by) in target.translation_m.iter_mut().zip(shift) {
        *axis += by;
    }
    let targets = Targets {
        tcp: [Some(target), Some(right)],
        ..Targets::default()
    };
    let width = q.len();
    let measured = vec![None; width];
    let available = vec![true; width];
    let mut peak = vec![0.0_f64; width];
    let mut reached_90: Vec<Option<usize>> = vec![None; width];
    let (mut within_5mm, mut within_1mm) = (None, None);
    solve.reset();
    for tick in 1..=400 {
        let step = solve
            .step(&q, &measured, &available, &targets, DT)
            .expect("a step")
            .clone();
        for (index, v) in step.qdot.iter().enumerate() {
            peak[index] = peak[index].max(v.abs());
            if reached_90[index].is_none() && v.abs() >= 0.9 * cap(&names[index]) {
                reached_90[index] = Some(tick);
            }
        }
        q.copy_from_slice(&step.q);
        let now = body.frame(&body.model().poses(&q), 0).t;
        let error = (0..3)
            .map(|k| (now[k] - target.translation_m[k]).powi(2))
            .sum::<f64>()
            .sqrt();
        if [1, 5, 10, 25, 50, 100, 200, 400].contains(&tick) {
            let limb = &step.limbs[0];
            println!(
                "  tick {tick:3}: error {:6.1} mm, residual {:5.1} mm {:4.1} deg, {:?}, solve {:?}, active {}",
                error * 1000.0,
                limb.residual_m * 1000.0,
                limb.residual_rad.to_degrees(),
                limb.reach,
                step.solve,
                limb.active
                    .iter()
                    .map(|a| a.describe(solve.ik().body()))
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }
        if within_5mm.is_none() && error < 0.005 {
            within_5mm = Some(tick);
        }
        if within_1mm.is_none() && error < 0.001 {
            within_1mm = Some(tick);
            break;
        }
    }
    println!(
        "ramp {ramp_s} s, jump {shift:?} m: within 5 mm after {within_5mm:?} ticks, within 1 mm \
         after {within_1mm:?} ticks (10 ms)"
    );
    for (index, name) in names.iter().enumerate() {
        if name.starts_with("left_") && !name.contains("finger") {
            println!(
                "  {name:28} peak {:5.2} of cap {:4.1} rad/s ({:3.0} %), 90 % of cap at tick {:?}",
                peak[index],
                cap(name),
                100.0 * peak[index] / cap(name),
                reached_90[index]
            );
        }
    }
}
