//! A steady-state tick allocates nothing, counted by a counting global allocator
//! (`allocation-counter`, whose `unsafe` lives outside this `unsafe`-forbidding workspace).

// Test and evaluation code: counts as f64, exact comparisons of held values, formula names.
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::many_single_char_names,
    clippy::float_cmp,
    clippy::doc_markdown
)]

use sx_embodiment_ik::{Body, Caps, Ik, Settings, Target};

/// Default settings with a deadline far past any tick: the tests are about the answer, not
/// the clock, and run on a loaded machine.
fn settings() -> Settings {
    Settings {
        deadline: std::time::Duration::from_secs(1),
        ..Settings::default()
    }
}

/// Every axis available.
const ALL: [bool; 64] = [true; 64];

#[test]
fn a_steady_state_tick_allocates_nothing() {
    let body = Body::static_upper_body();
    let caps = Caps::declared(&body, 0.25).expect("caps").at_most(0.5);
    let mut ik = Ik::new(body, caps, settings(), Vec::new()).expect("ik");
    let mut q = ik.rest().to_vec();
    let mut next = q.clone();
    let mut worst = 0;
    for tick in 0..400 {
        let t = f64::from(tick) * 0.01;
        let targets = [
            Some(Target::Pose {
                translation_m: [0.35 + 0.1 * t.sin(), 0.22, 0.12 + 0.05 * t.cos()],
                rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
            }),
            Some(Target::Pose {
                translation_m: [0.4, -0.15 - 0.1 * t.sin(), 0.1],
                rotation_xyzw: [0.0, 0.2, 0.0, 1.0],
            }),
            Some(Target::LookAt {
                point_m: [0.6, 0.2 * t.sin(), 0.05],
            }),
        ];
        let counted = allocation_counter::measure(|| {
            let step = ik.step(&q, &ALL[..q.len()], &targets, 0.01).expect("step");
            next.copy_from_slice(&step.q);
        });
        q.copy_from_slice(&next);
        if tick > 10 {
            worst = worst.max(counted.count_total);
        }
    }
    assert_eq!(worst, 0, "a tick allocated {worst} times");
}

/// The clearance a control thread checks each commanded configuration with allocates nothing,
/// and agrees with the allocating one.
#[test]
fn a_clearance_check_allocates_nothing_and_agrees() {
    let body = Body::static_upper_body();
    let caps = Caps::declared(&body, 0.25).expect("caps");
    let scene = vec![sx_embodiment_ik::Obstacle {
        min: [0.3, -0.5, -0.3],
        max: [1.0, 0.5, -0.1],
    }];
    let ik = Ik::new(body, caps, settings(), scene).expect("ik");
    let q = ik.rest().to_vec();
    let mut frames = ik.body().model().poses(&q);
    let mut answer = None;
    let counted = allocation_counter::measure(|| {
        answer = Some(ik.clearance_into(&q, &mut frames));
    });
    assert_eq!(counted.count_total, 0);
    let answer = answer.expect("answer");
    let (least, _, _) = ik.clearance(&q);
    assert!(
        (answer.body_m.min(answer.scene_m) - least).abs() < 1e-12,
        "{answer:?} {least}"
    );
}
