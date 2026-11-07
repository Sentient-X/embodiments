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

use sx_embodiment_ik::{Caps, Ik, Settings, Target};
use sx_frames::{Pose, QuaternionLayout};

/// The static upper body's committed render pack: one fixture among the registry's bodies.
fn static_files() -> sx_embodiment_ik::BodyFiles<'static> {
    sx_embodiment_ik::registered("sentient-static-upper-body").expect("its pack is committed")
}

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
    let body = static_files().body().expect("the static upper body solves");
    let caps = Caps::declared(&body, 0.25).expect("caps").at_most(0.5);
    let mut ik = Ik::new(body, caps, settings(), Vec::new()).expect("ik");
    let mut q = ik.rest().to_vec();
    let mut next = q.clone();
    let mut worst = 0;
    for tick in 0..400 {
        let t = f64::from(tick) * 0.01;
        let targets = [
            Some(Target::Pose(Pose {
                translation_m: [0.35 + 0.1 * t.sin(), 0.22, 0.12 + 0.05 * t.cos()],
                rotation: QuaternionLayout::Xyzw.read([0.0, 0.0, 0.0, 1.0]),
            })),
            Some(Target::Pose(Pose {
                translation_m: [0.4, -0.15 - 0.1 * t.sin(), 0.1],
                rotation: QuaternionLayout::Xyzw.read([0.0, 0.2, 0.0, 1.0]),
            })),
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

/// A tick measured behind its command (pairs kept at both poses), one joint stuck and every
/// axis given a window allocates nothing either.
#[test]
fn a_measured_tick_with_windows_allocates_nothing() {
    let body = static_files().body().expect("the static upper body solves");
    let caps = Caps::declared(&body, 0.25).expect("caps").at_most(0.5);
    let mut ik = Ik::new(body, caps, settings(), Vec::new()).expect("ik");
    let mut q = ik.rest().to_vec();
    let mut next = q.clone();
    let mut measured: Vec<Option<f64>> = q.iter().copied().map(Some).collect();
    let mut stuck = vec![false; q.len()];
    stuck[3] = true;
    let mut worst = 0;
    for tick in 0..400 {
        let t = f64::from(tick) * 0.01;
        let targets = [
            Some(Target::Pose(Pose {
                translation_m: [0.35 + 0.1 * t.sin(), 0.22, 0.12 + 0.05 * t.cos()],
                rotation: QuaternionLayout::Xyzw.read([0.0, 0.0, 0.0, 1.0]),
            })),
            Some(Target::Pose(Pose {
                translation_m: [0.4, -0.15 - 0.1 * t.sin(), 0.1],
                rotation: QuaternionLayout::Xyzw.read([0.0, 0.2, 0.0, 1.0]),
            })),
            Some(Target::LookAt {
                point_m: [0.6, 0.2 * t.sin(), 0.05],
            }),
        ];
        let counted = allocation_counter::measure(|| {
            for (axis, at) in measured.iter().enumerate() {
                let at = at.unwrap_or(0.0);
                ik.set_window(axis, at - 0.1, at + 0.1);
            }
            ik.set_measured(&measured, &stuck).expect("measured");
            let step = ik.step(&q, &ALL[..q.len()], &targets, 0.01).expect("step");
            next.copy_from_slice(&step.q);
        });
        // The robot one tick behind.
        for (slot, at) in measured.iter_mut().zip(&q) {
            *slot = Some(*at);
        }
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
    let body = static_files().body().expect("the static upper body solves");
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
