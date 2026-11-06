//! Bounded jerk: the one rule for how a command starts moving and how it comes to rest.
//!
//! A command whose acceleration jumps (from rest to the cap on its first tick, from the cap to
//! zero as it arrives, from a sweep to standing still when the operator lets go) jerks the
//! arm: a lightly damped follower (the static upper body's shoulders ring at zeta 0.1) takes
//! the whole step at once. So every start ramps its acceleration up (an S-curve), every move
//! eases its acceleration back to zero as it reaches its speed cap and as it lands, and every
//! stop brings speed and acceleration to zero together. Nothing is slowed past the ramp: once
//! the acceleration is up the move runs at its full speed and acceleration caps.
//!
//! The bound is each joint's acceleration cap over its **rise time**
//! ([`crate::governor::AxisLimits::acceleration_rise_s`]): how fast its follower takes a change
//! of acceleration, `sqrt(J / kp)` from its gains where the chain knows them
//! ([`crate::chain::TrackingCeiling::rise_s`]: a Damiao joint, 12-200 ms on the static upper
//! body). A ramp shorter than that is a step to the follower.
//!
//! Allocation-free, per axis, one tick `dt` ahead from a commanded velocity `v` and
//! acceleration `a`:
//!
//! - [`acceleration_window`]: the accelerations the next tick may take within the jerk bound and
//!   the acceleration cap, from which the velocity can still settle into an interval by easing
//!   the acceleration back to zero at the jerk bound. A stop is its interval `[0, 0]`: the
//!   fastest stop the bound allows, never delayed further (the solve stops every joint it holds
//!   this way, an operator letting go included).
//! - [`settling_acceleration`]: the acceleration that leaves the velocity exactly at a target
//!   once eased back to zero: the solve's per-joint box eases each joint into its speed cap by
//!   it.
//! - [`velocity_toward`]: the next tick's velocity for a motion that should come to rest on a
//!   target: the fastest in the window from which the jerk-bounded stop
//!   ([`stopping_distance`]) still ends on it. The solve's approach moves each limb's reference
//!   by it (`sx_embodiment_ik::solve`), and [`toward`] one joint (a program's joint rows,
//!   `fleet/station` `program/chunk.rs`).

/// The accelerations, `[low, high]` (unit/s²), the next tick of `dt` seconds may command,
/// from velocity `v` and acceleration `a`: within `jerk * dt` of `a` and `max_acceleration` of
/// zero, and such that easing the acceleration back to zero at `jerk` from there leaves the
/// velocity inside `settle`. Where the two disagree (a stop asked of a joint at speed) the jerk
/// bound holds and the window is the edge of it nearest `settle`: the stop takes as long as the
/// jerk bound needs and no longer. A non-positive or non-finite `jerk` bounds nothing but the
/// acceleration cap.
#[must_use]
pub fn acceleration_window(
    v: f64,
    a: f64,
    settle: (f64, f64),
    max_acceleration: f64,
    jerk: f64,
    dt: f64,
) -> (f64, f64) {
    let bounded = jerk.is_finite() && jerk > 0.0;
    let reach = if bounded { jerk * dt } else { f64::INFINITY };
    let low = (a - reach).clamp(-max_acceleration, max_acceleration);
    let high = (a + reach).clamp(-max_acceleration, max_acceleration);
    let into = |target: f64| settling_acceleration(v, target, jerk, dt);
    let (settle_low, settle_high) = (into(settle.0), into(settle.1));
    if settle_high < low {
        (low, low)
    } else if settle_low > high {
        (high, high)
    } else {
        (low.max(settle_low), high.min(settle_high))
    }
}

/// The acceleration that, commanded this tick and eased back to zero in steps of `jerk dt`
/// over the next ticks, leaves the velocity exactly at `target`. In units of `jerk dt`, an
/// acceleration `u` (with `m = floor(u)`) adds `u` this tick and `m u - m (m + 1) / 2` while it
/// eases: piecewise linear and increasing, so it inverts exactly. With no jerk bound: in one
/// tick.
#[must_use]
pub fn settling_acceleration(v: f64, target: f64, jerk: f64, dt: f64) -> f64 {
    let gap = target - v;
    if gap.is_infinite() {
        return gap;
    }
    if !bounded(jerk) {
        return gap / dt;
    }
    let unit = jerk * dt;
    let g = gap.abs() / (unit * dt);
    let m = ((8.0 * g + 1.0).sqrt() - 1.0)
        .mul_add(0.5, 0.0)
        .floor()
        .max(0.0);
    let u = (g + m * (m + 1.0) / 2.0) / (m + 1.0);
    gap.signum() * u * unit
}

fn bounded(jerk: f64) -> bool {
    jerk.is_finite() && jerk > 0.0
}

/// How far a stop from velocity `v` and acceleration `a` runs, decelerating at most
/// `max_acceleration` with the acceleration changing at most `jerk`: the acceleration taken
/// down (to the cap, or as far as the speed needs), held, and eased back to zero as the speed
/// reaches it. Signed as `v`. A joint already decelerating harder than its speed needs comes
/// to rest where its velocity first reaches zero (and reverses past it, which no bounded jerk
/// avoids). Continuous time.
#[must_use]
pub fn stopping_distance(v: f64, a: f64, max_acceleration: f64, jerk: f64) -> f64 {
    if v < 0.0 {
        return -stopping_distance(-v, -a, max_acceleration, jerk);
    }
    let cap = max_acceleration;
    if !bounded(jerk) {
        return v * v / (2.0 * cap);
    }
    let a = a.clamp(-cap, cap);
    let j = jerk;
    if a < 0.0 && v <= a * a / (2.0 * j) {
        // Easing the deceleration off at once still stops it: where the velocity first is 0.
        let t = (-a - (a * a - 2.0 * j * v).max(0.0).sqrt()) / j;
        return v * t + a * t * t / 2.0 + j * t * t * t / 6.0;
    }
    let peak = (j * v + a * a / 2.0).sqrt().min(cap);
    let t1 = (a + peak) / j;
    let v1 = v + a * t1 - j * t1 * t1 / 2.0;
    let d1 = v * t1 + a * t1 * t1 / 2.0 - j * t1 * t1 * t1 / 6.0;
    let v2 = peak * peak / (2.0 * j);
    let t2 = ((v1 - v2) / peak).max(0.0);
    let d2 = v1 * t2 - peak * t2 * t2 / 2.0;
    let t3 = peak / j;
    let d3 = v2 * t3 - peak * t3 * t3 / 2.0 + j * t3 * t3 * t3 / 6.0;
    d1 + d2 + d3
}

/// The velocity of the next tick of `dt` for a joint (or a tool along its error) `remaining`
/// short of a target it should come to rest on, moving at `v` with acceleration `a`: the
/// fastest whose acceleration lies in [`acceleration_window`] (speed at most `speed`) and from
/// which the stop at `max_acceleration` and `jerk` still ends on the target. Where even the
/// hardest deceleration the jerk bound allows overshoots, that deceleration.
#[must_use]
pub fn velocity_toward(
    remaining: f64,
    v: f64,
    a: f64,
    speed: f64,
    max_acceleration: f64,
    jerk: f64,
    dt: f64,
) -> f64 {
    let s = if remaining > 0.0 {
        1.0
    } else if remaining < 0.0 {
        -1.0
    } else if v != 0.0 {
        -v.signum()
    } else {
        1.0
    };
    let (room, v, a) = (remaining.abs(), s * v, s * a);
    let (low, high) = acceleration_window(v, a, (-speed, speed), max_acceleration, jerk, dt);
    // A tick's own travel, and half a tick more for the stop's discrete start.
    let fits = |x: f64| {
        let v1 = x.mul_add(dt, v);
        stopping_distance(v1, x, max_acceleration, jerk) + 1.5 * v1.max(0.0) * dt <= room
    };
    let x = if fits(high) {
        high
    } else if fits(low) {
        let (mut ok, mut bad) = (low, high);
        for _ in 0..40 {
            let mid = 0.5 * (ok + bad);
            if fits(mid) {
                ok = mid;
            } else {
                bad = mid;
            }
        }
        ok
    } else {
        low
    };
    s * x.mul_add(dt, v)
}

/// One joint's commanded motion: where it was sent, at what velocity and acceleration.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Motion {
    pub position: f64,
    pub velocity: f64,
    pub acceleration: f64,
}

/// One tick of `dt` toward `target`, to rest on it ([`velocity_toward`]), never past `speed`:
/// onto the target exactly when this tick can land there and the next stand still within the
/// bounds (the jerk-bounded approach otherwise closes the last fraction of a millimetre
/// geometrically). Returns the motion sent.
pub fn toward(
    motion: &mut Motion,
    target: f64,
    speed: f64,
    max_acceleration: f64,
    jerk: f64,
    dt: f64,
) -> Motion {
    let error = target - motion.position;
    let mut velocity = velocity_toward(
        error,
        motion.velocity,
        motion.acceleration,
        speed,
        max_acceleration,
        jerk,
        dt,
    );
    let jerk_room = if bounded(jerk) {
        jerk * dt
    } else {
        f64::INFINITY
    };
    let exact = error / dt;
    let landing = (exact - motion.velocity) / dt;
    let standing = -exact / dt;
    let lands = exact.abs() <= speed
        && landing.abs() <= max_acceleration
        && (landing - motion.acceleration).abs() <= jerk_room
        && standing.abs() <= max_acceleration
        && (standing - landing).abs() <= jerk_room;
    if lands {
        velocity = exact;
    }
    *motion = Motion {
        position: if lands {
            target
        } else {
            velocity.mul_add(dt, motion.position)
        },
        velocity,
        acceleration: (velocity - motion.velocity) / dt,
    };
    *motion
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.01;

    /// Steps a joint toward `target` until it rests there; the largest acceleration and jerk
    /// it was commanded, how far past the target it went, and how long it took.
    #[allow(clippy::float_cmp)] // at rest exactly on the target
    fn drive(target: f64, speed: f64, accel: f64, jerk: f64) -> (f64, f64, f64, f64) {
        let mut motion = Motion::default();
        let (mut peak_a, mut peak_j, mut past) = (0.0_f64, 0.0_f64, 0.0_f64);
        for tick in 1..2000 {
            let before = motion;
            toward(&mut motion, target, speed, accel, jerk, DT);
            peak_a = peak_a.max(motion.acceleration.abs());
            peak_j = peak_j.max((motion.acceleration - before.acceleration).abs() / DT);
            past = past.max((motion.position - target) * target.signum());
            if motion.position == target && motion.velocity == 0.0 && motion.acceleration == 0.0 {
                return (peak_a, peak_j, past, f64::from(tick) * DT);
            }
        }
        panic!("never came to rest: {motion:?}");
    }

    #[test]
    #[allow(clippy::float_cmp)] // it lands exactly on the target, at rest
    fn a_move_from_rest_ramps_its_acceleration_and_lands_on_its_target() {
        // The static upper body's shoulder: 1.21 rad/s, 14.7 rad/s², 83 ms rise.
        let (speed, accel) = (1.21, 14.7);
        let jerk = accel / 0.083;
        for target in [0.002, 0.05, 0.3, -1.0, 2.5] {
            let (peak_a, peak_j, past, took) = drive(target, speed, accel, jerk);
            assert!(peak_a <= accel + 1e-9, "{target}: acceleration {peak_a}");
            assert!(
                peak_j <= jerk * (1.0 + 1e-9),
                "{target}: jerk {peak_j} > {jerk}"
            );
            assert!(past <= 1e-12, "{target}: {past} past the target");
            // No permanent slowdown: the acceleration-only move plus about one rise each way.
            let plain = target.abs() / speed + speed / accel;
            eprintln!("{target}: {took:.2} s, acceleration alone {plain:.2} s");
            if target.abs() >= 0.05 {
                assert!(
                    took <= plain + 2.0 * 0.083 + 0.05,
                    "{target}: {took} s against {plain}"
                );
            }
        }
    }

    #[test]
    fn a_stop_brings_speed_and_acceleration_to_zero_together_within_its_jerk() {
        let (accel, jerk) = (14.7, 177.0);
        for (v0, a0) in [
            (1.2, 0.0),
            (1.2, 14.7),
            (-0.8, -5.0),
            (0.7, -14.7),
            (0.001, 0.0),
        ] {
            let (mut v, mut a) = (v0, a0);
            let mut ticks = 0;
            while v != 0.0 || a != 0.0 {
                let (low, high) = acceleration_window(v, a, (0.0, 0.0), accel, jerk, DT);
                assert!((low - high).abs() < 1e-12, "a stop has one answer");
                assert!(
                    (low - a).abs() <= jerk * DT * (1.0 + 1e-9),
                    "jerk from {v0}, {a0}"
                );
                assert!(low.abs() <= accel + 1e-9);
                a = low;
                v = a.mul_add(DT, v);
                if v.abs() < 1e-12 {
                    v = 0.0;
                }
                if a.abs() < 1e-12 {
                    a = 0.0;
                }
                // Never reversing: the stop ends at rest, not past it.
                assert!(v * v0 >= 0.0, "reversed from {v0}, {a0}: {v}");
                ticks += 1;
                assert!(ticks < 500, "never stopped from {v0}, {a0}");
            }
        }
    }

    #[test]
    fn the_stop_distance_is_the_jerk_bounded_stop() {
        let (accel, jerk) = (14.7, 177.0);
        for (v, a) in [
            (1.2_f64, 0.0_f64),
            (0.05, 0.0),
            (1.2, 10.0),
            (0.9, -14.7),
            (-0.4, 3.0),
        ] {
            // Simulated at a fine tick by the window itself.
            let (mut x, mut speed, mut acc) = (0.0_f64, v, a);
            let dt = 1e-4;
            while speed.abs() > 1e-9 || acc.abs() > 1e-9 {
                let (low, _) = acceleration_window(speed, acc, (0.0, 0.0), accel, jerk, dt);
                acc = low;
                speed = acc.mul_add(dt, speed);
                x += speed * dt;
                if speed * v < 0.0 {
                    break;
                }
            }
            let d = stopping_distance(v, a, accel, jerk);
            assert!(
                (d - x).abs() < 1e-3 * (1.0 + d.abs()),
                "{v}, {a}: {d} against {x}"
            );
        }
    }
}
