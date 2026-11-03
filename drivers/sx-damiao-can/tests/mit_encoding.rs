//! Every MIT field (position, velocity, kp, kd, torque) is encoded to the nearest code under the
//! vendor's own decoding (`uint_to_float`, the mapping the motor's firmware decodes with): a
//! round trip is within half a step, zero and the range midpoint take the code nearest zero, and
//! values past either end clamp to it.

use sx_damiao_can::dm_can::{DmCanError, LIMIT_PARAM, float_to_uint, uint_to_float};

/// Each MIT field's range and width, for every Damiao model's limit row.
fn fields() -> Vec<(&'static str, f64, f64, u32)> {
    let mut fields = vec![("kp", 0.0, 500.0, 12), ("kd", 0.0, 5.0, 12)];
    for [q_max, dq_max, tau_max] in LIMIT_PARAM {
        fields.push(("q", -q_max, q_max, 16));
        fields.push(("dq", -dq_max, dq_max, 12));
        fields.push(("tau", -tau_max, tau_max, 12));
    }
    fields
}

fn top(bits: u32) -> u16 {
    u16::try_from((1_u32 << bits) - 1).unwrap()
}

/// The vendor's decoding in double precision (`uint_to_float` narrows to `float32` after).
fn decode(code: u16, min: f64, max: f64, bits: u32) -> f64 {
    f64::from(code) / f64::from(top(bits)) * (max - min) + min
}

fn step(min: f64, max: f64, bits: u32) -> f64 {
    (max - min) / f64::from(top(bits))
}

#[test]
fn a_round_trip_is_within_half_a_step_on_every_field() {
    for (name, min, max, bits) in fields() {
        let half = step(min, max, bits) / 2.0;
        let samples = 20_011_u32;
        for i in 0..=samples {
            let x = min + (max - min) * f64::from(i) / f64::from(samples);
            let code = float_to_uint(x, min, max, bits).unwrap();
            let error = (decode(code, min, max, bits) - x).abs();
            assert!(
                error <= half * (1.0 + 1e-9),
                "{name} [{min}, {max}] x={x}: {error} > {half}"
            );
            let narrowed = f64::from(uint_to_float(code, min, max, bits));
            assert!(
                (narrowed - x).abs() <= half + 1e-5,
                "{name} x={x} float32 decode"
            );
        }
    }
}

#[test]
fn every_code_encodes_back_to_itself() {
    for (name, min, max, bits) in fields() {
        for code in 0..=top(bits) {
            let x = decode(code, min, max, bits);
            assert_eq!(
                float_to_uint(x, min, max, bits).unwrap(),
                code,
                "{name} [{min}, {max}]"
            );
        }
    }
}

#[test]
fn zero_and_the_midpoint_take_the_code_nearest_zero() {
    for (name, min, max, bits) in fields() {
        let nearest = (0..=top(bits))
            .map(|code| decode(code, min, max, bits).abs())
            .fold(f64::INFINITY, f64::min);
        let zero = float_to_uint(0.0, min, max, bits).unwrap();
        let negative_zero = float_to_uint(-0.0, min, max, bits).unwrap();
        assert_eq!(zero, negative_zero, "{name}");
        let error = decode(zero, min, max, bits).abs();
        assert!(
            (error - nearest).abs() <= 1e-12 * (max - min),
            "{name}: zero decodes to {error}"
        );
        if min < 0.0 {
            // A symmetric range has no code for zero; it takes the lower of the two half a step
            // either side, the one DM_CAN.py's truncation sent too.
            assert_eq!(zero, top(bits) / 2, "{name}");
            assert!(error <= step(min, max, bits) / 2.0 * (1.0 + 1e-9), "{name}");
        } else {
            assert_eq!(zero, 0, "{name}");
        }
        // The midpoint lies half a step from two codes; it takes the one nearer zero.
        let middle = float_to_uint(f64::midpoint(min, max), min, max, bits).unwrap();
        assert_eq!(middle, top(bits) / 2, "{name} midpoint");
    }
}

#[test]
fn the_endpoints_and_beyond_clamp() {
    for (name, min, max, bits) in fields() {
        assert_eq!(float_to_uint(min, min, max, bits).unwrap(), 0, "{name}");
        assert_eq!(
            float_to_uint(max, min, max, bits).unwrap(),
            top(bits),
            "{name}"
        );
        for below in [min - 1.0, min - 1e6, f64::NEG_INFINITY] {
            assert_eq!(
                float_to_uint(below, min, max, bits).unwrap(),
                0,
                "{name} {below}"
            );
        }
        for above in [max + 1.0, max + 1e6, f64::INFINITY] {
            assert_eq!(
                float_to_uint(above, min, max, bits).unwrap(),
                top(bits),
                "{name} {above}"
            );
        }
        assert!(matches!(
            float_to_uint(f64::NAN, min, max, bits),
            Err(DmCanError::NotANumber)
        ));
    }
}

#[test]
fn a_planned_zero_velocity_is_no_further_off_than_half_a_step() {
    // The DM8009's VMAX of 45 rad/s: 12 bits put the two codes nearest zero at -0.011 and
    // +0.011 rad/s. Neither is zero; the encoder sends the vendor's and no worse.
    let [_, dq_max, _] = LIMIT_PARAM[6];
    let code = float_to_uint(0.0, -dq_max, dq_max, 12).unwrap();
    assert_eq!(code, 2047);
    let sent = decode(code, -dq_max, dq_max, 12);
    assert!((sent + 0.010_989).abs() < 1e-5, "{sent}");
    // A small planned velocity no longer drops a whole step below what was asked.
    let asked = 0.02;
    let sent = decode(
        float_to_uint(asked, -dq_max, dq_max, 12).unwrap(),
        -dq_max,
        dq_max,
        12,
    );
    assert!(
        (sent - asked).abs() <= step(-dq_max, dq_max, 12) / 2.0,
        "{sent}"
    );
}
