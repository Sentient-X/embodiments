//! Every MIT field (position, velocity, kp, kd, torque) is encoded to the nearest code under the
//! vendor's own decoding (`uint_to_float`, the mapping the motor's firmware decodes with): a
//! round trip is within half a step, zero and the range midpoint take the code nearest zero, and
//! values past either end clamp to it. Position, velocity and torque are also error-diffused:
//! each motor carries their quantisation residuals into its next frame, so a zero averages to
//! zero.

use std::io;
use std::time::Duration;

use sx_damiao_can::dm_can::{
    ControlType, DmCanError, DmMotorType, LIMIT_PARAM, Motor, MotorCommand, MotorControl,
    float_to_uint, float_to_uint_diffused, uint_to_float,
};
use sx_embodiment_drivers::can::{CanFrame, CanPort};

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

/// The velocity codes `ticks` frames send for one asked velocity, the residual carried.
fn diffused(dq: f64, dq_max: f64, ticks: u32) -> Vec<(u16, f64)> {
    let mut residual = 0.0;
    (0..ticks)
        .map(|_| {
            let (code, carried) =
                float_to_uint_diffused(dq, -dq_max, dq_max, 12, residual).unwrap();
            residual = carried;
            (code, carried)
        })
        .collect()
}

#[test]
fn a_diffused_velocity_averages_to_the_asked_one_within_one_code_over_n() {
    for [_, dq_max, _] in LIMIT_PARAM {
        let lsb = step(-dq_max, dq_max, 12);
        for asked in [
            0.0,
            -0.0,
            0.003,
            -0.004,
            0.011,
            0.02,
            -1.234_567,
            dq_max * 0.999,
            dq_max,
        ] {
            for ticks in [1_u32, 2, 3, 10, 101, 1000] {
                let sent = diffused(asked, dq_max, ticks);
                let mut sum = 0.0;
                for (code, residual) in &sent {
                    let value = decode(*code, -dq_max, dq_max, 12);
                    assert!(
                        (value - asked).abs() <= lsb * (1.0 + 1e-9),
                        "every frame within one code"
                    );
                    assert!(residual.abs() <= 0.5 + 1e-9, "residual within half a code");
                    sum += value;
                }
                let mean = sum / f64::from(ticks);
                let bound = lsb / f64::from(ticks) * (1.0 + 1e-9);
                assert!(
                    (mean - asked).abs() <= bound,
                    "VMAX {dq_max}, {asked} over {ticks}: mean {mean}, off {}",
                    (mean - asked).abs()
                );
            }
        }
    }
}

#[test]
fn a_planned_zero_alternates_around_zero_and_its_torque_ripple_is_half_a_step_times_kd() {
    // The DM8009 at kd 5: kd times the velocity error is the torque the error holds.
    let [_, dq_max, _] = LIMIT_PARAM[6];
    let kd = 5.0;
    let sent = diffused(0.0, dq_max, 8);
    let codes: Vec<u16> = sent.iter().map(|(code, _)| *code).collect();
    assert_eq!(codes, [2047, 2048, 2047, 2048, 2047, 2048, 2047, 2048]);
    for (code, _) in sent {
        let ripple = kd * decode(code, -dq_max, dq_max, 12).abs();
        assert!(ripple <= 0.055, "{ripple} N·m");
    }
}

#[test]
fn a_representable_velocity_is_sent_unchanged_every_frame() {
    let [_, dq_max, _] = LIMIT_PARAM[6];
    let asked = decode(3000, -dq_max, dq_max, 12);
    assert!(
        diffused(asked, dq_max, 50)
            .iter()
            .all(|(code, _)| *code == 3000)
    );
}

#[test]
fn a_residual_passed_in_is_bounded_and_one_not_finite_dropped() {
    let [_, dq_max, _] = LIMIT_PARAM[6];
    let (code, carried) = float_to_uint_diffused(0.0, -dq_max, dq_max, 12, 1e9).unwrap();
    assert_eq!(code, 2048, "a residual is clamped to one code");
    assert!(carried.abs() <= 0.5);
    for residual in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            float_to_uint_diffused(0.0, -dq_max, dq_max, 12, residual)
                .unwrap()
                .0,
            2047
        );
    }
    // At the ends of the range nothing past them is carried.
    let (code, carried) = float_to_uint_diffused(dq_max, -dq_max, dq_max, 12, 0.9).unwrap();
    assert_eq!((code, carried), (4095, 0.0));
}

#[derive(Default)]
struct Capture(Vec<CanFrame>);

impl CanPort for Capture {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        self.0.push(*frame);
        Ok(())
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        Ok(None)
    }

    fn sleep(&mut self, _: Duration) {}
}

fn last_dq(control: &MotorControl<Capture>) -> u16 {
    let data = control.port().0.last().unwrap().data;
    (u16::from(data[2]) << 4) | (u16::from(data[3]) >> 4)
}

#[test]
fn the_residual_is_per_motor_and_restarts_at_enable_disable_mode_and_limit_changes() {
    let mut control = MotorControl::new(Capture::default());
    control.add_motor(Motor::new(DmMotorType::DM8009, 0x01, 0x11));
    control.add_motor(Motor::new(DmMotorType::DM8009, 0x02, 0x12));
    let zero = |control: &mut MotorControl<Capture>, id: u16| {
        control.send_mit(id, 10.0, 5.0, 0.0, 0.0, 0.0).unwrap();
        last_dq(control)
    };
    assert_eq!(zero(&mut control, 0x01), 2047);
    assert_eq!(
        zero(&mut control, 0x02),
        2047,
        "each motor carries its own residual"
    );
    assert_eq!(zero(&mut control, 0x01), 2048);
    assert_eq!(zero(&mut control, 0x01), 2047);
    control.send_enable(0x01).unwrap();
    assert_eq!(zero(&mut control, 0x01), 2047, "enable restarts it");
    control.send_command(0x01, MotorCommand::Disable).unwrap();
    assert_eq!(zero(&mut control, 0x01), 2047, "disable restarts it");
    control.change_limit_param(0x01, 12.5, 45.0, 54.0).unwrap();
    assert_eq!(zero(&mut control, 0x01), 2047, "a limit change restarts it");
    let _ = control.switch_control_mode(0x01, ControlType::MIT).unwrap();
    assert_eq!(zero(&mut control, 0x01), 2047, "a mode switch restarts it");
    assert_eq!(
        zero(&mut control, 0x02),
        2048,
        "the other motor's is untouched"
    );
    // A frame built but never sent leaves the residual where it was.
    let _ = control.mit_frame(0x02, 10.0, 5.0, 0.0, 0.0, 0.0).unwrap();
    assert_eq!(zero(&mut control, 0x02), 2047);
}

#[test]
fn a_zero_position_and_torque_alternate_around_zero_too() {
    // On a DM8009 a zero torque has no code (54 N m over 12 bits): sent alone it would be
    // -0.013 N m held for good, which settles a joint short of every target.
    let mut control = MotorControl::new(Capture::default());
    control.add_motor(Motor::new(DmMotorType::DM8009, 0x01, 0x11));
    let mut sent = Vec::new();
    for _ in 0..4 {
        control.send_mit(0x01, 10.0, 5.0, 0.0, 0.0, 0.0).unwrap();
        let data = control.port().0.last().unwrap().data;
        let q = (u16::from(data[0]) << 8) | u16::from(data[1]);
        let tau = (u16::from(data[6] & 0xf) << 8) | u16::from(data[7]);
        sent.push((q, tau));
    }
    assert_eq!(
        sent,
        [(32767, 2047), (32768, 2048), (32767, 2047), (32768, 2048)]
    );
}
