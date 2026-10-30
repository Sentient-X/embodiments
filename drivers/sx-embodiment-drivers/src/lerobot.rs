//! A servo's per-unit calibration and wire normalization, as `LeRobot` records and applies it.
//!
//! Reference: `huggingface/lerobot` at `v0.6.0` (`30da8e687a6dfc617fcd94afc367ac7071c376ce`),
//! `src/lerobot/motors/motors_bus.py`: `MotorCalibration` (lines 175-180), `MotorNormMode`, and
//! `MotorsBus._unnormalize` (lines 879-907). Every serial-servo driver here (Feetech STS,
//! Dynamixel X) takes a goal position the same way: the registry's drive map gives the
//! actuator coordinate, `LeRobot`'s normalization makes it degrees or percent, and the unit's
//! calibrated raw range takes that to encoder ticks. The bus crates differ only in whether the
//! calibration's drive mode is applied in software (`apply_drive_mode`: Feetech `True`,
//! Dynamixel `False`, whose servo applies its own `Drive_Mode` register) and in how ticks are
//! encoded on the wire.

/// How `LeRobot` normalizes a servo's coordinate on the wire (`MotorNormMode`).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotorNormMode {
    /// `DEGREES`: degrees about the middle of the calibrated range.
    Degrees,
    /// `RANGE_0_100`: percent of the calibrated range.
    Range0100,
}

/// One servo's per-unit calibration, as `LeRobot`'s `MotorCalibration` records it. A driver
/// refuses to open a servo whose own registers disagree with it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MotorCalibration {
    /// An inverted drive.
    pub drive_mode: bool,
    /// The servo's `Homing_Offset` register, in encoder ticks.
    pub homing_offset: i16,
    /// The calibrated raw range, in encoder ticks: the servo's position limits.
    pub range_min: u16,
    pub range_max: u16,
}

/// `MotorsBus._unnormalize` for one servo: the raw tick count (not yet truncated to the
/// register) that drives it to `actuator`, its actuator coordinate in radians. `low` and `high`
/// are the actuator coordinates of the joint's declared bounds, which a percent servo's 0 and
/// 100 are; `resolution` is the model's encoder steps per turn (`MODEL_RESOLUTION`).
///
/// The arithmetic is `LeRobot`'s, step for step, so every rounding matches: degrees are
/// `int(val * (resolution - 1) / 360 + (min + max) / 2)`, with no drive mode applied; percent is
/// `int(clamp(val or 100 - val, 0, 100) / 100 * (max - min) + min)`, inverted when the drive
/// mode is set and `apply_drive_mode` holds.
#[must_use]
pub fn unnormalize(
    norm_mode: MotorNormMode,
    calibration: &MotorCalibration,
    apply_drive_mode: bool,
    resolution: f64,
    actuator: f64,
    (low, high): (f64, f64),
) -> f64 {
    let min = f64::from(calibration.range_min);
    let max = f64::from(calibration.range_max);
    match norm_mode {
        // Python's `(min_ + max_) / 2`, written the same way so every rounding step matches.
        #[allow(clippy::manual_midpoint)]
        MotorNormMode::Degrees => {
            let val = actuator.to_degrees();
            let mid = (min + max) / 2.0;
            let max_res = resolution - 1.0;
            (val * max_res / 360.0 + mid).trunc()
        }
        MotorNormMode::Range0100 => {
            let val = (actuator - low) * 100.0 / (high - low);
            let val = if apply_drive_mode && calibration.drive_mode {
                100.0 - val
            } else {
                val
            };
            let bounded_val = val.clamp(0.0, 100.0);
            ((bounded_val / 100.0) * (max - min) + min).trunc()
        }
    }
}

/// `MotorsBus._normalize` inverted to the actuator coordinate: the radians a servo reading
/// `raw` ticks is at, `low` and `high` as [`unnormalize`] takes them.
///
/// Degrees are `(raw - (min + max) / 2) * 360 / (resolution - 1)`, as `LeRobot` computes them;
/// percent is `(raw - min) / (max - min) * 100`, inverted when the drive mode is applied, then
/// mapped onto `low..high`. One departure: `LeRobot` clamps a percent reading into the
/// calibrated range first; here it is not clamped, so a servo resting outside its range is
/// reported where it is, and whatever moves it starts from there rather than from the edge.
#[must_use]
pub fn normalize(
    norm_mode: MotorNormMode,
    calibration: &MotorCalibration,
    apply_drive_mode: bool,
    resolution: f64,
    raw: f64,
    (low, high): (f64, f64),
) -> f64 {
    let min = f64::from(calibration.range_min);
    let max = f64::from(calibration.range_max);
    match norm_mode {
        #[allow(clippy::manual_midpoint)]
        MotorNormMode::Degrees => {
            ((raw - (min + max) / 2.0) * 360.0 / (resolution - 1.0)).to_radians()
        }
        MotorNormMode::Range0100 => {
            let norm = (raw - min) / (max - min) * 100.0;
            let norm = if apply_drive_mode && calibration.drive_mode {
                100.0 - norm
            } else {
                norm
            };
            low + norm / 100.0 * (high - low)
        }
    }
}

/// A servo's joint units per radian of its shaft: how far the joint moves while the shaft turns
/// one radian, read from `joint_from_raw` (the driver's [`normalize`] through the drive map)
/// across the calibrated range. For a [`MotorNormMode::Degrees`] servo it is about one over the
/// binding's reduction. For a [`MotorNormMode::Range0100`] servo (a gripper) it is the declared
/// range over the calibrated tick span, which no reduction carries: a finger whose calibrated
/// span is four times its declared range moves a quarter as fast as its shaft (act-sim's
/// 1024..3072 over 0.788 rad, 2026-10-30, where the governor ramped it at the shaft's speed and
/// its tracking stop fired). `None` for an empty calibrated range.
#[must_use]
pub fn joint_per_shaft_radian(
    calibration: &MotorCalibration,
    resolution: f64,
    joint_from_raw: impl Fn(i32) -> f64,
) -> Option<f64> {
    let (min, max) = (
        i32::from(calibration.range_min),
        i32::from(calibration.range_max),
    );
    let shaft = f64::from((max - min).abs()) * std::f64::consts::TAU / resolution;
    let joint = (joint_from_raw(max) - joint_from_raw(min)).abs();
    Some(joint / shaft).filter(|ratio| ratio.is_finite() && *ratio > 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    const CALIBRATION: MotorCalibration = MotorCalibration {
        drive_mode: true,
        homing_offset: 0,
        range_min: 1000,
        range_max: 3000,
    };

    #[test]
    fn a_percent_servo_moves_its_joint_at_its_declared_range_over_its_calibrated_span() {
        let ratio = |mode: MotorNormMode, range: (f64, f64)| {
            joint_per_shaft_radian(&CALIBRATION, 4096.0, |raw| {
                normalize(mode, &CALIBRATION, true, 4096.0, f64::from(raw), range)
            })
            .expect("a calibrated range")
        };
        // Degrees: one joint radian per shaft radian, to LeRobot's 4095-over-4096.
        assert!((ratio(MotorNormMode::Degrees, (-1.0, 1.0)) - 4096.0 / 4095.0).abs() < 1e-12);
        // 2000 ticks (3.07 rad of shaft) spread over a declared 0.5 rad.
        let shaft = 2000.0 * std::f64::consts::TAU / 4096.0;
        assert!((ratio(MotorNormMode::Range0100, (0.0, 0.5)) - 0.5 / shaft).abs() < 1e-12);
        let empty = MotorCalibration {
            range_max: 1000,
            ..CALIBRATION
        };
        assert_eq!(joint_per_shaft_radian(&empty, 4096.0, f64::from), None);
    }

    #[test]
    fn degrees_are_about_the_middle_and_percent_spans_the_range() {
        let degrees = |radians: f64| {
            unnormalize(
                MotorNormMode::Degrees,
                &CALIBRATION,
                true,
                4096.0,
                radians,
                (-1.0, 1.0),
            )
        };
        assert!((degrees(0.0) - 2000.0).abs() < f64::EPSILON);
        assert!((degrees(std::f64::consts::FRAC_PI_2) - 3023.0).abs() < f64::EPSILON);
        let percent = |actuator: f64, apply: bool| {
            unnormalize(
                MotorNormMode::Range0100,
                &CALIBRATION,
                apply,
                4096.0,
                actuator,
                (0.0, 2.0),
            )
        };
        assert!((percent(0.5, false) - 1500.0).abs() < f64::EPSILON);
        assert!((percent(0.5, true) - 2500.0).abs() < f64::EPSILON);
        assert!((percent(3.0, false) - 3000.0).abs() < f64::EPSILON);
    }

    #[test]
    fn a_reading_maps_back_to_its_target_and_outside_the_range_is_not_clamped() {
        for (mode, range, target) in [
            (MotorNormMode::Degrees, (-1.0, 1.0), 0.7),
            (MotorNormMode::Range0100, (0.0, 2.0), 0.5),
        ] {
            for apply in [false, true] {
                let raw = unnormalize(mode, &CALIBRATION, apply, 4096.0, target, range);
                let back = normalize(mode, &CALIBRATION, apply, 4096.0, raw, range);
                // Within the one tick the truncation can lose.
                assert!(
                    (back - target).abs() < 2.0 * std::f64::consts::TAU / 4096.0,
                    "{mode:?}"
                );
            }
        }
        let below = normalize(
            MotorNormMode::Range0100,
            &CALIBRATION,
            false,
            4096.0,
            500.0,
            (0.0, 2.0),
        );
        assert!(below < 0.0, "{below}");
    }
}
