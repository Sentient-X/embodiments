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
}
