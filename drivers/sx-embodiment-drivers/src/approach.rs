//! The speed a servo itself approaches any goal at, set on the device by its driver.
//!
//! A serial servo's firmware can bound its own motion: Feetech STS servos by `Goal_Speed` and
//! `Acceleration`, Dynamixel X servos by `Profile Velocity` and `Profile Acceleration`. A body
//! that declares an [`ApproachLimit`] for a servo has its driver write those registers and read
//! them back, so no goal is approached faster than the limit: a backstop to the governor, and
//! the speed at which the firmware pulls a servo resting outside its position limits in when
//! torque comes on, which no governor bounds. Each driver converts the joint's limit to its
//! vendor's register steps. A body's limits are registry data per port
//! (`sx_embodiments.layout.ServoApproach`, rendered as the bindings' `approach`);
//! [`BRING_UP_APPROACH`] is the slow one a servo resting outside its position limits is pulled
//! in at when torque comes on: a move no governor bounds, so it stays slow. Every governed
//! motion's limits are the joint's rated ones (`AxisLimits::rated`), never this.

/// The fastest a servo moves toward any goal, in the joint's units: rad/s and rad/s².
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ApproachLimit {
    pub speed: f64,
    pub acceleration: f64,
}

/// The static upper body's bring-up speed (its received driver's 0.25 rad/s), reached over
/// half a second.
pub const BRING_UP_APPROACH: ApproachLimit = ApproachLimit {
    speed: 0.25,
    acceleration: 0.5,
};

impl ApproachLimit {
    /// This limit for an actuator geared at `reduction`, in register steps of `speed_step`
    /// rad/s and `acceleration_step` rad/s², each rounded down so the device is never faster;
    /// `None` when either rounds below one step (which a servo reads as no limit at all) or
    /// above its register's `max`.
    #[must_use]
    pub fn steps(
        &self,
        reduction: f64,
        (speed_step, speed_max): (f64, u32),
        (acceleration_step, acceleration_max): (f64, u32),
    ) -> Option<(u32, u32)> {
        let speed = (self.speed * reduction / speed_step).floor();
        let acceleration = (self.acceleration * reduction / acceleration_step).floor();
        if !(1.0..=f64::from(speed_max)).contains(&speed)
            || !(1.0..=f64::from(acceleration_max)).contains(&acceleration)
        {
            return None;
        }
        // Both are integers checked inside their registers just above.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        Some((speed as u32, acceleration as u32))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_limit_rounds_down_and_never_to_no_limit() {
        let steps = |limit: ApproachLimit| limit.steps(1.0, (0.1, 100), (0.1, 100));
        assert_eq!(steps(BRING_UP_APPROACH), Some((2, 5)));
        assert_eq!(
            steps(ApproachLimit {
                speed: 0.05,
                acceleration: 0.5
            }),
            None
        );
        assert_eq!(
            steps(ApproachLimit {
                speed: 50.0,
                acceleration: 0.5
            }),
            None
        );
    }
}
