//! Inertial sensors: what a robot's media plan declares, and the surface every IMU adapter
//! implements.
//!
//! `sx_embodiments.media.media_document` lists every declared inertial sensor under the plan's
//! `inertial`, as an [`InertialSensor`]: the product, the unit the instance placed in its slot
//! (the serial a driver opens it by), what it measures, and where its axes sit, at
//! `mount_from_sensor` in the link it is mounted on, that link placed exactly as a camera view's
//! mount is.
//!
//! An adapter implements [`InertialSource`] for one planned sensor: a blocking `next` returning
//! the next [`InertialSample`], on the sensor's own axes in REP-103 units (specific force in
//! m/s², angular velocity in rad/s, magnetic flux density in tesla, pressure in pascal,
//! temperature in degrees Celsius), stamped on the device's clock and when the host had it.
//! The vendor protocol stays in the adapter's crate; nothing here names a product.
//!
//! [`Stillness`] summarises samples taken while the sensor is held still: the mean specific
//! force (gravity's reaction, so it points up), the gyroscope's bias, and how much each
//! wandered. [`Tilt`] turns the up direction in any frame with z up into that frame's roll and
//! pitch, which is how an IMU levels the camera or link it rides on.

use std::sync::Arc;
use std::time::Duration;

use serde::Deserialize;

use crate::kinematics::Chain;
use crate::media::{CaptureError, CapturedAt, Pose};

/// The fastest rate a plan may declare for an inertial sensor.
pub const MAX_INERTIAL_HZ: f64 = 10_000.0;

/// What an inertial sensor measures (`sx_embodiments.parts.InertialQuantity`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InertialQuantity {
    Acceleration,
    AngularVelocity,
    MagneticField,
    Pressure,
    Temperature,
}

/// One declared inertial sensor as the instance resolved it.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InertialSensor {
    /// The sensor's component name; also the name of its axes' frame.
    pub sensor: String,
    /// The product's registry id.
    pub model: String,
    /// The unit placed in this sensor's slot, the serial a driver opens it by.
    pub unit: Option<String>,
    /// The rate acceleration and angular velocity arrive at.
    pub rate_hz: f64,
    pub measures: Vec<InertialQuantity>,
    /// The sensing axes in the mount link's frame.
    pub mount_from_sensor: Pose,
    /// The description link the sensor is mounted on.
    pub mount_frame: String,
    /// The description's root link.
    pub root_frame: String,
    /// The mount link in the root frame, when only fixed joints lie between them.
    pub root_from_mount: Option<Pose>,
    /// The independent joints whose positions place the mount, root first; empty when fixed.
    pub moves_with: Vec<String>,
    /// Every joint from the root down to the mount.
    pub chain: Chain,
}

impl InertialSensor {
    /// Whether the entry is one coherent sensor: a rate, an accelerometer and a gyroscope, a
    /// unit-quaternion pose, and a placement of the same shape a view's is.
    #[must_use]
    pub fn is_coherent(&self) -> bool {
        let mut seen = std::collections::HashSet::new();
        self.rate_hz.is_finite()
            && self.rate_hz > 0.0
            && self.rate_hz <= MAX_INERTIAL_HZ
            && self.measures.iter().all(|quantity| seen.insert(*quantity))
            && seen.contains(&InertialQuantity::Acceleration)
            && seen.contains(&InertialQuantity::AngularVelocity)
            && is_rigid(&self.mount_from_sensor)
            && crate::media::is_placed(self.root_from_mount.as_ref(), &self.moves_with, &self.chain)
    }

    /// Whether the sensor reports `quantity`.
    #[must_use]
    pub fn measures(&self, quantity: InertialQuantity) -> bool {
        self.measures.contains(&quantity)
    }

    /// A vector on the sensor's axes, on the mount link's.
    #[must_use]
    pub fn in_mount(&self, vector: [f64; 3]) -> [f64; 3] {
        rotate(self.mount_from_sensor.rotation_xyzw, vector)
    }
}

/// Whether a pose is finite with a unit quaternion.
pub(crate) fn is_rigid(pose: &Pose) -> bool {
    let norm: f64 = pose.rotation_xyzw.iter().map(|value| value * value).sum();
    pose.translation_m
        .iter()
        .chain(&pose.rotation_xyzw)
        .all(|value| value.is_finite())
        && (norm.sqrt() - 1.0).abs() <= 1e-6
}

/// `vector` rotated by the unit quaternion `(x, y, z, w)`.
#[must_use]
pub fn rotate(rotation_xyzw: [f64; 4], vector: [f64; 3]) -> [f64; 3] {
    // v' = v + 2w (q x v) + 2 q x (q x v), q the vector part.
    let axis = [rotation_xyzw[0], rotation_xyzw[1], rotation_xyzw[2]];
    let twice = scale(cross(axis, vector), 2.0);
    let turned = cross(axis, twice);
    let real = rotation_xyzw[3];
    [
        vector[0] + real * twice[0] + turned[0],
        vector[1] + real * twice[1] + turned[1],
        vector[2] + real * twice[2] + turned[2],
    ]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn scale(v: [f64; 3], by: f64) -> [f64; 3] {
    [v[0] * by, v[1] * by, v[2] * by]
}

fn norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// One reading of an inertial sensor, on its own axes.
#[derive(Clone, Debug, PartialEq)]
pub struct InertialSample {
    /// The frame the vectors are on: the planned sensor's name.
    pub frame: Arc<str>,
    /// When the host had the reading.
    pub captured: CapturedAt,
    /// When the sensor took it, on the device's own clock from an arbitrary origin.
    pub device_time: Duration,
    /// Specific force, m/s²: at rest, gravity's reaction, pointing up.
    pub acceleration_mps2: [f64; 3],
    /// Angular velocity, rad/s.
    pub angular_velocity_radps: [f64; 3],
    /// Magnetic flux density, tesla, when this reading carries a new one.
    pub magnetic_field_t: Option<[f64; 3]>,
    /// Static pressure, pascal, when this reading carries a new one.
    pub pressure_pa: Option<f64>,
    /// The inertial sensor die's temperature, degrees Celsius.
    pub temperature_c: Option<f64>,
}

/// One planned inertial sensor's capture.
pub trait InertialSource: Send {
    /// Block until the next reading of acceleration and angular velocity.
    ///
    /// # Errors
    ///
    /// Returns [`CaptureError`] when the device fails or ends.
    fn next(&mut self) -> Result<InertialSample, CaptureError>;
}

/// Samples taken while the sensor was held still, summarised on its own axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stillness {
    pub samples: usize,
    /// Device-clock time from the first sample to the last.
    pub span: Duration,
    /// Samples per second of device time.
    pub rate_hz: f64,
    /// The mean specific force, m/s²: up, at gravity's magnitude, when still.
    pub mean_acceleration_mps2: [f64; 3],
    /// The root-mean-square distance of each specific force from the mean, m/s².
    pub acceleration_spread_mps2: f64,
    /// The mean angular velocity, rad/s: the gyroscope's bias, when still.
    pub gyro_bias_radps: [f64; 3],
    /// The root-mean-square distance of each angular velocity from the bias, rad/s.
    pub gyro_spread_radps: f64,
}

impl Stillness {
    /// The summary of `samples`, or `None` for fewer than two.
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // a sample count is far below 2^52
    pub fn of(samples: &[InertialSample]) -> Option<Self> {
        let (first, last) = (samples.first()?, samples.last()?);
        if samples.len() < 2 {
            return None;
        }
        let count = samples.len() as f64;
        let mean = |pick: fn(&InertialSample) -> [f64; 3]| {
            let mut sum = [0.0; 3];
            for sample in samples {
                let value = pick(sample);
                for axis in 0..3 {
                    sum[axis] += value[axis];
                }
            }
            scale(sum, 1.0 / count)
        };
        let spread = |pick: fn(&InertialSample) -> [f64; 3], centre: [f64; 3]| {
            let sum: f64 = samples
                .iter()
                .map(|sample| {
                    let value = pick(sample);
                    norm([
                        value[0] - centre[0],
                        value[1] - centre[1],
                        value[2] - centre[2],
                    ])
                    .powi(2)
                })
                .sum();
            (sum / count).sqrt()
        };
        let acceleration = |sample: &InertialSample| sample.acceleration_mps2;
        let angular = |sample: &InertialSample| sample.angular_velocity_radps;
        let mean_acceleration_mps2 = mean(acceleration);
        let gyro_bias_radps = mean(angular);
        let span = last.device_time.saturating_sub(first.device_time);
        Some(Self {
            samples: samples.len(),
            span,
            rate_hz: if span.is_zero() {
                0.0
            } else {
                (count - 1.0) / span.as_secs_f64()
            },
            mean_acceleration_mps2,
            acceleration_spread_mps2: spread(acceleration, mean_acceleration_mps2),
            gyro_bias_radps,
            gyro_spread_radps: spread(angular, gyro_bias_radps),
        })
    }
}

/// Which way is up in a frame whose z axis is its up (REP-103 body axes: x forward, y left), and
/// the frame's roll and pitch from level.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tilt {
    /// The unit up direction on the frame's axes.
    pub up: [f64; 3],
    /// Rotation about x, positive rolling the left side up, radians.
    pub roll_rad: f64,
    /// Rotation about y, positive pitching the nose (x) down, radians.
    pub pitch_rad: f64,
}

impl Tilt {
    /// The tilt a still specific force on the frame's axes implies, or `None` for one too
    /// small to have a direction.
    #[must_use]
    pub fn of_specific_force(force: [f64; 3]) -> Option<Self> {
        let magnitude = norm(force);
        if !magnitude.is_finite() || magnitude < 1e-6 {
            return None;
        }
        let up = scale(force, 1.0 / magnitude);
        Some(Self {
            up,
            roll_rad: up[1].atan2(up[2]),
            pitch_rad: (-up[0]).atan2(up[1].hypot(up[2])),
        })
    }
}

#[cfg(test)]
mod tests {
    use std::time::{Instant, SystemTime};

    use super::*;

    fn sample(at_ms: u64, acceleration: [f64; 3], angular: [f64; 3]) -> InertialSample {
        InertialSample {
            frame: Arc::from("imu"),
            captured: CapturedAt {
                monotonic: Instant::now(),
                wall: SystemTime::now(),
            },
            device_time: Duration::from_millis(at_ms),
            acceleration_mps2: acceleration,
            angular_velocity_radps: angular,
            magnetic_field_t: None,
            pressure_pa: None,
            temperature_c: None,
        }
    }

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9)
    }

    #[test]
    fn a_quarter_turn_about_y_takes_x_down_and_z_forward() {
        let half = 0.5_f64.sqrt();
        let quarter = [0.0, half, 0.0, half];
        assert!(close(rotate(quarter, [1.0, 0.0, 0.0]), [0.0, 0.0, -1.0]));
        assert!(close(rotate(quarter, [0.0, 0.0, 1.0]), [1.0, 0.0, 0.0]));
        assert!(close(rotate(quarter, [0.0, 1.0, 0.0]), [0.0, 1.0, 0.0]));
    }

    #[test]
    fn stillness_is_the_mean_force_the_gyro_bias_and_the_device_rate() {
        let samples: Vec<_> = (0..5)
            .map(|index| {
                let wobble = if index % 2 == 0 { 0.01 } else { -0.01 };
                sample(
                    index * 10,
                    [0.0, wobble, 9.81],
                    [0.001, -0.002, 0.003 + wobble],
                )
            })
            .collect();
        let still = Stillness::of(&samples).expect("five samples");
        assert_eq!(still.samples, 5);
        assert!((still.rate_hz - 100.0).abs() < 1e-9);
        assert!((still.mean_acceleration_mps2[2] - 9.81).abs() < 1e-12);
        assert!((still.gyro_bias_radps[0] - 0.001).abs() < 1e-12);
        assert!(still.acceleration_spread_mps2 < 0.011);
        assert!(Stillness::of(&samples[..1]).is_none());
    }

    #[test]
    fn a_level_frame_has_no_tilt_and_a_nose_down_frame_pitches_positive() {
        let level = Tilt::of_specific_force([0.0, 0.0, 9.81]).expect("tilt");
        assert!(level.roll_rad.abs() < 1e-12 && level.pitch_rad.abs() < 1e-12);
        let angle = 0.3_f64;
        // Nose down by `angle`: up, seen from the frame, leans towards -x.
        let down = Tilt::of_specific_force([-angle.sin(), 0.0, angle.cos()]).expect("tilt");
        assert!((down.pitch_rad - angle).abs() < 1e-12);
        // Left side up by `angle`: up leans towards +y.
        let rolled = Tilt::of_specific_force([0.0, angle.sin(), angle.cos()]).expect("tilt");
        assert!((rolled.roll_rad - angle).abs() < 1e-12);
        assert!(Tilt::of_specific_force([0.0; 3]).is_none());
    }
}
