//! The sensor module's HID reports, ported from Stereolabs' `zed-open-capture`.
//!
//! Source: <https://github.com/stereolabs/zed-open-capture> at
//! `93739154903a66c112ee52c3c6fbfd83b7218617`, MIT, Copyright (c) 2021 Stereolabs
//! (`LICENSE-THIRD-PARTY`): the report ids, request ids and packed `RawData` layout of
//! `include/sensorcapture_def.hpp`, the scale factors defined there, the firmware threshold of
//! `include/sensorcapture_def.hpp`'s `ZED_2_FW::FW_3_9`, and the conversion and validity rules of
//! `SensorCapture::grabThreadFunc` in `src/sensorcapture.cpp`.
//!
//! # Departures from the source
//!
//! * **Units.** Angular velocity is radians per second (the source's degrees per second times
//!   pi/180), magnetic flux density tesla (its microtesla times 1e-6), pressure pascal (its hPa
//!   times 100): REP-103, as `sx_embodiment_drivers::InertialSample` states. Acceleration keeps
//!   the source's scale, including its `DEFAULT_GRAVITY` of 9.8189 m/s² per g.
//! * **Time.** The device timestamp is converted exactly (`ticks * 78125 / 2` ns, the source's
//!   `TS_SCALE` of 39062.5 ns per tick) and reported on the device's own clock. The source instead
//!   rebases it onto the host's wall clock at the first sample and rescales its drift against video
//!   frame-sync counts (`mNTPTsScaling`, `mSyncOffset`); aligning to video is the consumer's.
//! * **First sample.** The source discards the first valid report to seed its clock; it is kept.
//! * **Not read.** Humidity, the two imager temperatures and the motion and free-fall flags are
//!   parsed but not converted: none is an inertial quantity.

use std::time::Duration;

use thiserror::Error;

/// The input report carrying one sensor reading (`REP_ID_SENSOR_DATA`).
pub const SENSOR_DATA: u8 = 0x01;
/// The feature report that starts and stops streaming (`REP_ID_SENSOR_STREAM_STATUS`).
pub const STREAM_STATUS: u8 = 0x32;
/// The feature report carrying a request to the MCU (`REP_ID_REQUEST_SET`).
pub const REQUEST_SET: u8 = 0x21;
/// The request that tells the MCU the host is alive (`RQ_CMD_PING`); without one about every
/// second the module stops streaming.
pub const PING: u8 = 0xF2;
/// Reports between pings: the source pings every 400 reads, about one a second at 400 Hz.
pub const PING_EVERY: u32 = 400;
/// The packed size of `RawData`.
pub const REPORT_LEN: usize = 62;

/// `DEFAULT_GRAVITY`, m/s² per g, as the source scales its accelerometer.
pub const DEFAULT_GRAVITY: f64 = 9.8189;
/// `ACC_SCALE`: m/s² per count, ±8 g full scale.
pub const ACC_SCALE: f64 = DEFAULT_GRAVITY * (8.0 / 32768.0);
/// `GYRO_SCALE`: degrees per second per count, ±1000 °/s full scale.
pub const GYRO_SCALE_DPS: f64 = 1000.0 / 32768.0;
/// `MAG_SCALE`: microtesla per count.
pub const MAG_SCALE_UT: f64 = 1.0 / 16.0;
/// `TEMP_SCALE`: degrees Celsius per count.
pub const TEMP_SCALE: f64 = 0.01;
/// `PRESS_SCALE_NEW` (firmware 3.9 and later): hPa per count.
pub const PRESS_SCALE_NEW_HPA: f64 = 0.0001;
/// `PRESS_SCALE_OLD`: hPa per count.
pub const PRESS_SCALE_OLD_HPA: f64 = 0.01;
/// `ZED_2_FW::FW_3_9`, the USB release number from which pressure uses the new scale.
pub const FW_3_9: u16 = 777;
/// `TEMP_NOT_VALID`: a temperature the module did not measure.
pub const TEMP_NOT_VALID: i16 = -27315;
/// A validity byte's value for a reading new in this report (`NEW_VAL`).
pub const NEW_VALUE: u8 = 2;

/// A report that is not one sensor reading.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ReportError {
    #[error("a sensor report is {REPORT_LEN} bytes, this one {0}")]
    Short(usize),
    #[error("report id {0:#04x} is not sensor data")]
    NotSensorData(u8),
}

/// `RawData`, field for field, as the module packs it (little-endian, no padding).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RawReport {
    /// `imu_not_valid`: 1 when the accelerometer and gyroscope repeat the last reading.
    pub imu_not_valid: u8,
    /// Device clock in ticks of 39.0625 µs.
    pub timestamp: u64,
    pub gyro: [i16; 3],
    pub accel: [i16; 3],
    pub frame_sync: u8,
    pub sync_capabilities: u8,
    pub frame_sync_count: u32,
    pub imu_temp: i16,
    /// 0 absent, 1 repeated, 2 new.
    pub mag_valid: u8,
    pub mag: [i16; 3],
    pub camera_moving: u8,
    pub camera_moving_count: u32,
    pub camera_falling: u8,
    pub camera_falling_count: u32,
    /// 0 absent, 1 repeated, 2 new.
    pub env_valid: u8,
    pub temp: i16,
    pub press: u32,
    pub humid: u32,
    pub temp_cam_left: i16,
    pub temp_cam_right: i16,
}

struct Cursor<'a>(&'a [u8]);

impl Cursor<'_> {
    fn take<const N: usize>(&mut self) -> [u8; N] {
        let (head, rest) = self.0.split_at(N);
        self.0 = rest;
        head.try_into().unwrap_or([0; N])
    }
    fn u8(&mut self) -> u8 {
        self.take::<1>()[0]
    }
    fn i16(&mut self) -> i16 {
        i16::from_le_bytes(self.take())
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take())
    }
    fn u64(&mut self) -> u64 {
        u64::from_le_bytes(self.take())
    }
    fn i16x3(&mut self) -> [i16; 3] {
        [self.i16(), self.i16(), self.i16()]
    }
}

impl RawReport {
    /// Parse one input report, its report id first, as `hid_read` returns it.
    ///
    /// # Errors
    ///
    /// [`ReportError::Short`] for fewer than [`REPORT_LEN`] bytes and
    /// [`ReportError::NotSensorData`] for another report id.
    pub fn parse(bytes: &[u8]) -> Result<Self, ReportError> {
        if bytes.len() < REPORT_LEN {
            return Err(ReportError::Short(bytes.len()));
        }
        if bytes[0] != SENSOR_DATA {
            return Err(ReportError::NotSensorData(bytes[0]));
        }
        let mut at = Cursor(&bytes[1..REPORT_LEN]);
        Ok(Self {
            imu_not_valid: at.u8(),
            timestamp: at.u64(),
            gyro: at.i16x3(),
            accel: at.i16x3(),
            frame_sync: at.u8(),
            sync_capabilities: at.u8(),
            frame_sync_count: at.u32(),
            imu_temp: at.i16(),
            mag_valid: at.u8(),
            mag: at.i16x3(),
            camera_moving: at.u8(),
            camera_moving_count: at.u32(),
            camera_falling: at.u8(),
            camera_falling_count: at.u32(),
            env_valid: at.u8(),
            temp: at.i16(),
            press: at.u32(),
            humid: at.u32(),
            temp_cam_left: at.i16(),
            temp_cam_right: at.i16(),
        })
    }

    /// Whether the accelerometer and gyroscope are a new reading.
    #[must_use]
    pub fn imu_is_new(&self) -> bool {
        self.imu_not_valid != 1
    }

    /// The device clock.
    #[must_use]
    pub fn device_time(&self) -> Duration {
        let nanos = u128::from(self.timestamp) * 78_125 / 2;
        Duration::from_nanos(u64::try_from(nanos).unwrap_or(u64::MAX))
    }

    /// Specific force, m/s², on the module's axes.
    #[must_use]
    pub fn acceleration_mps2(&self) -> [f64; 3] {
        self.accel.map(|count| f64::from(count) * ACC_SCALE)
    }

    /// Angular velocity, rad/s, on the module's axes.
    #[must_use]
    pub fn angular_velocity_radps(&self) -> [f64; 3] {
        self.gyro
            .map(|count| (f64::from(count) * GYRO_SCALE_DPS).to_radians())
    }

    /// Magnetic flux density, tesla, when this report carries a new one.
    #[must_use]
    pub fn magnetic_field_t(&self) -> Option<[f64; 3]> {
        (self.mag_valid == NEW_VALUE)
            .then(|| self.mag.map(|count| f64::from(count) * MAG_SCALE_UT * 1e-6))
    }

    /// Static pressure, pascal, when this report carries a new one; the scale follows the
    /// module's firmware (its USB release number).
    #[must_use]
    pub fn pressure_pa(&self, firmware: u16) -> Option<f64> {
        let hpa = if firmware >= FW_3_9 {
            PRESS_SCALE_NEW_HPA
        } else {
            PRESS_SCALE_OLD_HPA
        };
        (self.env_valid == NEW_VALUE).then(|| f64::from(self.press) * hpa * 100.0)
    }

    /// The inertial sensor's temperature, °C, unless the module marks it unmeasured.
    #[must_use]
    pub fn imu_temperature_c(&self) -> Option<f64> {
        (self.imu_temp != TEMP_NOT_VALID).then(|| f64::from(self.imu_temp) * TEMP_SCALE)
    }
}

/// The feature report that starts (`true`) or stops streaming.
#[must_use]
pub fn stream(enable: bool) -> [u8; 2] {
    [STREAM_STATUS, u8::from(enable)]
}

/// The feature report that keeps streaming alive.
#[must_use]
pub fn ping() -> [u8; 2] {
    [REQUEST_SET, PING]
}
